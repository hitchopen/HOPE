"""Where inside the station-relative reach box does a checkpoint actually hit?

Rolls a checkpoint out headless, and on every exact-strike frame records the
commanded contact offset (station-relative x/y, absolute z), the swing side,
the question cohort (core/wide) and the achieved racket position/velocity
error. Prints pass rates per lateral/depth/height bin per side. Diagnostic
only: no reward, export or training state is touched.

Usage (inside the training container):
    hope_isaac_py scripts/probe_reach_box.py task=HOPEPingPongStationaryRally \
        checkpoint=logs/rsl_rl/<exp>/<run>/model_<iter>.pt num_envs=512 num_steps=1500 \
        task.racket.venue_tuple_wide_start_steps=0 task.racket.venue_tuple_wide_ramp_steps=1 \
        task.racket.venue_tuple_wide_final_probability=0.8 +json_out=/tmp/probe.json
"""

import json
import pathlib
import sys
import time

import hydra
from omegaconf import OmegaConf

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from play import _resolve_motion_sources  # noqa: E402


def _run(cfg, simulation_app):
    import os

    import gymnasium as gym
    import numpy as np
    import torch

    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
    from isaaclab_tasks.utils import parse_env_cfg

    import importlib

    importlib.import_module("whole_body_tracking.tasks")
    from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
    from whole_body_tracking.utils.ppo_cfg import runner_kwargs
    from whole_body_tracking.utils.task_reward_overrides import apply_reward_overrides
    from whole_body_tracking.utils.task_command_overrides import apply_command_overrides
    from whole_body_tracking.utils.task_sim2real_overrides import apply_sim2real_overrides

    task_id = str(cfg.task.gym_task)
    env_cfg = parse_env_cfg(task_id, device=str(cfg.device), num_envs=int(cfg.num_envs))
    env_cfg.seed = int(cfg.get('seed', 1))
    applied: list = []
    apply_command_overrides(env_cfg, OmegaConf.to_container(cfg.task, resolve=True), applied)
    apply_sim2real_overrides(env_cfg, OmegaConf.to_container(cfg.task, resolve=True), applied)
    apply_reward_overrides(env_cfg.rewards, cfg.task.get("rewards"), applied)
    motion_files = _resolve_motion_sources(cfg)
    env_cfg.commands.motion.motion_file = motion_files if len(motion_files) > 1 else motion_files[0]
    env_cfg.commands.motion.debug_vis = False
    env_cfg.scene.contact_forces.debug_vis = False

    env = gym.make(task_id, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env)
    algo = OmegaConf.to_container(cfg.algo, resolve=True)
    agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(algo, str(cfg.task.experiment_name)))
    agent_cfg.device = str(cfg.device)
    runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(os.path.abspath(str(cfg.checkpoint)))
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    cmd = env.unwrapped.command_manager.get_term("racket_target")
    motion = env.unwrapped.command_manager.get_term("motion")
    origins = env.unwrapped.scene.env_origins
    threshold = float(cmd.cfg.strike_success_pos_thresh)
    rows = []
    falls = []
    obs = env.get_observations().to(agent_cfg.device)
    for step in range(int(cfg.num_steps)):
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            # Time-based exact-strike frame, the same denominator as the training metrics.
            fired = cmd.time_to_strike.abs() <= (0.5 * env.unwrapped.step_dt + 1e-6)
            # Falls: record the question the robot was answering when it tipped over.
            fell = env.unwrapped.termination_manager.get_term("base_fell_tilt")
            if bool(fell.any()):
                ids = torch.where(fell)[0]
                offset = cmd.racket_target_pos_w[ids] - origins[ids]
                offset[:, :2] = cmd.racket_target_pos_w[ids, :2] - cmd.base_target_pos_w[ids]
                falls.append(torch.cat([motion.clip_id[ids, None].float(), cmd._question_wide_selected[ids, None].float(),
                                        offset, cmd.time_to_strike[ids, None]], dim=-1).cpu())
            if bool(fired.any()):
                ids = torch.where(fired)[0]
                offset = cmd.racket_target_pos_w[ids] - origins[ids]
                offset[:, :2] = cmd.racket_target_pos_w[ids, :2] - cmd.base_target_pos_w[ids]
                err_vec = cmd.racket_pos_w[ids] - cmd.racket_target_pos_w[ids]
                vel_err = torch.linalg.norm(cmd.racket_lin_vel_w[ids] - cmd.racket_target_vel_w[ids], dim=-1)
                wide = cmd._question_wide_selected[ids].float()
                rows.append(torch.cat([
                    motion.clip_id[ids, None].float(), wide[:, None], offset, err_vec,
                    torch.linalg.norm(err_vec, dim=-1)[:, None], vel_err[:, None],
                    cmd.racket_target_vel_w[ids], (env.unwrapped.episode_length_buf[ids, None]).float(),
                ], dim=-1).cpu())
        if step % 10 == 0:
            print(f"[probe] step {step} strikes so far {sum(len(r) for r in rows)} t={time.time():.0f}", flush=True)
    reach_accuracy = {k: float(v[0]) for k, v in cmd.metrics.items() if k.startswith('reach_')}
    position_std = float(env.unwrapped.reward_manager.get_term_cfg('racket_position').params['std'])
    env.close()
    data = torch.cat(rows).numpy() if rows else np.zeros((0, 16))
    cols = ["clip", "wide", "x", "y", "z", "ex", "ey", "ez", "err", "verr", "tvx", "tvy", "tvz", "ep_len"]
    fall_data = torch.cat(falls).numpy() if falls else np.zeros((0, 6))
    out = {"checkpoint": str(cfg.checkpoint), "threshold": threshold, "columns": cols, "rows": data.tolist(),
           "fall_columns": ["clip", "wide", "x", "y", "z", "tts"], "falls": fall_data.tolist(), "steps": int(cfg.num_steps),
           "num_envs": int(cfg.num_envs), "seed": env_cfg.seed, "ppo_updates": 0,
           "reach_accuracy": reach_accuracy, "position_std": position_std,
           "bank_sha256": str(cfg.task.racket.venue_tuple_bank_sha256)}
    json_out = cfg.get("json_out")
    if json_out:
        pathlib.Path(str(json_out)).write_text(json.dumps(out))
    _report(data, threshold)
    print(f"[probe] falls={len(fall_data)} strikes={len(data)}")


def _report(data, threshold):
    import numpy as np
    if len(data) == 0:
        print("[probe] no exact strikes recorded"); return
    clip, wide, x, y, z, ex, ey, ez, err, verr = (data[:, i] for i in range(10))
    passed = err <= threshold
    for side, name in ((0, "forehand"), (1, "backhand")):
        m = clip == side
        print(f"\n=== {name}: strikes={int(m.sum())} pass={passed[m].mean():.3f} "
              f"core pass={passed[m & (wide == 0)].mean() if (m & (wide == 0)).any() else float('nan'):.3f} "
              f"wide pass={passed[m & (wide == 1)].mean() if (m & (wide == 1)).any() else float('nan'):.3f}")
        for label, values, edges in (("y", y, np.linspace(y[m].min(), y[m].max(), 7)),
                                     ("x", x, np.linspace(x[m].min(), x[m].max(), 4)),
                                     ("z", z, np.linspace(z[m].min(), z[m].max(), 4))):
            print(f"  by {label}:")
            for lo, hi in zip(edges[:-1], edges[1:]):
                b = m & (values >= lo) & (values <= hi)
                if b.sum() == 0:
                    continue
                print(f"    [{lo:6.3f},{hi:6.3f}] n={int(b.sum()):5d} pass={passed[b].mean():.3f} "
                      f"err={err[b].mean():.3f} signed(ex,ey,ez)=({ex[b].mean():+.3f},{ey[b].mean():+.3f},{ez[b].mean():+.3f}) "
                      f"verr={verr[b].mean():.2f}")


@hydra.main(version_base=None, config_path="../cfg", config_name="play")
def main(cfg):
    OmegaConf.resolve(cfg)
    OmegaConf.set_struct(cfg, False)
    sys.argv = sys.argv[:1]
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=True, device=str(cfg.device))
    simulation_app = app_launcher.app
    try:
        _run(cfg, simulation_app)
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush(); sys.stderr.flush()
        raise
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
