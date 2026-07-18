"""Evaluate a trained HOPE PingPong policy in-sim and report ``success_rate`` (the only metric).

Runs the policy across many parallel environments, detects each strike (the reference clock reaching
the strike frame), and rolls out the no-spin outgoing ball to decide whether the return succeeded
(racket contact AND net crossing AND opponent-half first bounce). Forehand, backhand and rally rounds
are merged into one number:

    success_rate = successful_return_tasks / incoming_balls_that_entered_a_strike_task

Scoring contract (kept consistent with the shared ``success_metric`` module):

* positions are mapped from the sim world into the TABLE frame the metric expects (origin at the
  near-side left corner of the table surface, x in [0, length], y in [-width, 0], z = 0 at the
  surface) using the same table placement the training command term uses for its return shaping
  (``table_near_x`` / ``table_surface_z`` / station-centred y);
* the outgoing ball leaves with the racket's ACHIEVED velocity at the strike frame (not the
  commanded target velocity);
* strikes that coincide with an environment reset (time-out / fall) are excluded — a reset re-seeds
  the reference clock, which is not a swing.

This remains the fast in-Isaac estimate; the authoritative physical number comes from
``mujoco_eval_onnx.py`` (real simulated ball). Emits only a machine-readable
``{"success_rate": <float>}`` to stdout (and optionally to --json-out). No thresholds, no exit-code
changes, no other metrics.

Usage:
    python scripts/evaluate.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt \
        --num-envs 256 --num-steps 4000
"""

import argparse
import json
import os
import pathlib
import sys


def _repo_root() -> pathlib.Path:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "hope_training").is_dir():
            return parent
    return here.parents[2]


def _resolve_motion_path(value: str) -> str:
    p = pathlib.Path(str(value))
    if p.is_file():
        return str(p.resolve())
    rooted = _repo_root() / value
    return str(rooted.resolve()) if rooted.is_file() else str(rooted)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True, help="Local checkpoint (.pt) to evaluate.")
    parser.add_argument("--task", default="HOPE-PingPong-AgibotA3-v0", help="Gym task id.")
    parser.add_argument("--num-envs", type=int, default=256, help="Parallel environments.")
    parser.add_argument("--num-steps", type=int, default=4000, help="Policy steps to roll out.")
    parser.add_argument("--device", default="cuda:0", help="Compute device.")
    parser.add_argument("--contact-radius", type=float, default=0.10, help="Racket-to-target contact gate (m).")
    parser.add_argument("--json-out", default=None, help="Also write {'success_rate': ...} to this file.")
    parser.add_argument("--experiment-name", default="hope_pingpong", help="rsl_rl experiment name.")
    parser.add_argument(
        "--motion-file", default="hope_training/motions/preprocessed/hope_forehand.npz", help="Forehand clip."
    )
    parser.add_argument(
        "--motion-file-2", default="hope_training/motions/preprocessed/hope_backhand.npz", help="Backhand clip."
    )
    return parser.parse_args()


def _first_attr(obj, names):
    """Return the first present attribute among ``names`` (else None). Coupling shim for the env API."""
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def main() -> int:
    args = parse_args()
    checkpoint = os.path.abspath(args.checkpoint)
    if not os.path.isfile(checkpoint):
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")

    sys.argv = sys.argv[:1]
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=True, device=args.device)
    simulation_app = app_launcher.app

    status = 0
    try:
        import gymnasium as gym
        import torch

        from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
        from isaaclab_tasks.utils import parse_env_cfg

        import whole_body_tracking.tasks  # noqa: F401
        from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
        from whole_body_tracking.utils.ppo_cfg import load_ppo_params, runner_kwargs
        from whole_body_tracking.utils.success_metric import (
            BallPhysics,
            SuccessRate,
            TableGeometry,
            evaluate_return,
        )

        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
        clips = [_resolve_motion_path(c) for c in (args.motion_file, args.motion_file_2) if c]
        env_cfg.commands.motion.motion_file = clips if len(clips) > 1 else clips[0]

        env = gym.make(args.task, cfg=env_cfg, render_mode=None)
        base_env = env.unwrapped
        env = RslRlVecEnvWrapper(env)

        agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(load_ppo_params(), args.experiment_name))
        agent_cfg.device = args.device
        runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
        runner.load(checkpoint)
        policy = runner.get_inference_policy(device=base_env.device)

        physics = BallPhysics.from_config()
        table = TableGeometry.from_config()
        accumulator = SuccessRate()

        # The racket-target command term exposes the per-strike quantities we score. Attribute names
        # are read defensively (see COUPLING NOTES in the report): the command must expose the racket
        # target position (world), the achieved racket position AND velocity (world), the
        # time-to-strike, and the swing side.
        cmd = base_env.command_manager.get_term("racket_target")
        env_origins = base_env.scene.env_origins  # (N, 3)

        # Table placement in the ENV-LOCAL frame — the same constants the command term's return
        # shaping uses (tasks/tracking/mdp/hope_commands.py _evaluate_return). The shared
        # success-metric TABLE frame has its origin at the near-side LEFT (+y) corner of the table
        # surface: x_table = x_env - table_near_x, y_table = y_env - (station_y + width/2),
        # z_table = z_env - table_surface_z. Placement (near_x / surface_z / station y) comes from
        # the command cfg; table DIMENSIONS come from the same TableGeometry the scoring uses
        # (configs/ball_physics.yaml), so a re-fitted table width keeps the two in agreement.
        table_near_x = float(cmd.cfg.table_near_x)
        table_surface_z = float(cmd.cfg.table_surface_z)
        table_half_w = 0.5 * float(table.width)

        def read_state():
            target_pos = _first_attr(cmd, ["racket_target_pos_w", "target_pos_w", "racket_target_w"])
            racket_pos = _first_attr(cmd, ["racket_pos_w", "achieved_racket_pos_w", "current_racket_pos_w"])
            racket_vel = _first_attr(cmd, ["racket_lin_vel_w", "racket_vel_w", "achieved_racket_vel_w"])
            tts = _first_attr(cmd, ["time_to_strike", "tts"])
            swing = _first_attr(cmd, ["swing_side", "swing_sign"])
            missing = [n for n, v in [
                ("racket_target_pos_w", target_pos), ("racket_pos_w", racket_pos),
                ("racket_lin_vel_w", racket_vel), ("time_to_strike", tts), ("swing_side", swing)]
                if v is None]
            if missing:
                raise AttributeError(
                    "evaluate.py could not read the strike quantities from the 'racket_target' command "
                    f"term (missing: {missing}). Expose these tensors on the command term (world frame, "
                    "shape (N,3) for positions/velocities, (N,) for tts/swing) or adjust read_state()."
                )
            return target_pos, racket_pos, racket_vel, tts, swing

        def to_table_frame(pos_w_row, e):
            """Sim-world position -> the shared metric's table frame for env ``e``."""
            p = (pos_w_row - env_origins[e]).cpu().numpy().astype(float)
            station_y = float((cmd.fixed_station_w[e, 1] - env_origins[e, 1]).item())
            p[0] -= table_near_x
            p[1] -= station_y + table_half_w   # table centred on the station: left edge -> y = 0
            p[2] -= table_surface_z
            return p

        obs, _ = env.get_observations()
        prev_tts = read_state()[3].clone()
        for _ in range(args.num_steps):
            with torch.inference_mode():
                actions = policy(obs)
                obs, _, dones, _ = env.step(actions)
            target_pos, racket_pos, racket_vel, tts, swing = read_state()
            # A strike happens when the reference clock crosses the strike frame (tts: >0 -> <=0).
            # Environments that RESET this step are excluded: a time-out/fall reset re-seeds the
            # clock, and counting it would contaminate the denominator with non-swings.
            reset_now = dones.reshape(-1).to(dtype=torch.bool, device=tts.device)
            struck = (prev_tts > 0.0) & (tts <= 0.0) & (~reset_now)
            idx = struck.nonzero(as_tuple=False).flatten().tolist()
            for e in idx:
                tp = to_table_frame(target_pos[e], e)
                rp = to_table_frame(racket_pos[e], e)
                rv = racket_vel[e].cpu().numpy().astype(float)  # achieved racket velocity at strike
                outcome = evaluate_return(tp, rp, rv, physics, table, contact_radius=args.contact_radius)
                accumulator.add(outcome)
            prev_tts = tts.clone()

        result = accumulator.as_dict()
        print(json.dumps(result))
        if args.json_out:
            with open(args.json_out, "w", encoding="utf-8") as f:
                json.dump(result, f)
                f.write("\n")
        env.close()
    except Exception:
        import traceback

        print("\n[evaluate] ERROR:", flush=True)
        traceback.print_exc()
        status = 1
    finally:
        simulation_app.close()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
