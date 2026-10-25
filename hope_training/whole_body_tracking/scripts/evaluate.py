"""Evaluate a trained HOPE policy in-sim and report ``success_rate`` (the only metric).

Runs the policy across many parallel environments, detects each strike (the reference clock reaching
the strike frame), and rolls out the full PPQ004 paddle impulse and spinning ball to decide whether the return succeeded
(racket contact AND net crossing AND opponent-half first bounce). Forehand, backhand and rally rounds
are merged into one number:

    success_rate = successful_return_tasks / incoming_balls_that_entered_a_strike_task

Scoring contract (kept consistent with the shared ``success_metric`` module):

* positions are mapped from the sim world into the TABLE frame the metric expects (origin at the
  near-side left corner of the table surface, x in [0, length], y in [-width, 0], z = 0 at the
  surface) using the same table placement the training command term uses for its return shaping
  (``table_near_x`` / ``table_surface_z`` / fixed table centre);
* the outgoing ball is computed from incoming velocity/spin and the racket's ACHIEVED
  velocity and face normal at the strike frame;
* strikes that coincide with an environment reset (time-out / fall) are excluded — a reset re-seeds
  the reference clock, which is not a swing.

The environment and policy architecture come from the checkpoint's adjacent
``params/task_resolved.yaml``, or an explicit ``--resolved-config``. This preserves
legacy task recipes; scoring always measures the full PPQ004 return model, including
when evaluating an older policy. MuJoCo offers a separate physical-solver evaluation.
Emits a machine-readable
``{"success_rate": <float>}`` to stdout (and optionally to --json-out). No thresholds, no exit-code
changes, no other metrics.

Usage:
    python scripts/evaluate.py --checkpoint logs/rsl_rl/agibot_a3_hitter_pingpong/<run>/model_<iter>.pt \
        --num-envs 256 --num-steps 4000
"""

import argparse
import json
import os
import pathlib
import sys

from omegaconf import OmegaConf


def _repo_root() -> pathlib.Path:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "hope_training").is_dir():
            return parent
    return here.parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True, help="Local checkpoint (.pt) to evaluate.")
    parser.add_argument("--resolved-config", help="Saved full training YAML (task and algo); defaults to the checkpoint's params/task_resolved.yaml.")
    parser.add_argument("--task", default=None, help="Optional assertion of the saved Gym task id.")
    parser.add_argument("--num-envs", type=int, default=256, help="Parallel environments.")
    parser.add_argument("--num-steps", type=int, default=4000, help="Policy steps to roll out.")
    parser.add_argument("--device", default="cuda:0", help="Compute device.")
    parser.add_argument("--contact-radius", type=float, default=0.10, help="Racket-to-target contact gate (m).")
    parser.add_argument("--json-out", default=None, help="Also write {'success_rate': ...} to this file.")
    parser.add_argument("--experiment-name", default=None, help="Override the saved rsl_rl experiment name.")
    parser.add_argument(
        "--motion-file", default=None, help="Override the saved forehand clip."
    )
    parser.add_argument(
        "--motion-file-2", default=None, help="Override the saved backhand clip."
    )
    return parser.parse_args()


def _load_evaluation_config(checkpoint, resolved_config=None, *, task_id=None):
    """Require the actual training recipe instead of guessing from checkpoint shapes."""
    path = (pathlib.Path(resolved_config).expanduser() if resolved_config else
            pathlib.Path(checkpoint).parent / "params/task_resolved.yaml")
    if not path.is_file():
        raise FileNotFoundError(
            f"Saved training configuration not found: {path}. Supply --resolved-config with "
            "the checkpoint's full task/algo YAML; evaluation cannot reconstruct its recipe safely."
        )
    raw = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
    if not isinstance(raw, dict) or not all(isinstance(raw.get(k), dict) for k in ("task", "algo")):
        raise ValueError("Evaluation requires a resolved training configuration containing task and algo mappings")
    task = raw["task"]
    if not all(isinstance(task.get(k), str) and task[k] for k in ("gym_task", "experiment_name")):
        raise ValueError("Saved task must specify gym_task and experiment_name")
    if not all(isinstance(raw["algo"].get(k), dict) for k in ("runner", "policy", "algorithm")):
        raise ValueError("Saved algo must specify runner, policy and algorithm mappings")
    if task_id is not None and task_id != task["gym_task"]:
        raise ValueError(f"--task={task_id} differs from saved task {task['gym_task']}")
    # Old resolved files may omit top-level optional motion overrides.
    raw.setdefault("motion_file", None)
    raw.setdefault("motion_file_2", None)
    return OmegaConf.create(raw)


def _bind_saved_physics(cfg, *, environ=None):
    """Verify full-model bank identity; keep explicit legacy recipes on venue physics."""
    from ppq004_training import BANK_KEYS, configure_refitted_task, _source_identities
    target = os.environ if environ is None else environ
    saved = OmegaConf.to_container(cfg.task, resolve=True)
    verified = OmegaConf.create(OmegaConf.to_container(cfg, resolve=True))
    selected = {}
    full = configure_refitted_task(verified, environ=selected)
    if full:
        for key in BANK_KEYS:
            if key.endswith("_path"):
                continue
            if saved.get("racket", {}).get(key) != verified.task.racket[key]:
                raise ValueError(f"Saved checkpoint bank identity differs from refit manifest: {key}")
        if saved.get("ppq004_physics_sha256") != verified.task.ppq004_physics_sha256:
            raise ValueError("Saved checkpoint physics profile differs from refit manifest")
        if _source_identities(saved.get("ppq004_physics_source_hashes")) != _source_identities(
                OmegaConf.to_container(verified.task.ppq004_physics_source_hashes)):
            raise ValueError("Saved checkpoint referenced physics differs from refit manifest")
        # Canonicalize only paths after preserving all saved content identities.
        cfg.task = verified.task
    target["HOPE_BALL_PHYSICS_YAML"] = selected["HOPE_BALL_PHYSICS_YAML"]


def _apply_evaluation_recipe(env_cfg, cfg, *, motion_file=None, motion_file_2=None):
    """Use the trainer's complete override path (commands, rewards, pose and sim2real)."""
    from train import _apply_task_overrides, _resolve_motion_sources
    applied = []
    _apply_task_overrides(env_cfg, cfg, applied)
    if motion_file is not None:
        cfg.motion_file = motion_file
    if motion_file_2 is not None:
        cfg.motion_file_2 = motion_file_2
    clips = _resolve_motion_sources(cfg)
    env_cfg.commands.motion.motion_file = clips if len(clips) > 1 else clips[0]
    if cfg.get("seed") is not None:
        env_cfg.seed = int(cfg.seed)
    return applied


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
    cfg = _load_evaluation_config(checkpoint, args.resolved_config, task_id=args.task)
    _bind_saved_physics(cfg)
    task_id = str(cfg.task.gym_task)

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

        import importlib

        importlib.import_module("whole_body_tracking.tasks")  # registers the gym tasks
        from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
        from whole_body_tracking.utils.ppo_cfg import runner_kwargs
        from whole_body_tracking.utils.success_metric import (
            BallPhysics,
            SuccessRate,
            TableGeometry,
            evaluate_return,
        )

        env_cfg = parse_env_cfg(task_id, device=args.device, num_envs=args.num_envs)
        _apply_evaluation_recipe(env_cfg, cfg, motion_file=args.motion_file, motion_file_2=args.motion_file_2)

        env = gym.make(task_id, cfg=env_cfg, render_mode=None)
        base_env = env.unwrapped
        env = RslRlVecEnvWrapper(env)

        algo = OmegaConf.to_container(cfg.algo, resolve=True)
        experiment = args.experiment_name or str(cfg.task.experiment_name)
        agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(algo, experiment))
        agent_cfg.device = args.device
        runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
        runner.load(checkpoint, load_optimizer=False, map_location=args.device)
        policy = runner.get_inference_policy(device=base_env.device)

        from ball_physics_profile import load_profile
        scoring_profile = load_profile(cfg.task.get("ppq004_physics_yaml") or
                                       _repo_root() / "configs/ball_physics_ppq004_training.yaml")
        physics = BallPhysics.from_config(scoring_profile)
        table = TableGeometry.from_config(scoring_profile)
        accumulator = SuccessRate()

        # The racket-target command term exposes the per-strike quantities we score. Attribute names
        # are read defensively (see COUPLING NOTES in the report): the command must expose the racket
        # target position (world), the achieved racket position AND velocity (world), the
        # time-to-strike, and the swing side.
        cmd = base_env.command_manager.get_term("racket_target")
        env_origins = base_env.scene.env_origins  # (N, 3)

        # Table placement in the ENV-LOCAL frame — the same constants the command term's virtual
        # ball / return shaping uses (tasks/tracking/mdp/hope_commands.py, ``vb_table_near_x`` /
        # ``vb_table_surface_z``). The shared success-metric TABLE frame has its origin at the
        # near-side LEFT (+y) corner of the table surface: x_table = x_env - table_near_x,
        # y_table = y_env - (table_y_center + width/2), z_table = z_env - table_surface_z.
        # The fixed table placement comes from the restored task and scoring YAML.
        table_near_x = float(cmd.cfg.vb_table_near_x)
        table_surface_z = float(cmd.cfg.vb_table_surface_z)
        table_half_w = 0.5 * float(table.width)

        def read_state():
            target_pos = _first_attr(cmd, ["racket_target_pos_w", "target_pos_w", "racket_target_w"])
            racket_pos = _first_attr(cmd, ["racket_pos_w", "achieved_racket_pos_w", "current_racket_pos_w"])
            racket_vel = _first_attr(cmd, ["racket_lin_vel_w", "racket_vel_w", "achieved_racket_vel_w"])
            tts = _first_attr(cmd, ["time_to_strike", "tts"])
            swing = _first_attr(cmd, ["swing_sign"])
            missing = [n for n, v in [
                ("racket_target_pos_w", target_pos), ("racket_pos_w", racket_pos),
                ("racket_lin_vel_w", racket_vel), ("time_to_strike", tts), ("swing_sign", swing)]
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
            # The table is fixed in each environment; robot station motion does not move it.
            table_center_y = float(physics.profile["training_frame"]["y_center"])
            p[0] -= table_near_x
            p[1] -= table_center_y + table_half_w
            p[2] -= table_surface_z
            return p

        obs = env.get_observations().to(args.device)
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
                outcome = evaluate_return(
                    tp, rp, rv, physics, table, contact_radius=args.contact_radius,
                    incoming_ball_velocity=cmd.vb_vel_in_w[e].cpu().numpy().astype(float),
                    incoming_ball_spin=cmd.vb_spin_in_w[e].cpu().numpy().astype(float),
                    racket_normal=cmd.racket_normal_w[e].cpu().numpy().astype(float),
                )
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
