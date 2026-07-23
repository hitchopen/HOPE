"""Export a trained HOPE checkpoint to a deployable ONNX policy + manifest.

Loads a local checkpoint, rebuilds the policy, and writes:

* ``hope_pingpong.onnx``     — single-output actor graph, observation[1, 111] -> raw_action[1, 31]
* ``policy_manifest.json``   — the contract (name, dims, control rate, joint order, obs
                               normalization = none, ActionAdapter config path)

Usage:
    python scripts/export_onnx.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt

By default the files are written to ``<checkpoint_dir>/exported/``.
"""

import argparse
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
    parser.add_argument("--checkpoint", required=True, help="Local checkpoint (.pt) to export.")
    parser.add_argument("--output-dir", default=None, help="Output directory (default: <ckpt_dir>/exported).")
    parser.add_argument("--task", default="HOPE-PingPong-AgibotA3-v0", help="Gym task id.")
    parser.add_argument("--onnx-name", default="hope_pingpong.onnx", help="Exported ONNX filename.")
    parser.add_argument("--num-envs", type=int, default=1, help="Number of envs to build (1 is enough to export).")
    parser.add_argument("--device", default="cuda:0", help="Compute device.")
    parser.add_argument(
        "--motion-file",
        default="hope_training/motions/preprocessed/hope_forehand.npz",
        help="Forehand clip (only needed so the env instantiates).",
    )
    parser.add_argument(
        "--motion-file-2",
        default="hope_training/motions/preprocessed/hope_backhand.npz",
        help="Backhand clip (only needed so the env instantiates).",
    )
    parser.add_argument(
        "--motion-manifest",
        default=None,
        help="TSV manifest with motion clips and optional strike/racket-target metadata.",
    )
    parser.add_argument("--experiment-name", default="hope_pingpong", help="rsl_rl experiment name.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checkpoint = os.path.abspath(args.checkpoint)
    if not os.path.isfile(checkpoint):
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")
    output_dir = args.output_dir or os.path.join(os.path.dirname(checkpoint), "exported")

    # Launch Isaac (headless) before importing isaaclab modules; clear argv so Kit ignores our args.
    sys.argv = sys.argv[:1]
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=True, device=args.device)
    simulation_app = app_launcher.app

    status = 0
    try:
        import gymnasium as gym

        from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
        from isaaclab_tasks.utils import parse_env_cfg

        import whole_body_tracking.tasks  # noqa: F401
        from whole_body_tracking.utils.exporter import export_policy
        from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
        from whole_body_tracking.utils.ppo_cfg import load_ppo_params, runner_kwargs
        from train import _apply_motion_metadata, _resolve_motion_plan

        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
        clips, motion_metadata = _resolve_motion_plan(args)
        env_cfg.commands.motion.motion_file = clips if len(clips) > 1 else clips[0]
        _apply_motion_metadata(env_cfg, clips, motion_metadata, [])

        env = gym.make(args.task, cfg=env_cfg, render_mode=None)
        joint_names = list(env.unwrapped.scene["robot"].data.joint_names)
        # JOINT-ORDER GATE: the exported ONNX contract is canonical. The live Isaac
        # articulation may enumerate differently as long as the env has an explicit
        # canonical<->articulation permutation and the joint set matches exactly.
        from whole_body_tracking.utils.action_adapter_config import load_joint_order, resolve_joint_order_mapping

        expected_order = list(load_joint_order())
        try:
            mapping = resolve_joint_order_mapping(joint_names, canonical_joint_names=expected_order)
        except ValueError as exc:
            raise RuntimeError(
                "Articulation joint set does not match the canonical deploy joint set "
                "(hope_training/config/joint_order_agibot_a3.yaml).\n"
                f"  articulation: {joint_names}\n"
                f"  canonical:    {expected_order}\n"
                "Fix your A3 URDF/USD or canonical config before exporting."
            ) from exc
        if mapping.is_identity:
            print("[export_onnx] joint-order gate: articulation matches the canonical deploy order.", flush=True)
        else:
            print(
                "[export_onnx] joint-order gate: using canonical->articulation permutation "
                f"{list(mapping.canonical_to_articulation)}",
                flush=True,
            )
        env = RslRlVecEnvWrapper(env)

        agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(load_ppo_params(), args.experiment_name))
        agent_cfg.device = args.device
        runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
        runner.load(checkpoint)

        onnx_path, manifest_path = export_policy(
            runner.alg.policy, output_dir, joint_names=expected_order, onnx_filename=args.onnx_name
        )
        print(f"[export_onnx] wrote {onnx_path}", flush=True)
        print(f"[export_onnx] wrote {manifest_path}", flush=True)
        env.close()
    except Exception:
        import traceback

        print("\n[export_onnx] ERROR:", flush=True)
        traceback.print_exc()
        status = 1
    finally:
        simulation_app.close()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
