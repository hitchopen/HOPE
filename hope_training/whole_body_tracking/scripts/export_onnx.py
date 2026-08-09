"""Export a trained HOPE checkpoint to a deployable ONNX policy + manifest.

Loads a local checkpoint, rebuilds the policy, and writes:

* ``policy.onnx``            — single-output actor graph, observation[1, 110] -> raw_action[1, 31]
* ``policy_manifest.json``   — the contract (name, dims, control rate, joint order, obs
                               normalization = none, ActionAdapter config path)

The contract name and dims are NOT hardcoded: they come from the actor observation
contract registry entry the environment actually implements (the shipped
HitterPingPong task uses ``hitter_pure``, 110-D). The canonical joint order is
embedded in the ONNX metadata (key ``joint_order``) together with the contract
name (key ``contract``) so the deploy loader can reject a permuted or
wrong-contract model at load time.

Usage:
    python scripts/export_onnx.py --checkpoint logs/rsl_rl/agibot_a3_hitter_pingpong/<run>/model_<iter>.pt

By default the files are written to ``<checkpoint_dir>/exported/``.
"""

import argparse
import json
import os
import pathlib
import sys

MANIFEST_NAME = "policy_manifest.json"
# Repo-root-relative path of the shared adapter config recorded in the manifest.
ACTION_ADAPTER_RELPATH = "a3_deploy/a3_deploy_example/config/action_adapter.yaml"


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


def assert_canonical_joint_order(joint_names, expected_order) -> None:
    """HARD GATE: the articulation enumeration must equal the canonical deploy joint order.

    The articulation's joint enumeration fixes the obs joint_pos/joint_vel/actions
    slices and all 31 action columns of the exported ONNX. If the asset enumerates
    differently, every column would be silently permuted at deploy.
    """
    if list(joint_names) != list(expected_order):
        raise RuntimeError(
            "Articulation joint order does not match the canonical deploy joint order "
            "(hope_training/config/joint_order_agibot_a3.yaml).\n"
            f"  articulation: {list(joint_names)}\n"
            f"  canonical:    {list(expected_order)}\n"
            "Fix your A3 URDF/USD so its joint enumeration matches the canonical order "
            "(or update the canonical order everywhere: training, planner, deploy runner)."
        )


def build_manifest(contract, joint_names, control_rate_hz: int = 50, onnx_name: str = "policy.onnx") -> dict:
    """Build the deploy manifest from a registry contract entry (duck-typed).

    ``contract`` needs ``.name``, ``.total_dim`` and ``.layout`` (name, dim) pairs —
    the shape of :class:`ActorObservationContract` from the contract registry.
    """
    action_dim = len(list(joint_names))
    layout = []
    cursor = 0
    for term_name, term_dim in contract.layout:
        layout.append({"name": str(term_name), "dim": int(term_dim), "slice": [cursor, cursor + int(term_dim)]})
        cursor += int(term_dim)
    if cursor != int(contract.total_dim):
        raise ValueError(
            f"contract layout dims sum to {cursor}, expected total_dim {contract.total_dim}"
        )
    return {
        "contract_name": str(contract.name),
        "obs_dim": int(contract.total_dim),
        "action_dim": action_dim,
        "control_rate_hz": int(control_rate_hz),
        "observation_normalization": "none",
        "observation_layout": layout,
        "onnx_file": str(onnx_name),
        "onnx_signature": {
            "input": {"name": "observation", "shape": [1, int(contract.total_dim)]},
            "output": {"name": "raw_action", "shape": [1, action_dim]},
        },
        "joint_order": list(joint_names),
        "action_adapter_config": ACTION_ADAPTER_RELPATH,
    }


def export_deploy_policy(policy, contract, joint_names, output_dir: str, onnx_name: str, control_rate_hz: int):
    """Trace the actor into a single-input/single-output ONNX + write the manifest.

    Returns ``(onnx_path, manifest_path)``. Embeds the canonical ``joint_order``
    and the ``contract`` name into the ONNX metadata for the loader-side gate.
    """
    import onnx
    import torch

    os.makedirs(output_dir, exist_ok=True)
    onnx_path = os.path.join(output_dir, onnx_name)
    manifest_path = os.path.join(output_dir, MANIFEST_NAME)

    class _ActorWrapper(torch.nn.Module):
        def __init__(self, actor_critic):
            super().__init__()
            self.actor_critic = actor_critic

        def forward(self, observation):
            return self.actor_critic.act_inference(observation)

    module = _ActorWrapper(policy).to("cpu").eval()
    dummy = torch.zeros(1, int(contract.total_dim))
    torch.onnx.export(
        module,
        (dummy,),
        onnx_path,
        export_params=True,
        opset_version=17,
        input_names=["observation"],
        output_names=["raw_action"],
        dynamic_axes={},
    )

    model = onnx.load(onnx_path)
    for key, value in (
        ("joint_order", ",".join(joint_names)),
        ("contract", str(contract.name)),
    ):
        entry = model.metadata_props.add()
        entry.key = key
        entry.value = value
    onnx.save(model, onnx_path)

    manifest = build_manifest(contract, joint_names, control_rate_hz=control_rate_hz, onnx_name=onnx_name)
    with open(manifest_path, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2, sort_keys=True)
        fh.write("\n")
    return onnx_path, manifest_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True, help="Local checkpoint (.pt) to export.")
    parser.add_argument("--output-dir", default=None, help="Output directory (default: <ckpt_dir>/exported).")
    parser.add_argument("--task", default="HOPE-HitterPingPong-AgibotA3-v0", help="Gym task id.")
    parser.add_argument("--onnx-name", default="policy.onnx", help="Exported ONNX filename.")
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
    parser.add_argument("--experiment-name", default="agibot_a3_hitter_pingpong", help="rsl_rl experiment name.")
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

        import importlib

        importlib.import_module("whole_body_tracking.tasks")  # registers the gym tasks
        from whole_body_tracking.tasks.tracking.actor_observation_contract import (
            infer_actor_observation_contract,
        )
        from whole_body_tracking.utils.action_adapter_config import load_joint_order
        from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
        from whole_body_tracking.utils.ppo_cfg import load_ppo_params, runner_kwargs

        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
        clips = [_resolve_motion_path(c) for c in (args.motion_file, args.motion_file_2) if c]
        env_cfg.commands.motion.motion_file = clips if len(clips) > 1 else clips[0]

        env = gym.make(args.task, cfg=env_cfg, render_mode=None)
        joint_names = list(env.unwrapped.scene["robot"].data.joint_names)
        assert_canonical_joint_order(joint_names, load_joint_order())

        # The deploy contract comes from the registry entry the built env implements
        # (the shipped HitterPingPong task -> hitter_pure, 110-D). Refuse to export a
        # policy whose observation layout matches no registered contract.
        contract = infer_actor_observation_contract(env.unwrapped)
        if contract is None:
            raise RuntimeError(
                "The environment's policy observation layout matches no registered actor "
                "observation contract (see tasks/tracking/actor_observation_contract.py). "
                "Refusing to export an unidentifiable policy."
            )
        control_rate_hz = int(round(1.0 / (float(env_cfg.sim.dt) * float(env_cfg.decimation))))
        print(
            f"[export_onnx] contract={contract.name} obs_dim={contract.total_dim} "
            f"control_rate={control_rate_hz} Hz",
            flush=True,
        )

        env = RslRlVecEnvWrapper(env)

        agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(load_ppo_params(), args.experiment_name))
        agent_cfg.device = args.device
        runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
        runner.load(checkpoint)

        onnx_path, manifest_path = export_deploy_policy(
            runner.alg.policy,
            contract,
            joint_names,
            output_dir,
            onnx_name=args.onnx_name,
            control_rate_hz=control_rate_hz,
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
