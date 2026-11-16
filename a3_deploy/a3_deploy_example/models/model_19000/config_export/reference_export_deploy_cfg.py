#!/usr/bin/env python3
"""Export a Unitree-style deployment contract for one HOPE policy directory.

The resulting directory has the same two-file ownership split used by
``unitree_rl_lab``::

    <run>/exported/policy.onnx
    <run>/params/deploy.yaml

The ONNX contains the network and immutable policy metadata.  ``deploy.yaml``
contains the manager/runtime contract: control period, SDK joint mapping,
default pose, PD gains, action affine/clip, and ordered observation terms.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import onnx
import yaml


SDK_JOINT_NAMES = [
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _csv(metadata: dict[str, str], key: str, cast: type = float) -> list[Any]:
    value = metadata.get(key, "")
    if not value:
        raise ValueError(f"ONNX metadata is missing {key!r}")
    return [cast(item) for item in value.split(",")]


def _finite_vector(values: list[float], key: str, length: int) -> list[float]:
    if len(values) != length:
        raise ValueError(f"{key} length {len(values)} != {length}")
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{key} contains non-finite values")
    return values


def _load_frozen_env(path: Path) -> dict[str, Any]:
    # Hydra serializes tuples with !!python/tuple.  This is a trusted, local,
    # run-frozen config rather than user-supplied YAML.
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.load(stream, Loader=yaml.Loader)
    if not isinstance(value, dict):
        raise ValueError(f"frozen env config is not a mapping: {path}")
    return value


def _model_dims(model: onnx.ModelProto) -> tuple[int, int]:
    graph_inputs = {value.name: value for value in model.graph.input}
    graph_outputs = {value.name: value for value in model.graph.output}
    if "obs" not in graph_inputs or "actions" not in graph_outputs:
        raise ValueError("policy ONNX must expose obs input and actions output")
    obs_shape = graph_inputs["obs"].type.tensor_type.shape.dim
    action_shape = graph_outputs["actions"].type.tensor_type.shape.dim
    if len(obs_shape) != 2 or len(action_shape) != 2:
        raise ValueError("policy ONNX obs/actions must be rank-2")
    return int(obs_shape[1].dim_value), int(action_shape[1].dim_value)


def build_deploy_cfg(policy_path: Path, frozen_env_path: Path) -> dict[str, Any]:
    model = onnx.load(str(policy_path), load_external_data=False)
    metadata = {item.key: item.value for item in model.metadata_props}
    obs_dim, action_dim = _model_dims(model)
    if action_dim != 31:
        raise ValueError(f"A3 policy action dim {action_dim} != 31")

    joint_names = _csv(metadata, "joint_names", str)
    if len(joint_names) != action_dim or len(set(joint_names)) != action_dim:
        raise ValueError("joint_names must be a 31-entry bijection")
    if set(joint_names) != set(SDK_JOINT_NAMES):
        missing = sorted(set(SDK_JOINT_NAMES) - set(joint_names))
        extra = sorted(set(joint_names) - set(SDK_JOINT_NAMES))
        raise ValueError(f"joint set does not match A3 SDK layout; missing={missing}, extra={extra}")
    joint_ids_map = [SDK_JOINT_NAMES.index(name) for name in joint_names]

    default_q = _finite_vector(_csv(metadata, "default_joint_pos"), "default_joint_pos", action_dim)
    action_scale = _finite_vector(_csv(metadata, "action_scale"), "action_scale", action_dim)
    stiffness_policy = _finite_vector(_csv(metadata, "joint_stiffness"), "joint_stiffness", action_dim)
    damping_policy = _finite_vector(_csv(metadata, "joint_damping"), "joint_damping", action_dim)
    safe_lower = _finite_vector(_csv(metadata, "qdes_safe_lower_rad"), "qdes_safe_lower_rad", action_dim)
    safe_upper = _finite_vector(_csv(metadata, "qdes_safe_upper_rad"), "qdes_safe_upper_rad", action_dim)
    hard_lower = _finite_vector(_csv(metadata, "qdes_hard_lower_rad"), "qdes_hard_lower_rad", action_dim)
    hard_upper = _finite_vector(_csv(metadata, "qdes_hard_upper_rad"), "qdes_hard_upper_rad", action_dim)

    for index, name in enumerate(joint_names):
        if not (
            hard_lower[index]
            <= safe_lower[index]
            < default_q[index]
            < safe_upper[index]
            <= hard_upper[index]
        ):
            raise ValueError(f"invalid hard/safe/default interval for {name}")

    stiffness_sdk = [0.0] * action_dim
    damping_sdk = [0.0] * action_dim
    for policy_index, sdk_index in enumerate(joint_ids_map):
        stiffness_sdk[sdk_index] = stiffness_policy[policy_index]
        damping_sdk[sdk_index] = damping_policy[policy_index]

    frozen_env = _load_frozen_env(frozen_env_path)
    sim_cfg = frozen_env.get("sim") or {}
    sim_dt = float(sim_cfg.get("dt"))
    decimation = int(frozen_env.get("decimation"))
    step_dt = sim_dt * decimation
    if not math.isfinite(step_dt) or step_dt <= 0.0:
        raise ValueError(f"invalid policy step_dt={step_dt}")

    obs_names = _csv(metadata, "observation_names", str)
    obs_dims = _csv(metadata, "actor_obs_term_dims", int)
    history_lengths = _csv(metadata, "observation_history_lengths", int)
    if not (len(obs_names) == len(obs_dims) == len(history_lengths)):
        raise ValueError("observation name/dim/history metadata lengths disagree")
    if sum(dim * history for dim, history in zip(obs_dims, history_lengths)) != obs_dim:
        raise ValueError("ordered observation terms do not sum to the ONNX obs dimension")
    sources = json.loads(metadata.get("actor_obs_term_sources_json", "{}"))
    observations: dict[str, Any] = {}
    for name, dim, history in zip(obs_names, obs_dims, history_lengths):
        observations[name] = {
            "params": {},
            "scale": [1.0] * dim,
            "clip": None,
            "history_length": history,
            "dim": dim,
            "source": sources.get(name, "unspecified"),
        }

    qdes_contract = metadata.get("qdes_action_contract", "")
    supported_qdes_contracts = {
        "v11_affine_safe_qdes_v1",
        "v12_affine_safe_slew_qdes_v1",
    }
    if qdes_contract not in supported_qdes_contracts:
        raise ValueError(
            "HitterPingPong Unitree-style export currently requires "
            "qdes_action_contract=v11_affine_safe_qdes_v1 or "
            f"v12_affine_safe_slew_qdes_v1, got {qdes_contract!r}"
        )
    if qdes_contract == "v12_affine_safe_slew_qdes_v1":
        slew = _finite_vector(
            _csv(metadata, "qdes_slew_limit_rad_per_tick"),
            "qdes_slew_limit_rad_per_tick",
            action_dim,
        )
        leg_mask = _finite_vector(
            _csv(metadata, "qdes_slew_leg_mask"),
            "qdes_slew_leg_mask",
            action_dim,
        )
        leg_count = 0
        head_count = 0
        for index, name in enumerate(joint_names):
            if leg_mask[index] not in (0.0, 1.0):
                raise ValueError(f"qdes_slew_leg_mask is not binary for {name}")
            is_leg = any(token in name for token in ("hip", "knee", "ankle"))
            if (leg_mask[index] == 1.0) != is_leg:
                raise ValueError(f"qdes_slew_leg_mask does not match joint role for {name}")
            if is_leg:
                leg_count += 1
            if name in ("head_yaw_joint", "head_pitch_joint"):
                head_count += 1
                continue
            if slew[index] <= 0.0 or action_scale[index] <= 0.0:
                raise ValueError(f"V12 executable joint has non-positive slew/scale: {name}")
        if leg_count != 12 or head_count != 2:
            raise ValueError(
                f"V12 joint-role count mismatch: legs={leg_count}, passive_heads={head_count}"
            )
        tilt_full = float(metadata.get("qdes_slew_tilt_full_scale_rad", "nan"))
        tilt_min = float(metadata.get("qdes_slew_tilt_min_scale_rad", "nan"))
        speed_full = float(metadata.get("qdes_slew_speed_full_scale_mps", "nan"))
        speed_min = float(metadata.get("qdes_slew_speed_min_scale_mps", "nan"))
        min_scale = float(metadata.get("qdes_slew_min_scale", "nan"))
        if not (
            all(math.isfinite(value) for value in (
                tilt_full, tilt_min, speed_full, speed_min, min_scale
            ))
            and 0.0 <= tilt_full < tilt_min
            and 0.0 <= speed_full < speed_min
            and 0.0 < min_scale <= 1.0
        ):
            raise ValueError("V12 slew scale metadata is invalid")
        if metadata.get("hitter_pure_last_action_feedback") != "executed_qdes_raw_v12":
            raise ValueError("V12 requires hitter_pure_last_action_feedback=executed_qdes_raw_v12")
        if metadata.get("qdes_policy_feedback_contract") != "executed_qdes_raw_v12":
            raise ValueError("V12 requires qdes_policy_feedback_contract=executed_qdes_raw_v12")
        if metadata.get("qdes_slew_state_source") != (
            "actor_visible_projected_gravity_tilt_and_filtered_mocap_speed_v1"
        ):
            raise ValueError("V12 qdes_slew_state_source is invalid")
    actual_q_tolerance = float(metadata.get("qdes_actual_q_hard_tolerance_rad", "nan"))
    if not math.isfinite(actual_q_tolerance) or actual_q_tolerance < 0.0:
        raise ValueError("qdes_actual_q_hard_tolerance_rad is invalid")

    env_sha = _sha256(frozen_env_path)
    return {
        # Unitree-compatible manager fields.  Extra HOPE fields are ignored by
        # a stock ManagerBasedRLEnv and consumed by the A3 ping-pong plugin.
        "format": "unitree_rl_lab.deploy",
        "format_version": 1,
        "task_plugin": "a3_pingpong_hitter_pure",
        "joint_ids_map": joint_ids_map,
        "joint_sdk_names": SDK_JOINT_NAMES,
        "step_dt": step_dt,
        "stiffness": stiffness_sdk,
        "damping": damping_sdk,
        "default_joint_pos": default_q,
        "actions": {
            "JointPositionAction": {
                "joint_ids": None,
                "joint_names": joint_names,
                "scale": action_scale,
                "offset": default_q,
                "clip": [[lo, hi] for lo, hi in zip(safe_lower, safe_upper)],
            }
        },
        "observations": {"policy": observations},
        "contracts": {
            "actor_observation": {
                "name": metadata.get("actor_obs_contract", ""),
                "total_dim": obs_dim,
                "ordered_terms": obs_names,
            },
            "action": {
                "name": qdes_contract,
                "total_dim": action_dim,
                "hard_lower_rad": hard_lower,
                "hard_upper_rad": hard_upper,
                "actual_q_hard_tolerance_rad": actual_q_tolerance,
                "policy_feedback": metadata.get("qdes_policy_feedback_contract", ""),
            },
            "runtime": metadata.get("hitter_pure_runtime_contract", ""),
            "training_recipe": {
                "name": metadata.get("hitter_pure_training_recipe", ""),
                "version": metadata.get("hitter_pure_training_recipe_version", ""),
            },
        },
        "provenance": {
            "policy_sha256": _sha256(policy_path),
            "frozen_env_sha256": env_sha,
            "checkpoint_sha256": metadata.get("hitter_pure_checkpoint_sha256", ""),
            "export_source": "run_frozen_env_plus_onnx_metadata",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--policy", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    policy_path = (args.policy or (run_dir / "exported" / "policy.onnx")).resolve()
    frozen_env_path = run_dir / "params" / "env.yaml"
    output_path = (args.output or (run_dir / "params" / "deploy.yaml")).resolve()
    if not policy_path.is_file():
        raise SystemExit(f"policy ONNX not found: {policy_path}")
    if not frozen_env_path.is_file():
        raise SystemExit(f"frozen env config not found: {frozen_env_path}")

    cfg = build_deploy_cfg(policy_path, frozen_env_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    print(f"[deploy] policy: {policy_path}")
    print(f"[deploy] config: {output_path}")
    print(f"[deploy] policy_sha256: {cfg['provenance']['policy_sha256']}")
    print(
        "[deploy] contract: "
        f"obs={cfg['contracts']['actor_observation']['total_dim']} "
        f"action={cfg['contracts']['action']['total_dim']} "
        f"step_dt={cfg['step_dt']:.6f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
