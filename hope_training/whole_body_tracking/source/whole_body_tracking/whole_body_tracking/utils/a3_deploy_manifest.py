"""Canonical A3 deployment contracts emitted beside V17 ONNX artifacts.

This module is intentionally independent of Isaac Lab.  The exporter passes in
the values already resolved by the live training environment; host tests and
read-only artifact inspectors can therefore validate the exact same contract
without starting a simulator.

V17-r6/r10 artifacts produced here are P0 contract candidates only.  They are
explicitly not qualification receipts and must never authorize hardware motion.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, MutableMapping, Sequence


A3_DEPLOY_MANIFEST_SCHEMA = "hope_a3_deploy_manifest_v1"
A3_DEPLOY_MANIFEST_STATUS = "p0_contract_only_not_hardware_authorized"
A3_DEPLOYMENT_STATUS = "p0_contract_candidate"
A3_QUALIFICATION_STATUS = "not_qualified"

# Backend slots used by robot_io::MakeA3Layout31 and pp_joint_map.hpp.
A3_BACKEND_JOINT_ORDER = (
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
)

# The fingerprint binds the human-readable manifest to the exact ONNX metadata
# consumed by the C++ runner.  Keep this order byte-identical to
# pp_deploy_contract.hpp.
V17_R6_P0_FINGERPRINT_KEYS = (
    "joint_names",
    "default_joint_pos",
    "action_scale",
    "joint_stiffness",
    "joint_damping",
    "a3_training_joint_damping",
    "a3_passive_joint_damping",
    "a3_joint_effort_limit",
    "qdes_action_contract",
    "qdes_policy_feedback_contract",
    "qdes_joint_names",
    "qdes_safe_lower_rad",
    "qdes_safe_upper_rad",
    "qdes_hard_lower_rad",
    "qdes_hard_upper_rad",
    "qdes_actual_q_hard_tolerance_rad",
    "actor_obs_contract",
    "actor_obs_total_dim",
    "actor_obs_term_dims",
    "actor_obs_term_sources_json",
    "hitter_pure_training_recipe",
    "hitter_pure_training_recipe_version",
    "hitter_pure_runtime_contract",
    "hitter_pure_action_contract",
    "hitter_pure_v17_recipe_revision",
    "hitter_pure_v17_sensor_contract",
    "base_localization_contract",
    "base_pose_source",
    "base_pose_schema",
    "orientation_contract",
    "angular_velocity_contract",
    "yaw_align_contract",
    "world_frame_contract",
    "calibration_contract",
    "base_mocap_max_age_s",
    "base_mocap_max_propagation_s",
    "a3_control_physics_dt_s",
    "a3_control_decimation",
    "a3_control_policy_dt_s",
    "v17_ground_plant_contract_json",
    "a3_qdes_parity_csv_sha256",
    "hitter_pure_checkpoint_sha256",
)

# R10 keeps the exact 110-D/31-D/A3 actuator contract above, but deliberately
# replaces the old move-to-station/READY release state machine.  These extra
# fields make that semantic change part of the signed payload instead of an
# unversioned runner flag.
V17_R10_P0_FINGERPRINT_KEYS = V17_R6_P0_FINGERPRINT_KEYS + (
    "hitter_pure_v17_fixed_station_contract",
    "hitter_pure_v17_release_contract",
    "hitter_pure_v17_target_stream_contract",
    "hitter_pure_planner_schema",
    "hitter_pure_planner_stability_contract",
    "hitter_pure_fixed_hit_plane_relative_x_m",
)


def metadata_wire_value(value: Any) -> str:
    """Serialize one ONNX metadata value without the old three-decimal loss."""
    if isinstance(value, (list, tuple)):
        if value and all(isinstance(item, str) for item in value):
            return ",".join(value)
        return ",".join(f"{float(item):.9g}" for item in value)
    return str(value)


def canonical_json(value: Any) -> str:
    return json.dumps(
        _jsonable(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "item"):
        return _jsonable(value.item())
    raise TypeError(f"value is not JSON serializable: {type(value).__name__}")


def _sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def _required_wire_metadata(
    metadata: Mapping[str, Any],
    keys: Sequence[str] = V17_R6_P0_FINGERPRINT_KEYS,
    *,
    label: str = "V17-r6 P0",
) -> dict[str, str]:
    missing = [key for key in keys if key not in metadata]
    if missing:
        raise ValueError(
            f"{label} deploy manifest is missing metadata: " + ", ".join(missing)
        )
    return {key: metadata_wire_value(metadata[key]) for key in keys}


def deploy_contract_fingerprint_payload(
    metadata: Mapping[str, Any],
    keys: Sequence[str] = V17_R6_P0_FINGERPRINT_KEYS,
    *,
    label: str = "V17-r6 P0",
) -> str:
    wire = _required_wire_metadata(metadata, keys, label=label)
    return "".join(f"{key}={wire[key]}\n" for key in keys)


def deploy_contract_fingerprint(
    metadata: Mapping[str, Any],
    keys: Sequence[str] = V17_R6_P0_FINGERPRINT_KEYS,
    *,
    label: str = "V17-r6 P0",
) -> str:
    return _sha256_text(
        deploy_contract_fingerprint_payload(metadata, keys, label=label)
    )


def _csv(value: Any, *, strings: bool = False) -> list[Any]:
    wire = metadata_wire_value(value)
    if not wire:
        return []
    fields = wire.split(",")
    return fields if strings else [float(field) for field in fields]


def _require_finite(values: Sequence[float], label: str, length: int = 31) -> None:
    if len(values) != length:
        raise ValueError(f"{label} must contain {length} values, got {len(values)}")
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"{label} contains NaN/Inf")


def decode_v11_affine_safe_qdes(
    action: Sequence[float],
    default_q: Sequence[float],
    action_scale: Sequence[float],
    safe_lower: Sequence[float],
    safe_upper: Sequence[float],
) -> list[float]:
    """Pure reference for the unchanged V17 affine -> final-clamp decoder."""
    lengths = {
        len(action),
        len(default_q),
        len(action_scale),
        len(safe_lower),
        len(safe_upper),
    }
    if len(lengths) != 1:
        raise ValueError("affine q_des vectors must have identical lengths")
    result: list[float] = []
    for raw, offset, scale, lower, upper in zip(
        action, default_q, action_scale, safe_lower, safe_upper
    ):
        values = tuple(float(value) for value in (raw, offset, scale, lower, upper))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("affine q_des input contains NaN/Inf")
        if not lower < upper:
            raise ValueError("affine q_des safe interval is empty")
        target = float(offset) + float(scale) * float(raw)
        result.append(min(max(target, float(lower)), float(upper)))
    return result


def build_v11_qdes_parity_csv(metadata: Mapping[str, Any]) -> str:
    """Deterministic raw-action/final-q_des vectors consumed by the C++ probe."""
    required = (
        "default_joint_pos",
        "action_scale",
        "qdes_safe_lower_rad",
        "qdes_safe_upper_rad",
    )
    missing = [key for key in required if key not in metadata]
    if missing:
        raise ValueError("q_des parity metadata is missing: " + ", ".join(missing))
    default_q = _csv(metadata["default_joint_pos"])
    action_scale = _csv(metadata["action_scale"])
    safe_lower = _csv(metadata["qdes_safe_lower_rad"])
    safe_upper = _csv(metadata["qdes_safe_upper_rad"])
    for label, values in (
        ("default_joint_pos", default_q),
        ("action_scale", action_scale),
        ("qdes_safe_lower_rad", safe_lower),
        ("qdes_safe_upper_rad", safe_upper),
    ):
        _require_finite(values, label)
    actions = [
        [0.0] * 31,
        [0.25 if index % 2 == 0 else -0.25 for index in range(31)],
        [1.0 if index % 3 else -1.0 for index in range(31)],
        [10.0 if index % 2 == 0 else -10.0 for index in range(31)],
        [((index * 17) % 23 - 11) / 3.0 for index in range(31)],
    ]
    rows: list[str] = []
    for action in actions:
        qdes = decode_v11_affine_safe_qdes(
            action, default_q, action_scale, safe_lower, safe_upper
        )
        rows.append(
            ",".join(f"{float(value):.17g}" for value in (*action, *qdes))
        )
    return "\n".join(rows) + "\n"


def qdes_parity_csv_sha256(content: str) -> str:
    return _sha256_text(content)


def write_qdes_parity_sidecar(
    onnx_path: str | Path, content: str, expected_sha256: str
) -> Path:
    if qdes_parity_csv_sha256(content) != expected_sha256:
        raise ValueError("q_des parity CSV content SHA256 mismatch before write")
    path = Path(onnx_path).with_suffix(".qdes_parity.csv")
    path.write_text(content, encoding="utf-8")
    return path


def _build_v17_p0_manifest(
    metadata: Mapping[str, Any],
    *,
    recipe_revision: int,
    runtime_contract: str,
    fingerprint_keys: Sequence[str],
) -> dict[str, Any]:
    """Validate resolved values and build one immutable V17 P0 document."""
    label = f"V17-r{recipe_revision} P0"
    wire = _required_wire_metadata(metadata, fingerprint_keys, label=label)
    expected_scalars = {
        "hitter_pure_training_recipe": "rally_v17",
        "hitter_pure_training_recipe_version": str(recipe_revision),
        "hitter_pure_runtime_contract": runtime_contract,
        "hitter_pure_action_contract": "v11_affine_safe_qdes_v1",
        "hitter_pure_v17_recipe_revision": str(recipe_revision),
        "qdes_action_contract": "v11_affine_safe_qdes_v1",
        "qdes_policy_feedback_contract": "legacy_applied_raw_v1",
        "actor_obs_contract": "hitter_pure",
        "actor_obs_total_dim": "110",
    }
    if recipe_revision == 10:
        expected_scalars.update(
            {
                "hitter_pure_v17_fixed_station_contract": (
                    "session_anchor_xy_with_10cm_recovery_v1"
                ),
                "hitter_pure_v17_release_contract": (
                    "telemetry_only_ball_clock_v1"
                ),
                "hitter_pure_v17_target_stream_contract": "freeze_at_engage_v1",
                "hitter_pure_planner_schema": "2",
                "hitter_pure_planner_stability_contract": (
                    "three_revisions_v1,0.0300,0.2500,0.0300"
                ),
                "hitter_pure_fixed_hit_plane_relative_x_m": "0.5800",
            }
        )
    drift = {
        key: (wire[key], expected)
        for key, expected in expected_scalars.items()
        if wire[key] != expected
    }
    if drift:
        raise ValueError(f"{label} contract identity drifted: {drift}")

    checkpoint_sha256 = wire["hitter_pure_checkpoint_sha256"]
    if not _is_sha256(checkpoint_sha256):
        raise ValueError(f"{label} checkpoint SHA256 is missing or malformed")
    qdes_parity_sha256 = wire["a3_qdes_parity_csv_sha256"]
    if not _is_sha256(qdes_parity_sha256):
        raise ValueError(f"{label} q_des parity SHA256 is missing or malformed")

    joint_names = _csv(wire["joint_names"], strings=True)
    qdes_joint_names = _csv(wire["qdes_joint_names"], strings=True)
    if len(joint_names) != 31 or len(set(joint_names)) != 31:
        raise ValueError(f"{label} policy joint order is not a 31-joint bijection")
    if joint_names != qdes_joint_names:
        raise ValueError("qdes_joint_names do not exactly match policy joint order")
    if set(joint_names) != set(A3_BACKEND_JOINT_ORDER):
        raise ValueError(f"{label} joint names do not form the A3 backend joint set")
    policy_to_backend = [A3_BACKEND_JOINT_ORDER.index(name) for name in joint_names]

    default_q = _csv(wire["default_joint_pos"])
    action_scale = _csv(wire["action_scale"])
    kp = _csv(wire["joint_stiffness"])
    kd_wire = _csv(wire["joint_damping"])
    kd_training = _csv(wire["a3_training_joint_damping"])
    kd_passive = _csv(wire["a3_passive_joint_damping"])
    effort = _csv(wire["a3_joint_effort_limit"])
    safe_lower = _csv(wire["qdes_safe_lower_rad"])
    safe_upper = _csv(wire["qdes_safe_upper_rad"])
    hard_lower = _csv(wire["qdes_hard_lower_rad"])
    hard_upper = _csv(wire["qdes_hard_upper_rad"])
    arrays = {
        "default_joint_pos": default_q,
        "action_scale": action_scale,
        "joint_stiffness": kp,
        "joint_damping": kd_wire,
        "a3_training_joint_damping": kd_training,
        "a3_passive_joint_damping": kd_passive,
        "a3_joint_effort_limit": effort,
        "qdes_safe_lower_rad": safe_lower,
        "qdes_safe_upper_rad": safe_upper,
        "qdes_hard_lower_rad": hard_lower,
        "qdes_hard_upper_rad": hard_upper,
    }
    for array_label, values in arrays.items():
        _require_finite(values, array_label)

    for index, name in enumerate(joint_names):
        if kp[index] <= 0.0 or kd_wire[index] <= 0.0 or effort[index] <= 0.0:
            raise ValueError(f"non-positive actuator value for {name}")
        if abs(kd_training[index] - (kd_wire[index] + kd_passive[index])) > 1e-5:
            raise ValueError(
                f"A3 damping split drifted for {name}: training != wire + passive"
            )
        expected_scale = 0.25 * effort[index] / kp[index]
        if abs(action_scale[index] - expected_scale) > 2e-6:
            raise ValueError(
                f"A3 Unitree-style action scale drifted for {name}: "
                f"{action_scale[index]} != 0.25*{effort[index]}/{kp[index]}"
            )
        if not (
            hard_lower[index]
            <= safe_lower[index]
            < default_q[index]
            < safe_upper[index]
            <= hard_upper[index]
        ):
            raise ValueError(f"A3 q_des hard/safe/default nesting drifted for {name}")

    physics_dt_s = float(wire["a3_control_physics_dt_s"])
    decimation = int(wire["a3_control_decimation"])
    policy_dt_s = float(wire["a3_control_policy_dt_s"])
    if (
        not math.isfinite(physics_dt_s)
        or physics_dt_s <= 0.0
        or decimation <= 0
        or abs(policy_dt_s - physics_dt_s * decimation) > 1e-12
        or abs(policy_dt_s - 0.02) > 1e-12
    ):
        raise ValueError(
            f"{label} control timing must resolve to the shared 50 Hz policy contract"
        )

    plant = json.loads(wire["v17_ground_plant_contract_json"])
    if not isinstance(plant, dict) or not plant:
        raise ValueError(f"{label} ground plant receipt is empty")
    observation_term_dimensions = [
        int(round(value)) for value in _csv(wire["actor_obs_term_dims"])
    ]
    if (
        not observation_term_dimensions
        or any(value <= 0 for value in observation_term_dimensions)
        or sum(observation_term_dimensions) != 110
    ):
        raise ValueError(
            f"{label} actor observation term dimensions do not sum to 110"
        )
    observation_term_sources = json.loads(
        wire["actor_obs_term_sources_json"]
    )
    if not isinstance(observation_term_sources, dict):
        raise ValueError(f"{label} actor observation sources are not a mapping")
    fingerprint = deploy_contract_fingerprint(
        metadata, fingerprint_keys, label=label
    )
    manifest = {
        "schema": A3_DEPLOY_MANIFEST_SCHEMA,
        "status": A3_DEPLOY_MANIFEST_STATUS,
        "hardware_authorized": False,
        "qualification_status": A3_QUALIFICATION_STATUS,
        "contract_fingerprint_sha256": fingerprint,
        "robot": {
            "family": "agibot_a3",
            "dof": 31,
            "policy_joint_order": joint_names,
            "backend_joint_order": list(A3_BACKEND_JOINT_ORDER),
            "policy_to_backend": policy_to_backend,
        },
        "timing": {
            "physics_dt_s": physics_dt_s,
            "control_decimation": decimation,
            "policy_dt_s": policy_dt_s,
            "policy_hz": 1.0 / policy_dt_s,
        },
        "observation": {
            "contract": wire["actor_obs_contract"],
            "dimension": int(wire["actor_obs_total_dim"]),
            "term_dimensions": observation_term_dimensions,
            "term_sources": observation_term_sources,
        },
        "action": {
            "dimension": 31,
            "contract": wire["qdes_action_contract"],
            "policy_feedback_contract": wire["qdes_policy_feedback_contract"],
            "decode": "q_des=clip(default_q+action_scale*raw_action,safe_lower,safe_upper)",
            "default_q_rad": default_q,
            "action_scale_rad": action_scale,
            "safe_lower_rad": safe_lower,
            "safe_upper_rad": safe_upper,
            "hard_lower_rad": hard_lower,
            "hard_upper_rad": hard_upper,
            "actual_q_hard_tolerance_rad": float(
                wire["qdes_actual_q_hard_tolerance_rad"]
            ),
        },
        "actuator": {
            "kp_nominal": kp,
            "kd_wire": kd_wire,
            "kd_passive_plant": kd_passive,
            "kd_training_total": kd_training,
            "effort_limit": effort,
            "action_scale_formula": "0.25*effort_limit/kp_nominal",
            "pd_randomization_changes_plant_only": True,
        },
        "localization": {
            "base_localization_contract": wire["base_localization_contract"],
            "base_pose_source": wire["base_pose_source"],
            "base_pose_schema": int(wire["base_pose_schema"]),
            "orientation_contract": wire["orientation_contract"],
            "angular_velocity_contract": wire["angular_velocity_contract"],
            "yaw_align_contract": wire["yaw_align_contract"],
            "world_frame_contract": wire["world_frame_contract"],
            "calibration_contract": wire["calibration_contract"],
            "max_age_s": float(wire["base_mocap_max_age_s"]),
            "max_propagation_s": float(wire["base_mocap_max_propagation_s"]),
        },
        "training": {
            "recipe": wire["hitter_pure_training_recipe"],
            "recipe_revision": int(wire["hitter_pure_v17_recipe_revision"]),
            "runtime_contract": wire["hitter_pure_runtime_contract"],
            "sensor_contract": wire["hitter_pure_v17_sensor_contract"],
            "ground_plant": plant,
        },
        "provenance": {
            "checkpoint_sha256": checkpoint_sha256,
            "qdes_parity_csv_sha256": qdes_parity_sha256,
        },
    }
    if recipe_revision == 10:
        manifest["execution"] = {
            "station": wire["hitter_pure_v17_fixed_station_contract"],
            "release": wire["hitter_pure_v17_release_contract"],
            "target_stream": wire["hitter_pure_v17_target_stream_contract"],
            "planner_schema": int(wire["hitter_pure_planner_schema"]),
            "planner_stability": wire[
                "hitter_pure_planner_stability_contract"
            ],
            "hit_plane_relative_x_m": float(
                wire["hitter_pure_fixed_hit_plane_relative_x_m"]
            ),
        }
    return manifest


def build_v17_r6_p0_manifest(metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Backward-compatible V17-r6 P0 manifest builder."""
    return _build_v17_p0_manifest(
        metadata,
        recipe_revision=6,
        runtime_contract="rally_final_v2",
        fingerprint_keys=V17_R6_P0_FINGERPRINT_KEYS,
    )


def build_v17_r10_p0_manifest(metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Build the fixed-station, ball-clock V17-r10 P0 contract."""
    return _build_v17_p0_manifest(
        metadata,
        recipe_revision=10,
        runtime_contract="rally_v17_fixed_station_ball_clock_v1",
        fingerprint_keys=V17_R10_P0_FINGERPRINT_KEYS,
    )


def attach_v17_r6_p0_manifest_metadata(
    metadata: MutableMapping[str, Any],
) -> tuple[dict[str, Any], str, str]:
    manifest = build_v17_r6_p0_manifest(metadata)
    manifest_json = canonical_json(manifest)
    manifest_sha256 = _sha256_text(manifest_json)
    metadata.update(
        {
            "a3_deploy_manifest_schema": A3_DEPLOY_MANIFEST_SCHEMA,
            "a3_deploy_manifest_status": A3_DEPLOY_MANIFEST_STATUS,
            "a3_deploy_hardware_authorized": "false",
            "a3_deploy_contract_fingerprint_sha256": manifest[
                "contract_fingerprint_sha256"
            ],
            "a3_deploy_manifest_json": manifest_json,
            "a3_deploy_manifest_sha256": manifest_sha256,
        }
    )
    return manifest, manifest_json, manifest_sha256


def attach_v17_r10_p0_manifest_metadata(
    metadata: MutableMapping[str, Any],
) -> tuple[dict[str, Any], str, str]:
    """Attach the immutable fixed-station R10 contract to ONNX metadata."""
    manifest = build_v17_r10_p0_manifest(metadata)
    manifest_json = canonical_json(manifest)
    manifest_sha256 = _sha256_text(manifest_json)
    metadata.update(
        {
            "a3_deploy_manifest_schema": A3_DEPLOY_MANIFEST_SCHEMA,
            "a3_deploy_manifest_status": A3_DEPLOY_MANIFEST_STATUS,
            "a3_deploy_hardware_authorized": "false",
            "a3_deploy_contract_fingerprint_sha256": manifest[
                "contract_fingerprint_sha256"
            ],
            "a3_deploy_manifest_json": manifest_json,
            "a3_deploy_manifest_sha256": manifest_sha256,
        }
    )
    return manifest, manifest_json, manifest_sha256


def write_deploy_manifest_sidecar(
    onnx_path: str | Path,
    manifest: Mapping[str, Any],
    manifest_sha256: str,
) -> Path:
    """Write the human-readable receipt; the ONNX-embedded copy is authoritative."""
    onnx_path = Path(onnx_path)
    sidecar = onnx_path.with_suffix(".deploy.json")
    receipt = {
        "schema": "hope_a3_deploy_manifest_sidecar_v1",
        "onnx_file": onnx_path.name,
        "manifest_sha256": manifest_sha256,
        "manifest": _jsonable(manifest),
        "note": (
            "P0 contract receipt only; qualification_status=not_qualified and "
            "hardware_authorized=false"
        ),
    }
    sidecar.write_text(
        json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return sidecar


def verify_v17_r6_p0_metadata(metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Read-only verification used by tests and the artifact inspection CLI."""
    expected = {
        "a3_deploy_manifest_schema": A3_DEPLOY_MANIFEST_SCHEMA,
        "a3_deploy_manifest_status": A3_DEPLOY_MANIFEST_STATUS,
        "a3_deploy_hardware_authorized": "false",
        "hitter_pure_deployment_status": A3_DEPLOYMENT_STATUS,
        "hitter_pure_qualification_status": A3_QUALIFICATION_STATUS,
    }
    drift = {
        key: (metadata.get(key), value)
        for key, value in expected.items()
        if metadata.get(key) != value
    }
    if drift:
        raise ValueError(f"V17-r6 P0 status metadata drifted: {drift}")
    fingerprint = deploy_contract_fingerprint(metadata)
    if metadata.get("a3_deploy_contract_fingerprint_sha256") != fingerprint:
        raise ValueError("V17-r6 P0 metadata fingerprint mismatch")
    manifest_json = str(metadata.get("a3_deploy_manifest_json", ""))
    if metadata.get("a3_deploy_manifest_sha256") != _sha256_text(manifest_json):
        raise ValueError("V17-r6 P0 embedded manifest SHA256 mismatch")
    manifest = json.loads(manifest_json)
    if canonical_json(manifest) != manifest_json:
        raise ValueError("V17-r6 P0 embedded manifest is not canonical JSON")
    if manifest.get("contract_fingerprint_sha256") != fingerprint:
        raise ValueError("V17-r6 P0 manifest/metadata fingerprint mismatch")
    if manifest.get("hardware_authorized") is not False:
        raise ValueError("V17-r6 P0 manifest unexpectedly authorizes hardware")
    expected_manifest = build_v17_r6_p0_manifest(metadata)
    if manifest != expected_manifest:
        raise ValueError(
            "V17-r6 P0 embedded manifest does not match resolved metadata"
        )
    return manifest


def verify_v17_r10_p0_metadata(metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Read-only verifier for a fixed-station R10 P0 artifact."""
    expected = {
        "a3_deploy_manifest_schema": A3_DEPLOY_MANIFEST_SCHEMA,
        "a3_deploy_manifest_status": A3_DEPLOY_MANIFEST_STATUS,
        "a3_deploy_hardware_authorized": "false",
        "hitter_pure_deployment_status": A3_DEPLOYMENT_STATUS,
        "hitter_pure_qualification_status": A3_QUALIFICATION_STATUS,
        "hitter_pure_training_recipe_version": "10",
        "hitter_pure_v17_recipe_revision": "10",
        "hitter_pure_runtime_contract": (
            "rally_v17_fixed_station_ball_clock_v1"
        ),
    }
    drift = {
        key: (metadata.get(key), value)
        for key, value in expected.items()
        if metadata.get(key) != value
    }
    if drift:
        raise ValueError(f"V17-r10 P0 status metadata drifted: {drift}")
    fingerprint = deploy_contract_fingerprint(
        metadata, V17_R10_P0_FINGERPRINT_KEYS, label="V17-r10 P0"
    )
    if metadata.get("a3_deploy_contract_fingerprint_sha256") != fingerprint:
        raise ValueError("V17-r10 P0 metadata fingerprint mismatch")
    manifest_json = str(metadata.get("a3_deploy_manifest_json", ""))
    if metadata.get("a3_deploy_manifest_sha256") != _sha256_text(manifest_json):
        raise ValueError("V17-r10 P0 embedded manifest SHA256 mismatch")
    manifest = json.loads(manifest_json)
    if canonical_json(manifest) != manifest_json:
        raise ValueError("V17-r10 P0 embedded manifest is not canonical JSON")
    if manifest.get("contract_fingerprint_sha256") != fingerprint:
        raise ValueError("V17-r10 P0 manifest/metadata fingerprint mismatch")
    if manifest.get("hardware_authorized") is not False:
        raise ValueError("V17-r10 P0 manifest unexpectedly authorizes hardware")
    expected_manifest = build_v17_r10_p0_manifest(metadata)
    if manifest != expected_manifest:
        raise ValueError(
            "V17-r10 P0 embedded manifest does not match resolved metadata"
        )
    return manifest
