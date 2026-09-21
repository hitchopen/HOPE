#!/usr/bin/env python3
"""Headless MuJoCo regression for Spin001 over exact official PD_STAND.

The simulation model has no gripper. The production 50 Hz arm/neck Spin001
commands are replayed unchanged, while all waist/leg command fields are
replaced on every tick by the Runner's official PD_STAND vectors. An optional
left-wrist payload represents the missing gripper and ball. No policy command,
stance offset, gain multiplier, or IMU correction is used.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


REPO = Path(__file__).resolve().parents[3]
DEFAULT_MODEL = REPO / (
    "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/models/bin/cfg/model/"
    "a3_pingpong/a3_pingpong.xml"
)
MOTION_ROOT = REPO / "a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions"
SERVE_HEADER = REPO / (
    "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/"
    "a3_pingpong/pp_serve_controller.hpp"
)
SERVE_SOURCE = REPO / (
    "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/src/a3_deploy/"
    "pp_serve_controller.cpp"
)
RUNNER_SOURCE = REPO / (
    "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/src/a3_deploy/"
    "a3_pingpong_main.cpp"
)
PARAMETERS_HEADER = REPO / (
    "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/"
    "a3_policy_parameters.hpp"
)

MOTION_FILES = {
    "entry": MOTION_ROOT / "a3p_op3_spin001_deploy_parity_entry_v1.csv",
    "timed_serve": MOTION_ROOT
    / "a3p_op3_spin001_deploy_parity_timed_serve_v1.csv",
    "recovery": MOTION_ROOT
    / "a3p_op3_spin001_deploy_parity_recovery_v1.csv",
}
MOTION_SHA256 = {
    "entry": "db74f7461e0fe93886f0f1cdc84ed740561203bbf48ec485a5316f93da79fa53",
    "timed_serve": "f03e3f0f7825d8f079b5ec36b14c52723db145f8e8989212dc1530f16815ba4e",
    "recovery": "dbb59e646c103020a393feb3ebcff7504794116d47b0c92dac5b1732c87a590d",
}
EXPECTED_FRAMES = {"entry": 250, "timed_serve": 84, "recovery": 151}

SDK_ORDER = (
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "head_yaw_joint", "head_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    "left_hip_pitch_joint", "left_hip_roll_joint",
    "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
)
POLICY_TO_SDK = (
    0, 1, 2,
    5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30,
)
FIXED_PD_STAND_SDK = tuple(range(0, 3)) + tuple(range(19, 31))
UPPER_BODY_SDK = tuple(range(3, 19))
WAIT_READY_FRAMES = 100
CONTROL_SUBSTEPS = 20


def _cpp_array(name: str) -> np.ndarray:
    text = PARAMETERS_HEADER.read_text(encoding="utf-8")
    match = re.search(rf"\b{name}\s*=\s*\{{(.*?)\}};", text, re.DOTALL)
    if match is None:
        raise RuntimeError(f"production array {name} is missing")
    body = re.sub(r"//.*", "", match.group(1))
    values = [
        float(value)
        for value in re.findall(
            r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", body
        )
    ]
    if len(values) != 29:
        raise RuntimeError(f"production array {name} is not 29-D")
    return np.asarray(values, dtype=np.float64)


def _official_stand_vectors() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = np.zeros(31, dtype=np.float64)
    kp = np.zeros(31, dtype=np.float64)
    kd = np.zeros(31, dtype=np.float64)
    q[list(POLICY_TO_SDK)] = _cpp_array("a3_default_angles")
    kp[list(POLICY_TO_SDK)] = _cpp_array("a3_pd_stand_kps")
    kd[list(POLICY_TO_SDK)] = _cpp_array("a3_pd_stand_kds")
    # Neck is not part of the fixed support set, but these are the exact
    # official Runner PD_STAND neck values and keep the 31-D receipt complete.
    kp[3:5] = 40.0
    kd[3:5] = 2.0
    return q, kp, kd


def _verify_production_contract() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    header = SERVE_HEADER.read_text(encoding="utf-8")
    source = SERVE_SOURCE.read_text(encoding="utf-8")
    runner = RUNNER_SOURCE.read_text(encoding="utf-8")
    forbidden = (
        "ServeStanceConfig",
        "stance_offset_rad",
        "fixed_stance_kd_scale",
        "ankle_pitch_kd_boost",
        "ApplyPitchCompensation_",
        "policy_balance",
    )
    for token in forbidden:
        if token in header or token in source:
            raise RuntimeError(
                f"production Serve still contains forbidden support logic: {token}"
            )
    for contract in (
        "for (const int sdk : kFixedStandSdk)",
        "command.q_des[sdk] = official_stand_q_sdk_[sdk];",
        "command.dq_des[sdk] = 0.0;",
        "command.tau_ff[sdk] = 0.0;",
        "command.kp[sdk] = official_stand_kp_sdk_[sdk];",
        "command.kd[sdk] = official_stand_kd_sdk_[sdk];",
    ):
        if contract not in source:
            raise RuntimeError(
                f"production Serve is missing exact PD_STAND copy: {contract}"
            )
    for contract in (
        "pp->official_stand_q()",
        "pp->official_stand_kp()",
        "pp->official_stand_kd()",
    ):
        if contract not in runner:
            raise RuntimeError(
                f"Runner does not pass authoritative PD_STAND: {contract}"
            )
    return _official_stand_vectors()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_segment(name: str, path: Path) -> dict[str, np.ndarray]:
    if _sha256(path) != MOTION_SHA256[name]:
        raise RuntimeError(f"Spin001 {name} SHA-256 mismatch: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != EXPECTED_FRAMES[name]:
        raise RuntimeError(f"Spin001 {name} frame-count mismatch")
    for frame, row in enumerate(rows):
        if int(row["frame_index"]) != frame or not math.isclose(
            float(row["time_s"]), frame / 50.0, abs_tol=1.0e-12
        ):
            raise RuntimeError(f"Spin001 {name} is not on the exact 50 Hz grid")
    prefixes = {
        "q": "q_des_rad::",
        "dq": "dq_des_rad_s::",
        "tau": "tau_ff_nm::",
        "kp": "kp_nm_per_rad::",
        "kd": "kd_nm_s_per_rad::",
    }
    return {
        key: np.asarray(
            [[float(row[prefix + joint]) for joint in SDK_ORDER] for row in rows],
            dtype=np.float64,
        )
        for key, prefix in prefixes.items()
    }


def _tilt_deg(quaternion_wxyz: np.ndarray) -> float:
    _, x, y, _ = quaternion_wxyz
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(min(max(up_z, -1.0), 1.0)))


def _compile_model(path: Path):
    model = mujoco.MjModel.from_xml_path(str(path))
    if not math.isclose(float(model.opt.timestep), 0.001, abs_tol=1.0e-12):
        raise RuntimeError("Spin001 regression requires the exact 1 kHz plant")
    bindings = []
    for joint in SDK_ORDER:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint + "_motor"
        )
        if joint_id < 0 or actuator_id < 0:
            raise RuntimeError(f"MuJoCo binding missing for {joint}")
        bindings.append(
            (
                int(model.jnt_qposadr[joint_id]),
                int(model.jnt_dofadr[joint_id]),
                actuator_id,
            )
        )
    return model, bindings


def _execution(segments: dict[str, dict[str, np.ndarray]]):
    yield "entry", segments["entry"]
    yield "wait_ready", {
        key: np.repeat(value[-1:, :], WAIT_READY_FRAMES, axis=0)
        for key, value in segments["entry"].items()
    }
    yield "timed_serve", segments["timed_serve"]
    yield "recovery", segments["recovery"]


def _run_case(
    model,
    bindings,
    segments: dict[str, dict[str, np.ndarray]],
    official_q: np.ndarray,
    official_kp: np.ndarray,
    official_kd: np.ndarray,
    *,
    name: str,
    left_wrist_payload_kg: float,
    forward_impulse_n: float = 0.0,
) -> dict[str, Any]:
    data = mujoco.MjData(model)
    stand_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, stand_key)
    for sdk, (qadr, dadr, _) in enumerate(bindings):
        data.qpos[qadr] = official_q[sdk]
        data.qvel[dadr] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_Link")
    wrist = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_Link"
    )
    left_foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_foot")
    right_foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_foot")

    initial_z = float(data.qpos[2])
    max_tilt = _tilt_deg(data.qpos[3:7])
    max_z_drop = 0.0
    both_feet_steps = 0
    steps = 0
    first_fall_s: float | None = None
    fields = ("q", "dq", "tau", "kp", "kd")
    max_contract_error = {field: 0.0 for field in fields}
    max_upper_body_error = {field: 0.0 for field in fields}

    support = list(FIXED_PD_STAND_SDK)
    upper = list(UPPER_BODY_SDK)
    for _segment_name, segment in _execution(segments):
        for frame in range(len(segment["q"])):
            source = {field: segment[field][frame].copy() for field in fields}
            command = {field: value.copy() for field, value in source.items()}
            command["q"][support] = official_q[support]
            command["dq"][support] = 0.0
            command["tau"][support] = 0.0
            command["kp"][support] = official_kp[support]
            command["kd"][support] = official_kd[support]

            expected_support = {
                "q": official_q[support],
                "dq": np.zeros(len(support)),
                "tau": np.zeros(len(support)),
                "kp": official_kp[support],
                "kd": official_kd[support],
            }
            for field in fields:
                max_contract_error[field] = max(
                    max_contract_error[field],
                    float(np.max(np.abs(
                        command[field][support] - expected_support[field]
                    ))),
                )
                max_upper_body_error[field] = max(
                    max_upper_body_error[field],
                    float(np.max(np.abs(
                        command[field][upper] - source[field][upper]
                    ))),
                )

            for _ in range(CONTROL_SUBSTEPS):
                data.xfrc_applied[:, :] = 0.0
                data.xfrc_applied[wrist, 2] = -left_wrist_payload_kg * 9.81
                elapsed = steps * float(model.opt.timestep)
                if 6.40 <= elapsed < 6.50:
                    data.xfrc_applied[torso, 0] = forward_impulse_n
                for sdk, (qadr, dadr, actuator) in enumerate(bindings):
                    effort = (
                        command["tau"][sdk]
                        + command["kp"][sdk]
                        * (command["q"][sdk] - data.qpos[qadr])
                        + command["kd"][sdk]
                        * (command["dq"][sdk] - data.qvel[dadr])
                    )
                    low, high = model.actuator_ctrlrange[actuator]
                    data.ctrl[actuator] = min(max(effort, low), high)
                mujoco.mj_step(model, data)
                steps += 1
                tilt = _tilt_deg(data.qpos[3:7])
                z_drop = initial_z - float(data.qpos[2])
                max_tilt = max(max_tilt, tilt)
                max_z_drop = max(max_z_drop, z_drop)
                both_feet_steps += int(
                    data.site_xpos[left_foot, 2] < 0.035
                    and data.site_xpos[right_foot, 2] < 0.035
                )
                if first_fall_s is None and (tilt > 30.0 or z_drop > 0.25):
                    first_fall_s = float(data.time)

    return {
        "name": name,
        "policy_used": False,
        "left_wrist_payload_kg": left_wrist_payload_kg,
        "forward_impulse_n": forward_impulse_n,
        "duration_s": steps * float(model.opt.timestep),
        "fell": first_fall_s is not None,
        "first_fall_s": first_fall_s,
        "pelvis_tilt_max_deg": max_tilt,
        "pelvis_z_max_drop_m": max_z_drop,
        "both_feet_near_floor_fraction": both_feet_steps / steps,
        "max_fixed_pd_stand_command_error": max_contract_error,
        "max_upper_body_spin001_command_error": max_upper_body_error,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    official_q, official_kp, official_kd = _verify_production_contract()
    segments = {
        name: _load_segment(name, path) for name, path in MOTION_FILES.items()
    }
    if not np.array_equal(
        segments["entry"]["q"][0][list(FIXED_PD_STAND_SDK)],
        official_q[list(FIXED_PD_STAND_SDK)],
    ):
        raise RuntimeError(
            "Spin001 ENTRY support endpoint differs from official PD_STAND"
        )

    model, bindings = _compile_model(args.model)
    cases = [
        _run_case(
            model,
            bindings,
            segments,
            official_q,
            official_kp,
            official_kd,
            name="exact_pd_stand_nominal",
            left_wrist_payload_kg=0.0,
        ),
        _run_case(
            model,
            bindings,
            segments,
            official_q,
            official_kp,
            official_kd,
            name="exact_pd_stand_gripper_ball_payload",
            left_wrist_payload_kg=0.6,
        ),
        _run_case(
            model,
            bindings,
            segments,
            official_q,
            official_kp,
            official_kd,
            name="exact_pd_stand_payload_impulse",
            left_wrist_payload_kg=0.6,
            forward_impulse_n=35.0,
        ),
    ]
    command_exact = all(
        all(
            error == 0.0
            for error in case["max_fixed_pd_stand_command_error"].values()
        )
        and all(
            error == 0.0
            for error in case["max_upper_body_spin001_command_error"].values()
        )
        for case in cases
    )
    stable = all(
        not case["fell"]
        and case["pelvis_tilt_max_deg"] <= 15.0
        and case["pelvis_z_max_drop_m"] <= 0.10
        and case["both_feet_near_floor_fraction"] >= 0.95
        for case in cases
    )
    receipt = {
        "schema": 2,
        "verdict": "PASS" if command_exact and stable else "FAIL",
        "production_config_bound": True,
        "policy_used": False,
        "support_controller": "exact_official_pd_stand",
        "model": str(args.model.resolve()),
        "motion_sha256": MOTION_SHA256,
        "fixed_pd_stand_sdk_indices": list(FIXED_PD_STAND_SDK),
        "upper_body_spin001_sdk_indices": list(UPPER_BODY_SDK),
        "cases": cases,
    }
    rendered = json.dumps(receipt, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if receipt["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
