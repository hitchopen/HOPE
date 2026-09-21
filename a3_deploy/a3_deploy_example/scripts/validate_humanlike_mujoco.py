#!/usr/bin/env python3
"""Native HumanLike actor/encoder and command transitions in local MuJoCo.
No HAL, ROS, gripper or physical ball. Reports are simulation evidence only.
"""

import argparse
import ctypes
import json
import time
from pathlib import Path

import mujoco
import numpy as np
import validate_spin001_serve_mujoco as sim
from runner_sim_profile import apply_hope_contact_profile


def run(args, name, velocity):
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    lib = ctypes.CDLL(str(args.library.resolve()))
    lib.humanlike_create.argtypes = [ctypes.c_char_p]
    lib.humanlike_create.restype = ctypes.c_void_p
    lib.humanlike_reset.argtypes = [ctypes.c_void_p, ptr, ptr]
    lib.humanlike_step.argtypes = [ctypes.c_void_p, ptr, ptr, ptr]
    lib.humanlike_destroy.argtypes = [ctypes.c_void_p]
    lib.probe_create.argtypes = [ctypes.c_char_p, ptr]
    lib.probe_create.restype = ctypes.c_void_p
    lib.probe_begin.argtypes = [ctypes.c_void_p, ptr, ptr, ctypes.c_double]
    lib.probe_apply.argtypes = [ctypes.c_void_p, ctypes.c_double, ptr]
    lib.probe_destroy.argtypes = [ctypes.c_void_p]
    m, bindings = sim._compile_model(sim.DEFAULT_MODEL)
    apply_hope_contact_profile(m)
    qa, va, ac = np.array(bindings).T
    d = mujoco.MjData(m)
    stand_q, kp, kd = sim._official_stand_vectors()
    stand = np.array([stand_q, stand_q * 0, stand_q * 0, kp, kd])
    mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
    d.qpos[:2] = [-0.45, -0.7625]
    d.qpos[qa] = stand_q
    ball = m.jnt_qposadr[m.joint("gate3_ball_free_joint").id]
    d.qpos[ball : ball + 3] = [100, 0, -10]
    mujoco.mj_forward(m, d)
    pelvis = m.body("pelvis_link").id
    policy = lib.humanlike_create(str(args.policy_dir.resolve()).encode())
    csv = (
        sim.MOTION_ROOT
        / "a3p_op3_serve025_new_build4_deep_1p07_compact50_lowdrop35_strikewindow180_full31_balanced_face20deg_forwardhit_v4.csv"
    )
    blend = lib.probe_create(str(csv).encode(), stand)
    if not policy or not blend:
        raise RuntimeError("native controller initialization failed")
    cmd = stand.copy()
    rows = []
    boundary = []
    exit_time = None
    stop_dwell = 0
    failed = False
    settled = 0
    compute_us = []
    stop_origin = None
    stop_distance = 0.0
    try:
        for tick in range(int(args.max_seconds / .002)):
            t = tick * 0.002
            before = cmd.copy()
            spatial = np.zeros(6)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, pelvis, spatial, 1)
            state = np.r_[d.qpos[qa], d.qvel[va], d.xquat[pelvis], spatial[:3]]
            if tick == 500:
                assert lib.humanlike_reset(policy, state, before) == 0
            if tick >= 500 and exit_time is None:
                target = np.array(
                    velocity if 5 <= t < args.stop_time else [0.0, 0.0, 0.0], dtype=np.float64
                )
                if name.startswith("reverse") and (5 + args.stop_time) / 2 <= t < args.stop_time:
                    target[0] = -0.15
                compute_start = time.perf_counter_ns()
                settled = lib.humanlike_step(policy, state, target, cmd)
                compute_us.append((time.perf_counter_ns() - compute_start) / 1000)
                if settled < 0:
                    raise RuntimeError("native policy rejected state/command")
                if tick == 500:
                    assert lib.probe_begin(blend, before, cmd, 2.0) == 0
                assert lib.probe_apply(blend, t - 1, cmd) == 0
                if tick == 500:
                    boundary.append(np.max(abs(cmd - before), axis=1).tolist())
                quiet = (
                    settled == 1
                    and max(abs(state[31:62])) < 0.5
                    and np.linalg.norm(spatial[:3]) < 0.15
                )
                stop_dwell = stop_dwell + 1 if t >= args.stop_time and quiet else 0
                if stop_dwell >= 150:
                    exit_time = t + 0.002
                    assert lib.probe_begin(blend, cmd, stand, 0.5) == 0
            elif exit_time is not None:
                cmd = stand.copy()
                assert lib.probe_apply(blend, max(0.0, t - exit_time), cmd) == 0
                if abs(t - exit_time) < 1e-8:
                    boundary.append(np.max(abs(cmd - before), axis=1).tolist())
            for _ in range(2):
                torque = (
                    cmd[3] * (cmd[0] - d.qpos[qa])
                    + cmd[4] * (cmd[1] - d.qvel[va])
                    + cmd[2]
                )
                d.ctrl[ac] = np.clip(
                    torque, m.actuator_ctrlrange[ac, 0], m.actuator_ctrlrange[ac, 1]
                )
                mujoco.mj_step(m, d)
            if t >= args.stop_time and stop_origin is None:
                stop_origin = d.qpos[:2].copy()
            if stop_origin is not None:
                stop_distance = max(
                    stop_distance, float(np.linalg.norm(d.qpos[:2] - stop_origin))
                )
            tilt = float(np.arccos(np.clip(d.xmat[pelvis].reshape(3, 3)[2, 2], -1, 1)))
            if tick % 10 == 0:
                rows.append(
                    [
                        float(d.time),
                        *d.qpos[:3],
                        tilt,
                        *np.max(abs(cmd - before), axis=1),
                        settled,
                        float(np.max(abs(state[31:62]))),
                        float(np.max(abs(state[50:62]))),
                        float(np.linalg.norm(spatial[:3])),
                    ]
                )
            if tilt > 0.8 or d.qpos[2] < 0.55:
                failed = True
                break
            if exit_time is not None and t - exit_time >= 3:
                break
    finally:
        lib.humanlike_destroy(policy)
        lib.probe_destroy(blend)
    trace = np.asarray(rows)
    np.save(args.output.parent / f"humanlike_{name}.npy", trace)
    foot_delta = d.xmat[pelvis].reshape(3, 3).T @ (
        d.xpos[m.body("left_ankle_roll_Link").id]
        - d.xpos[m.body("right_ankle_roll_Link").id]
    )
    return {
        "status": "PASS" if not failed and exit_time is not None else "FAIL",
        "seconds": float(d.time),
        "max_tilt_rad": float(trace[:, 4].max()),
        "min_root_height_m": float(trace[:, 3].min()),
        "stop_to_stand_s": None if exit_time is None else exit_time - args.stop_time,
        "handoff_boundary_max_abs_q_dq_tau_kp_kd": boundary,
        "end_root_xyz": d.qpos[:3].tolist(),
        "final_settled": int(settled),
        "final_foot_delta_pelvis": foot_delta.tolist(),
        "final_max_joint_speed": float(np.max(abs(state[31:62]))),
        "final_max_leg_speed": float(np.max(abs(state[50:62]))),
        "final_gyro_norm": float(np.linalg.norm(spatial[:3])),
        "step_time_us_p50_p99_max": np.percentile(compute_us, [50, 99, 100]).tolist(),
        "max_displacement_after_stop_m": stop_distance,
        "scope": "native HumanLike + native transitions + MuJoCo; no real transport/hardware",
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--library", type=Path, required=True)
    p.add_argument("--policy-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--case", default="all")
    p.add_argument("--max-seconds", type=float, default=20)
    p.add_argument("--stop-time", type=float, default=8)
    args = p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cases = {
        "zero": [0, 0, 0],
        "forward": [0.2, 0, 0],
        "backward": [-0.15, 0, 0],
        "left": [0, 0.1, 0],
        "right": [0, -0.1, 0],
        "turn": [0, 0, 0.25],
        "reverse": [0.2, 0, 0],
    }
    report = {}
    for name, v in cases.items():
        if args.case not in ("all", name):
            continue
        report[name] = run(args, name, v)
        print(name, json.dumps(report[name]), flush=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    raise SystemExit(
        0 if report and all(v["status"] == "PASS" for v in report.values()) else 1
    )
