#!/usr/bin/env python3
"""Exercise native Serve/command-transition code in MuJoCo, without HAL.

Build runner_transition_probe.cpp into a shared library and pass --library.
The report distinguishes command continuity from plant stability.
HumanLike is exercised by validate_humanlike_mujoco.py. Optional receive
checks cover planner waiting without an incoming ball, not a rally.
"""

import argparse
import ctypes
import json
from pathlib import Path

import mujoco
import numpy as np
import validate_spin001_serve_mujoco as sim
from runner_sim_profile import apply_hope_contact_profile


def run(
    library,
    csv,
    duration,
    cancel_at,
    complete,
    receive_dir=None,
    from_stand=False,
    play=False,
    kernel=False,
    yaw=0.,
    lower_model=None,
    wait_seconds=3.,
    delivery_feedback=True,
    pre_receive_seconds=0.,
):
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    lib = ctypes.CDLL(str(library.resolve()))
    lib.probe_create.argtypes = [ctypes.c_char_p, ptr]
    lib.probe_create.restype = ctypes.c_void_p
    lib.probe_destroy.argtypes = [ctypes.c_void_p]
    lib.probe_action.argtypes = [ctypes.c_void_p, ctypes.c_int, ptr]
    lib.probe_serve.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr]
    lib.probe_begin.argtypes = [ctypes.c_void_p, ptr, ptr, ctypes.c_double]
    lib.probe_apply.argtypes = [ctypes.c_void_p, ctypes.c_double, ptr]
    receiver = None
    lower = None
    if receive_dir:
        create = lib.receive_create_kernel if kernel else lib.receive_create
        create.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        create.restype = ctypes.c_void_p
        lib.receive_destroy.argtypes = [ctypes.c_void_p]
        lib.receive_rearm.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.receive_observation_size.argtypes = [ctypes.c_void_p]
        lib.receive_stand_command.argtypes = [ctypes.c_void_p, ptr]
        lib.receive_delivered.argtypes = [ctypes.c_void_p, ptr, ctypes.c_int]
        lib.receive_step.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint,
            ptr,
            ctypes.c_int,
            ptr,
        ]
        receiver = create(
            str(receive_dir / "exported/policy.onnx").encode(),
            str(receive_dir / "params/deploy.yaml").encode(),
        )
        if not receiver:
            raise RuntimeError("receive policy load failed")
    if lower_model:
        lib.lower_create.argtypes = [ctypes.c_char_p]
        lib.lower_create.restype = ctypes.c_void_p
        lib.lower_destroy.argtypes = [ctypes.c_void_p]
        lib.lower_step.argtypes = [ctypes.c_void_p, ptr, ptr]
        lower = lib.lower_create(str(lower_model).encode())
        if not lower:
            raise RuntimeError("legacy hybrid lower load failed")
    model, bindings = sim._compile_model(sim.DEFAULT_MODEL)
    apply_hope_contact_profile(model)
    qa, va, ac = np.array(bindings).T
    data = mujoco.MjData(model)
    q, kp, kd = sim._official_stand_vectors()
    stand = np.array([q, q * 0, q * 0, kp, kd])
    native_stand = stand.copy()
    if receiver:
        # Compare against the actual Runner stand target, not the Python
        # fixture whose decimal precision differs from exported model defaults.
        lib.receive_stand_command(receiver, native_stand)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    data.qpos[:2] = [-0.45, -0.7625]
    data.qpos[3:7] = [np.cos(yaw/2), 0., 0., np.sin(yaw/2)]
    data.qpos[qa] = q
    ball = model.jnt_qposadr[model.joint("gate3_ball_free_joint").id]
    data.qpos[ball : ball + 3] = [100, 0, -10]
    mujoco.mj_forward(model, data)
    pelvis = model.body("pelvis_link").id
    probe = lib.probe_create(str(csv.resolve()).encode(), stand)
    if not probe:
        raise RuntimeError("native controller could not load CSV")
    cmd = stand.copy()
    handoff = None
    boundary = None
    samples = []
    exit_tick = 100 if from_stand else (601 + 468 if complete else cancel_at)
    target = stand.copy()
    kernel_wait_error = 0.
    actor_observation_size = 0
    def advance_plant(command):
        for _ in range(10):
            torque = command[3] * (command[0] - data.qpos[qa]) + command[4] * (command[1] - data.qvel[va]) + command[2]
            data.ctrl[ac] = np.clip(torque, model.actuator_ctrlrange[ac, 0], model.actuator_ctrlrange[ac, 1])
            mujoco.mj_step(model, data)
    try:
        # Reuse the same policy instance across waiting -> Stand -> Serve.
        # Mirrors the failing field session instead of always using a cold actor.
        if receiver and pre_receive_seconds > 0:
            lib.receive_rearm(receiver, 0)
            warm_target = stand.copy()
            for warm_tick in range(int(pre_receive_seconds * 100)):
                state = np.concatenate([data.qpos[qa], data.qvel[va], data.xquat[pelvis], data.qvel[3:6], data.qpos[:3]])
                if warm_tick % 2 == 0:
                    assert lib.receive_step(receiver, warm_tick // 2, state, int(warm_tick < 50), warm_target) == 0
                if warm_tick == 0:
                    assert lib.probe_begin(probe, cmd, warm_target, duration) == 0
                cmd = warm_target.copy()
                assert lib.probe_apply(probe, warm_tick * .01, cmd) == 0
                lib.receive_delivered(receiver, cmd, 1)
                advance_plant(cmd)
            assert lib.probe_begin(probe, cmd, stand, duration) == 0
            for stand_tick in range(200):
                cmd = stand.copy()
                assert lib.probe_apply(probe, stand_tick * .01, cmd) == 0
                lib.receive_delivered(receiver, cmd, 1)
                advance_plant(cmd)
        for tick in range(exit_tick + int(wait_seconds * 100) + 1):
            before = cmd.copy()
            if tick == 100 and not from_stand:
                assert lib.probe_action(probe, 0, before) == 0
            if tick == 601 and (complete or play):
                assert lib.probe_action(probe, 1, before) == 0
            if 100 <= tick < exit_tick and not from_stand:
                assert (
                    lib.probe_serve(
                        probe,
                        data.qpos[qa].copy(),
                        data.qvel[va].copy(),
                        data.xquat[pelvis].copy(),
                        cmd,
                    )
                    >= 0
                )
            elif tick >= exit_tick:
                if receiver and tick == exit_tick:
                    lib.receive_rearm(receiver, int(complete))
                state = np.concatenate([data.qpos[qa], data.qvel[va], data.xquat[pelvis], data.qvel[3:6], data.qpos[:3]])
                if receiver and (tick - exit_tick) % 2 == 0:
                    state = np.concatenate(
                        [
                            data.qpos[qa],
                            data.qvel[va],
                            data.xquat[pelvis],
                            data.qvel[3:6],
                            data.qpos[:3],
                        ]
                    )
                    assert (
                        lib.receive_step(
                            receiver,
                            (tick - exit_tick) // 2,
                            state,
                            int((tick - exit_tick) * 0.01 < duration),
                            target,
                        )
                        == 0
                    )
                composed = target.copy()
                if lower:
                    assert lib.lower_step(lower, state, composed) == 0
                if handoff is None:
                    if not complete and not from_stand:
                        assert lib.probe_action(probe, 2, before) == 0
                        held = np.empty((5, 31))
                        assert (
                            lib.probe_serve(
                                probe,
                                data.qpos[qa].copy(),
                                data.qvel[va].copy(),
                                data.xquat[pelvis].copy(),
                                held,
                            )
                            >= 0
                        )
                        np.testing.assert_array_equal(held, before)
                    assert lib.probe_begin(probe, before, composed, duration) == 0
                    handoff = tick
                cmd = composed.copy()
                assert lib.probe_apply(probe, (tick - handoff) * 0.01, cmd) == 0
                if tick == handoff:
                    boundary = np.max(abs(cmd - before), axis=1).tolist()
            if receiver and delivery_feedback:
                lib.receive_delivered(receiver, cmd, 1)
            if receiver:
                actor_observation_size = max(actor_observation_size, lib.receive_observation_size(receiver))
                if kernel and tick >= exit_tick + int(duration * 100):
                    kernel_wait_error = max(kernel_wait_error, float(np.max(abs(cmd - native_stand))))
            advance_plant(cmd)
            tilt = float(
                np.arccos(np.clip(data.xmat[pelvis].reshape(3, 3)[2, 2], -1, 1))
            )
            samples.append(
                [
                    float(data.time),
                    tilt,
                    float(data.qpos[2]),
                    *np.max(abs(cmd - before), axis=1),
                ]
            )
            if tilt > 0.8 or data.qpos[2] < 0.55:
                break
    finally:
        lib.probe_destroy(probe)
        if receiver:
            lib.receive_destroy(receiver)
        if lower:
            lib.lower_destroy(lower)
    values = np.asarray(samples)
    fell = values[-1, 1] > 0.8 or values[-1, 2] < 0.55
    return {
        "status": "FAIL" if fell else "PASS",
        "scope": "native Serve + transition"
        + (" + PpPolicy planner waiting (no ball)" if receiver else " to stand"),
        "receive_policy": str(receive_dir) if receiver else None,
        "kernel_imu_actor_test": kernel,
        "delivery_feedback": delivery_feedback,
        "pre_receive_seconds": pre_receive_seconds,
        "actor_observation_size": actor_observation_size,
        "kernel_post_blend_max_difference_from_stand": kernel_wait_error,
        "initial_yaw_rad": yaw,
        "legacy_hybrid_lower": str(lower_model) if lower else None,
        "stand_transition_s": duration,
        "seconds": float(data.time),
        "max_tilt_rad": float(values[:, 1].max()),
        "min_root_height_m": float(values[:, 2].min()),
        "handoff_boundary_max_abs_q_dq_tau_kp_kd": boundary,
        "whole_run_max_tick_delta_q_dq_tau_kp_kd": values[:, 3:].max(axis=0).tolist(),
        "model": str(sim.DEFAULT_MODEL),
        "contact_profile": "normal HOPE: ankle hulls, vendor sole spheres disabled",
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--library", type=Path, required=True)
    p.add_argument(
        "--csv",
        type=Path,
        default=sim.MOTION_ROOT
        / "a3p_op3_serve025_smooth_center_v14.csv",
    )
    p.add_argument("--receive-policy-dir", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    report = {
        name: run(args.library, args.csv, duration, cancel, complete)
        for name, duration, cancel, complete in [
            ("cancel_during_raise_0p5s", 0.5, 300, False),
            ("cancel_ready_0p5s", 0.5, 701, False),
            ("complete_to_stand_0p5s", 0.5, 0, True),
        ]
    }
    for name, frame in [
        ("cancel_before_hit", 60),
        ("cancel_during_hit", 83),
        ("cancel_follow_through", 200),
    ]:
        report[name] = run(args.library, args.csv, 0.5, 601 + frame, False, play=True)
    if args.receive_policy_dir:
        report["stand_to_receive_wait"] = run(
            args.library, args.csv, 0.5, 0, False, args.receive_policy_dir, True
        )
        report["serve_to_receive_wait"] = run(
            args.library, args.csv, 0.5, 0, True, args.receive_policy_dir
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if all(x["status"] == "PASS" for x in report.values()) else 1)
