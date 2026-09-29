#!/usr/bin/env python3
"""Run repeated Start-to-Serve -> Serve -> Ready cycles through native controllers.

Uses actual Runner action admission and command-delivery feedback, with no HAL,
DDS, gripper transport or physical ball. Tests command continuity separately from
floating-base stability. A short Ready dwell also interrupts the receive blend.
"""
import argparse
import ctypes
import json
from pathlib import Path

import mujoco
import numpy as np
import validate_spin001_serve_mujoco as sim
from runner_sim_profile import apply_hope_contact_profile


def run(library, policy_dir, csv, kernel, yaw, ready_seconds, cycles=5, stand_offset=0.):
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    lib = ctypes.CDLL(str(library.resolve()))
    lib.probe_create.argtypes = [ctypes.c_char_p, ptr]
    lib.probe_create.restype = ctypes.c_void_p
    lib.probe_handoff_frame.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.probe_return_seconds.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.probe_serve_with_gyro.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr, ptr]
    lib.probe_destroy.argtypes = [ctypes.c_void_p]
    lib.probe_control_action.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.probe_mode.argtypes = [ctypes.c_void_p]
    lib.probe_serve.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr]
    lib.probe_serve_with_gyro.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr, ptr]
    lib.probe_begin.argtypes = [ctypes.c_void_p, ptr, ptr, ctypes.c_double]
    lib.probe_apply.argtypes = [ctypes.c_void_p, ctypes.c_double, ptr]
    create = lib.receive_create_kernel if kernel else lib.receive_create
    create.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    create.restype = ctypes.c_void_p
    lib.receive_set_torso.argtypes = [ctypes.c_void_p, ptr]
    lib.receive_destroy.argtypes = [ctypes.c_void_p]
    lib.receive_rearm.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.receive_stand_command.argtypes = [ctypes.c_void_p, ptr]
    lib.receive_delivered.argtypes = [ctypes.c_void_p, ptr, ctypes.c_int]
    lib.receive_step.argtypes = [ctypes.c_void_p, ctypes.c_uint, ptr, ctypes.c_int, ptr]
    receive = create(str(policy_dir / 'exported/policy.onnx').encode(),
                     str(policy_dir / 'params/deploy.yaml').encode())
    assert receive
    lib.receive_set_stand_offset.argtypes = [ctypes.c_void_p, ctypes.c_double]
    assert lib.receive_set_stand_offset(receive, stand_offset) == 0
    stand = np.empty((5, 31))
    lib.receive_stand_command(receive, stand)
    probe = lib.probe_create(str(csv.resolve()).encode(), stand)
    assert probe
    assert lib.probe_handoff_frame(probe, 110) == 0
    assert lib.probe_return_seconds(probe, 1.0) == 0
    model, bindings = sim._compile_model(sim.DEFAULT_MODEL)
    apply_hope_contact_profile(model)
    qa, va, ac = np.array(bindings).T
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('stand').id)
    data.qpos[:2] = [-.45, -.7625]
    data.qpos[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
    data.qpos[qa] = stand[0]
    ball = model.jnt_qposadr[model.joint('gate3_ball_free_joint').id]
    data.qpos[ball:ball+3] = [100, 0, -10]
    mujoco.mj_forward(model, data)
    pelvis = model.body('pelvis_link').id
    torso = model.body('torso_Link').id
    command = stand.copy()
    max_tilt, min_height = 0., float('inf')
    boundaries, loop_results = [], []

    def step():
        nonlocal max_tilt, min_height
        assert np.isfinite(command).all()
        assert (command[3:] > 0).all(), 'support gains lost'
        lib.receive_delivered(receive, command, 1)
        for _ in range(10):
            torque = command[3] * (command[0] - data.qpos[qa]) + command[4] * (command[1] - data.qvel[va]) + command[2]
            data.ctrl[ac] = np.clip(torque, model.actuator_ctrlrange[ac, 0], model.actuator_ctrlrange[ac, 1])
            mujoco.mj_step(model, data)
        tilt = float(np.arccos(np.clip(data.xmat[pelvis].reshape(3,3)[2,2], -1, 1)))
        max_tilt = max(max_tilt, tilt)
        min_height = min(min_height, float(data.qpos[2]))
        assert tilt < .8 and data.qpos[2] >= .55, f'plant fell at {data.time:.3f}s'

    def serve_tick():
        phase = lib.probe_serve_with_gyro(probe, data.qpos[qa].copy(), data.qvel[va].copy(),
            data.xquat[pelvis].copy(), data.qvel[3:6].copy(), command)
        assert phase >= 0
        step()
        return phase

    try:
        assert lib.probe_control_action(probe, 1) == 1  # SERVER
        for _ in range(100):
            step()
        for cycle in range(cycles):
            assert lib.probe_mode(probe) == (1 if cycle == 0 else 3)
            assert lib.probe_control_action(probe, 7) == 1  # PREPARE_SERVE
            assert lib.probe_mode(probe) == 5
            before = command.copy()
            phase = serve_tick()
            boundaries.append({'cycle': cycle, 'edge': 'ready_to_prepare',
                               'q_dq_tau_kp_kd': np.max(abs(command-before), axis=1).tolist()})
            np.testing.assert_array_equal(command, before)
            ticks = 1
            # Bounded entry transition followed by the five-second arm raise.
            while phase != 5 and ticks < 1200:  # WAIT_READY_TO_SERVE
                phase = serve_tick()
                ticks += 1
            assert phase == 5, ('repeat prepare did not reach loading pose: '
                f'phase={phase} q_error={np.max(abs(data.qpos[qa]-stand[0])):.4f} '
                f'dq={np.max(abs(data.qvel[va])):.4f} gyro={np.linalg.norm(data.qvel[3:6]):.4f}')
            for _ in range(30):  # operator loads ball while stationary
                assert serve_tick() == 5
            assert lib.probe_control_action(probe, 9) == 3  # ACCEPTED_PENDING
            play_ticks = 0
            while phase != 12 and play_ticks < 1000:
                phase = serve_tick()
                play_ticks += 1
            assert phase == 12 and 111 < play_ticks < 1000, f"serve failed to settle: phase={phase}, ticks={play_ticks}, q_error={np.max(abs(data.qpos[qa]-stand[0])):.4f}"
            assert lib.probe_mode(probe) == 3  # automatic READY / MOTION
            lib.receive_rearm(receive, 1)
            target = stand.copy()
            for tick in range(int(ready_seconds*100)):
                before = command.copy()
                if tick % 2 == 0:
                    state = np.concatenate([data.qpos[qa], data.qvel[va], data.xquat[pelvis], data.qvel[3:6], data.qpos[:3]])
                    lib.receive_set_torso(receive, data.xquat[torso].copy())
                    assert lib.receive_step(receive, tick//2, state, int(tick < 50), target) == 0
                if tick == 0:
                    assert lib.probe_begin(probe, command, target, .5) == 0
                command = target.copy()
                assert lib.probe_apply(probe, tick*.01, command) == 0
                if tick == 0:
                    boundaries.append({'cycle': cycle, 'edge': 'serve_to_ready',
                                       'q_dq_tau_kp_kd': np.max(abs(command-before), axis=1).tolist()})
                    np.testing.assert_array_equal(command, before)
                step()
            loop_results.append({'cycle':cycle, 'prepare_ticks':ticks, 'play_ticks':play_ticks})
    finally:
        lib.probe_destroy(probe)
        lib.receive_destroy(receive)
    return {'status':'PASS', 'kernel':kernel, 'yaw_deg':float(np.degrees(yaw)),
            'stand_lateral_offset_rad':stand_offset,
            'ready_seconds':ready_seconds, 'cycles':loop_results, 'boundaries':boundaries,
            'max_tilt_rad':max_tilt, 'min_root_height_m':min_height, 'seconds':float(data.time),
            'scope':'Runner action admission, native Serve/receive, final command feedback; no HAL or physical ball'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--policy-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stand-offset', type=float, default=.03,
                   help='lateral Stand offset matching the selected CSV; v12 uses 0.03 rad')
    p.add_argument('--cycles', type=int, default=5,
                   help='number of complete serve/Ready cycles per case')
    p.add_argument('--ready-seconds', type=float, nargs='+', default=[0, .1, 3],
                   help='Ready dwell times; values below 0.5 interrupt the receive blend')
    p.add_argument('--csv', type=Path, default=sim.MOTION_ROOT / 'a3p_op3_serve025_photo_right30_advance20_v12.csv')
    args = p.parse_args()
    if args.cycles < 1 or any(not np.isfinite(t) or t < 0 for t in args.ready_seconds):
        p.error('cycles must be positive and Ready dwell times must be finite and nonnegative')
    report = {}
    failed = False
    for kernel, degrees in [(True, 0), (True, 90), (True, 169.2), (False, 0)]:
        for dwell in args.ready_seconds:
            key = f'{"kernel" if kernel else "normal"}_{degrees}_wait{dwell}'
            try:
                report[key] = run(args.library, args.policy_dir, args.csv, kernel, np.radians(degrees), dwell, args.cycles, args.stand_offset)
            except (AssertionError, RuntimeError) as error:
                failed = True
                report[key] = {'status': 'FAIL', 'error': str(error), 'kernel': kernel, 'yaw_deg': degrees, 'ready_seconds': dwell}
            args.output.write_text(json.dumps(report, indent=2)+'\n')
            print(key, report[key]['status'], report[key].get('max_tilt_rad', report[key].get('error')), flush=True)
    raise SystemExit(1 if failed else 0)
