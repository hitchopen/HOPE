#!/usr/bin/env python3
"""Validate the annotated serve bundle through native controllers and real ONNX.

Uses actual Runner action admission and command-delivery feedback, with no HAL,
DDS, gripper transport or physical ball. Tests command continuity separately from
floating-base stability, then holds the actual actor in WAIT for 15 seconds.
"""
import argparse
import ctypes
import json
from pathlib import Path

import mujoco
import numpy as np
import validate_spin001_serve_mujoco as sim
from runner_sim_profile import apply_hope_contact_profile


def run(library, policy_dir, csv, kernel, yaw, ready_seconds, cycles=1, stand_offset=0., handoff_frame=110, full_stand=False, trace_path=None, segmented=True, blend=.15, branch_pitch_rate=0.):
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    lib = ctypes.CDLL(str(library.resolve()))
    # The rally ABI supplies simulation time to native receipt/dwell clocks.
    # A fast offline loop must not silently qualify real-wall-clock timing.
    simulation_clock = getattr(lib, 'rally_time', None)
    if simulation_clock is not None:
        simulation_clock.argtypes = [ctypes.c_double]
        simulation_clock(0.)
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
    lib.probe_begin_impedance.argtypes = [ctypes.c_void_p, ptr, ptr, ctypes.c_double]
    lib.probe_apply.argtypes = [ctypes.c_void_p, ctypes.c_double, ptr]
    create = lib.receive_create_kernel if kernel else lib.receive_create
    create.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    create.restype = ctypes.c_void_p
    lib.receive_set_torso.argtypes = [ctypes.c_void_p, ptr]
    lib.receive_destroy.argtypes = [ctypes.c_void_p]
    lib.receive_rearm.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.receive_stand_command.argtypes = [ctypes.c_void_p, ptr]
    lib.receive_delivered_state.argtypes = [ctypes.c_void_p, ptr, ctypes.c_int, ptr]
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
    if segmented:
        lib.probe_select_segments.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p]
        assert lib.probe_select_segments(probe, int(not full_stand), str(policy_dir/'exported/policy.onnx').encode(), str(policy_dir/'params/deploy.yaml').encode()) == 0
    assert lib.probe_handoff_frame(probe, handoff_frame) == 0
    lib.probe_phase.argtypes = [ctypes.c_void_p]; lib.probe_phase.restype = ctypes.c_char_p
    assert lib.probe_return_seconds(probe, 2.5 if full_stand else 1.0) == 0
    lib.probe_kernel_completion.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.probe_kernel_completion(probe, int(full_stand))
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
    active_handoff = False
    stage = "stand"
    csv_start = None
    records = []
    torque_records = []
    entry_records = []

    def step():
        nonlocal max_tilt, min_height
        if csv_start is not None:
            records.append((float(data.time-csv_start), stage, command.copy(), data.qpos.copy(), data.qvel.copy()))
        assert np.isfinite(command).all()
        assert (command[3:] > 0).all(), 'support gains lost'
        state = np.concatenate([data.qpos[qa], data.qvel[va], data.xquat[pelvis], data.qvel[3:6], data.qpos[:3]])
        lib.receive_delivered_state(receive, command, int(active_handoff), state)
        requested_ratio = 0.
        for _ in range(10):
            torque = command[3] * (command[0] - data.qpos[qa]) + command[4] * (command[1] - data.qvel[va]) + command[2]
            requested_ratio = max(requested_ratio, float((abs(torque)/model.actuator_ctrlrange[ac,1]).max()))
            data.ctrl[ac] = np.clip(torque, model.actuator_ctrlrange[ac, 0], model.actuator_ctrlrange[ac, 1])
            mujoco.mj_step(model, data)
        if simulation_clock is not None:
            simulation_clock(float(data.time))
        if csv_start is not None: torque_records.append(requested_ratio)
        tilt = float(np.arccos(np.clip(data.xmat[pelvis].reshape(3,3)[2,2], -1, 1)))
        max_tilt = max(max_tilt, tilt)
        min_height = min(min_height, float(data.qpos[2]))
        assert tilt < .8 and data.qpos[2] >= .55, f'plant fell at {data.time:.3f}s'

    def serve_tick():
        nonlocal stage
        if csv_start is not None and abs(data.time-csv_start-(handoff_frame+1)*.01) < 1e-6:
            data.qvel[4] += branch_pitch_rate
            mujoco.mj_forward(model, data)
        phase = lib.probe_serve_with_gyro(probe, data.qpos[qa].copy(), data.qvel[va].copy(),
            data.xquat[pelvis].copy(), data.qvel[3:6].copy(), command)
        assert phase >= 0
        if csv_start is not None:
            elapsed = data.time-csv_start
            stage = 'preplay' if elapsed < -1e-6 else ('csv' if elapsed <= handoff_frame*.01+1e-6 else ('return_to_stand' if elapsed < handoff_frame*.01+1.02 else 'wait_stand_settle'))
            if segmented: stage = lib.probe_phase(probe).decode()
            if phase == 12: stage = 'complete'
        step()
        return phase

    try:
        assert lib.probe_control_action(probe, 1) == 1  # SERVER
        for _ in range(100):
            step()
        for cycle in range(cycles):
            active_handoff = False
            assert lib.probe_mode(probe) == (1 if cycle == 0 or full_stand else 3)
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
            csv_start = data.time + 1.01
            play_ticks = 0
            while phase != 12 and play_ticks < 1000:
                phase = serve_tick()
                play_ticks += 1
            assert phase == 12 and 111 < play_ticks < 1000, f"serve failed to settle: phase={phase}, ticks={play_ticks}, q_error={np.max(abs(data.qpos[qa]-stand[0])):.4f}"
            assert lib.probe_mode(probe) == (1 if full_stand else 3)
            if full_stand:
                stage = 'pd_stand'
                for _ in range(int(ready_seconds*100)): step()
                loop_results.append({'cycle':cycle, 'prepare_ticks':ticks, 'play_ticks':play_ticks})
                continue
            entry_records.append({'t':float(data.time-csv_start), 'goal_error':float(abs(data.qpos[qa]-command[0]).max()), 'joint_speed':float(abs(data.qvel[va]).max()), 'gyro':float(np.linalg.norm(data.qvel[3:6]))})
            lib.receive_rearm(receive, 1)
            target = stand.copy()
            for tick in range(int(ready_seconds*100)):
                stage = "policy_blend" if tick*.01 < blend else "receive_wait"
                active_handoff = tick*.01 < blend
                before = command.copy()
                if tick % 2 == 0:
                    state = np.concatenate([data.qpos[qa], data.qvel[va], data.xquat[pelvis], data.qvel[3:6], data.qpos[:3]])
                    lib.receive_set_torso(receive, data.xquat[torso].copy())
                    assert lib.receive_step(receive, tick//2, state, int(tick*.01 < blend), target) == 0
                if tick == 0:
                    assert lib.probe_begin_impedance(probe, command, target, blend) == 0
                command = target.copy()
                assert lib.probe_apply(probe, tick*.01, command) == 0
                if tick == 0:
                    boundaries.append({'cycle': cycle, 'edge': 'serve_to_ready',
                                       'q_dq_tau_kp_kd': np.max(abs(command-before), axis=1).tolist()})
                    np.testing.assert_array_equal(command, before)
                step()
            loop_results.append({'cycle':cycle, 'prepare_ticks':ticks, 'play_ticks':play_ticks})
    finally:
        if trace_path and records:
            np.savez_compressed(trace_path, t=np.array([r[0] for r in records]), stage=np.array([r[1] for r in records]), command=np.stack([r[2] for r in records]), qpos=np.stack([r[3] for r in records]), qvel=np.stack([r[4] for r in records]), qa=qa, va=va, stand=stand, requested_torque_ratio=np.array(torque_records))
        lib.probe_destroy(probe)
        lib.receive_destroy(receive)
    transition_mask = np.isin(np.array([r[1] for r in records]), ['CSV_SERVE_TO_POLICY', 'WAIT_POLICY_READY', 'policy_blend', 'complete'])
    transition_ratio = float(np.array(torque_records)[transition_mask].max(initial=0))
    if not full_stand:
        assert transition_ratio <= 1., f'new transition requests torque above actuator limit: {transition_ratio:.4f}'
        assert all(e['goal_error'] <= .10 and e['joint_speed'] <= .3 for e in entry_records), entry_records
    return {'status':'PASS', 'clock':'simulation' if simulation_clock else 'wall_clock_unqualified', 'branch_pitch_rate_rad_s':branch_pitch_rate, 'transition_torque_ratio':transition_ratio, 'kernel':kernel, 'yaw_deg':float(np.degrees(yaw)),
            'stand_lateral_offset_rad':stand_offset,
            'ready_seconds':ready_seconds, 'cycles':loop_results, 'boundaries':boundaries,
            'max_tilt_rad':max_tilt, 'min_root_height_m':min_height, 'seconds':float(data.time),
            'entry':entry_records, 'max_requested_torque_ratio':max(torque_records,default=0), 'scope':'Runner action admission, native Serve/receive, final command feedback; no HAL or physical ball'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--policy-dir', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--csv', type=Path, default=sim.MOTION_ROOT / 'a3p_op3_serve025_v23_branch110_model14600.csv')
    p.add_argument('--cycles', type=int, default=3)
    args = p.parse_args()
    if args.cycles < 1: p.error('cycles must be positive')
    # Read timing from the selected bundle; the native parser validates all rows.
    import csv
    with args.csv.open() as stream:
        marker = next(row for row in csv.reader(stream) if row and row[0] == '#HOPE_SERVE_SEGMENTS_V1')
    branch, blend = int(marker[1]), float(marker[2])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {}
    cases = [('normal', False, 0., args.cycles, 0.), ('kernel', True, 0., args.cycles, 0.),
             ('normal_yaw', False, 1., 1, 0.), ('pitch_plus', False, 0., 1, .1), ('pitch_minus', False, 0., 1, -.1)]
    for label, kernel, yaw, cycles, perturbation in cases:
        try:
            report[label] = run(args.library, args.policy_dir, args.csv, kernel, yaw, 15., cycles=cycles,
                stand_offset=.03, handoff_frame=branch, full_stand=kernel, segmented=True, blend=blend,
                branch_pitch_rate=perturbation, trace_path=args.output_dir/(label+'.npz'))
        except (AssertionError, RuntimeError) as error:
            report[label] = {'status':'FAIL', 'error':str(error)}
        (args.output_dir/'results.json').write_text(json.dumps(report,indent=2)+'\n')
        print(label, report[label], flush=True)
    raise SystemExit(0 if all(r['status']=='PASS' for r in report.values()) else 1)
