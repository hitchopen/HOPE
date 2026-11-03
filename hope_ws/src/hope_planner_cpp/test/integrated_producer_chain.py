#!/usr/bin/env python3
"""Installed raw MCT -> real relays -> one planner -> real C++ decoder.

Synthetic surveyed metadata; replay starts AFTER the camera driver. No referee
or complete-history publisher. Camera pacing runs in an independent process.
"""
import argparse
from collections import Counter
import gc
import json
import multiprocessing as mp
from pathlib import Path
import os
import subprocess
import signal
import sys
from queue import Empty
from qualification_accounting import SourceCadence, stage_parameters, resources as capture_resources, validate_source_epoch, write_receipt
import time

import numpy as np
from scipy.optimize import brentq
from independent_ball_flight import flight
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray
from hope_msgs.msg import BallObservationV2
from motion_capture_tracking_interfaces.msg import NamedPoseArrayV2, NamedPose, LatencyInfo
from std_msgs.msg import Float64MultiArray




def source(rate, stop, ready, stats, speed=5., spin='unknown', scenario='standard', scored_flights=2, base_yaw=0.):
    rclpy.init()
    node = rclpy.create_node('integrated_synthetic_camera')
    pub = node.create_publisher(NamedPoseArrayV2, '/optitrack/poses', qos_profile_sensor_data)
    before, after, bounce = flight(speed, spin, scenario)
    plane_x = .15
    if scenario == 'uncertain_contact': plane_x = float(before.sol(bounce)[0]-.002)
    plane_time = brentq(lambda t: after.sol(t)[0]-plane_x, bounce, .7)
    visible_end = min(.45, plane_time+.025)
    if scenario == 'net_collision':
        visible_end = brentq(lambda t: before.sol(t)[0]-1.37, 0., bounce)
    discovery_deadline = time.monotonic()+10.
    while pub.get_subscription_count() == 0 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    assert pub.get_subscription_count() > 0, 'installed relay did not subscribe'
    # This short-lived publisher has no per-frame reference cycles. Avoid avoidable
    # cyclic-GC pauses during capture; SourceCadence also accounts for other
    # scheduling overruns. All launched source rows remain in the receipt.
    gc.disable()
    start = time.monotonic()
    cadence = SourceCadence(rate, round(start * 1e9))
    clock_origin_ns = time.time_ns()
    ready.set()
    index = 0
    sent = []
    source_rows = []
    eligible_stamps = []
    while not stop.is_set():
        deadline = cadence.deadline_ns * 1e-9
        remaining = deadline-time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        # The synthetic state and acquisition stamp describe the same instant.
        # Message construction/publish delays are measured after acquisition.
        stamp_mono_ns = time.monotonic_ns()
        stamp = time.time_ns()
        cadence.acquired(stamp_mono_ns)
        elapsed = (stamp - clock_origin_ns) * 1e-9
        state_mono_ns = stamp_mono_ns
        phase = (elapsed-.8) % 1.0
        msg = NamedPoseArrayV2()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        # A physical frame fault at the end must revoke pending work.
        msg.header.frame_id = 'wrong_frame' if 1.0 < elapsed < 1.08 or elapsed > scored_flights+2.0 else 'world'
        latency = LatencyInfo()
        latency.source, latency.latency = 'NatNetClockUncertainty', 20
        msg.latencies.append(latency)
        base = NamedPose()
        base.name = 'UCB_P1'
        base.pose.position.x, base.pose.position.y = plane_x-.58, -.7
        base.pose.orientation.w = float(np.cos(np.deg2rad(base_yaw)/2))
        base.pose.orientation.z = float(np.sin(np.deg2rad(base_yaw)/2))
        msg.poses.append(base)
        if elapsed > .8 and phase < visible_end:
            state = before.sol(phase) if phase < bounce else after.sol(phase)
            if scenario == 'held': state = np.array([1.8, -.7, .4, 0., 0., 0., 0.])
            ball = NamedPose()
            ball.name = 'Ball'
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = map(float, state[:3])
            # Authority is explicitly configured on the real relay.
            if spin != 'unknown':
                ball.pose.orientation.y = float(np.sin(state[6]/2))
                ball.pose.orientation.w = float(np.cos(state[6]/2))
            msg.poses.append(ball)
        publish_wall_ns = time.time_ns()
        publish_mono_ns = time.monotonic_ns()
        pub.publish(msg)
        published_mono_ns = time.monotonic_ns()
        source_rows.append({
            'index': index, 'stamp_ns': stamp, 'state_elapsed_s': elapsed, 'intended_mono_ns': round(deadline * 1e9),
            'state_mono_ns': state_mono_ns, 'stamp_mono_ns': stamp_mono_ns,
            'publish_wall_ns': publish_wall_ns, 'publish_mono_ns': publish_mono_ns,
            'published_mono_ns': published_mono_ns, 'phase': phase,
            'frame': msg.header.frame_id,
            'poses': [{'name': pose.name,
                       'position': [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
                       'quaternion_wxyz': [pose.pose.orientation.w, pose.pose.orientation.x,
                                          pose.pose.orientation.y, pose.pose.orientation.z]}
                      for pose in msg.poses],
        })
        sent.append(stamp)
        # Frozen before this scored run: normal first-two-flight epochs after
        # 100 ms acquisition and before 130 ms lead, excluding the deliberate
        # physical frame fault and required-base outage. All game histories
        # remain in the denominator. Every launched source exposure is retained.
        if (.8 < elapsed < scored_flights+.4 and .1 < phase < plane_time-.13 and
                not (1.0 < elapsed < 1.08)):
            eligible_stamps.append(stamp)
        index += 1
    stats.put({'cadence': cadence.receipt(), 'clock_origin_ns': clock_origin_ns, 'rows': source_rows, 'sent': index, 'synthetic_source_cyclic_gc': 'disabled_during_pacing', 'source_intervals_ns': np.diff(sent).tolist(), 'bounce_s': bounce,
               'plane_s': plane_time, 'eligible_stamps': eligible_stamps,
               'command_epoch_contract': f'first_{scored_flights}_flights; phase>0.100; lead>0.130; physical_frame_fault_excluded; unknown_game_history_included'})
    node.destroy_node()
    rclpy.shutdown()


def run(args, rate, policy):
    # Fast returns have short eligible windows (as few as four source epochs
    # per flight). Freeze ten flights before scoring, retaining every exposure
    # and the original 100 ms warmup / 130 ms lead / 95% / 30 ms gates.
    scored_flights = 10 if args.speed in [10., 12.] else 2
    processes = []
    logs = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    def launch(exe, params=()):
        cmd = [str(exe), '--ros-args']
        for key, value in [*params, *stage_parameters(args, Path(exe))]:
            cmd += ['-p', f'{key}:={value}']
        log = (args.output.parent/(args.output.stem+'-'+Path(exe).name+'.log')).open('w')
        logs.append(log)
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, text=True)
        processes.append(p)
        return p
    world_sha, base_sha = 'a'*64, 'b'*64
    registration = 'SYNTHETIC_MCT_P1'
    base_params = [('extrinsic_calibrated', 'true'), ('world_frame_calibrated', 'true'),
                   ('calibration_sha256', base_sha), ('world_frame_sha256', world_sha),
                   ('source_stamp_mode', 'input_header'), ('policy_z_offset', '0.76')]
    node = camera = None
    decoded, wire, traces, statuses, observations, legacy, bases = [], [], [], [], [], [], []
    stop, ready, queue = mp.Event(), mp.Event(), mp.Queue()
    try:
        relay = launch(args.bringup/'optitrack_mct_relay', [('publish_tf', 'false'), ('input_history_depth', 8),
            ('ball_calibration_id', registration), ('ball_orientation_authoritative', str(args.spin != 'unknown').lower()), ('planner_poses_topic', '/poses/planner_full')])
        base = launch(args.bringup/'hope_base_pose_flat_relay', base_params)
        planner = launch(args.planner, [('prediction_backend', 'v2'), ('registration_id', registration),
            ('world_frame_id', int(world_sha[:13], 16)), ('plane_policy', policy),
            ('test.worker_delay_ms', args.worker_delay_ms),
            ('physics.drag_k', 0. if args.scenario in ['net_clear', 'net_collision'] else .12861635416914574)])
        launch(args.decoder)
        node = rclpy.create_node(f'integrated_chain_observer_{rate}_{policy.lower()}')
        decoded, wire, traces, statuses, observations, legacy, bases = [], [], [], [], [], [], []
        # Observation callbacks may pause briefly under Python GC. Preserve the
        # diagnostic evidence rather than dropping a 5-deep best-effort reader's
        # rows; this does not alter the planner's one-running/one-pending bound.
        trace_qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Float64MultiArray, '/a3/base_pose_flat', lambda m: bases.append(list(m.data)), qos_profile_sensor_data)
        node.create_subscription(Float64MultiArray, '/hope/test/integrated_runner/decoded',
                                 lambda m: decoded.append(list(m.data)), 128)
        node.create_subscription(Float64MultiArray, '/hope/shadow/integrated_planner/racket_target',
                                 lambda m: wire.append(list(m.data)), 128)
        def trace(m):
            for d in m.status:
                row = {kv.key: kv.value for kv in d.values}
                row['receipt_time_ns'] = time.time_ns()
                traces.append(row)
        node.create_subscription(DiagnosticArray, '/hope/shadow/integrated_planner/trace', trace, trace_qos)
        node.create_subscription(DiagnosticArray, '/hope/shadow/integrated_planner/status',
            lambda m: statuses.extend([{**{kv.key: kv.value for kv in d.values}, 'reason': d.message} for d in m.status]), 10)
        node.create_subscription(BallObservationV2, '/hope/ball/observation_v2', observations.append, qos_profile_sensor_data)
        node.create_subscription(PoseArray, '/poses/planner_full', legacy.append, qos_profile_sensor_data)
        camera = mp.Process(target=source, args=(rate, stop, ready, queue, args.speed, args.spin, args.scenario, scored_flights, args.base_yaw))
        until = time.monotonic()+1.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        for p in processes:
            assert p.poll() is None, p.args
        camera.start()
        deadline = time.monotonic()+15.
        while not ready.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert ready.is_set(), 'source failed to initialize'
        start = time.monotonic()
        base_stopped = False
        while time.monotonic()-start < scored_flights+2.7:
            rclpy.spin_once(node, timeout_sec=.002)
            # Missing mandatory pose producer must become observable after TTL.
            if time.monotonic()-start > scored_flights+.5 and not base_stopped:
                base.send_signal(signal.SIGINT)
                base.wait(timeout=5)
                base_stopped = True
        stop.set()
        source_stats = queue.get(timeout=5)
        validate_source_epoch(source_stats)
        assert source_stats['cadence']['valid'], ('source cadence', source_stats['cadence'])
        camera.join(timeout=5)
        assert not camera.is_alive()
        if args.test_fail_after_collection:
            raise AssertionError('injected failure after source collection')
        valid = [w for w in wire if w[1] == 1.]
        if args.worker_delay_ms >= 30:
            assert not valid and not decoded and not traces, 'expired solve published a command'
            assert observations and source_stats['eligible_stamps'], 'vacuous delayed-worker fixture'
            superseded = max(int(s.get('superseded', 0)) for s in statuses)
            assert superseded > 0, 'delay did not exercise latest-pending backpressure'
            return {'status': 'PASS', 'case': 'delayed_worker_expiry', 'worker_delay_ms': args.worker_delay_ms,
                    'valid_commands': 0, 'source_eligible_epochs': len(source_stats['eligible_stamps']),
                    'superseded_work': superseded, 'queue_bound': {'running': 1, 'pending': 1},
                    'reasons': dict(Counter(s['reason'] for s in statuses)), 'source': source_stats}
        physical = [s for s in statuses if s.get('physical_available') == 'true']
        if args.scenario != 'held': assert physical, ('no physical forecast', Counter(s['reason'] for s in statuses))
        if args.scenario in ['held', 'net_collision', 'uncertain_contact']:
            assert not valid and not decoded, ('unsupported flight emitted command', len(valid))
            reasons = Counter(s['reason'] for s in statuses)
            if args.scenario == 'held':
                assert not physical and observations, 'held ball acquired a physical shot'
            elif args.scenario == 'net_collision':
                assert reasons['PHYSICAL_TERMINATION_BEFORE_CONTACT'] > 0, reasons
            else:
                assert reasons['CONTACT_ORDER_UNCERTAIN']+reasons['BOUNCE_ORDER_UNCERTAIN'] > 0, reasons
            return {'status': 'PASS', 'case': args.scenario, 'rate_hz': rate,
                    'valid_commands': 0, 'reasons': dict(reasons), 'source': source_stats,
                    'physical_rows': physical}
        if args.scenario == 'net_clear':
            assert any(s.get('return_eligible') == 'true' for s in physical), 'flyover never became physically eligible'
            assert not any(s.get('return_reason') == 'PHYSICAL_TERMINATION_BEFORE_CONTACT'
                           for s in physical if int(s.get('observed_contacts', 0)) == 0), 'false net termination'
            assert valid or any(s['reason'] == 'IMPACT_OR_VELOCITY_CONTRACT' for s in statuses)
            return {'status': 'PASS', 'case': 'clearing_net_physics_actor_separate', 'rate_hz': rate,
                    'valid_commands': len(valid), 'decoded_callbacks': len(decoded), 'physical_rows': physical,
                    'reasons': dict(Counter(s['reason'] for s in statuses)), 'source': source_stats}
        if args.speed == 20.:
            assert any(int(s.get('accepted', 0)) > 10 for s in statuses), 'fast flight never acquired'
            assert any(int(s.get('observed_contacts', 0)) == 1 for s in physical), 'fast bounce not observed'
            assert not any(int(s.get('observed_contacts', 0)) > 1 for s in physical), 'duplicate bounce'
            assert all(float(t['source_age_ms']) <= 30 for t in traces)
            return {'status': 'PASS', 'case': 'fast_physical_tracking_execution_separate',
                    'speed_mps': args.speed, 'spin': args.spin, 'rate_hz': rate,
                    'valid_commands': len(valid), 'decoded_callbacks': len(decoded),
                    'physical_rows': physical, 'reasons': dict(Counter(s['reason'] for s in statuses)),
                    'source': source_stats, 'wire': wire, 'decoded': decoded, 'traces': traces}
        if abs(args.base_yaw) > 40:
            assert not valid and not decoded and physical
            assert any(s['reason'] == 'target_turn_exceeds_limit' for s in statuses)
            assert any(s.get('outgoing_return_valid') == 'true' for s in statuses), 'vacuous yaw rejection'
            return {'status':'PASS','case':'rotated_base_turn_rejected','base_yaw_deg':args.base_yaw,
                    'valid_commands':0,'reasons':dict(Counter(s['reason'] for s in statuses)), 'source':source_stats}
        assert valid and decoded and traces, (Counter(s['reason'] for s in statuses),
            bases[:1], bases[-1:], len(observations), len(legacy))
        assert all(t['game_history'] != 'COMPLETE' for t in traces)
        assert any(t['bounce_observed'] == 'false' for t in traces), 'no pre-bounce command'
        assert all(float(t['source_age_ms']) <= 30 for t in traces)
        assert any(w[1] == 0 for w in wire), 'no pending revocation'
        assert any(s['reason'] == 'BASE_UNAVAILABLE' for s in statuses)
        assert any(s['reason'] == 'SOURCE_UNQUALIFIED' for s in statuses)
        if args.spin == 'unknown':
            assert all(not o.orientation_valid and not o.omega_valid for o in observations)
        else:
            assert any(o.orientation_valid and o.orientation_authoritative for o in observations)
        assert observations and legacy
        assert all(o.calibration_id == registration for o in observations)
        wire_by_seq = {int(w[14]): w for w in valid}
        matched = 0
        for d in decoded:
            if int(d[0]) in wire_by_seq:
                w = wire_by_seq[int(d[0])]
                assert np.allclose(d[3:9], w[3:9], rtol=0, atol=1e-12)
                assert int(d[1]) == int(w[15])
                matched += 1
        assert matched > 0
        expected = set(source_stats['eligible_stamps'])
        published = {int(t['source_time_ns']) for t in traces}
        covered = expected & published
        coverage = len(covered)/len(expected) if expected else 0.
        assert expected and coverage >= .95, ('command coverage', len(covered), len(expected), coverage)
        graph = {name: types for name, types in node.get_topic_names_and_types()}
        assert '/ball/flight_packet' not in graph
        assert node.count_publishers('/racket/command_flat') == 0
        assert node.get_node_names().count('hope_planner') == 1
        latencies = [(int(t['receipt_time_ns'])-int(t['source_time_ns']))/1e6 for t in traces]
        by_seq = {int(t['command_sequence']): t for t in traces}
        decode_latency = [((int(d[13])*10**9+int(d[14]))-int(by_seq[int(d[0])]['source_time_ns']))/1e6
                          for d in decoded if int(d[0]) in by_seq and d[10] == 0]
        assert decode_latency and np.percentile(decode_latency, 95) <= 30., ('decoder latency', decode_latency)
        publication_age = [float(t['source_age_ms']) for t in traces]
        resources = {}
        for line in Path(f'/proc/{planner.pid}/status').read_text().splitlines():
            if line.startswith(('VmRSS:', 'VmHWM:')):
                key, value = line.split(':', 1)
                resources[key+'_kib'] = int(value.split()[0])
        cpu_fields = Path(f'/proc/{planner.pid}/stat').read_text().rsplit(')', 1)[1].split()
        resources['node_cpu_s'] = (int(cpu_fields[11])+int(cpu_fields[12]))/os.sysconf('SC_CLK_TCK')
        def distribution(values):
            return dict(zip(['p50', 'p95', 'p99', 'max'], map(float, np.percentile(values, [50, 95, 99, 100]))))
        return {'status': 'PASS', 'rate_hz': rate, 'plane_policy': policy, 'speed_mps': args.speed, 'spin': args.spin, 'scenario': args.scenario,
                'valid_commands': len(valid), 'decoded_callbacks': len(decoded), 'matched_commands': matched,
                'before_bounce_commands': sum(t['bounce_observed'] == 'false' for t in traces),
                'trace_receipt_ms': distribution(latencies),
                'actual_decoder_ms': distribution(decode_latency),
                'source_to_publication_ms': distribution(publication_age), 'resources': resources,
                'relay_ball_observations': len(observations), 'legacy_full_rate_frames': len(legacy),
                'command_coverage_gate': 'PASS', 'command_coverage': coverage, 'observer_trace_depth': 128, 'scored_flights': scored_flights,
                'command_denominator': len(expected), 'covered_command_epochs': len(covered),
                'reasons': dict(Counter(s['reason'] for s in statuses)), 'source': source_stats,
                'traces': traces, 'wire': wire, 'decoded': decoded, 'graph': graph,
                'camera_driver_included': False, 'synthetic_registration': True,
                'context_publisher': False, 'base_yaw_deg': args.base_yaw}
    finally:
        original_failure = sys.exc_info()[1]
        failure = sys.exc_info()[1]
        failure = None if failure is None else f'{type(failure).__name__}: {failure}'
        resource_receipt = capture_resources(processes)
        stop.set()
        if camera is not None and camera.pid:
            if not locals().get('source_stats'):
                try:
                    source_stats = queue.get(timeout=5)
                except Empty:
                    source_stats = {}
            camera.join(timeout=5)
            if camera.is_alive():
                camera.terminate()
                camera.join(timeout=5)
                failure = (failure or '') + ' SOURCE_SHUTDOWN_TIMEOUT'
        if node is not None:
            node.destroy_node()
        for p in reversed(processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait(timeout=5)
                    failure = (failure or '') + ' CHILD_SHUTDOWN_TIMEOUT:' + Path(p.args[0]).name
            resource_receipt[Path(p.args[0]).name]['final_exit'] = p.returncode
            if p.returncode != 0:
                failure = (failure or '') + f' CHILD_EXIT:{Path(p.args[0]).name}:{p.returncode}'
        for log in logs:
            log.close()
        write_receipt(args, {
            'rate_hz': rate, 'speed_mps': args.speed, 'spin': args.spin,
            'statuses': statuses, 'traces': traces, 'wire': wire, 'decoded': decoded,
            'source': locals().get('source_stats', {}),
            'observer_source_ns': [int(o.source_time_ns) for o in observations],
        }, resource_receipt, failure)
        if failure and original_failure is None:
            raise RuntimeError(failure)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--planner', type=Path, required=True)
    p.add_argument('--decoder', type=Path, required=True)
    p.add_argument('--bringup', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--rate', type=int, choices=[150, 300], default=300)
    p.add_argument('--plane-policy', choices=['FOLLOW_POSE', 'ON_REFRESH'], default='FOLLOW_POSE')
    p.add_argument('--speed', type=float, choices=[5., 10., 12., 20.], default=5.)
    p.add_argument('--scenario', choices=['standard', 'net_clear', 'net_collision', 'uncertain_contact', 'held'], default='standard')
    p.add_argument('--spin', choices=['unknown', 'zero', 'known'], default='unknown')
    p.add_argument('--base-yaw', type=float, default=0.)
    p.add_argument('--worker-delay-ms', type=int, default=0)
    p.add_argument('--stage-accounting', action='store_true')
    p.add_argument('--test-fail-after-collection', action='store_true')
    args = p.parse_args()
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    try:
        result = run(args, args.rate, args.plane_policy)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({k: v for k, v in result.items() if k not in ['source', 'traces', 'wire', 'decoded', 'graph']}))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    mp.set_start_method('spawn')
    main()
