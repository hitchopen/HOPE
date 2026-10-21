#!/usr/bin/env python3
"""Installed raw MCT -> real relays -> one planner -> real C++ decoder.

Synthetic surveyed metadata; replay starts AFTER the camera driver. No referee
or complete-history publisher. Camera pacing runs in an independent process.
"""
import argparse
from collections import Counter
import json
import multiprocessing as mp
from pathlib import Path
import os
import subprocess
import time

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq
import rclpy
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray
from hope_msgs.msg import BallObservationV2
from motion_capture_tracking_interfaces.msg import NamedPoseArrayV2, NamedPose, LatencyInfo
from std_msgs.msg import Float64MultiArray


def flight():
    """Independent DOP853 plant; bounce impulse is explicit position-only."""
    def rhs(_, s):
        v = s[3:]
        return np.r_[v, np.array([0., 0., -9.81]) - .12861635416914574*np.linalg.norm(v)*v]
    def contact(_, s):
        return s[2] - .02
    contact.direction = -1
    contact.terminal = True
    before = solve_ivp(rhs, [0, 1], [1.8, -.7, .4, -5., 0., -.5],
                       events=contact, dense_output=True, rtol=1e-11, atol=1e-13)
    bounce = before.t[-1]
    post = before.y[:, -1].copy()
    speed = np.linalg.norm(post[3:5])
    impulse = min(.38744443855065164*speed,
                  .16706204822993975*(1+.9313775047019153)*abs(post[5]))
    post[3:5] *= 1-impulse/speed
    post[5] *= -.9313775047019153
    after = solve_ivp(rhs, [bounce, .7], post, dense_output=True, rtol=1e-11, atol=1e-13)
    return before, after, bounce


def source(rate, stop, ready, stats):
    rclpy.init()
    node = rclpy.create_node('integrated_synthetic_camera')
    pub = node.create_publisher(NamedPoseArrayV2, '/optitrack/poses', qos_profile_sensor_data)
    before, after, bounce = flight()
    plane_time = brentq(lambda t: after.sol(t)[0]-.15, bounce, .7)
    discovery_deadline = time.monotonic()+10.
    while pub.get_subscription_count() == 0 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    assert pub.get_subscription_count() > 0, 'installed relay did not subscribe'
    start = time.monotonic()
    ready.set()
    index = 0
    sent = []
    eligible_stamps = []
    while not stop.is_set():
        deadline = start + index/rate
        remaining = deadline-time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        elapsed = time.monotonic()-start
        phase = (elapsed-.8) % 1.0
        msg = NamedPoseArrayV2()
        stamp = time.time_ns()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        # A physical frame fault at the end must revoke pending work.
        msg.header.frame_id = 'wrong_frame' if 1.0 < elapsed < 1.08 or elapsed > 4.0 else 'world'
        latency = LatencyInfo()
        latency.source, latency.latency = 'NatNetClockUncertainty', 20
        msg.latencies.append(latency)
        base = NamedPose()
        base.name = 'UCB_P1'
        base.pose.position.x, base.pose.position.y = -.43, -.7
        base.pose.orientation.w = 1.
        msg.poses.append(base)
        if elapsed > .8 and phase < .45:
            state = before.sol(phase) if phase < bounce else after.sol(phase)
            ball = NamedPose()
            ball.name = 'Ball'
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = map(float, state[:3])
            # Unavailable orientation must preserve XYZ without inventing spin.
            msg.poses.append(ball)
        pub.publish(msg)
        sent.append(stamp)
        # Frozen before this scored run: normal first-two-flight epochs after
        # 100 ms acquisition and before 130 ms lead, excluding the deliberate
        # physical frame fault and required-base outage. All game histories
        # remain in the denominator. Every launched source exposure is retained.
        if (.8 < elapsed < 2.4 and .1 < phase < plane_time-.13 and
                not (1.0 < elapsed < 1.08)):
            eligible_stamps.append(stamp)
        index += 1
    stats.put({'sent': index, 'source_intervals_ns': np.diff(sent).tolist(), 'bounce_s': bounce,
               'plane_s': plane_time, 'eligible_stamps': eligible_stamps,
               'command_epoch_contract': 'first_two_flights; phase>0.100; lead>0.130; physical_frame_fault_excluded; unknown_game_history_included'})
    node.destroy_node()
    rclpy.shutdown()


def run(args, rate, policy):
    processes = []
    logs = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    def launch(exe, params=()):
        cmd = [str(exe), '--ros-args']
        for key, value in params:
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
    relay = launch(args.bringup/'optitrack_mct_relay', [('publish_tf', 'false'),
        ('ball_calibration_id', registration), ('planner_poses_topic', '/poses/planner_full')])
    base = launch(args.bringup/'hope_base_pose_flat_relay', base_params)
    planner = launch(args.planner, [('prediction_backend', 'v2'), ('registration_id', registration),
        ('world_frame_id', int(world_sha[:13], 16)), ('plane_policy', policy)])
    launch(args.decoder)
    node = rclpy.create_node(f'integrated_chain_observer_{rate}_{policy.lower()}')
    decoded, wire, traces, statuses, observations, legacy, bases = [], [], [], [], [], [], []
    node.create_subscription(Float64MultiArray, '/a3/base_pose_flat', lambda m: bases.append(list(m.data)), qos_profile_sensor_data)
    node.create_subscription(Float64MultiArray, '/hope/test/integrated_runner/decoded',
                             lambda m: decoded.append(list(m.data)), 10)
    node.create_subscription(Float64MultiArray, '/hope/shadow/integrated_planner/racket_target',
                             lambda m: wire.append(list(m.data)), 10)
    def trace(m):
        for d in m.status:
            row = {kv.key: kv.value for kv in d.values}
            row['receipt_time_ns'] = time.time_ns()
            traces.append(row)
    node.create_subscription(DiagnosticArray, '/hope/shadow/integrated_planner/trace', trace, qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/hope/shadow/integrated_planner/status',
        lambda m: statuses.extend([{**{kv.key: kv.value for kv in d.values}, 'reason': d.message} for d in m.status]), 10)
    node.create_subscription(BallObservationV2, '/hope/ball/observation_v2', observations.append, qos_profile_sensor_data)
    node.create_subscription(PoseArray, '/poses/planner_full', legacy.append, qos_profile_sensor_data)
    stop, ready, queue = mp.Event(), mp.Event(), mp.Queue()
    camera = mp.Process(target=source, args=(rate, stop, ready, queue))
    try:
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
        while time.monotonic()-start < 4.7:
            rclpy.spin_once(node, timeout_sec=.002)
            # Missing mandatory pose producer must become observable after TTL.
            if time.monotonic()-start > 2.5 and not base_stopped:
                base.terminate()
                base.wait(timeout=5)
                base_stopped = True
        stop.set()
        source_stats = queue.get(timeout=5)
        camera.join(timeout=5)
        assert not camera.is_alive()
        valid = [w for w in wire if w[1] == 1.]
        assert valid and decoded and traces, (Counter(s['reason'] for s in statuses),
            bases[:1], bases[-1:], len(observations), len(legacy))
        assert all(t['game_history'] != 'COMPLETE' for t in traces)
        assert any(t['bounce_observed'] == 'false' for t in traces), 'no pre-bounce command'
        assert all(float(t['source_age_ms']) <= 30 for t in traces)
        assert any(w[1] == 0 for w in wire), 'no pending revocation'
        assert any(s['reason'] == 'BASE_UNAVAILABLE' for s in statuses)
        assert any(s['reason'] == 'SOURCE_UNQUALIFIED' for s in statuses)
        assert all(not o.orientation_valid and not o.omega_valid for o in observations)
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
        return {'status': 'PASS', 'rate_hz': rate, 'plane_policy': policy,
                'valid_commands': len(valid), 'decoded_callbacks': len(decoded), 'matched_commands': matched,
                'before_bounce_commands': sum(t['bounce_observed'] == 'false' for t in traces),
                'trace_receipt_ms': distribution(latencies),
                'actual_decoder_ms': distribution(decode_latency),
                'source_to_publication_ms': distribution(publication_age), 'resources': resources,
                'relay_ball_observations': len(observations), 'legacy_full_rate_frames': len(legacy),
                'command_coverage_gate': 'PASS', 'command_coverage': coverage,
                'command_denominator': len(expected), 'covered_command_epochs': len(covered),
                'reasons': dict(Counter(s['reason'] for s in statuses)), 'source': source_stats,
                'traces': traces, 'wire': wire, 'decoded': decoded, 'graph': graph,
                'camera_driver_included': False, 'synthetic_registration': True,
                'context_publisher': False}
    finally:
        stop.set()
        if camera.pid:
            camera.join(timeout=5)
            if camera.is_alive():
                camera.terminate()
        node.destroy_node()
        for p in reversed(processes):
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
        for log in logs:
            log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--planner', type=Path, required=True)
    p.add_argument('--decoder', type=Path, required=True)
    p.add_argument('--bringup', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--rate', type=int, choices=[150, 300], default=300)
    p.add_argument('--plane-policy', choices=['FOLLOW_POSE', 'ON_REFRESH'], default='FOLLOW_POSE')
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
