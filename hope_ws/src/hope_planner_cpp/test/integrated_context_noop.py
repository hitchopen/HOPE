#!/usr/bin/env python3
"""Regression for unconditional revocation by ignored/repeated V2 context."""
import argparse
import json
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from hope_msgs.msg import BallObservationV2, FlightContextV2, FlightPredictionV2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--executable', required=True)
    p.add_argument('--worker-delay-ms', type=int, default=0)
    args = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('v2_context_noop_regression')
    process = subprocess.Popen([args.executable, '--ros-args', '-p', 'synthetic_plane:=true',
        '-p', 'session_id:=noop_regression', '-p', 'plane_policy:=ON_REFRESH',
        '-p', f'test.worker_delay_ms:={args.worker_delay_ms}',
        '-p', 'test.worker_delay_every:=1'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    predictions = []
    node.create_subscription(FlightPredictionV2, '/hope/shadow/flight_v2/prediction',
                             predictions.append, qos_profile_sensor_data)
    observation = node.create_publisher(BallObservationV2, '/hope/ball/observation_v2', qos_profile_sensor_data)
    context = node.create_publisher(FlightContextV2, '/hope/shadow/flight_v2/context', 32)
    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.001)
    def revoked():
        return sum(m.reason == 'semantic_context_superseded' for m in predictions)
    try:
        until = time.monotonic()+8
        while (not observation.get_subscription_count() or not context.get_subscription_count()) and time.monotonic() < until:
            spin(.01)
        assert observation.get_subscription_count() and context.get_subscription_count()
        start = time.time_ns()
        last = start
        for i in range(45):
            last = time.time_ns()
            t = (last-start)*1e-9
            m = BallObservationV2()
            m.source_time_ns, m.source_time_valid = last, True
            m.clock_domain, m.clock_mapping_id, m.clock_mapping_valid = 'ros_system', 'test', True
            m.frame_id, m.ball_id, m.position_valid = 'table', 'ball', True
            m.position.x, m.position.y, m.position.z = 1.8-5*t, -.7, .4-.5*t-4.905*t*t
            observation.publish(m)
            spin(.003)
        assert any(m.physical_available for m in predictions)
        generation = predictions[-1].shot_generation
        message = FlightContextV2()
        message.session_id, message.authority = 'noop_regression', 'fixture'
        message.shot_generation, message.source_time_ns = generation, last
        initial = revoked()
        message.operation, message.match_context = 'CONTEXT', 'SERVE'
        for field, bad in [('shot_generation', generation+1), ('authority', 'wrong-authority'),
                           ('session_id', 'wrong-session')]:
            original = getattr(message, field)
            setattr(message, field, bad)
            context.publish(message)
            spin(.001)
            setattr(message, field, original)
        assert revoked() == initial, 'rejected context revoked a forecast'
        for op in ['UNSUPPORTED', 'REFRESH_PLANE', 'CONTEXT', 'CONTEXT']:
            message.operation = op
            message.match_context = 'UNKNOWN'
            context.publish(message)
            spin(.002)
        assert revoked() == initial, 'ignored context revoked valid physical output'
        message.operation, message.match_context = 'CONTEXT', 'NORMAL_RALLY'
        context.publish(message)
        spin(.003)
        assert revoked() == initial+1, 'accepted semantic change did not revoke'
        fence = next(m.context_revision for m in reversed(predictions)
                     if m.reason == 'semantic_context_superseded')
        index = len(predictions)
        context.publish(message)
        spin(.002)
        assert revoked() == initial+1, 'repeat of accepted context revoked again'
        for invalid in ['UNSUPPORTED_MATCH_CONTEXT', '']:
            message.match_context = invalid
            context.publish(message)
            spin(.002)
        assert revoked() == initial+1, 'unsupported enum value withdrew accepted context'
        spin(.04)
        assert all(not m.physical_available or m.context_revision >= fence
                   for m in predictions[index:]), 'stale in-flight result reappeared after semantic fence'
        print(json.dumps({'status': 'PASS', 'worker_delay_ms': args.worker_delay_ms,
                          'ignored_revocations': 0, 'accepted_semantic_revocations': 1,
                          'post_fence_stale_results': 0}))
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
