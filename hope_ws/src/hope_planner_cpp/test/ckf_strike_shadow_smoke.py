#!/usr/bin/env python3
"""Tiny generated-data smoke; no private fixtures, production publishers or runner."""
import argparse
import os
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    for name in ('forecaster', 'selector', 'parser'):
        parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 90)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseArray, Pose
    from hope_msgs.msg import BallForecast, HybridTrajectoryContext, RacketCommand
    from std_msgs.msg import Float64MultiArray, String

    rclpy.init()
    node = rclpy.create_node('ckf_shadow_smoke')
    pose_pub = node.create_publisher(PoseArray, '/poses', QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT))
    context_pub = node.create_publisher(HybridTrajectoryContext, '/ball/hybrid/context', 32)
    typed, flat, compact, full, diagnostics = [], [], [], [], []
    subscriptions = [
        node.create_subscription(RacketCommand, '/planner/shadow_racket_command', typed.append, 32),
        node.create_subscription(Float64MultiArray, '/planner/shadow_racket_command_flat', flat.append, 32),
        node.create_subscription(BallForecast, '/ball/forecast/v1/event', compact.append, 512),
        node.create_subscription(BallForecast, '/ball/forecast/v1/trajectory', full.append, 512),
        node.create_subscription(String, '/planner/strike_target_selector/diagnostics', diagnostics.append, 64),
    ]

    def wait(predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.002)
            if predicate():
                return
        raise AssertionError('condition timed out; diagnostics=' + repr([m.data for m in diagnostics[-8:]]))

    common = ['--ros-args', '-p', 'frame_registration_confirmed:=true', '-p',
              'frame_registration_id:=fixture', '-p', 'source_clock_registered:=true',
              '-p', 'base_world_frame_id:=1']
    processes = []
    with tempfile.TemporaryDirectory(prefix='hope-shadow-') as directory:
        logs = []
        def start(path, params):
            log = open(os.path.join(directory, str(len(logs)) + '.log'), 'w+')
            logs.append(log)
            process = subprocess.Popen([path] + common + params, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process
        try:
            for binary, topic in ((args.selector, '/planner/shadow_racket_command'),
                                  (args.forecaster, '/ball/forecast/v1/event')):
                rejected = subprocess.run([binary] + common + ['-r', topic + ':=/racket/command'],
                                          capture_output=True, text=True, timeout=4)
                assert rejected.returncode != 0 and 'remapping forbidden' in rejected.stderr, rejected.stderr
            start(args.forecaster, ['-p', 'use_known_test_plane:=true', '-p', 'known_test_plane_x_m:=0.0'])
            start(args.selector, ['-p', 'tagged_synthetic_test:=true', '-p', 'context_evidence_source_id:=fixture',
                                  '-p', 'nominal_base_y_m:=0.0', '-p', 'publish_shadow_flat:=true'])
            wait(lambda: pose_pub.get_subscription_count() == 2 and context_pub.get_subscription_count() == 2
                 and len(node.get_publishers_info_by_topic('/planner/shadow_racket_command')) == 1)
            # Exact tagged request/ack establishes a ready session, not a startup sleep.
            reset = HybridTrajectoryContext()
            reset.kind = reset.RESET
            reset.frame_id = 'table'
            reset.evidence_source_id = 'fixture'
            # Discovery counts can precede DDS data readiness. Retry fresh, exactly
            # tagged resets until both nodes acknowledge the same request.
            ready = False
            for _ in range(40):
                reset.source_time_ns = node.get_clock().now().nanoseconds
                context_pub.publish(reset)
                end = time.monotonic() + .1
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.002)
                    ready = (any(m.reason == 'explicit_reset' and m.identity.source_time_ns == reset.source_time_ns for m in compact)
                             and any(m.data == 'transition=session_reset source_ns=' + str(reset.source_time_ns) for m in diagnostics))
                    if ready:
                        break
                if ready:
                    break
            assert ready, 'missing exact reset acknowledgments; compact=' + repr([(m.reason, m.identity.source_time_ns) for m in compact])
            typed.clear(); flat.clear(); diagnostics.clear(); compact.clear(); full.clear()
            origin = node.get_clock().now().nanoseconds + 10000000
            # Stop at the measured negative-X net straddle; timer must finish alone.
            for i in range(23):
                stamp = origin + i * 5000000
                wait(lambda: node.get_clock().now().nanoseconds >= stamp, 2)
                message = PoseArray()
                message.header.frame_id = 'table'
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                pose = Pose(); t = i * .005
                pose.position.x = 1.8 - 4*t
                pose.position.y = -.7
                pose.position.z = .5 + .5*t - 4.905*t*t
                pose.orientation.w = 1.
                message.poses = [pose]
                pose_pub.publish(message)
            wait(lambda: len(typed) >= 1 and len(flat) >= 1)
            wait(lambda: any('decision_emitted' in m.data for m in diagnostics))
            until = time.monotonic() + .15
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.005)
            assert len(typed) == len(flat) == 1, (len(typed), len(flat))
            assert typed[0].valid and len(flat[0].data) == 19
            assert any(m.crossing_exists for m in compact), 'missing compact crossing'
            assert any(m.waypoints for m in full), 'missing full CKF state'
            wire = list(flat[0].data)
            assert list((typed[0].position.x, typed[0].position.y, typed[0].position.z)) == wire[3:6]
            assert list((typed[0].velocity.x, typed[0].velocity.y, typed[0].velocity.z)) == wire[6:9]
            assert typed[0].time_to_strike == wire[9]
            assert typed[0].strike_time == wire[10]
            assert wire[14:17] == [1., 1., 1.]
            assert typed[0].header.frame_id == 'table'
            for topic in ('/racket/command', '/racket/command_flat'):
                assert not node.get_publishers_info_by_topic(topic), topic
            packet = os.path.join(directory, 'packet.txt')
            with open(packet, 'w') as output:
                output.write(' '.join(format(x, '.17g') for x in wire))
            subprocess.run([args.parser, packet], check=True, timeout=3)
            print('PASS: compact/full CKF, stopped-after-net timer, one typed/flat decision, exact vectors, current parser, no production publisher')
        finally:
            for process in processes:
                process.terminate()
            for process in processes:
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            for log in logs:
                log.seek(0)
                text = log.read()
                if not typed:
                    print(text[-5000:])
                log.close()
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
