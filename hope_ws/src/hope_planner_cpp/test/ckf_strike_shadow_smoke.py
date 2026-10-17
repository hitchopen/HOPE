#!/usr/bin/env python3
"""Generated synthetic fixtures only: registered plane, contact gate and wire intake."""
import argparse
import json
import math
import os
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    for name in ('forecaster', 'selector', 'parser'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--report', help='Optional generated scenario results, outside source control')
    args = parser.parse_args()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseArray, Pose
    from hope_msgs.msg import BallForecast, HybridTrajectoryContext, RacketCommand
    from std_msgs.msg import Float64MultiArray, String
    from std_srvs.srv import Trigger

    common = ['--ros-args', '-p', 'frame_registration_confirmed:=true', '-p',
              'frame_registration_id:=fixture', '-p', 'source_clock_registered:=true',
              '-p', 'base_world_frame_id:=1']
    limits = ['-p', 'hit_y_min_table_m:=-1.0', '-p', 'hit_y_max_table_m:=-0.3']
    # Missing/partial/reversed/equal/nonfinite configuration must refuse startup.
    bad_limits = [[], ['-p', 'hit_y_min_table_m:=-1.0'], ['-p', 'hit_y_max_table_m:=-0.3']]
    for lo, hi in [('-.3', '-1.'), ('-.7', '-.7'), ('.nan', '-.3'),
                   ('-1.', '.nan'), ('-.inf', '-.3'), ('-1.', '.inf')]:
        bad_limits.append(['-p', 'hit_y_min_table_m:=' + lo, '-p', 'hit_y_max_table_m:=' + hi])
    for configuration in bad_limits:
        rejected = subprocess.run([args.selector] + common + configuration,
                                  capture_output=True, text=True, timeout=4)
        assert rejected.returncode != 0 and 'finite hit_y_min_table_m < hit_y_max_table_m required' in rejected.stderr, rejected.stderr
    for binary, topic in ((args.selector, '/planner/shadow_racket_command'),
                          (args.forecaster, '/ball/forecast/v1/event')):
        rejected = subprocess.run([binary] + common + limits + ['-r', topic + ':=/racket/command'],
                                  capture_output=True, text=True, timeout=4)
        assert rejected.returncode != 0 and 'remapping forbidden' in rejected.stderr, rejected.stderr

    rows = []

    def session(index, base_x, contacts, bounds=(-1., -.3), hz=200):
        # Fresh domains prevent retired endpoints from satisfying discovery.
        domain = 30 + (os.getpid() + index) % 160
        environment = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        context = Context()
        rclpy.init(context=context, domain_id=domain)
        node = rclpy.create_node('ckf_shadow_smoke', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        pose_pub = node.create_publisher(PoseArray, '/poses', QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT))
        base_pub = node.create_publisher(Float64MultiArray, '/a3/base_pose_flat', 32)
        context_pub = node.create_publisher(HybridTrajectoryContext, '/ball/hybrid/context', 32)
        freeze = node.create_client(Trigger, '/hope_hybrid_trajectory_node/freeze_x_hit')
        typed, flat, compact, full, diagnostics = [], [], [], [], []
        subscriptions = [
            node.create_subscription(RacketCommand, '/planner/shadow_racket_command', typed.append, 32),
            node.create_subscription(Float64MultiArray, '/planner/shadow_racket_command_flat', flat.append, 32),
            node.create_subscription(BallForecast, '/ball/forecast/v1/event', compact.append, 512),
            node.create_subscription(BallForecast, '/ball/forecast/v1/trajectory', full.append, 512),
            node.create_subscription(String, '/planner/strike_target_selector/diagnostics', diagnostics.append, 64),
        ]
        processes, logs = [], []

        def spin_for(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.001)

        def wait(predicate, timeout=5):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.001)
                if predicate():
                    return
            raise AssertionError('condition timed out; diagnostics=' + repr([m.data for m in diagnostics[-8:]]))

        def call_freeze():
            future = freeze.call_async(Trigger.Request())
            wait(future.done)
            return future.result()

        sequence = 0

        def bases(x, y):
            nonlocal sequence
            for i in range(12):
                sequence += 1
                ns = node.get_clock().now().nanoseconds
                sec, nsec = divmod(ns, 10**9)
                # Valid schema 2, wxyz identity, required flags and registered IDs.
                packet = Float64MultiArray()
                packet.data = [2., 1., float(sequence), float(sec), float(nsec),
                               x + (.0002 if i % 2 else -.0002), y, .76,
                               1., 0., 0., 0., 1., 47., 11., 1.]
                base_pub.publish(packet)
                spin_for(.005)
            spin_for(.005)

        def reset_session():
            reset = HybridTrajectoryContext()
            reset.kind = reset.RESET
            reset.frame_id = 'table'
            reset.evidence_source_id = 'fixture'
            for _ in range(40):
                reset.source_time_ns = node.get_clock().now().nanoseconds
                context_pub.publish(reset)
                end = time.monotonic() + .1
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.001)
                    if (any(m.reason == 'explicit_reset' and m.identity.source_time_ns == reset.source_time_ns for m in compact)
                            and any(m.data == 'transition=session_reset source_ns=' + str(reset.source_time_ns) for m in diagnostics)):
                        return
            raise AssertionError('missing exact reset acknowledgments')

        def publish_pose(stamp, x, y, z):
            wait(lambda: node.get_clock().now().nanoseconds >= stamp, 2)
            message = PoseArray()
            message.header.frame_id = 'table'
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.
            message.poses = [pose]
            pose_pub.publish(message)

        with tempfile.TemporaryDirectory(prefix='hope-shadow-') as directory:
            try:
                for path, parameters in [
                        (args.forecaster, []),
                        (args.selector, ['-p', 'context_evidence_source_id:=fixture', '-p', 'publish_shadow_flat:=true',
                                         '-p', 'hit_y_min_table_m:=' + str(bounds[0]),
                                         '-p', 'hit_y_max_table_m:=' + str(bounds[1])])]:
                    log = open(os.path.join(directory, str(len(logs)) + '.log'), 'w+')
                    logs.append(log)
                    processes.append(subprocess.Popen([path] + common + parameters, env=environment,
                                                      stdout=log, stderr=subprocess.STDOUT))
                wait(lambda: pose_pub.get_subscription_count() == 2 and base_pub.get_subscription_count() == 2
                     and context_pub.get_subscription_count() == 2 and freeze.service_is_ready())
                # Real validator and real service: never use_known_test_plane.
                for _ in range(3):
                    bases(base_x, 0.)
                    response = call_freeze()
                    if response.success:
                        break
                assert response.success, response.message
                reset_session()
                begin = HybridTrajectoryContext()
                begin.kind = begin.BEGIN
                begin.frame_id = 'table'
                begin.evidence_source_id = 'fixture'
                begin.rally_id = 1
                begin.source_time_ns = node.get_clock().now().nanoseconds
                context_pub.publish(begin)
                # Moving the robot cannot move either frozen X or the table Y bounds.
                bases(base_x + .04, .2)
                response = call_freeze()
                assert not response.success and response.message == 'refresh_forbidden_active_rally', response.message
                compact.clear(); full.clear(); diagnostics.clear()
                accepted = 0
                last_x = 1.36
                for flight, y in enumerate(contacts, 1):
                    if flight > 1:
                        origin = node.get_clock().now().nanoseconds + 5000000
                        for i in range(1, 46):
                            publish_pose(origin + i*5000000, last_x + (2.2-last_x)*i/45, y, .5)
                    old_typed, old_flat = len(typed), len(flat)
                    old_compact, old_full = len(compact), len(full)
                    origin = node.get_clock().now().nanoseconds + 5000000
                    start_x = 1.8 if flight == 1 else 2.2
                    count = math.ceil((start_x-1.37)/4*hz)
                    for i in range(count+1):
                        stamp = origin + round(i*1e9/hz)
                        t = (stamp-origin)*1e-9
                        last_x = start_x - 4*t
                        publish_pose(stamp, last_x, y, .5+.5*t-4.905*t*t)
                    # No further poses: timer and retained estimate must finish alone.
                    wait(lambda: len(typed) > old_typed)
                    wait(lambda: any('transition=decision_emitted' in m.data and 'physical_flight_id=' + str(flight) + ' ' in m.data for m in diagnostics))
                    spin_for(.025)
                    trace = next(m.data for m in reversed(diagnostics) if 'transition=decision_emitted' in m.data)
                    metadata = dict(part.split('=', 1) for part in trace.split() if '=' in part)
                    expected = bounds[0] <= y <= bounds[1]
                    assert len(typed) == old_typed+1 and typed[-1].valid == expected, trace
                    assert len(flat) == old_flat+int(expected), trace
                    assert metadata['stage3_attempted'] == str(int(expected)), trace
                    assert metadata['hit_y_min_table_m'] == format(bounds[0], '.17g'), trace
                    assert metadata['hit_y_max_table_m'] == format(bounds[1], '.17g'), trace
                    assert metadata['base_y_source'] in ('registered_current', 'registered_retained'), trace
                    crossing = [m for m in compact[old_compact:] if m.crossing_exists]
                    trajectory = [m for m in full[old_full:] if m.waypoints]
                    assert crossing and trajectory, 'forecasting suppressed by Y decision'
                    for message in crossing + trajectory:
                        assert abs(message.x_hit_m - (base_x+.58)) < 1e-12
                        assert message.hit_plane_generation == 1 and not message.known_test_plane
                        assert message.world_frame_id == 1 and message.calibration_id == 11
                    if expected:
                        accepted += 1
                        wire = list(flat[-1].data)
                        assert wire[14:17] == [float(accepted), float(flight), 1.], wire
                        assert abs(typed[-1].position.x - (base_x+.58)) < 1e-10
                        assert list((typed[-1].position.x, typed[-1].position.y, typed[-1].position.z)) == wire[3:6]
                        assert list((typed[-1].velocity.x, typed[-1].velocity.y, typed[-1].velocity.z)) == wire[6:9]
                        assert typed[-1].strike_time == wire[10] and typed[-1].time_to_strike == wire[9]
                        assert typed[-1].header.frame_id == 'table'
                        assert abs(typed[-1].normal.x**2 + typed[-1].normal.y**2 + typed[-1].normal.z**2 - 1) < 1e-10
                    else:
                        assert metadata['reason'] == 'hit_y_out_of_range' and metadata['fatal_identity_reset'] == '0', trace
                        assert typed[-1].time_to_strike == typed[-1].strike_time == 0
                    row = dict(fixture='synthetic', base_start_x=base_x, base_start_y=0., moved_base_y=.2,
                               expected_hit_x=base_x+.58, hz=hz, y_bounds_table=bounds, fixture_contact_y=y, selected_contact_y=float(metadata['contact_y_table_m']),
                               typed_count=1, valid=expected, flat_count=int(expected),
                               compact_crossings=len(crossing), full_trajectories=len(trajectory), diagnostics=metadata)
                    rows.append(row)
                    print(json.dumps(row, sort_keys=True))
                if flat:
                    packet = os.path.join(directory, 'packet.txt')
                    with open(packet, 'w') as output:
                        output.write(' '.join(format(x, '.17g') for x in flat[-1].data))
                    subprocess.run([args.parser, packet], check=True, timeout=3)
                for topic in ('/racket/command', '/racket/command_flat'):
                    assert not node.get_publishers_info_by_topic(topic), topic
            except BaseException:
                for log in logs:
                    log.flush(); log.seek(0); print(log.read()[-5000:])
                raise
            finally:
                for process in processes:
                    process.terminate()
                for process in processes:
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
                for log in logs:
                    log.close()
                for subscription in subscriptions:
                    node.destroy_subscription(subscription)
                executor.remove_node(node); executor.shutdown()
                node.destroy_node(); rclpy.shutdown(context=context)

    cases = [(-.58, [-.7], (-1., -.3), 200), (-.50, [-.7], (-1., -.3), 200),
             (-.68, [-.7], (-1., -.3), 200),
             (-.58, [-.700001, -.65], (-.7, -.6), 200),
             (-.50, [-.599999, -.65], (-.7, -.6), 200)]
    try:
        for index, (base, contacts, bounds, hz) in enumerate(cases):
            session(index, base, contacts, bounds, hz)
    finally:
        if args.report:
            with open(args.report, 'w') as output:
                json.dump(dict(scenarios=rows, startup_rejections=len(bad_limits)), output, indent=2)
    print('PASS: registered frozen session planes, Y rejection/recovery, uninterrupted compact/full forecasts, current parser')


if __name__ == '__main__':
    main()
