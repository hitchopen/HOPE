"""Observer status heartbeats and source loss on localhost domain 219 only."""
import ast
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest


@unittest.skipUnless(os.environ.get('ROS_LOCALHOST_ONLY') == '1'
                     and os.environ.get('ROS_DOMAIN_ID') == '219'
                     and os.environ.get('HOPE_OBSERVER_DIR'),
                     'requires isolated localhost domain 219 and staged observer')
class ObserverStatusTests(unittest.TestCase):
    def test_status_change_heartbeat_and_source_loss(self):
        sys.path.insert(0, os.environ['HOPE_OBSERVER_DIR'])
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Bool, Float64MultiArray, String
        from hope_observer import HopeObserver
        from hope_runner_control_core import decode_runner_state

        # Remap all wires into a test prefix in addition to DDS isolation.
        source = Path(os.environ['HOPE_OBSERVER_DIR']) / 'hope_observer.py'
        names = {n.value for n in ast.walk(ast.parse(source.read_text()))
                 if isinstance(n, ast.Constant) and isinstance(n.value, str)
                 and n.value.startswith(('/hope/', '/a3/', '/racket/', '/poses'))}
        args = ['--ros-args']
        for name in sorted(names):
            args += ['-r', f'{name}:=/cpu_observer_test{name}']
        rclpy.init(args=args)
        observer = HopeObserver()
        self.assertEqual(observer.get_parameter('runtime_snapshot_max_age_s').value, 2.0)
        probe = rclpy.create_node('observer_contract_probe', start_parameter_services=False)
        executor = SingleThreadedExecutor()
        executor.add_node(observer)
        executor.add_node(probe)
        data = {key: [] for key in ('alive', 'standing', 'mode', 'summary')}
        subs = []
        for key, kind in [('alive', Bool), ('standing', Bool), ('mode', String), ('summary', String)]:
            subs.append(probe.create_subscription(kind, f'/cpu_observer_test/hope/runner/{key}',
                lambda msg, key=key: data[key].append((time.monotonic(), msg.data)), 10))
        pub = probe.create_publisher(Float64MultiArray,
            '/cpu_observer_test/hope/runner/state_hdu_flat', 10)
        # Valid standing Runner state; no command request is ever published.
        packet = [2.,1234.,8.,1.,1.,1.,0.,2.,1.,1.,1.,1.,0.,-1.,-1.,0.,99.,2.,1.,1.,555.]
        self.assertEqual(decode_runner_state(packet).run_mode, 'PD_STAND')
        enabled = [True]
        def publish():
            if enabled[0]:
                packet[2] += 1
                pub.publish(Float64MultiArray(data=packet))
        timer = probe.create_timer(.05, publish)
        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)
        try:
            spin(2.)
            self.assertTrue(data['standing'][-1][1])
            data = {key: [] for key in data}
            spin(4.)
            self.assertGreaterEqual(len(data['alive']), 18)
            self.assertGreaterEqual(len(data['mode']), 18)
            self.assertGreaterEqual(len(data['standing']), 6)
            self.assertLessEqual(len(data['standing']), 11)
            self.assertTrue(all(value for _, value in data['standing']))
            # Source loss is still evaluated every original snapshot tick.
            enabled[0] = False
            stopped = time.monotonic()
            spin(1.4)
            self.assertFalse(data['alive'][-1][1])
            self.assertFalse(data['standing'][-1][1])
            lost_at = next(at for at, value in data['standing'] if not value)
            self.assertLess(lost_at - stopped, 1.35)
            # Recovery changes bypass the unchanged-status heartbeat.
            enabled[0] = True
            resumed = time.monotonic()
            spin(.4)
            self.assertTrue(data['standing'][-1][1])
            recovered_at = next(at for at, value in data['standing'] if at > resumed and value)
            self.assertLess(recovered_at - resumed, .35)
            # The shared command cache preserves its last good frame, but the
            # observer must still expose the latest malformed wire as invalid.
            enabled[0] = False
            pub.publish(Float64MultiArray(data=[2.]))
            spin(.3)
            self.assertFalse(data['alive'][-1][1])
            self.assertEqual(data['mode'][-1][1], 'UNKNOWN')
            self.assertIn('MALFORMED RUNNER STATE', data['summary'][-1][1])
            enabled[0] = True
            spin(.4)
            self.assertTrue(data['alive'][-1][1])
            self.assertEqual(data['mode'][-1][1], 'PD_STAND')
            print({'alive_count': len(data['alive']), 'standing_count': len(data['standing']),
                   'source_loss_s': lost_at - stopped,
                   'recovery_s': recovered_at - resumed}, flush=True)
        finally:
            executor.shutdown()
            probe.destroy_node()
            observer.destroy_node()
            rclpy.shutdown()

    def test_runtime_cache_stall_error_and_recovery_are_explicit_on_topics(self):
        sys.path.insert(0, os.environ['HOPE_OBSERVER_DIR'])
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Bool, String
        from hope_observer import HopeObserver

        session = 'model21800_20261022T010203Z'
        attempt = 'planner_attempt_001'
        source = Path(os.environ['HOPE_OBSERVER_DIR']) / 'hope_observer.py'
        names = {n.value for n in ast.walk(ast.parse(source.read_text()))
                 if isinstance(n, ast.Constant) and isinstance(n.value, str)
                 and n.value.startswith(('/hope/', '/a3/', '/racket/', '/poses'))}
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session_path = root / 'session'
            session_path.write_text(session)
            hdu = root / 'runtime' / session / 'hdu'
            (hdu / attempt).mkdir(parents=True)
            (hdu / 'current_planner_attempt').write_text(attempt)
            (hdu / attempt / 'pid.txt').write_text(str(os.getpid()))
            (hdu / 'x_hit.status').write_text('request=test\nsuccess=1\nmessage=x_hit=1.2\n')
            args = ['--ros-args', '-p', f'session_id_path:={session_path}',
                    '-p', f'runtime_root:={root / "runtime"}', '-p', 'planner_process_fragment:=python',
                    '-p', 'publish_period_s:=0.03', '-p', 'runtime_snapshot_max_age_s:=0.15']
            for name in sorted(names):
                args += ['-r', f'{name}:=/cache_observer_test{name}']
            rclpy.init(args=args)
            observer = HopeObserver()
            probe = rclpy.create_node('observer_cache_probe', start_parameter_services=False)
            executor = SingleThreadedExecutor()
            executor.add_node(observer)
            executor.add_node(probe)
            latest, heartbeats = {}, []
            topics = (
                ('known', Bool, '/hope/session/runtime_known'),
                ('active', Bool, '/hope/session/active'),
                ('session', String, '/hope/session/id'),
                ('session_text', String, '/hope/session/text'),
                ('planner', Bool, '/hope/planner/process_alive'),
                ('planner_status', String, '/hope/planner/process_status'),
                ('attempt', String, '/hope/planner/attempt'),
                ('x_hit', Bool, '/hope/x_hit/status_available'),
                ('x_hit_text', String, '/hope/x_hit/status'),
            )
            for key, kind, topic in topics:
                probe.create_subscription(kind, '/cache_observer_test' + topic,
                                          lambda msg, key=key: latest.__setitem__(key, msg.data), 10)
            probe.create_subscription(Bool, '/cache_observer_test/hope/observer_alive',
                                      lambda msg: heartbeats.append(msg.data), 10)
            release, entered = threading.Event(), threading.Event()
            block = [True]
            read = observer._read_runtime_snapshot
            def refresh():
                if block[0]:
                    entered.set()
                    release.wait(5)
                return read()
            observer._runtime_cache._callback = refresh
            def until(predicate, timeout=3):
                deadline = time.monotonic() + timeout
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.005)
                self.assertTrue(predicate(), repr(latest))
            try:
                until(lambda: entered.is_set() and
                      'initial snapshot refresh overdue' in latest.get('planner_status', ''), timeout=1.0)
                self.assertFalse(latest['known'])
                self.assertFalse(latest['planner'])
                self.assertIn('session=UNKNOWN', latest['session_text'])
                self.assertGreater(len(heartbeats), 2)

                block[0] = False
                release.set()
                until(lambda: latest.get('known') and latest.get('planner') and latest.get('x_hit')
                      and latest.get('planner_status') == 'ALIVE' and latest.get('session') == session
                      and latest.get('attempt') == attempt)
                self.assertEqual(latest['session'], session)
                self.assertEqual(latest['attempt'], attempt)
                self.assertEqual(latest['planner_status'], 'ALIVE')

                # Stall a later refresh: identity survives, live health does not.
                release.clear()
                entered.clear()
                block[0] = True
                heartbeat_count = len(heartbeats)
                until(lambda: entered.is_set() and latest.get('planner_status') ==
                      'UNKNOWN: snapshot refresh overdue' and not latest.get('planner')
                      and not latest.get('known') and not latest.get('x_hit')
                      and 'runtime=UNKNOWN' in latest.get('session_text', ''), timeout=1.0)
                self.assertEqual(latest['session'], session)
                self.assertEqual(latest['attempt'], attempt)
                self.assertIn('runtime=UNKNOWN', latest['session_text'])
                self.assertGreater(len(heartbeats), heartbeat_count + 2)

                block[0] = False
                release.set()
                until(lambda: latest.get('known') and latest.get('planner'))
                # A real filesystem error must be UNKNOWN, not normal process loss.
                session_path.unlink()
                session_path.mkdir()
                until(lambda: 'IsADirectoryError' in latest.get('planner_status', '')
                      and not latest.get('known') and not latest.get('planner')
                      and 'UNKNOWN' in latest.get('x_hit_text', ''))
                self.assertEqual(latest['session'], session)
                self.assertIn('UNKNOWN', latest['x_hit_text'])
                session_path.rmdir()
                session_path.write_text(session)
                until(lambda: latest.get('known') and latest.get('planner') and latest.get('x_hit')
                      and latest.get('planner_status') == 'ALIVE')
                self.assertEqual(latest['planner_status'], 'ALIVE')
            finally:
                release.set()
                executor.shutdown()
                probe.destroy_node()
                observer.destroy_node()
                rclpy.shutdown()

    def test_runtime_cache_budget_rejects_nonpositive_or_nonfinite_values(self):
        sys.path.insert(0, os.environ['HOPE_OBSERVER_DIR'])
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from hope_observer import ObserverModule

        rclpy.init(args=[])
        try:
            for value in (0.0, -1.0, float('inf'), float('-inf'), float('nan')):
                with self.subTest(value=value):
                    node = Node('invalid_observer_cache_budget', start_parameter_services=False,
                                parameter_overrides=[Parameter('runtime_snapshot_max_age_s', value=value)])
                    try:
                        with self.assertRaisesRegex(ValueError, 'positive and finite'):
                            ObserverModule(node)
                    finally:
                        node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
