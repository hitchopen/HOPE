"""Observer status heartbeats and source loss on localhost domain 219 only."""
import ast
import os
from pathlib import Path
import sys
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
        probe = rclpy.create_node('observer_contract_probe', start_parameter_services=False)
        executor = SingleThreadedExecutor()
        executor.add_node(observer)
        executor.add_node(probe)
        data = {key: [] for key in ('alive', 'standing', 'mode')}
        subs = []
        for key, kind in [('alive', Bool), ('standing', Bool), ('mode', String)]:
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
            print({'alive_count': len(data['alive']), 'standing_count': len(data['standing']),
                   'source_loss_s': lost_at - stopped,
                   'recovery_s': recovered_at - resumed}, flush=True)
        finally:
            executor.shutdown()
            probe.destroy_node()
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
