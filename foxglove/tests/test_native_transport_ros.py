"""Isolated DDS contracts for the native tools; never uses robot domain.

Run with ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 and HOPE_NATIVE_BIN_DIR.
All topics additionally use /cpu_native_test, including command-like payloads.
"""
import math
import os
from pathlib import Path
import subprocess
import struct
import tempfile
import sys
import time
import unittest


def wire_fields(msg):
    # CDR alignment padding is not initialized consistently by the middleware.
    # Compare every actual field and IEEE float bit instead.
    return (msg.layout.data_offset,
            tuple((d.label, d.size, d.stride) for d in msg.layout.dim),
            struct.pack('<' + 'd' * len(msg.data), *msg.data))


@unittest.skipUnless(os.environ.get('ROS_LOCALHOST_ONLY') == '1'
                     and os.environ.get('ROS_DOMAIN_ID') == '219'
                     and os.environ.get('HOPE_NATIVE_BIN_DIR'),
                     'requires isolated localhost domain 219 and candidate binaries')
class NativeTransportTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        rclpy.init(args=[])
        self.ros = rclpy
        self.node = rclpy.create_node('cpu_native_contract_test')
        self.extra_nodes = []
        self.processes = []
        self.logs = []
        self.log_dir = tempfile.TemporaryDirectory(prefix="hope-native-test-")
        self.addCleanup(self.log_dir.cleanup)

    def tearDown(self):
        for proc in self.processes:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
        for stream in self.logs:
            stream.close()
        for node in self.extra_nodes:
            node.destroy_node()
        self.node.destroy_node()
        self.ros.shutdown()

    def start(self, name, parameters, remaps=None):
        if name == 'hope-transport-gateway':
            # Keep every route off the real topic namespace, even when a test
            # exercises only home transport.
            names = ('remote_state_topic', 'local_state_topic', 'local_control_topic',
                     'remote_control_topic', 'local_teleop_input_topic',
                     'remote_teleop_input_topic', 'remote_teleop_state_topic',
                     'local_teleop_state_topic', 'remote_session_home_topic',
                     'local_session_home_topic')
            parameters = {**{key: '/cpu_native_test/' + key for key in names}, **parameters}
        output = open(Path(self.log_dir.name) / (name + '-isolated-test.log'), 'w')
        self.logs.append(output)
        args = [str(Path(os.environ['HOPE_NATIVE_BIN_DIR']) / name), '--ros-args']
        for key, value in parameters.items():
            args += ['-p', f'{key}:={value}']
        for key, value in (remaps or {}).items():
            args += ['-r', f'{key}:={value}']
        proc = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
        self.processes.append(proc)
        return proc

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ros.spin_once(self.node, timeout_sec=.005)
            for node in self.extra_nodes:
                self.ros.spin_once(node, timeout_sec=0)

    def discover(self, pubs, proc):
        end = time.monotonic() + 8
        while time.monotonic() < end and not all(p.get_subscription_count() for p in pubs):
            self.assertIsNone(proc.poll(), 'native tool exited; inspect isolated-test log')
            self.spin(.05)
        self.assertTrue(all(p.get_subscription_count() for p in pubs))
        self.spin(.3)

    def test_five_runner_routes_preserve_wire_and_never_replay(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayDimension
        parameters = {}
        input_names = ['remote_state_topic', 'local_control_topic',
                       'local_teleop_input_topic', 'remote_teleop_state_topic',
                       'remote_session_home_topic']
        output_names = ['local_state_topic', 'remote_control_topic',
                        'remote_teleop_input_topic', 'local_teleop_state_topic',
                        'local_session_home_topic']
        received = [[] for _ in range(5)]
        pubs, subs = [], []
        for route in range(5):
            source, target = f'/cpu_native_test/in{route}', f'/cpu_native_test/out{route}'
            parameters[input_names[route]] = source
            parameters[output_names[route]] = target
            pubs.append(self.node.create_publisher(Float64MultiArray, source, 10))
            subs.append(self.node.create_subscription(Float64MultiArray, target,
                lambda msg, i=route: received[i].append(wire_fields(msg)), 10))
        proc = self.start('hope-transport-gateway', parameters)
        self.discover(pubs, proc)
        self.assertEqual(received, [[] for _ in range(5)], 'startup must not generate a command')
        expected = []
        for route, pub in enumerate(pubs):
            # Includes an old source timestamp, disabled input, signed zero,
            # negative numbers and layout fields; relay must not normalize any.
            msg = Float64MultiArray(data=[1., 42., 7., float(route + 1), 1., 1., 0., -0., -2.5, 0., 1.])
            msg.layout.data_offset = route
            msg.layout.dim = [MultiArrayDimension(label='identity', size=11, stride=11)]
            expected.append(wire_fields(msg))
            pub.publish(msg)
        self.spin(.6)
        self.assertEqual(received, [[value] for value in expected])
        self.spin(.7)
        self.assertEqual(received, [[value] for value in expected], 'no timer replay after input stops')

    def test_base_is_independent_before_and_after_runner_gateway(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayDimension
        received = []
        pub = self.node.create_publisher(Float64MultiArray, '/cpu_native_test/base_in', 10)
        sub = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/base_out',
            lambda msg: received.append(wire_fields(msg)), 10)
        base = self.start('hope-base-pose-transport-relay', {
            'input_topic': '/cpu_native_test/base_in', 'output_topic': '/cpu_native_test/base_out'})
        self.discover([pub], base)
        self.spin(1.1)
        self.assertEqual(received, [])
        base_log = (Path(self.log_dir.name) / 'hope-base-pose-transport-relay-isolated-test.log')
        self.assertIn('received=0 published=0 base_age=unseen', base_log.read_text())
        expected = []
        def send(sequence):
            msg = Float64MultiArray(data=[2., 123., float(sequence), -0., -3.])
            msg.layout.data_offset = 2
            msg.layout.dim = [MultiArrayDimension(label='original', size=5, stride=5)]
            expected.append(wire_fields(msg))
            pub.publish(msg)
            self.spin(.3)
            self.assertEqual(received, expected)
        send(41)  # Base is usable before a Runner gateway participant exists.
        gateway = self.start('hope-transport-gateway', {})
        self.spin(.5)
        self.assertIsNone(gateway.poll())
        send(42)
        gateway.terminate()
        gateway.wait(timeout=5)
        self.assertIsNone(base.poll())
        send(43)  # Runner transport failure cannot interrupt base transport.
        self.spin(1.1)
        self.assertEqual(received, expected, 'source loss cannot trigger replay')
        self.assertNotIn('base_age=unseen', base_log.read_text().splitlines()[-1])

    def test_late_base_participant_reconnect_preserves_source_without_replay(self):
        from std_msgs.msg import Float64MultiArray
        received = []
        sub = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/base_out',
            lambda msg: received.append(list(msg.data)), 10)
        proc = self.start('hope-base-pose-transport-relay', {
            'input_topic': '/cpu_native_test/base_in',
            'output_topic': '/cpu_native_test/base_out'})
        self.spin(.5)
        self.assertEqual(received, [], 'absent base must produce no synthetic sample')
        # A separate ROS process joins after the base relay. Recreate the source
        # participant to exercise Laptop reconnect, not only publisher recreation.
        source = """
import sys, time
import rclpy
from std_msgs.msg import Float64MultiArray
rclpy.init(args=[])
node = rclpy.create_node('late_base_source')
pub = node.create_publisher(Float64MultiArray, '/cpu_native_test/base_in', 10)
deadline = time.monotonic() + 8
while not pub.get_subscription_count() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.05)
if not pub.get_subscription_count():
    raise RuntimeError('gateway reader not discovered')
pub.publish(Float64MultiArray(data=[2., 123., float(sys.argv[1]), -0., -3.]))
end = time.monotonic() + .5
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.01)
node.destroy_node()
rclpy.shutdown()
"""
        for sequence in (41, 42):
            endpoint = subprocess.Popen([sys.executable, '-c', source, str(sequence)])
            self.processes.append(endpoint)
            deadline = time.monotonic() + 10
            while endpoint.poll() is None and time.monotonic() < deadline:
                self.assertIsNone(proc.poll())
                self.spin(.05)
            self.assertEqual(endpoint.poll(), 0)
            self.spin(.3)
            self.assertEqual(received[-1], [2., 123., float(sequence), -0., -3.])
            self.assertEqual(len(received), sequence - 40)
            self.spin(.3)
            self.assertEqual(len(received), sequence - 40, 'source loss cannot trigger replay')

    def test_one_operations_consumer_is_healthy_without_base(self):
        from std_msgs.msg import Float64MultiArray
        received = []
        state = self.node.create_publisher(Float64MultiArray, '/cpu_native_test/state', 10)
        operations = self.ros.create_node('hope_operations')
        self.extra_nodes.append(operations)
        control = operations.create_publisher(Float64MultiArray, '/cpu_native_test/control', 10)
        state_sub = operations.create_subscription(Float64MultiArray, '/cpu_native_test/local_state',
            lambda msg: received.append(list(msg.data)), 10)
        control_sub = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/remote_control',
            lambda msg: None, 10)
        proc = self.start('hope-transport-gateway', {
            'remote_state_topic': '/cpu_native_test/configured_state',
            'local_state_topic': '/cpu_native_test/configured_local_state',
            'local_control_topic': '/cpu_native_test/configured_control',
            'remote_control_topic': '/cpu_native_test/configured_remote_control'}, {
            '/cpu_native_test/configured_state': '/cpu_native_test/state',
            '/cpu_native_test/configured_local_state': '/cpu_native_test/local_state',
            '/cpu_native_test/configured_control': '/cpu_native_test/control',
            '/cpu_native_test/configured_remote_control': '/cpu_native_test/remote_control'})
        self.discover([state, control], proc)
        for _ in range(15):
            state.publish(Float64MultiArray(data=[2., 123., 1.]))
            self.spin(.1)
        self.assertTrue(received)
        text = (Path(self.log_dir.name) / 'hope-transport-gateway-isolated-test.log').read_text()
        self.assertIn('TRANSPORT GATEWAY HEALTHY', text)
        self.assertIn('local_state_subscribers=1', text)
        self.assertIn('operations_state_readers=1 operations_control_writers=1', text)
        self.assertIn('readiness=transport_only', text)

    def test_echo_and_wrong_namespace_cannot_satisfy_operations_readiness(self):
        from std_msgs.msg import Float64MultiArray
        state = self.node.create_publisher(Float64MultiArray, '/cpu_native_test/state', 10)
        control = self.node.create_publisher(Float64MultiArray, '/cpu_native_test/control', 10)
        echo = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/local_state', lambda msg: None, 10)
        remote = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/remote_control', lambda msg: None, 10)
        wrong = self.ros.create_node('hope_operations', namespace='/unrelated')
        self.extra_nodes.append(wrong)
        wrong_reader = wrong.create_subscription(Float64MultiArray, '/cpu_native_test/local_state', lambda msg: None, 10)
        wrong_writer = wrong.create_publisher(Float64MultiArray, '/cpu_native_test/control', 10)
        proc = self.start('hope-transport-gateway', {
            'remote_state_topic': '/cpu_native_test/state',
            'local_state_topic': '/cpu_native_test/local_state',
            'local_control_topic': '/cpu_native_test/control',
            'remote_control_topic': '/cpu_native_test/remote_control'})
        self.discover([state, control], proc)
        def feed():
            for _ in range(15):
                state.publish(Float64MultiArray(data=[2., 123., 1.]))
                self.spin(.1)
        log = (Path(self.log_dir.name) / 'hope-transport-gateway-isolated-test.log')
        feed()
        self.assertNotIn('TRANSPORT GATEWAY HEALTHY', log.read_text())
        self.assertIn('operations_state_readers=0 operations_control_writers=0', log.read_text())
        operations = self.ros.create_node('hope_operations')
        self.extra_nodes.append(operations)
        reader = operations.create_subscription(Float64MultiArray, '/cpu_native_test/local_state', lambda msg: None, 10)
        feed()
        self.assertNotIn('TRANSPORT GATEWAY HEALTHY', log.read_text(), 'reader alone is not bidirectional operations transport')
        writer = operations.create_publisher(Float64MultiArray, '/cpu_native_test/control', 10)
        self.discover([writer], proc)
        feed()
        self.assertIn('TRANSPORT GATEWAY HEALTHY', log.read_text())

    def test_gateway_requires_state_progress_and_requalifies_after_loss(self):
        from std_msgs.msg import Float64MultiArray
        state = self.node.create_publisher(Float64MultiArray, '/cpu_native_test/state', 10)
        operations = self.ros.create_node('hope_operations')
        self.extra_nodes.append(operations)
        control = operations.create_publisher(Float64MultiArray, '/cpu_native_test/control', 10)
        reader = operations.create_subscription(Float64MultiArray, '/cpu_native_test/local_state',
            lambda msg: None, 10)
        remote = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/remote_control',
            lambda msg: None, 10)
        proc = self.start('hope-transport-gateway', {
            'remote_state_topic': '/cpu_native_test/state',
            'local_state_topic': '/cpu_native_test/local_state',
            'local_control_topic': '/cpu_native_test/control',
            'remote_control_topic': '/cpu_native_test/remote_control'})
        self.discover([state, control], proc)
        log = (Path(self.log_dir.name) / 'hope-transport-gateway-isolated-test.log')

        def one_packet_then_wait():
            state.publish(Float64MultiArray(data=[2., 123., 1.]))
            # A single packet can be <=1 s old on two consecutive 500 ms
            # evaluations. It must not qualify even when every endpoint exists.
            self.spin(1.3)

        def feed():
            for sequence in range(20):
                state.publish(Float64MultiArray(data=[2., 123., float(sequence)]))
                self.spin(.1)

        one_packet_then_wait()
        self.assertNotIn('TRANSPORT GATEWAY HEALTHY', log.read_text())
        feed()
        text = log.read_text()
        self.assertIn('TRANSPORT GATEWAY HEALTHY readiness=transport_only healthy_evaluations=2', text)
        healthy_count = text.count('TRANSPORT GATEWAY HEALTHY')
        self.spin(1.6)
        self.assertIn('TRANSPORT GATEWAY WAITING', log.read_text().splitlines()[-1])
        one_packet_then_wait()
        self.assertEqual(log.read_text().count('TRANSPORT GATEWAY HEALTHY'), healthy_count,
                         'source loss must reset the two-evaluation readiness gate')
        feed()
        self.assertGreater(log.read_text().count('TRANSPORT GATEWAY HEALTHY'), healthy_count)

    def test_runner_home_has_no_base_transport_dependency(self):
        from std_msgs.msg import Float64MultiArray
        received = []
        home = self.node.create_publisher(Float64MultiArray, '/cpu_native_test/kernel_home', 10)
        sub = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/kernel_home_out',
            lambda msg: received.append(list(msg.data)), 10)
        proc = self.start('hope-transport-gateway', {
            'remote_session_home_topic': '/cpu_native_test/kernel_home',
            'local_session_home_topic': '/cpu_native_test/kernel_home_out'})
        self.discover([home], proc)
        deadline = time.monotonic() + 5
        while ('hope_transport_gateway', '/') not in self.node.get_node_names_and_namespaces():
            self.assertLess(time.monotonic(), deadline, 'gateway node graph was not discovered')
            self.spin(.05)
        graph = self.node.get_subscriber_names_and_types_by_node('hope_transport_gateway', '/')
        self.assertFalse(any('base_pose' in topic for topic, _types in graph))
        home.publish(Float64MultiArray(data=[1., 5., 17.]))
        self.spin(.3)
        self.assertEqual(received, [[1., 5., 17.]])

    def test_remapped_self_loops_are_rejected(self):
        cases = (
            ('hope-transport-gateway', {}, {
                '/cpu_native_test/remote_state_topic': '/cpu_native_test/local_state_topic'}),
            ('hope-base-pose-transport-relay', {
                'input_topic': '/cpu_native_test/base_in', 'output_topic': '/cpu_native_test/base_out'}, {
                '/cpu_native_test/base_in': '/cpu_native_test/base_out'}),
        )
        for name, parameters, remaps in cases:
            with self.subTest(name=name):
                proc = self.start(name, parameters, remaps)
                self.assertEqual(proc.wait(timeout=8), 1, 'self-loop must fail before forwarding')

    def test_imu_rate_receipt_clock_and_source_loss(self):
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Bool, Float64MultiArray
        received = []
        freshness = []
        pub = self.node.create_publisher(Imu, '/cpu_native_test/imu', 10)
        sub = self.node.create_subscription(Float64MultiArray, '/cpu_native_test/sample',
            lambda msg: received.append((time.monotonic(), list(msg.data))), 10)
        fresh_sub = self.node.create_subscription(Bool, '/cpu_native_test/fresh',
            lambda msg: freshness.append(msg.data), 10)
        proc = self.start('hope-imu-telemetry', {
            'imu_topic': '/cpu_native_test/imu', 'sample_topic': '/cpu_native_test/sample',
            'publish_hz': 20.0, 'publish_clock_topics': 'true'}, {
            '/hope/clock/message_fresh': '/cpu_native_test/fresh',
            '/hope/clock/message_latency_ms': '/cpu_native_test/latency',
            '/hope/clock/message_text': '/cpu_native_test/text'})
        self.discover([pub], proc)
        self.assertEqual(received, [])
        self.assertFalse(freshness[-1], 'no IMU on startup must remain unready')
        start = time.monotonic()
        sent = 0
        while time.monotonic() - start < 1.2:
            msg = Imu()
            stamp = self.node.get_clock().now().nanoseconds - 10_000_000
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            pub.publish(msg)
            sent += 1
            self.ros.spin_once(self.node, timeout_sec=.002)
        self.spin(.2)
        self.assertGreater(len(received), 5)
        self.assertLessEqual(len(received), math.ceil((time.monotonic() - start) * 20) + 2)
        self.assertLess(len(received), sent)
        self.assertTrue(freshness[-1])
        for delivered, sample in received:
            self.assertEqual(sample[0], 1.)
            self.assertTrue(math.isfinite(sample[1]))
            self.assertLess(abs(sample[1]), 1000)
            self.assertGreater(sample[2], start)
            self.assertLessEqual(sample[2], delivered)
        count = len(received)
        last_receipt = received[-1][1][2]
        self.spin(.65)
        self.assertEqual(len(received), count, 'missing IMU must not be replayed')
        self.assertGreater(time.monotonic() - last_receipt, .5)
        self.assertFalse(freshness[-1], 'source loss must publish false freshness')
        pub.publish(Imu())
        self.spin(.2)
        self.assertEqual(len(received), count + 1)
        self.assertTrue(math.isnan(received[-1][1][1]), 'zero stamp must stay invalid')
        self.assertFalse(freshness[-1])


if __name__ == '__main__':
    unittest.main()
