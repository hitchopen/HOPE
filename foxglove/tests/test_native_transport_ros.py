"""Isolated DDS contracts for the two native tools; never uses robot domain.

Run with ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 and HOPE_NATIVE_BIN_DIR.
All topics additionally use /cpu_native_test, including command-like payloads.
"""
import math
import os
from pathlib import Path
import subprocess
import struct
import time
import unittest


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
        self.processes = []
        self.logs = []

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
        self.node.destroy_node()
        self.ros.shutdown()

    def start(self, name, parameters, remaps=None):
        output = open('/tmp/' + name + '-isolated-test.log', 'w')
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

    def discover(self, pubs, proc):
        end = time.monotonic() + 8
        while time.monotonic() < end and not all(p.get_subscription_count() for p in pubs):
            self.assertIsNone(proc.poll(), 'native tool exited; inspect isolated-test log')
            self.spin(.05)
        self.assertTrue(all(p.get_subscription_count() for p in pubs))
        self.spin(.3)

    def test_four_routes_preserve_wire_and_never_replay(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayDimension
        def payload_fields(msg):
            # CDR alignment padding is not initialized consistently by the
            # middleware. Compare every actual field and IEEE float bit instead.
            return (msg.layout.data_offset,
                    tuple((d.label, d.size, d.stride) for d in msg.layout.dim),
                    struct.pack('<' + 'd' * len(msg.data), *msg.data))
        parameters = {}
        input_names = ['remote_state_topic', 'local_control_topic',
                       'local_teleop_input_topic', 'remote_teleop_state_topic']
        output_names = ['local_state_topic', 'remote_control_topic',
                        'remote_teleop_input_topic', 'local_teleop_state_topic']
        received = [[] for _ in range(4)]
        pubs, subs = [], []
        for route in range(4):
            source, target = f'/cpu_native_test/in{route}', f'/cpu_native_test/out{route}'
            parameters[input_names[route]] = source
            parameters[output_names[route]] = target
            pubs.append(self.node.create_publisher(Float64MultiArray, source, 10))
            subs.append(self.node.create_subscription(Float64MultiArray, target,
                lambda msg, i=route: received[i].append(payload_fields(msg)), 10))
        proc = self.start('hope-runner-transport-relay-native', parameters)
        self.discover(pubs, proc)
        self.assertEqual(received, [[], [], [], []], 'startup must not generate a command')
        expected = []
        for route, pub in enumerate(pubs):
            # Includes an old source timestamp, disabled input, signed zero,
            # negative numbers and layout fields; relay must not normalize any.
            msg = Float64MultiArray(data=[1., 42., 7., float(route + 1), 1., 1., 0., -0., -2.5, 0., 1.])
            msg.layout.data_offset = route
            msg.layout.dim = [MultiArrayDimension(label='identity', size=11, stride=11)]
            expected.append(payload_fields(msg))
            pub.publish(msg)
        self.spin(.6)
        self.assertEqual(received, [[value] for value in expected])
        self.spin(.7)
        self.assertEqual(received, [[value] for value in expected], 'no timer replay after input stops')

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
