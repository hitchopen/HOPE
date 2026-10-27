"""Native config transport contracts: ONLY localhost domain 229, no robot access.

Requires HOPE_NATIVE_BIN_DIR. The deployment-file check runs without ROS.
Network-change tests use a non-installed driver linked to the actual relay core;
they never change host interfaces or publish to a production ROS domain.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest


class MocapConfigDeploymentTests(unittest.TestCase):
    def test_deployment_uses_native_not_python(self):
        root = Path(__file__).resolve().parents[1]
        unit = root / 'a3/hope-mocap-config-transport.service'
        if not unit.exists():
            self.skipTest('repository unit not present in isolated test staging')
        text = unit.read_text()
        self.assertIn('ExecStart=:/bin/bash', text)
        self.assertIn('exec /home/agi/hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh', text)
        self.assertIn('-- /usr/local/libexec/hope-mocap-config-transport', text)
        self.assertIn('EnvironmentFile=-/etc/hope-foxglove/network.env', text)
        self.assertIn('${ROS_STATIC_PEERS:-}', text)
        self.assertNotIn('hope_mocap_config_transport.py', text)
        self.assertFalse((root / 'a3/hope_mocap_config_transport.py').exists())


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '229'
                     and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
                     and os.environ.get('HOPE_NATIVE_BIN_DIR'),
                     'requires isolated localhost domain 229 and candidate binaries')
class NativeMocapConfigTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from std_msgs.msg import String
        self.ros, self.String = rclpy, String
        rclpy.init(args=[])
        self.node = rclpy.create_node('mocap_config_contract_test')
        self.temp = tempfile.TemporaryDirectory(prefix='hope-config-transport-test-')
        self.process = None
        self.logs, self.subs = [], []
        self.values = []
        self.publisher = self.node.create_publisher(String, '/hope/lifecycle/config/mocap', 10)
        self.subs.append(self.node.create_subscription(String, '/hope/lifecycle/config/mocap_transport',
                                                      lambda msg: self.values.append(msg.data), 10))

    def tearDown(self):
        self.stop()
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        self.ros.shutdown()
        self.temp.cleanup()

    def stop(self):
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(5)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(5)
                self.fail('config transport did not shut down promptly')
            self.process = None

    def start(self, driver=False, remaps=None, discover=True):
        self.stop()
        binary = 'hope-mocap-config-transport' + ('-test-driver' if driver else '')
        executable = Path(os.environ['HOPE_NATIVE_BIN_DIR']) / binary
        self.assertTrue(executable.exists(), 'build candidate and maintained test driver first')
        args = [str(executable), '--ros-args']
        for source, target in (remaps or {}).items():
            args += ['-r', f'{source}:={target}']
        env = os.environ.copy()
        env['HOPE_TEST_MOCAP_ADDRESS_FILE'] = str(Path(self.temp.name) / 'addresses')
        self.log_file = Path(self.temp.name) / f'{len(self.logs)}.log'
        log = self.log_file.open('w'); self.logs.append(log)
        self.process = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT)
        if discover:
            self.until(lambda: self.publisher.get_subscription_count() == 1
                       and self.node.count_publishers('/hope/lifecycle/config/mocap_transport') == 1)
            self.spin(.2)

    def until(self, predicate, seconds=8, allow_exit=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return
            if not allow_exit and self.process and self.process.poll() is not None:
                self.fail('config transport exited: ' + self.log_file.read_text())
            self.ros.spin_once(self.node, timeout_sec=.01)
        self.assertTrue(predicate(), 'condition timed out: ' + self.log_file.read_text())

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ros.spin_once(self.node, timeout_sec=.01)

    def addresses(self, text):
        (Path(self.temp.name) / 'addresses').write_text(text)

    def test_original_strings_order_qos_graph_and_no_cached_heartbeat(self):
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
        self.start()
        self.assertEqual(self.values, [], 'startup must not fabricate a config')
        snapshots = [
            '{"revision":39,"state":"RUNNING","busy":false,"pants_profile":"v2"}',
            ' {\n "revision": 40, "label": "青瞳 / CMTracker", "old_timestamp": 1.25\n} ',
            '{not valid JSON: the transport must not parse or repair it}',
            '{"padding":"' + 'x' * 65536 + '"}',
        ]
        for index, data in enumerate(snapshots):
            self.publisher.publish(self.String(data=data))
            self.until(lambda: len(self.values) == index + 1)
        self.assertEqual(self.values, snapshots)
        self.spin(1.3)
        self.assertEqual(self.values, snapshots, 'input loss must not become a fresh heartbeat')
        info = [i for i in self.node.get_subscriptions_info_by_topic('/hope/lifecycle/config/mocap')
                if i.node_name == 'hope_mocap_config_transport']
        self.assertEqual(len(info), 1)
        self.assertEqual(info[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(info[0].qos_profile.durability, DurabilityPolicy.VOLATILE)
        # Fast DDS graph discovery reports depth=0 (unknown) on some versions;
        # it does not expose the remote endpoint's configured KEEP_LAST depth.
        self.assertIn(info[0].qos_profile.depth, (0, 10))
        services = [name for name, _ in self.node.get_service_names_and_types()
                    if name.startswith('/hope_mocap_config_transport/')]
        # Jazzy can expose a framework type-description query even with parameter
        # services disabled. It is read-only, not a configuration/control API.
        self.assertTrue(all(name.endswith('/get_type_description') for name in services), services)

    def test_restart_and_late_subscriber_do_not_replay_old_config(self):
        self.start()
        self.publisher.publish(self.String(data='original'))
        self.until(lambda: self.values == ['original'])
        late = []
        self.subs.append(self.node.create_subscription(self.String, '/hope/lifecycle/config/mocap_transport',
                                                      lambda msg: late.append(msg.data), 10))
        self.spin(.5)
        self.assertEqual(late, [], 'volatile relay must not cache for late subscribers')
        self.start()
        self.spin(1.2)
        self.assertEqual(self.values, ['original'])
        self.assertEqual(late, [])
        self.publisher.publish(self.String(data='new snapshot'))
        self.until(lambda: self.values == ['original', 'new snapshot'] and late == ['new snapshot'])

    def test_network_change_exits_cleanly_and_fresh_process_does_not_replay(self):
        self.addresses('eth_test 10.1.1.1\nwlan_test 192.0.2.2\n')
        self.start(driver=True)
        self.publisher.publish(self.String(data='before change'))
        self.until(lambda: self.values == ['before change'])
        # The former implementation sorted its snapshot; order is not a change.
        self.addresses('wlan_test 192.0.2.2\neth_test 10.1.1.1\n')
        self.spin(1.2)
        self.assertIsNone(self.process.poll())
        self.addresses('eth_test 10.1.1.2\nwlan_test 192.0.2.2\n')
        self.until(lambda: self.process.poll() is not None, 3, allow_exit=True)
        self.assertEqual(self.process.returncode, 1, self.log_file.read_text())
        self.assertIn('Network addresses changed', self.log_file.read_text())
        self.start(driver=True)
        self.spin(1.2)
        self.assertEqual(self.values, ['before change'])
        self.publisher.publish(self.String(data='after change'))
        self.until(lambda: self.values == ['before change', 'after change'])

    def test_network_probe_failure_exits_without_abort_or_synthetic_output(self):
        self.addresses('eth_test 10.1.1.1\n')
        self.start(driver=True)
        self.addresses('ERROR probe\n')
        self.until(lambda: self.process.poll() is not None, 3, allow_exit=True)
        self.assertEqual(self.process.returncode, 1)
        self.assertIn('Test address probe failure', self.log_file.read_text())
        self.assertEqual(self.values, [])
        self.start(driver=True, discover=False)
        self.until(lambda: self.process.poll() is not None, 3, allow_exit=True)
        self.assertEqual(self.process.returncode, 1)

    def test_topic_alias_loop_is_refused(self):
        self.start(remaps={'/hope/lifecycle/config/mocap': '/hope/lifecycle/config/mocap_transport'}, discover=False)
        self.until(lambda: self.process.poll() is not None, 3, allow_exit=True)
        self.assertEqual(self.process.returncode, 1)
        self.assertIn('must remain distinct', self.log_file.read_text())
        self.assertEqual(self.values, [])

    def test_kernel_address_snapshot_matches_previous_ip_projection(self):
        driver = Path(os.environ['HOPE_NATIVE_BIN_DIR']) / 'hope-mocap-config-transport-test-driver'
        native = subprocess.check_output([str(driver), '--dump-addresses'], text=True, timeout=3)
        actual = [tuple(line.split('\t')) for line in native.splitlines()]
        original = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show'], text=True, timeout=3))
        expected = sorted((item['ifname'], address['local'])
                          for item in original for address in item.get('addr_info', [])
                          if item['ifname'] != 'lo')
        self.assertEqual(actual, expected)


if __name__ == '__main__':
    unittest.main()
