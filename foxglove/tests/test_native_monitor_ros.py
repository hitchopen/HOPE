"""Native monitor integration tests: ONLY localhost domain 219, mocked backends.

Requires HOPE_NATIVE_BIN_DIR plus the monitor_rpc_mock test fixture (never robot
domain 232). No test calls any real vendor or Runner endpoint.
"""
import json
import math
import os
import re
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest


class NativeMonitorDeploymentTests(unittest.TestCase):
    """Repository checks run without ROS, a candidate build, or domain 219."""
    def test_unit_launches_native_not_reference_python(self):
        unit = Path(__file__).resolve().parents[1] / 'a3/hope-monitor.service'
        if not unit.exists():
            self.skipTest('repository unit not copied to this isolated test staging directory')
        text = unit.read_text()
        self.assertIn('exec /usr/local/libexec/hope-monitor', text)
        self.assertNotIn('hope_monitor.py', text)


@unittest.skipUnless(os.environ.get('ROS_LOCALHOST_ONLY') == '1'
                     and os.environ.get('ROS_DOMAIN_ID') == '219'
                     and os.environ.get('HOPE_NATIVE_BIN_DIR'),
                     'requires isolated localhost domain 219 and candidate binaries')
class NativeMonitorTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.ros = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('native_monitor_contract_test')
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.temp = tempfile.TemporaryDirectory(prefix='hope-monitor-ros-')
        self.process = None
        self.log = None
        self.received = {}
        self.inputs = []

    def tearDown(self):
        self.stop()
        if self.log:
            self.log.close()
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(5)
        self.node.destroy_node()
        self.ros.shutdown()
        self.temp.cleanup()

    def start(self, environment=None, **parameters):
        self.stop()
        args = [str(Path(os.environ['HOPE_NATIVE_BIN_DIR']) / 'hope-monitor'), '--ros-args']
        defaults = dict(enable_tf_fallback='false', native_clock_topics='true',
                        estop_latch_path=str(Path(self.temp.name) / 'latch'))
        defaults.update(parameters)
        for key, value in defaults.items():
            args += ['-p', f'{key}:={value}']
        self.log = open(Path(self.temp.name) / 'monitor.log', 'a')
        env = os.environ.copy()
        env.update(environment or {})
        self.process = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, env=env)
        self.until(lambda: any(name == 'hope_monitor' for name, _ in self.node.get_node_names_and_namespaces()))
        self.until(lambda: self.node.count_publishers('/hope/safety/estop_latched') == 1)
        time.sleep(.2)

    def stop(self):
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(8)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(5)
                self.fail('native monitor failed bounded shutdown')
            self.process = None
            time.sleep(.25)

    def until(self, predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            if self.process and self.process.poll() is not None:
                self.fail('monitor exited: ' + (Path(self.temp.name) / 'monitor.log').read_text())
            time.sleep(.01)
        log = Path(self.temp.name) / 'monitor.log'
        self.assertTrue(predicate(), 'condition timed out\n' + (log.read_text()[-12000:] if log.exists() else ''))

    def subscribe(self, cls, topic):
        result = self.received.setdefault(topic, [])
        self.inputs.append(self.node.create_subscription(cls, topic, result.append, 10,
                                                        callback_group=self.group))
        return result

    def call_async(self, topic):
        from std_srvs.srv import Trigger
        client = self.node.create_client(Trigger, topic, callback_group=self.group)
        self.inputs.append(client)
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        return client.call_async(Trigger.Request())

    def call(self, topic):
        future = self.call_async(topic)
        self.until(future.done)
        return future.result()

    def backends(self, vendor_code=0, runner_success=True, delay=0, *,
                 vendor_present=True, runner_present=True, runner_delay=0, wrapper_code=0):
        from ros2_plugin_proto.srv import RosRpcWrapper
        from std_srvs.srv import Trigger
        calls = {'vendor': [], 'runner': []}
        self.dispatch_times = {}
        def vendor(request, response):
            calls['vendor'].append(request)
            self.dispatch_times['vendor'] = time.monotonic()
            if delay:
                time.sleep(delay)
            response.code = wrapper_code
            response.serialization_type = 'pb'
            response.data = bytes([10, 0]) if vendor_code == 0 else bytes([10, 2, 8, vendor_code])
            return response
        def runner(request, response):
            calls['runner'].append(request)
            self.dispatch_times['runner'] = time.monotonic()
            if runner_delay:
                time.sleep(runner_delay)
            response.success = runner_success
            response.message = 'mock runner response'
            return response
        if vendor_present:
            self.inputs.append(self.node.create_service(RosRpcWrapper,
                '/aimdk_2Eprotocol_2EHalEmergencyService/SetEmergencyCommand', vendor, callback_group=self.group))
        if runner_present:
            self.inputs.append(self.node.create_service(Trigger, '/hope/runner/emergency_stop', runner, callback_group=self.group))
        return calls

    def test_graph_startup_health_and_no_unsolicited_command(self):
        from std_msgs.msg import Bool, Float64MultiArray, String
        modes = self.subscribe(Float64MultiArray, '/hope/runner/mode_command')
        latch = self.subscribe(Bool, '/hope/safety/estop_latched')
        ntp = self.subscribe(String, '/hope/ntp/qualification')
        cpu = self.subscribe(String, '/hope/system/cpu_text')
        calls = self.backends()
        self.start()
        self.until(lambda: len(ntp) >= 2 and cpu and latch)
        self.assertFalse(latch[-1].data)
        self.assertEqual(modes, [])
        self.assertEqual(calls, {'vendor': [], 'runner': []})
        first, last = json.loads(ntp[0].data), json.loads(ntp[-1].data)
        self.assertEqual(last['schema_version'], 1)
        self.assertEqual(first['clock_boot_id'], last['clock_boot_id'])
        self.assertGreater(last['sequence'], first['sequence'])
        self.assertGreater(last['observed_monotonic_s'], first['observed_monotonic_s'])
        self.assertLess(last['observed_monotonic_s'], time.monotonic())
        # Domain isolation does not isolate the host's read-only chrony socket:
        # the A3 may legitimately qualify while a development container cannot.
        self.assertIsInstance(last['gate_pass'], bool)
        graph = dict(self.node.get_topic_names_and_types())
        reference = Path(os.environ.get('HOPE_MONITOR_REFERENCE_SOURCE',
                         str(Path(__file__).resolve().parents[1] / 'a3/hope_monitor.py'))).read_text()
        publishers = re.findall(r'(?:display_publisher|create_publisher)\(\s*(?:self,\s*)?\w+,\s*"([^"]+)"', reference)
        for topic in publishers:
            if not topic.startswith('/hope/clock/'):
                self.assertIn(topic, graph, 'missing Python-compatible output: ' + topic)
        self.assertEqual(graph['/hope/pelvis/pose'], ['geometry_msgs/msg/PoseStamped'])
        self.assertNotIn('/hope/clock/message_fresh', graph)  # native clock owns this
        for topic in ('enter_prepare', 'enter_policy', 'exit_policy', 'enter_passive'):
            self.assertFalse(self.call('/hope/control/' + topic).success)
        self.assertEqual(modes, [])

    def test_pose_300hz_preserves_stamp_and_stales_without_replay(self):
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool
        from visualization_msgs.msg import Marker
        from tf2_msgs.msg import TFMessage
        poses = self.subscribe(PoseStamped, '/hope/pelvis/pose')
        readiness = self.subscribe(Bool, '/hope/vendor/tf_ready')
        markers = self.subscribe(Marker, '/hope/pelvis/marker')
        tfs = self.subscribe(TFMessage, '/hope/pelvis/tf')
        pub = self.node.create_publisher(PoseStamped, '/a3/mocap/pelvis_pose', 10)
        self.start()
        self.until(lambda: pub.get_subscription_count())
        sent = set()
        started = time.monotonic()
        for i in range(300):
            msg = PoseStamped()
            msg.header.frame_id = 'world'
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.z = 1.2, .8
            msg.pose.orientation.w = 1.
            sent.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
            pub.publish(msg)
            remaining = started + (i + 1) / 300 - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        self.until(lambda: poses and tfs and markers and readiness[-1].data)
        self.assertLessEqual(len(poses), math.ceil((time.monotonic() - started) * 5) + 2)
        self.assertLess(len(poses), len(sent))
        for msg in poses:
            self.assertIn((msg.header.stamp.sec, msg.header.stamp.nanosec), sent)
            self.assertEqual(msg.pose.position.x, 1.2)
        self.assertEqual({m.id for m in markers}, {0, 1})
        self.assertTrue(all(m.frame_locked for m in markers))
        self.assertTrue(all(m.transforms[0].child_frame_id == 'pelvis_link' for m in tfs))
        self.until(lambda: readiness and not readiness[-1].data, 2)
        count = len(poses)
        time.sleep(.5)
        self.assertEqual(len(poses), count)
        bad = PoseStamped(); bad.header.frame_id = 'wrong'; bad.pose.orientation.w = 1.0
        pub.publish(bad); time.sleep(.25)
        self.assertFalse(readiness[-1].data)

    def test_dual_estop_exact_assert_bytes_persistence_reassert_and_reset(self):
        from std_msgs.msg import Bool, Float64MultiArray
        latch = self.subscribe(Bool, '/hope/safety/estop_latched')
        full = self.subscribe(Bool, '/hope/safety/estop_full_ready')
        modes = self.subscribe(Float64MultiArray, '/hope/runner/mode_command')
        calls = self.backends()
        self.start()
        self.until(lambda: full and full[-1].data)
        result = self.call('/hope/safety/trigger_estop')
        self.assertTrue(result.success, result.message)
        self.assertIn('not independently confirmed', result.message)
        self.assertEqual(len(calls['runner']), 1)
        request = calls['vendor'][0]
        self.assertEqual(request.serialization_type, 'pb')
        self.assertEqual(list(request.context), ['aimdk.protocol.EmergencyCommandReq'])
        wire = bytes(v[0] if isinstance(v, bytes) else v for v in request.data)
        self.assertTrue(wire.endswith(bytes([18,2,16,1])))
        self.assertTrue((Path(self.temp.name) / 'latch').exists())
        self.assertIn('REASSERTED', self.call('/hope/safety/trigger_estop').message)
        before = len(latch)
        self.start()
        self.until(lambda: len(latch) > before and latch[-1].data)
        for action in ('enter_prepare', 'enter_policy', 'exit_policy', 'enter_passive'):
            self.assertFalse(self.call('/hope/control/' + action).success)
        n = len(calls['vendor'])
        self.assertTrue(self.call('/hope/safety/reset_software_estop').success)
        self.assertFalse((Path(self.temp.name) / 'latch').exists())
        self.assertEqual(len(calls['vendor']), n, 'reset must never release vendor stop')
        self.assertEqual(modes, [], 'reset must never request motion')

    def test_delayed_vendor_does_not_block_health_or_allow_reset(self):
        from std_msgs.msg import String
        calls = self.backends(delay=1.5)
        health = self.subscribe(String, '/hope/control/state_text')
        self.start()
        self.until(lambda: health)
        stop = self.call_async('/hope/safety/trigger_estop')
        self.until(lambda: calls['vendor'])
        count = len(health)
        reset = self.call('/hope/safety/reset_software_estop')
        self.assertFalse(reset.success)
        # Prove progress while the RPC is pending, with scheduling headroom for
        # a loaded ROS VM rather than a 100 ms margin before mock completion.
        self.until(lambda: len(health) > count, 1.)
        self.assertFalse(stop.done(), 'health must progress before the delayed RPC completes')
        self.until(stop.done)
        self.assertTrue(stop.result().success)

    def test_application_rejection_reports_partial_and_keeps_latch(self):
        calls = self.backends(vendor_code=17)
        self.start()
        time.sleep(.5)
        result = self.call('/hope/safety/trigger_estop')
        self.assertTrue(result.success)
        self.assertIn('PARTIAL', result.message)
        self.assertIn('not accepted', result.message)
        self.assertTrue(calls['runner'])
        self.assertTrue((Path(self.temp.name) / 'latch').exists())

    def test_runner_only_dispatch_does_not_wait_for_missing_vendor(self):
        from std_msgs.msg import Bool
        ready = self.subscribe(Bool, '/hope/safety/estop_ready')
        full = self.subscribe(Bool, '/hope/safety/estop_full_ready')
        calls = self.backends(vendor_present=False)
        self.start()
        self.until(lambda: ready and ready[-1].data and full and not full[-1].data)
        started = time.monotonic()
        future = self.call_async('/hope/safety/trigger_estop')
        self.until(lambda: calls['runner'], .4)
        self.assertLess(self.dispatch_times['runner'] - started, .4,
                        'Runner dispatch must precede the 500 ms vendor discovery grace')
        self.until(future.done)
        self.assertTrue(future.result().success)
        self.assertIn('PARTIAL', future.result().message)
        self.assertEqual(calls['vendor'], [])

    def test_vendor_timeout_is_partial_and_health_remains_live(self):
        from std_msgs.msg import String
        health = self.subscribe(String, '/hope/control/state_text')
        calls = self.backends(delay=2.4)
        self.start()
        self.until(lambda: health)
        future = self.call_async('/hope/safety/trigger_estop')
        self.until(lambda: calls['vendor'] and calls['runner'])
        before = len(health)
        # Leave scheduler headroom on a loaded development VM, but still
        # require progress well before the monitor's 2-second RPC timeout.
        self.until(lambda: len(health) > before, 1.)
        self.until(future.done, 3)
        self.assertTrue(future.result().success)
        self.assertIn('vendor call timed out', future.result().message)
        self.assertFalse(self.call('/hope/control/enter_policy').success)
        time.sleep(.5)  # late mock response must not clear the inhibit
        self.assertTrue((Path(self.temp.name) / 'latch').exists())

    def test_runner_rejection_timeout_and_wrapper_error_have_details(self):
        # Each iteration is a new process and mock graph, not a real backend.
        for success, delay, wrapper, expected in (
                (False, 0, 0, 'mock runner response'),
                (True, 3.1, 0, 'managed runner stop timed out'),
                (True, 0, 23, 'code=23')):
            with self.subTest(expected=expected):
                calls = self.backends(runner_success=success, runner_delay=delay, wrapper_code=wrapper)
                self.start()
                time.sleep(.5)
                result = self.call('/hope/safety/trigger_estop')
                self.assertTrue(result.success, result.message)
                self.assertIn('PARTIAL', result.message)
                self.assertIn(expected, result.message)
                self.assertTrue(calls['vendor'] and calls['runner'])
                self.stop()
                time.sleep(max(0, delay - 2.5))
                for entity in list(self.inputs):
                    if isinstance(entity, self.ros.service.Service):
                        self.node.destroy_service(entity)
                        self.inputs.remove(entity)
                (Path(self.temp.name) / 'latch').unlink()

    def test_concurrent_assertions_and_reset_serialize_without_duplicate_calls(self):
        calls = self.backends(delay=.7)
        self.start()
        time.sleep(.5)
        first = self.call_async('/hope/safety/trigger_estop')
        self.until(lambda: calls['vendor'])
        second = self.call('/hope/safety/trigger_estop')
        self.assertFalse(second.success)
        self.assertIn('in progress', second.message)
        self.assertFalse(self.call('/hope/safety/reset_software_estop').success)
        self.until(first.done)
        self.assertTrue(first.result().success)
        self.assertEqual(len(calls['vendor']), 1)
        self.assertEqual(len(calls['runner']), 1)

    def test_assertion_during_reset_waits_then_asserts_instead_of_being_dropped(self):
        from std_msgs.msg import Bool, Float64MultiArray
        fixture = Path(os.environ['HOPE_NATIVE_BIN_DIR']) / 'libhope-monitor-probe-faults.so'
        self.assertTrue(fixture.exists())
        path = Path(self.temp.name) / 'latch'
        release = Path(self.temp.name) / 'release_unlink'
        path.write_text('asserted_utc_ns=42\n')
        latch = self.subscribe(Bool, '/hope/safety/estop_latched')
        modes = self.subscribe(Float64MultiArray, '/hope/runner/mode_command')
        calls = self.backends(delay=.3)
        self.start(environment={'LD_PRELOAD': str(fixture), 'HOPE_TEST_SLOW_UNLINK_PATH': str(path),
                                'HOPE_TEST_UNLINK_RELEASE_FILE': str(release)})
        self.until(lambda: latch and latch[-1].data)
        reset = self.call_async('/hope/safety/reset_software_estop')
        self.until(lambda: Path(str(path) + '.unlink_entered').exists())
        try:
            triggers = [self.call_async('/hope/safety/trigger_estop') for _ in range(2)]
            self.until(lambda: any(f.done() for f in triggers), 1.)
            time.sleep(.1)
            # Either callback can run first. Exactly one owns the assertion flag
            # and waits behind reset; only the duplicate is refused.
            completed = [f for f in triggers if f.done()]
            pending = [f for f in triggers if not f.done()]
            self.assertEqual(len(completed), 1, 'a reset must not drop both assertions')
            self.assertEqual(len(pending), 1)
            self.assertFalse(completed[0].result().success)
            self.assertIn('assertion already in progress', completed[0].result().message)
            self.assertFalse(reset.done(), 'test must hold reset before unlink completion')
            self.assertEqual(calls, {'vendor': [], 'runner': []})
            release.touch()
            self.until(reset.done)
            self.assertTrue(reset.result().success)
            self.until(pending[0].done)
            self.assertTrue(pending[0].result().success, pending[0].result().message)
            self.until(lambda: latch[-1].data)
            self.assertTrue(path.exists(), 'new assertion must restore the durable latch')
            self.assertEqual(len(calls['runner']), 1)
            self.assertEqual(len(calls['vendor']), 1)
            self.assertEqual(modes, [], 'reset/assert concurrency must never request motion')
        finally:
            release.touch()  # Never leave the test-only reset callback stalled.

    def test_monitor_global_network_reader_matches_previous_ip_projection(self):
        driver = Path(os.environ['HOPE_NATIVE_BIN_DIR']) / 'hope-monitor-network-test-driver'
        self.assertTrue(driver.exists())
        lines = subprocess.check_output([str(driver)], text=True, timeout=3).splitlines()
        actual = [(int(index), name, ip) for index, name, ip in (line.split('\t') for line in lines)]
        original = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show'], text=True, timeout=3))
        expected = sorted({(item['ifindex'], item['ifname'], a['local'])
                           for item in original for a in item.get('addr_info', [])
                           if item['ifname'] != 'lo' and a.get('family') == 'inet' and a.get('scope') == 'global'})
        self.assertEqual(actual, expected)

    def test_slow_storage_and_busy_callbacks_do_not_precede_stop_dispatch(self):
        from std_msgs.msg import Float64MultiArray, String
        fixture = Path(os.environ['HOPE_NATIVE_BIN_DIR']) / 'libhope-monitor-probe-faults.so'
        self.assertTrue(fixture.exists(), 'build the maintained test-only fault fixture')
        calls = self.backends()
        modes = self.subscribe(Float64MultiArray, '/hope/runner/mode_command')
        health = self.subscribe(String, '/hope/control/state_text')
        state = self.node.create_publisher(Float64MultiArray, '/hope/runner/mode_state', 10)
        base = self.node.create_publisher(Float64MultiArray, '/a3/base_pose_flat', 10)
        self.start(environment={'LD_PRELOAD': str(fixture), 'HOPE_TEST_SLOW_SYNC_DIR': self.temp.name})
        self.until(lambda: health and state.get_subscription_count())
        running = threading.Event(); running.set()
        def flood():
            while running.is_set():
                state.publish(Float64MultiArray(data=[1., 0., 0., 0., 0., 0., 1., 0.]))
                base.publish(Float64MultiArray(data=[2., 1.] + [0.] * 12 + [1., 0.]))
                time.sleep(.001)
        thread = threading.Thread(target=flood, daemon=True); thread.start()
        try:
            started = time.monotonic()
            future = self.call_async('/hope/safety/trigger_estop')
            marker = Path(self.temp.name) / 'sync_entered'
            self.until(marker.exists)
            self.until(lambda: calls['runner'] and calls['vendor'], .3)
            self.assertLess(max(self.dispatch_times.values()) - started, .5)
            self.assertFalse(future.done(), 'slow fsync must still be in progress')
            before = len(health)
            for action in ('enter_prepare', 'enter_policy', 'exit_policy', 'enter_passive'):
                result = self.call('/hope/control/' + action)
                self.assertFalse(result.success)
                self.assertIn('latched', result.message)
            self.assertFalse(self.call('/hope/safety/reset_software_estop').success)
            self.until(lambda: len(health) > before, .6)
            self.until(future.done)
            self.assertTrue(future.result().success)
            self.assertEqual(modes, [])
        finally:
            running.clear(); thread.join(3)

    def test_cpu_probe_failure_is_unavailable_not_warming_and_does_not_restart(self):
        from std_msgs.msg import String
        fixture = Path(os.environ['HOPE_NATIVE_BIN_DIR']) / 'libhope-monitor-probe-faults.so'
        self.assertTrue(fixture.exists())
        cpu = self.subscribe(String, '/hope/system/cpu_text')
        top = self.subscribe(String, '/hope/system/cpu_top_process')
        self.start(environment={'LD_PRELOAD': str(fixture), 'HOPE_TEST_FAIL_PROC_STAT': '1'})
        pid = self.process.pid
        self.until(lambda: len(cpu) >= 2 and top)
        self.assertIn('UNAVAILABLE', cpu[-1].data)
        self.assertEqual(top[-1].data, 'TOP CPU PROCESS UNAVAILABLE')
        self.assertEqual(self.process.pid, pid)

    def test_runtime_parameters_take_effect_and_startup_parameters_refuse_changes(self):
        from geometry_msgs.msg import PoseStamped
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        from std_msgs.msg import Bool
        ready = self.subscribe(Bool, '/hope/vendor/tf_ready')
        poses = self.subscribe(PoseStamped, '/hope/pelvis/pose')
        pub = self.node.create_publisher(PoseStamped, '/a3/mocap/pelvis_pose', 10)
        self.start()
        self.until(lambda: pub.get_subscription_count())
        client = self.node.create_client(SetParametersAtomically, '/hope_monitor/set_parameters_atomically', callback_group=self.group)
        self.inputs.append(client)
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        def set_parameters(**values):
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]
            future = client.call_async(request); self.until(future.done)
            return future.result().result
        self.assertFalse(set_parameters(native_clock_topics=False).successful)
        self.assertFalse(set_parameters(runner_state_stale_after_s=-1.).successful)
        self.assertTrue(set_parameters(reference_frame='test_world', pelvis_frame='test_pelvis', tf_stale_after_s=.15).successful)
        msg = PoseStamped(); msg.header.frame_id = 'test_world'
        msg.header.stamp = self.node.get_clock().now().to_msg(); msg.pose.orientation.w = 1.0
        pub.publish(msg)
        # Avoid sampling around the 200 ms output tick with a 150 ms limit.
        end = time.monotonic() + .6
        while time.monotonic() < end:
            msg.header.stamp = self.node.get_clock().now().to_msg(); pub.publish(msg); time.sleep(.02)
        self.until(lambda: poses and ready[-1].data)
        self.assertEqual(poses[-1].header.frame_id, 'test_world')
        self.until(lambda: not ready[-1].data, .6)

    def test_native_latency_keeps_source_receipt_time_and_invalid_stays_invalid(self):
        from std_msgs.msg import Bool, Float64MultiArray
        fresh = self.subscribe(Bool, '/hope/clock/message_fresh')
        pub = self.node.create_publisher(Float64MultiArray, '/monitor_test/latency', 10)
        self.start(native_clock_topics='false', message_latency_sample_topic='/monitor_test/latency')
        self.until(lambda: pub.get_subscription_count() and fresh)
        self.assertFalse(fresh[-1].data)
        pub.publish(Float64MultiArray(data=[1., 2.5, time.monotonic()]))
        self.until(lambda: fresh[-1].data)
        self.until(lambda: not fresh[-1].data, 2)
        pub.publish(Float64MultiArray(data=[1., 2.5, time.monotonic() - 1]))
        time.sleep(.2)
        self.assertFalse(fresh[-1].data, 'queued old receipt must not become fresh')
        pub.publish(Float64MultiArray(data=[1., math.nan, time.monotonic()]))
        time.sleep(.2)
        self.assertFalse(fresh[-1].data)

    def test_failed_backends_still_expose_assert_service_and_persist_inhibit(self):
        from std_msgs.msg import Bool
        ready = self.subscribe(Bool, '/hope/safety/estop_ready')
        self.start()
        self.until(lambda: ready)
        self.assertFalse(ready[-1].data)
        result = self.call('/hope/safety/trigger_estop')
        self.assertFalse(result.success)
        self.assertIn('UNCONFIRMED', result.message)
        self.assertTrue((Path(self.temp.name) / 'latch').exists())

    def test_persistence_and_reset_failure_never_hide_inhibit(self):
        from std_msgs.msg import Bool
        latch = self.subscribe(Bool, '/hope/safety/estop_latched')
        calls = self.backends()
        path = Path(self.temp.name) / 'latch'
        path.mkdir()
        self.start()
        self.until(lambda: latch and latch[-1].data)
        result = self.call('/hope/safety/trigger_estop')
        self.assertFalse(result.success)
        self.assertIn('PERSISTENCE FAILED', result.message)
        self.assertTrue(calls['vendor'] and calls['runner'])
        self.assertFalse(self.call('/hope/safety/reset_software_estop').success)
        self.assertTrue(latch[-1].data)

    def test_legacy_prepare_requires_ack_and_matching_calibration_before_motion(self):
        from std_msgs.msg import Bool, Float64MultiArray
        from std_srvs.srv import Trigger
        modes = self.subscribe(Float64MultiArray, '/hope/runner/mode_command')
        ready = self.subscribe(Bool, '/hope/control/calibration_ready')
        state = self.node.create_publisher(Float64MultiArray, '/hope/runner/mode_state', 10)
        base = self.node.create_publisher(Float64MultiArray, '/a3/base_pose_flat', 10)
        calls = []
        sha = 'a' * 64
        def calibrate(request, response):
            calls.append(request)
            response.success, response.message = True, sha
            return response
        self.inputs.append(self.node.create_service(Trigger, '/a3/calibration/recompute_ucb_robot_v2',
                                                   calibrate, callback_group=self.group))
        self.start()
        self.until(lambda: state.get_subscription_count() and base.get_subscription_count())
        def publish_state(mode, pd, seq):
            state.publish(Float64MultiArray(data=[1., float(mode), float(pd), float(seq), float(seq), 1., 1., 0.]))
            time.sleep(.05)
        publish_state(0, False, 0)
        self.assertFalse(self.call('/hope/control/enter_policy').success)
        self.assertTrue(self.call('/hope/control/enter_prepare').success)
        self.until(lambda: modes)
        seq = int(modes[-1].data[1])
        self.assertEqual(modes[-1].data[2], 1.)
        self.assertEqual(calls, [], 'calibration must wait for PD_STAND acknowledgement')
        publish_state(1, True, seq)
        self.until(lambda: calls)
        packet = [0.] * 16
        packet[0:2] = [2., 1.]
        packet[14] = float(int(sha[:13], 16))
        self.assertFalse(self.call('/hope/control/enter_policy').success)
        for _ in range(10):
            base.publish(Float64MultiArray(data=packet))
            publish_state(1, True, seq)
            if ready and ready[-1].data:
                break
        self.until(lambda: ready and ready[-1].data)
        self.assertTrue(self.call('/hope/control/enter_policy').success)
        self.until(lambda: modes[-1].data[2] == 2.)
        self.assertTrue(self.call('/hope/control/exit_policy').success)
        self.until(lambda: modes[-1].data[2] == 1.)
        self.assertTrue(self.call('/hope/control/enter_passive').success)
        self.until(lambda: modes[-1].data[2] == 0.)


if __name__ == '__main__':
    unittest.main()
