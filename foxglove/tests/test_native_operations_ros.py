"""Native ROS + non-ROS worker contracts on isolated localhost domain 219.

Backend fixture substitutes all SSH/hardware mutations. Never run on robot DDS.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from operations_processes import child_pids

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "a3"))


@unittest.skipUnless(os.environ.get("ROS_LOCALHOST_ONLY") == "1"
                     and os.environ.get("ROS_DOMAIN_ID") == "219"
                     and os.environ.get("HOPE_NATIVE_BIN_DIR"),
                     "requires isolated localhost domain 219 and native binary")
class NativeOperationsTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Float64MultiArray, String, Bool, Float64, UInt64
        from std_srvs.srv import Trigger
        self.ros = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node("operations_contract_fixture")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_stop = threading.Event()
        def spin():
            while not self.spin_stop.is_set():
                self.executor.spin_once(timeout_sec=.05)
        self.thread = threading.Thread(target=spin, daemon=True)
        self.thread.start()
        self.temp = tempfile.TemporaryDirectory(prefix="hope-operations-test-")
        self.root = Path(self.temp.name)
        startup_test = self._testMethodName.startswith("test_startup_")
        if startup_test:
            (self.root / "hold_startup").touch()
        self.output = (self.root / "native.log").open("w")
        self.status = []
        self.modes = []
        self.base_fresh = []
        self.base_age = []
        self.lifecycle = []
        self.results = []
        self.fields = []
        self.player_publishing = False
        self.markers = []
        self.ball_rates = []
        self.ball_live = []
        self.requests = []
        self.telemetry_faults = []
        self.telemetry_fault_counts = []
        self.metrics = []
        self.sequence = 0
        self.mode = 0
        self.ack = (0, 0, 0, 0)
        self.accept = True
        self.accept_emergency = True
        self.reject = False
        self.publishing = True
        self.subs = [
            self.node.create_subscription(String, "/hope/operations/status", lambda m: self.status.append(m.data), 10),
            self.node.create_subscription(String, "/hope/operations/metrics", lambda m: self.metrics.append(json.loads(m.data)), 10),
            self.node.create_subscription(String, "/hope/runner/mode", lambda m: self.modes.append(m.data), 10),
            self.node.create_subscription(Bool, "/hope/base/fresh", lambda m: self.base_fresh.append(m.data), 10),
            self.node.create_subscription(Float64, "/hope/base/receipt_age_ms", lambda m: self.base_age.append(m.data), 10),
            self.node.create_subscription(String, "/hope/lifecycle/state", lambda m: self.lifecycle.append(m.data), 10),
            self.node.create_subscription(String, "/hope/lifecycle/last_result", lambda m: self.results.append(m.data), 10),
            self.node.create_subscription(String, "/hope/field/status", lambda m: self.fields.append(json.loads(m.data)), 10),
            self.node.create_subscription(String, "/hope/operations/telemetry_fault", lambda m: self.telemetry_faults.append(m.data), 10),
            self.node.create_subscription(UInt64, "/hope/operations/telemetry_fault_count", lambda m: self.telemetry_fault_counts.append(m.data), 10),
            self.node.create_subscription(Float64MultiArray, "/hope/runner/control_request_hdu_flat", self.on_action, 10),
        ]
        self.base_pub = self.node.create_publisher(Float64MultiArray, "/a3/base_pose_flat", 10)
        from geometry_msgs.msg import PoseArray, PoseStamped
        from visualization_msgs.msg import Marker
        self.ball_pub = self.node.create_publisher(PoseArray, "/poses", 10)
        self.player_pubs = {name: self.node.create_publisher(PoseStamped,
            "/hope/mocap/players/" + name, 10) for name in ("UCB_P1", "UCB_P2")}
        self.subs.extend([
            self.node.create_subscription(Marker, "/hope/ball/marker", lambda m: self.markers.append(m), 10),
            self.node.create_subscription(Float64, "/hope/ball/rate_hz", lambda m: self.ball_rates.append(m.data), 10),
            self.node.create_subscription(Bool, "/hope/ball/live", lambda m: self.ball_live.append(m.data), 10),
        ])
        self.runner_pub = self.node.create_publisher(Float64MultiArray, "/hope/runner/state_hdu_flat", 10)
        self.timer = self.node.create_timer(.02, self.runner)
        # The reconnect watcher samples a cached OS probe at 1 Hz. Allow that
        # observation and DDS delivery without racing a two-second fake ACK
        # timeout. This changes only these fixtures, not production deadlines.
        reconnect_test = self._testMethodName in {
            "test_idle_network_change_waits_for_unacknowledged_command_and_urgent_rpc",
            "test_idle_reconnect_waits_for_native_emergency_alone",
        }
        action_timeout = "6.0" if reconnect_test else (
            "4.0" if "emergency_drain" in self._testMethodName else "2.0")
        args = [str(Path(os.environ["HOPE_NATIVE_BIN_DIR"]) / "hope-operations"), "--ros-args",
                "-p", "worker_path:=" + str(ROOT / "tests/fixtures/operations_worker.py"),
                "-p", "session_id_path:=" + str(self.root / "session"),
                "-p", "runtime_root:=" + str(self.root / "runtime"),
                "-p", "worker_rpc_timeout_s:=8.0",
                "-p", "runner_action_timeout_s:=" + action_timeout]
        env = dict(os.environ, HOPE_OPERATIONS_TEST_ROOT=str(self.root))
        if getattr(self, "reference", False):
            args = [sys.executable, str(ROOT / "tests/fixtures/operations_worker.py")] + args[1:]
            env["HOPE_OPERATIONS_TEST_REFERENCE"] = "1"
        self.process = subprocess.Popen(args, env=env, stdout=self.output, stderr=subprocess.STDOUT)
        self.addCleanup(self.cleanup)
        if startup_test:
            self.wait(lambda: (self.root / "startup_waiting").exists(), 15)
        else:
            self.wait(lambda: any(s.startswith("READY") for s in self.status), 15)
        self.Trigger = Trigger

    def cleanup(self):
        self.publishing = False
        self.timer.cancel()
        self.spin_stop.set()
        self.thread.join(timeout=5)
        self.executor.shutdown(timeout_sec=5)
        self.process.terminate()
        try:
            self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.output.close()
        self.node.destroy_node()
        self.ros.shutdown()
        self.temp.cleanup()

    def wait(self, predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process.poll() is not None:
                raise AssertionError((self.root / "native.log").read_text())
            time.sleep(.01)
        raise AssertionError("timed out\n" + (self.root / "native.log").read_text()[-6000:])

    def runner(self):
        from std_msgs.msg import Float64MultiArray
        from hope_runner_control_core import runner_session_fingerprint
        if not self.publishing:
            return
        if self.player_publishing:
            self.publish_player("UCB_P2", -.5)
        self.sequence += 1
        session = (self.root / "session").read_text() if (self.root / "session").exists() else "none"
        req, action, result, reason = self.ack
        values = [2, 42, self.sequence, self.mode, 0, 1, 0, 1, 0, 1, 0, 0,
                  1, 0, 0, 0, req, action, result, reason, runner_session_fingerprint(session)]
        self.runner_pub.publish(Float64MultiArray(data=[float(x) for x in values]))

    def wait_reconnect_deferred(self, pending):
        def observed():
            self.assertIsNone(self.process.poll(), "reconnect dropped an unacknowledged command")
            self.assertFalse(pending.done(), "command ended before the test supplied ACK/supersession")
            return any(s.startswith("DDS_RECONNECT_DEFERRED") for s in self.status)
        # One cached probe and one watcher cycle, plus IPC/DDS scheduling.
        # Check retention throughout the wait, not only the displayed status.
        self.wait(observed, 3.0)

    def publish_player(self, name, x):
        from geometry_msgs.msg import PoseStamped
        message = PoseStamped()
        message.header.frame_id = "hope_table_global"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.z = x, 1.1
        message.pose.orientation.w = 1.
        self.player_pubs[name].publish(message)

    def player_stats(self):
        try:
            return json.loads((self.root / "player_stats.json").read_text())
        except (OSError, ValueError):
            return {}

    def on_action(self, message):
        values = list(message.data)
        self.requests.append(values)
        req, action = int(values[1]), int(values[2])
        if action == 5 and not self.accept_emergency:
            return
        if not self.accept and action != 5:
            return
        if self.reject:
            self.ack = req, action, 4, 23
        else:
            self.ack = req, action, 1, 3
            if action == 3:
                self.mode = 1
            elif action == 4:
                self.mode = 3
            elif action == 5:
                self.mode = 0
            elif action == 7:
                self.mode = 5
        self.runner()

    def trigger(self, path, wait=True):
        client = self.node.create_client(self.Trigger, path)
        self.assertTrue(client.wait_for_service(timeout_sec=5), path)
        future = client.call_async(self.Trigger.Request())
        if not wait:
            return future
        self.wait(future.done, 7)
        return future.result()

    def test_one_ros_node_fixed_actions_truthful_ack_and_no_startup_command(self):
        self.assertEqual(self.requests, [])
        names = self.node.get_node_names()
        self.assertIn("hope_operations", names)
        self.assertNotIn("hope_command_proxy", names)
        self.assertNotIn("hope_lifecycle_supervisor", names)
        self.assertNotIn("hope_observer", names)
        self.wait(lambda: self.runner_pub.get_subscription_count() == 1
                  and self.base_pub.get_subscription_count() == 1)
        for name in ("set_server", "enter_pd_stand", "prepare_serve", "confirm_ball_loaded",
                     "confirm_grip_secure", "ready_to_serve", "open_gripper", "enter_motion",
                     "set_receiver", "enter_teleop", "emergency_passive"):
            response = self.trigger("/hope/runner/" + name)
            self.assertTrue(response.success, response.message)
        self.reject = True
        response = self.trigger("/hope/runner/enter_motion")
        self.assertFalse(response.success)
        self.assertIn("REJECTED_WRONG_MODE", response.message)
        self.wait(lambda: self.metrics and self.metrics[-1]["runner"]["dispatched"] >= 12)
        self.assertGreaterEqual(self.metrics[-1]["runner"]["ack_latency_max_s"], 0.)

    def test_native_emergency_survives_paused_worker_without_replaying_prior_motion(self):
        if getattr(self, "reference", False):
            self.skipTest("native-only failure isolation")
        children = child_pids(self.process.pid)
        self.assertEqual(len(children), 1)
        worker = int(children[0])
        os.kill(worker, signal.SIGSTOP)
        try:
            prior = self.trigger("/hope/runner/enter_motion", wait=False)
            time.sleep(.15)  # Queued in the frozen worker, never a robot action.
            start = time.monotonic()
            stop = self.trigger("/hope/runner/emergency_passive")
            self.assertTrue(stop.success, stop.message)
            self.assertLess(time.monotonic()-start, 1.)
            self.assertFalse(prior.done())
            self.assertEqual([int(v[2]) for v in self.requests], [5])
        finally:
            os.kill(worker, signal.SIGCONT)
        self.wait(prior.done)
        self.assertFalse(prior.result().success)
        self.assertIn("superseded by emergency", prior.result().message)
        self.assertEqual([int(v[2]) for v in self.requests], [5])
        self.assertTrue(self.trigger("/hope/runner/enter_pd_stand").success)

    def test_restart_recovers_identity_for_explicit_cleanup_without_start_replay(self):
        self.player_publishing = True
        self.wait(lambda: self.fields and self.fields[-1].get("player_positions", {}).get("P1"))
        self.assertTrue(self.trigger("/hope/lifecycle/start").success)
        self.wait(lambda: self.lifecycle and self.lifecycle[-1] == "RUNNING")
        self.assertTrue((self.root / "operations_session.json").is_file())
        args = self.process.args
        self.process.terminate()
        self.process.wait(timeout=8)
        self.status.clear();self.lifecycle.clear()
        self.process = subprocess.Popen(args, env=dict(os.environ, HOPE_OPERATIONS_TEST_ROOT=str(self.root)),
                                        stdout=self.output, stderr=subprocess.STDOUT)
        self.wait(lambda: any(s.startswith("READY") for s in self.status), 15)
        self.wait(lambda: self.lifecycle and self.lifecycle[-1] == "FAILED")
        self.assertEqual((self.root / "actions").read_text().splitlines(), ["START"])
        self.assertFalse(self.trigger("/hope/lifecycle/start").success)
        self.assertTrue(self.trigger("/hope/lifecycle/kill_all_and_collect").success)
        self.wait(lambda: self.lifecycle[-1] == "STOPPED")
        self.assertEqual((self.root / "actions").read_text().splitlines(), ["START", "STOP"])
        self.assertFalse((self.root / "operations_session.json").exists())

    def startup_requests(self):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        # Both advertised service types must remain in DDS until all modules
        # finish construction, just as before rclpy's executor starts spinning.
        trigger = self.trigger("/hope/runner/set_receiver", wait=False)
        client = self.node.create_client(SetParameters, "/hope/lifecycle/apply_config")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        parameters = client.call_async(SetParameters.Request(parameters=[Parameter(
            name="table_side", value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                    string_value="NOT_A_SIDE"))]))
        time.sleep(.4)
        self.assertFalse(trigger.done(), "startup must not reply worker unavailable")
        self.assertFalse(parameters.done(), "parameter requests must also wait")
        self.assertEqual(self.requests, [], "no action before initialization")
        return trigger, parameters

    def test_startup_requests_wait_for_initialization_then_complete_once(self):
        trigger, parameters = self.startup_requests()
        (self.root / "hold_startup").unlink()
        self.wait(lambda: trigger.done() and parameters.done())
        self.assertTrue(trigger.result().success, trigger.result().message)
        self.assertFalse(parameters.result().results[0].successful)
        self.assertNotIn("worker unavailable", parameters.result().results[0].reason)
        self.assertEqual(len(self.requests), 1)
        time.sleep(.3)
        self.assertEqual(len(self.requests), 1, "queued action must not be replayed")

    def test_startup_failure_rejects_queued_requests_without_any_action(self):
        trigger, parameters = self.startup_requests()
        (self.root / "fail_startup").touch()
        (self.root / "hold_startup").unlink()
        self.wait(lambda: trigger.done() and parameters.done())
        self.assertFalse(trigger.result().success)
        self.assertIn("worker unavailable", trigger.result().message)
        self.assertFalse(parameters.result().results[0].successful)
        self.assertIn("worker unavailable", parameters.result().results[0].reason)
        self.assertEqual(self.requests, [])

    def test_startup_shutdown_rejects_queue_and_ignores_late_ready(self):
        trigger, parameters = self.startup_requests()
        self.process.send_signal(signal.SIGTERM)
        self.wait(lambda: trigger.done() and parameters.done())
        self.assertFalse(trigger.result().success)
        self.assertFalse(parameters.result().results[0].successful)
        # The fixture finishes construction after shutdown was requested. Its
        # late READY must never reopen admission or execute the queued action.
        (self.root / "hold_startup").unlink()
        self.assertEqual(self.process.wait(timeout=5), 0)
        self.assertEqual(self.requests, [])

    def test_urgent_passive_during_unacknowledged_ordinary_action(self):
        client = self.node.create_client(self.Trigger, "/hope/runner/emergency_passive")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        self.accept = False
        pending = self.trigger("/hope/runner/enter_motion", wait=False)
        self.wait(lambda: any(int(r[2]) == 4 for r in self.requests))
        start = time.monotonic()
        future = client.call_async(self.Trigger.Request())
        self.wait(future.done)
        emergency = future.result()
        self.assertTrue(emergency.success, emergency.message)
        self.assertLess(time.monotonic() - start, 1.)
        self.wait(pending.done, 4)
        self.assertFalse(pending.result().success)

    def test_oversize_requests_and_invalid_utf8_do_not_kill_owner(self):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        client = self.node.create_client(SetParameters, "/hope/field/command")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        # First uses the original field cap; second fits that cap but expands
        # beyond 32 MiB during JSON escaping and exercises call() cleanup.
        for text, expected in (("x" * (9*1024*1024 + 1), "field request exceeds 9 MiB"),
                               ("\x01" * (6*1024*1024), "IPC frame exceeds limit")):
            future = client.call_async(SetParameters.Request(parameters=[Parameter(name="request",
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=text))]))
            self.wait(future.done, 15)
            self.assertFalse(future.result().results[0].successful)
            self.assertIn(expected, future.result().results[0].reason)
            self.assertTrue(self.trigger("/hope/runner/emergency_passive").success)
        # rclpy refuses invalid Unicode before serialization; use the native
        # fixture to exercise actual ROS byte strings on both callback paths.
        for kind in ("service", "qualification", "qualification_oversize"):
            result = subprocess.run([str(Path(os.environ["HOPE_NATIVE_BIN_DIR"]) / "hope-operations-test-ingress"),
                "--ros-args", "-p", "malformed_input:=" + kind], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            if kind == "qualification_oversize":
                self.wait(lambda: "Dropped operations input" in (self.root / "native.log").read_text(), 5)
            self.assertIsNone(self.process.poll())
            self.assertTrue(self.trigger("/hope/runner/emergency_passive").success)
        count = len(self.requests)
        time.sleep(.4)
        self.assertEqual(len(self.requests), count, "actions must never be replayed")

    def test_laptop_calibration_client_preserves_rejected_result(self):
        (self.root / "session").write_text("model21800_20261028T010203Z")
        self.mode = 1
        self.wait(lambda: self.modes and self.modes[-1] == "PD_STAND")
        def reject(_request, response):
            response.success = False
            response.message = "isolated calibration rejection"
            return response
        self.node.create_service(self.Trigger, "/a3/calibration/recompute_p1", reject)
        response = self.trigger("/hope/calibrate_stickers_v3")
        self.assertFalse(response.success)
        self.assertIn("isolated calibration rejection", response.message)

    def test_real_receipt_ages_and_invalid_base_are_preserved(self):
        from std_msgs.msg import Float64MultiArray
        self.wait(lambda: self.base_pub.get_subscription_count() == 1)
        packet = [2.,1.,1.,1700000000.,0.,-.4,0.,1.,1.,0.,0.,0.,1.,47.,1.,1.]
        for _ in range(15):
            self.base_pub.publish(Float64MultiArray(data=packet))
            time.sleep(.02)
        self.wait(lambda: any(self.base_fresh))
        time.sleep(.8)
        self.wait(lambda: not self.base_fresh[-1])
        self.assertGreater(self.base_age[-1], 500.)
        age_start = len(self.base_age)
        freshness_start = len(self.base_fresh)
        packet[1] = 0.
        # Raw poses use best-effort QoS: do not make a one-datagram delivery
        # guarantee that the actual transport deliberately does not provide.
        for _ in range(10):
            self.base_pub.publish(Float64MultiArray(data=packet))
            time.sleep(.02)
        # Inspect received history rather than only the latest display tick:
        # an overloaded test process can wake after the short age window.
        self.wait(lambda: any(age < 400. for age in self.base_age[age_start:]))
        self.assertFalse(any(self.base_fresh[freshness_start:]),
                         "invalid packets must never renew a fresh-valid verdict")

    def test_start_stop_and_configuration_use_existing_worker_rules(self):
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        client = self.node.create_client(SetParameters, "/hope/lifecycle/apply_config")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        # Wrong-type input still follows the original validation path.
        bad = SetParameters.Request(parameters=[Parameter(name="table_side", value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=1))])
        future = client.call_async(bad)
        self.wait(future.done)
        self.assertFalse(future.result().results[0].successful)
        self.player_publishing = True
        self.wait(lambda: self.fields and self.fields[-1].get("player_positions", {}).get("P1"))
        start = self.trigger("/hope/lifecycle/start")
        self.assertTrue(start.success, start.message)
        self.wait(lambda: self.lifecycle and self.lifecycle[-1] == "RUNNING", 10)
        self.assertEqual((self.root / "actions").read_text().splitlines(), ["START"])
        self.player_publishing = False
        self.wait(lambda: not self.fields[-1]["player_positions"]["P1"])
        self.assertEqual(self.lifecycle[-1], "RUNNING", "the side check is START-only")
        stop = self.trigger("/hope/lifecycle/kill_all_and_collect")
        self.assertTrue(stop.success, stop.message)
        self.wait(lambda: self.lifecycle[-1] == "STOPPED", 5)
        self.assertEqual((self.root / "actions").read_text().splitlines(), ["START", "STOP"])

    def test_two_300hz_player_streams_remain_native_and_expire_without_replay(self):
        binary = Path(os.environ["HOPE_NATIVE_BIN_DIR"]) / "hope-operations-test-ingress"
        with subprocess.Popen([str(binary), "--ros-args", "-p", "players_only:=true"],
                              stdout=self.output, stderr=subprocess.STDOUT) as ingress:
            self.assertEqual(ingress.wait(timeout=8), 0)
        self.assertFalse(self.player_stats()["counts"], "no raw player callback may enter Python")
        self.assertTrue(any(f.get("player_positions", {}).get("P1") for f in self.fields))
        self.assertTrue(any(f.get("player_positions", {}).get("P2") for f in self.fields))
        self.wait(lambda: self.fields and not self.fields[-1]["player_positions"]["P1"]
                  and not self.fields[-1]["player_positions"]["P2"])

    def test_network_change_preserves_running_session_emergency_and_stop_until_idle(self):
        import fcntl
        self.player_publishing = True
        self.wait(lambda: self.fields and self.fields[-1].get("player_positions", {}).get("P1"))
        start = self.trigger("/hope/lifecycle/start")
        self.assertTrue(start.success, start.message)
        self.wait(lambda: self.lifecycle and self.lifecycle[-1] == "RUNNING", 10)
        self.wait(lambda: self.player_stats().get("owner", {}).get("hardware_lock"))
        owner = self.player_stats()["owner"]
        original_children = child_pids(self.process.pid)
        self.assertEqual(len(original_children), 1)
        with (self.root / "hardware.lock").open("r+") as hardware:
            with self.assertRaises(BlockingIOError):
                fcntl.flock(hardware, fcntl.LOCK_EX | fcntl.LOCK_NB)
            (self.root / "network_signature").write_text("DHCP_OR_USB_ADAPTER_CHANGE")
            self.wait(lambda: any(s.startswith("DDS_RECONNECT_DEFERRED") for s in self.status))
            time.sleep(1.2)  # More than one network poll; deferred status must persist.
            self.assertEqual(self.status[-1].split(":")[0], "DDS_RECONNECT_DEFERRED")
            self.assertEqual(child_pids(self.process.pid), original_children)
            self.assertEqual(self.player_stats()["owner"], owner)
            self.assertEqual(self.lifecycle[-1], "RUNNING")
            with self.assertRaises(BlockingIOError):
                fcntl.flock(hardware, fcntl.LOCK_EX | fcntl.LOCK_NB)
            emergency = self.trigger("/hope/runner/emergency_passive")
            self.assertTrue(emergency.success, emergency.message)
            self.assertTrue(any(int(r[2]) == 5 for r in self.requests))
            stop = self.trigger("/hope/lifecycle/kill_all_and_collect")
            self.assertTrue(stop.success, stop.message)
            self.wait(lambda: self.lifecycle[-1] == "STOPPED")
            self.wait(lambda: any(s.startswith("DDS_RECONNECTING") for s in self.status))
            self.assertEqual(self.process.wait(timeout=6), 1, "only idle owner may rebuild DDS through systemd")
            fcntl.flock(hardware, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.assertEqual((self.root / "actions").read_text().splitlines(), ["START", "STOP"])

    def test_idle_network_change_waits_for_unacknowledged_command_and_urgent_rpc(self):
        self.accept = False
        pending = self.trigger("/hope/runner/enter_motion", wait=False)
        self.wait(lambda: any(int(r[2]) == 4 for r in self.requests))
        (self.root / "network_signature").write_text("CHANGED_WHILE_COMMAND_IN_FLIGHT")
        self.wait_reconnect_deferred(pending)
        emergency = self.trigger("/hope/runner/emergency_passive")
        self.assertTrue(emergency.success, emergency.message)
        self.wait(pending.done)
        self.assertFalse(pending.result().success)
        self.wait(lambda: any(s.startswith("DDS_RECONNECTING") for s in self.status))
        self.assertEqual(self.process.wait(timeout=6), 1)
        self.assertEqual([int(r[2]) for r in self.requests], [4, 5], "never replay either action")

    def test_idle_reconnect_waits_for_native_emergency_alone(self):
        self.accept_emergency = False
        pending = self.trigger("/hope/runner/emergency_passive", wait=False)
        self.wait(lambda: bool(self.requests))
        (self.root / "network_signature").write_text("CHANGED_WITH_NATIVE_EMERGENCY_ONLY")
        self.wait_reconnect_deferred(pending)
        self.assertIsNone(self.process.poll())
        req, action = int(self.requests[-1][1]), int(self.requests[-1][2])
        self.ack = req, action, 1, 3
        self.wait(pending.done)
        self.assertTrue(pending.result().success)
        self.wait(lambda: any(s.startswith("DDS_RECONNECTING") for s in self.status))
        self.assertEqual(self.process.wait(timeout=6), 1)
        self.assertEqual(len(self.requests), 1)

    def test_failed_worker_emergency_drain_waits_for_timeout_reply(self):
        self.accept_emergency = False
        pending = self.trigger("/hope/runner/emergency_passive", wait=False)
        self.wait(lambda: bool(self.requests))
        children = child_pids(self.process.pid)
        self.assertEqual(len(children), 1)
        os.kill(int(children[0]), signal.SIGKILL)
        time.sleep(2.2)  # Longer than failed-worker grace, shorter than action timeout.
        self.assertIsNone(self.process.poll(), "failed-worker shutdown discarded native emergency")
        self.assertFalse(pending.done())
        self.wait(pending.done, 3.)
        self.assertFalse(pending.result().success)
        self.assertIn("not confirmed within 4.0s", pending.result().message)
        self.assertEqual(self.process.wait(timeout=5), 1)
        self.assertEqual(len(self.requests), 1)

    def test_split_supervisor_recovers_using_direct_runner_identity(self):
        self.player_publishing = True
        self.wait(lambda: self.fields and self.fields[-1]["player_positions"]["P1"])
        self.assertTrue(self.trigger("/hope/lifecycle/start").success)
        self.wait(lambda: self.lifecycle[-1] == "RUNNING")
        self.process.terminate()
        self.assertEqual(self.process.wait(timeout=8), 0)
        self.lifecycle.clear()
        self.process = subprocess.Popen(
            [sys.executable, str(ROOT / "tests/fixtures/operations_worker.py")],
            env=dict(os.environ, HOPE_OPERATIONS_TEST_ROOT=str(self.root), HOPE_OPERATIONS_TEST_SPLIT="1"),
            stdout=self.output, stderr=subprocess.STDOUT)
        self.wait(lambda: self.lifecycle and self.lifecycle[-1] == "FAILED", 10)
        # No observer scalar topics are publishing after the native owner exits.
        self.wait(lambda: self.runner_pub.get_subscription_count() > 0)
        time.sleep(.15)
        start = self.trigger("/hope/lifecycle/start")
        self.assertFalse(start.success)
        self.assertIn("MANAGED_RECOVERY_REQUIRED", start.message)
        stop = self.trigger("/hope/lifecycle/kill_all_and_collect")
        self.assertTrue(stop.success, stop.message)
        self.wait(lambda: self.lifecycle[-1] == "STOPPED")
        self.assertFalse((self.root / "operations_session.json").exists())
        self.assertEqual((self.root / "actions").read_text().splitlines(), ["START", "STOP"])

    def test_failed_worker_drain_keeps_emergency_drain_alive_until_ack(self):
        self.accept_emergency = False
        pending = self.trigger("/hope/runner/emergency_passive", wait=False)
        self.wait(lambda: bool(self.requests))
        (self.root / "fail_control_timer").touch()
        self.wait(lambda: any(s.startswith("DRAINING") for s in self.status))
        time.sleep(1.2)  # At least two failed-status ticks since the forced drain.
        self.assertIsNone(self.process.poll())
        self.assertFalse(pending.done())
        req, action = int(self.requests[-1][1]), int(self.requests[-1][2])
        self.ack = req, action, 1, 3
        self.wait(pending.done, 2.)
        self.assertTrue(pending.result().success, pending.result().message)
        self.assertEqual(self.process.wait(timeout=5), 1)
        self.assertEqual(len(self.requests), 1)

    def test_telemetry_fault_is_visible_and_cannot_destroy_managed_session(self):
        self.player_publishing = True
        self.wait(lambda: self.fields and self.fields[-1].get("player_positions", {}).get("P1"))
        self.assertTrue(self.trigger("/hope/lifecycle/start").success)
        self.wait(lambda: self.lifecycle and self.lifecycle[-1] == "RUNNING")
        self.wait(lambda: self.player_stats().get("owner", {}).get("hardware_lock"))
        owner = self.player_stats()["owner"]
        failure = self.root / "fail_observer_snapshot"
        failure.touch()
        self.wait(lambda: any("isolated observer snapshot failure" in s for s in self.telemetry_faults))
        self.assertGreater(self.telemetry_fault_counts[-1], 0)
        self.assertEqual(self.player_stats()["owner"], owner)
        self.assertEqual(self.lifecycle[-1], "RUNNING")
        self.assertTrue(self.trigger("/hope/runner/emergency_passive").success)
        failure.unlink()
        self.wait(lambda: self.telemetry_faults[-1].startswith("OK"))
        self.assertTrue(self.trigger("/hope/lifecycle/kill_all_and_collect").success)
        self.wait(lambda: self.lifecycle[-1] == "STOPPED")

    def test_record_off_and_emergency_remain_available_behind_blocked_asset_rpc(self):
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        client = self.node.create_client(SetParameters, "/hope/field/command")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        def call(operation):
            return client.call_async(SetParameters.Request(parameters=[Parameter(name="request", value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=json.dumps({"op": operation})))]))
        hold = self.root / "hold_asset"
        hold.touch()
        try:
            pending = call("refresh_calibration")
            self.wait(lambda: (self.root / "asset_entered").exists())
            started = time.monotonic()
            off = call("record_off")
            self.wait(off.done, 1.)
            self.assertTrue(off.result().results[0].successful, off.result().results[0].reason)
            self.assertLess(time.monotonic() - started, 1.)
            self.assertFalse(pending.done(), "recording must use the field's independent RPC lane")
            self.assertTrue(self.trigger("/hope/runner/emergency_passive").success)
            self.assertFalse(pending.done())
        finally:
            hold.unlink()
        self.wait(pending.done)
        self.assertTrue(pending.result().results[0].successful)

    def test_expired_native_player_pose_cannot_qualify_start(self):
        self.wait(lambda: self.player_pubs["UCB_P2"].get_subscription_count() > 0)
        for _ in range(5):
            self.publish_player("UCB_P2", -.5)
            time.sleep(.005)
        time.sleep(.4)
        response = self.trigger("/hope/lifecycle/start")
        self.assertTrue(response.success, response.message)  # Asynchronous admission attempt.
        self.wait(lambda: any("PLAYER_POSITION_UNVERIFIED" in result for result in self.results))
        self.assertEqual(self.lifecycle[-1], "STOPPED")
        self.assertFalse((self.root / "actions").exists(), "stale player evidence must not reach the backend")

    def test_300hz_ingress_preserves_rate_and_native_marker_without_replay(self):
        binary = Path(os.environ["HOPE_NATIVE_BIN_DIR"]) / "hope-operations-test-ingress"
        with subprocess.Popen([str(binary)], stdout=self.output, stderr=subprocess.STDOUT) as ingress:
            self.assertEqual(ingress.wait(timeout=8), 0)
        time.sleep(.3)
        self.assertTrue(any(rate > 150. for rate in self.ball_rates),
                        f"rates={self.ball_rates}, markers={len(self.markers)}")
        self.assertGreater(len(self.markers), 20,
                           f"rates={self.ball_rates}")
        self.assertGreaterEqual(sum(self.ball_live), 4,
                                f"healthy 300 Hz input must not appear stale: {self.ball_live}")
        self.assertLess(len(self.markers), 55)
        self.assertEqual(self.markers[-1].header.frame_id, "world")
        self.assertAlmostEqual(self.markers[-1].pose.position.x, 1.)
        self.assertEqual(self.markers[-1].lifetime.nanosec, 200_000_000)
        self.wait(lambda: self.ball_live and not self.ball_live[-1])
        count = len(self.markers)
        time.sleep(.3)
        self.assertEqual(len(self.markers), count, "cached input must not create new marker arrivals")

    def test_worker_loss_returns_failure_without_replaying_action(self):
        self.accept = False
        future = self.trigger("/hope/runner/enter_motion", wait=False)
        self.wait(lambda: bool(self.requests))
        children = child_pids(self.process.pid)
        self.assertEqual(len(children), 1)
        os.kill(int(children[0]), signal.SIGKILL)
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("disconnected", future.result().message)
        self.wait(lambda: self.status[-1].startswith("ERROR:"))
        time.sleep(.3)
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.process.wait(timeout=5), 1, "allow systemd to restart a failed worker")

    def test_sigterm_drains_accepted_stop_and_retains_hardware_lock(self):
        import fcntl
        self.player_publishing = True
        self.wait(lambda: self.fields and self.fields[-1]["player_positions"]["P1"])
        self.assertTrue(self.trigger("/hope/lifecycle/start").success)
        self.wait(lambda: self.lifecycle[-1] == "RUNNING")
        hold = self.root / "hold_stop"
        hold.touch()
        try:
            self.assertTrue(self.trigger("/hope/lifecycle/kill_all_and_collect").success)
            self.wait(lambda: "STOP" in (self.root / "actions").read_text())
            self.process.terminate()
            time.sleep(.6)
            self.assertIsNone(self.process.poll(), "accepted cleanup must survive SIGTERM")
            with (self.root / "hardware.lock").open("r+") as lock:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
            hold.unlink()
        self.assertEqual(self.process.wait(timeout=8), 0)
        with (self.root / "hardware.lock").open("r+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.assertEqual((self.root / "actions").read_text().splitlines(), ["START", "STOP"])
