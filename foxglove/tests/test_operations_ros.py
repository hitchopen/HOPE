"""One real rclpy Node, async RPC completion and saturated-worker progress.

Only run inside the isolated localhost test domain; no helper/SSH/hardware calls.
"""
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch


@unittest.skipUnless(os.environ.get("ROS_LOCALHOST_ONLY") == "1"
                     and os.environ.get("ROS_DOMAIN_ID") == "219",
                     "requires isolated localhost domain 219")
class OperationsRosTests(unittest.TestCase):
    def test_one_node_keeps_telemetry_ack_and_record_off_live_under_saturation(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import ReliabilityPolicy
        from rcl_interfaces.msg import SetParametersResult
        from rcl_interfaces.srv import SetParameters
        from std_msgs.msg import Float64MultiArray, String
        from std_srvs.srv import Trigger
        import hope_lifecycle_supervisor as lifecycle_module
        from hope_command_proxy import CommandProxyModule
        from hope_operations import DdsReconnectRequired, HopeOperations
        from hope_runner_control_core import runner_session_fingerprint
        from hope_observer_core import REQUIRED_BASE_FLAGS

        release = threading.Event()
        release_telemetry = threading.Event()
        entered_command = threading.Event()
        entered_config = []
        recorder = Mock()
        recorder.status.return_value = {"state": "OFF", "output": "", "error": ""}
        recorder.stop.return_value = recorder.status.return_value

        def block_command(_self, _request, response):
            entered_command.set()
            release.wait(20)
            response.success, response.message = True, "completed in worker"
            return response

        def block_config(_self, request, response):
            entered_config.append(True)
            release.wait(20)
            response.results = [SetParametersResult(successful=True) for _ in request.parameters]
            return response

        with tempfile.TemporaryDirectory() as temporary, \
                patch.object(lifecycle_module, "CONFIG_PATH", Path(temporary) / "config.json"), \
                patch.object(lifecycle_module, "DistributedRecorder", return_value=recorder), \
                patch.object(lifecycle_module, "LifecycleBackend", return_value=Mock()), \
                patch.object(lifecycle_module.LifecycleModule, "_apply_config", block_config), \
                patch.object(CommandProxyModule, "_refresh_x_hit", block_command), \
                patch("hope_operations.read_ipv4_interface_signature", return_value=None):
            args = ["--ros-args", "-p", f"session_id_path:={temporary}/session",
                    "-p", f"runtime_root:={temporary}/runtime"]
            rclpy.init(args=args)
            operations = HopeOperations()
            self.assertIn("STICKERS_V3", operations.commands._calibration_services)
            self.assertEqual(operations.get_parameter("calibration_service_stickers_v3").value, "/a3/calibration/recompute_p1")
            probe = rclpy.create_node("operations_contract_probe", start_parameter_services=False)
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(operations)
            executor.add_node(probe)
            heartbeats = []
            probe.create_subscription(String, "/hope/lifecycle/state", lambda m: heartbeats.append(m.data), 10)
            state_pub = probe.create_publisher(Float64MultiArray, "/hope/runner/state_hdu_flat", 10)
            base_pub = probe.create_publisher(Float64MultiArray, "/a3/base_pose_flat", 10)
            session = "model21800_20261022T010203Z"
            sequence = [8.]
            def acknowledge(request):
                sequence[0] += 1
                state_pub.publish(Float64MultiArray(data=[
                    2., 1234., sequence[0], 0., 0., 1., 0., 2., 1., 1., 1., 1.,
                    0., -1., -1., 0., request.data[1], request.data[2], 1., 3.,
                    float(runner_session_fingerprint(session))]))
            probe.create_subscription(Float64MultiArray, "/hope/runner/control_request_hdu_flat", acknowledge, 10)
            clients = {name: probe.create_client(kind, service) for name, kind, service in (
                ("ordinary", Trigger, "/hope/refresh_x_hit"),
                ("busy", Trigger, "/hope/runner/enter_pd_stand"),
                ("urgent", Trigger, "/hope/runner/emergency_passive"),
                ("config", SetParameters, "/hope/lifecycle/apply_config"),
                ("record", SetParameters, "/hope/field/command"))}
            def until(predicate, timeout=4):
                deadline = time.monotonic() + timeout
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(predicate(), "condition timed out")
            try:
                self.assertTrue(all(client.wait_for_service(timeout_sec=3) for client in clients.values()))
                self.assertIsInstance(operations, Node)
                self.assertTrue(all(not isinstance(module, Node) for module in
                                    (operations.commands, operations.observer, operations.lifecycle)))
                self.assertEqual(sum(s.topic_name == "/hope/runner/state_hdu_flat"
                                     for s in operations.subscriptions), 1)
                self.assertEqual(sum(s.topic_name == "/a3/base_pose_flat"
                                     for s in operations.subscriptions), 1)
                inputs = {s.topic_name: s for s in operations.subscriptions
                          if s.topic_name in {"/a3/base_pose_flat", "/hope/runner/state_hdu_flat"}}
                base_reader = inputs["/a3/base_pose_flat"]
                runner_reader = inputs["/hope/runner/state_hdu_flat"]
                self.assertEqual(base_reader.qos_profile.reliability, ReliabilityPolicy.RELIABLE)
                self.assertEqual(base_reader.qos_profile.depth, 10)
                self.assertIsNot(base_reader.callback_group, operations.default_callback_group)
                self.assertIsNot(runner_reader.callback_group, operations.default_callback_group)
                self.assertIsNot(base_reader.callback_group, runner_reader.callback_group)
                until(lambda: {name for name, _ in probe.get_node_names_and_namespaces()}
                      == {"hope_operations", "operations_contract_probe"})
                ordinary = clients["ordinary"].call_async(Trigger.Request())
                config_request = SetParameters.Request(parameters=[Parameter("test", value="blocked").to_parameter_msg()])
                configs = [clients["config"].call_async(config_request) for _ in range(2)]
                until(lambda: entered_command.is_set() and len(entered_config) == 2)
                count = len(heartbeats)
                until(lambda: len(heartbeats) >= count + 2)
                # Deliberately stall the default telemetry group. Each input
                # wire must still run on the other executor thread.
                entered_telemetry = threading.Event()
                def block_telemetry():
                    telemetry_timer.cancel()
                    entered_telemetry.set()
                    release_telemetry.wait(10)
                telemetry_timer = operations.create_timer(.01, block_telemetry)
                until(entered_telemetry.is_set)
                base_pub.publish(Float64MultiArray(data=[2., 1., 5., 1., 0., 0., 0., .2,
                    1., 0., 0., 0., 1., float(REQUIRED_BASE_FLAGS), 77., 1.]))
                rejected = clients["busy"].call_async(Trigger.Request())
                urgent = clients["urgent"].call_async(Trigger.Request())
                off = clients["record"].call_async(SetParameters.Request(parameters=[
                    Parameter("request", value='{"op":"record_off"}').to_parameter_msg()]))
                until(lambda: rejected.done() and urgent.done() and off.done())
                self.assertFalse(rejected.result().success)
                self.assertTrue(urgent.result().success, urgent.result().message)
                self.assertTrue(off.result().results[0].successful, off.result())
                until(lambda: operations.shared_state.base.snapshot().value is not None)
                self.assertEqual(operations.shared_state.base.snapshot().value.calibration_id, 77)
                recorder.stop.assert_called_once()
                self.assertFalse(ordinary.done())
                self.assertTrue(all(not future.done() for future in configs))
                release_telemetry.set()
                until(lambda: operations.observer._runner_state is not None)
                self.assertIs(operations.shared_state.runner.snapshot().value, operations.observer._runner_state)
                operations.lifecycle._session_id = session
                operations.lifecycle._wait_for_authoritative_runner()
                release.set()
                until(lambda: ordinary.done() and all(future.done() for future in configs))
                self.assertTrue(ordinary.result().success)
                self.assertTrue(all(future.result().results[0].successful for future in configs))
                malformed = clients["record"].call_async(SetParameters.Request(parameters=[
                    Parameter("request", value="[" * 1500 + "]" * 1500).to_parameter_msg()]))
                until(malformed.done)
                self.assertFalse(malformed.result().results[0].successful)
                # A changed interface must retain active ownership; only a
                # stopped, sealed process may request a clean DDS restart.
                operations._network.poll = lambda: (("eth0", "10.0.0.2"),)
                operations._dds_interfaces = (("eth0", "10.0.0.1"),)
                operations.lifecycle._state = "RUNNING"
                operations._check_network()
                self.assertIn("DDS_RECONNECT_DEFERRED", operations._network_status)
                self.assertFalse(operations.shared_state.admission.closed)
                operations.lifecycle._state = "STOPPED"
                with self.assertRaises(DdsReconnectRequired):
                    operations._check_network()
                self.assertTrue(operations.shared_state.admission.closed)
                self.assertTrue(operations._network_status.startswith("DDS_RECONNECTING:"))
            finally:
                release_telemetry.set()
                release.set()
                operations.stop_accepting()
                operations.close()
                __import__("hope_operations_runtime").shutdown_executor(executor)
                probe.destroy_node()
                operations.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
