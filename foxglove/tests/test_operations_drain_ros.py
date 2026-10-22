"""Graceful shutdown on real rclpy, with hardware/SSH backends replaced."""
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch


@unittest.skipUnless(os.environ.get("ROS_LOCALHOST_ONLY") == "1"
                     and os.environ.get("ROS_DOMAIN_ID") == "219",
                     "requires isolated localhost domain 219")
class OperationsDrainTests(unittest.TestCase):
    def test_status_publish_failure_does_not_skip_component_cleanup(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
        from hope_operations import HopeOperations
        components = [Mock() for _ in range(4)]
        node = SimpleNamespace(_closed=False,
                               stop_accepting=Mock(side_effect=RuntimeError("status publish failed")),
                               commands=components[0], lifecycle=components[1],
                               observer=components[2], _network=components[3])
        with self.assertRaisesRegex(RuntimeError, "status publish failed"):
            HopeOperations.close(node)
        for component in components:
            component.close.assert_called_once()

    def test_combined_and_legacy_drains_keep_responses_and_telemetry_live(self):
        for kind in ("operations", "lifecycle", "proxy"):
            with self.subTest(kind=kind):
                self.exercise(kind)

    def exercise(self, kind):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.task import Future
        from rcl_interfaces.msg import SetParametersResult
        from rcl_interfaces.srv import SetParameters
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        import hope_lifecycle_supervisor as lifecycle_module
        from hope_command_proxy import CommandProxyModule, HopeCommandProxy
        from hope_operations import HopeOperations
        from hope_operations_runtime import drain_and_destroy

        entered, release = threading.Event(), threading.Event()
        job_release, job_done = threading.Event(), threading.Event()
        response_waiting, permit_response = threading.Event(), threading.Event()
        cleanup_done, destroyed = threading.Event(), threading.Event()
        errors, destroyed_after_response = [], []
        recorder = Mock()
        recorder.status.return_value = {"state": "OFF", "output": "", "error": ""}
        recorder.stop.return_value = recorder.status.return_value

        class HeldResponseFuture(Future):
            def __await__(self):
                result = yield from super().__await__()
                response_waiting.set()
                # Hold the service coroutine after the worker is done, so a
                # drain that only joins workers would destroy the Node early.
                while not permit_response.is_set():
                    yield
                return result

        def delayed_config(_self, request, response):
            entered.set()
            release.wait(15)
            response.results = [SetParametersResult(successful=True, reason="accepted response")
                                for _ in request.parameters]
            return response

        def delayed_command(_self, _request, response):
            entered.set()
            release.wait(15)
            response.success, response.message = True, "accepted response"
            return response

        with tempfile.TemporaryDirectory() as temporary, \
                patch.object(lifecycle_module, "CONFIG_PATH", Path(temporary) / "config.json"), \
                patch.object(lifecycle_module, "DistributedRecorder", return_value=recorder), \
                patch.object(lifecycle_module, "LifecycleBackend", return_value=Mock()), \
                patch.object(lifecycle_module.LifecycleModule, "_apply_config", delayed_config), \
                patch.object(CommandProxyModule, "_refresh_x_hit", delayed_command), \
                patch("hope_operations.read_ipv4_interface_signature", return_value=None), \
                patch("hope_command_proxy.read_ipv4_interface_signature", return_value=None), \
                patch("rclpy.task.Future", HeldResponseFuture):
            rclpy.init(args=["--ros-args", "-p", f"session_id_path:={temporary}/session",
                            "-p", f"runtime_root:={temporary}/runtime"])
            factory = {"operations": HopeOperations, "lifecycle": lifecycle_module.HopeLifecycleSupervisor,
                       "proxy": HopeCommandProxy}[kind]
            node = factory()
            probe = rclpy.create_node("drain_probe", start_parameter_services=False)
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor.add_node(probe)
            summaries, states, steps, statuses, fields = [], [], [], [], []
            for topic, received in (("/hope/lifecycle/summary", summaries),
                                    ("/hope/lifecycle/state", states),
                                    ("/hope/lifecycle/step", steps),
                                    ("/hope/operations/status", statuses),
                                    ("/hope/field/status", fields)):
                probe.create_subscription(String, topic, lambda msg, target=received: target.append(msg.data), 10)
            if kind == "proxy":
                service_type, service_name = Trigger, "/hope/refresh_x_hit"
                request = Trigger.Request()
                lifecycle = None
            else:
                service_type, service_name = SetParameters, "/hope/lifecycle/apply_config"
                request = SetParameters.Request(parameters=[Parameter("test", value="drain").to_parameter_msg()])
                lifecycle = node.lifecycle if kind == "operations" else node
            client = probe.create_client(service_type, service_name)
            logger = Mock()
            drain_thread = None

            def until(predicate, *, spinning=False, timeout=4):
                deadline = time.monotonic() + timeout
                while not predicate() and time.monotonic() < deadline:
                    if spinning:
                        executor.spin_once(timeout_sec=.01)
                    else:
                        time.sleep(.01)
                self.assertTrue(predicate(), f"{kind}: condition timed out")

            original_close, original_destroy = node.close, node.destroy_node
            def close():
                try:
                    original_close()
                finally:
                    cleanup_done.set()
            def destroy():
                destroyed_after_response.append(accepted.done())
                try:
                    return original_destroy()
                finally:
                    destroyed.set()
            def drain():
                try:
                    drain_and_destroy(node, executor)
                except Exception as exc:
                    errors.append(exc)
            def delayed_job():
                with lifecycle._lock:
                    lifecycle._state, lifecycle._step = "KILLING", "COLLECT"
                job_release.wait(15)
                with lifecycle._lock:
                    lifecycle._step = "COLLECT_COMPLETE"
                job_done.set()

            try:
                self.assertTrue(client.wait_for_service(timeout_sec=3))
                accepted = client.call_async(request)
                until(entered.is_set, spinning=True)
                if lifecycle is not None:
                    lifecycle._pool.submit(delayed_job)
                with patch.object(node, "close", side_effect=close), \
                        patch.object(node, "destroy_node", side_effect=destroy), \
                        patch.object(node, "get_logger", return_value=logger):
                    drain_thread = threading.Thread(target=drain)
                    drain_thread.start()
                    until(lambda: node._shared_state.admission.closed if kind == "proxy" else
                          (node.shared_state.admission.closed if kind == "operations" else node._admission.closed))
                    # A callback failure during shutdown is logged; it cannot
                    # abort the drain or stop later status/response callbacks.
                    def fail_once():
                        failure_timer.cancel()
                        raise ValueError("injected drain callback failure")
                    failure_timer = node.create_timer(.01, fail_once)
                    rejected = client.call_async(request)
                    until(rejected.done)
                    response = rejected.result()
                    reason = response.message if kind == "proxy" else response.results[0].reason
                    self.assertTrue(reason.startswith("DRAINING:"), reason)
                    until(lambda: any("injected drain callback failure" in str(call)
                                      for call in logger.error.call_args_list))
                    if lifecycle is not None:
                        until(lambda: len([s for s in summaries if "draining=1" in s]) >= 2)
                        self.assertEqual(states[-1], "KILLING")
                        self.assertEqual(steps[-1], "COLLECT")
                        self.assertIn('"draining": true', fields[-1])
                        self.assertTrue(lifecycle._refresh_timer.is_canceled())
                        self.assertFalse(lifecycle._publish_timer.is_canceled())
                    if kind == "operations":
                        until(lambda: len([s for s in statuses if s.startswith("DRAINING:")]) >= 2)
                    release.set()
                    until(response_waiting.is_set)
                    job_release.set()
                    until(cleanup_done.is_set)
                    # Cleanup and worker futures are done; the held coroutine
                    # still owns the accepted response and must keep DDS alive.
                    self.assertGreater(node._hope_rpc_tracker.pending, 0)
                    self.assertFalse(accepted.done())
                    self.assertFalse(destroyed.is_set())
                    self.assertTrue(drain_thread.is_alive())
                    if lifecycle is not None:
                        self.assertTrue(job_done.is_set())
                        count = len(summaries)
                        until(lambda: len(summaries) > count)
                        self.assertEqual(steps[-1], "COLLECT_COMPLETE")
                    permit_response.set()
                    until(accepted.done)
                    response = accepted.result()
                    self.assertTrue(response.success if kind == "proxy" else response.results[0].successful)
                    drain_thread.join(4)
                    self.assertFalse(drain_thread.is_alive())
                    self.assertEqual(errors, [])
                    self.assertEqual(destroyed_after_response, [True])
            finally:
                release.set()
                job_release.set()
                permit_response.set()
                if drain_thread is not None:
                    drain_thread.join(5)
                if not destroyed.is_set():
                    node.close()
                    executor.shutdown(wait_for_threads=True)
                    node.destroy_node()
                probe.destroy_node()
                rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main()
