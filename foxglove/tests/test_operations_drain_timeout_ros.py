"""Bound a suspended RPC after cleanup on real Jazzy, in a watchdog subprocess."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "219",
    reason="requires isolated localhost domain 219",
)


def test_suspended_response_times_out_only_after_worker_cleanup(tmp_path):
    assert os.environ.get("ROS_DISTRO") == "jazzy", "source the intended Jazzy installation"
    report_path = tmp_path / "drain-timeout.json"
    # An unbounded-drain regression must fail this test, not hang the suite.
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), "--child", str(report_path)],
        capture_output=True, text=True, timeout=20, check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert report_path.exists(), output
    report = json.loads(report_path.read_text())
    assert report["ros_distro"] == "jazzy"
    assert report["cleanup_held_longer_than_response_timeout"] is True
    assert report["response_unanswered"] is True
    assert report["context_ok_before_shutdown"] is True
    assert report["context_ok_after_shutdown"] is False
    assert report["pending_at_destroy"] == 1
    assert report["events"] == ["admission_sealed", "cleanup_started", "cleanup_released",
                                "cleanup_complete", "executor_shutdown", "node_destroyed",
                                "context_shutdown"]
    assert report["shutdown_arguments"] == {"timeout_sec": 0.0, "wait_for_threads": True}
    assert report["response_timeout_s"] - 0.02 <= report["post_cleanup_elapsed_s"] < 3.0
    diagnostics = "\n".join(report["diagnostics"])
    assert "shutdown response drain timed out" in diagnostics
    assert "pending=1" in diagnostics and "PendingService.respond" in diagnostics
    assert "pending responses are abandoned" in diagnostics
    assert "shutdown cleanup:" not in diagnostics
    assert "Exception ignored in" not in output, output
    assert "was never retrieved" not in output, output
    assert "cannot use Destroyable" not in output, output
    assert "Traceback (most recent call last)" not in output, output


def _run_child(report_path):
    import threading
    import time
    from unittest.mock import Mock, patch

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.task import Future
    from std_srvs.srv import Trigger

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
    from hope_operations_core import BoundedWorker, RosComponent
    from hope_operations_runtime import drain_and_destroy

    response_timeout_s = 0.25
    response_waiting, cleanup_started = threading.Event(), threading.Event()
    release_cleanup = threading.Event()
    cleanup_finished_at, controller_errors = [], []
    logger = Mock()
    report = {"ros_distro": os.environ.get("ROS_DISTRO"), "events": [],
              "response_timeout_s": response_timeout_s}
    events = report["events"]

    class PendingService(RosComponent):
        def respond(self, _request, response):
            response.success, response.message = True, "worker finished"
            return response

    class DrainNode(Node):
        def __init__(self):
            super().__init__("drain_timeout_server", start_parameter_services=False)
            self.worker = BoundedWorker("drain-timeout-test")
            self.component = PendingService(self)
            self.create_service(Trigger, "/test/drain_timeout",
                                self.component._worker_callback(self.component.respond, self.worker))

        def stop_accepting(self):
            self.worker.stop_accepting()
            events.append("admission_sealed")

        def close(self):
            events.append("cleanup_started")
            cleanup_started.set()
            assert release_cleanup.wait(5), "test did not release cleanup"
            self.worker.close()
            cleanup_finished_at.append(time.monotonic())
            events.append("cleanup_complete")

        def destroy_node(self):
            report["pending_at_destroy"] = self._hope_rpc_tracker.pending
            result = super().destroy_node()
            events.append("node_destroyed")
            return result

    class ObservedExecutor(MultiThreadedExecutor):
        def shutdown(self, **kwargs):
            report["shutdown_arguments"] = kwargs
            result = super().shutdown(**kwargs)
            events.append("executor_shutdown")
            return result

    rclpy.init()
    node = DrainNode()
    probe = rclpy.create_node("drain_timeout_client", start_parameter_services=False)
    executor = ObservedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(probe)
    client = probe.create_client(Trigger, "/test/drain_timeout")
    class SuspendedResponseFuture(Future):
        def __await__(self):
            result = yield from super().__await__()
            # The real worker is finished, but its service coroutine can never
            # finish. This yields a real rclpy Future without blocking a thread.
            yield from Future.__await__(never_completed)
            return result

        def add_done_callback(self, callback):
            super().add_done_callback(callback)
            if self is never_completed:
                # Confirm Task accepted the yielded future and registered its
                # resume callback; a failed executor thread is not suspension.
                response_waiting.set()

    # Task checks against rclpy.task.Future at runtime, so this must also be an
    # instance of the patched class. Explicit base __await__ above prevents a
    # second injection when the service begins waiting on this unresolved one.
    never_completed = SuspendedResponseFuture(executor=executor)

    def hold_cleanup_past_response_timeout():
        try:
            assert cleanup_started.wait(3), "cleanup did not start"
            time.sleep(response_timeout_s * 2.5)
            assert "node_destroyed" not in events
            assert "executor_shutdown" not in events
            assert node.context.ok()
            assert node._hope_rpc_tracker.pending == 1
            report["cleanup_held_longer_than_response_timeout"] = True
        except BaseException as exc:
            controller_errors.append(exc)
        finally:
            events.append("cleanup_released")
            release_cleanup.set()

    with patch("rclpy.task.Future", SuspendedResponseFuture), \
            patch.object(node, "get_logger", return_value=logger):
        assert client.wait_for_service(timeout_sec=3), "test service was not discovered"
        accepted = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3
        while not response_waiting.is_set() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert response_waiting.is_set(), "service coroutine did not suspend"
        assert not accepted.done()
        controller = threading.Thread(target=hold_cleanup_past_response_timeout)
        controller.start()
        drain_and_destroy(node, executor, response_timeout_s=response_timeout_s)
        report["post_cleanup_elapsed_s"] = time.monotonic() - cleanup_finished_at[0]
        controller.join(1)
        assert not controller.is_alive()
        assert not controller_errors, repr(controller_errors)
        report["response_unanswered"] = not accepted.done()
        report["context_ok_before_shutdown"] = node.context.ok()
        report["diagnostics"] = [str(call.args[0]) for call in logger.error.call_args_list]

    probe.destroy_node()
    rclpy.try_shutdown()
    report["context_ok_after_shutdown"] = node.context.ok()
    events.append("context_shutdown")
    report_path.write_text(json.dumps(report))


if __name__ == "__main__" and len(sys.argv) == 3 and sys.argv[1] == "--child":
    _run_child(Path(sys.argv[2]))
