"""Jazzy reconnect deployment gate: real timers, executor and process teardown.

Run explicitly in isolated localhost domain 219. Backends and network-address
input are fixtures; production callbacks, executor and main remain intact.
This proves the process exit required by systemd, not a systemd restart itself.
"""
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


def test_network_timer_exits_nonzero_after_drain_and_replacement_starts_open(tmp_path):
    # A skipped test or a different distro cannot qualify this Jazzy gate.
    assert os.environ.get("ROS_DISTRO") == "jazzy", "source the intended Jazzy installation"
    reports = {}
    for mode in ("reconnect", "replacement"):
        report_path = tmp_path / f"{mode}.json"
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--child", mode, str(report_path)],
            capture_output=True, text=True, timeout=20, check=False,
        )
        output = result.stdout + result.stderr
        assert report_path.exists(), output
        report = json.loads(report_path.read_text())
        reports[mode] = report
        events = [event["event"] for event in report["events"]]
        assert report["ros_distro"] == "jazzy"
        assert report["initial_admission_open"] is True
        assert events.index("close_complete") < events.index("destroy_complete")
        assert events.index("destroy_complete") < events.index("context_shutdown")
        assert report["pending_rpcs_after_close"] == 0
        assert set(report["worker_states_after_close"]) == {"CLOSED"}
        assert report["context_ok_after_main"] is False
        assert "shutdown callback:" not in output, output
        assert "shutdown cleanup:" not in output, output
        if mode == "reconnect":
            # A timeout/signal kill, constructor error or directly invoked
            # _check_network() must never count as the required failure exit.
            assert result.returncode == 1, output
            assert "main_returned" not in events
            assert events.index("network_input_changed") < events.index("reconnect_sealed")
            assert events.index("reconnect_sealed") < events.index("close_complete")
            statuses = report["operations_status_samples"]
            reconnect = next(i for i, value in enumerate(statuses) if value.startswith("DDS_RECONNECTING:"))
            draining = next(i for i, value in enumerate(statuses) if value.startswith("DRAINING:"))
            assert reconnect < draining
            assert "DdsReconnectRequired: network interfaces changed" in output
            assert "_check_network" in output and "_spin_once_impl" in output
            assert "future.result()" in output and "executor.spin_once(" in output
        else:
            # A new process starts OPEN; no reopen transition is added to the
            # sealed instance. The fixture sends SIGTERM only after a real
            # network timer has established its stable initial signature.
            assert result.returncode == 0, output
            assert "replacement_ready_open" in events
            assert "reconnect_sealed" not in events
            assert events[-1] == "main_returned"
    assert reports["reconnect"]["pid"] != reports["replacement"]["pid"]


def _run_child(mode, report_path):
    import inspect
    import signal
    import tempfile
    from unittest.mock import Mock, patch

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.publisher import Publisher

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
    import hope_lifecycle_supervisor as lifecycle
    import hope_operations as operations
    from hope_operations_core import WorkerAdmission

    old_address = (("fixture0", "192.0.2.1"),)
    new_address = (("fixture0", "192.0.2.2"),)
    report = {"pid": os.getpid(), "ros_distro": os.environ.get("ROS_DISTRO"),
              "rclpy_executor_source": inspect.getfile(MultiThreadedExecutor), "events": [],
              "operations_status_samples": []}
    owner = {}

    def record(event):
        report["events"].append({"event": event})

    def network_input():
        # Wait until the production timer has latched A before introducing B.
        # The actual CachedCall refresh and one-second timer discover the change.
        node = owner.get("node")
        if mode == "reconnect" and node is not None and node._dds_interfaces == old_address:
            if not owner.get("changed"):
                owner["changed"] = True
                record("network_input_changed")
            return new_address
        return old_address

    original_init = operations.HopeOperations.__init__
    original_close = operations.HopeOperations.close
    original_destroy = operations.HopeOperations.destroy_node
    original_seal_if = WorkerAdmission.seal_if
    original_shutdown = rclpy.try_shutdown
    original_publish = Publisher.publish

    def publish(publisher, message):
        result = original_publish(publisher, message)
        if publisher.topic_name == "/hope/operations/status":
            report["operations_status_samples"].append(message.data)
        return result

    def initialize(node):
        original_init(node)
        owner["node"] = node
        report["initial_admission_open"] = not node.shared_state.admission.closed
        record("constructed")
        if mode == "replacement":
            def stop_ready_fixture():
                if node._dds_interfaces == old_address:
                    timer.cancel()
                    assert not node.shared_state.admission.closed
                    record("replacement_ready_open")
                    os.kill(os.getpid(), signal.SIGTERM)
            timer = node.create_timer(0.05, stop_ready_fixture)

    def seal_if(admission, predicate):
        sealed = original_seal_if(admission, predicate)
        if sealed:
            assert admission.closed
            record("reconnect_sealed")
        return sealed

    def close(node):
        already_closed = node._closed
        original_close(node)
        if not already_closed:
            report["pending_rpcs_after_close"] = node._hope_rpc_tracker.pending
            report["worker_states_after_close"] = [worker._state for worker in (
                node.commands._command_workers, node.commands._urgent_workers,
                node.lifecycle._rpc_workers, node.lifecycle._record_workers,
                node.observer._runtime_cache._worker, node._network._worker)]
            record("close_complete")

    def destroy(node):
        result = original_destroy(node)
        record("destroy_complete")
        return result

    def shutdown(*args, **kwargs):
        result = original_shutdown(*args, **kwargs)
        record("context_shutdown")
        return result

    recorder = Mock()
    recorder.status.return_value = {"state": "OFF", "output": "", "error": ""}
    recorder.stop.return_value = recorder.status.return_value
    with tempfile.TemporaryDirectory() as temporary, \
            patch.object(lifecycle, "CONFIG_PATH", Path(temporary) / "config.json"), \
            patch.object(lifecycle, "LifecycleBackend", return_value=Mock()), \
            patch.object(lifecycle, "DistributedRecorder", return_value=recorder), \
            patch.object(operations, "read_ipv4_interface_signature", side_effect=network_input), \
            patch.object(operations.HopeOperations, "__init__", initialize), \
            patch.object(operations.HopeOperations, "close", close), \
            patch.object(operations.HopeOperations, "destroy_node", destroy), \
            patch.object(WorkerAdmission, "seal_if", seal_if), \
            patch.object(Publisher, "publish", publish), \
            patch.object(rclpy, "try_shutdown", shutdown):
        sys.argv = [sys.argv[0], "--ros-args", "-p", f"session_id_path:={temporary}/session",
                    "-p", f"runtime_root:={temporary}/runtime"]
        try:
            operations.main()
            record("main_returned")
        finally:
            report["context_ok_after_main"] = rclpy.ok()
            report_path.write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    assert sys.argv[1] == "--child"
    _run_child(sys.argv[2], Path(sys.argv[3]))
