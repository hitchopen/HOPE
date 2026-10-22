"""Telemetry failures and slow publication must not interrupt operation ownership."""
from contextlib import ExitStack
import os
from pathlib import Path
import sys
import threading
import time
from unittest.mock import Mock, patch

import pytest

pytest.importorskip("rclpy")
pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "219",
    reason="requires isolated localhost domain 219",
)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float64MultiArray, String, UInt64
from std_srvs.srv import Trigger
import hope_lifecycle_supervisor as lifecycle
import hope_observer as observer_module
from hope_command_proxy import CommandProxyModule
from hope_operations import HopeOperations


@pytest.fixture
def runtime(tmp_path):
    recorder = Mock()
    recorder.status.return_value = {"state": "OFF", "output": "", "error": ""}
    recorder.stop.return_value = recorder.status.return_value

    def inert_command(_self, _request, response):
        response.success, response.message = True, "fixture response"
        return response

    with patch.object(lifecycle, "CONFIG_PATH", tmp_path / "config.json"), \
            patch.object(lifecycle, "LifecycleBackend", return_value=Mock()), \
            patch.object(lifecycle, "DistributedRecorder", return_value=recorder), \
            patch.object(CommandProxyModule, "_refresh_x_hit", inert_command), \
            patch("hope_operations.read_ipv4_interface_signature", return_value=None):
        rclpy.init(args=["--ros-args", "-p", f"session_id_path:={tmp_path}/session",
                        "-p", f"runtime_root:={tmp_path}/runtime"])
        node = HopeOperations()
        probe = rclpy.create_node("telemetry_isolation_probe", start_parameter_services=False)
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(node)
        executor.add_node(probe)

        def until(predicate, timeout=4):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert predicate(), "condition timed out"

        try:
            yield node, probe, executor, until
        finally:
            node.stop_accepting()
            node.close()
            executor.shutdown(wait_for_threads=True)
            probe.destroy_node()
            node.destroy_node()
            rclpy.try_shutdown()


def ball_message():
    message = PoseArray()
    message.header.frame_id = "table"
    message.poses = [Pose()]
    message.poses[0].position.x = 1.0
    return message


def test_blocked_snapshot_allows_ball_command_and_lifecycle_callbacks(runtime):
    node, probe, _executor, until = runtime
    observer = node.observer
    assert observer._snapshot_group is not node.default_callback_group
    assert observer._snapshot_timer.callback_group is observer._snapshot_group
    entered, release = threading.Event(), threading.Event()
    read = observer._runtime_snapshot

    def blocked_read():
        entered.set()
        assert release.wait(5)
        return read()

    beats = []
    probe.create_subscription(String, "/hope/lifecycle/state", lambda m: beats.append(m.data), 10)
    balls = probe.create_publisher(PoseArray, "/poses", 10)
    commands = probe.create_publisher(Float64MultiArray, "/racket/command_flat", 10)
    with patch.object(observer, "_runtime_snapshot", side_effect=blocked_read):
        try:
            until(entered.is_set)
            until(lambda: balls.get_subscription_count() and commands.get_subscription_count())
            before = len(beats)
            balls.publish(ball_message())
            commands.publish(Float64MultiArray(data=[2.]))

            def inputs_received():
                with observer._input_lock:
                    return len(observer._ball_receipts_ns) > 0 and observer._command_receipt_ns > 0

            until(lambda: inputs_received() and len(beats) >= before + 2)
            assert not release.is_set()
            assert node.lifecycle._state == "STOPPED"
        finally:
            release.set()


def test_telemetry_exceptions_are_visible_and_services_survive_then_recover(runtime):
    node, probe, _executor, until = runtime
    latest = {}
    for topic, kind, key in (
        ("/hope/operations/telemetry_fault", String, "fault"),
        ("/hope/operations/telemetry_fault_count", UInt64, "count"),
        ("/hope/observer_alive", Bool, "alive"),
    ):
        probe.create_subscription(kind, topic, lambda m, k=key: latest.__setitem__(k, m.data), 10)
    balls = probe.create_publisher(PoseArray, "/poses", 10)
    commands = probe.create_publisher(Float64MultiArray, "/racket/command_flat", 10)
    client = probe.create_client(Trigger, "/hope/refresh_x_hit")
    assert client.wait_for_service(timeout_sec=3)
    until(lambda: balls.get_subscription_count() and commands.get_subscription_count())
    expected_sources = {"ObserverModule._publish_snapshot", "ObserverModule._command_callback",
                        "ObserverModule._ball_callback", "LifecycleModule._publish",
                        "CommandProxyModule._poll_field_status"}
    marker_publisher = Mock()
    marker_publisher.publish.side_effect = ValueError("marker fixture fault")
    with ExitStack() as stack:
        stack.enter_context(patch.object(node.observer, "_runtime_snapshot", side_effect=ValueError("snapshot fixture fault")))
        stack.enter_context(patch.object(node.lifecycle._field_snapshot, "poll", side_effect=ValueError("field fixture fault")))
        stack.enter_context(patch.object(node.shared_state, "field_status", side_effect=ValueError("fanout fixture fault")))
        stack.enter_context(patch.object(observer_module, "decode_racket_packet", side_effect=IndexError("decode fixture fault")))
        stack.enter_context(patch.object(node.observer, "_ball_marker_publisher", marker_publisher))
        balls.publish(ball_message())
        commands.publish(Float64MultiArray(data=[2.]))
        until(lambda: latest.get("count", 0) >= 5 and latest.get("alive") is False
              and all(name in latest.get("fault", "") for name in expected_sources))
        response = client.call_async(Trigger.Request())
        until(response.done)
        assert response.result().success
        assert not node.shared_state.admission.closed
        assert not node._closed
    # Recover every source: the cumulative counter remains while active faults
    # clear, so a transient issue is distinguishable from a dead telemetry path.
    balls.publish(ball_message())
    commands.publish(Float64MultiArray(data=[2.]))
    until(lambda: latest.get("alive") is True and latest.get("fault", "").startswith("OK "))
    assert latest["count"] >= 5
