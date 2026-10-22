"""Exercise the actual start transition without creating ROS nodes or hardware."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_lifecycle_supervisor import HopeLifecycleSupervisor, LifecycleFailure


def supervisor():
    recorder = Mock()
    node = SimpleNamespace(
        _lock=threading.Lock(), _backend=Mock(),
        _field=SimpleNamespace(recorder=recorder, runner_selection=Mock(return_value=("OPTITRACK", "")), status=lambda: {}),
        _wait_for_authoritative_runner=Mock(), _on_event=Mock(),
        _state="STARTING", _busy=True, _hardware_operation_lock=None,
        get_logger=Mock(return_value=Mock()),
    )
    return node, recorder


def test_recording_starts_only_after_authoritative_runner_ready(monkeypatch):
    monkeypatch.setenv("HOPE_AUTO_RECORD_ON_START", "1")
    node, recorder = supervisor()
    order = []
    node._backend.start.side_effect = lambda *args, **kwargs: order.append("backend")
    node._wait_for_authoritative_runner.side_effect = lambda: order.append("ready")
    recorder.start.side_effect = lambda: order.append("record")
    HopeLifecycleSupervisor._run_start(node, object(), "test")
    assert order == ["backend", "ready", "record"]
    assert node._state == "RUNNING" and not node._busy
    assert node._last_result == "START_COMPLETE_RUNNER_PASSIVE"


def test_failed_start_does_not_record(monkeypatch):
    monkeypatch.setenv("HOPE_AUTO_RECORD_ON_START", "1")
    node, recorder = supervisor()
    node._backend.start.side_effect = LifecycleFailure("preflight failure")
    HopeLifecycleSupervisor._run_start(node, object(), "test")
    recorder.start.assert_not_called()
    assert node._state == "STOPPED" and not node._busy


def test_manual_recording_override(monkeypatch):
    monkeypatch.setenv("HOPE_AUTO_RECORD_ON_START", "0")
    node, recorder = supervisor()
    HopeLifecycleSupervisor._run_start(node, object(), "test")
    recorder.start.assert_not_called()
    assert node._state == "RUNNING"


def test_capture_failure_preserves_controller_state(monkeypatch):
    monkeypatch.setenv("HOPE_AUTO_RECORD_ON_START", "1")
    node, recorder = supervisor()
    recorder.start.side_effect = OSError("capture unavailable")
    HopeLifecycleSupervisor._run_start(node, object(), "test")
    assert node._state == "RUNNING" and not node._busy
    node.get_logger().error.assert_called_once()


def test_public_default_keeps_manual_recording(monkeypatch):
    monkeypatch.delenv("HOPE_AUTO_RECORD_ON_START", raising=False)
    node, recorder = supervisor()
    HopeLifecycleSupervisor._run_start(node, object(), "test")
    recorder.start.assert_not_called()
    assert node._state == "RUNNING"
