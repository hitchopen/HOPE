"""Parsed local fanout keeps the external wire and receipt freshness contract."""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_operations_core import OperationsState
from hope_observer_core import REQUIRED_BASE_FLAGS


def field_status():
    return {"calibration": {"sha256": "a" * 64, "name": "venue 校准", "profile": "v3"},
            "nested": [{"label": "original"}]}


def test_shared_field_snapshot_owns_parsed_data_and_preserves_wire():
    state = OperationsState()
    original = field_status()
    wire = json.dumps(original, ensure_ascii=False)
    state.set_field_status(wire, original)
    snapshot = state.field_status()
    original["calibration"]["name"] = "mutated producer"
    original["nested"][0]["label"] = "mutated producer"
    assert snapshot.wire == wire
    assert snapshot.parsed["calibration"]["name"] == "venue 校准"
    assert snapshot.parsed["nested"][0]["label"] == "original"
    with pytest.raises(TypeError):
        snapshot.parsed["calibration"]["name"] = "mutated consumer"
    with pytest.raises(TypeError):
        snapshot.parsed["nested"][0]["label"] = "mutated consumer"
    state.set_field_status("{}", {})
    assert state.field_status().wire == "{}"
    assert snapshot.wire == wire  # A reader can retain its previous snapshot.


@pytest.fixture
def proxy():
    pytest.importorskip("rclpy")
    from hope_command_proxy import CommandProxyModule
    module = CommandProxyModule.__new__(CommandProxyModule)
    module._node = SimpleNamespace(_hope_telemetry_faults=Mock())
    module._shared_state = OperationsState()
    module._base_packet_condition = module._shared_state.base.condition
    module._action_lock = threading.Lock()
    module._publish_calibration_status = Mock()
    return module


def test_composed_poll_skips_json_but_reevaluates_live_receipt_age(proxy):
    from hope_command_core import calibration_receipt_id
    parsed = field_status()
    proxy._shared_state.set_field_status(json.dumps(parsed, ensure_ascii=False), parsed)
    receipt = calibration_receipt_id(parsed["calibration"]["sha256"])
    packet = [2., 1., 5., 1., 0., 0., 0., .2, 1., 0., 0., 0., 1.,
              float(REQUIRED_BASE_FLAGS), float(receipt), 1.]
    with patch("hope_operations_core.time.monotonic_ns", return_value=100_000_000_000):
        proxy._shared_state.base.update(packet)
    with patch("hope_command_proxy.json.loads", side_effect=AssertionError("local fanout decoded JSON")):
        with patch("hope_command_proxy.time.monotonic", return_value=100.5):
            proxy._poll_field_status()
            assert proxy._publish_calibration_status.call_args.args == (
                True, "SAVED venue 校准 · LIVE RECEIPT VALID", "V3")
        # The shared metadata is unchanged. A later poll must still expire the
        # calibration display when the base receipt becomes stale.
        with patch("hope_command_proxy.time.monotonic", return_value=101.1):
            proxy._poll_field_status()
            assert proxy._publish_calibration_status.call_args.args == (
                False, "SAVED venue 校准 · WAITING FOR MATCHING LIVE BASE", "V3")
    proxy._node._hope_telemetry_faults.failed.assert_not_called()


@pytest.mark.parametrize("parsed,expected", [
    ({}, "NO SAVED CALIBRATION"),
    ({"calibration": {"sha256": "invalid"}}, "INVALID SAVED CALIBRATION STATUS"),
    (["not a status object"], "INVALID SAVED CALIBRATION STATUS"),
])
def test_composed_and_legacy_paths_keep_invalid_metadata_behavior(proxy, parsed, expected):
    from std_msgs.msg import String
    wire = json.dumps(parsed)
    proxy._shared_state.set_field_status(wire, parsed)
    proxy._poll_field_status()
    assert proxy._publish_calibration_status.call_args.args == (False, expected)
    proxy._publish_calibration_status.reset_mock()
    proxy._field_status_callback(String(data=wire))
    assert proxy._publish_calibration_status.call_args.args == (False, expected)
    proxy._node._hope_telemetry_faults.failed.assert_not_called()


def test_action_lock_suppresses_both_paths_and_next_poll_rechecks(proxy):
    from std_msgs.msg import String
    proxy._shared_state.set_field_status("{}", {})
    with proxy._action_lock:
        proxy._poll_field_status()
        proxy._field_status_callback(String(data="not JSON"))
    proxy._publish_calibration_status.assert_not_called()
    proxy._poll_field_status()
    proxy._publish_calibration_status.assert_called_once_with(False, "NO SAVED CALIBRATION")


@pytest.mark.parametrize("native", [False, True])
def test_lifecycle_external_field_wire_is_unchanged_by_parsed_fanout(monkeypatch, native):
    monkeypatch.delenv("HOPE_AUTO_RECORD_ON_START", raising=False)
    pytest.importorskip("rclpy")
    from hope_lifecycle_supervisor import LifecycleConfig, LifecycleModule
    module = LifecycleModule.__new__(LifecycleModule)
    module._node = SimpleNamespace(_hope_telemetry_faults=Mock())
    module._lock = threading.Lock()
    module._config = LifecycleConfig()
    module._state, module._step, module._session_id = "STOPPED", "IDLE", ""
    module._busy, module._stopping, module._field_busy = False, False, False
    module._last_result = "fixture"
    from hope_player_positions import PlayerPositions
    module._player_positions = PlayerPositions()
    if native:
        from hope_operations_native_state import NativePlayerPositions
        def status(*args, **kwargs):
            assert module._lock.acquire(blocking=False), "native player IPC held lifecycle lock"
            module._lock.release()
            return PlayerPositions().status(0)
        module._player_positions = NativePlayerPositions(SimpleNamespace(query=status))
    module._elink_sequence = 0
    module._elink_status_locked = Mock(return_value={"state": "UNKNOWN"})
    module._elink_publisher = Mock()
    raw = field_status()
    module._field_snapshot = Mock(error="")
    module._field_snapshot.poll.return_value = raw
    module._shared_state = OperationsState()
    module._field_publisher = Mock()
    module._lifecycle_publishers = {key: Mock() for key in
                                   ("state", "step", "session", "result", "summary", "busy", "revision")}
    module._config_publishers = {key: Mock() for key in module._config.values()}
    module._mocap_config_pub = Mock()
    module._publish()
    expected = json.dumps({**raw, "auto_record_on_start": False, "busy": False, "draining": False,
                           "player_positions": module._player_positions.status(0),
                           "elink_observation": {"state": "UNKNOWN"},
                           "ball_model_verification": None}, ensure_ascii=False)
    assert module._field_publisher.publish.call_args.args[0].data == expected
    assert module._shared_state.field_status().wire == expected
    assert module._shared_state.field_status().parsed["calibration"]["name"] == "venue 校准"
    module._node._hope_telemetry_faults.failed.assert_not_called()
