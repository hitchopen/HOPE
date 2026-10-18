import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location(
    "catch_background", Path(__file__).resolve().parents[1] / "helpers/hope_catch_background.py")
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def test_only_explicit_camera_voice_and_watchdogs_match():
    assert guard.unwanted(["./agent", "--cfg_file_path=../config/agent/agent.yaml"])
    assert guard.unwanted(["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"])
    assert guard.unwanted(["/bin/bash", "../scripts/hal_audio/hal_audio_monitor.sh"])
    for argv in (["./hal_ethercat"], ["./agent", "other.yaml"],
                 ["./aimrt_main", "--cfg_file_path=../config/body_drive/record.yaml"],
                 ["python3", "hope_monitor.py"], ["./recordbag1"], ["./hal_remote"]):
        assert not guard.unwanted(argv)


def fake_environment(monkeypatch, respawn=False):
    states = {"agent": 12, "hal_audio": 13, "hal_hdu_camera": 14, "recordbag1": 15}
    calls = []
    def rpc(method, body):
        if method == "GetAllAppInfos":
            return {"infos": [{"app": name, "pid": pid} for name, pid in states.items()]}
        calls.append(body["app"])
        if not respawn:
            states[body["app"]] = -1
        return {"result": {"type": "Type_SUCCESS"}}
    ticks = iter(i * .5 for i in range(1000))
    monkeypatch.setattr(guard.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(guard.time, "sleep", lambda _: None)
    monkeypatch.setattr(guard, "rpc", rpc)
    monkeypatch.setattr(guard, "unwanted_processes", lambda: {})
    monkeypatch.setattr(guard, "ui_active", lambda: False)
    monkeypatch.setattr(guard, "wifi_only", lambda: False)
    receipt = dict(stop_results=[], terminated_watchdogs=[], terminated_leftovers=[])
    return receipt, calls, states


def test_cleanup_preserves_recording_and_other_apps(monkeypatch):
    receipt, calls, states = fake_environment(monkeypatch)
    guard.clean(receipt)
    assert set(calls) == set(guard.APPS)
    assert states["recordbag1"] == 15
    assert "after" in receipt


def test_respawning_background_rejects_start(monkeypatch):
    receipt, _, _ = fake_environment(monkeypatch, respawn=True)
    with pytest.raises(RuntimeError, match="respawning"):
        guard.clean(receipt)


def test_manager_failure_is_not_silently_accepted(monkeypatch):
    receipt, _, _ = fake_environment(monkeypatch)
    monkeypatch.setattr(guard, "rpc", Mock(side_effect=TimeoutError("manager unavailable")))
    with pytest.raises(TimeoutError):
        guard.clean(receipt)


@pytest.mark.parametrize("mode", ["OPTITRACK", "KERNEL"])
def test_launch_rejects_cleanup_failure_before_hardware_start(mode):
    import subprocess
    helper = Path(__file__).resolve().parents[1] / "helpers/hope-lifecycle"
    script = r'''source "$1"
require_user() { :; }
tmux_exists() { return 1; }
pgrep() { return 1; }
python3() { [[ "$1" == */hope_wifi_policy.py ]] && return 0; return 1; }
SESSION_ID=model21800_20261016T105100Z
FIELD_MODE="$2"
preflight_hdu
printf 'UNSAFE_CONTINUATION\n'
'''
    result = subprocess.run(["bash", "-c", script, "test", str(helper), mode],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "UNRELATED_BACKGROUND_NOT_STOPPED" in result.stdout
    assert "UNSAFE_CONTINUATION" not in result.stdout


@pytest.mark.parametrize("code,state,expected", [(0, "active", True), (3, "activating", True),
                                                (3, "inactive", False), (4, "unknown", False)])
def test_ui_probe_distinguishes_service_states(monkeypatch, code, state, expected):
    monkeypatch.setattr(guard.subprocess, "run", Mock(return_value=Mock(
        returncode=code, stdout=state + "\n", stderr="")))
    assert guard.ui_active() is expected


def test_ui_probe_failure_does_not_count_as_stopped(monkeypatch):
    monkeypatch.setattr(guard.subprocess, "run", Mock(return_value=Mock(
        returncode=1, stdout="", stderr="Failed to connect to bus")))
    with pytest.raises(RuntimeError, match="Cannot determine"):
        guard.ui_active()


def test_ui_stop_permission_failure_rejects_start(monkeypatch):
    import subprocess
    receipt, _, _ = fake_environment(monkeypatch)
    monkeypatch.setattr(guard, "ui_active", lambda: True)
    monkeypatch.setattr(guard.subprocess, "run", Mock(
        side_effect=subprocess.CalledProcessError(1, "sudo")))
    with pytest.raises(subprocess.CalledProcessError):
        guard.clean(receipt)


def test_process_identity_change_never_sends_signal(monkeypatch):
    monkeypatch.setattr(guard.os, "pidfd_open", lambda _: 99)
    close = Mock()
    send_signal = Mock()
    monkeypatch.setattr(guard.os, "close", close)
    monkeypatch.setattr(guard.signal, "pidfd_send_signal", send_signal)
    monkeypatch.setattr(guard.Path, "read_bytes", lambda _: b"./hal_ethercat\0")
    with pytest.raises(RuntimeError, match="identity changed"):
        guard.terminate(123, ["./agent", "--cfg_file_path=../config/agent/agent.yaml"])
    send_signal.assert_not_called()
    close.assert_called_once_with(99)


def test_failure_writes_receipt_and_returns_nonzero(monkeypatch, tmp_path):
    import json
    import sys
    receipt = tmp_path / "attempt.json"
    monkeypatch.setattr(sys, "argv", ["guard", "--receipt", str(receipt)])
    monkeypatch.setattr(guard, "clean", Mock(side_effect=TimeoutError("manager unavailable")))
    assert guard.main() == 1
    result = json.loads(receipt.read_text())
    assert result["result"] == "FAIL"
    assert "manager unavailable" in result["error"]


def test_cellular_is_opt_in_and_invalid_profile_fails(monkeypatch, tmp_path):
    marker = tmp_path / "profile"
    monkeypatch.setattr(guard, "WIFI_ONLY", marker)
    assert not guard.wifi_only()
    marker.write_text("wifi-only-v1\n")
    assert guard.wifi_only()
    marker.write_text("unexpected")
    with pytest.raises(RuntimeError, match="invalid"):
        guard.wifi_only()


def test_wifi_only_stops_cellular_and_records_readback(monkeypatch):
    receipt, calls, states = fake_environment(monkeypatch)
    monkeypatch.setattr(guard, "wifi_only", lambda: True)
    active = {"value": True}
    monkeypatch.setattr(guard, "service_active", lambda unit: active["value"])
    def stop(argv, **kwargs):
        assert argv == ["systemctl", "--no-ask-password", "stop", guard.CELLULAR]
        active["value"] = False
    monkeypatch.setattr(guard.subprocess, "run", stop)
    guard.clean(receipt)
    assert receipt["cellular"] == dict(profile="wifi-only-v1", was_active=True, active_after=False)
    assert states["recordbag1"] == 15


def test_cellular_respawn_rejects_start(monkeypatch):
    receipt, _, _ = fake_environment(monkeypatch)
    monkeypatch.setattr(guard, "wifi_only", lambda: True)
    monkeypatch.setattr(guard, "service_active", lambda _: True)
    monkeypatch.setattr(guard.subprocess, "run", Mock())
    with pytest.raises(RuntimeError, match="respawning"):
        guard.clean(receipt)


def test_cellular_stop_denied_rejects_start(monkeypatch):
    import subprocess
    receipt, _, _ = fake_environment(monkeypatch)
    monkeypatch.setattr(guard, "wifi_only", lambda: True)
    monkeypatch.setattr(guard, "service_active", lambda _: True)
    monkeypatch.setattr(guard.subprocess, "run", Mock(side_effect=subprocess.CalledProcessError(1, "systemctl")))
    with pytest.raises(subprocess.CalledProcessError):
        guard.clean(receipt)
