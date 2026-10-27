import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location(
    "catch_background", Path(__file__).resolve().parents[1] / "helpers/hope_catch_background.py")
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def test_only_explicit_camera_matches_and_startup_dependencies_are_preserved():
    assert guard.unwanted(["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"])
    assert not guard.unwanted([])
    for argv in (["./hal_ethercat"], ["./agent", "other.yaml"],
                 ["./agent", "--cfg_file_path=../config/agent/agent.yaml"],
                 ["./hal_audio", "--cfg_file_path=../config/hal_audio/hal_audio.yaml"],
                 ["/bin/bash", "../scripts/agent/process_monitor.sh"],
                 ["/bin/bash", "../scripts/hal_audio/hal_audio_monitor.sh"],
                 ["./aimrt_main", "--cfg_file_path=../config/body_drive/record.yaml"],
                 ["python3", "hope_monitor.py"], ["./recordbag1"], ["./hal_remote"],
                 ["./hal_remote", "--cfg_file_path=../config/hal_remote/hal_remote_config.yaml"],
                 ["/bin/bash", "/agibot/software/v0/scripts/hal_remote/start_hal_remote.sh"],
                 ["./aimrt_main", "--cfg_file_path=../config/hal_remote/hal_remote_config.yaml"],
                 ["./flutter-pi", "--release", "-r", "90", "./"]):
        assert not guard.unwanted(argv)


def fake_environment(monkeypatch, respawn=False):
    states = {"agent": 12, "hal_audio": 13, "hal_hdu_camera": 14, "recordbag1": 15,
              "hal_remote": 16}
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
    monkeypatch.setattr(guard, "wifi_only", lambda: False)
    receipt = dict(stop_results=[], terminated_watchdogs=[], terminated_leftovers=[])
    return receipt, calls, states


def test_cleanup_preserves_recording_and_other_apps(monkeypatch):
    receipt, calls, states = fake_environment(monkeypatch)
    guard.clean(receipt)
    assert set(calls) == set(guard.APPS)
    assert states["recordbag1"] == 15
    assert states["hal_remote"] == 16
    assert states["agent"] == 12
    assert states["hal_audio"] == 13
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
prepare_background_hdu
printf 'UNSAFE_CONTINUATION\n'
'''
    result = subprocess.run(["bash", "-c", script, "test", str(helper), mode],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "UNRELATED_BACKGROUND_NOT_STOPPED" in result.stdout
    assert "UNSAFE_CONTINUATION" not in result.stdout


def test_readonly_preflight_does_not_contain_background_mutations():
    helper = Path(__file__).resolve().parents[1] / "helpers/hope-lifecycle"
    text = helper.read_text()
    preflight = text.split("preflight_hdu() {", 1)[1].split("\n}\n", 1)[0]
    assert "hope_catch_background.py" not in preflight
    background = text.split("prepare_background_hdu() {", 1)[1].split("\n}\n", 1)[0]
    assert "hope_catch_background.py" in background


def test_restored_vendor_children_must_be_observed_before_kill_completion():
    import subprocess
    helper = Path(__file__).resolve().parents[1] / "helpers/hope-lifecycle"
    script = r'''source "$1"
require_user() { :; }
stop_tmux() { :; }
sudo() { :; }
systemctl() { return 0; }
check_elink_process_presence() {
  [[ "$1" == RUNNER_HAL && "$2" == 8 ]] || exit 98
  die RUNNER_HAL ELINK_VENDOR_STACK_NOT_READY
}
stop_mdu
'''
    result = subprocess.run(["bash", "-c", script, "test", str(helper)], capture_output=True, text=True)
    assert result.returncode == 1 and "ELINK_VENDOR_STACK_NOT_READY" in result.stdout
    assert "AGIBOT_PM_RESTORED" not in result.stdout


@pytest.mark.parametrize("code,state,expected", [(0, "active", True), (3, "activating", True),
                                                (3, "inactive", False), (4, "unknown", False)])
def test_service_probe_distinguishes_service_states(monkeypatch, code, state, expected):
    monkeypatch.setattr(guard.subprocess, "run", Mock(return_value=Mock(
        returncode=code, stdout=state + "\n", stderr="")))
    assert guard.service_active(guard.CELLULAR) is expected


def test_service_probe_failure_does_not_count_as_stopped(monkeypatch):
    monkeypatch.setattr(guard.subprocess, "run", Mock(return_value=Mock(
        returncode=1, stdout="", stderr="Failed to connect to bus")))
    with pytest.raises(RuntimeError, match="Cannot determine"):
        guard.service_active(guard.CELLULAR)


def test_normal_cleanup_never_probes_or_stops_vendor_ui_or_remote(monkeypatch):
    receipt, _, _ = fake_environment(monkeypatch)
    command = Mock(side_effect=AssertionError("normal cleanup must not control UI/remote services"))
    monkeypatch.setattr(guard.subprocess, "run", command)
    guard.clean(receipt)
    command.assert_not_called()
    assert guard.APPS == ("hal_hdu_camera",)


def test_cleanup_never_signals_live_startup_dependency_processes(monkeypatch):
    receipt, calls, _ = fake_environment(monkeypatch)
    processes = {
        12: ["./agent", "--cfg_file_path=../config/agent/agent.yaml"],
        13: ["./hal_audio", "--cfg_file_path=../config/hal_audio/hal_audio.yaml"],
        17: ["/bin/bash", "../scripts/agent/process_monitor.sh"],
        18: ["/bin/bash", "../scripts/hal_audio/hal_audio_monitor.sh"],
    }
    monkeypatch.setattr(guard, "unwanted_processes",
                        lambda: {pid: argv for pid, argv in processes.items() if guard.unwanted(argv)})
    terminate = Mock(side_effect=AssertionError("must not signal startup dependencies"))
    monkeypatch.setattr(guard, "terminate", terminate)
    guard.clean(receipt)
    terminate.assert_not_called()
    assert calls == ["hal_hdu_camera"]


def test_camera_leftovers_are_still_stopped_without_touching_other_apps(monkeypatch):
    receipt, calls, states = fake_environment(monkeypatch)
    argv = ["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"]
    remaining = {99: argv}
    monkeypatch.setattr(guard, "unwanted_processes", lambda: dict(remaining))

    def terminate(pid, current):
        assert pid == 99 and current == argv
        remaining.pop(pid)

    monkeypatch.setattr(guard, "terminate", terminate)
    guard.clean(receipt)
    assert calls == ["hal_hdu_camera"]
    assert receipt["terminated_leftovers"] == [99]
    assert not remaining
    assert states["agent"] == 12 and states["hal_audio"] == 13
    assert states["recordbag1"] == 15 and states["hal_remote"] == 16


def test_vendor_launch_default_strings_static_tripwire():
    root = Path(__file__).resolve().parents[1]
    paths = [root / "helpers/hope-lifecycle", *sorted((root / "a3").glob("install_*.sh"))]
    for path in paths:
        source = path.read_text()
        # Limited static tripwire for these known entry points, not proof that
        # every script preserves vendor configuration. This also passes on the
        # original main; runtime protection is covered by the cleanup/signal tests.
        assert "A3_DEFAULT.yaml" not in source, path
        assert "default_apps" not in source, path


def test_process_identity_change_never_sends_signal(monkeypatch):
    monkeypatch.setattr(guard.os, "pidfd_open", lambda _: 99)
    close = Mock()
    send_signal = Mock()
    monkeypatch.setattr(guard.os, "close", close)
    monkeypatch.setattr(guard.signal, "pidfd_send_signal", send_signal)
    monkeypatch.setattr(guard.Path, "read_bytes", lambda _: b"./hal_ethercat\0")
    with pytest.raises(RuntimeError, match="identity changed"):
        guard.terminate(123, ["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"])
    send_signal.assert_not_called()
    close.assert_called_once_with(99)


def test_matching_camera_process_still_receives_sigterm(monkeypatch):
    argv = ["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"]
    monkeypatch.setattr(guard.os, "pidfd_open", lambda _: 99)
    close = Mock()
    send_signal = Mock()
    monkeypatch.setattr(guard.os, "close", close)
    monkeypatch.setattr(guard.signal, "pidfd_send_signal", send_signal)
    monkeypatch.setattr(guard.Path, "read_bytes", lambda _: ("\0".join(argv) + "\0").encode())
    guard.terminate(123, argv)
    send_signal.assert_called_once_with(99, guard.signal.SIGTERM)
    close.assert_called_once_with(99)


@pytest.mark.parametrize("argv", [
    ["./agent", "--cfg_file_path=../config/agent/agent.yaml"],
    ["./hal_audio", "--cfg_file_path=../config/hal_audio/hal_audio.yaml"],
    ["bash", "../scripts/agent/process_monitor.sh"],
    ["bash", "../scripts/hal_audio/hal_audio_monitor.sh"],
    ["./hal_remote", "--cfg_file_path=../config/hal_remote/hal_remote_config.yaml"],
    ["./flutter-pi", "--release"],
])
def test_terminate_refuses_protected_dependencies_even_with_matching_identity(monkeypatch, argv):
    monkeypatch.setattr(guard.os, "pidfd_open", lambda _: 99)
    close = Mock()
    send_signal = Mock()
    monkeypatch.setattr(guard.os, "close", close)
    monkeypatch.setattr(guard.signal, "pidfd_send_signal", send_signal)
    monkeypatch.setattr(guard.Path, "read_bytes", lambda _: ("\0".join(argv) + "\0").encode())
    with pytest.raises(RuntimeError):
        guard.terminate(123, argv)
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


def test_success_receipt_keeps_watchdog_key_and_preserves_startup_apps(monkeypatch, tmp_path):
    import json
    import sys
    _, calls, states = fake_environment(monkeypatch)
    receipt = tmp_path / "attempt.json"
    monkeypatch.setattr(sys, "argv", ["guard", "--receipt", str(receipt)])
    monkeypatch.setattr(guard.subprocess, "run", Mock(
        side_effect=AssertionError("normal cleanup must not control units")))
    assert guard.main() == 0
    result = json.loads(receipt.read_text())
    assert result["result"] == "PASS"
    assert result["allowed_apps"] == ["hal_hdu_camera"]
    assert result["terminated_watchdogs"] == []
    assert calls == ["hal_hdu_camera"]
    assert states["agent"] == 12 and states["hal_audio"] == 13
    assert states["recordbag1"] == 15 and states["hal_remote"] == 16


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
