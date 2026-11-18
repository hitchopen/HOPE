import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location(
    "catch_background", Path(__file__).resolve().parents[1] / "helpers/hope_catch_background.py")
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def test_only_agent_and_camera_match_and_startup_dependencies_are_preserved():
    assert guard.unwanted(["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"])
    assert guard.unwanted(["./agent", "--cfg_file_path=../config/agent/agent.yaml"])
    assert not guard.unwanted([])
    for argv in (["./hal_ethercat"], ["./agent", "other.yaml"],
                 ["./hal_audio", "--cfg_file_path=../config/hal_audio/hal_audio.yaml"],
                 ["/bin/bash", "../scripts/agent/process_monitor.sh"],
                 ["/bin/bash", "../scripts/hal_audio/hal_audio_monitor.sh"],
                 ["/bin/bash", "/agibot/software/v0/scripts/agent/start_agent.sh"],
                 ["./aimrt_main", "--cfg_file_path=../config/body_drive/record.yaml"],
                 ["python3", "hope_monitor.py"], ["./recordbag1"], ["./hal_remote"],
                 ["./hal_remote", "--cfg_file_path=../config/hal_remote/hal_remote_config.yaml"],
                 ["/bin/bash", "/agibot/software/v0/scripts/hal_remote/start_hal_remote.sh"],
                 ["./aimrt_main", "--cfg_file_path=../config/hal_remote/hal_remote_config.yaml"],
                 ["./flutter-pi", "--release", "-r", "90", "./"]):
        assert not guard.unwanted(argv)


def fake_environment(monkeypatch, respawn=False, ui_respawn=False):
    states = {"agent": 12, "hal_audio": 13, "hal_hdu_camera": 14, "recordbag1": 15,
              "hal_remote": 16}
    calls = []
    units = {guard.UI: True, guard.CELLULAR: True}
    commands = []
    def rpc(method, body):
        if method == "GetAllAppInfos":
            return {"infos": [{"app": name, "pid": pid} for name, pid in states.items()]}
        calls.append(body["app"])
        if not respawn:
            states[body["app"]] = -1
        return {"result": {"type": "Type_SUCCESS"}}
    def service_active(unit):
        assert unit in units
        return units[unit]
    def run(argv, **kwargs):
        commands.append(argv)
        if argv == ["sudo", "-n", "systemctl", "stop", guard.UI]:
            units[guard.UI] = ui_respawn
        elif argv == ["sudo", "-n", "systemctl", "stop", guard.CELLULAR]:
            units[guard.CELLULAR] = False
        else:
            raise AssertionError(f"unexpected command {argv}")
    ticks = iter(i * .5 for i in range(1000))
    monkeypatch.setattr(guard.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(guard.time, "sleep", lambda _: None)
    monkeypatch.setattr(guard, "rpc", rpc)
    monkeypatch.setattr(guard, "unwanted_processes", lambda: {})
    monkeypatch.setattr(guard, "service_active", service_active)
    monkeypatch.setattr(guard.subprocess, "run", run)
    receipt = dict(stop_results=[], terminated_watchdogs=[], terminated_leftovers=[])
    env = dict(units=units, commands=commands)
    return receipt, calls, states, env


def test_cleanup_stops_agent_camera_face_display_and_cellular_only(monkeypatch):
    receipt, calls, states, env = fake_environment(monkeypatch)
    guard.clean(receipt)
    assert set(calls) == {"agent", "hal_hdu_camera"} == set(guard.APPS)
    assert states["agent"] == -1 and states["hal_hdu_camera"] == -1
    assert states["recordbag1"] == 15
    assert states["hal_remote"] == 16
    assert states["hal_audio"] == 13
    assert env["commands"] == [["sudo", "-n", "systemctl", "stop", guard.CELLULAR], ["sudo", "-n", "systemctl", "stop", guard.UI]]
    assert receipt["ui"] == {"unit": guard.UI, "was_active": True, "active_after": False}
    assert receipt["cellular"] == {"unit": guard.CELLULAR, "was_active": True, "active_after": False}
    assert "after" in receipt


def test_already_stopped_face_display_and_cellular_are_not_stopped_again(monkeypatch):
    # Also covers robots without zy_cellular.service: systemctl reports it inactive.
    receipt, _, _, env = fake_environment(monkeypatch)
    env["units"][guard.UI] = False
    env["units"][guard.CELLULAR] = False
    guard.clean(receipt)
    assert env["commands"] == []
    assert receipt["ui"] == {"unit": guard.UI, "was_active": False, "active_after": False}
    assert receipt["cellular"] == {"unit": guard.CELLULAR, "was_active": False, "active_after": False}


@pytest.mark.parametrize("cellular_running", [False, True])
def test_four_start_cleanups_restop_targets_and_preserve_handheld_stack(monkeypatch, cellular_running):
    _, calls, states, env = fake_environment(monkeypatch)
    protected = {name: pid for name, pid in states.items() if name not in guard.APPS}
    processes = {
        13: ["./hal_audio", "--cfg_file_path=../config/hal_audio/hal_audio.yaml"],
        16: ["./hal_remote", "--cfg_file_path=../config/hal_remote/hal_remote_config.yaml"],
        17: ["bash", "../scripts/agent/process_monitor.sh"],
        18: ["bash", "../scripts/hal_audio/hal_audio_monitor.sh"],
        19: ["./flutter-pi", "--release"],
    }
    protected_processes = dict(processes)
    monkeypatch.setattr(guard, "unwanted_processes", lambda: {
        pid: argv for pid, argv in processes.items() if guard.unwanted(argv)})

    def terminate(pid, argv):
        assert pid in (98, 99) and guard.unwanted(argv)
        del processes[pid]

    monkeypatch.setattr(guard, "terminate", terminate)
    for attempt in range(4):
        # Model vendor restoration (agibot_pm restore / reboot) between STARTs.
        states["agent"] = 200 + attempt
        states["hal_hdu_camera"] = 100 + attempt
        env["units"][guard.UI] = True
        env["units"][guard.CELLULAR] = cellular_running
        processes[98] = ["./agent", "--cfg_file_path=../config/agent/agent.yaml"]
        processes[99] = ["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"]
        receipt = dict(stop_results=[], terminated_watchdogs=[], terminated_leftovers=[])
        guard.clean(receipt)
        assert {name: states[name] for name in protected} == protected
        assert processes == protected_processes
        assert not env["units"][guard.UI] and not env["units"][guard.CELLULAR]
        assert receipt["terminated_watchdogs"] == []
        assert receipt["terminated_leftovers"] == [98, 99]
    assert sorted(calls) == sorted(["agent", "hal_hdu_camera"] * 4)


def test_respawning_background_rejects_start(monkeypatch):
    receipt, _, _, _ = fake_environment(monkeypatch, respawn=True)
    with pytest.raises(RuntimeError, match="respawning"):
        guard.clean(receipt)


def test_respawning_face_display_rejects_start(monkeypatch):
    receipt, _, _, _ = fake_environment(monkeypatch, ui_respawn=True)
    with pytest.raises(RuntimeError, match="ui_active=True"):
        guard.clean(receipt)


def test_face_display_stop_denied_rejects_start(monkeypatch):
    import subprocess
    receipt, calls, _, _ = fake_environment(monkeypatch)
    monkeypatch.setattr(guard.subprocess, "run", Mock(side_effect=subprocess.CalledProcessError(1, "sudo")))
    with pytest.raises(subprocess.CalledProcessError):
        guard.clean(receipt)
    assert calls == []


def test_manager_failure_is_not_silently_accepted(monkeypatch):
    receipt, _, _, _ = fake_environment(monkeypatch)
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


def test_cleanup_controls_only_the_cellular_and_face_display_units(monkeypatch):
    receipt, _, _, env = fake_environment(monkeypatch)
    guard.clean(receipt)
    assert env["commands"] == [["sudo", "-n", "systemctl", "stop", "zy_cellular.service"],
                               ["sudo", "-n", "systemctl", "stop", "agibot_ui.service"]]
    assert guard.APPS == ("agent", "hal_hdu_camera")


def test_cleanup_never_signals_live_startup_dependency_processes(monkeypatch):
    receipt, calls, _, _ = fake_environment(monkeypatch)
    processes = {
        13: ["./hal_audio", "--cfg_file_path=../config/hal_audio/hal_audio.yaml"],
        17: ["/bin/bash", "../scripts/agent/process_monitor.sh"],
        18: ["/bin/bash", "../scripts/hal_audio/hal_audio_monitor.sh"],
        19: ["./flutter-pi", "--release", "-r", "90", "./"],
    }
    monkeypatch.setattr(guard, "unwanted_processes",
                        lambda: {pid: argv for pid, argv in processes.items() if guard.unwanted(argv)})
    terminate = Mock(side_effect=AssertionError("must not signal startup dependencies"))
    monkeypatch.setattr(guard, "terminate", terminate)
    guard.clean(receipt)
    terminate.assert_not_called()
    assert sorted(calls) == ["agent", "hal_hdu_camera"]


def test_camera_leftovers_are_still_stopped_without_touching_other_apps(monkeypatch):
    receipt, calls, states, _ = fake_environment(monkeypatch)
    argv = ["./aimrt_main", "--cfg_file_path=../config/hdu_camera/hdu_camera_cfg.yaml"]
    remaining = {99: argv}
    monkeypatch.setattr(guard, "unwanted_processes", lambda: dict(remaining))

    def terminate(pid, current):
        assert pid == 99 and current == argv
        remaining.pop(pid)

    monkeypatch.setattr(guard, "terminate", terminate)
    guard.clean(receipt)
    assert sorted(calls) == ["agent", "hal_hdu_camera"]
    assert receipt["terminated_leftovers"] == [99]
    assert not remaining
    assert states["hal_audio"] == 13
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


def test_success_receipt_records_targets_and_preserves_startup_apps(monkeypatch, tmp_path):
    import json
    import sys
    _, calls, states, _ = fake_environment(monkeypatch)
    receipt = tmp_path / "attempt.json"
    monkeypatch.setattr(sys, "argv", ["guard", "--receipt", str(receipt)])
    assert guard.main() == 0
    result = json.loads(receipt.read_text())
    assert result["result"] == "PASS"
    assert result["allowed_apps"] == ["agent", "hal_hdu_camera"]
    assert result["ui"] == {"unit": "agibot_ui.service", "was_active": True, "active_after": False}
    assert result["cellular"] == {"unit": "zy_cellular.service", "was_active": True, "active_after": False}
    assert result["terminated_watchdogs"] == []
    assert sorted(calls) == ["agent", "hal_hdu_camera"]
    assert states["hal_audio"] == 13
    assert states["recordbag1"] == 15 and states["hal_remote"] == 16


def test_cellular_watchdog_is_stopped_every_start_without_any_profile(monkeypatch):
    assert not hasattr(guard, "wifi_only") and not hasattr(guard, "WIFI_ONLY")
    for attempt in range(3):
        receipt, _, states, env = fake_environment(monkeypatch)
        guard.clean(receipt)
        assert ["sudo", "-n", "systemctl", "stop", guard.CELLULAR] in env["commands"]
        assert receipt["cellular"]["active_after"] is False
        assert states["recordbag1"] == 15


def test_cellular_respawn_rejects_start(monkeypatch):
    receipt, _, _, _ = fake_environment(monkeypatch)
    monkeypatch.setattr(guard, "service_active", lambda _: True)
    monkeypatch.setattr(guard.subprocess, "run", Mock())
    with pytest.raises(RuntimeError, match="respawning"):
        guard.clean(receipt)


def test_cellular_stop_denied_rejects_start(monkeypatch):
    import subprocess
    receipt, _, _, _ = fake_environment(monkeypatch)
    monkeypatch.setattr(guard, "service_active", lambda _: True)
    monkeypatch.setattr(guard.subprocess, "run", Mock(side_effect=subprocess.CalledProcessError(1, "sudo")))
    with pytest.raises(subprocess.CalledProcessError):
        guard.clean(receipt)
