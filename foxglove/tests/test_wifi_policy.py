import importlib.util
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("wifi_policy", ROOT / "helpers/hope_wifi_policy.py")
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


def fake_radio(monkeypatch, *, connected, power=True, blocked=False, manual=False):
    state = dict(present=True, connected=connected, power_save=power,
                 scan_blocked=blocked, manual_scan=manual)
    monkeypatch.setattr(policy, "snapshot", lambda: dict(state))
    monkeypatch.setattr(policy, "block_scan", lambda value: state.update(scan_blocked=value))
    def run(*argv, **kwargs):
        if "set" in argv:
            state["power_save"] = False
        return ""
    monkeypatch.setattr(policy, "run", run)
    return state


def test_connected_disconnect_reconnect_releases_scans(monkeypatch):
    state = fake_radio(monkeypatch, connected=True)
    assert policy.enforce()["scan_blocked"]
    assert not state["power_save"]
    state["connected"] = False
    assert not policy.enforce()["scan_blocked"]
    state["connected"] = True
    assert policy.enforce()["scan_blocked"]


def test_explicit_manual_scan_window_and_expiry(monkeypatch):
    state = fake_radio(monkeypatch, connected=True, blocked=True, manual=True)
    assert not policy.enforce()["scan_blocked"]
    state["manual_scan"] = False
    assert policy.enforce()["scan_blocked"]


def test_disconnect_releases_before_power_setting_failure(monkeypatch):
    state = fake_radio(monkeypatch, connected=False, blocked=True)
    def fail(*args, **kwargs):
        raise RuntimeError("driver unavailable")
    monkeypatch.setattr(policy, "run", fail)
    with pytest.raises(RuntimeError):
        policy.enforce()
    assert not state["scan_blocked"]


@pytest.mark.parametrize("contents,expected", [("110", True), ("99", False),
    ("131", False), ("nan", False), ("broken", False)])
def test_manual_window_is_bounded(monkeypatch, tmp_path, contents, expected):
    window = tmp_path / "manual-until"
    window.write_text(contents)
    monkeypatch.setattr(policy, "WINDOW", window)
    monkeypatch.setattr(policy.time, "monotonic", lambda: 100)
    assert policy.manual_window() is expected


def test_manual_scan_failure_closes_window(monkeypatch, tmp_path):
    state = fake_radio(monkeypatch, connected=True, power=False, blocked=True)
    window = tmp_path / "manual-until"
    monkeypatch.setattr(policy, "WINDOW", window)
    def run(*argv, **kwargs):
        if argv[0] == policy.IW:
            raise subprocess.TimeoutExpired(argv, 10)
        return ""
    monkeypatch.setattr(policy, "run", run)
    with pytest.raises(subprocess.TimeoutExpired):
        policy.rescan()
    assert not window.exists()
    assert state["scan_blocked"]


def test_start_rejects_manual_scan_and_power_drift(monkeypatch):
    state = fake_radio(monkeypatch, connected=True, power=False, blocked=True)
    policy.verify()
    state["manual_scan"] = True
    with pytest.raises(RuntimeError, match="manual"):
        policy.verify()
    state.update(manual_scan=False, power_save=True)
    with pytest.raises(RuntimeError, match="power saving"):
        policy.verify()


def test_start_rejects_disabled_service_even_with_good_radio(monkeypatch):
    fake_radio(monkeypatch, connected=True, power=False, blocked=True)
    def run(*argv, **kwargs):
        if "is-enabled" in argv:
            raise subprocess.CalledProcessError(1, argv)
        return ""
    monkeypatch.setattr(policy, "run", run)
    with pytest.raises(subprocess.CalledProcessError):
        policy.verify()


def test_lifecycle_fails_before_robot_actions_when_policy_bad():
    script = '''source "$1"
require_user() { :; }
python3() { return 1; }
tmux_exists() { printf 'UNSAFE_CONTINUATION'; return 1; }
preflight_hdu
'''
    result = subprocess.run(["bash", "-c", script, "test", str(ROOT / "helpers/hope-lifecycle")],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "WIFI_POLICY_NOT_READY" in result.stdout
    assert "UNSAFE_CONTINUATION" not in result.stdout


def test_empty_driver_read_is_a_handled_error(monkeypatch, tmp_path):
    scan = tmp_path / "scan_deny"
    scan.write_text("")
    monkeypatch.setattr(policy, "SCAN", scan)
    with pytest.raises(RuntimeError, match="unrecognized"):
        policy.scan_blocked()
