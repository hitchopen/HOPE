"""Vendor restore failures must never become lifecycle STOP success."""
import ast
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
HELPER = (ROOT / "helpers/hope-lifecycle").read_text()


@pytest.fixture
def probe():
    code = HELPER.split("verify_vendor_mdu() {", 1)[1].split("<<'PY'\n", 1)[1].split("\nPY\n", 1)[0]
    module = ModuleType("vendor_restore_probe")
    exec(compile(code, "hope-lifecycle:verify_vendor_mdu", "exec"), module.__dict__)
    return module


@pytest.fixture
def replies(probe):
    replies = {
        "GetSystemState": {"cur_state": "Ready", "cur_status": "SystemStatus_IN_READY"},
        "GetAllAppInfos": {"infos": [{"app": app, "state": "State_RUNNING", "pid": n + 10}
                                  for n, app in enumerate(("sm", "mc", "skillpilot", "hal_ethercat", "hal_elink"))]},
        "GetSmModuleStatus": {"states": [{"node_name": name, "module_states": {"health_monitor": {}}}
                                        for name in ("health_monitor0", "health_monitor1")]},
        "GetAction": {"info": {"current_action": "MotionControlAction_PASSIVE",
                               "status": "MotionControlActionStatus_RUNNING"}},
    }
    probe.rpc = Mock(side_effect=lambda _port, _service, method: replies[method])
    return replies


def test_failed_fsm_with_in_ready_is_not_restored(probe, replies):
    replies["GetSystemState"]["cur_state"] = "Failed"
    assert probe.verify_vendor_restore() == (False, "VENDOR_SM_FAILED")
    assert probe.rpc.call_count == 1


@pytest.mark.parametrize("mode,status", [("Startup", "SystemStatus_IN_READY"),
                                        ("Ready", "SystemStatus_IN_INITIAL"), (None, None)])
def test_transition_or_missing_state_is_not_restored(probe, replies, mode, status):
    replies["GetSystemState"] = {"cur_state": mode, "cur_status": status}
    assert probe.vendor_restore_status() == "VENDOR_SM_NOT_READY"


@pytest.mark.parametrize("change", ["missing", "idle", "dead", "duplicate"])
def test_pm_and_elink_running_cannot_hide_missing_mc(probe, replies, change):
    infos = replies["GetAllAppInfos"]["infos"]
    mc = next(x for x in infos if x["app"] == "mc")
    if change == "missing": infos.remove(mc)
    elif change == "idle": mc["state"] = "State_IDLE"
    elif change == "dead": mc["pid"] = -1
    else: infos.append(dict(mc))
    assert probe.vendor_restore_status() == "VENDOR_REQUIRED_APP_NOT_RUNNING"


def test_missing_hdu_health_monitor_rejected(probe, replies):
    replies["GetSmModuleStatus"]["states"].pop()
    assert probe.vendor_restore_status() == "VENDOR_HEALTH_MONITOR_UNAVAILABLE"


def test_success_requires_two_live_snapshots(probe, replies, monkeypatch):
    monkeypatch.setattr(probe.time, "sleep", lambda _: None)
    assert probe.verify_vendor_restore() == (True, "AGIBOT_PM_RESTORED")
    assert probe.rpc.call_count == 8


@pytest.mark.parametrize("info", [{}, {"current_action": "MotionControlAction_UNKNOWN"},
                                  {"current_action": "MotionControlAction_STAND",
                                   "status": "MotionControlActionStatus_RUNNING"},
                                  {"current_action": "MotionControlAction_PASSIVE"}])
def test_mc_process_without_live_passive_action_is_not_restored(probe, replies, info):
    replies["GetAction"]["info"] = info
    assert probe.vendor_restore_status() == "VENDOR_MC_NOT_PASSIVE"


def test_unavailable_probe_times_out_closed(probe, monkeypatch):
    clock = iter([0, 0, 1, 2, 3])
    monkeypatch.setattr(probe.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(probe.time, "sleep", lambda _: None)
    probe.rpc = Mock(side_effect=OSError("unreachable"))
    assert probe.verify_vendor_restore(timeout_s=3) == (False, "VENDOR_RESTORE_UNAVAILABLE")


def test_rpc_rejects_missing_or_error_header(probe, monkeypatch):
    import io
    for body in (b'{}', b'{"header":{"code":"7"}}', b'not json'):
        monkeypatch.setattr(probe.urllib.request, "urlopen", lambda *a, **kw: io.BytesIO(body))
        with pytest.raises(ValueError):
            probe.rpc(51011, "SystemService", "GetSystemState")


def test_failed_restore_still_stops_hdu_and_collects():
    # Execute the actual backend method with external helpers replaced. ROS is
    # deliberately absent: this is command ordering/error propagation only.
    tree = ast.parse((ROOT / "a3/hope_lifecycle_supervisor.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LifecycleBackend")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "kill_all_and_collect")
    import subprocess
    class Failure(RuntimeError): pass
    namespace = dict(LifecycleConfig=object, validate_session_id=lambda _: None,
                     ROBOT_USER="agi", LIFECYCLE_HELPER="helper", HelperEvent=lambda **kw: kw,
                     LifecycleFailure=Failure, subprocess=subprocess,
                     parse_helper_event=lambda line: SimpleNamespace(step="COLLECT", reason="LOGS_COLLECTED"))
    exec(compile(ast.Module(body=[method], type_ignores=[]), "backend", "exec"), namespace)
    calls = []
    def invoke(argv, **kwargs):
        calls.append(argv)
        if "verify-vendor-mdu" in argv: raise Failure("VENDOR_SM_FAILED")
        return "collected"
    backend = SimpleNamespace(_laptop=lambda _: ("laptop", "helper"),
                              _ssh=lambda *args: list(args), _invoke=invoke)
    config = SimpleNamespace(mdu_internal_ip="10.42.10.12", laptop_wifi_ip="192.168.1.2",
                             hdu_wifi_ip="192.168.1.3", motive_ip="", table_side="P1")
    with pytest.raises(Failure, match="VENDOR_SM_FAILED"):
        namespace["kill_all_and_collect"](backend, config, "session", lambda _: None)
    assert [next(x for x in argv if x in {"stop-mdu", "stop-hdu", "verify-vendor-mdu", "stop-capture-laptop", "collect"})
            for argv in calls] == ["stop-mdu", "stop-hdu", "verify-vendor-mdu", "stop-capture-laptop", "collect"]
