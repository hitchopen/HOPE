"""Observe-only ELink diagnostics and verified, retryable START checks.

Actual supervisor methods use in-memory backends; no hardware/clock mutations.
"""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import json
import sys
import threading
import time

import pytest
pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_lifecycle_supervisor import HopeLifecycleSupervisor as Supervisor, LifecycleFailure
from hope_elink_health import ElinkHealthLatch, PassiveSnapshot, ProcessIdentity, ProbeError
from hope_elink_fault import PersistentElinkFault
from hope_elink_telemetry import ElinkTelemetryTracker
import hope_lifecycle_supervisor as module

BOOT = "12345678-1234-1234-1234-123456789abc"


def snapshot(listener=True, hal_pid=20):
    return PassiveSnapshot(BOOT, (ProcessIdentity(BOOT, 10, 100),),
                           (ProcessIdentity(BOOT, hal_pid, 200),), listener)


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv("HOPE_AUTO_RECORD_ON_START", "0")
    monkeypatch.setattr(module, "try_acquire_hardware_operation_lock", Mock(return_value=object()))
    monkeypatch.setattr(module, "release_hardware_operation_lock", Mock())
    monkeypatch.setattr(module.time, "sleep", Mock())
    n = SimpleNamespace(
        _lock=threading.Lock(), _stopping=False, _closed=False, _state="RUNNING", _busy=False, _field_busy=False,
        _session_id="session-a", _session_mode="KERNEL", _step="RUNNER",
        _last_result="START_COMPLETE_RUNNER_PASSIVE", _config=SimpleNamespace(revision=1),
        _elink_fault=PersistentElinkFault(tmp_path / "fault.json"),
        _elink_health=ElinkHealthLatch(), _elink_sequence=1, _elink_observed_at=time.monotonic(),
        _elink_monitor_id="observer", _elink_clock_boot_id=BOOT, _elink_observing=False,
        # Exercise default unaudited telemetry and missing decoder, not a fake audit.
        _elink_telemetry=ElinkTelemetryTracker(),
        _elink_telemetry_subscriptions=SimpleNamespace(available=False),
        _elink_clock_received=None, _elink_clock_qualified=False, _elink_audit={},
        _backend=Mock(), _pool=Mock(), _elink_pool=Mock(), _on_event=Mock(),
        _run_start=Mock(), _run_kill=Mock(), _run_elink_observation=Mock(),
        _field=SimpleNamespace(recorder=Mock(), runner_selection=Mock(return_value=("KERNEL", "")), status=lambda: {"mode": "KERNEL"}),
        _wait_for_authoritative_runner=Mock(), _hardware_operation_lock=None,
        get_logger=Mock(return_value=Mock()))
    n._elink_health.observe(snapshot(), n._elink_observed_at)
    n._backend.observe_elink.return_value = snapshot()
    n._backend.start.side_effect = lambda *a, **kw: kw["before_mutation"]()
    n._backend.kill_all_and_collect.return_value = "LOGS_RETAINED_ON_ROBOT"
    n._bind_elink_telemetry_locked = lambda value, now: Supervisor._bind_elink_telemetry_locked(n, value, now)
    return n


def observe(n):
    Supervisor._run_elink_observation(n, n._config, n._session_id)


def execute_start(n):
    args = n._pool.submit.call_args.args
    Supervisor._run_start(n, *args[1:])


def telemetry(n, fault=False):
    n._elink_telemetry = Mock()
    n._elink_telemetry.status.return_value = dict(
        message_fresh=False, functional_ready=False, fault_latched=fault,
        reason="DEVICE_FAULT" if fault else "CLOCK_UNQUALIFIED", valid_until_monotonic_s=None)


@pytest.mark.parametrize("condition", ["defaults", "stale_clock", "telemetry_fault"])
def test_diagnostics_do_not_block_normal_start(node, condition):
    node._state = "STOPPED"
    if condition != "defaults":
        telemetry(node, fault=condition == "telemetry_fault")
        node._elink_clock_received = time.monotonic() - 10
    assert Supervisor._start(node, None, SimpleNamespace()).success
    execute_start(node)
    assert node._state == "RUNNING" and not node._busy
    node._backend.start.assert_called_once()
    node._wait_for_authoritative_runner.assert_called_once()
    assert not node._elink_fault.path.exists()
    assert not node._elink_fault.active_path.exists()


def test_listener_loss_is_visible_without_changing_lifecycle_or_hardware(node):
    node._backend.observe_elink.return_value = snapshot(False)
    observe(node)
    value = Supervisor._elink_status_locked(node, time.monotonic())
    assert value["state"] == "FAULT" and value["reason"] == "ELINK_IPC_NOT_LISTENING"
    assert value["admission_enforced"] is False
    assert node._state == "RUNNING" and not node._elink_fault.reason
    assert node._last_result == "START_COMPLETE_RUNNER_PASSIVE"
    node._backend.kill_all_and_collect.assert_not_called()
    node._backend.start.assert_not_called()
    node._pool.submit.assert_not_called()


def test_recovered_listener_updates_diagnostic_without_automatic_action(node):
    node._backend.observe_elink.return_value = snapshot(False)
    observe(node)
    node._backend.observe_elink.return_value = snapshot()
    observe(node)
    value = Supervisor._elink_status_locked(node, time.monotonic())
    assert value["state"] == "OBSERVED" and not value["fault_latched"]
    assert node._state == "RUNNING" and not node._elink_fault.reason
    node._backend.kill_all_and_collect.assert_not_called()


def test_old_session_result_discarded(node):
    node._backend.observe_elink.side_effect = lambda config: (setattr(node, "_session_id", "new") or snapshot(False))
    Supervisor._run_elink_observation(node, node._config, "session-a")
    assert node._elink_sequence == 1 and not node._elink_fault.reason


def test_repeated_observer_misses_are_unknown_not_permanent_fault(node):
    node._backend.observe_elink.side_effect = ProbeError("timeout")
    for _ in range(5):
        observe(node)
    value = Supervisor._elink_status_locked(node, time.monotonic())
    assert value["state"] == "UNKNOWN" and not value["functional_ready"]
    assert value["admission_enforced"] is False
    assert node._state == "RUNNING" and not node._elink_fault.reason
    assert not node._elink_fault.path.exists()


def test_failed_observer_submission_is_diagnostic_only(node):
    node._elink_pool.submit.side_effect = RuntimeError("closed")
    Supervisor._observe_elink(node)
    assert node._state == "RUNNING" and not node._elink_fault.reason
    assert not node._elink_observing
    assert node._elink_health.status(time.monotonic())["reason"] == "ELINK_OBSERVER_UNAVAILABLE"


def test_live_identity_replacement_does_not_block(node):
    node._backend.observe_elink.return_value = snapshot(hal_pid=21)
    observe(node)
    assert node._state == "RUNNING" and not node._elink_fault.reason
    assert Supervisor._elink_status_locked(node, time.monotonic())["state"] == "OBSERVED"


def test_missing_contract_is_not_falsely_promoted_to_healthy(node):
    value = Supervisor._elink_status_locked(node, time.monotonic())
    assert value["state"] == "OBSERVED" and value["admission_enforced"] is False
    assert not value["functional_ready"] and not value["message_fresh"]


def test_optional_telemetry_bind_error_does_not_veto_start(node):
    node._state = "STOPPED"
    node._elink_telemetry.bind_generation = Mock(side_effect=ValueError("invalid UTC evidence"))
    assert Supervisor._start(node, None, SimpleNamespace()).success
    execute_start(node)
    assert node._state == "RUNNING" and not node._busy
    assert not node._elink_telemetry.generation_id
    assert not Supervisor._elink_status_locked(node, time.monotonic())["functional_ready"]


def test_telemetry_fault_and_clock_expiry_are_diagnostic_only(node):
    telemetry(node, fault=True)
    node._elink_clock_qualified = True
    node._elink_clock_received = time.monotonic() - 10
    observe(node)
    value = Supervisor._elink_status_locked(node, time.monotonic())
    assert node._elink_telemetry.status.call_args.args[2] is False
    assert value["telemetry"]["fault_latched"] and value["state"] == "OBSERVED"
    assert not node._elink_fault.reason and node._state == "RUNNING"


def test_no_operational_permit_callbacks():
    assert not hasattr(Supervisor, "_on_elink_challenge")
    assert not hasattr(Supervisor, "_on_runner_permit_state")


@pytest.mark.parametrize("reason", ["ELINK_FAILURE_AFTER_READY", "ELINK_PROCESS_FAULT"])
def test_verified_start_failure_rejects_then_allows_cleanup_and_retry(node, reason):
    def fail(*args, **kwargs):
        kwargs["before_mutation"]()
        raise LifecycleFailure(reason)
    node._backend.start.side_effect = fail
    Supervisor._run_start(node, node._config, "new")
    assert node._state == "FAILED" and node._session_id == "new"
    assert reason in node._last_result and not node._elink_fault.reason
    assert not node._elink_fault.active_path.exists()
    node._backend.start.side_effect = lambda *a, **kw: kw["before_mutation"]()
    assert Supervisor._start(node, None, SimpleNamespace()).success
    execute_start(node)
    node._backend.kill_all_and_collect.assert_called_once()
    assert node._state == "RUNNING" and not node._busy


@pytest.mark.parametrize("reason", ["LAPTOP_ROOT_MISSING", "ELINK_OBSERVATION_UNKNOWN",
                                    "ELINK_OBSERVATION_TOOL_UNAVAILABLE", "ELINK_VENDOR_STACK_NOT_READY"])
def test_read_only_preflight_rejection_is_retryable_without_marker(node, reason):
    node._backend.start.side_effect = LifecycleFailure(reason)
    Supervisor._run_start(node, node._config, "new")
    assert node._state == "STOPPED" and not node._elink_fault.reason
    assert node._session_id == "" and not node._elink_fault.active_path.exists()
    assert Supervisor._start(node, None, SimpleNamespace()).success


def test_mutating_background_failure_retains_session_for_cleanup(node):
    def fail(*args, **kwargs):
        kwargs["before_mutation"]()
        raise LifecycleFailure("UNRELATED_BACKGROUND_NOT_STOPPED")
    node._backend.start.side_effect = fail
    Supervisor._run_start(node, node._config, "new")
    assert node._state == "FAILED" and node._session_id == "new"
    assert not node._elink_fault.active_path.exists()
    assert Supervisor._kill_all_and_collect(node, None, SimpleNamespace()).success


def test_initial_unknown_is_bounded_retryable_not_durable(node):
    node._backend.observe_elink.side_effect = ProbeError("timeout")
    Supervisor._run_start(node, node._config, "new")
    assert node._state == "FAILED" and node._backend.observe_elink.call_count == 3
    assert "ELINK_START_OBSERVATION_UNAVAILABLE" in node._last_result
    assert not node._elink_fault.reason and not node._elink_fault.active_path.exists()
    assert Supervisor._kill_all_and_collect(node, None, SimpleNamespace()).success


def test_one_initial_unknown_can_settle(node):
    node._backend.observe_elink.side_effect = [ProbeError("timeout"), snapshot()]
    Supervisor._run_start(node, node._config, "new")
    assert node._state == "RUNNING" and node._backend.observe_elink.call_count == 2


def test_confirmed_start_fault_not_retried_away_but_not_permanent(node):
    node._backend.observe_elink.side_effect = [snapshot(False), snapshot()]
    Supervisor._run_start(node, node._config, "new")
    assert node._state == "FAILED" and node._backend.observe_elink.call_count == 1
    assert "ELINK_START_PROCESS_FAULT: ELINK_IPC_NOT_LISTENING" in node._last_result
    assert not node._elink_fault.reason
    assert Supervisor._kill_all_and_collect(node, None, SimpleNamespace()).success


def test_audit_lookup_failure_is_diagnostic_only(node):
    node._backend.audit_elink.side_effect = ValueError("missing")
    Supervisor._run_start(node, node._config, "new")
    assert node._state == "RUNNING" and not node._elink_fault.reason
    node.get_logger().warning.assert_called_once()


def test_malformed_audit_does_not_crash_or_validate_build(node):
    telemetry(node)
    node._elink_audit = {"binary_sha256": [], "hardware_profile": 42}
    Supervisor._bind_elink_telemetry_locked(node, snapshot(), time.monotonic())
    fields = node._elink_telemetry.bind_generation.call_args.kwargs
    assert fields["hardware_profile"] == fields["hal_library_sha256"] == ""


@pytest.mark.parametrize("legacy", ["fault", "active_marker", "unreadable"])
def test_legacy_candidate_records_are_not_authority_and_not_deleted(node, legacy):
    path = node._elink_fault.path
    if legacy == "fault":
        path.write_text(json.dumps({"schema": 1, "reason": "ELINK_OBSERVATION_LOST"}))
    elif legacy == "active_marker":
        node._elink_fault.active_path.write_text('{"schema":1,"session_id":"old"}')
    else:
        path.write_text("unreadable-old-record")
    node._elink_fault = PersistentElinkFault(path)
    record = path if path.exists() else node._elink_fault.active_path
    before = record.read_bytes()
    node._state = "STOPPED"
    assert Supervisor._start(node, None, SimpleNamespace()).success
    execute_start(node)
    assert node._state == "RUNNING"
    value = Supervisor._elink_status_locked(node, time.monotonic())
    assert value["historical_fault_reason"] and value["admission_enforced"] is False
    assert Supervisor._kill_all_and_collect(node, None, SimpleNamespace()).success
    Supervisor._run_kill(node, node._config, node._session_id)
    assert node._state == "STOPPED" and record.read_bytes() == before


def test_kill_runs_backend_despite_diagnostic_faults(node):
    telemetry(node, fault=True)
    node._backend.observe_elink.return_value = snapshot(False)
    observe(node)
    def complete(*args):
        node._elink_telemetry.unbind_generation.assert_called_once_with()
        return "LOGS_RETAINED_ON_ROBOT"
    node._backend.kill_all_and_collect.side_effect = complete
    assert Supervisor._kill_all_and_collect(node, None, SimpleNamespace()).success
    Supervisor._run_kill(node, node._config, "session-a")
    assert node._state == "STOPPED"
    assert node._last_result == "KILL_COMPLETE_AGIBOT_PM_RESTORED_LOGS_RETAINED_ON_ROBOT"
    node._field.recorder.stop.assert_called_once_with(wait=False)
    node._backend.kill_all_and_collect.assert_called_once()
    module.release_hardware_operation_lock.assert_called_once()


def test_failed_restore_stays_explicit_and_retryable(node):
    node._backend.kill_all_and_collect.side_effect = LifecycleFailure("ELINK_VENDOR_STACK_NOT_READY")
    Supervisor._run_kill(node, node._config, "session-a")
    assert node._state == "FAILED" and "ELINK_VENDOR_STACK_NOT_READY" in node._last_result
    assert not node._elink_fault.reason
    assert Supervisor._kill_all_and_collect(node, None, SimpleNamespace()).success


def test_republication_does_not_refresh_diagnostics(node):
    old = node._elink_observed_at
    value = Supervisor._elink_status_locked(node, old + 4)
    assert value["state"] == "UNKNOWN" and value["observed_monotonic_s"] == old
    assert value["sequence"] == 1 and value["admission_enforced"] is False
    json.dumps(value, allow_nan=False)
