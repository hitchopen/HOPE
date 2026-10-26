"""Clock-step admission methods, with no SSH, service or clock operations."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys
import threading

import pytest

pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
import hope_time_calibration as coordinator
from hope_time_calibration import HopeTimeCalibration as Calibration
from hope_time_calibration_core import CalibrationStatus


@pytest.fixture
def subject(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(coordinator, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    config = SimpleNamespace(revision=1)
    monkeypatch.setattr(coordinator, "load_config", Mock(return_value=config))
    token = object()
    acquire = Mock(return_value=token)
    release = Mock()
    save = Mock()
    monkeypatch.setattr(coordinator, "try_acquire_hardware_operation_lock", acquire)
    monkeypatch.setattr(coordinator, "release_hardware_operation_lock", release)
    monkeypatch.setattr(coordinator, "save_status_atomic", save)
    node = SimpleNamespace(
        _lock=threading.Lock(),
        _boot_id="test-boot",
        _status=CalibrationStatus(boot_id="test-boot"),
        _lifecycle_state="STOPPED", _lifecycle_state_received=10.0,
        _lifecycle_busy=False, _lifecycle_busy_received=10.0,
        _ntp_gate=False, _ntp_gate_received=10.0,
        _hardware_operation_lock=None,
        _pool=Mock(), _backend=Mock(), _run_calibration=Mock(),
    )
    node._request_rejection_locked = lambda now: Calibration._request_rejection_locked(node, now)
    node._release_operation_lock = lambda: Calibration._release_operation_lock(node)
    node._replace_status = lambda **kwargs: Calibration._replace_status(node, **kwargs)
    node._clean_result = Calibration._clean_result
    return SimpleNamespace(node=node, clock=clock, token=token, acquire=acquire,
                           release=release, save=save, config=config)


def request(subject):
    return Calibration._request_calibration(subject.node, None, SimpleNamespace())


def assert_rejected_without_worker(subject, *, acquired=False):
    response = request(subject)
    assert not response.success
    subject.node._pool.submit.assert_not_called()
    subject.node._backend.assert_not_called()
    subject.node._backend.calibrate.assert_not_called()
    subject.save.assert_not_called()
    assert subject.node._hardware_operation_lock is None
    if acquired:
        subject.release.assert_called_once_with(subject.token)
    else:
        subject.acquire.assert_not_called()
        subject.release.assert_not_called()
    return response


def test_fresh_stopped_request_persists_and_submits_exactly_once(subject):
    response = request(subject)
    assert response.success
    assert subject.node._status.state == "RUNNING"
    assert subject.node._hardware_operation_lock is subject.token
    subject.save.assert_called_once_with(coordinator.STATUS_PATH, subject.node._status)
    subject.node._pool.submit.assert_called_once_with(subject.node._run_calibration, subject.config)
    subject.node._backend.calibrate.assert_not_called()  # Fake pool never runs hardware.
    subject.release.assert_not_called()
    assert not request(subject).success
    subject.acquire.assert_called_once()
    subject.node._pool.submit.assert_called_once()


@pytest.mark.parametrize("state", ["RUNNING", "STARTING", "STOPPING", "FAILED",
                                   "PASSIVE", "PD_STAND", "SERVE", "RALLY"])
def test_active_or_unknown_lifecycle_cannot_request_clock_step(subject, state):
    # Runner PD_STAND/SERVE/RALLY normally share lifecycle RUNNING; neither
    # that state nor an unexpected mode string is accepted as STOPPED.
    subject.node._lifecycle_state = state
    assert "idle STOPPED" in assert_rejected_without_worker(subject).message


@pytest.mark.parametrize("field", ["_lifecycle_state_received", "_lifecycle_busy_received", "_ntp_gate_received"])
@pytest.mark.parametrize("received", [0.0, 8.4, 10.1, float("nan")])
def test_stale_missing_future_or_nonfinite_evidence_rejects(subject, field, received):
    setattr(subject.node, field, received)
    assert_rejected_without_worker(subject)


@pytest.mark.parametrize("field,value", [("_lifecycle_busy", True), ("_ntp_gate", True)])
def test_busy_or_already_qualified_rejects(subject, field, value):
    setattr(subject.node, field, value)
    assert_rejected_without_worker(subject)


@pytest.mark.parametrize("state,attempted", [("RUNNING", False), ("INTERRUPTED", False),
                                            ("FAILED_SAFE_STOP", False), ("COMPLETE", True)])
def test_maintenance_or_previous_attempt_blocks_request(subject, state, attempted):
    subject.node._status = replace(subject.node._status, state=state, hard_step_attempted=attempted)
    assert_rejected_without_worker(subject)


@pytest.mark.parametrize("change", ["running", "busy", "qualified", "expired", "future",
                                    "maintenance_running", "failed", "attempted"])
def test_interlock_acquisition_rechecks_latest_evidence_before_accepting(subject, change):
    def acquire_after_change():
        if change == "running":
            subject.node._lifecycle_state = "RUNNING"
        elif change == "busy":
            subject.node._lifecycle_busy = True
        elif change == "qualified":
            subject.node._ntp_gate = True
        elif change == "expired":
            subject.clock[0] += 2.0
        elif change == "future":
            subject.node._lifecycle_state_received = subject.clock[0] + .1
        elif change == "maintenance_running":
            subject.node._status = replace(subject.node._status, state="RUNNING")
        elif change == "failed":
            subject.node._status = replace(subject.node._status, state="FAILED_SAFE_STOP")
        else:
            subject.node._status = replace(subject.node._status, state="COMPLETE", hard_step_attempted=True)
        return subject.token
    subject.acquire.side_effect = acquire_after_change
    assert_rejected_without_worker(subject, acquired=True)


def test_persistence_failure_releases_lock_without_worker(subject):
    subject.save.side_effect = OSError("test write failure")
    response = request(subject)
    assert not response.success and "cannot persist" in response.message
    assert subject.node._status.state == "IDLE"
    assert subject.node._hardware_operation_lock is None
    subject.release.assert_called_once_with(subject.token)
    subject.node._pool.submit.assert_not_called()


def test_worker_submission_failure_releases_lock_and_records_rejection(subject):
    subject.node._pool.submit.side_effect = RuntimeError("test stopped executor")
    response = request(subject)
    assert not response.success and "cannot start" in response.message
    assert subject.node._status.state == "REJECTED"
    assert subject.node._hardware_operation_lock is None
    subject.release.assert_called_once_with(subject.token)
    subject.node._backend.calibrate.assert_not_called()


def test_busy_hardware_interlock_denies_without_status_change(subject):
    subject.acquire.return_value = None
    response = request(subject)
    assert not response.success and "interlock" in response.message
    subject.node._pool.submit.assert_not_called()
    subject.save.assert_not_called()
    subject.release.assert_not_called()


def test_confirmed_state_is_persisted_while_callback_lock_is_held(subject):
    def inspect_save(_path, status):
        assert subject.node._lock.locked()
        assert status.state == "RUNNING"
        assert not status.hard_step_attempted
    subject.save.side_effect = inspect_save
    assert request(subject).success
