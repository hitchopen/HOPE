"""Receipt security and restoration behavior without any robot operations."""
from pathlib import Path
import copy
import hashlib
import json
import sys
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
import hope_guarded_clock as guard


@pytest.fixture
def receipt():
    return dict(schema=1, boot_id='boot', operation_id='timecal_20261031T050000Z',
                coordinator_pid=123, coordinator_start='456', issued_monotonic=100.,
                mdu_ip='10.42.10.12', robot_user='agi')


@pytest.mark.parametrize('field,value', [
    ('schema', 2), ('boot_id', 'old'), ('operation_id', 'other'),
    ('issued_monotonic', 0), ('issued_monotonic', 101),
    ('issued_monotonic', float('nan')), ('issued_monotonic', float('inf')),
    ('coordinator_start', 'reused-pid'), ('mdu_ip', '10.42.10.12;reboot'),
    ('robot_user', 'agi;reboot'),
])
def test_wrong_boot_operation_expiry_pid_or_target_cannot_authorize(receipt, field, value):
    bad = copy.deepcopy(receipt)
    bad[field] = value
    with pytest.raises((RuntimeError, ValueError)):
        guard.validate_receipt(bad, receipt, 100., 'boot', '456')


@pytest.fixture
def launch_env(tmp_path, monkeypatch, receipt):
    for name in ('RECEIPT', 'ACK', 'STATUS'):
        monkeypatch.setattr(guard, name, tmp_path / name)
    monkeypatch.setattr(guard, 'secure_bytes', lambda path: path.read_bytes())
    monkeypatch.setattr(guard, 'check_installation', Mock())
    monkeypatch.setattr(guard, 'active_status', lambda: receipt)
    monkeypatch.setattr(guard, 'boot_id', lambda: 'boot')
    monkeypatch.setattr(guard, 'process_start', lambda pid: '456')
    monkeypatch.setattr(guard.time, 'monotonic', lambda: 100.)
    monkeypatch.setattr(guard, 'checked', lambda argv, **kwargs: '123')
    for name in ('require_locked', 'require_no_ptp', 'require_qualified', 'verify_mdu_stopped'):
        monkeypatch.setattr(guard, name, Mock())
    monkeypatch.setattr(guard.subprocess, 'run', Mock())
    monkeypatch.setattr(guard.os, 'execv', Mock(side_effect=SystemExit('vendor guard')))
    return tmp_path


def test_normal_boot_executes_original_guard(launch_env):
    with pytest.raises(SystemExit, match='vendor guard'):
        guard.launch()
    guard.os.execv.assert_called_once_with('/usr/bin/python3', ['/usr/bin/python3', str(guard.VENDOR)])
    guard.subprocess.run.assert_not_called()


def test_one_use_restoration_preserves_cgroup_and_requires_stopped_mdu(launch_env, receipt):
    guard.RECEIPT.write_text(json.dumps(receipt))
    guard.launch()
    guard.verify_mdu_stopped.assert_called_once_with('10.42.10.12', 'agi')
    guard.require_locked.assert_called_once()
    guard.require_qualified.assert_called_once()
    guard.subprocess.run.assert_called_once_with(['/bin/bash', str(guard.TIMESYNC)], check=True, timeout=30)
    assert not guard.RECEIPT.exists()
    assert json.loads(guard.ACK.read_text()) == receipt
    guard.verify()
    with pytest.raises(SystemExit, match='vendor guard'):
        guard.launch()  # No replay of the exceptional restore path.
    assert guard.subprocess.run.call_count == 1


@pytest.mark.parametrize('failure', ['require_locked', 'require_no_ptp', 'require_qualified', 'verify_mdu_stopped'])
def test_failed_live_recheck_consumes_receipt_without_ptp_or_ack(launch_env, receipt, monkeypatch, failure):
    guard.RECEIPT.write_text(json.dumps(receipt))
    monkeypatch.setattr(guard, failure, Mock(side_effect=RuntimeError('unsafe')))
    with pytest.raises(RuntimeError, match='unsafe'):
        guard.launch()
    assert not guard.RECEIPT.exists()
    assert not guard.ACK.exists()
    guard.subprocess.run.assert_not_called()
    guard.os.execv.assert_not_called()


def test_coordinator_pid_replaced_after_receipt_fails_closed(launch_env, receipt, monkeypatch):
    guard.RECEIPT.write_text(json.dumps(receipt))
    monkeypatch.setattr(guard, 'checked', lambda argv, **kwargs: '987')
    with pytest.raises(RuntimeError, match='identity changed'):
        guard.launch()
    guard.subprocess.run.assert_not_called()
    assert not guard.ACK.exists()


def test_no_ack_on_timesync_failure(launch_env, receipt):
    guard.RECEIPT.write_text(json.dumps(receipt))
    guard.subprocess.run.side_effect = RuntimeError('timesync failed')
    with pytest.raises(RuntimeError, match='timesync failed'):
        guard.launch()
    assert not guard.ACK.exists()
    assert not guard.RECEIPT.exists()


def test_stale_ack_never_allows_mdu_restore(launch_env, receipt):
    guard.ACK.write_text(json.dumps({**receipt, 'boot_id': 'old'}))
    with pytest.raises(RuntimeError, match='acknowledgement'):
        guard.verify()


def test_ack_rendezvous_waits_for_launcher_atomic_write(launch_env, receipt, monkeypatch):
    sleep = Mock(side_effect=lambda delay: guard.ACK.write_text(json.dumps(receipt)))
    monkeypatch.setattr(guard.time, 'sleep', sleep)
    guard.verify()
    sleep.assert_called_once_with(0.1)


def test_missing_ack_has_a_bounded_wait(launch_env, monkeypatch):
    sleep = Mock()
    monkeypatch.setattr(guard.time, 'sleep', sleep)
    with pytest.raises(FileNotFoundError):
        guard.verify()
    assert sleep.call_count == 50


@pytest.mark.parametrize('field,value', [('state', 'IDLE'), ('step', 'HARD_STEP'),
                                        ('hard_step_attempted', False), ('boot_id', 'old')])
def test_arm_requires_post_step_restore_status(tmp_path, monkeypatch, field, value):
    document = dict(state='RUNNING', step='HDU_RESTORE', hard_step_attempted=True,
                    boot_id='boot', operation_id='timecal_20261031T050000Z')
    document[field] = value
    monkeypatch.setattr(guard, 'secure_bytes', lambda path: json.dumps(document).encode())
    monkeypatch.setattr(guard, 'boot_id', lambda: 'boot')
    with pytest.raises(RuntimeError, match='live restoration'):
        guard.active_status()


def test_arm_grants_only_for_live_coordinator_with_all_consumers_stopped(launch_env, monkeypatch, receipt):
    monkeypatch.setattr(guard.os, 'getppid', lambda: 123)
    monkeypatch.setattr(guard, 'checked', lambda argv, **kwargs: '123' if 'MainPID' in argv else 'inactive')
    guard.arm('10.42.10.12', 'agi')
    assert json.loads(guard.RECEIPT.read_text()) == receipt
    guard.verify_mdu_stopped.assert_called_once()
    guard.require_qualified.assert_called_once()
    guard.subprocess.run.assert_not_called()  # Arming never launches PTP.
    with pytest.raises(RuntimeError, match='unconsumed'):
        guard.arm('10.42.10.12', 'agi')


@pytest.mark.parametrize('state', ['active', 'activating', 'deactivating', 'unknown'])
def test_arm_refuses_running_or_unknown_vendor_consumers(launch_env, monkeypatch, state):
    monkeypatch.setattr(guard.os, 'getppid', lambda: 123)
    monkeypatch.setattr(guard, 'checked', lambda argv, **kwargs: '123' if 'MainPID' in argv else state)
    with pytest.raises(RuntimeError, match='not stopped'):
        guard.arm('10.42.10.12', 'agi')
    assert not guard.RECEIPT.exists()


def test_arm_refuses_non_coordinator_caller(launch_env, monkeypatch):
    monkeypatch.setattr(guard.os, 'getppid', lambda: 987)
    with pytest.raises(RuntimeError, match='caller is not'):
        guard.arm('10.42.10.12', 'agi')
    assert not guard.RECEIPT.exists()


@pytest.fixture
def installation(tmp_path, monkeypatch):
    for name in ('GUARD', 'VENDOR', 'TIMESYNC', 'HELPER'):
        monkeypatch.setattr(guard, name, tmp_path / name)
    monkeypatch.setattr(guard, 'BACKUPS', tmp_path)
    monkeypatch.setattr(guard, 'secure_bytes', lambda path, **kwargs: path.read_bytes())
    monkeypatch.setattr(guard.os, 'chown', Mock())
    guard.GUARD.write_bytes(b'vendor fixture')
    guard.TIMESYNC.write_bytes(b'timesync fixture')
    monkeypatch.setattr(guard, 'VENDOR_SHA', hashlib.sha256(guard.GUARD.read_bytes()).hexdigest())
    monkeypatch.setattr(guard, 'TIMESYNC_SHA', hashlib.sha256(guard.TIMESYNC.read_bytes()).hexdigest())
    monkeypatch.setattr(guard.subprocess, 'run', Mock())


def test_install_preserves_original_and_never_starts_services(installation):
    guard.install()
    assert guard.GUARD.read_text() == guard.WRAPPER
    assert guard.VENDOR.read_bytes() == b'vendor fixture'
    guard.install()  # Idempotent upgrade retains the real original, not the wrapper.
    assert guard.VENDOR.read_bytes() == b'vendor fixture'
    guard.subprocess.run.assert_not_called()


def test_unknown_bsp_is_rejected_without_installing(installation):
    guard.GUARD.write_bytes(b'unknown vendor update')
    with pytest.raises(RuntimeError, match='unsupported vendor guard'):
        guard.install()
    assert not guard.HELPER.exists()
    assert not guard.VENDOR.exists()


@pytest.mark.parametrize('existing_helper', [False, True])
def test_failed_install_restores_original_files(installation, monkeypatch, existing_helper):
    if existing_helper:
        guard.HELPER.write_bytes(b'old helper')
    monkeypatch.setattr(guard, 'check_installation', Mock(side_effect=RuntimeError('validation failed')))
    with pytest.raises(RuntimeError, match='validation failed'):
        guard.install()
    assert guard.GUARD.read_bytes() == b'vendor fixture'
    if existing_helper:
        assert guard.HELPER.read_bytes() == b'old helper'
    else:
        assert not guard.HELPER.exists()
