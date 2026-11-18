"""Failed-session retry executes cleanup and START under one operation lock."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
import hope_lifecycle_supervisor as module


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setenv("HOPE_AUTO_RECORD_ON_START", "0")
    monkeypatch.setattr(module, "require_applied", Mock())
    monkeypatch.setattr(module, "try_acquire_hardware_operation_lock", Mock(return_value=object()))
    monkeypatch.setattr(module, "release_hardware_operation_lock", Mock())
    n = SimpleNamespace(
        _lock=threading.Lock(), _state="FAILED", _busy=False, _field_busy=False,
        _config=SimpleNamespace(revision=1, table_side="P1"), _session_id="model21800_20261021T090033Z",
        _player_positions=Mock(),
        _session_mode="OPTITRACK", _hardware_operation_lock=object(),
        _backend=Mock(), _pool=Mock(), _run_start=Mock(), _on_event=Mock(),
        _field=SimpleNamespace(recorder=Mock(), runner_selection=Mock(return_value=("OPTITRACK", "")), status=lambda: {}),
        _wait_for_authoritative_runner=Mock(), get_logger=Mock(return_value=Mock()),
    )
    return n


def accept(n):
    return module.HopeLifecycleSupervisor._start(n, None, SimpleNamespace())


def execute(n):
    args=n._pool.submit.call_args.args
    module.HopeLifecycleSupervisor._run_start(n, *args[1:])


def test_passive_start_does_not_require_visible_player(node):
    node._player_positions.require.side_effect = AssertionError("PASSIVE must not query player visibility")
    assert accept(node).success
    execute(node)
    node._player_positions.require.assert_not_called()
    node._backend.start.assert_called_once()
    assert node._last_result == "START_COMPLETE_RUNNER_PASSIVE"


@pytest.mark.parametrize("equals", [False, True])
def test_attended_recovery_cli_never_initializes_ros(monkeypatch, equals, capsys):
    import os
    import pwd
    import hope_operations_recovery
    session = "model21800_20261029T110000Z"
    args = ["--recover-session=" + session] if equals else ["--recover-session", session]
    monkeypatch.setattr(sys, "argv", ["hope_lifecycle_supervisor.py", *args,
        "--confirm-supported-stopped-wired-estop", "--confirm-vendor-restore"])
    monkeypatch.setattr(pwd, "getpwnam", lambda _: SimpleNamespace(pw_uid=os.geteuid()))
    recover = Mock(return_value=Path("/isolated/archived.json"))
    monkeypatch.setattr(hope_operations_recovery, "recover_stopped_session", recover)
    run_node = Mock(side_effect=AssertionError("offline recovery must not create a ROS owner"))
    monkeypatch.setattr(module, "run_node", run_node)
    module.main()
    run_node.assert_not_called()
    assert recover.call_args.args[:2] == (module.CONFIG_PATH, session)
    assert recover.call_args.kwargs["confirmed"] is True
    assert "receipt archived" in capsys.readouterr().out


def test_incomplete_recovery_cli_cannot_fall_back_to_starting_ros(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["hope_lifecycle_supervisor.py", "--confirm-vendor-restore"])
    run_node = Mock()
    monkeypatch.setattr(module, "run_node", run_node)
    with pytest.raises(SystemExit):module.main()
    run_node.assert_not_called()


@pytest.mark.parametrize("fresh_matching", [False, True])
def test_receipt_only_records_verified_runner_boot_without_new_start_gate(node, fresh_matching):
    from hope_operations_core import WireSnapshot
    from hope_runner_control_core import runner_session_fingerprint
    node._session_receipt = Mock()
    node._recovery_runner = SimpleNamespace(snapshot=lambda: WireSnapshot(
        SimpleNamespace(boot_id=42, session_fingerprint=runner_session_fingerprint(
            node._session_id) if fresh_matching else 123), module.time.monotonic_ns()))
    assert accept(node).success
    execute(node)
    assert node._state == "RUNNING"
    assert node._session_receipt.record.call_args.args[3] == (42 if fresh_matching else None)


def test_failed_start_reuses_lock_cleans_old_session_then_starts_once(node):
    old=node._session_id; lock=node._hardware_operation_lock; order=[]
    node._backend.kill_all_and_collect.side_effect=lambda *a:order.append(("clean",a[1]))
    node._backend.start.side_effect=lambda *a,**k:order.append(("start",a[1]))
    node._wait_for_authoritative_runner.side_effect=lambda:order.append(("ready",node._session_id))
    assert accept(node).success
    assert node._state=="KILLING" and node._busy and node._session_id==old
    assert not accept(node).success  # second click cannot create another worker
    execute(node)
    assert [x[0] for x in order]==["clean","start","ready"]
    assert order[0][1]==old and order[1][1]!=old
    assert node._state=="RUNNING" and not node._busy
    assert node._last_result=="START_COMPLETE_RUNNER_PASSIVE"
    assert node._hardware_operation_lock is lock
    module.try_acquire_hardware_operation_lock.assert_not_called()
    module.release_hardware_operation_lock.assert_not_called()


def test_failed_cleanup_keeps_old_identity_and_never_starts(node):
    old=node._session_id
    node._backend.kill_all_and_collect.side_effect=module.LifecycleFailure("cleanup not confirmed")
    assert accept(node).success
    execute(node)
    assert node._state=="FAILED" and not node._busy and node._session_id==old
    assert node._last_result.startswith("START_CLEANUP_FAILED:")
    node._backend.start.assert_not_called()
    module.release_hardware_operation_lock.assert_not_called()
    # Another START can retry the cleanup without a manual KILL.
    assert accept(node).success


def test_asset_rejection_after_confirmed_cleanup_returns_stopped_and_unlocks(node):
    lock = node._hardware_operation_lock
    node._field.runner_selection.side_effect = ValueError("SELECTED_MODEL_UNAVAILABLE")
    assert accept(node).success
    execute(node)
    node._backend.kill_all_and_collect.assert_called_once()
    node._backend.start.assert_not_called()
    assert node._state == "STOPPED" and not node._busy
    assert node._session_id == "" and node._hardware_operation_lock is None
    assert node._last_result.startswith("PREFLIGHT_REJECTED:")
    module.release_hardware_operation_lock.assert_called_once_with(lock)
    # The next click is a regular preflight attempt, never another cleanup.
    assert accept(node).success
    execute(node)
    node._backend.kill_all_and_collect.assert_called_once()
    assert node._state == "STOPPED"


def test_worker_submit_failure_keeps_failed_session_and_lock(node):
    old=node._session_id; lock=node._hardware_operation_lock
    node._pool.submit.side_effect=RuntimeError("executor closed")
    assert not accept(node).success
    assert node._state=="FAILED" and node._session_id==old and not node._busy
    assert node._hardware_operation_lock is lock
    module.release_hardware_operation_lock.assert_not_called()


@pytest.mark.parametrize("state",["RUNNING","STARTING","KILLING","CONFIG_ERROR"])
def test_other_states_cannot_start(node,state):
    node._state=state
    assert not accept(node).success
    node._pool.submit.assert_not_called()


def test_missing_session_or_interlock_does_not_bypass_cleanup(node):
    node._session_id=""
    assert not accept(node).success
    node._session_id="model21800_20261021T090033Z";node._hardware_operation_lock=None
    module.try_acquire_hardware_operation_lock.return_value=None
    assert not accept(node).success
    node._pool.submit.assert_not_called()


def test_stopped_start_skips_cleanup(node):
    node._state="STOPPED";node._hardware_operation_lock=None
    assert accept(node).success
    execute(node)
    node._backend.kill_all_and_collect.assert_not_called()
    node._backend.start.assert_called_once()
    assert node._state=="RUNNING"


def test_empty_cleanup_error_still_leaves_a_retryable_failure(node):
    node._backend.kill_all_and_collect.side_effect=ValueError()
    assert accept(node).success
    execute(node)
    assert node._state=="FAILED" and not node._busy
    assert node._last_result=="START_CLEANUP_FAILED: ValueError"


def test_mismatched_config_blocks_start_before_worker_or_new_lock(node, monkeypatch):
    monkeypatch.setattr(module, "require_applied", Mock(side_effect=ValueError("CONFIG_NOT_APPLIED: old peer")))
    response=accept(node)
    assert not response.success and "CONFIG_NOT_APPLIED" in response.message
    node._pool.submit.assert_not_called()
    module.try_acquire_hardware_operation_lock.assert_not_called()


def test_report_change_after_lock_rejects_and_releases_new_lock(node, monkeypatch):
    node._state="STOPPED"; node._hardware_operation_lock=None
    monkeypatch.setattr(module, "require_applied", Mock(side_effect=[None, ValueError("CONFIG_NOT_APPLIED")]))
    assert not accept(node).success
    node._pool.submit.assert_not_called()
    module.release_hardware_operation_lock.assert_called_once()
