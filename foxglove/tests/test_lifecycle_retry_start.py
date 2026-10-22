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
    monkeypatch.setattr(module, "try_acquire_hardware_operation_lock", Mock(return_value=object()))
    monkeypatch.setattr(module, "release_hardware_operation_lock", Mock())
    n = SimpleNamespace(
        _lock=threading.Lock(), _state="FAILED", _busy=False, _field_busy=False,
        _config=SimpleNamespace(revision=1), _session_id="model21800_20261021T090033Z",
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
