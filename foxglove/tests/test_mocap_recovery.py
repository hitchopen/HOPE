import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
import hope_mocap_recovery as recovery
from hope_mocap_recovery import RecoveryPolicy, MocapRecoveryMixin
from hope_lifecycle_core import LifecycleConfig
from hope_lifecycle_supervisor import LifecycleModule


def test_disconnect_grace_cooldown_and_three_attempt_limit():
    p = RecoveryPolicy()
    observe = lambda now: p.observe(now, eligible=True, tracked=False, selected_tracked=False)
    assert not observe(0)
    assert not observe(9.9)
    assert observe(10)
    for start in (10, 72, 134):
        assert observe(start)
        p.begin(start)
        p.finish(start + 2)
        assert not observe(start + 61.9)
    assert not observe(1000)
    assert p.state == "NEEDS_ATTENTION"


def test_brief_tracking_does_not_reset_retry_budget():
    p = RecoveryPolicy(attempts=3)
    p.observe(10, eligible=True, tracked=True, selected_tracked=True)
    assert p.attempts == 3
    assert not p.observe(11, eligible=True, tracked=False, selected_tracked=False)
    p.observe(12, eligible=True, tracked=True, selected_tracked=True)
    p.observe(17, eligible=True, tracked=True, selected_tracked=True)
    assert p.attempts == 0


def test_running_time_does_not_count_towards_stopped_grace():
    p = RecoveryPolicy()
    assert not p.observe(0, eligible=False, tracked=False, selected_tracked=False)
    assert not p.observe(100, eligible=True, tracked=False, selected_tracked=False)
    assert not p.observe(109, eligible=True, tracked=False, selected_tracked=False)
    assert p.observe(110, eligible=True, tracked=False, selected_tracked=False)


def test_other_pants_asset_is_not_a_dead_receiver():
    p = RecoveryPolicy()
    for now in (0, 20, 100):
        assert not p.observe(now, eligible=True, tracked=True, selected_tracked=False)
    assert p.state == "ASSET_NOT_TRACKED"


class DeferredPool:
    def __init__(self):
        self.calls = []

    def submit(self, fn, *args):
        self.calls.append((fn, args))

    def run(self):
        fn, args = self.calls.pop(0)
        fn(*args)


class Harness(MocapRecoveryMixin):
    def __init__(self):
        self._lock = threading.Lock()
        self._state = "STOPPED"
        self._stopping = self._closed = self._busy = self._field_busy = False
        self._hardware_operation_lock = None
        self._config = LifecycleConfig(mocap_source="vrpn", revision=56)
        self._mocap_policy = RecoveryPolicy()
        self._mocap_recovery_active = False
        self._mocap_recovery_blocked = ""
        self._mocap_runner_flags = {key: (False, time.monotonic()) for key in ("alive", "command_publishing")}
        self._field_pool = DeferredPool()
        self._backend = Mock()
        self._refresh_field = Mock()
        self._logger = Mock()
        self._player_positions = Mock(status=Mock(return_value={"players_x_m": {}, "P1": False, "P2": False}))
        self._field_snapshot = Mock(poll=Mock(return_value={"mode": "OPTITRACK"}))

    def get_logger(self):
        return self._logger


@pytest.fixture
def node(monkeypatch):
    h = Harness()
    h.hardware_lock = threading.Lock()
    monkeypatch.setattr(recovery, "require_receiver_recovery", lambda config: None)
    monkeypatch.setattr(recovery, "try_acquire_hardware_operation_lock",
                        lambda: h.hardware_lock if h.hardware_lock.acquire(False) else None)
    monkeypatch.setattr(recovery, "release_hardware_operation_lock", lambda lock: lock.release())
    return h


@pytest.mark.parametrize("state", ["STARTING", "RUNNING", "FAILED", "KILLING", "CONFIG_ERROR"])
def test_manual_and_auto_never_reconnect_nonstopped_robot(node, state):
    node._state = state
    node._mocap_policy.missing_since = time.monotonic() - 100
    node._watch_mocap()
    with pytest.raises(ValueError):
        node._request_mocap_reconnect(automatic=False)
    assert not node._field_pool.calls
    node._backend.reconnect_mocap.assert_not_called()


@pytest.mark.parametrize("key", ["alive", "command_publishing"])
@pytest.mark.parametrize("value,age", [(True, 0), (False, 5), (False, -5), (None, 0)])
def test_missing_stale_or_active_runner_telemetry_blocks(node, key, value, age):
    node._mocap_runner_flags[key] = (value, time.monotonic() - age)
    with pytest.raises(ValueError, match="telemetry"):
        node._request_mocap_reconnect(automatic=False)
    assert not node._field_pool.calls


def test_recovery_owns_interlock_and_blocks_start_until_finished(node):
    result = node._request_mocap_reconnect(automatic=False)
    assert result["state"] == "RECONNECTING"
    assert node._field_busy and node.hardware_lock.locked()
    response = LifecycleModule._start(node, None, SimpleNamespace())
    assert not response.success
    with pytest.raises(ValueError):
        node._request_mocap_reconnect(automatic=False)
    node._field_pool.run()
    node._backend.reconnect_mocap.assert_called_once_with(node._config)
    assert not node._field_busy and not node.hardware_lock.locked()
    assert node._mocap_policy.state == "VERIFYING"  # helper success is not live pose evidence
    node._refresh_field.assert_called_once()


def test_external_clock_or_start_lock_is_never_bypassed(node):
    node.hardware_lock.acquire()
    with pytest.raises(ValueError, match="interlock"):
        node._request_mocap_reconnect(automatic=True)
    assert not node._field_busy and not node._field_pool.calls


@pytest.mark.parametrize("change", ["running", "telemetry", "config", "shutdown"])
def test_queued_recovery_rechecks_before_mutating_receiver(node, change):
    node._request_mocap_reconnect(automatic=False)
    if change == "running": node._state = "RUNNING"
    if change == "telemetry": node._mocap_runner_flags["alive"] = (True, time.monotonic())
    if change == "config": node._config = LifecycleConfig(mocap_source="vrpn", revision=57)
    if change == "shutdown": node._stopping = True
    node._field_pool.run()
    node._backend.reconnect_mocap.assert_not_called()
    assert not node.hardware_lock.locked() and not node._field_busy
    assert node._mocap_policy.state == "FAILED"


def test_helper_failure_releases_lock_without_claiming_recovered(node):
    node._backend.reconnect_mocap.side_effect = RuntimeError("OTHER_ROBOT_RECEIVER_UNTOUCHED")
    node._request_mocap_reconnect(automatic=False)
    node._field_pool.run()
    assert node._mocap_policy.state == "FAILED"
    assert "OTHER_ROBOT" in node._mocap_policy.error
    assert not node.hardware_lock.locked() and not node._field_busy
    node._refresh_field.assert_not_called()


def test_watchdog_recovers_only_confirmed_optitrack_without_live_players(node):
    node._mocap_policy.missing_since = time.monotonic() - 11
    node._watch_mocap()
    assert node._mocap_recovery_active
    node._field_pool.run()
    assert node._backend.reconnect_mocap.call_count == 1
    node._watch_mocap()
    assert not node._field_pool.calls  # cooldown


@pytest.mark.parametrize("mode", ["KERNEL", None])
def test_no_automatic_reconnect_in_kernel_or_unknown_mode(node, mode):
    node._field_snapshot.poll.return_value = {"mode": mode}
    node._mocap_policy.missing_since = time.monotonic() - 11
    node._watch_mocap()
    assert not node._field_pool.calls


def test_stream_recovers_while_worker_queued_no_restart(node):
    node._request_mocap_reconnect(automatic=True)
    node._player_positions.status.return_value = {"players_x_m": {"UCB_P1": -0.4}, "P1": True}
    node._field_pool.run()
    node._backend.reconnect_mocap.assert_not_called()
    assert not node.hardware_lock.locked()


def test_observation_failure_disables_recovery_without_killing_owner(node):
    node._player_positions.status.side_effect = RuntimeError("native state unavailable")
    node._watch_mocap()
    assert not node._field_pool.calls
    assert node._mocap_policy.state == "BLOCKED"
    assert node._mocap_policy.missing_since is None


def test_submit_failure_releases_admission_and_returns_rejection(node):
    node._field_pool.submit = Mock(side_effect=RuntimeError("worker closed"))
    with pytest.raises(ValueError, match="worker unavailable"):
        node._request_mocap_reconnect(automatic=False)
    assert not node.hardware_lock.locked() and not node._field_busy


def test_field_command_reconnect_uses_saved_config_and_rejects_overrides(node):
    from rclpy.parameter import Parameter
    import json
    request = lambda payload: SimpleNamespace(parameters=[
        Parameter("request", value=json.dumps(payload)).to_parameter_msg()])
    rejected = LifecycleModule._field_command(node,
        request({"op": "reconnect_mocap", "vrpn_ip": "192.0.2.7"}), SimpleNamespace())
    assert not rejected.results[0].successful and not node._field_pool.calls
    accepted = LifecycleModule._field_command(node, request({"op": "reconnect_mocap"}), SimpleNamespace())
    assert accepted.results[0].successful and node._mocap_recovery_active
    node._field_pool.run()
    node._backend.reconnect_mocap.assert_called_once_with(node._config)


def test_manual_reconnect_also_serves_motive_but_automatic_stays_vrpn_only(node):
    node._config = LifecycleConfig(mocap_source="motive", motive_ip="169.254.10.221", revision=56)
    status = node._mocap_recovery_status()
    assert status["manual_allowed"] and status["manual_blocked_reason"] == "" and status["manual_blocked_code"] == ""
    node._mocap_policy.missing_since = time.monotonic() - 100
    node._watch_mocap()
    assert not node._field_pool.calls  # no automatic restart of a Motive receiver
    assert node._request_mocap_reconnect(automatic=False)["state"] == "RECONNECTING"
    node._field_pool.run()
    node._backend.reconnect_mocap.assert_called_once_with(node._config)
    with pytest.raises(ValueError):
        node._request_mocap_reconnect(automatic=True)


def test_manual_reconnect_refuses_mocap_none_with_its_own_reason(node):
    node._config = LifecycleConfig(mocap_source="none", revision=56)
    status = node._mocap_recovery_status()
    assert not status["manual_allowed"] and status["manual_blocked_code"] == "NONE"
    with pytest.raises(ValueError, match="None"):
        node._request_mocap_reconnect(automatic=False)


def test_manual_reconnect_has_short_cooldown_automatic_keeps_sixty_seconds(node):
    node._request_mocap_reconnect(automatic=False)
    node._field_pool.run()
    status = node._mocap_recovery_status()
    assert 0 < status["retry_after_s"] <= recovery.MANUAL_COOLDOWN_S and not status["manual_allowed"]
    assert status["auto_retry_after_s"] > 50 and node._mocap_policy.attempts == 0
    node._mocap_policy.next_attempt = 0
    node._request_mocap_reconnect(automatic=True)
    node._field_pool.run()
    assert node._mocap_recovery_status()["auto_retry_after_s"] > 50
    assert node._mocap_policy.attempts == 1


def test_automatic_recovery_never_restarts_a_receiver_the_operator_just_reconnected(node):
    node._request_mocap_reconnect(automatic=False)
    node._field_pool.run()
    policy = node._mocap_policy
    now = time.monotonic()
    # 10 s without poses after the manual attempt: the automatic back-off (60 s) still holds.
    policy.missing_since = now - 30
    assert not policy.observe(now + 15, eligible=True, tracked=False, selected_tracked=False)
    assert policy.observe(now + 61, eligible=True, tracked=False, selected_tracked=False)


def test_manual_reconnect_is_not_held_by_the_automatic_back_off(node):
    node._mocap_policy.next_attempt = 0
    node._request_mocap_reconnect(automatic=True)
    node._field_pool.run()
    node._mocap_policy.manual_next_attempt = time.monotonic() - 1
    assert node._mocap_recovery_status()["manual_allowed"]
    assert node._request_mocap_reconnect(automatic=False)["state"] == "RECONNECTING"


@pytest.mark.parametrize("setup,code", [
    (lambda n: setattr(n, "_state", "RUNNING"), "RUNNING"),
    (lambda n: n._mocap_runner_flags.update(alive=(True, time.monotonic())), "TELEMETRY"),
    (lambda n: setattr(n, "_hardware_operation_lock", object()), "BUSY"),
    (lambda n: setattr(n, "_field_busy", True), "BUSY"),
    (lambda n: setattr(n, "_state", "FAILED"), "RUNNING"),
    (lambda n: setattr(n, "_config", LifecycleConfig(mocap_source="vrpn", revision=0)), "CONFIG")])
def test_every_manual_refusal_has_a_stable_code(node, setup, code):
    setup(node)
    status = node._mocap_recovery_status()
    assert not status["manual_allowed"] and status["manual_blocked_code"] == code
    with pytest.raises(ValueError):
        node._request_mocap_reconnect(automatic=False)
