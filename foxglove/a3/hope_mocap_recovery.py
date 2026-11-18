"""Bounded receiver recovery while the managed robot is stopped.

Recovery restores receiver diagnostics; player visibility does not gate PASSIVE START.
The shared hardware interlock excludes START, clock calibration and recovery.
"""
from contextlib import nullcontext
from dataclasses import dataclass
import math
import time

from std_msgs.msg import Bool
from hope_config_runtime import require_receiver_recovery
from hope_lifecycle_core import try_acquire_hardware_operation_lock, release_hardware_operation_lock

# After an operator-requested reconnect the button is usable again sooner than the automatic
# 60 s back-off; the interlock and field_busy still exclude overlapping operations.
MANUAL_COOLDOWN_S = 10.0

# Stable codes for the operator reconnect refusal, so the Console can explain it in its language.
_MANUAL_BLOCK_CODES = {
    "Operations owner is stopping": "STOPPING",
    "Stop the managed lifecycle before reconnecting mocap": "RUNNING",
    "Another lifecycle or configuration operation is in progress": "BUSY",
    "A hardware operation owns the interlock": "BUSY",
    "Save the configuration first": "CONFIG",
    "Mocap system is None; select Motive or VRPN and confirm first": "NONE",
    "Waiting for fresh Runner stopped telemetry": "TELEMETRY",
}


@dataclass
class RecoveryPolicy:
    missing_since: float | None = None
    healthy_since: float | None = None
    # Automatic back-off (60 s after ANY attempt, so automatic recovery never restarts a receiver the
    # operator just reconnected) and the shorter operator cooldown are tracked separately.
    next_attempt: float = 0.0
    manual_next_attempt: float = 0.0
    attempts: int = 0
    state: str = "WAITING"
    error: str = ""

    def observe(self, now, *, eligible, tracked, selected_tracked):
        if not math.isfinite(now):
            return False
        if tracked:
            self.missing_since = None
            if self.healthy_since is None:
                self.healthy_since = now
            if now - self.healthy_since >= 5:
                self.attempts = 0
                self.error = ""
            self.state = "HEALTHY" if selected_tracked else "ASSET_NOT_TRACKED"
            return False
        self.healthy_since = None
        if not eligible:
            self.missing_since = None
            self.state = "BLOCKED"
            return False
        if self.missing_since is None:
            self.missing_since = now
        if self.attempts >= 3:
            self.state = "NEEDS_ATTENTION"
            return False
        self.state = "WAITING"
        return now - self.missing_since >= 10 and now >= self.next_attempt

    def begin(self, now, *, manual=False):
        # Only automatic attempts count toward the automatic three-attempt budget.
        if not manual:
            self.attempts += 1
        self.next_attempt = now + 60
        self.manual_next_attempt = now + MANUAL_COOLDOWN_S
        self.state = "RECONNECTING"
        self.error = ""

    def finish(self, now, error=""):
        self.next_attempt = now + 60
        self.manual_next_attempt = now + MANUAL_COOLDOWN_S
        self.missing_since = now
        self.state = "FAILED" if error else "VERIFYING"
        self.error = error


class MocapRecoveryMixin:
    def _init_mocap_recovery(self):
        self._mocap_policy = RecoveryPolicy()
        self._mocap_runner_flags = {}
        self._mocap_recovery_active = False
        self._mocap_recovery_blocked = "Waiting for stopped Runner telemetry"
        self._mocap_recovery_subscriptions = [
            self.create_subscription(
                Bool, "/hope/runner/" + name,
                lambda message, key=name: self._on_mocap_runner_flag(key, message),
                10, callback_group=self._callback_group)
            for name in ("alive", "command_publishing")
        ]
        self._mocap_recovery_timer = self.create_timer(1.0, self._watch_mocap)

    def _on_mocap_runner_flag(self, name, message):
        with self._lock:
            self._mocap_runner_flags[name] = (message.data, time.monotonic())

    def _mocap_blocked_locked(self, now, *, manual=False):
        """Why a reconnect cannot run now ("" = allowed).

        Automatic recovery stays VRPN-only. An operator-requested reconnect (Console
        "重新连接动捕") restarts the saved receiver for either source, e.g. Motive after
        the Motive NIC was plugged back in.
        """
        if self._stopping or self._closed:
            return "Operations owner is stopping"
        if self._busy or self._field_busy:
            return "Another lifecycle or configuration operation is in progress"
        if self._state != "STOPPED":
            return "Stop the managed lifecycle before reconnecting mocap"
        if self._hardware_operation_lock is not None:
            return "A hardware operation owns the interlock"
        if self._config.revision < 1:
            return "Save the configuration first"
        if manual and self._config.mocap_source == "none":
            return "Mocap system is None; select Motive or VRPN and confirm first"
        if self._config.mocap_source not in ({"vrpn", "motive"} if manual else {"vrpn"}):
            return "Save the VRPN configuration first"
        for name in ("alive", "command_publishing"):
            value, received = self._mocap_runner_flags.get(name, (None, -math.inf))
            if value is not False or not 0 <= now - received <= 1.5:
                return "Waiting for fresh Runner stopped telemetry"
        return ""

    def _mocap_positions(self, now):
        with (nullcontext() if getattr(self._player_positions, "status_threadsafe", False) is True else self._lock):
            return self._player_positions.status(now)

    def _watch_mocap(self):
        try:
            self._watch_mocap_once()
        except Exception as exc:
            # Missing native/status evidence disables recovery; it must not
            # bring down the operations owner or recycle the receiver.
            with self._lock:
                self._mocap_policy.missing_since = None
                self._mocap_policy.state = "BLOCKED"
                self._mocap_policy.error = str(exc)[:400]

    def _watch_mocap_once(self):
        now = time.monotonic()
        positions = self._mocap_positions(now)
        field = self._field_snapshot.poll()
        with self._lock:
            if self._mocap_recovery_active:
                return
            blocked = self._mocap_blocked_locked(now)
            if not blocked and field.get("mode") != "OPTITRACK":
                blocked = "Automatic recovery requires confirmed OPTITRACK mode"
            self._mocap_recovery_blocked = blocked
            due = self._mocap_policy.observe(
                now, eligible=not blocked, tracked=bool(positions.get("players_x_m")),
                selected_tracked="UCB_" + self._config.table_side in positions.get("players_x_m", {}))
        if due:
            try:
                self._request_mocap_reconnect(automatic=True)
            except (OSError, ValueError, RuntimeError) as exc:
                # Another operation may win admission between observation and
                # request. Do not stop that operation or bypass its lock.
                with self._lock:
                    self._mocap_policy.error = str(exc)[:400]

    def _request_mocap_reconnect(self, *, automatic):
        operation_lock = None
        with self._lock:
            now = time.monotonic()
            blocked = self._mocap_blocked_locked(now, manual=not automatic)
            if blocked:
                raise ValueError(blocked)
            ready_at = self._mocap_policy.next_attempt if automatic else self._mocap_policy.manual_next_attempt
            if now < ready_at:
                raise ValueError("Mocap reconnect is cooling down; wait before retrying")
            if automatic and self._mocap_policy.attempts >= 3:
                raise ValueError("Automatic reconnect limit reached")
            # The config object and selected receiver owner cannot change while
            # field_busy is held. The helper also checks laptop ownership.
            require_receiver_recovery(self._config)
            operation_lock = try_acquire_hardware_operation_lock()
            if operation_lock is None:
                raise ValueError("Another lifecycle or clock operation owns the interlock")
            self._field_busy = True
            self._mocap_recovery_active = True
            self._mocap_policy.begin(now, manual=not automatic)
            config = self._config
            try:
                self._field_pool.submit(self._run_mocap_reconnect, config, operation_lock, automatic)
            except Exception as exc:
                self._field_busy = False
                self._mocap_recovery_active = False
                self._mocap_policy.finish(now, "Recovery worker unavailable")
                release_hardware_operation_lock(operation_lock)
                raise ValueError("Recovery worker unavailable") from exc
        return {"state": "RECONNECTING", "automatic": automatic}

    def _run_mocap_reconnect(self, config, operation_lock, automatic):
        error = ""
        try:
            # Recheck fresh telemetry after waiting behind any field worker.
            with self._lock:
                now = time.monotonic()
                if self._stopping or self._closed or self._state != "STOPPED" or self._busy or self._config is not config:
                    raise ValueError("State changed before mocap reconnect")
                for name in ("alive", "command_publishing"):
                    value, received = self._mocap_runner_flags.get(name, (None, -math.inf))
                    if value is not False or not 0 <= now - received <= 1.5:
                        raise ValueError("Runner stopped telemetry changed before reconnect")
            if automatic and self._mocap_positions(time.monotonic()).get("players_x_m"):
                return  # The stream recovered while the worker was queued.
            self._backend.reconnect_mocap(config)
        except Exception as exc:
            error = str(exc)[:400] or type(exc).__name__
            self.get_logger().warning("Mocap reconnect failed: " + error)
        finally:
            with self._lock:
                self._mocap_policy.finish(time.monotonic(), error)
                self._mocap_recovery_active = False
                self._field_busy = False
                release_hardware_operation_lock(operation_lock)
        if not error:
            self._refresh_field()

    def _mocap_recovery_status(self):
        with self._lock:
            now = time.monotonic()
            p = self._mocap_policy
            blocked = self._mocap_blocked_locked(now)
            manual_blocked = self._mocap_blocked_locked(now, manual=True)
            return {"state": p.state, "attempts": p.attempts, "max_attempts": 3,
                    # Operator-facing cooldown; the automatic back-off is reported separately.
                    "retry_after_s": max(0, math.ceil(p.manual_next_attempt - now)),
                    "auto_retry_after_s": max(0, math.ceil(p.next_attempt - now)),
                    "manual_allowed": not manual_blocked and now >= p.manual_next_attempt,
                    "manual_blocked_reason": manual_blocked,
                    "manual_blocked_code": _MANUAL_BLOCK_CODES.get(manual_blocked, "OTHER") if manual_blocked else "",
                    "blocked_reason": blocked or self._mocap_recovery_blocked,
                    "error": p.error}
