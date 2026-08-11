#!/usr/bin/env python3
"""HDU-resident fixed lifecycle orchestration for the Runner hardware runbook."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import os
import re
import subprocess
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger

from hope_lifecycle_core import (
    CONFIG_FIELDS,
    HelperEvent,
    LifecycleConfig,
    apply_config_updates,
    load_config,
    parse_helper_event,
    save_config_atomic,
    validate_session_id,
)


LIFECYCLE_HELPER = "/usr/local/libexec/hope-lifecycle"
CONFIG_PATH = Path("/var/lib/hope-lifecycle/config.json")
LAPTOP_USER = os.environ.get("HOPE_LAPTOP_USER", "operator").strip()
ROBOT_USER = os.environ.get("HOPE_ROBOT_USER", "agi").strip()
for _name, _value in (
    ("HOPE_LAPTOP_USER", LAPTOP_USER),
    ("HOPE_ROBOT_USER", ROBOT_USER),
):
    if re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", _value) is None:
        raise RuntimeError(f"{_name} is not a safe POSIX account name")
SSH_OPTIONS = (
    "-T",
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=3",
    "-o", "ConnectionAttempts=1",
    "-o", "ServerAliveInterval=2",
    "-o", "ServerAliveCountMax=2",
)
RUNNER_STATE_FRESHNESS_S = 1.5
RUNNER_START_VERIFY_TIMEOUT_S = 15.0


class LifecycleFailure(RuntimeError):
    pass


class LifecycleBackend:
    """Execute only checked-in helpers with a validated fixed argument layout."""

    def __init__(self, *, run=subprocess.run):
        self._run = run

    @staticmethod
    def _ssh(host: str, *remote: str) -> list[str]:
        return ["/usr/bin/ssh", *SSH_OPTIONS, host, *remote]

    def _invoke(
        self,
        argv: list[str],
        *,
        timeout_s: float,
        event_callback,
    ) -> str:
        completed = self._run(
            argv,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_s,
            check=False,
        )
        last_event = None
        for raw_line in completed.stdout.splitlines():
            event = parse_helper_event(raw_line)
            if event is not None:
                last_event = event
                event_callback(event)
        if completed.returncode != 0:
            detail = completed.stderr.strip().splitlines()
            if last_event is not None and last_event.state == "FAILED":
                reason = last_event.reason
            else:
                reason = detail[-1] if detail else f"exit code {completed.returncode}"
            raise LifecycleFailure(f"helper failed: {reason[:240]}")
        return completed.stdout

    def start(self, config: LifecycleConfig, session_id: str, event_callback) -> None:
        validate_session_id(session_id)
        laptop = f"{LAPTOP_USER}@{config.laptop_wifi_ip}"
        mdu = f"{ROBOT_USER}@{config.mdu_internal_ip}"
        common = [
            session_id,
            config.laptop_wifi_ip,
            config.hdu_wifi_ip,
            config.mdu_internal_ip,
            config.motive_ip,
        ]
        commands = (
            ("PREFLIGHT", self._ssh(laptop, LIFECYCLE_HELPER, "preflight-laptop", *common), 15.0),
            ("PREFLIGHT", [LIFECYCLE_HELPER, "preflight-hdu", *common], 10.0),
            ("PREFLIGHT", self._ssh(mdu, LIFECYCLE_HELPER, "preflight-mdu", *common), 15.0),
            ("SESSION", self._ssh(laptop, LIFECYCLE_HELPER, "prepare-laptop", *common), 45.0),
            ("SESSION", [LIFECYCLE_HELPER, "prepare-hdu", *common], 15.0),
            ("SESSION", self._ssh(mdu, LIFECYCLE_HELPER, "prepare-mdu", *common), 15.0),
            ("OPTITRACK", self._ssh(laptop, LIFECYCLE_HELPER, "start-laptop", *common), 25.0),
            ("BASE_RELAY", [LIFECYCLE_HELPER, "start-base", *common], 20.0),
            ("PLANNER", [LIFECYCLE_HELPER, "start-planner", *common], 20.0),
            # agibot_pm has a 90 s systemd stop timeout on the MDU. Field
            # shutdown commonly takes 40-60 s, so do not orphan a successful
            # remote HAL transition behind an undersized SSH timeout.
            ("HAL", self._ssh(mdu, LIFECYCLE_HELPER, "start-hal", *common), 120.0),
            ("RUNNER", self._ssh(mdu, LIFECYCLE_HELPER, "start-runner", *common), 25.0),
        )
        for step, argv, timeout_s in commands:
            event_callback(HelperEvent(step=step, state="STARTING", reason="REQUESTED"))
            self._invoke(
                list(argv), timeout_s=timeout_s, event_callback=event_callback
            )

    def kill_all_and_collect(
        self, config: LifecycleConfig, session_id: str, event_callback
    ) -> str:
        validate_session_id(session_id)
        laptop = f"{LAPTOP_USER}@{config.laptop_wifi_ip}"
        mdu = f"{ROBOT_USER}@{config.mdu_internal_ip}"
        common = [
            session_id,
            config.laptop_wifi_ip,
            config.hdu_wifi_ip,
            config.mdu_internal_ip,
            config.motive_ip,
        ]
        commands = (
            ("RUNNER_HAL", self._ssh(mdu, LIFECYCLE_HELPER, "stop-mdu", *common), 30.0),
            ("PLANNER_BASE", [LIFECYCLE_HELPER, "stop-hdu", *common], 30.0),
            ("OPTITRACK", self._ssh(laptop, LIFECYCLE_HELPER, "stop-laptop", *common), 30.0),
            ("COLLECT", self._ssh(laptop, LIFECYCLE_HELPER, "collect", *common), 120.0),
        )
        errors: list[str] = []
        collection_reason = "COLLECTION_RESULT_MISSING"
        for step, argv, timeout_s in commands:
            event_callback(HelperEvent(step=step, state="KILLING", reason="REQUESTED"))
            try:
                output = self._invoke(
                    list(argv), timeout_s=timeout_s, event_callback=event_callback
                )
                if step == "COLLECT":
                    for raw_line in output.splitlines():
                        event = parse_helper_event(raw_line)
                        if event is not None and event.step == "COLLECT":
                            collection_reason = event.reason
            except (LifecycleFailure, subprocess.TimeoutExpired) as exc:
                errors.append(f"{step}: {exc}")
        if errors:
            raise LifecycleFailure("; ".join(errors))
        if collection_reason not in {
            "LOGS_COLLECTED",
            "PARTIAL_LOGS_COLLECTED",
            "NO_REMOTE_SESSION_LOGS",
        }:
            raise LifecycleFailure(
                f"COLLECT: helper returned {collection_reason}"
            )
        return collection_reason


class HopeLifecycleSupervisor(Node):
    def __init__(self):
        super().__init__(
            "hope_lifecycle_supervisor",
            start_parameter_services=False,
        )
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hope-lifecycle")
        self._backend = LifecycleBackend()
        try:
            self._config = load_config(CONFIG_PATH)
            self._state = "STOPPED"
            self._last_result = "CONFIG_LOADED" if self._config.revision > 0 else "CONFIG_NOT_CONFIRMED"
        except (OSError, ValueError) as exc:
            self._config = LifecycleConfig()
            self._state = "CONFIG_ERROR"
            self._last_result = f"CONFIG_ERROR: {exc}"
        self._step = "IDLE"
        self._session_id = ""
        self._busy = False
        self._runner_mode = ""
        self._runner_mode_received = 0.0
        self._runner_session_matches = False
        self._runner_session_matches_received = 0.0

        self._lifecycle_publishers = {
            "state": self.create_publisher(String, "/hope/lifecycle/state", 10),
            "step": self.create_publisher(String, "/hope/lifecycle/step", 10),
            "summary": self.create_publisher(String, "/hope/lifecycle/summary", 10),
            "session": self.create_publisher(String, "/hope/lifecycle/session_id", 10),
            "result": self.create_publisher(String, "/hope/lifecycle/last_result", 10),
            "busy": self.create_publisher(Bool, "/hope/lifecycle/busy", 10),
            "revision": self.create_publisher(UInt32, "/hope/lifecycle/config/revision", 10),
        }
        self._config_publishers = {
            name: self.create_publisher(
                String, f"/hope/lifecycle/config/{name}", 10
            )
            for name in CONFIG_FIELDS
        }
        self.create_service(
            SetParameters,
            "/hope/lifecycle/apply_config",
            self._apply_config,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/hope/lifecycle/start",
            self._start,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/hope/lifecycle/kill_all_and_collect",
            self._kill_all_and_collect,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            "/hope/runner/mode",
            self._on_runner_mode,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            "/hope/runner/session_matches",
            self._on_runner_session_matches,
            10,
            callback_group=self._callback_group,
        )
        self.create_timer(0.5, self._publish)

    @staticmethod
    def _string(value: str) -> String:
        message = String()
        message.data = value
        return message

    def _publish(self) -> None:
        with self._lock:
            config = self._config
            state = self._state
            step = self._step
            session_id = self._session_id
            busy = self._busy
            result = self._last_result
        self._lifecycle_publishers["state"].publish(self._string(state))
        self._lifecycle_publishers["step"].publish(self._string(step))
        self._lifecycle_publishers["session"].publish(self._string(session_id))
        self._lifecycle_publishers["result"].publish(self._string(result))
        self._lifecycle_publishers["summary"].publish(
            self._string(
                f"state={state} step={step} session={session_id or 'NONE'} "
                f"config_revision={config.revision} result={result}"
            )
        )
        busy_message = Bool()
        busy_message.data = busy
        self._lifecycle_publishers["busy"].publish(busy_message)
        revision_message = UInt32()
        revision_message.data = config.revision
        self._lifecycle_publishers["revision"].publish(revision_message)
        for name, value in config.values().items():
            self._config_publishers[name].publish(self._string(value))

    def _apply_config(self, request, response):
        with self._lock:
            if self._busy or self._state not in {"STOPPED", "CONFIG_ERROR"}:
                reason = "configuration can only be confirmed while the lifecycle is stopped"
                response.results = [
                    SetParametersResult(successful=False, reason=reason)
                    for _parameter in request.parameters
                ]
                return response
            current = self._config
        updates: list[tuple[str, object]] = []
        for parameter in request.parameters:
            if parameter.value.type != ParameterType.PARAMETER_STRING:
                reason = f"{parameter.name} must use PARAMETER_STRING"
                response.results = [
                    SetParametersResult(successful=False, reason=reason)
                    for _parameter in request.parameters
                ]
                return response
            updates.append((parameter.name, parameter.value.string_value))
        try:
            updated = apply_config_updates(current, updates)
            save_config_atomic(CONFIG_PATH, updated)
        except (OSError, ValueError) as exc:
            response.results = [
                SetParametersResult(successful=False, reason=str(exc))
                for _parameter in request.parameters
            ]
            return response
        with self._lock:
            self._config = updated
            self._state = "STOPPED"
            self._last_result = f"CONFIG_CONFIRMED_REVISION_{updated.revision}"
        response.results = [
            SetParametersResult(
                successful=True,
                reason=f"confirmed lifecycle configuration revision {updated.revision}",
            )
            for _parameter in request.parameters
        ]
        return response

    def _on_event(self, event) -> None:
        with self._lock:
            self._step = event.step
            self._last_result = f"{event.state}:{event.reason}"

    def _on_runner_mode(self, message: String) -> None:
        with self._lock:
            self._runner_mode = str(message.data)
            self._runner_mode_received = time.monotonic()

    def _on_runner_session_matches(self, message: Bool) -> None:
        with self._lock:
            self._runner_session_matches = bool(message.data)
            self._runner_session_matches_received = time.monotonic()

    def _wait_for_authoritative_runner(self) -> None:
        """Require fresh Runner-owned PASSIVE state for the current session."""
        deadline = time.monotonic() + RUNNER_START_VERIFY_TIMEOUT_S
        with self._lock:
            self._step = "RUNNER_VERIFY"
            self._last_result = "WAITING_FOR_AUTHORITATIVE_RUNNER_PASSIVE"
        while time.monotonic() < deadline:
            now = time.monotonic()
            with self._lock:
                mode = self._runner_mode
                mode_fresh = (
                    self._runner_mode_received > 0.0
                    and now - self._runner_mode_received
                    <= RUNNER_STATE_FRESHNESS_S
                )
                session_matches = self._runner_session_matches
                session_matches_fresh = (
                    self._runner_session_matches_received > 0.0
                    and now - self._runner_session_matches_received
                    <= RUNNER_STATE_FRESHNESS_S
                )
            if (
                mode_fresh
                and mode == "PASSIVE"
                and session_matches_fresh
                and session_matches
            ):
                return
            time.sleep(0.1)
        raise LifecycleFailure(
            "authoritative Runner did not publish fresh PASSIVE state "
            "matching the managed session within 15 seconds"
        )

    def _start(self, _request, response):
        with self._lock:
            if self._busy or self._state != "STOPPED":
                response.success = False
                response.message = f"lifecycle start rejected in state {self._state}"
                return response
            if self._config.revision < 1:
                response.success = False
                response.message = "confirm the four IPv4 fields before starting"
                return response
            config = self._config
            session_id = datetime.now(timezone.utc).strftime("model21800_%Y%m%dT%H%M%SZ")
            self._busy = True
            self._state = "STARTING"
            self._step = "SESSION"
            self._session_id = session_id
            self._last_result = "START_ACCEPTED"
        self._pool.submit(self._run_start, config, session_id)
        response.success = True
        response.message = f"start accepted for {session_id}; follow lifecycle state topics"
        return response

    def _run_start(self, config: LifecycleConfig, session_id: str) -> None:
        try:
            self._backend.start(config, session_id, self._on_event)
            self._wait_for_authoritative_runner()
        except (LifecycleFailure, OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(
                f"Lifecycle start failed for {session_id}: {exc}"
            )
            with self._lock:
                managed_recovery = "MANAGED_" in str(exc)
                if self._step == "PREFLIGHT" and not managed_recovery:
                    self._state = "STOPPED"
                    self._session_id = ""
                    self._last_result = f"PREFLIGHT_REJECTED: {str(exc)[:300]}"
                else:
                    self._state = "FAILED"
                    prefix = "MANAGED_RECOVERY_REQUIRED" if managed_recovery else "START_FAILED"
                    if managed_recovery:
                        try:
                            previous_session = validate_session_id(
                                Path("/tmp/hope_model21800_session_id")
                                .read_text(encoding="utf-8")
                                .strip()
                            )
                        except (OSError, ValueError):
                            pass
                        else:
                            self._session_id = previous_session
                    self._last_result = f"{prefix}: {str(exc)[:280]}"
                self._busy = False
            return
        except Exception as exc:  # noqa: BLE001 - never leave hardware lifecycle busy
            self.get_logger().error(
                f"Unexpected lifecycle start failure for {session_id}: "
                f"{type(exc).__name__}: {exc}"
            )
            with self._lock:
                self._state = "FAILED"
                self._last_result = (
                    f"START_INTERNAL_ERROR: {type(exc).__name__}: {str(exc)[:240]}"
                )
                self._busy = False
            return
        with self._lock:
            self._state = "RUNNING"
            self._step = "RUNNER"
            self._last_result = "START_COMPLETE_RUNNER_PASSIVE"
            self._busy = False

    def _kill_all_and_collect(self, _request, response):
        with self._lock:
            if self._busy or self._state not in {"RUNNING", "FAILED"}:
                response.success = False
                response.message = f"kill rejected in state {self._state}"
                return response
            if not self._session_id:
                response.success = False
                response.message = "no managed session is available"
                return response
            config = self._config
            session_id = self._session_id
            self._busy = True
            self._state = "KILLING"
            self._step = "RUNNER_HAL"
            self._last_result = "KILL_ACCEPTED"
        self._pool.submit(self._run_kill, config, session_id)
        response.success = True
        response.message = (
            "kill accepted; managed robot processes may lose active support immediately"
        )
        return response

    def _run_kill(self, config: LifecycleConfig, session_id: str) -> None:
        try:
            collection_reason = self._backend.kill_all_and_collect(
                config, session_id, self._on_event
            )
        except (LifecycleFailure, OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(
                f"Lifecycle kill failed for {session_id}: {exc}"
            )
            with self._lock:
                self._state = "FAILED"
                self._last_result = f"KILL_FAILED: {str(exc)[:320]}"
                self._busy = False
            return
        except Exception as exc:  # noqa: BLE001 - never leave hardware lifecycle busy
            self.get_logger().error(
                f"Unexpected lifecycle kill failure for {session_id}: "
                f"{type(exc).__name__}: {exc}"
            )
            with self._lock:
                self._state = "FAILED"
                self._last_result = (
                    f"KILL_INTERNAL_ERROR: {type(exc).__name__}: {str(exc)[:240]}"
                )
                self._busy = False
            return
        with self._lock:
            self._state = "STOPPED"
            self._step = "IDLE"
            self._last_result = (
                "KILL_COMPLETE_AGIBOT_PM_RESTORED_" + collection_reason
            )
            self._busy = False

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)


def main() -> None:
    rclpy.init()
    node = HopeLifecycleSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
