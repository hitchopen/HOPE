#!/usr/bin/env python3
"""HDU-resident fixed lifecycle orchestration for the Runner hardware runbook."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import os
import json
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

from hope_field_operations import FieldOperations

from hope_lifecycle_core import (
    CONFIG_FIELDS,
    HelperEvent,
    LifecycleConfig,
    apply_config_updates,
    load_config,
    parse_helper_event,
    release_hardware_operation_lock,
    save_config_atomic,
    try_acquire_hardware_operation_lock,
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

    def start(self, config: LifecycleConfig, session_id: str, event_callback,
              mode: str = "OPTITRACK", csv_sha: str = "DEFAULT") -> None:
        validate_session_id(session_id)
        laptop = f"{LAPTOP_USER}@{config.laptop_wifi_ip}"
        mdu = f"{ROBOT_USER}@{config.mdu_internal_ip}"
        motive_arg = config.motive_ip or "NONE"
        common = [
            session_id,
            config.laptop_wifi_ip,
            config.hdu_wifi_ip,
            config.mdu_internal_ip,
            motive_arg,
            config.table_side,
            mode,
            csv_sha,
        ]
        commands = (
            ("PREFLIGHT", self._ssh(laptop, LIFECYCLE_HELPER, "preflight-laptop", *common), 15.0),
            ("PREFLIGHT", [LIFECYCLE_HELPER, "preflight-hdu", *common], 10.0),
            ("PREFLIGHT", self._ssh(mdu, LIFECYCLE_HELPER, "preflight-mdu", *common), 15.0),
            ("SESSION", self._ssh(laptop, LIFECYCLE_HELPER, "prepare-laptop", *common), 45.0),
            ("SESSION", [LIFECYCLE_HELPER, "prepare-hdu", *common], 15.0),
            ("SESSION", self._ssh(mdu, LIFECYCLE_HELPER, "prepare-mdu", *common), 15.0),
            # OptiTrack is deliberately outside Runner lifecycle admission.
            # The operator may run the Laptop bridge separately when ball/base
            # telemetry is needed, but an absent Motive stream must not block
            # starting HAL and the authoritative Runner in PASSIVE.
            ("BASE_RELAY", [LIFECYCLE_HELPER, "start-base", *common], 20.0),
            ("PLANNER", [LIFECYCLE_HELPER, "start-planner", *common], 20.0),
            # agibot_pm has a 90 s systemd stop timeout on the MDU. Field
            # shutdown commonly takes 40-60 s, so do not orphan a successful
            # remote HAL transition behind an undersized SSH timeout.
            ("HAL", self._ssh(mdu, LIFECYCLE_HELPER, "start-hal", *common), 120.0),
            ("RUNNER", self._ssh(mdu, LIFECYCLE_HELPER, "start-runner", *common), 25.0),
            # The MDU AimRT participant must exist before this explicit-peer
            # HDU participant.  Reversing these two steps reproduces the field
            # failure where Runner DATA reaches the HDU UDP socket but the old
            # reader callback never matches.
            (
                "RUNNER_TRANSPORT",
                [LIFECYCLE_HELPER, "start-runner-transport", *common],
                20.0,
            ),
        )
        if mode == "KERNEL":
            commands = tuple(item for item in commands
                             if not (item[1][0] == "/usr/bin/ssh" and laptop in item[1])
                             and item[0] not in {"BASE_RELAY", "PLANNER"})
        for step, argv, timeout_s in commands:
            event_callback(HelperEvent(step=step, state="STARTING", reason="REQUESTED"))
            self._invoke(
                list(argv), timeout_s=timeout_s, event_callback=event_callback
            )

    def kill_all_and_collect(
        self, config: LifecycleConfig, session_id: str, event_callback,
        mode: str = "OPTITRACK"
    ) -> str:
        validate_session_id(session_id)
        laptop = f"{LAPTOP_USER}@{config.laptop_wifi_ip}"
        mdu = f"{ROBOT_USER}@{config.mdu_internal_ip}"
        motive_arg = config.motive_ip or "NONE"
        common = [
            session_id,
            config.laptop_wifi_ip,
            config.hdu_wifi_ip,
            config.mdu_internal_ip,
            motive_arg,
            config.table_side,
        ]
        commands = (
            ("RUNNER_HAL", self._ssh(mdu, LIFECYCLE_HELPER, "stop-mdu", *common), 30.0),
            ("PLANNER_BASE", [LIFECYCLE_HELPER, "stop-hdu", *common], 30.0),
            ("COLLECT", self._ssh(laptop, LIFECYCLE_HELPER, "collect", *common), 120.0),
        )
        if mode == "KERNEL":
            commands = tuple(item for item in commands if item[0] != "COLLECT")
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
        if mode == "KERNEL":
            return "LOGS_RETAINED_ON_ROBOT"
        if collection_reason not in {
            "LOGS_COLLECTED",
            "PARTIAL_LOGS_COLLECTED",
            "NO_REMOTE_SESSION_LOGS",
        }:
            raise LifecycleFailure(
                f"COLLECT: helper returned {collection_reason}"
            )
        return collection_reason


    def exchange_assets(self, host: str, config: LifecycleConfig, request: dict) -> dict:
        if config.revision < 1:
            raise ValueError("confirm network configuration first")
        target = (f"{LAPTOP_USER}@{config.laptop_wifi_ip}" if host == "laptop"
                  else f"{ROBOT_USER}@{config.mdu_internal_ip}")
        result = self._run(self._ssh(target, LIFECYCLE_HELPER, f"field-assets-{host}"),
                           input=json.dumps(request, ensure_ascii=False), capture_output=True,
                           text=True, timeout=30, check=False)
        if result.returncode:
            raise ValueError((result.stderr or result.stdout)[-1500:])
        return json.loads(result.stdout)


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
        self._field = FieldOperations(CONFIG_PATH.parent / "field", self._backend.exchange_assets)
        self._field_busy = False
        self._field_refresh_running = False
        self._session_mode = "OPTITRACK"
        self._field_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hope-field")
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
        self._hardware_operation_lock = None

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
        self._field_publisher = self.create_publisher(String, "/hope/field/status", 10)
        self.create_service(SetParameters, "/hope/field/command", self._field_command,
                            callback_group=self._callback_group)
        self.create_timer(5.0, self._refresh_field)
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
        field_status = self._field.status()
        field_status["busy"] = self._field_busy
        self._field_publisher.publish(self._string(json.dumps(field_status, ensure_ascii=False)))
        self._lifecycle_publishers["state"].publish(self._string(state))
        self._lifecycle_publishers["step"].publish(self._string(step))
        self._lifecycle_publishers["session"].publish(self._string(session_id))
        self._lifecycle_publishers["result"].publish(self._string(result))
        self._lifecycle_publishers["summary"].publish(
            self._string(
                f"state={state} step={step} session={session_id or 'NONE'} "
                f"config_revision={config.revision} table_side={config.table_side} "
                f"result={result}"
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

    def _refresh_field(self) -> None:
        with self._lock:
            if self._field_busy or self._field_refresh_running or self._config.revision < 1:
                return
            self._field_refresh_running = True
            config = self._config
        def refresh():
            try:
                self._field.refresh(config)
            finally:
                with self._lock:
                    self._field_refresh_running = False
        self._field_pool.submit(refresh)

    def _field_command(self, request, response):
        try:
            if (len(request.parameters) != 1 or request.parameters[0].name != "request"
                    or request.parameters[0].value.type != ParameterType.PARAMETER_STRING):
                raise ValueError("expected one string parameter named request")
            raw = request.parameters[0].value.string_value
            if len(raw.encode()) > 9 * 1024 * 1024:
                raise ValueError("field request exceeds 9 MiB")
            payload = json.loads(raw)
            if payload.get("op") in {"record_on", "record_off"}:
                # OFF must never wait behind a laptop SSH refresh or CSV upload.
                with self._lock:
                    config = self._config
                result = self._field.command(payload, config, stopped=False)
                response.results = [SetParametersResult(successful=True, reason=json.dumps(result))]
                return response
            with self._lock:
                if self._field_busy:
                    raise ValueError("another field operation is in progress")
                stopped = self._state == "STOPPED" and not self._busy
                config = self._config
                self._field_busy = True
            try:
                # Share the single asset worker with refresh so an old refresh
                # can never overwrite a just-loaded file's receipt or label.
                if payload.get("op") in {"set_mode", "upload_csv", "use_default_csv"}:
                    # Kernel Mode selections never queue behind laptop SSH.
                    result = self._field.command(payload, config, stopped=stopped)
                else:
                    result = self._field_pool.submit(
                        self._field.command, payload, config, stopped=stopped).result()
            finally:
                with self._lock:
                    self._field_busy = False
            response.results = [SetParametersResult(successful=True, reason=json.dumps(result, ensure_ascii=False))]
        except (OSError, ValueError, KeyError, TypeError, AttributeError, subprocess.SubprocessError) as exc:
            response.results = [SetParametersResult(successful=False, reason=str(exc))]
        return response

    def _apply_config(self, request, response):
        with self._lock:
            if self._busy or self._field_busy or self._state not in {"STOPPED", "CONFIG_ERROR"}:
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
                reason=(
                    "confirmed lifecycle configuration revision "
                    f"{updated.revision} table_side={updated.table_side}"
                ),
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
            if self._busy or self._field_busy or self._state != "STOPPED":
                response.success = False
                response.message = f"lifecycle start rejected in state {self._state}"
                return response
            if self._config.revision < 1:
                response.success = False
                response.message = (
                    "confirm the three Runner IPv4 fields and table side before starting"
                )
                return response
            try:
                operation_lock = try_acquire_hardware_operation_lock()
            except OSError as exc:
                response.success = False
                response.message = f"hardware-operation interlock unavailable: {exc}"
                return response
            if operation_lock is None:
                response.success = False
                response.message = (
                    "another lifecycle or time-calibration operation owns the interlock"
                )
                return response
            config = self._config
            session_id = datetime.now(timezone.utc).strftime("model21800_%Y%m%dT%H%M%SZ")
            self._busy = True
            self._state = "STARTING"
            self._step = "SESSION"
            self._session_id = session_id
            self._last_result = "START_ACCEPTED"
            self._hardware_operation_lock = operation_lock
        try:
            self._pool.submit(self._run_start, config, session_id)
        except Exception as exc:  # noqa: BLE001 - hardware has not changed yet
            with self._lock:
                self._busy = False
                self._state = "STOPPED"
                self._step = "IDLE"
                self._session_id = ""
                self._last_result = f"START_SUBMIT_FAILED: {type(exc).__name__}"
                release_hardware_operation_lock(
                    self._hardware_operation_lock
                )
                self._hardware_operation_lock = None
            response.success = False
            response.message = f"cannot start lifecycle worker: {exc}"
            return response
        response.success = True
        response.message = f"start accepted for {session_id}; follow lifecycle state topics"
        return response

    def _run_start(self, config: LifecycleConfig, session_id: str) -> None:
        try:
            with self._lock:
                self._step = "PREFLIGHT"
            mode, csv_sha = self._field.runner_selection(config)
            self._session_mode = mode
            self._backend.start(config, session_id, self._on_event, mode, csv_sha)
            self._wait_for_authoritative_runner()
        except (LifecycleFailure, OSError, ValueError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(
                f"Lifecycle start failed for {session_id}: {exc}"
            )
            with self._lock:
                managed_recovery = "MANAGED_" in str(exc)
                if self._step == "PREFLIGHT" and not managed_recovery:
                    self._state = "STOPPED"
                    self._session_id = ""
                    self._last_result = f"PREFLIGHT_REJECTED: {str(exc)[:300]}"
                    release_hardware_operation_lock(
                        self._hardware_operation_lock
                    )
                    self._hardware_operation_lock = None
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
        try:
            self._pool.submit(self._run_kill, config, session_id)
        except Exception as exc:  # noqa: BLE001 - managed system may still run
            with self._lock:
                self._busy = False
                self._state = "FAILED"
                self._last_result = f"KILL_SUBMIT_FAILED: {type(exc).__name__}"
            response.success = False
            response.message = f"cannot start lifecycle kill worker: {exc}"
            return response
        response.success = True
        response.message = (
            "kill accepted; managed robot processes may lose active support immediately"
        )
        return response

    def _run_kill(self, config: LifecycleConfig, session_id: str) -> None:
        try:
            collection_reason = self._backend.kill_all_and_collect(
                config, session_id, self._on_event, self._session_mode
            )
        except (LifecycleFailure, OSError, ValueError, subprocess.TimeoutExpired) as exc:
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
            release_hardware_operation_lock(self._hardware_operation_lock)
            self._hardware_operation_lock = None

    def close(self) -> None:
        with self._lock:
            release_hardware_operation_lock(self._hardware_operation_lock)
            self._hardware_operation_lock = None
        self._field_pool.shutdown(wait=True, cancel_futures=True)
        self._field.recorder.stop()
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
