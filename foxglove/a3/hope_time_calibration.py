#!/usr/bin/env python3
"""Root-owned, fixed Foxglove coordinator for the runbook 10.4 clock step."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
import os
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from hope_lifecycle_core import (
    LifecycleConfig,
    load_config,
    release_hardware_operation_lock,
    try_acquire_hardware_operation_lock,
)
from hope_time_calibration_core import (
    CalibrationFailure,
    CalibrationProgress,
    CalibrationRejected,
    CalibrationStatus,
    TimeCalibrationBackend,
    load_status,
    read_boot_id,
    save_status_atomic,
)


CONFIG_PATH = Path("/var/lib/hope-lifecycle/config.json")
STATUS_PATH = Path("/var/lib/hope-time-calibration/status.json")
ROBOT_USER = os.environ.get("HOPE_ROBOT_USER", "agi").strip()
LIFECYCLE_FRESHNESS_S = 1.5
NTP_FRESHNESS_S = 1.5


class HopeTimeCalibration(Node):
    def __init__(self):
        super().__init__(
            "hope_time_calibration",
            start_parameter_services=False,
        )
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="hope-time-calibration"
        )
        self._backend = TimeCalibrationBackend(
            ROBOT_USER,
            audit=lambda message: self.get_logger().info(
                f"Time calibration audit: {message}"
            ),
        )
        self._boot_id = read_boot_id()
        self._hardware_operation_lock = None

        self._lifecycle_state = ""
        self._lifecycle_state_received = 0.0
        self._lifecycle_busy = True
        self._lifecycle_busy_received = 0.0
        self._ntp_gate = True
        self._ntp_gate_received = 0.0

        self._status = self._load_initial_status()
        self._publishers = {
            "state": self.create_publisher(
                String, "/hope/lifecycle/time_calibration/state", 10
            ),
            "step": self.create_publisher(
                String, "/hope/lifecycle/time_calibration/step", 10
            ),
            "result": self.create_publisher(
                String, "/hope/lifecycle/time_calibration/result", 10
            ),
            "operation": self.create_publisher(
                String, "/hope/lifecycle/time_calibration/operation_id", 10
            ),
            "busy": self.create_publisher(
                Bool, "/hope/lifecycle/time_calibration/busy", 10
            ),
        }
        self.create_subscription(
            String,
            "/hope/lifecycle/state",
            self._on_lifecycle_state,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            "/hope/lifecycle/busy",
            self._on_lifecycle_busy,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            "/hope/ntp/gate_pass",
            self._on_ntp_gate,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/hope/lifecycle/time_calibration",
            self._request_calibration,
            callback_group=self._callback_group,
        )
        self.create_timer(0.5, self._publish)

        if self._status.state in {"INTERRUPTED", "FAILED_SAFE_STOP"}:
            try:
                self._hardware_operation_lock = (
                    try_acquire_hardware_operation_lock()
                )
            except OSError as exc:
                self.get_logger().error(
                    f"Cannot acquire failed-maintenance interlock: {exc}"
                )
            if self._hardware_operation_lock is None:
                self.get_logger().error(
                    "Failed maintenance cannot own the hardware-operation interlock"
                )
        if (
            self._status.state == "INTERRUPTED"
            and self._hardware_operation_lock is not None
        ):
            self._pool.submit(self._recover_interrupted_operation)

    @staticmethod
    def _string(value: str) -> String:
        message = String()
        message.data = value
        return message

    @staticmethod
    def _clean_result(value: object) -> str:
        cleaned = " ".join(str(value).split())
        return cleaned[:800] if cleaned else "UNKNOWN_FAILURE"

    def _load_initial_status(self) -> CalibrationStatus:
        try:
            saved = load_status(STATUS_PATH)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f"Failing closed on calibration status error: {exc}")
            status = CalibrationStatus(
                state="FAILED_SAFE_STOP",
                step="STATUS_ERROR",
                result=self._clean_result(f"STATUS_FILE_ERROR: {exc}"),
                boot_id=self._boot_id,
                hard_step_attempted=True,
            )
            save_status_atomic(STATUS_PATH, status)
            return status
        if saved is None or saved.boot_id != self._boot_id:
            status = CalibrationStatus(boot_id=self._boot_id)
            save_status_atomic(STATUS_PATH, status)
            return status
        if saved.state == "RUNNING":
            interrupted = replace(
                saved,
                state="INTERRUPTED",
                step="FAIL_SAFE",
                result="COORDINATOR_RESTARTED_DURING_CALIBRATION",
            )
            save_status_atomic(STATUS_PATH, interrupted)
            return interrupted
        return saved

    def _replace_status(self, **updates: object) -> CalibrationStatus:
        with self._lock:
            status = replace(self._status, **updates)
            save_status_atomic(STATUS_PATH, status)
            self._status = status
            return status

    def _release_operation_lock(self) -> None:
        with self._lock:
            operation_lock = self._hardware_operation_lock
            self._hardware_operation_lock = None
        release_hardware_operation_lock(operation_lock)

    def _publish(self) -> None:
        with self._lock:
            status = self._status
        self._publishers["state"].publish(self._string(status.state))
        self._publishers["step"].publish(self._string(status.step))
        self._publishers["result"].publish(self._string(status.result))
        self._publishers["operation"].publish(self._string(status.operation_id))
        busy = Bool()
        busy.data = status.state in {"RUNNING", "INTERRUPTED"}
        self._publishers["busy"].publish(busy)

    def _on_lifecycle_state(self, message: String) -> None:
        rearm = False
        with self._lock:
            self._lifecycle_state = str(message.data)
            self._lifecycle_state_received = time.monotonic()
            rearm = (
                self._lifecycle_state == "RUNNING"
                and self._status.state == "COMPLETE"
            )
        if rearm:
            self._replace_status(
                state="IDLE",
                step="IDLE",
                result="REARMED_AFTER_COMPLETED_LIFECYCLE",
                operation_id="",
                hard_step_attempted=False,
                active_hdu_vendor=(),
                active_hdu_foxglove=(),
            )

    def _on_lifecycle_busy(self, message: Bool) -> None:
        with self._lock:
            self._lifecycle_busy = bool(message.data)
            self._lifecycle_busy_received = time.monotonic()

    def _on_ntp_gate(self, message: Bool) -> None:
        with self._lock:
            self._ntp_gate = bool(message.data)
            self._ntp_gate_received = time.monotonic()

    def _request_calibration(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        now = time.monotonic()
        with self._lock:
            status = self._status
            lifecycle_state = self._lifecycle_state
            lifecycle_state_fresh = (
                self._lifecycle_state_received > 0.0
                and now - self._lifecycle_state_received <= LIFECYCLE_FRESHNESS_S
            )
            lifecycle_busy = self._lifecycle_busy
            lifecycle_busy_fresh = (
                self._lifecycle_busy_received > 0.0
                and now - self._lifecycle_busy_received <= LIFECYCLE_FRESHNESS_S
            )
            ntp_gate = self._ntp_gate
            ntp_gate_fresh = (
                self._ntp_gate_received > 0.0
                and now - self._ntp_gate_received <= NTP_FRESHNESS_S
            )

        if status.state in {"RUNNING", "INTERRUPTED"}:
            response.success = False
            response.message = "time calibration is already running"
            return response
        if status.state == "FAILED_SAFE_STOP" and status.boot_id == self._boot_id:
            response.success = False
            response.message = (
                "previous calibration failed this boot; keep robot services stopped "
                "and follow the documented manual recovery"
            )
            return response
        if status.hard_step_attempted and status.boot_id == self._boot_id:
            response.success = False
            response.message = (
                "one hard-step was already attempted in this maintenance cycle"
            )
            return response
        if not lifecycle_state_fresh or not lifecycle_busy_fresh:
            response.success = False
            response.message = "fresh lifecycle STOPPED state is unavailable"
            return response
        if lifecycle_state != "STOPPED" or lifecycle_busy:
            response.success = False
            response.message = (
                "time calibration requires idle STOPPED lifecycle; "
                f"current state={lifecycle_state}"
            )
            return response
        if not ntp_gate_fresh or ntp_gate:
            response.success = False
            response.message = "time calibration requires a fresh failing NTP gate"
            return response
        try:
            config = load_config(CONFIG_PATH)
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = f"cannot load confirmed lifecycle configuration: {exc}"
            return response
        if config.revision < 1:
            response.success = False
            response.message = "confirm the four lifecycle IPv4 fields first"
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
        with self._lock:
            if (
                self._hardware_operation_lock is not None
                or self._status.state
                in {"RUNNING", "INTERRUPTED", "FAILED_SAFE_STOP"}
            ):
                release_hardware_operation_lock(operation_lock)
                response.success = False
                response.message = "time calibration state changed; request again"
                return response
            self._hardware_operation_lock = operation_lock

        operation_id = datetime.now(timezone.utc).strftime("timecal_%Y%m%dT%H%M%SZ")
        try:
            self._replace_status(
                state="RUNNING",
                step="HANDOFF",
                result="REQUEST_ACCEPTED_CONTROL_CONNECTION_WILL_RESTART",
                operation_id=operation_id,
                boot_id=self._boot_id,
                hard_step_attempted=False,
                active_hdu_vendor=(),
                active_hdu_foxglove=(),
            )
        except Exception as exc:
            self._release_operation_lock()
            response.success = False
            response.message = f"cannot persist calibration request: {exc}"
            return response
        try:
            self._pool.submit(self._run_calibration, config)
        except Exception as exc:  # noqa: BLE001 - request has not mutated hardware
            self._replace_status(
                state="REJECTED",
                step="HANDOFF",
                result=self._clean_result(f"WORKER_SUBMIT_FAILED: {exc}"),
            )
            self._release_operation_lock()
            response.success = False
            response.message = f"cannot start calibration worker: {exc}"
            return response
        response.success = True
        response.message = (
            f"accepted {operation_id}; port 8766 will disconnect during the hard-step "
            "and return after control services are restored"
        )
        return response

    def _on_progress(self, progress: CalibrationProgress) -> None:
        self.get_logger().info(
            f"Time calibration {progress.step}: {progress.result}"
        )
        self._replace_status(
            state="RUNNING",
            step=progress.step,
            result=self._clean_result(progress.result),
            hard_step_attempted=progress.hard_step_attempted,
            active_hdu_vendor=progress.active_hdu_vendor,
            active_hdu_foxglove=progress.active_hdu_foxglove,
        )

    def _run_calibration(self, config: LifecycleConfig) -> None:
        try:
            self._backend.calibrate(config, self._on_progress)
        except CalibrationRejected as exc:
            self.get_logger().warning(f"Time calibration rejected: {exc}")
            self._replace_status(
                state="REJECTED",
                step="PREFLIGHT",
                result=self._clean_result(exc),
            )
            self._release_operation_lock()
            return
        except (CalibrationFailure, OSError, subprocess.SubprocessError) as exc:
            self.get_logger().error(f"Time calibration failed safe: {exc}")
            self._replace_status(
                state="FAILED_SAFE_STOP",
                step="FAIL_SAFE",
                result=self._clean_result(exc),
            )
            return
        except Exception as exc:  # noqa: BLE001 - never leave maintenance busy
            self.get_logger().error(
                f"Unexpected time-calibration failure: {type(exc).__name__}: {exc}"
            )
            with self._lock:
                active_foxglove = self._status.active_hdu_foxglove
            recovery_errors = self._backend.fail_safe(
                config, active_hdu_foxglove=active_foxglove
            )
            detail = f"INTERNAL_ERROR {type(exc).__name__}: {exc}"
            if recovery_errors:
                detail += " | " + "; ".join(recovery_errors)
            self._replace_status(
                state="FAILED_SAFE_STOP",
                step="FAIL_SAFE",
                result=self._clean_result(detail),
            )
            return
        with self._lock:
            result = self._status.result
        self._replace_status(state="COMPLETE", step="COMPLETE", result=result)
        self._release_operation_lock()

    def _recover_interrupted_operation(self) -> None:
        try:
            config = load_config(CONFIG_PATH)
            with self._lock:
                active_foxglove = self._status.active_hdu_foxglove
            errors = self._backend.fail_safe(
                config, active_hdu_foxglove=active_foxglove
            )
            result = "INTERRUPTED_OPERATION_ROBOT_SERVICES_STOPPED_CONTROL_RESTORED"
            if errors:
                result += " | " + "; ".join(errors)
        except Exception as exc:  # noqa: BLE001
            result = f"INTERRUPTED_RECOVERY_FAILED: {type(exc).__name__}: {exc}"
        self._replace_status(
            state="FAILED_SAFE_STOP",
            step="FAIL_SAFE",
            result=self._clean_result(result),
        )

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)
        self._release_operation_lock()


def main() -> None:
    rclpy.init()
    node = HopeTimeCalibration()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
