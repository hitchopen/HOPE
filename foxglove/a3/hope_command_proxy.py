#!/usr/bin/env python3
"""Explicit Foxglove actions for the local model_21800 Runner and Planner.

Every service maps to one frozen action.  The node accepts no shell, argv,
path, PID, signal, arbitrary mode, topic payload, or parameter from clients.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from hope_command_core import (
    calibration_receipt_id,
    parse_calibration_receipt_sha,
    publish_x_hit_request,
    wait_for_x_hit_status,
)
from hope_observer_core import (
    DecodeError,
    decode_base_packet,
    parse_planner_attempt,
    parse_positive_pid,
    parse_session_id,
    process_cmdline_matches,
)
from hope_runner_control_core import (
    MAX_EXACT_FLOAT_INTEGER,
    RunnerState,
    action_succeeded,
    decode_runner_state,
    encode_runner_request,
    runner_session_fingerprint,
)


class HopeCommandProxy(Node):
    def __init__(self) -> None:
        super().__init__("hope_command_proxy", start_parameter_services=False)
        self.declare_parameter(
            "session_id_path", "/tmp/hope_model21800_session_id"
        )
        self.declare_parameter("runtime_root", "/tmp/hope_real")
        self.declare_parameter("planner_process_fragment", "hope_planner_cpp_node")
        self.declare_parameter("x_hit_timeout_s", 5.0)
        self.declare_parameter("runner_action_timeout_s", 2.0)
        self.declare_parameter(
            "calibration_service", "/a3/calibration/recompute_p1"
        )
        self.declare_parameter("calibration_timeout_s", 25.0)
        self.declare_parameter("base_pose_timeout_s", 5.0)
        self.declare_parameter("runner_state_stale_after_s", 1.5)

        self._session_id_path = Path(
            str(self.get_parameter("session_id_path").value)
        )
        self._runtime_root = Path(str(self.get_parameter("runtime_root").value))
        self._planner_process_fragment = str(
            self.get_parameter("planner_process_fragment").value
        )
        self._x_hit_timeout_s = float(self.get_parameter("x_hit_timeout_s").value)
        self._runner_action_timeout_s = float(
            self.get_parameter("runner_action_timeout_s").value
        )
        self._calibration_timeout_s = float(
            self.get_parameter("calibration_timeout_s").value
        )
        self._base_pose_timeout_s = float(
            self.get_parameter("base_pose_timeout_s").value
        )
        self._runner_state_stale_after_s = float(
            self.get_parameter("runner_state_stale_after_s").value
        )
        if self._x_hit_timeout_s <= 0.0:
            raise ValueError("x_hit_timeout_s must be positive")
        if self._runner_action_timeout_s <= 0.0:
            raise ValueError("runner_action_timeout_s must be positive")
        if self._calibration_timeout_s <= 0.0:
            raise ValueError("calibration_timeout_s must be positive")
        if self._base_pose_timeout_s <= 0.0:
            raise ValueError("base_pose_timeout_s must be positive")
        if self._runner_state_stale_after_s <= 0.0:
            raise ValueError("runner_state_stale_after_s must be positive")

        self._action_lock = threading.Lock()
        self._runner_request_lock = threading.Lock()
        self._runner_state_condition = threading.Condition()
        self._runner_state: RunnerState | None = None
        self._runner_state_received_monotonic = 0.0
        self._runner_state_error = "NO RUNNER STATE"
        self._base_packet_condition = threading.Condition()
        self._base_packet = None
        self._base_packet_received_monotonic = 0.0
        self._base_packet_error = "NO BASE POSE"
        self._runner_request_id = time.time_ns() & (MAX_EXACT_FLOAT_INTEGER - 1)
        if self._runner_request_id == 0:
            self._runner_request_id = 1
        self._callback_group = ReentrantCallbackGroup()
        self._calibration_client = self.create_client(
            Trigger,
            str(self.get_parameter("calibration_service").value),
            callback_group=self._callback_group,
        )
        receipt_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._calibration_success_publisher = self.create_publisher(
            Bool, "/hope/calibration/success", receipt_qos
        )
        self._calibration_status_publisher = self.create_publisher(
            String, "/hope/calibration/status", receipt_qos
        )
        self._publish_calibration_status(
            False, "NOT CALIBRATED SINCE COMMAND PROXY START"
        )
        self._runner_request_publisher = self.create_publisher(
            Float64MultiArray,
            "/hope/runner/control_request_flat",
            10,
        )
        self._runner_state_subscription = self.create_subscription(
            Float64MultiArray,
            "/hope/runner/state_flat",
            self._runner_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._base_pose_subscription = self.create_subscription(
            Float64MultiArray,
            "/a3/base_pose_flat",
            self._base_pose_callback,
            10,
            callback_group=self._callback_group,
        )
        self._calibration_service = self.create_service(
            Trigger,
            "/hope/calibrate",
            self._calibrate,
            callback_group=self._callback_group,
        )
        self._refresh_service = self.create_service(
            Trigger,
            "/hope/refresh_x_hit",
            self._refresh_x_hit,
            callback_group=self._callback_group,
        )
        self._runner_services = {
            service_name: self.create_service(
                Trigger,
                f"/hope/runner/{service_name}",
                self._runner_action_callback(action_name),
                callback_group=self._callback_group,
            )
            for service_name, action_name in (
                ("set_server", "SET_SERVER"),
                ("set_receiver", "SET_RECEIVER"),
                ("enter_pd_stand", "ENTER_PD_STAND"),
                ("enter_motion", "ENTER_MOTION"),
                ("emergency_passive", "EMERGENCY_PASSIVE"),
                ("ready_to_serve", "READY_TO_SERVE"),
                ("serve", "SERVE"),
            )
        }
        self.get_logger().info(
            "Runner command proxy started: seven fixed local-Runner actions plus "
            "separate world-to-pelvis calibration and Planner refresh_x_hit"
        )

    def _runner_state_callback(self, message: Float64MultiArray) -> None:
        try:
            state = decode_runner_state(message.data)
        except DecodeError as exc:
            with self._runner_state_condition:
                self._runner_state_error = f"MALFORMED RUNNER STATE: {exc}"
                self._runner_state_condition.notify_all()
            return
        session_changed = False
        with self._runner_state_condition:
            previous = self._runner_state
            session_changed = bool(
                previous is not None
                and previous.session_fingerprint != state.session_fingerprint
            )
            self._runner_state = state
            self._runner_state_received_monotonic = time.monotonic()
            self._runner_state_error = ""
            self._runner_state_condition.notify_all()
        if session_changed:
            self._publish_calibration_status(
                False, "NOT CALIBRATED FOR CURRENT RUNNER SESSION"
            )

    def _base_pose_callback(self, message: Float64MultiArray) -> None:
        try:
            packet = decode_base_packet(message.data)
        except DecodeError as exc:
            with self._base_packet_condition:
                self._base_packet_error = f"MALFORMED BASE POSE: {exc}"
                self._base_packet_condition.notify_all()
            return
        with self._base_packet_condition:
            self._base_packet = packet
            self._base_packet_received_monotonic = time.monotonic()
            self._base_packet_error = "" if packet.valid else packet.reason
            self._base_packet_condition.notify_all()

    def _next_runner_request_id(self) -> int:
        with self._runner_request_lock:
            self._runner_request_id += 1
            if self._runner_request_id > MAX_EXACT_FLOAT_INTEGER:
                self._runner_request_id = 1
            return self._runner_request_id

    def _runner_action_callback(self, action_name: str):
        def callback(
            _request: Trigger.Request, response: Trigger.Response
        ) -> Trigger.Response:
            return self._run_runner_action(
                action_name,
                response,
                urgent=action_name == "EMERGENCY_PASSIVE",
            )

        return callback

    def _run_runner_action(
        self,
        action_name: str,
        response: Trigger.Response,
        *,
        urgent: bool = False,
    ) -> Trigger.Response:
        lock_acquired = False
        if not urgent and not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "another explicit operator action is already in progress"
            return response
        if not urgent:
            lock_acquired = True
        try:
            request_id = self._next_runner_request_id()
            message = Float64MultiArray()
            message.data = encode_runner_request(request_id, action_name)
            self._runner_request_publisher.publish(message)

            deadline = time.monotonic() + self._runner_action_timeout_s
            with self._runner_state_condition:
                while True:
                    state = self._runner_state
                    if state is not None and state.last_action_id == request_id:
                        response.success = action_succeeded(
                            state.last_action_result
                        )
                        response.message = (
                            f"request={request_id} action={state.last_action} "
                            f"result={state.last_action_result} "
                            f"reason={state.last_action_reason} "
                            f"boot={state.boot_id} seq={state.state_sequence} "
                            f"mode={state.run_mode} role={state.local_role}"
                        )
                        return response
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        response.success = False
                        response.message = (
                            f"request={request_id} action={action_name} not confirmed "
                            f"within {self._runner_action_timeout_s:.1f}s; "
                            f"{self._runner_state_error or 'latest Runner state did not acknowledge it'}"
                        )
                        return response
                    self._runner_state_condition.wait(timeout=remaining)
        except (DecodeError, ValueError) as exc:
            response.success = False
            response.message = f"Runner action was not published: {exc}"
            return response
        finally:
            if lock_acquired:
                self._action_lock.release()

    def _current_session_and_hdu_directory(self) -> tuple[str, Path]:
        session_id = self._current_session_id()
        hdu_dir = self._runtime_root / session_id / "hdu"
        attempt = parse_planner_attempt(
            (hdu_dir / "current_planner_attempt").read_text(encoding="utf-8")
        )
        pid = parse_positive_pid(
            (hdu_dir / attempt / "pid.txt").read_text(encoding="utf-8")
        )
        cmdline = (Path("/proc") / str(pid) / "cmdline").read_bytes()
        if not process_cmdline_matches(cmdline, self._planner_process_fragment):
            raise DecodeError(
                "recorded Planner PID is absent or does not match hope_planner_cpp_node"
            )
        return session_id, hdu_dir

    def _current_session_id(self) -> str:
        return parse_session_id(
            self._session_id_path.read_text(encoding="utf-8")
        )

    def _current_hdu_directory(self) -> Path:
        return self._current_session_and_hdu_directory()[1]

    def _runner_pd_stand_error(self, session_id: str) -> str:
        with self._runner_state_condition:
            state = self._runner_state
            received = self._runner_state_received_monotonic
            state_error = self._runner_state_error
        if (
            state is None
            or received <= 0.0
            or time.monotonic() - received > self._runner_state_stale_after_s
        ):
            return state_error or "Runner state is absent or stale"
        if state.session_fingerprint != runner_session_fingerprint(session_id):
            return "Runner state does not match the managed lifecycle session"
        if state.command_fault_latched:
            return "Runner command fault is latched"
        if state.run_mode != "PD_STAND":
            return f"Runner mode is {state.run_mode}; settled PD_STAND is required"
        return ""

    def _publish_calibration_status(self, success: bool, detail: str) -> None:
        self._calibration_success_publisher.publish(Bool(data=success))
        self._calibration_status_publisher.publish(String(data=detail))

    @staticmethod
    def _wait_for_future(future, timeout_s: float) -> bool:
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        return completed.wait(timeout=timeout_s)

    def _wait_for_calibrated_base(
        self, expected_calibration_id: int, not_before_monotonic: float
    ) -> None:
        deadline = time.monotonic() + self._base_pose_timeout_s
        with self._base_packet_condition:
            while True:
                packet = self._base_packet
                received = self._base_packet_received_monotonic
                if (
                    packet is not None
                    and packet.valid
                    and packet.calibration_id == expected_calibration_id
                    and received >= not_before_monotonic
                    and time.monotonic() - received <= 1.0
                ):
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        "new calibration receipt was not observed in fresh "
                        f"/a3/base_pose_flat: {self._base_packet_error or 'receipt mismatch'}"
                    )
                self._base_packet_condition.wait(timeout=remaining)

    def _calibrate(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "another explicit operator action is already in progress"
            return response
        self._publish_calibration_status(False, "CALIBRATION STARTING")
        try:
            try:
                session_id = self._current_session_id()
                runner_error = self._runner_pd_stand_error(session_id)
                if runner_error:
                    raise DecodeError(runner_error)
                if not self._calibration_client.service_is_ready():
                    raise DecodeError(
                        "laptop /a3/calibration/recompute_p1 service is unavailable"
                    )

                self._publish_calibration_status(
                    False,
                    "CAPTURING 10 MARKERS · deriving world→pelvis JSON snapshot",
                )
                future = self._calibration_client.call_async(Trigger.Request())
                if not self._wait_for_future(future, self._calibration_timeout_s):
                    future.cancel()
                    raise TimeoutError("10-marker calibration service timed out")
                try:
                    calibration_response = future.result()
                except Exception as exc:  # noqa: BLE001 - surface ROS call failure
                    raise DecodeError(
                        f"10-marker calibration service call failed: {exc}"
                    ) from exc
                if calibration_response is None:
                    raise DecodeError("10-marker calibration returned no response")
                if not calibration_response.success:
                    raise DecodeError(
                        calibration_response.message or "10-marker calibration failed"
                    )
                receipt_sha = parse_calibration_receipt_sha(
                    calibration_response.message
                )
                receipt_id = calibration_receipt_id(receipt_sha)
                calibration_completed = time.monotonic()

                self._publish_calibration_status(
                    False, "WAITING FOR FRESH world→pelvis_link RECEIPT"
                )
                self._wait_for_calibrated_base(receipt_id, calibration_completed)
                runner_error = self._runner_pd_stand_error(session_id)
                if runner_error:
                    raise DecodeError(
                        f"Runner left safe calibration state: {runner_error}"
                    )

                self._publish_calibration_status(
                    False,
                    "WORLD→PELVIS READY · JSON receipt persisted; x_hit unchanged",
                )
            except (OSError, DecodeError, TimeoutError, ValueError) as exc:
                detail = str(exc)
                self._publish_calibration_status(False, f"CALIBRATION FAILED · {detail}")
                response.success = False
                response.message = f"calibration not confirmed: {detail}"
                return response

            detail = (
                f"calibration_sha={receipt_sha} base_calibration_id={receipt_id} "
                "world_to_pelvis_snapshot=persisted x_hit=unchanged"
            )
            self._publish_calibration_status(True, f"CALIBRATION COMPLETE · {detail}")
            response.success = True
            response.message = detail
            return response
        finally:
            self._action_lock.release()

    def _refresh_x_hit(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "another explicit operator action is already in progress"
            return response
        try:
            try:
                hdu_dir = self._current_hdu_directory()
                request_id = str(time.time_ns())
                publish_x_hit_request(hdu_dir / "x_hit.request", request_id)
                status = wait_for_x_hit_status(
                    hdu_dir / "x_hit.status",
                    request_id,
                    timeout_s=self._x_hit_timeout_s,
                )
            except FileExistsError:
                response.success = False
                response.message = (
                    "x_hit.request already exists; it was not overwritten. "
                    "Inspect Planner and the existing request before retrying"
                )
                return response
            except (OSError, DecodeError, TimeoutError, ValueError) as exc:
                response.success = False
                response.message = f"refresh_x_hit not confirmed: {exc}"
                return response

            response.success = status.success
            response.message = (
                f"request={status.request_id} success={int(status.success)} "
                f"{status.message}"
            )
            return response
        finally:
            self._action_lock.release()


def main() -> None:
    rclpy.init()
    node = HopeCommandProxy()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
