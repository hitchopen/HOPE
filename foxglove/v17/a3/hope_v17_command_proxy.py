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
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from hope_v17_command_core import publish_x_hit_request, wait_for_x_hit_status
from hope_v17_observer_core import (
    DecodeError,
    parse_planner_attempt,
    parse_positive_pid,
    parse_session_id,
    process_cmdline_matches,
)
from hope_v17_runner_control_core import (
    MAX_EXACT_FLOAT_INTEGER,
    RunnerState,
    action_succeeded,
    decode_runner_state,
    encode_runner_request,
)


class HopeV17CommandProxy(Node):
    def __init__(self) -> None:
        super().__init__("hope_v17_command_proxy", start_parameter_services=False)
        self.declare_parameter(
            "session_id_path", "/tmp/hope_model21800_session_id"
        )
        self.declare_parameter("runtime_root", "/tmp/hope_real")
        self.declare_parameter("planner_process_fragment", "hope_planner_cpp_node")
        self.declare_parameter("x_hit_timeout_s", 2.0)
        self.declare_parameter("runner_action_timeout_s", 2.0)

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
        if self._x_hit_timeout_s <= 0.0:
            raise ValueError("x_hit_timeout_s must be positive")
        if self._runner_action_timeout_s <= 0.0:
            raise ValueError("runner_action_timeout_s must be positive")

        self._action_lock = threading.Lock()
        self._runner_state_condition = threading.Condition()
        self._runner_state: RunnerState | None = None
        self._runner_state_error = "NO RUNNER STATE"
        self._runner_request_id = time.time_ns() & (MAX_EXACT_FLOAT_INTEGER - 1)
        if self._runner_request_id == 0:
            self._runner_request_id = 1
        self._callback_group = ReentrantCallbackGroup()
        self._runner_request_publisher = self.create_publisher(
            Float64MultiArray,
            "/hope/v17/runner/control_request_flat",
            10,
        )
        self._runner_state_subscription = self.create_subscription(
            Float64MultiArray,
            "/hope/v17/runner/state_flat",
            self._runner_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._refresh_service = self.create_service(
            Trigger,
            "/hope/v17/refresh_x_hit",
            self._refresh_x_hit,
            callback_group=self._callback_group,
        )
        self._runner_services = {
            service_name: self.create_service(
                Trigger,
                f"/hope/v17/runner/{service_name}",
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
            "V17 command proxy started: seven fixed local-Runner actions plus "
            "Planner refresh_x_hit"
        )

    def _runner_state_callback(self, message: Float64MultiArray) -> None:
        try:
            state = decode_runner_state(message.data)
        except DecodeError as exc:
            with self._runner_state_condition:
                self._runner_state_error = f"MALFORMED RUNNER STATE: {exc}"
                self._runner_state_condition.notify_all()
            return
        with self._runner_state_condition:
            self._runner_state = state
            self._runner_state_error = ""
            self._runner_state_condition.notify_all()

    def _next_runner_request_id(self) -> int:
        self._runner_request_id += 1
        if self._runner_request_id > MAX_EXACT_FLOAT_INTEGER:
            self._runner_request_id = 1
        return self._runner_request_id

    def _runner_action_callback(self, action_name: str):
        def callback(
            _request: Trigger.Request, response: Trigger.Response
        ) -> Trigger.Response:
            return self._run_runner_action(action_name, response)

        return callback

    def _run_runner_action(
        self, action_name: str, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "another explicit operator action is already in progress"
            return response
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
            self._action_lock.release()

    def _current_hdu_directory(self) -> Path:
        session_id = parse_session_id(
            self._session_id_path.read_text(encoding="utf-8")
        )
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
        return hdu_dir

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
    node = HopeV17CommandProxy()
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
