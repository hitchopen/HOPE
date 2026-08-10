#!/usr/bin/env python3
"""Read-only Foxglove telemetry for the model_21800 hardware trial.

This node never starts, stops, signals, calibrates, or controls another
process.  It translates existing ROS wires and local audit files into small
standard-message topics already covered by the fleet bridge's ``/hope/.*``
topic allowlist.
"""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64, Float64MultiArray, String, UInt64

from hope_v17_observer_core import (
    BasePacket,
    DecodeError,
    RacketPacket,
    XHitStatus,
    decode_base_packet,
    decode_racket_packet,
    parse_planner_attempt,
    parse_positive_pid,
    parse_session_id,
    parse_x_hit_status,
    process_cmdline_matches,
)
from hope_v17_runner_control_core import (
    RunnerState,
    decode_runner_state,
    opponent_expected_role,
    runner_session_fingerprint,
)


def _sensor_qos(depth: int = 4) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class HopeV17Observer(Node):
    def __init__(self) -> None:
        super().__init__("hope_v17_observer", start_parameter_services=False)
        self.declare_parameter(
            "session_id_path", "/tmp/hope_model21800_session_id"
        )
        self.declare_parameter("runtime_root", "/tmp/hope_real")
        self.declare_parameter("base_stale_after_s", 0.5)
        self.declare_parameter("ball_stale_after_s", 0.1)
        self.declare_parameter("publish_period_s", 0.2)
        self.declare_parameter("runner_stale_after_s", 1.0)
        self.declare_parameter("planner_process_fragment", "hope_planner_cpp_node")

        self._session_id_path = Path(
            str(self.get_parameter("session_id_path").value)
        )
        self._runtime_root = Path(str(self.get_parameter("runtime_root").value))
        self._base_stale_after_s = float(
            self.get_parameter("base_stale_after_s").value
        )
        self._ball_stale_after_s = float(
            self.get_parameter("ball_stale_after_s").value
        )
        publish_period_s = float(self.get_parameter("publish_period_s").value)
        self._runner_stale_after_s = float(
            self.get_parameter("runner_stale_after_s").value
        )
        self._planner_process_fragment = str(
            self.get_parameter("planner_process_fragment").value
        )
        if self._base_stale_after_s <= 0.0:
            raise ValueError("base_stale_after_s must be positive")
        if self._ball_stale_after_s <= 0.0:
            raise ValueError("ball_stale_after_s must be positive")
        if publish_period_s <= 0.0:
            raise ValueError("publish_period_s must be positive")
        if self._runner_stale_after_s <= 0.0:
            raise ValueError("runner_stale_after_s must be positive")

        self._base_packet: BasePacket | None = None
        self._base_error = "NO BASE PACKET"
        self._base_receipt_ns = 0
        self._command_packet: RacketPacket | None = None
        self._command_error = "NO RACKET PACKET"
        self._command_receipt_ns = 0
        self._ball_receipts_ns: deque[int] = deque(maxlen=512)
        self._runner_state: RunnerState | None = None
        self._runner_state_error = "NO RUNNER STATE"
        self._runner_state_receipt_ns = 0

        self._topic_publishers = {
            "observer_alive": self.create_publisher(
                Bool, "/hope/v17/observer_alive", 10
            ),
            "mdu_active": self.create_publisher(
                Bool, "/hope/v17/system/mdu_active", 10
            ),
            "mdu_text": self.create_publisher(
                String, "/hope/v17/system/mdu_text", 10
            ),
            "session_active": self.create_publisher(
                Bool, "/hope/v17/session/active", 10
            ),
            "session_id": self.create_publisher(
                String, "/hope/v17/session/id", 10
            ),
            "session_text": self.create_publisher(
                String, "/hope/v17/session/text", 10
            ),
            "planner_alive": self.create_publisher(
                Bool, "/hope/v17/planner/process_alive", 10
            ),
            "planner_attempt": self.create_publisher(
                String, "/hope/v17/planner/attempt", 10
            ),
            "base_seen": self.create_publisher(Bool, "/hope/v17/base/seen", 10),
            "base_valid": self.create_publisher(Bool, "/hope/v17/base/valid", 10),
            "base_fresh": self.create_publisher(Bool, "/hope/v17/base/fresh", 10),
            "base_receipt_age_ms": self.create_publisher(
                Float64, "/hope/v17/base/receipt_age_ms", 10
            ),
            "base_source_age_ms": self.create_publisher(
                Float64, "/hope/v17/base/source_age_ms", 10
            ),
            "base_x": self.create_publisher(
                Float64, "/hope/v17/base/position_x_m", 10
            ),
            "base_y": self.create_publisher(
                Float64, "/hope/v17/base/position_y_m", 10
            ),
            "base_z": self.create_publisher(
                Float64, "/hope/v17/base/position_z_m", 10
            ),
            "base_summary": self.create_publisher(
                String, "/hope/v17/base/summary", 10
            ),
            "ball_live": self.create_publisher(Bool, "/hope/v17/ball/live", 10),
            "ball_receipt_age_ms": self.create_publisher(
                Float64, "/hope/v17/ball/receipt_age_ms", 10
            ),
            "ball_rate_hz": self.create_publisher(
                Float64, "/hope/v17/ball/rate_hz", 10
            ),
            "command_seen": self.create_publisher(
                Bool, "/hope/v17/command/seen", 10
            ),
            "command_valid": self.create_publisher(
                Bool, "/hope/v17/command/valid", 10
            ),
            "command_receipt_age_ms": self.create_publisher(
                Float64, "/hope/v17/command/receipt_age_ms", 10
            ),
            "command_countdown_s": self.create_publisher(
                Float64, "/hope/v17/command/hdu_wall_countdown_s", 10
            ),
            "command_sequence": self.create_publisher(
                UInt64, "/hope/v17/command/sequence", 10
            ),
            "command_flight": self.create_publisher(
                UInt64, "/hope/v17/command/flight_id", 10
            ),
            "command_revision": self.create_publisher(
                UInt64, "/hope/v17/command/revision_id", 10
            ),
            "command_summary": self.create_publisher(
                String, "/hope/v17/command/summary", 10
            ),
            "x_hit_available": self.create_publisher(
                Bool, "/hope/v17/x_hit/status_available", 10
            ),
            "x_hit_success": self.create_publisher(
                Bool, "/hope/v17/x_hit/success", 10
            ),
            "x_hit_value": self.create_publisher(
                Float64, "/hope/v17/x_hit/value_m", 10
            ),
            "x_hit_status": self.create_publisher(
                String, "/hope/v17/x_hit/status", 10
            ),
            "runner_alive": self.create_publisher(
                Bool, "/hope/v17/runner/alive", 10
            ),
            "runner_boot_id": self.create_publisher(
                UInt64, "/hope/v17/runner/boot_id", 10
            ),
            "runner_session_id": self.create_publisher(
                String, "/hope/v17/runner/session_id", 10
            ),
            "runner_session_matches": self.create_publisher(
                Bool, "/hope/v17/runner/session_matches", 10
            ),
            "runner_state_sequence": self.create_publisher(
                UInt64, "/hope/v17/runner/state_sequence", 10
            ),
            "runner_mode": self.create_publisher(
                String, "/hope/v17/runner/mode", 10
            ),
            "runner_command_publishing": self.create_publisher(
                Bool, "/hope/v17/runner/command_publishing", 10
            ),
            "runner_policy_native": self.create_publisher(
                Bool, "/hope/v17/runner/policy_native", 10
            ),
            "runner_command_fault": self.create_publisher(
                Bool, "/hope/v17/runner/command_fault_latched", 10
            ),
            "runner_local_role": self.create_publisher(
                String, "/hope/v17/runner/local_role", 10
            ),
            "runner_role_epoch": self.create_publisher(
                UInt64, "/hope/v17/runner/role_epoch", 10
            ),
            "runner_role_change_allowed": self.create_publisher(
                Bool, "/hope/v17/runner/role_change_allowed", 10
            ),
            "runner_role_last_result": self.create_publisher(
                String, "/hope/v17/runner/role_last_result", 10
            ),
            "runner_serve_capability": self.create_publisher(
                String, "/hope/v17/runner/serve_capability", 10
            ),
            "runner_serve_state": self.create_publisher(
                String, "/hope/v17/runner/serve_state", 10
            ),
            "runner_standing": self.create_publisher(
                Bool, "/hope/v17/runner/standing", 10
            ),
            "runner_ready": self.create_publisher(
                Bool, "/hope/v17/runner/ready", 10
            ),
            "runner_ready_to_serve": self.create_publisher(
                Bool, "/hope/v17/runner/is_ready_to_serve", 10
            ),
            "runner_serving": self.create_publisher(
                Bool, "/hope/v17/runner/serving", 10
            ),
            "runner_last_action_id": self.create_publisher(
                UInt64, "/hope/v17/runner/last_action_id", 10
            ),
            "runner_last_action": self.create_publisher(
                String, "/hope/v17/runner/last_action", 10
            ),
            "runner_last_action_result": self.create_publisher(
                String, "/hope/v17/runner/last_action_result", 10
            ),
            "runner_last_action_reason": self.create_publisher(
                String, "/hope/v17/runner/last_action_reason", 10
            ),
            "runner_summary": self.create_publisher(
                String, "/hope/v17/runner/summary", 10
            ),
            "opponent_expected_role": self.create_publisher(
                String, "/hope/v17/opponent/expected_role", 10
            ),
            "opponent_role_source": self.create_publisher(
                String, "/hope/v17/opponent/role_source", 10
            ),
            "opponent_role_confirmed": self.create_publisher(
                Bool, "/hope/v17/opponent/role_confirmed", 10
            ),
            "opponent_summary": self.create_publisher(
                String, "/hope/v17/opponent/summary", 10
            ),
        }

        self.create_subscription(
            Float64MultiArray,
            "/a3/base_pose_flat",
            self._base_callback,
            _sensor_qos(),
        )
        self.create_subscription(
            Float64MultiArray,
            "/racket/command_flat",
            self._command_callback,
            _sensor_qos(),
        )
        self.create_subscription(PoseArray, "/poses", self._ball_callback, _sensor_qos())
        self.create_subscription(
            Float64MultiArray,
            "/hope/v17/runner/state_flat",
            self._runner_state_callback,
            _sensor_qos(),
        )
        self.create_timer(publish_period_s, self._publish_snapshot)
        self.get_logger().info(
            "read-only model_21800 observer started; Runner state is decoded, "
            "and no process or control APIs are exposed"
        )

    @staticmethod
    def _bool_message(value: bool) -> Bool:
        message = Bool()
        message.data = bool(value)
        return message

    @staticmethod
    def _float_message(value: float) -> Float64:
        message = Float64()
        message.data = float(value)
        return message

    @staticmethod
    def _string_message(value: str) -> String:
        message = String()
        message.data = str(value)
        return message

    @staticmethod
    def _uint64_message(value: int) -> UInt64:
        message = UInt64()
        message.data = int(value)
        return message

    def _base_callback(self, message: Float64MultiArray) -> None:
        self._base_receipt_ns = time.monotonic_ns()
        try:
            self._base_packet = decode_base_packet(message.data)
            self._base_error = self._base_packet.reason
        except DecodeError as exc:
            self._base_packet = None
            self._base_error = f"MALFORMED BASE PACKET: {exc}"

    def _command_callback(self, message: Float64MultiArray) -> None:
        self._command_receipt_ns = time.monotonic_ns()
        try:
            self._command_packet = decode_racket_packet(message.data)
            self._command_error = self._command_packet.reason
        except DecodeError as exc:
            self._command_packet = None
            self._command_error = f"MALFORMED RACKET PACKET: {exc}"

    def _ball_callback(self, _message: PoseArray) -> None:
        self._ball_receipts_ns.append(time.monotonic_ns())

    def _runner_state_callback(self, message: Float64MultiArray) -> None:
        self._runner_state_receipt_ns = time.monotonic_ns()
        try:
            self._runner_state = decode_runner_state(message.data)
            self._runner_state_error = ""
        except DecodeError as exc:
            self._runner_state = None
            self._runner_state_error = f"MALFORMED RUNNER STATE: {exc}"

    @staticmethod
    def _receipt_age_ms(now_ns: int, receipt_ns: int) -> float:
        if receipt_ns <= 0:
            return math.nan
        return (now_ns - receipt_ns) * 1.0e-6

    def _runtime_snapshot(self) -> tuple[str, str, bool, XHitStatus | None, str]:
        try:
            session_id = parse_session_id(
                self._session_id_path.read_text(encoding="utf-8")
            )
        except (OSError, DecodeError) as exc:
            return "", "", False, None, f"NO VALID SESSION: {exc}"

        hdu_dir = self._runtime_root / session_id / "hdu"
        attempt = ""
        planner_alive = False
        try:
            attempt = parse_planner_attempt(
                (hdu_dir / "current_planner_attempt").read_text(encoding="utf-8")
            )
            pid = parse_positive_pid(
                (hdu_dir / attempt / "pid.txt").read_text(encoding="utf-8")
            )
            cmdline = (Path("/proc") / str(pid) / "cmdline").read_bytes()
            planner_alive = process_cmdline_matches(
                cmdline, self._planner_process_fragment
            )
        except (OSError, DecodeError):
            planner_alive = False

        x_hit_status: XHitStatus | None = None
        x_hit_text = "NO X_HIT STATUS"
        try:
            x_hit_status = parse_x_hit_status(
                (hdu_dir / "x_hit.status").read_text(encoding="utf-8")
            )
            x_hit_text = (
                f"request={x_hit_status.request_id} success={int(x_hit_status.success)} "
                f"{x_hit_status.message}"
            )
        except (OSError, DecodeError) as exc:
            x_hit_text = f"NO VALID X_HIT STATUS: {exc}"
        return session_id, attempt, planner_alive, x_hit_status, x_hit_text

    def _publish_snapshot(self) -> None:
        now_monotonic_ns = time.monotonic_ns()
        now_ros_ns = self.get_clock().now().nanoseconds
        self._topic_publishers["observer_alive"].publish(self._bool_message(True))

        session_id, attempt, planner_alive, x_hit, x_hit_text = (
            self._runtime_snapshot()
        )
        self._topic_publishers["session_active"].publish(
            self._bool_message(bool(session_id))
        )
        self._topic_publishers["session_id"].publish(self._string_message(session_id))
        self._topic_publishers["session_text"].publish(
            self._string_message(
                f"session={session_id or 'NONE'} planner_attempt={attempt or 'NONE'}"
            )
        )
        self._topic_publishers["planner_alive"].publish(
            self._bool_message(planner_alive)
        )
        self._topic_publishers["planner_attempt"].publish(self._string_message(attempt))

        base_seen = self._base_receipt_ns > 0
        base_receipt_age_ms = self._receipt_age_ms(
            now_monotonic_ns, self._base_receipt_ns
        )
        base_valid = self._base_packet is not None and self._base_packet.valid
        base_fresh = (
            base_valid
            and math.isfinite(base_receipt_age_ms)
            and base_receipt_age_ms <= self._base_stale_after_s * 1000.0
        )
        self._topic_publishers["base_seen"].publish(self._bool_message(base_seen))
        self._topic_publishers["base_valid"].publish(self._bool_message(base_valid))
        self._topic_publishers["base_fresh"].publish(self._bool_message(base_fresh))
        self._topic_publishers["base_receipt_age_ms"].publish(
            self._float_message(base_receipt_age_ms)
        )
        base_source_age_ms = math.nan
        base_xyz = (math.nan, math.nan, math.nan)
        base_summary = self._base_error
        if self._base_packet is not None:
            if self._base_packet.source_time_ns > 0:
                base_source_age_ms = (
                    now_ros_ns - self._base_packet.source_time_ns
                ) * 1.0e-6
            if self._base_packet.valid:
                base_xyz = self._base_packet.position_xyz
            base_summary = (
                f"valid={int(self._base_packet.valid)} fresh={int(base_fresh)} "
                f"seq={self._base_packet.sequence} "
                f"receipt_age_ms={base_receipt_age_ms:.3f} "
                f"source_age_ms={base_source_age_ms:.3f} "
                f"reason={self._base_packet.reason}"
            )
        self._topic_publishers["base_source_age_ms"].publish(
            self._float_message(base_source_age_ms)
        )
        self._topic_publishers["base_x"].publish(self._float_message(base_xyz[0]))
        self._topic_publishers["base_y"].publish(self._float_message(base_xyz[1]))
        self._topic_publishers["base_z"].publish(self._float_message(base_xyz[2]))
        self._topic_publishers["base_summary"].publish(
            self._string_message(base_summary)
        )

        one_second_ago = now_monotonic_ns - 1_000_000_000
        while (
            len(self._ball_receipts_ns) > 1
            and self._ball_receipts_ns[0] < one_second_ago
        ):
            self._ball_receipts_ns.popleft()
        ball_last_ns = self._ball_receipts_ns[-1] if self._ball_receipts_ns else 0
        ball_age_ms = self._receipt_age_ms(now_monotonic_ns, ball_last_ns)
        ball_live = (
            math.isfinite(ball_age_ms)
            and ball_age_ms <= self._ball_stale_after_s * 1000.0
        )
        ball_rate_hz = math.nan
        if len(self._ball_receipts_ns) >= 2:
            span_s = (
                self._ball_receipts_ns[-1] - self._ball_receipts_ns[0]
            ) * 1.0e-9
            if span_s > 0.0:
                ball_rate_hz = (len(self._ball_receipts_ns) - 1) / span_s
        self._topic_publishers["ball_live"].publish(self._bool_message(ball_live))
        self._topic_publishers["ball_receipt_age_ms"].publish(
            self._float_message(ball_age_ms)
        )
        self._topic_publishers["ball_rate_hz"].publish(
            self._float_message(ball_rate_hz)
        )

        command_seen = self._command_receipt_ns > 0
        command_age_ms = self._receipt_age_ms(
            now_monotonic_ns, self._command_receipt_ns
        )
        command_valid = (
            self._command_packet is not None and self._command_packet.valid
        )
        command_countdown_s = math.nan
        command_sequence = 0
        command_flight = 0
        command_revision = 0
        command_summary = self._command_error
        if self._command_packet is not None:
            command_sequence = self._command_packet.command_sequence
            command_flight = self._command_packet.flight_id
            command_revision = self._command_packet.revision_id
            if self._command_packet.strike_deadline_wall_s > 0.0:
                command_countdown_s = (
                    self._command_packet.strike_deadline_wall_s
                    - now_ros_ns * 1.0e-9
                )
            command_summary = (
                f"valid={int(self._command_packet.valid)} "
                f"seq={command_sequence} flight={command_flight} "
                f"revision={command_revision} "
                f"receipt_age_ms={command_age_ms:.3f} "
                f"hdu_wall_countdown_s={command_countdown_s:.4f} "
                f"reason={self._command_packet.reason}"
            )
        self._topic_publishers["command_seen"].publish(
            self._bool_message(command_seen)
        )
        self._topic_publishers["command_valid"].publish(
            self._bool_message(command_valid)
        )
        self._topic_publishers["command_receipt_age_ms"].publish(
            self._float_message(command_age_ms)
        )
        self._topic_publishers["command_countdown_s"].publish(
            self._float_message(command_countdown_s)
        )
        self._topic_publishers["command_sequence"].publish(
            self._uint64_message(command_sequence)
        )
        self._topic_publishers["command_flight"].publish(
            self._uint64_message(command_flight)
        )
        self._topic_publishers["command_revision"].publish(
            self._uint64_message(command_revision)
        )
        self._topic_publishers["command_summary"].publish(
            self._string_message(command_summary)
        )

        self._topic_publishers["x_hit_available"].publish(
            self._bool_message(x_hit is not None)
        )
        self._topic_publishers["x_hit_success"].publish(
            self._bool_message(x_hit.success if x_hit is not None else False)
        )
        self._topic_publishers["x_hit_value"].publish(
            self._float_message(
                x_hit.x_hit_m
                if x_hit is not None and x_hit.x_hit_m is not None
                else math.nan
            )
        )
        self._topic_publishers["x_hit_status"].publish(
            self._string_message(x_hit_text)
        )

        runner_age_ms = self._receipt_age_ms(
            now_monotonic_ns, self._runner_state_receipt_ns
        )
        runner_alive = (
            self._runner_state is not None
            and math.isfinite(runner_age_ms)
            and runner_age_ms <= self._runner_stale_after_s * 1000.0
        )
        runner = self._runner_state
        session_matches = bool(
            runner is not None
            and session_id
            and runner.session_fingerprint
            == runner_session_fingerprint(session_id)
        )
        runner_session_id = session_id if session_matches else ""
        boot_id = runner.boot_id if runner is not None else 0
        state_sequence = runner.state_sequence if runner is not None else 0
        run_mode = runner.run_mode if runner is not None else "UNKNOWN"
        local_role = runner.local_role if runner is not None else "UNASSIGNED"
        expected_role = (
            opponent_expected_role(local_role) if runner_alive else "UNKNOWN"
        )
        summary = self._runner_state_error
        if runner is not None:
            summary = (
                f"alive={int(runner_alive)} receipt_age_ms={runner_age_ms:.3f} "
                f"boot={runner.boot_id} seq={runner.state_sequence} "
                f"session_matches={int(session_matches)} mode={runner.run_mode} "
                f"publishing={int(runner.command_publishing)} "
                f"fault={int(runner.command_fault_latched)} "
                f"local_role={runner.local_role} source=RUNNER_CONFIRMED "
                f"role_epoch={runner.role_epoch} "
                f"serve={runner.serve_capability}/{runner.serve_state} "
                f"last_action={runner.last_action} "
                f"result={runner.last_action_result} "
                f"reason={runner.last_action_reason}"
            )

        self._topic_publishers["runner_alive"].publish(
            self._bool_message(runner_alive)
        )
        self._topic_publishers["mdu_active"].publish(
            self._bool_message(runner_alive)
        )
        self._topic_publishers["mdu_text"].publish(
            self._string_message(
                "MDU Runner state fresh"
                if runner_alive
                else "MDU Runner state absent or stale"
            )
        )
        self._topic_publishers["runner_boot_id"].publish(
            self._uint64_message(boot_id)
        )
        self._topic_publishers["runner_session_id"].publish(
            self._string_message(runner_session_id)
        )
        self._topic_publishers["runner_session_matches"].publish(
            self._bool_message(session_matches)
        )
        self._topic_publishers["runner_state_sequence"].publish(
            self._uint64_message(state_sequence)
        )
        self._topic_publishers["runner_mode"].publish(
            self._string_message(run_mode)
        )
        self._topic_publishers["runner_command_publishing"].publish(
            self._bool_message(
                runner.command_publishing if runner is not None else False
            )
        )
        self._topic_publishers["runner_policy_native"].publish(
            self._bool_message(runner.policy_native if runner is not None else False)
        )
        self._topic_publishers["runner_command_fault"].publish(
            self._bool_message(
                runner.command_fault_latched if runner is not None else False
            )
        )
        self._topic_publishers["runner_local_role"].publish(
            self._string_message(local_role)
        )
        self._topic_publishers["runner_role_epoch"].publish(
            self._uint64_message(runner.role_epoch if runner is not None else 0)
        )
        self._topic_publishers["runner_role_change_allowed"].publish(
            self._bool_message(
                runner.role_change_allowed if runner is not None else False
            )
        )
        self._topic_publishers["runner_role_last_result"].publish(
            self._string_message(
                runner.role_last_result if runner is not None else "NONE"
            )
        )
        self._topic_publishers["runner_serve_capability"].publish(
            self._string_message(
                runner.serve_capability if runner is not None else "UNAVAILABLE"
            )
        )
        self._topic_publishers["runner_serve_state"].publish(
            self._string_message(
                runner.serve_state if runner is not None else "UNAVAILABLE"
            )
        )
        self._topic_publishers["runner_standing"].publish(
            self._bool_message(runner_alive and run_mode == "PD_STAND")
        )
        self._topic_publishers["runner_ready"].publish(
            self._bool_message(runner_alive and run_mode == "MOTION")
        )
        self._topic_publishers["runner_ready_to_serve"].publish(
            self._bool_message(
                runner_alive
                and run_mode == "SERVE"
                and runner is not None
                and runner.serve_state == "AWAIT_BALL_ON_PALM"
            )
        )
        self._topic_publishers["runner_serving"].publish(
            self._bool_message(
                runner_alive
                and run_mode == "SERVE"
                and runner is not None
                and runner.serve_state == "PLAYING"
            )
        )
        self._topic_publishers["runner_last_action_id"].publish(
            self._uint64_message(runner.last_action_id if runner is not None else 0)
        )
        self._topic_publishers["runner_last_action"].publish(
            self._string_message(runner.last_action if runner is not None else "NONE")
        )
        self._topic_publishers["runner_last_action_result"].publish(
            self._string_message(
                runner.last_action_result if runner is not None else "NONE"
            )
        )
        self._topic_publishers["runner_last_action_reason"].publish(
            self._string_message(
                runner.last_action_reason if runner is not None else "NONE"
            )
        )
        self._topic_publishers["runner_summary"].publish(
            self._string_message(summary)
        )
        self._topic_publishers["opponent_expected_role"].publish(
            self._string_message(expected_role)
        )
        self._topic_publishers["opponent_role_source"].publish(
            self._string_message("INFERRED_FROM_LOCAL_ROLE")
        )
        # We do not connect to the opponent Runner.  This is always false.
        self._topic_publishers["opponent_role_confirmed"].publish(
            self._bool_message(False)
        )
        self._topic_publishers["opponent_summary"].publish(
            self._string_message(
                f"expected_role={expected_role} "
                "source=INFERRED_FROM_LOCAL_ROLE confirmed=0"
            )
        )


def main() -> None:
    rclpy.init()
    node = HopeV17Observer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
