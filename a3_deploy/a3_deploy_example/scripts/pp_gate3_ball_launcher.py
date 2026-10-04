#!/usr/bin/env python3
"""Launch side-neutral physical balls into MuJoCo for Gate3."""

from __future__ import annotations

import argparse
import ast
import json
import math
import os
import time
from pathlib import Path

import rclpy
from mujoco_sim_msgs.msg import Gate3BallCommand, Gate3BallState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pp_gate3_core import TABLE_HEIGHT_M, parse_serves_list


def _duration_schedule(
    raw: str, *, count: int, fallback: float, label: str, allow_zero: bool
) -> tuple[float, ...]:
    if not raw.strip():
        values = [float(fallback)] * count
    else:
        parsed = ast.literal_eval(raw)
        if not isinstance(parsed, (list, tuple)) or len(parsed) != count:
            raise ValueError(f"{label} must contain exactly {count} values")
        values = [float(value) for value in parsed]
    if any(
        not math.isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0)
        for value in values
    ):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{label} values must be finite and {qualifier}")
    return tuple(values)


class Gate3BallLauncher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("gate3_ball_launcher")
        self._args = args
        self._stop_file = Path("/tmp/pp_gate3_stop_launches")
        self._serves = parse_serves_list(args.serves)
        if args.max_serves <= 0:
            raise ValueError("--max-serves must be positive")
        self._flight_schedule = _duration_schedule(
            args.flight_s_list,
            count=args.max_serves,
            fallback=args.flight_s,
            label="--flight-s-list",
            allow_zero=False,
        )
        self._pause_schedule = _duration_schedule(
            args.pause_s_list,
            count=args.max_serves,
            fallback=args.pause_s,
            label="--pause-s-list",
            allow_zero=True,
        )
        self._pub = self.create_publisher(
            Gate3BallCommand, "/sim/gate3/ball_command", 10
        )
        self.create_subscription(
            Gate3BallState, "/sim/gate3/ball_state", self._state_cb, 10
        )
        self._last_state: Gate3BallState | None = None

    def _state_cb(self, msg: Gate3BallState) -> None:
        self._last_state = msg

    def _spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def _publish_until_ack(
        self, msg: Gate3BallCommand, *, active: bool, timeout_s: float = 2.0
    ) -> None:
        deadline = time.monotonic() + timeout_s
        # Release can wait for a return slot; an airborne flight is never
        # overwritten merely because the normal launch-ack timeout elapsed.
        while rclpy.ok() and (not active or time.monotonic() < deadline):
            self._pub.publish(msg)
            if self._spin_until(
                lambda: self._last_state is not None
                and int(self._last_state.shot_id) == int(msg.shot_id)
                and bool(self._last_state.active) is active,
                0.12,
            ):
                return
        raise RuntimeError(
            f"MuJoCo did not acknowledge shot_id={msg.shot_id} active={active}"
        )

    def run(self) -> None:
        if not self._spin_until(
            lambda: self._pub.get_subscription_count() > 0, self._args.discovery_timeout_s
        ):
            raise RuntimeError("no MuJoCo subscriber on /sim/gate3/ball_command")
        if not self._spin_until(
            lambda: self.count_publishers("/sim/gate3/physical_ball_state") > 0,
            self._args.discovery_timeout_s,
        ):
            raise RuntimeError("simulator lacks continuous physical-flight telemetry; rebuild the a3_deploy simulator")

        if self._args.wait_log:
            markers = (
                [self._args.wait_marker]
                if self._args.wait_marker
                else [
                    "action=ENTER_MOTION result=APPLIED",
                    "MOTION (PUBLISHING)",
                ]
            )
            encoded_markers = [marker.encode() for marker in markers]
            deadline = time.monotonic() + self._args.wait_timeout_s
            while time.monotonic() < deadline:
                try:
                    payload = Path(self._args.wait_log).read_bytes()
                    if any(marker in payload for marker in encoded_markers):
                        break
                except OSError:
                    pass
                self._spin_until(lambda: False, 0.10)
            else:
                raise RuntimeError(
                    f"runner MOTION markers {markers!r} not seen in "
                    f"{self._args.wait_log}"
                )
        self.get_logger().info(
            f"runner entered MOTION; preserving {self._args.motion_idle_s:.2f}s "
            "policy-native idle window"
        )
        self._spin_until(self._stop_file.exists, self._args.motion_idle_s)

        receipt_rows: list[dict[str, object]] = []
        for index in range(self._args.max_serves):
            if self._stop_file.exists():
                break
            spec = self._serves[index % len(self._serves)]
            flight_s = self._flight_schedule[index]
            pause_s = self._pause_schedule[index]
            shot_id = index + 1
            world = spec.world_position(self._args.table_height_m)
            command = Gate3BallCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = "world"
            command.shot_id = shot_id
            command.active = True
            command.position.x, command.position.y, command.position.z = world
            (
                command.linear_velocity.x,
                command.linear_velocity.y,
                command.linear_velocity.z,
            ) = spec.velocity
            # Keep this exact prefix parseable by pp_rally_conductor.py.
            self.get_logger().info(
                "serve %d: shot_id=%d p_table=[%.4f,%.4f,%.4f] "
                "p_world=[%.4f,%.4f,%.4f] v=[%.4f,%.4f,%.4f]"
                % (
                    shot_id,
                    shot_id,
                    *spec.position,
                    *world,
                    *spec.velocity,
                )
            )
            launch_command_wall_time_ns = time.time_ns()
            self._publish_until_ack(command, active=True)
            launch_ack_wall_time_ns = time.time_ns()
            self.get_logger().info(
                "serve %d schedule: flight_s=%.3f pause_s=%.3f"
                % (shot_id, flight_s, pause_s)
            )
            self._spin_until(self._stop_file.exists, flight_s)

            park = Gate3BallCommand()
            park.header.stamp = self.get_clock().now().to_msg()
            park.header.frame_id = "world"
            park.shot_id = shot_id
            park.active = False
            self._publish_until_ack(park, active=False)
            park_ack_wall_time_ns = time.time_ns()
            receipt_rows.append(
                {
                    "flight_id": shot_id,
                    "shot_id": shot_id,
                    "scheduled_flight_s": flight_s,
                    "scheduled_pause_s": pause_s,
                    "launch_command_wall_time_ns": launch_command_wall_time_ns,
                    "launch_ack_wall_time_ns": launch_ack_wall_time_ns,
                    "park_ack_wall_time_ns": park_ack_wall_time_ns,
                    "release_ack_wall_time_ns": park_ack_wall_time_ns,
                }
            )
            if index + 1 < self._args.max_serves:
                self._spin_until(self._stop_file.exists, pause_s)

        cadence_errors_s: list[float] = []
        for index, row in enumerate(receipt_rows):
            if index == 0:
                row["scheduled_launch_gap_s"] = None
                row["actual_launch_gap_s"] = None
                row["launch_gap_error_s"] = None
                continue
            previous = receipt_rows[index - 1]
            scheduled_gap_s = float(previous["scheduled_flight_s"]) + float(
                previous["scheduled_pause_s"]
            )
            actual_gap_s = (
                int(row["launch_ack_wall_time_ns"])
                - int(previous["launch_ack_wall_time_ns"])
            ) * 1.0e-9
            error_s = actual_gap_s - scheduled_gap_s
            row["scheduled_launch_gap_s"] = round(scheduled_gap_s, 6)
            row["actual_launch_gap_s"] = round(actual_gap_s, 6)
            row["launch_gap_error_s"] = round(error_s, 6)
            cadence_errors_s.append(abs(error_s))

        receipt = {
            "schema_version": 1,
            "receipt_contract": "gate3_launch_cadence_wall_clock_v1",
            "scenario_id": self._args.scenario_id,
            "expected_flights": self._args.max_serves,
            "launches_finished": True,
            "ball_lifecycle_contract": "continuous_flight_to_natural_rest_v2",
            "flight_ids": [int(row["flight_id"]) for row in receipt_rows],
            "cadence_tolerance_s": self._args.cadence_tolerance_s,
            "maximum_abs_launch_gap_error_s": max(cadence_errors_s, default=0.0),
            "cadence_pass": bool(
                len(receipt_rows) == self._args.max_serves
                and [int(row["flight_id"]) for row in receipt_rows]
                == list(range(1, self._args.max_serves + 1))
                and all(
                    error <= self._args.cadence_tolerance_s
                    for error in cadence_errors_s
                )
            ),
            "rows": receipt_rows,
        }
        receipt_path = Path(self._args.launch_receipt_json)
        receipt_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = receipt_path.with_name(receipt_path.name + ".tmp")
        temporary_path.write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(receipt_path)
        self.get_logger().info(
            "launch cadence receipt: %s max_abs_gap_error_s=%.6f pass=%s"
            % (
                receipt_path,
                receipt["maximum_abs_launch_gap_error_s"],
                receipt["cadence_pass"],
            )
        )
        self.get_logger().info(
            f"completed configured max_serves={self._args.max_serves}; launcher idle"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serves", default=os.environ.get("PP_SERVES_LIST", ""))
    parser.add_argument(
        "--max-serves", type=int, default=int(os.environ.get("PP_SERVES", "12"))
    )
    parser.add_argument(
        "--flight-s", type=float, default=float(os.environ.get("PP_FLIGHT_S", "2.5"))
    )
    parser.add_argument(
        "--pause-s", type=float, default=float(os.environ.get("PP_PAUSE_S", "4.0"))
    )
    parser.add_argument(
        "--flight-s-list", default=os.environ.get("PP_FLIGHT_S_LIST", "")
    )
    parser.add_argument(
        "--pause-s-list", default=os.environ.get("PP_PAUSE_S_LIST", "")
    )
    parser.add_argument(
        "--motion-idle-s",
        type=float,
        default=float(os.environ.get("PP_MOTION_IDLE_S", "20.0")),
    )
    parser.add_argument("--wait-log", default="/tmp/pp_runner.log")
    parser.add_argument(
        "--wait-marker",
        default="action=ENTER_MOTION result=APPLIED reason=MODE_CHANGED mode=MOTION",
        help=(
            "runner log event that authoritatively confirms the MOTION transition; "
            "defaults to the Runner control contract introduced with Foxglove"
        ),
    )
    parser.add_argument("--wait-timeout-s", type=float, default=120.0)
    parser.add_argument("--discovery-timeout-s", type=float, default=10.0)
    parser.add_argument("--table-height-m", type=float, default=TABLE_HEIGHT_M)
    parser.add_argument(
        "--scenario-id",
        default=os.environ.get("PP_GATE3_SCENARIO", "unspecified"),
    )
    parser.add_argument(
        "--launch-receipt-json",
        default=os.environ.get(
            "PP_LAUNCH_RECEIPT_JSON", "/tmp/pp_gate3_launch_receipt.json"
        ),
    )
    parser.add_argument(
        "--cadence-tolerance-s",
        type=float,
        default=float(os.environ.get("PP_LAUNCH_CADENCE_TOLERANCE_S", "0.10")),
    )
    args = parser.parse_args()
    if args.flight_s <= 0.0 or args.pause_s < 0.0 or args.motion_idle_s < 0.0:
        parser.error("flight/pause/motion-idle durations must be non-negative")
    if not args.scenario_id.strip():
        parser.error("scenario-id must be non-empty")
    if not math.isfinite(args.cadence_tolerance_s) or args.cadence_tolerance_s < 0.0:
        parser.error("cadence-tolerance-s must be finite and non-negative")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Gate3BallLauncher(args)
    try:
        node.run()
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except Exception:
        if rclpy.ok():
            raise
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
