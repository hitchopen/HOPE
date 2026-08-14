#!/usr/bin/env python3
"""Launch side-neutral physical balls into MuJoCo for Gate3."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import rclpy
from mujoco_sim_msgs.msg import Gate3BallCommand, Gate3BallState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pp_gate3_core import TABLE_HEIGHT_M, parse_serves_list


class Gate3BallLauncher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("gate3_ball_launcher")
        self._args = args
        self._serves = parse_serves_list(args.serves)
        if args.max_serves <= 0:
            raise ValueError("--max-serves must be positive")
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
        while rclpy.ok() and time.monotonic() < deadline:
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
        self._spin_until(lambda: False, self._args.motion_idle_s)

        for index in range(self._args.max_serves):
            spec = self._serves[index % len(self._serves)]
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
            self._publish_until_ack(command, active=True)
            self._spin_until(lambda: False, self._args.flight_s)

            park = Gate3BallCommand()
            park.header.stamp = self.get_clock().now().to_msg()
            park.header.frame_id = "world"
            park.shot_id = shot_id
            park.active = False
            self._publish_until_ack(park, active=False)
            if index + 1 < self._args.max_serves:
                self._spin_until(lambda: False, self._args.pause_s)
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
        "--motion-idle-s",
        type=float,
        default=float(os.environ.get("PP_MOTION_IDLE_S", "20.0")),
    )
    parser.add_argument("--wait-log", default="/tmp/pp_runner.log")
    parser.add_argument(
        "--wait-marker",
        default=os.environ.get("PP_GATE3_MOTION_MARKER", ""),
        help=(
            "override the Runner MOTION log marker; by default both the "
            "current authoritative transition and the legacy marker are accepted"
        ),
    )
    parser.add_argument("--wait-timeout-s", type=float, default=120.0)
    parser.add_argument("--discovery-timeout-s", type=float, default=10.0)
    parser.add_argument("--table-height-m", type=float, default=TABLE_HEIGHT_M)
    args = parser.parse_args()
    if args.flight_s <= 0.0 or args.pause_s < 0.0 or args.motion_idle_s < 0.0:
        parser.error("flight/pause/motion-idle durations must be non-negative")
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
