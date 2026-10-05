#!/usr/bin/env python3
"""Non-actuating fake Runner endpoint for isolated-domain transport tests."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


MAX_EXACT_FLOAT_INTEGER = 1 << 52
ACTION_CODES = frozenset({1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12})


def session_fingerprint(session_id: str) -> int:
    value = 1469598103934665603
    for character in session_id.encode("utf-8"):
        value ^= character
        value = (value * 1099511628211) & ((1 << 64) - 1)
    value &= MAX_EXACT_FLOAT_INTEGER - 1
    return value or 1


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class FakeRunnerEndpoint(Node):
    def __init__(self, state_topic: str, control_topic: str, session_id: str) -> None:
        super().__init__("hope_runner_transport_fake_endpoint")
        self._publisher = self.create_publisher(
            Float64MultiArray, state_topic, reliable_qos()
        )
        self._subscription = self.create_subscription(
            Float64MultiArray, control_topic, self._on_control, reliable_qos()
        )
        self._sequence = 1
        self._last_action_id = 0
        self._last_action = 0
        self._fingerprint = session_fingerprint(session_id)
        self.create_timer(0.1, self._publish_state)
        self.get_logger().info(
            f"FAKE RUNNER READY state={state_topic} control={control_topic} "
            f"session_fingerprint={self._fingerprint}; no robot API is used"
        )

    def _on_control(self, message: Float64MultiArray) -> None:
        values = tuple(float(value) for value in message.data)
        if len(values) != 4 or values[0] != 2.0 or values[3] != 0.0:
            self.get_logger().error(f"FAKE RUNNER REJECT malformed={values}")
            return
        request_id = int(values[1])
        action = int(values[2])
        if (
            float(request_id) != values[1]
            or request_id < 1
            or request_id > MAX_EXACT_FLOAT_INTEGER
            or float(action) != values[2]
            or action not in ACTION_CODES
        ):
            self.get_logger().error(f"FAKE RUNNER REJECT invalid={values}")
            return
        self._last_action_id = request_id
        self._last_action = action
        self._sequence += 1
        self.get_logger().info(
            f"FAKE RUNNER ACK request={request_id} action={action}"
        )
        self._publish_state()

    def _publish_state(self) -> None:
        message = Float64MultiArray()
        message.data = [
            2.0,  # schema
            987654.0,  # boot id
            float(self._sequence),
            0.0,  # PASSIVE
            0.0,  # command publishing
            1.0,  # policy native
            0.0,  # command fault
            0.0,  # role unassigned
            0.0,  # role epoch
            1.0,  # role change allowed
            0.0,  # role result NONE
            0.0,  # role reason NONE
            1.0,  # serve capability
            0.0,  # serve IDLE
            0.0,  # gripper UNKNOWN
            0.0,  # cleanup not required
            float(self._last_action_id),
            float(self._last_action),
            1.0 if self._last_action_id else 0.0,  # APPLIED or NONE
            1.0 if self._last_action_id else 0.0,  # ROLE_CHANGED or NONE
            float(self._fingerprint),
        ]
        self._publisher.publish(message)
        self._sequence += 1


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--control-topic", required=True)
    parser.add_argument("--session-id", required=True)
    args = parser.parse_args()
    for topic in (args.state_topic, args.control_topic):
        if not topic.startswith("/hope/diagnostic/"):
            parser.error("fake endpoints are restricted to /hope/diagnostic/* topics")

    rclpy.init()
    node = FakeRunnerEndpoint(args.state_topic, args.control_topic, args.session_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
