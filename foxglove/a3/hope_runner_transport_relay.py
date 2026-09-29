#!/usr/bin/env python3
"""Bridge native Runner state/control through a late-started HDU participant.

The vendor AimRT ROS 2 participant on the MDU does not reliably rematch an HDU
reader which existed before Runner startup.  Lifecycle therefore starts this
relay only after Runner has reported local PASSIVE readiness.  HDU services use
the distinct ``*_hdu_flat`` topics, so this node cannot form a DDS echo loop.

The relay preserves Float64MultiArray messages unchanged.  It never constructs,
validates, or invents a Runner action; command authority remains exclusively in
``hope_command_proxy.py`` and the native Runner.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


REMOTE_STATE_TOPIC = "/hope/runner/state_flat"
LOCAL_STATE_TOPIC = "/hope/runner/state_hdu_flat"
LOCAL_CONTROL_TOPIC = "/hope/runner/control_request_hdu_flat"
REMOTE_CONTROL_TOPIC = "/hope/runner/control_request_flat"


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class HopeRunnerTransportRelay(Node):
    def __init__(self) -> None:
        super().__init__("hope_runner_transport_relay", start_parameter_services=False)
        self.declare_parameter("remote_state_topic", REMOTE_STATE_TOPIC)
        self.declare_parameter("local_state_topic", LOCAL_STATE_TOPIC)
        self.declare_parameter("local_control_topic", LOCAL_CONTROL_TOPIC)
        self.declare_parameter("remote_control_topic", REMOTE_CONTROL_TOPIC)

        self._remote_state_topic = self._topic("remote_state_topic")
        self._local_state_topic = self._topic("local_state_topic")
        self._local_control_topic = self._topic("local_control_topic")
        self._remote_control_topic = self._topic("remote_control_topic")
        topics = {
            self._remote_state_topic,
            self._local_state_topic,
            self._local_control_topic,
            self._remote_control_topic,
        }
        if len(topics) != 4:
            raise ValueError("Runner transport requires four distinct topics")

        qos = _reliable_qos()
        self._local_state_publisher = self.create_publisher(
            Float64MultiArray, self._local_state_topic, qos
        )
        self._remote_control_publisher = self.create_publisher(
            Float64MultiArray, self._remote_control_topic, qos
        )
        self._remote_state_subscription = self.create_subscription(
            Float64MultiArray,
            self._remote_state_topic,
            self._relay_state,
            qos,
        )
        self._local_control_subscription = self.create_subscription(
            Float64MultiArray,
            self._local_control_topic,
            self._relay_control,
            qos,
        )
        stream_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self._teleop_input_publisher = self.create_publisher(
            Float64MultiArray, '/hope/runner/teleop_input_flat', stream_qos)
        self._teleop_input_subscription = self.create_subscription(
            Float64MultiArray, '/hope/runner/teleop_input_hdu_flat',
            self._teleop_input_publisher.publish, stream_qos)
        self._teleop_state_publisher = self.create_publisher(
            Float64MultiArray, '/hope/runner/teleop_state_hdu_flat', stream_qos)
        self._teleop_state_subscription = self.create_subscription(
            Float64MultiArray, '/hope/runner/teleop_state_flat',
            self._teleop_state_publisher.publish, stream_qos)
        # Preserve original source timestamp/session/sequence. No timer replay
        # and no relay receipt time that could make an old input look fresh.

        self._state_received = 0
        self._state_published = 0
        self._control_received = 0
        self._control_published = 0
        self._previous_state_received = 0
        self._last_state_receipt = 0.0
        self._started = time.monotonic()
        self._healthy_reported = False
        self.create_timer(0.5, self._report)
        self.get_logger().info(
            "Runner transport relay ready: "
            f"{self._remote_state_topic} -> {self._local_state_topic}; "
            f"{self._local_control_topic} -> {self._remote_control_topic}; "
            "payloads are unchanged and no command is generated"
        )

    def _topic(self, parameter: str) -> str:
        value = str(self.get_parameter(parameter).value).strip()
        if not value or not value.startswith("/"):
            raise ValueError(f"{parameter} must be a non-empty absolute ROS topic")
        return value

    def _relay_state(self, message: Float64MultiArray) -> None:
        self._state_received += 1
        self._last_state_receipt = time.monotonic()
        self._local_state_publisher.publish(message)
        self._state_published += 1

    def _relay_control(self, message: Float64MultiArray) -> None:
        self._control_received += 1
        self._remote_control_publisher.publish(message)
        self._control_published += 1

    def _endpoint_counts(self) -> tuple[int, int, int, int]:
        return (
            self.count_publishers(self._remote_state_topic),
            self.count_subscribers(self._local_state_topic),
            self.count_publishers(self._local_control_topic),
            self.count_subscribers(self._remote_control_topic),
        )

    def _report(self) -> None:
        now = time.monotonic()
        state_rate = self._state_received - self._previous_state_received
        self._previous_state_received = self._state_received
        state_age = (
            now - self._last_state_receipt
            if self._last_state_receipt > 0.0
            else now - self._started
        )
        (
            remote_state_publishers,
            local_state_subscribers,
            local_control_publishers,
            remote_control_subscribers,
        ) = self._endpoint_counts()
        healthy = (
            self._state_received > 0
            and state_age <= 1.0
            and remote_state_publishers >= 1
            and local_state_subscribers >= 2
            and local_control_publishers >= 1
            and remote_control_subscribers >= 1
        )
        summary = (
            f"state_rate={state_rate}/0.5s state_received={self._state_received} "
            f"state_published={self._state_published} state_age={state_age:.3f}s "
            f"remote_state_publishers={remote_state_publishers} "
            f"local_state_subscribers={local_state_subscribers} "
            f"local_control_publishers={local_control_publishers} "
            f"remote_control_subscribers={remote_control_subscribers} "
            f"control_received={self._control_received} "
            f"control_published={self._control_published}"
        )
        if healthy:
            if not self._healthy_reported:
                self.get_logger().info(f"RUNNER TRANSPORT HEALTHY {summary}")
                self._healthy_reported = True
            else:
                self.get_logger().info(f"RUNNER TRANSPORT {summary}")
        else:
            self._healthy_reported = False
            self.get_logger().warning(f"RUNNER TRANSPORT WAITING {summary}")


def main() -> None:
    rclpy.init()
    node = HopeRunnerTransportRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
