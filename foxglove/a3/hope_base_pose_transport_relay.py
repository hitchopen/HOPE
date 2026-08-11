#!/usr/bin/env python3
"""Route the calibrated Laptop base packet through the dual-homed HDU.

Fast DDS discovery is deliberately unicast: the Laptop sees the HDU Wi-Fi
address and the MDU sees the HDU internal address, but they are not one DDS
participant mesh.  This node changes only the ROS topic name; it preserves the
schema-2 Float64MultiArray payload byte-for-byte so Runner/Planner semantics
remain those of build_1.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


class HopeBasePoseTransportRelay(Node):
    def __init__(self) -> None:
        super().__init__("hope_base_pose_transport_relay", start_parameter_services=False)
        self.declare_parameter("input_topic", "/a3/base_pose_laptop_flat")
        self.declare_parameter("output_topic", "/a3/base_pose_flat")

        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        if not input_topic or not output_topic or input_topic == output_topic:
            raise ValueError("base transport relay requires two distinct non-empty topics")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._publisher = self.create_publisher(Float64MultiArray, output_topic, qos)
        self._subscription = self.create_subscription(
            Float64MultiArray, input_topic, self._relay, qos
        )
        self._received = 0
        self._published = 0
        self._previous_received = 0
        self._started = time.monotonic()
        self._last_receipt = 0.0
        self._timer = self.create_timer(1.0, self._report)
        self.get_logger().info(
            f"base transport relay ready: {input_topic} -> {output_topic}; "
            "schema payload is unchanged"
        )

    def _relay(self, message: Float64MultiArray) -> None:
        self._received += 1
        self._last_receipt = time.monotonic()
        self._publisher.publish(message)
        self._published += 1

    def _report(self) -> None:
        now = time.monotonic()
        rate = self._received - self._previous_received
        self._previous_received = self._received
        age = now - self._last_receipt if self._last_receipt > 0.0 else now - self._started
        if age > 2.0:
            self.get_logger().warning(
                f"no Laptop base packet for {age:.1f}s; received={self._received} "
                f"published={self._published}"
            )
        else:
            self.get_logger().info(
                f"BASE TRANSPORT input={rate}/s received={self._received} "
                f"published={self._published} newest_age={age:.3f}s"
            )


def main() -> None:
    rclpy.init()
    node = HopeBasePoseTransportRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
