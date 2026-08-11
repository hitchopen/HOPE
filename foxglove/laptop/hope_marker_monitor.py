#!/usr/bin/env python3
"""Publish the laptop's actual live P1 marker count as standard ROS messages.

The source is the same ``RigidBodyMarkerArray`` used by the registration tool.
Only finite, non-occluded, point-cloud-solved live samples are counted.  Model
definition entries or predicted marker locations never inflate the UI count.
"""

from __future__ import annotations

import time

import rclpy
from motion_capture_tracking_interfaces.msg import RigidBodyMarkerArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String, UInt32

from hope_marker_monitor_core import (
    EXPECTED_P1_MARKERS,
    count_physical_markers,
    marker_count_text,
)


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class HopeMarkerMonitor(Node):
    def __init__(self) -> None:
        super().__init__("hope_marker_monitor", start_parameter_services=False)
        self.declare_parameter("source_topic", "/optitrack/rigid_body_markers")
        self.declare_parameter("asset_name", "P1")
        self.declare_parameter("expected_count", EXPECTED_P1_MARKERS)
        self.declare_parameter("stale_after_s", 0.25)
        self.declare_parameter("publish_period_s", 0.1)

        self._asset_name = str(self.get_parameter("asset_name").value)
        self._expected_count = int(self.get_parameter("expected_count").value)
        self._stale_after_s = float(self.get_parameter("stale_after_s").value)
        publish_period_s = float(self.get_parameter("publish_period_s").value)
        if self._expected_count <= 0:
            raise ValueError("expected_count must be positive")
        if self._stale_after_s <= 0.0 or publish_period_s <= 0.0:
            raise ValueError("stale_after_s and publish_period_s must be positive")

        self._count = 0
        self._raw_count = 0
        self._receipt_monotonic: float | None = None
        self._pub_count = self.create_publisher(
            UInt32, "/hope/mocap/p1_marker_count", 10
        )
        self._pub_fresh = self.create_publisher(
            Bool, "/hope/mocap/p1_marker_fresh", 10
        )
        self._pub_complete = self.create_publisher(
            Bool, "/hope/mocap/p1_markers_complete", 10
        )
        self._pub_text = self.create_publisher(
            String, "/hope/mocap/p1_marker_text", 10
        )
        self.create_subscription(
            RigidBodyMarkerArray,
            str(self.get_parameter("source_topic").value),
            self._on_markers,
            _sensor_qos(),
        )
        self.create_timer(publish_period_s, self._publish)

    def _on_markers(self, message: RigidBodyMarkerArray) -> None:
        if str(message.rigid_body_name) != self._asset_name:
            return
        self._count, self._raw_count = count_physical_markers(
            message.markers, expected_count=self._expected_count
        )
        self._receipt_monotonic = time.monotonic()

    def _publish(self) -> None:
        now = time.monotonic()
        fresh = bool(
            self._receipt_monotonic is not None
            and now - self._receipt_monotonic <= self._stale_after_s
        )
        visible_count = self._count if fresh else 0
        self._pub_count.publish(UInt32(data=visible_count))
        self._pub_fresh.publish(Bool(data=fresh))
        self._pub_complete.publish(
            Bool(data=fresh and self._raw_count == self._expected_count)
        )
        self._pub_text.publish(
            String(
                data=marker_count_text(
                    visible_count,
                    fresh=fresh,
                    expected_count=self._expected_count,
                    raw_count=self._raw_count,
                )
            )
        )


def main() -> None:
    rclpy.init()
    node = HopeMarkerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
