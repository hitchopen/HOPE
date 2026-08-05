#!/usr/bin/env python3
"""HOPE A3 monitoring publisher for Foxglove.

Read-only with respect to the robot: reads `chronyc tracking`, pings the
mocap host, and aggregates the vendor joint-state topics. Publishes:

  /hope/ntp/offset_ms           std_msgs/Float64  chrony System time offset
  /hope/ntp/skew_ppm            std_msgs/Float64
  /hope/ntp/root_dispersion_ms  std_msgs/Float64
  /hope/ntp/utc_qualified       std_msgs/Bool     Leap Normal + selected source
  /hope/ntp/gate_pass           std_msgs/Bool     qualified + offset/skew gates
  /hope/mocap/rtt_ms            std_msgs/Float64  ICMP RTT to the mocap host
  /hope/mocap/reachable         std_msgs/Bool
  /hope/pelvis/pose             geometry_msgs/PoseStamped  reference->pelvis TF
  /hope/pelvis/text             std_msgs/String   human-readable pose (or TF error)
  /hope/joints/fresh            std_msgs/Bool     all configured groups are fresh
  /hope/joints/text             std_msgs/String   group freshness details
  /joint_states                 sensor_msgs/JointState  bounded-rate merged joints
"""

from concurrent.futures import Future, ThreadPoolExecutor
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from hope_monitor_core import (
    MocapProbeResult,
    NtpProbeResult,
    probe_mocap,
    probe_ntp,
    stale_sources,
)

VENDOR_JOINT_TOPICS = [
    "/motion/control/leg_joint_state",
    "/motion/control/arm_joint_state",
    "/motion/control/hand_joint_state",
    "/motion/control/neck_joint_state",
    "/motion/control/waist_joint_state",
]

class HopeMonitor(Node):
    def __init__(self):
        super().__init__("hope_monitor")
        self.declare_parameter("mocap_host", "REPLACE_WITH_MOCAP_HOST")
        self.declare_parameter("period_s", 1.0)
        self.declare_parameter("pelvis_frame", "pelvis_link")
        self.declare_parameter("reference_frame", "odom")
        self.declare_parameter("ntp_max_offset_ms", 10.0)
        self.declare_parameter("ntp_max_skew_ppm", 5.0)
        self.declare_parameter("joint_publish_hz", 20.0)
        self.declare_parameter("joint_stale_after_s", 0.5)

        self.pub_offset = self.create_publisher(Float64, "/hope/ntp/offset_ms", 10)
        self.pub_skew = self.create_publisher(Float64, "/hope/ntp/skew_ppm", 10)
        self.pub_disp = self.create_publisher(
            Float64, "/hope/ntp/root_dispersion_ms", 10
        )
        self.pub_qualified = self.create_publisher(Bool, "/hope/ntp/utc_qualified", 10)
        self.pub_ntp_gate = self.create_publisher(Bool, "/hope/ntp/gate_pass", 10)
        self.pub_rtt = self.create_publisher(Float64, "/hope/mocap/rtt_ms", 10)
        self.pub_reach = self.create_publisher(Bool, "/hope/mocap/reachable", 10)
        self.pub_joints = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_joints_fresh = self.create_publisher(Bool, "/hope/joints/fresh", 10)
        self.pub_joints_text = self.create_publisher(String, "/hope/joints/text", 10)
        self.pub_pelvis = self.create_publisher(PoseStamped, "/hope/pelvis/pose", 10)
        self.pub_pelvis_text = self.create_publisher(String, "/hope/pelvis/text", 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._joint_groups = {}
        self._joint_received_monotonic = {}
        for topic in VENDOR_JOINT_TOPICS:
            self.create_subscription(
                JointState,
                topic,
                lambda msg, source=topic: self._on_joint_state(source, msg),
                10,
            )

        period = float(self.get_parameter("period_s").value)
        joint_publish_hz = float(self.get_parameter("joint_publish_hz").value)
        if period <= 0.0:
            raise ValueError("period_s must be positive")
        if joint_publish_hz <= 0.0:
            raise ValueError("joint_publish_hz must be positive")

        self._probe_pool = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="hope-monitor-probe"
        )
        self._ntp_future: Future[NtpProbeResult] | None = None
        self._mocap_future: Future[MocapProbeResult] | None = None
        self.create_timer(period, self._poll_ntp)
        self.create_timer(period, self._poll_mocap)
        self.create_timer(0.2, self._poll_pelvis)
        self.create_timer(1.0 / joint_publish_hz, self._publish_joint_states)

    # ---- joint aggregation -------------------------------------------------
    def _on_joint_state(self, source: str, msg: JointState):
        self._joint_groups[source] = msg
        self._joint_received_monotonic[source] = time.monotonic()

    def _publish_joint_states(self):
        now_monotonic = time.monotonic()
        stale_after_s = float(self.get_parameter("joint_stale_after_s").value)
        if stale_after_s <= 0.0:
            self.get_logger().error("joint_stale_after_s must be positive")
            return

        stale = stale_sources(
            self._joint_received_monotonic,
            VENDOR_JOINT_TOPICS,
            now_monotonic=now_monotonic,
            stale_after_s=stale_after_s,
        )
        fresh = not stale
        self.pub_joints_fresh.publish(Bool(data=fresh))
        if not fresh:
            short_names = ", ".join(topic.rsplit("/", 1)[-1] for topic in stale)
            self.pub_joints_text.publish(
                String(data=f"JOINT DATA NOT FRESH | missing/stale: {short_names}")
            )
            return

        joints = {}
        stamps = []
        for source in VENDOR_JOINT_TOPICS:
            msg = self._joint_groups[source]
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )
            if stamp_ns > 0:
                stamps.append(stamp_ns)
            for i, name in enumerate(msg.name):
                joints[name] = (
                    msg.position[i] if i < len(msg.position) else math.nan,
                    msg.velocity[i] if i < len(msg.velocity) else math.nan,
                    msg.effort[i] if i < len(msg.effort) else math.nan,
                )

        merged = JointState()
        if stamps:
            oldest_stamp_ns = min(stamps)
            merged.header.stamp.sec = oldest_stamp_ns // 1_000_000_000
            merged.header.stamp.nanosec = oldest_stamp_ns % 1_000_000_000
        else:
            merged.header.stamp = self.get_clock().now().to_msg()
        merged.name = list(joints.keys())
        merged.position = [value[0] for value in joints.values()]
        merged.velocity = [value[1] for value in joints.values()]
        merged.effort = [value[2] for value in joints.values()]
        self.pub_joints.publish(merged)
        self.pub_joints_text.publish(
            String(data=f"JOINT DATA FRESH | {len(joints)} joints at bounded publish rate")
        )

    # ---- chrony ------------------------------------------------------------
    def _poll_ntp(self):
        if self._ntp_future is None:
            self._ntp_future = self._submit_ntp_probe()
            return
        if not self._ntp_future.done():
            return

        result = self._ntp_future.result()
        self.pub_offset.publish(Float64(data=result.offset_ms))
        self.pub_skew.publish(Float64(data=result.skew_ppm))
        self.pub_disp.publish(Float64(data=result.root_dispersion_ms))
        self.pub_qualified.publish(Bool(data=result.utc_qualified))
        self.pub_ntp_gate.publish(Bool(data=result.gate_pass))
        if result.error:
            self.get_logger().warn(
                f"chrony probe unavailable: {result.error}", throttle_duration_sec=30
            )
        self._ntp_future = self._submit_ntp_probe()

    def _submit_ntp_probe(self):
        max_offset_ms = float(self.get_parameter("ntp_max_offset_ms").value)
        max_skew_ppm = float(self.get_parameter("ntp_max_skew_ppm").value)
        return self._probe_pool.submit(
            probe_ntp,
            max_offset_ms=max_offset_ms,
            max_skew_ppm=max_skew_ppm,
        )

    # ---- pelvis pose -------------------------------------------------------
    def _poll_pelvis(self):
        ref = str(self.get_parameter("reference_frame").value)
        pelvis = str(self.get_parameter("pelvis_frame").value)
        try:
            tf = self._tf_buffer.lookup_transform(ref, pelvis, Time())
        except Exception as exc:  # noqa: BLE001 - surface the error in the UI
            self.pub_pelvis_text.publish(
                String(data=f"TF lookup {ref} -> {pelvis} failed: {exc}")
            )
            return

        pose = PoseStamped()
        pose.header = tf.header
        t = tf.transform.translation
        q = tf.transform.rotation
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z
        pose.pose.orientation = q
        self.pub_pelvis.publish(pose)

        roll, pitch, yaw = _quat_to_rpy_deg(q.x, q.y, q.z, q.w)
        self.pub_pelvis_text.publish(
            String(
                data=(
                    f"{ref} -> {pelvis} | "
                    f"pos [m] x={t.x:+.3f} y={t.y:+.3f} z={t.z:+.3f} | "
                    f"quat x={q.x:+.4f} y={q.y:+.4f} z={q.z:+.4f} w={q.w:+.4f} | "
                    f"rpy [deg] r={roll:+.1f} p={pitch:+.1f} y={yaw:+.1f}"
                )
            )
        )

    # ---- mocap link --------------------------------------------------------
    def _poll_mocap(self):
        host = str(self.get_parameter("mocap_host").value)
        if self._mocap_future is None:
            self._mocap_future = self._probe_pool.submit(probe_mocap, host)
            return
        if not self._mocap_future.done():
            return

        result = self._mocap_future.result()
        self.pub_rtt.publish(Float64(data=result.rtt_ms))
        self.pub_reach.publish(Bool(data=result.reachable))
        if result.error and not result.error.startswith("mocap_host is not configured"):
            self.get_logger().warn(
                f"mocap ping unavailable: {result.error}", throttle_duration_sec=30
            )
        self._mocap_future = self._probe_pool.submit(probe_mocap, host)

    def stop_workers(self):
        self._probe_pool.shutdown(wait=False, cancel_futures=True)


def _quat_to_rpy_deg(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    rclpy.init()
    node = HopeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_workers()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
