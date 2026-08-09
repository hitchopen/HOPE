"""Independent, fail-closed PoseStamped -> native-runner base-pose relay.

The planner's ball solve intentionally runs in another process. Stage-2/3 solves
can exceed the runner's localization freshness horizon, so base forwarding must
not share that callback path.

Schema 2 preserves the HDU ROS receipt time and the complete calibrated
world->base pose.  The upstream PoseStamped clock may belong to the Laptop, so
production uses a local HDU receipt stamp while still rejecting invalid or
reordered upstream stamps.  V17 rejects schema 1, missing receipts, invalid
quaternions, reordered source time, and implausible pose jumps.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from .base_pose_contract import (
    FLAG_EXTRINSIC_CALIBRATED,
    FLAG_POLICY_Z_OFFSET_APPLIED,
    FLAG_QUATERNION_VALID,
    FLAG_SOURCE_STAMP_HDU_ROS,
    FLAG_TRACKING_VALID,
    FLAG_WORLD_FRAME_CALIBRATED,
    SOURCE_STAMP_INPUT_HEADER,
    SOURCE_STAMP_LOCAL_RECEIPT,
    invalid_base_flat,
    pose_to_base_flat,
    receipt_id_u52,
    resolve_wire_source_stamp_ns,
)


class BasePoseFlatRelay(Node):
    """Process-isolated, full-pose localization transport."""

    def __init__(self) -> None:
        super().__init__("hope_base_pose_flat_relay")
        self.declare_parameter("input_topic", "/P1/pose")
        self.declare_parameter("output_topic", "/a3/base_pose_flat")
        self.declare_parameter("expected_input_frame", "world")
        self.declare_parameter("marker_to_base_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "marker_to_base_quaternion_wxyz", [1.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter("policy_z_offset", 0.76)
        self.declare_parameter("extrinsic_calibrated", False)
        self.declare_parameter("world_frame_calibrated", False)
        self.declare_parameter("calibration_sha256", "")
        self.declare_parameter("world_frame_sha256", "")
        # The current OptiTrack ROS abstraction already drops tracking-invalid
        # rigid bodies but does not expose mean marker error. Until that API is
        # extended, a present P1 sample is binary quality=1.
        self.declare_parameter("tracking_quality", 1.0)
        self.declare_parameter("source_stamp_mode", SOURCE_STAMP_INPUT_HEADER)
        self.declare_parameter("max_source_age_s", 0.10)
        self.declare_parameter("max_linear_speed_mps", 3.0)
        self.declare_parameter("max_angular_speed_rps", 8.0)

        self._offset = tuple(
            float(v) for v in self.get_parameter("marker_to_base_xyz").value
        )
        self._offset_quat = tuple(
            float(v)
            for v in self.get_parameter("marker_to_base_quaternion_wxyz").value
        )
        self._z_offset = float(self.get_parameter("policy_z_offset").value)
        self._expected_input_frame = str(
            self.get_parameter("expected_input_frame").value
        )
        self._extrinsic_calibrated = bool(
            self.get_parameter("extrinsic_calibrated").value
        )
        self._world_calibrated = bool(
            self.get_parameter("world_frame_calibrated").value
        )
        self._quality = float(self.get_parameter("tracking_quality").value)
        self._source_stamp_mode = str(
            self.get_parameter("source_stamp_mode").value
        )
        if self._source_stamp_mode not in {
            SOURCE_STAMP_INPUT_HEADER,
            SOURCE_STAMP_LOCAL_RECEIPT,
        }:
            raise ValueError(
                "source_stamp_mode must be 'input_header' or 'local_receipt'"
            )
        self._max_source_age_s = float(self.get_parameter("max_source_age_s").value)
        self._max_linear_speed = float(
            self.get_parameter("max_linear_speed_mps").value
        )
        self._max_angular_speed = float(
            self.get_parameter("max_angular_speed_rps").value
        )
        calibration_sha = str(self.get_parameter("calibration_sha256").value)
        world_sha = str(self.get_parameter("world_frame_sha256").value)
        self._calibration_id = (
            receipt_id_u52(calibration_sha) if self._extrinsic_calibrated else 0
        )
        self._world_frame_id = (
            receipt_id_u52(world_sha) if self._world_calibrated else 0
        )
        self._flags = FLAG_SOURCE_STAMP_HDU_ROS | FLAG_POLICY_Z_OFFSET_APPLIED
        if self._extrinsic_calibrated:
            self._flags |= FLAG_EXTRINSIC_CALIBRATED
        if self._world_calibrated:
            self._flags |= FLAG_WORLD_FRAME_CALIBRATED

        self._sequence = 0
        self._last_input_source_ns: int | None = None
        self._last_source_ns: int | None = None
        self._last_base_position: tuple[float, float, float] | None = None
        self._last_base_quaternion: tuple[float, float, float, float] | None = None
        self._received = 0
        self._published = 0
        self._rejected = 0

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("output_topic").value),
            output_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("input_topic").value),
            self._pose_cb,
            input_qos,
        )
        self.create_timer(1.0, self._log_health)

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._received += 1
        self._sequence += 1
        position_msg = msg.pose.position
        quaternion_msg = msg.pose.orientation
        input_sec = int(msg.header.stamp.sec)
        input_nsec = int(msg.header.stamp.nanosec)
        source_sec = input_sec
        source_nsec = input_nsec
        receipt_ns = int(self.get_clock().now().nanoseconds)
        out = Float64MultiArray()
        try:
            if not self._extrinsic_calibrated or not self._world_calibrated:
                raise ValueError("marker/base or venue world calibration receipt is missing")
            if str(msg.header.frame_id) != self._expected_input_frame:
                raise ValueError(
                    f"input frame {msg.header.frame_id!r} != "
                    f"{self._expected_input_frame!r}"
                )
            input_source_ns = input_sec * 1_000_000_000 + input_nsec
            source_ns = resolve_wire_source_stamp_ns(
                input_sec,
                input_nsec,
                receipt_ns,
                self._source_stamp_mode,
            )
            source_sec, source_nsec = divmod(source_ns, 1_000_000_000)
            if self._source_stamp_mode == SOURCE_STAMP_INPUT_HEADER:
                source_age_s = (receipt_ns - source_ns) * 1.0e-9
                if source_age_s < -0.010 or source_age_s > self._max_source_age_s:
                    raise ValueError(
                        f"source stamp age {source_age_s:.3f}s is invalid/stale"
                    )
            if (
                self._last_input_source_ns is not None
                and input_source_ns <= self._last_input_source_ns
            ):
                raise ValueError("input source timestamp is duplicate or reordered")
            if self._last_source_ns is not None and source_ns <= self._last_source_ns:
                raise ValueError("wire source timestamp is duplicate or reordered")

            flags = self._flags | FLAG_TRACKING_VALID | FLAG_QUATERNION_VALID
            candidate = pose_to_base_flat(
                (position_msg.x, position_msg.y, position_msg.z),
                (quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z),
                self._offset,
                self._offset_quat,
                self._z_offset,
                sequence=self._sequence,
                source_sec=source_sec,
                source_nsec=source_nsec,
                tracking_quality=self._quality,
                flags=flags,
                calibration_id=self._calibration_id,
                world_frame_id=self._world_frame_id,
                previous_base_quaternion_wxyz=self._last_base_quaternion,
            )
            position = tuple(float(v) for v in candidate[5:8])
            quaternion = tuple(float(v) for v in candidate[8:12])
            if self._last_source_ns is not None:
                dt = (source_ns - self._last_source_ns) * 1.0e-9
                if dt <= 0.0:
                    raise ValueError("non-positive source timestep")
                if self._last_base_position is not None:
                    distance = math.sqrt(
                        sum(
                            (a - b) ** 2
                            for a, b in zip(position, self._last_base_position)
                        )
                    )
                    if distance / dt > self._max_linear_speed:
                        raise ValueError(
                            f"base linear jump {distance / dt:.2f}m/s exceeds limit"
                        )
                if self._last_base_quaternion is not None:
                    dot = min(
                        1.0,
                        max(
                            -1.0,
                            abs(
                                sum(
                                    a * b
                                    for a, b in zip(
                                        quaternion, self._last_base_quaternion
                                    )
                                )
                            ),
                        ),
                    )
                    angular_speed = 2.0 * math.acos(dot) / dt
                    if angular_speed > self._max_angular_speed:
                        raise ValueError(
                            f"base angular jump {angular_speed:.2f}rad/s exceeds limit"
                        )
            out.data = candidate
            self._last_input_source_ns = input_source_ns
            self._last_source_ns = source_ns
            self._last_base_position = position
            self._last_base_quaternion = quaternion
            self._published += 1
        except ValueError as exc:
            # Explicit invalidation is safer than a fabricated last-good pose.
            out.data = invalid_base_flat(
                sequence=self._sequence,
                source_sec=source_sec,
                source_nsec=source_nsec,
                flags=self._flags,
            )
            self._rejected += 1
            self.get_logger().warning(
                f"invalid base pose -> publishing valid=0 ({exc})",
                throttle_duration_sec=2.0,
            )
        self._pub.publish(out)

    def _log_health(self) -> None:
        self.get_logger().info(
            "BASE RELAY schema=2 stamp=%s calibrated=%d world=%d "
            "received=%d published=%d rejected=%d"
            % (
                self._source_stamp_mode,
                self._extrinsic_calibrated,
                self._world_calibrated,
                self._received,
                self._published,
                self._rejected,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BasePoseFlatRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
