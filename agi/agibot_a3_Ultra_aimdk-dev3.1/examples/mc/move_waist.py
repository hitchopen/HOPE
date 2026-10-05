#!/usr/bin/env python3

from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros2_plugin_proto.msg import RosMsgWrapper
from aimdk.protocol_pb2 import MotionControlMoveWaistChannel

CONTROL_SOURCE_MANUAL = 1
WAIST_LIMITS = {
    "waist_pitch": (-0.5, 0.5),
    "waist_roll": (-0.3, 0.3),
    "waist_yaw": (-1.57, 1.57),
    "waist_height": (-0.4, 0.0),
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def fill_header(header, seq: int) -> None:
    now = datetime.now(timezone.utc)
    timestamp_seconds = now.timestamp()
    header.seq = seq
    header.timestamp.seconds = int(timestamp_seconds)
    header.timestamp.nanos = now.microsecond * 1000
    header.timestamp.ms_since_epoch = int(timestamp_seconds * 1000)
    header.control_source = CONTROL_SOURCE_MANUAL


class MoveWaistPublisher(Node):
    def __init__(self, move_waist_topic_name: str):
        super().__init__("move_waist_publisher")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(
            RosMsgWrapper, move_waist_topic_name, qos_profile
        )

        timer_period = 0.05  # 20 Hz, matches legacy T_MoveWaist script cadence
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pose_sequences = self._build_pose_sequences()
        self.repeat_per_pose = 70
        self.max_publications = len(self.pose_sequences) * self.repeat_per_pose
        self.publications_count = 0
        self.current_sequence_index = 0
        self.current_repeat_count = 0

        self.get_logger().info(
            f"Publisher will send {self.max_publications} poses ({len(self.pose_sequences)} unique keyframes).",
        )

    @staticmethod
    def _build_pose_sequences() -> list[dict]:
        roll = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        pitch = [0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
        yaw = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0]
        height =[-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        sequences = []
        for idx in range(len(height)):
            pose = {
                "waist_pitch": pitch[idx],
                "waist_roll": roll[idx],
                "waist_yaw": yaw[idx],
                "waist_height": height[idx],
            }

            for field_name, field_value in pose.items():
                low, high = WAIST_LIMITS[field_name]
                pose[field_name] = clamp(field_value, low, high)

            sequences.append(pose)
        return sequences

    def timer_callback(self):
        if self.current_sequence_index >= len(self.pose_sequences):
            self.get_logger().info(
                "All waist poses have been published. Stopping publisher."
            )
            self.timer.cancel()
            self.destroy_node()
            return

        pose = self.pose_sequences[self.current_sequence_index]
        waist_command = MotionControlMoveWaistChannel()
        fill_header(waist_command.header, self.publications_count)
        waist_command.waist_pitch = pose["waist_pitch"]
        waist_command.waist_roll = pose["waist_roll"]
        waist_command.waist_yaw = pose["waist_yaw"]
        waist_command.waist_height = pose["waist_height"]

        serialized_bytes = waist_command.SerializeToString()

        msg = RosMsgWrapper()
        msg.serialization_type = "pb"
        msg.context = ["aimdk.protocol.MotionControlMoveWaistChannel"]
        msg.data = [bytes([byte]) for byte in serialized_bytes]

        self.publisher.publish(msg)
        if self.current_repeat_count == 0:
            self.get_logger().info(
                "Publishing waist pose: pitch=%.3f roll=%.3f yaw=%.3f height=%.3f"
                % (
                    waist_command.waist_pitch,
                    waist_command.waist_roll,
                    waist_command.waist_yaw,
                    waist_command.waist_height,
                )
            )
        self.publications_count += 1
        self.current_repeat_count += 1

        if self.current_repeat_count >= self.repeat_per_pose:
            self.current_sequence_index += 1
            self.current_repeat_count = 0

        if self.publications_count >= self.max_publications:
            self.get_logger().info(
                "Reached %d publications. Cancelling timer and destroying node.",
                self.max_publications,
            )
            self.timer.cancel()
            self.destroy_node()
            return


def main(args=None):
    rclpy.init(args=args)
    move_waist_publisher = MoveWaistPublisher(
        "/motion/control/move_waist/pb_3Aaimdk_2Eprotocol_2EMotionControlMoveWaistChannel",
    )
    try:
        rclpy.spin(move_waist_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        move_waist_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
