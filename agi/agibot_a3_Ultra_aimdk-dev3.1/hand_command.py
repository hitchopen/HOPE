#!/usr/bin/env python3

from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from ros2_plugin_proto.msg import RosMsgWrapper

from aimdk.protocol_pb2 import HandCommandChannel

TOPIC = "/body_drive/hand_joint_command/pb_3Aaimdk_2Eprotocol_2EHandCommandChannel"
TIMER_PERIOD = 0.5
MAX_PUBLICATIONS = 1
LEFT_CLAW_CMD = 0
LEFT_CLAW_POS = 100
LEFT_CLAW_FORCE = 100
LEFT_CLAW_METHOD = 1
LEFT_CLAW_VEL = 100

RIGHT_CLAW_CMD = 0
RIGHT_CLAW_POS = 100
RIGHT_CLAW_FORCE = 100
RIGHT_CLAW_METHOD = 1
RIGHT_CLAW_VEL = 100


def clamp(value, low, high):
    return max(low, min(high, int(value)))


def normalize_percent(value, field_name):
    normalized = clamp(value, 0, 100)
    if normalized != int(value):
        raise ValueError(f"{field_name} 超出范围 0~100，当前是 {value}")
    return normalized


def normalize_uint32(value, field_name):
    normalized = max(0, int(value))
    if normalized != int(value):
        raise ValueError(f"{field_name} 不能为负数，当前是 {value}")
    return normalized

class HandCommandPublisher(Node):
    def __init__(self):
        super().__init__("hand_command_publisher")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            RosMsgWrapper,
            TOPIC,
            qos_profile,
        )

        self.left_claw = self.build_claw_config(
            LEFT_CLAW_CMD,
            LEFT_CLAW_POS,
            LEFT_CLAW_FORCE,
            LEFT_CLAW_METHOD,
            LEFT_CLAW_VEL,
        )
        self.right_claw = self.build_claw_config(
            RIGHT_CLAW_CMD,
            RIGHT_CLAW_POS,
            RIGHT_CLAW_FORCE,
            RIGHT_CLAW_METHOD,
            RIGHT_CLAW_VEL,
        )
        self.seq = 0
        self.sent_count = 0

        self.timer = self.create_timer(TIMER_PERIOD, self.publish_hand_command)

        self.get_logger().info(f"已开始发布 HandCommandChannel: {TOPIC}")
        self.get_logger().info(f"left_claw={self.left_claw}")
        self.get_logger().info(f"right_claw={self.right_claw}")

    def fill_header(self, header):
        now = datetime.now(timezone.utc)
        ts = now.timestamp()

        header.seq = self.seq
        header.timestamp.seconds = int(ts)
        header.timestamp.nanos = now.microsecond * 1000
        header.timestamp.ms_since_epoch = int(ts * 1000)
        header.control_source = 1

    def build_claw_config(self, cmd, pos, force, clamp_method, vel):
        return {
            "cmd": normalize_uint32(cmd, "cmd"),
            "pos": normalize_percent(pos, "pos"),
            "force": normalize_percent(force, "force"),
            "clamp_method": normalize_uint32(clamp_method, "clamp_method"),
            "vel": normalize_percent(vel, "vel"),
        }

    def fill_agi_claw(self, single_hand_cmd, claw_config):
        claw = single_hand_cmd.agi_claw_cmd
        claw.cmd = claw_config["cmd"]
        claw.pos = claw_config["pos"]
        claw.force = claw_config["force"]
        claw.clamp_method = claw_config["clamp_method"]
        claw.vel = claw_config["vel"]

    def publish_hand_command(self):
        try:
            hand_cmd_channel = HandCommandChannel()
            self.fill_header(hand_cmd_channel.header)

            self.fill_agi_claw(hand_cmd_channel.data.left, self.left_claw)
            self.fill_agi_claw(hand_cmd_channel.data.right, self.right_claw)

            serialized_bytes = hand_cmd_channel.SerializeToString()

            msg = RosMsgWrapper()
            msg.serialization_type = "pb"
            msg.context = ["aimdk.protocol.HandCommandChannel"]
            msg.data = [bytes([b]) for b in serialized_bytes]

            self.publisher.publish(msg)

            self.get_logger().info(
                f"已发布 HandCommandChannel, seq={self.seq}, sent_count={self.sent_count + 1}"
            )

            self.seq += 1
            self.sent_count += 1

            if self.sent_count >= MAX_PUBLICATIONS:
                self.get_logger().info("达到发送次数，退出。")
                self.timer.cancel()
                self.destroy_node()

        except Exception as e:
            self.get_logger().error(f"发布 HandCommandChannel 失败: {e}")


def main():
    rclpy.init()

    node = HandCommandPublisher()

    try:
        node.get_logger().info("正在发布 HandCommandChannel，按 Ctrl+C 退出...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("退出中...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
