#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


class HandPublisher(Node):
    def __init__(self, hand_topic_name: str):
        super().__init__("hand_publisher")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(
            JointState,
            hand_topic_name,
            qos_profile,
        )

        # 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "O10Hand"

        # 20 个关节名称
        msg.name = [
            # Left Hand
            "left_thumb_rotation_0",
            "left_thumb_wiggles_1",
            "left_thumb_bent_2",
            "left_index_wiggles_0",
            "left_index_bent_1",
            "left_middle_bent",
            "left_ring_wiggles_0",
            "left_ring_bent_1",
            "left_pinky_wiggles_0",
            "left_pinky_bent_1",

            # Right Hand
            "right_thumb_rotation_0",
            "right_thumb_wiggles_1",
            "right_thumb_bent_2",
            "right_index_wiggles_0",
            "right_index_bent_1",
            "right_middle_bent",
            "right_ring_wiggles_0",
            "right_ring_bent_1",
            "right_pinky_wiggles_0",
            "right_pinky_bent_1",
        ]

        # 与 name 一一对应，共20个位置值（0~2000）
        msg.position = [
            # Left
            0.0,       # left_thumb_rotation_0
            1000.0,    # left_thumb_wiggles_1
            2000.0,    # left_thumb_bent_2
            2000.0,    # left_index_wiggles_0
            2000.0,    # left_index_bent_1
            2000.0,    # left_middle_bent
            2000.0,    # left_ring_wiggles_0
            2000.0,    # left_ring_bent_1
            2000.0,    # left_pinky_wiggles_0
            2000.0,    # left_pinky_bent_1

            # Right
            0.0,       # right_thumb_rotation_0
            1000.0,    # right_thumb_wiggles_1
            2000.0,    # right_thumb_bent_2
            2000.0,    # right_index_wiggles_0
            2000.0,    # right_index_bent_1
            2000.0,    # right_middle_bent
            2000.0,    # right_ring_wiggles_0
            2000.0,    # right_ring_bent_1
            2000.0,    # right_pinky_wiggles_0
            2000.0,    # right_pinky_bent_1
        ]

        # o10hand不填
        msg.effort = []

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = HandPublisher("/motion/control/hand_joint_command")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
