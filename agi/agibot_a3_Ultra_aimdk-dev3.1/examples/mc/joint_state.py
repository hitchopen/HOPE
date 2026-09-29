#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    def __init__(self, topic_name: str):
        super().__init__("joint_state_subscriber")

        self.topic_name = topic_name

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(JointState, topic_name, self.listener_callback, qos_profile)

    def listener_callback(self, msg: JointState):
        self.get_logger().info(f"=== Received {self.topic_name} ===")
        self.get_logger().info(f"  header: {msg.header}")
        self.get_logger().info(f"  name: {msg.name}")
        self.get_logger().info(f"  position: {msg.position}")
        self.get_logger().info(f"  velocity: {msg.velocity}")
        self.get_logger().info(f"  effort: {msg.effort}")


def main(args=None):
    rclpy.init(args=args)
    # 手臂关节状态: /motion/control/arm_joint_state
    # 脖子关节状态: /motion/control/neck_joint_state
    joint_state_node = JointStateSubscriber("/motion/control/arm_joint_state")
    try:
        rclpy.spin(joint_state_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
