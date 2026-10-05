#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


class ArmPublisher(Node):
    def __init__(self, arm_topic_name: str):
        super().__init__("arm_publisher")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(JointState, arm_topic_name, qos_profile)

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pos_ini = [
            0.0,
            1.2,
            0.0,
            -0.5,
            1.5,
            0.0,
            0.0,
            0.0,
            -1.2,
            0.0,
            0.5,
            1.5,
            0.0,
            0.0,
        ]

        self.current_pos_offset = 0.0
        self.increasing = True
        self.step = 0.002
        self.pos_offset_max = 0.2
        self.pos_offset_min = -0.2

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 必须正确传入所有手臂关节的名称
        msg.name = [
	    "left_shoulder_pitch_joint",
	    "left_shoulder_roll_joint",
	    "left_shoulder_yaw_joint",
	    "left_elbow_joint",
	    "left_wrist_roll_joint",
	    "left_wrist_pitch_joint",
	    "left_wrist_yaw_joint",
	    "right_shoulder_pitch_joint",
	    "right_shoulder_roll_joint",
	    "right_shoulder_yaw_joint",
	    "right_elbow_joint",
	    "right_wrist_roll_joint",
	    "right_wrist_pitch_joint",
	    "right_wrist_yaw_joint",           
        ]

        current_position = list(self.pos_ini)
        current_position[3] = self.pos_ini[3] + self.current_pos_offset
        current_position[10] = self.pos_ini[10] - self.current_pos_offset
        msg.position = current_position

        if self.increasing:
            self.current_pos_offset += self.step
            if self.current_pos_offset >= self.pos_offset_max:
                self.current_pos_offset = self.pos_offset_max
                self.increasing = False
        else:
            self.current_pos_offset -= self.step
            if self.current_pos_offset <= self.pos_offset_min:
                self.current_pos_offset = self.pos_offset_min
                self.increasing = True

        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing: {{pos[3]: {current_position[3]:.2f}, pos[10]: {current_position[10]:.2f}}}"
        )


def main(args=None):  
    rclpy.init(args=args)
    arm_publisher = ArmPublisher("/motion/control/arm_joint_command")
    try:
        rclpy.spin(arm_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        arm_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
