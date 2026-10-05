#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros2_plugin_proto.msg import RosMsgWrapper


class LocomotionVelocityPublisher(Node):
    def __init__(self, arm_topic_name: str, max_publications: int = -1):
        super().__init__("locomotion_velocity_publisher")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(
            RosMsgWrapper, arm_topic_name, qos_profile
        )

        timer_period = 0.02  # Corresponds to 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.max_publications = max_publications
        self.publications_count = 0
        if self.max_publications > 0:
            self.get_logger().info(
                f"Publisher will run for {self.max_publications} publications."
            )
        else:
            self.get_logger().info("Publisher will run indefinitely.")

    def timer_callback(self):
        json_str = '{"data": {"mode": 0, "forward_velocity": 0.1, "lateral_velocity": 0.0, "angular_velocity": 0.0}}'.encode()

        msg = RosMsgWrapper()
        msg.serialization_type = "json"
        msg.data = [bytes([x]) for x in json_str]

        self.publisher.publish(msg)
        self.publications_count += 1

        if (
            self.max_publications > 0
            and self.publications_count >= self.max_publications
        ):
            self.get_logger().info(
                f"Reached {self.max_publications} publications. Cancelling timer and destroying node."
            )
            self.timer.cancel()
            self.destroy_node()  # This should allow rclpy.spin() to exit
            return


def main(args=None):
    rclpy.init(args=args)
    # Example: run for 2 seconds. Timer period is 0.02s (50Hz), so 2 seconds = 2 / 0.02 = 100 publications.
    num_publications_for_2_seconds = int(2.0 / 0.02)
    locomotion_velocity_publisher = LocomotionVelocityPublisher(
        "/motion/control/locomotion_velocity/pb_3Aaimdk_2Eprotocol_2EMotionControlLocomotionVelocityChannel",
        max_publications=num_publications_for_2_seconds,
    )
    try:
        rclpy.spin(locomotion_velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        locomotion_velocity_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
