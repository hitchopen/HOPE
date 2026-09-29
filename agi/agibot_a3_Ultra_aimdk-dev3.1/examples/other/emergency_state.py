import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros2_plugin_proto.msg import RosMsgWrapper

from aimdk.protocol_pb2 import EmergencyStateChannel


class EmergencySubscriber(Node):
    def __init__(self):
        super().__init__('emergency_subscriber')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            RosMsgWrapper,
            '/hal_state/emergency/pb_3Aaimdk_2Eprotocol_2EEmergencyStateChannel',
            self.emergency_callback,
            qos_profile
        )

        self.get_logger().info("开始订阅 emergency 数据...")

    def emergency_callback(self, msg):
        try:
            if msg.serialization_type != "pb":
                self.get_logger().warn(
                    f"不支持的序列化类型: {msg.serialization_type}"
                )
                return

            raw_data = b"".join(msg.data)

            emergency_channel = EmergencyStateChannel()
            emergency_channel.ParseFromString(raw_data)

            if emergency_channel.HasField("data"):
                self.print_emergency(emergency_channel.data, tag="单个")

        except Exception as e:
            self.get_logger().error(f"处理 emergency 消息失败: {e}")

    def print_emergency(self, emergency_state, tag=""):
        """格式化输出 emergency 数据"""

        try:
            self.get_logger().info(
                f"[{tag}] "
                f"active: {emergency_state.active} | "
                f"reason: {emergency_state.reason} | "
                f"wired_stop: {emergency_state.wired_emergency_stop} | "
                f"wireless_stop: {emergency_state.wireless_emergency_stop} | "
                f"software_stop: {emergency_state.software_emergency_stop}"
            )

        except Exception as e:
            self.get_logger().error(f"emergency 打印失败: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = EmergencySubscriber()

    try:
        node.get_logger().info("正在监听 emergency 状态数据，按 Ctrl+C 退出...")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("收到退出信号，关闭节点...")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
