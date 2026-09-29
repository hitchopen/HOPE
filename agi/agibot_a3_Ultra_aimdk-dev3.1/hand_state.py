import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from ros2_plugin_proto.msg import RosMsgWrapper

from aimdk.protocol_pb2 import HandStateChannel

TOPIC = "/body_drive/hand_joint_state/pb_3Aaimdk_2Eprotocol_2EHandStateChannel"

class HandStateSubscriber(Node):
    def __init__(self):
        super().__init__("hand_state_subscriber")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            RosMsgWrapper,
            TOPIC,
            self.hand_state_callback,
            qos_profile,
        )

        self.get_logger().info(f"已开始订阅 HandStateChannel: {TOPIC}")

    def hand_state_callback(self, msg):
        try:
            if msg.serialization_type != "pb":
                self.get_logger().warn(
                    f"收到不支持的序列化类型: {msg.serialization_type}"
                )
                return

            # 拼接 bytes
            raw_bytes = b"".join(msg.data)

            # 解析 protobuf
            hand_state_channel = HandStateChannel()
            hand_state_channel.ParseFromString(raw_bytes)

            # 日志输出
            import json
            from google.protobuf.json_format import MessageToDict

            self.get_logger().info(
                f"HandStateChannel: {json.dumps(MessageToDict(hand_state_channel, preserving_proto_field_name=True), ensure_ascii=False, indent=2)}"        
            )

        except Exception as e:
            self.get_logger().error(f"解析 HandStateChannel 数据时出现错误: {e}") 
def main(args=None):
    rclpy.init(args=args)
    node = HandStateSubscriber()

    try:
        node.get_logger().info("正在监听 HandStateChannel，按 Ctrl+C 退出...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("退出中...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()