import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros2_plugin_proto.msg import RosMsgWrapper

from aimdk.protocol_pb2 import BmsStateChannel


class BmsSubscriber(Node):
    def __init__(self):
        super().__init__("bms_subscriber")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.subscription = self.create_subscription(
            RosMsgWrapper,
            "/aima/bms/data/pb_3Aaimdk_2Eprotocol_2EBmsStateChannel",
            self.bms_callback,
            qos_profile,
        )

        self.get_logger().info("开始订阅BMS数据...")

    def bms_callback(self, msg):
        try:
            if msg.serialization_type != "pb":
                self.get_logger().warn(
                    f"不支持的序列化类型: {msg.serialization_type}"
                )
                return

            raw_data = b"".join(msg.data)

            bms_channel = BmsStateChannel()
            bms_channel.ParseFromString(raw_data)

            if bms_channel.HasField("data"):
                self.print_bms(bms_channel.data, tag="单个")

            for index, bms in enumerate(bms_channel.bms_datas):
                self.print_bms(bms, tag=f"电池{index}")

        except Exception as e:
            self.get_logger().error(f"处理 BMS 消息失败: {e}")

    def print_bms(self, bms, tag=""):
        """格式化输出 BMS 数据"""

        try:
            voltage_v = bms.voltage / 1000.0
            current_a = bms.current / 1000.0
            power_w = bms.power / 1000.0
            temp_c = bms.temperature / 10.0

            self.get_logger().info(
                f"[{tag}] "
                f"电压: {voltage_v:.2f}V | "
                f"电流: {current_a:.2f}A | "
                f"功率: {power_w:.2f}W | "
                f"温度: {temp_c:.1f}℃ | "
                f"容量: {bms.capacity:.1f}mAh | "
                f"电量: {bms.charge:.1f}% | "
                f"循环次数: {bms.cycles_num} | "
                f"充电状态: {bms.power_supply_status} | "
                f"充电器状态: {bms.charger_state} | "
                f"电池状态: {bms.bms_state}"
            )

        except Exception as e:
            self.get_logger().error(f"BMS打印失败: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = BmsSubscriber()

    try:
        node.get_logger().info("正在监听BMS状态数据，按 Ctrl+C 退出...")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("收到退出信号，关闭节点...")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
