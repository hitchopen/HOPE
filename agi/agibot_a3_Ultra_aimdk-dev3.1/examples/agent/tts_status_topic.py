#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros2_plugin_proto.msg import RosMsgWrapper

from aimdk.protocol_pb2 import TTSStatus


class TTSStatusSubscriber(Node):
    def __init__(self):
        super().__init__("tts_status_subscriber")

        # 音频缓冲区，按stream_id分别存储
        self.audio_buffers = {}  # {stream_id: bytearray()}
        self.recording_state = {}  # {stream_id: bool} 记录是否正在录音

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            RosMsgWrapper,
            "/interaction/tts_status/pb_3Aaimdk_2Eprotocol_2ETTSStatus",
            self.tts_status_callback,
            qos_profile,
        )

        self.get_logger().info("开始订阅 tts 播报状态...")

    def tts_status_callback(self, msg):
        try:
            # 检查序列化类型是否为 pb
            if msg.serialization_type != "pb":
                self.get_logger().warn(f"不支持的序列化类型: {msg.serialization_type}")
                return

            # 将 data 字段从 list[bytes] 转换为 bytes
            tts_status_bytes = b"".join(msg.data)

            tts_status = TTSStatus()
            tts_status.ParseFromString(tts_status_bytes)

            import json
            from google.protobuf.json_format import MessageToDict
            self.get_logger().info(
                f"收到 TTS 播报状态: {json.dumps(MessageToDict(tts_status, preserving_proto_field_name=True), ensure_ascii=False, indent=2)}")

        except Exception as e:
            self.get_logger().error(f"处理 TTS 播报状态消息时出错: {e}")


def main(args=None):
    rclpy.init(args=args)

    tts_status_subscriber = TTSStatusSubscriber()

    try:
        tts_status_subscriber.get_logger().info(
            "正在监听 TTS 播报状态，按 Ctrl+C 退出..."
        )
        rclpy.spin(tts_status_subscriber)
    except KeyboardInterrupt:
        tts_status_subscriber.get_logger().info("收到退出信号，正在关闭...")
    finally:
        tts_status_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
