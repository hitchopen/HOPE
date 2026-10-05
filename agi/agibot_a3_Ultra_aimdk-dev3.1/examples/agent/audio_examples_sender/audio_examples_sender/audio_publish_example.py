#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from audio_msgs.msg import AudioPlayback,AudioInfo,AudioData  # 需确保 audio msg 可导入

class AudioPlaybackPublisher(Node):
    def __init__(self):
        super().__init__('audio_playback_publisher')
        self.pub = self.create_publisher(AudioPlayback, '/audiohal/audio/playback', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每秒发送

    def timer_callback(self):
        msg = AudioPlayback()

        # 填写时间戳（字段名 attachments 中为 stamps）
        now = self.get_clock().now().to_msg()
        # 兼容性：优先使用 stamps，否则尝试 stamp
        if hasattr(msg, 'stamps'):
            msg.stamps = now
        elif hasattr(msg, 'stamp'):
            msg.stamp = now

        # 填写 info（AudioInfo）
        info = AudioInfo()
        info.channels = 1          # 单声道
        info.sample_rate = 16000   # 16kHz
        info.sample_format = 'S16LE'
        info.coding_format = 'pcm'
        msg.info = info

        # 填写 data（AudioData），示例给一小段原始 bytes
        sample_bytes = b'\x00\x01\x02\x03\x04\x05\x06\x07'  # 示例数据
        msg.data = AudioData(data=list(sample_bytes))

        msg.pkg_name = 'audio_examples_sender'
        msg.token_id = 'token-0001'

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published AudioPlayback pkg={msg.pkg_name} token={msg.token_id} '
            f'data_len={len(msg.data.data)} sample_rate={msg.info.sample_rate}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlaybackPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()