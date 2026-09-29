#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros2_plugin_proto.msg import RosMsgWrapper

from aimdk.protocol_pb2 import ProcessedAudioOutput, AudioVADState

import datetime
import os


class AudioSubscriber(Node):
    def __init__(self):
        super().__init__("audio_subscriber")

        # 音频缓冲区，按stream_id分别存储
        self.audio_buffers = {}  # {stream_id: bytearray()}
        self.recording_state = {}  # {stream_id: bool} 记录是否正在录音

        # 创建音频文件存储目录
        self.audio_output_dir = "audio_recordings"
        os.makedirs(self.audio_output_dir, exist_ok=True)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            RosMsgWrapper,
            "/agent/process_audio_output/pb_3Aaimdk_2Eprotocol_2EProcessedAudioOutput",
            self.audio_callback,
            qos_profile,
        )

        self.get_logger().info("开始订阅降噪音频数据...")

    def audio_callback(self, msg):
        try:
            # 检查序列化类型是否为 pb
            if msg.serialization_type != "pb":
                self.get_logger().warn(f"不支持的序列化类型: {msg.serialization_type}")
                return

            # 将 data 字段从 list[bytes] 转换为 bytes
            audio_data_bytes = b"".join(msg.data)

            # 使用生成的 protobuf 类解析消息
            processed_audio = ProcessedAudioOutput()
            processed_audio.ParseFromString(audio_data_bytes)

            import json
            from google.protobuf.json_format import MessageToDict

            print(
                json.dumps(
                    MessageToDict(processed_audio, preserving_proto_field_name=True),
                    ensure_ascii=False,
                    indent=2,
                )
            )

            self.get_logger().info(
                f"收到音频数据: stream_id={processed_audio.stream_id}, "
                f"vad_state={processed_audio.vad_state}, "
                f"audio_size={len(processed_audio.audio_data)} bytes"
            )

            # 根据VAD状态处理音频
            self.handle_vad_state(processed_audio)

        except Exception as e:
            self.get_logger().error(f"处理音频消息时出错: {e}")

    def handle_vad_state(self, processed_audio):
        """处理不同的VAD状态"""
        vad_state = processed_audio.vad_state
        stream_id = processed_audio.stream_id
        audio_data = processed_audio.audio_data

        # 初始化该stream_id的缓冲区（如果不存在）
        if stream_id not in self.audio_buffers:
            self.audio_buffers[stream_id] = bytearray()
            self.recording_state[stream_id] = False

        # VAD状态名称映射
        vad_state_names = {
            AudioVADState.AUDIO_VAD_STATE_NONE: "无语音",
            AudioVADState.AUDIO_VAD_STATE_BEGIN: "语音开始",
            AudioVADState.AUDIO_VAD_STATE_PROCESSING: "语音处理中",
            AudioVADState.AUDIO_VAD_STATE_END: "语音结束",
        }

        stream_names = {1: "内置麦克风", 2: "外置麦克风"}

        self.get_logger().info(
            f"[{stream_names.get(stream_id, f'未知流{stream_id}')}] "
            f"VAD状态: {vad_state_names.get(vad_state, f'未知状态{vad_state}')} "
            f"音频数据: {len(audio_data)} bytes"
        )

        # 根据VAD状态处理音频数据
        if vad_state == AudioVADState.AUDIO_VAD_STATE_BEGIN:
            self.get_logger().info("🎤 检测到语音开始")
            # 开始新的录音，清空缓冲区
            self.audio_buffers[stream_id].clear()
            self.recording_state[stream_id] = True
            # 添加当前音频数据
            if len(audio_data) > 0:
                self.audio_buffers[stream_id].extend(audio_data)

        elif vad_state == AudioVADState.AUDIO_VAD_STATE_PROCESSING:
            self.get_logger().info("🔄 语音处理中...")
            # 如果正在录音，继续添加音频数据到缓冲区
            if self.recording_state[stream_id] and len(audio_data) > 0:
                self.audio_buffers[stream_id].extend(audio_data)

        elif vad_state == AudioVADState.AUDIO_VAD_STATE_END:
            self.get_logger().info("✅ 语音结束")
            # 添加最后的音频数据
            if self.recording_state[stream_id] and len(audio_data) > 0:
                self.audio_buffers[stream_id].extend(audio_data)

            # 保存完整的音频段
            if (
                self.recording_state[stream_id]
                and len(self.audio_buffers[stream_id]) > 0
            ):
                self.save_audio_segment(bytes(self.audio_buffers[stream_id]), stream_id)

            # 结束录音
            self.recording_state[stream_id] = False

        elif vad_state == AudioVADState.AUDIO_VAD_STATE_NONE:
            # 无语音状态，不进行录音
            if self.recording_state[stream_id]:
                self.get_logger().info("⏹️ 录音状态重置")
                self.recording_state[stream_id] = False

        # 输出当前缓冲区状态
        if stream_id in self.audio_buffers:
            buffer_size = len(self.audio_buffers[stream_id])
            recording = self.recording_state[stream_id]
            self.get_logger().debug(
                f"[Stream {stream_id}] 缓冲区大小: {buffer_size} bytes, 录音状态: {recording}"
            )

    def save_audio_segment(self, audio_data, stream_id):
        """保存音频段 16kHz, 16位, 单声道 PCM"""
        if len(audio_data) > 0:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")

            # 按stream_id创建子目录
            stream_dir = os.path.join(self.audio_output_dir, f"stream_{stream_id}")
            os.makedirs(stream_dir, exist_ok=True)

            # 生成文件名
            stream_names = {1: "internal_mic", 2: "external_mic"}
            stream_name = stream_names.get(stream_id, f"stream_{stream_id}")
            filename = f"{stream_name}_{timestamp}.pcm"
            filepath = os.path.join(stream_dir, filename)

            try:
                with open(filepath, "wb") as f:
                    f.write(audio_data)
                self.get_logger().info(
                    f"音频段已保存: {filepath} (大小: {len(audio_data)} bytes)"
                )

                # 记录音频文件的时长（假设16kHz, 16位, 单声道）
                sample_rate = 16000
                bits_per_sample = 16
                channels = 1
                bytes_per_sample = bits_per_sample // 8
                total_samples = len(audio_data) // (bytes_per_sample * channels)
                duration_seconds = total_samples / sample_rate

                self.get_logger().info(
                    f"音频时长: {duration_seconds:.2f} 秒 ({total_samples} 样本)"
                )

            except Exception as e:
                self.get_logger().error(f"保存音频文件失败: {e}")

    def get_buffer_info(self):
        """获取所有缓冲区的信息（用于调试）"""
        info = {}
        for stream_id in self.audio_buffers:
            info[stream_id] = {
                "buffer_size": len(self.audio_buffers[stream_id]),
                "recording": self.recording_state[stream_id],
            }
        return info


def main(args=None):
    rclpy.init(args=args)

    audio_subscriber = AudioSubscriber()

    try:
        audio_subscriber.get_logger().info("正在监听降噪音频数据，按 Ctrl+C 退出...")
        rclpy.spin(audio_subscriber)
    except KeyboardInterrupt:
        audio_subscriber.get_logger().info("收到退出信号，正在关闭...")
    finally:
        audio_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
