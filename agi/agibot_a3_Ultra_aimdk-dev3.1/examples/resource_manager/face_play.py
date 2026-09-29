#!/usr/bin/env python3

from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros2_plugin_proto.msg import RosMsgWrapper
from aimdk.protocol_pb2 import HFAEmoction

TOPIC = "/skill/pilot/face/play/pb_3Aaimdk_2Eprotocol_2EHFAEmoction"
SERIALIZATION_TYPE = "pb"
TIMER_PERIOD = 0.2
DEFAULT_PUBLICATION_COUNT = 20


def fill_timestamp(timestamp):
    now = datetime.now(timezone.utc)
    timestamp.seconds = int(now.timestamp())
    timestamp.nanos = now.microsecond * 1000
    timestamp.ms_since_epoch = int(now.timestamp() * 1000)


class EmotionPublisher(Node):
    def __init__(self, emotion_topic_name: str, max_publications: int = -1):
        super().__init__("emotion_publisher")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(
            RosMsgWrapper, emotion_topic_name, qos_profile
        )
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.max_publications = max_publications
        self.publications_count = 0

        if self.max_publications > 0:
            self.get_logger().info(
                f"Publisher will run for {self.max_publications} publications."
            )
        else:
            self.get_logger().info("Publisher will run indefinitely.")

    def timer_callback(self):
        emotion = HFAEmoction()
        fill_timestamp(emotion.header.timestamp)
        emotion.e_path = "/agibot/data/resources/default/emoticon/disable_voice/emoticon.mp4"
        emotion.e_id = 15
        emotion.repeat = 1
        emotion.priority = 440
        emotion.is_stop = False

        wrapper = RosMsgWrapper()
        wrapper.serialization_type = "pb"
        wrapper.context = ["aimdk.protocol.HFAEmoction"]
        serialized_bytes=emotion.SerializeToString()
        wrapper.data = [bytes([byte]) for byte in serialized_bytes]

        self.publisher.publish(wrapper)
        if self.publications_count == 0:
            self.get_logger().info(
                "Publishing emotion: e_path=%s e_id=%d repeat=%d priority=%d is_stop=%s"
                % (
                    emotion.e_path,
                    emotion.e_id,
                    emotion.repeat,
                    emotion.priority,
                    emotion.is_stop,
                )
            )
        self.publications_count += 1

        if (
            self.max_publications > 0
            and self.publications_count >= self.max_publications
        ):
            self.get_logger().info(
                "Reached %d publications. Cancelling timer and destroying node."
                % self.max_publications
            )
            self.timer.cancel()
            self.destroy_node()
            return


def main(args=None):
    rclpy.init(args=args)
    emotion_publisher = EmotionPublisher(
        TOPIC,
        max_publications=DEFAULT_PUBLICATION_COUNT,
    )
    try:
        rclpy.spin(emotion_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(emotion_publisher, "timer"):
            emotion_publisher.timer.cancel()
        emotion_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
