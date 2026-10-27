"""Display-only publishers without per-publisher diagnostic waitables.

The rclpy defaults add incompatible-QoS/type event handlers to every publisher.
Dozens of display topics make every executor wait-set rebuild expensive. Keep
the wire type, QoS, frequency and content unchanged; input subscriptions and
command publishers retain their normal event diagnostics. Set
HOPE_DISPLAY_QOS_EVENTS=1 to restore publisher-side warning callbacks for a
diagnostic session. Consumer-side QoS/type diagnostics are unaffected.
"""
import os

from rclpy.event_handler import PublisherEventCallbacks


def display_publisher(node, message_type, topic, qos, **kwargs):
    if os.environ.get("HOPE_DISPLAY_QOS_EVENTS", "0") != "1":
        kwargs.setdefault("event_callbacks", PublisherEventCallbacks(use_default_callbacks=False))
    return node.create_publisher(message_type, topic, qos, **kwargs)
