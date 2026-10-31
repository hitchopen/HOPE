"""Passive ROS adapter for the ELink telemetry tracker; no RPC or commands.

Generated vendor protobuf modules are required for decoding. Their absence is
reported as unavailable, never bypassed by guessing a wire layout. Publisher
names/GIDs are a trusted-LAN identity check, not DDS security authentication.
"""
from __future__ import annotations

import time

from hope_elink_telemetry import CHANNELS, decode_vendor_channel


class ElinkTelemetrySubscriptions:
    def __init__(self, node, tracker, lock, *, monotonic=time.monotonic, wall_ns=time.time_ns):
        self._node, self._tracker, self._lock = node, tracker, lock
        self._monotonic, self._wall_ns = monotonic, wall_ns
        self._endpoints = {}
        self._subscriptions = []
        self._timer = None
        self._closed = False
        self.available = False
        self.unavailable_reason = ""
        try:
            from aimdk.protocol_pb2 import BmsStateChannel, EmergencyStateChannel, HandStateChannel
            from ros2_plugin_proto.msg import RosMsgWrapper
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        except (ImportError, RuntimeError, TypeError):
            # Optional generated protobuf modules can be installed but ABI/
            # runtime incompatible. Diagnostics must not prevent lifecycle
            # startup; do not swallow process interruption or SystemExit.
            self._unavailable("ELINK_TELEMETRY_DECODER_UNAVAILABLE")
            return
        self._classes = {"left_hand": HandStateChannel, "right_hand": HandStateChannel,
                         "bms": BmsStateChannel, "emergency": EmergencyStateChannel}
        topics = {}
        for channel in CHANNELS:
            spec = tracker.contract.channels[channel]
            if spec.required:
                topics.setdefault(spec.topic, []).append(channel)
        self._topics = topics
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        try:
            for topic, channels in topics.items():
                subscription = node.create_subscription(
                    RosMsgWrapper, topic, self._callback(topic, tuple(channels)), qos)
                self._subscriptions.append(subscription)
            self._timer = node.create_timer(1.0, self._refresh_endpoints)
            self.available = True
            self._refresh_endpoints()
        except Exception:
            # A partially installed adapter must not retain usable endpoints.
            self.close()
            self._unavailable("ELINK_TELEMETRY_SUBSCRIPTION_UNAVAILABLE")

    def _unavailable(self, reason):
        self.available = False
        self.unavailable_reason = reason
        with self._lock:
            for channel in CHANNELS:
                if self._tracker.contract.channels[channel].required:
                    self._tracker.invalidate(channel, reason)

    def _refresh_endpoints(self):
        with self._lock:
            if self._closed:
                return
        candidates = {}
        try:
            for topic in self._topics:
                infos = self._node.get_publishers_info_by_topic(topic)
                # Never silently choose a publisher in an ambiguous graph.
                if len(infos) != 1:
                    continue
                info = infos[0]
                namespace = info.node_namespace.rstrip("/")
                name = namespace + "/" + info.node_name
                gid = bytes(info.endpoint_gid).hex()
                if gid and any(bytes(info.endpoint_gid)):
                    candidates[topic] = (name, gid)
        except Exception:
            candidates = {}
        observed_at = self._monotonic()
        with self._lock:
            # close() may run while the graph query is in progress.
            if self._closed:
                return
            self._endpoints = {topic: (name, gid, observed_at) for topic, (name, gid) in candidates.items()}
            for topic, channels in self._topics.items():
                if topic not in candidates:
                    for channel in channels:
                        self._tracker.invalidate(channel, "ELINK_TELEMETRY_PUBLISHER_UNAVAILABLE_OR_AMBIGUOUS")

    def _callback(self, topic, channels):
        def consume(message, info):
            # Native IPC supplies paired host receipt clocks, not dispatch time.
            # Generated ROS messages have slots; keep metadata on MessageInfo.
            now = getattr(info, "received_monotonic_s", None)
            wall_ns = getattr(info, "received_wall_ns", None)
            if now is None or wall_ns is None:
                now, wall_ns = self._monotonic(), self._wall_ns()
            with self._lock:
                if self._closed:
                    return
                generation = self._tracker.generation_id
                endpoint = self._endpoints.get(topic)
                try:
                    gid = bytes(info.publisher_gid).hex()
                    if (endpoint is None or not 0 <= self._monotonic() - endpoint[2] < 2.5 or gid != endpoint[1]):
                        raise ValueError("ELINK_TELEMETRY_PUBLISHER_IDENTITY_UNAVAILABLE")
                    if message.serialization_type != "pb" or len(message.data) > 65536:
                        raise ValueError("ELINK_TELEMETRY_WRAPPER_INVALID")
                    # ROS byte[] is normally uint8-like; some vendor examples
                    # expose a list of one-byte bytes objects instead.
                    try:
                        payload = bytes(message.data)
                    except (TypeError, ValueError):
                        if any(not isinstance(v, bytes) or len(v) != 1 for v in message.data):
                            raise ValueError("ELINK_TELEMETRY_WRAPPER_BYTES_INVALID")
                        payload = b"".join(message.data)
                    if not payload:
                        raise ValueError("ELINK_TELEMETRY_WRAPPER_EMPTY")
                    decoded = self._classes[channels[0]]()
                    decoded.ParseFromString(payload)
                except Exception:
                    for channel in channels:
                        self._tracker.invalidate(channel, "ELINK_TELEMETRY_DECODE_OR_IDENTITY_INVALID")
                    return
                for channel in channels:
                    try:
                        sample = decode_vendor_channel(channel, decoded,
                                                       publisher_node=endpoint[0], publisher_gid=gid)
                        self._tracker.update(channel, sample, now=now, wall_ns=wall_ns,
                                             generation_id=generation)
                    except (ValueError, TypeError, AttributeError, OverflowError):
                        self._tracker.invalidate(channel, "ELINK_TELEMETRY_CHANNEL_PAYLOAD_INVALID")
        return consume

    def close(self):
        with self._lock:
            self._closed = True
            self.available = False
            self._endpoints = {}
            for channel in CHANNELS:
                if self._tracker.contract.channels[channel].required:
                    self._tracker.invalidate(channel, "ELINK_TELEMETRY_SUBSCRIPTION_CLOSED")
            timer, subscriptions = self._timer, self._subscriptions
            self._timer, self._subscriptions = None, []
        # Revoke readiness before removing ROS entities: an executor may have
        # already queued callbacks, and one destroy failure must not retain the
        # remaining subscriptions. Never destroy while holding the shared lock.
        if timer is not None:
            try:
                self._node.destroy_timer(timer)
            except Exception:
                pass
        for subscription in subscriptions:
            try:
                self._node.destroy_subscription(subscription)
            except Exception:
                pass
