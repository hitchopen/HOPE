"""Operator-computer echo: no timer, command generation or receipt restamping.

The acquisition relay echoes only challenges for its managed launch session.
Runner independently checks the issued nonce, boot, role and 100 ms
MDU-monotonic age. This helper cannot keep the Console/UI role alive.
"""
import math
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def session_fingerprint(session):
    value = 1469598103934665603
    for byte in session.encode('utf-8'):
        value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return (value & ((1 << 52) - 1)) or 1


class OperatorHeartbeat:
    def __init__(self, node, session):
        # Bound by the managed launch, not learned from arbitrary DDS traffic.
        # A standalone or another robot's relay cannot acquire this identity.
        self._session = session
        self._last = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self._pub = node.create_publisher(Float64MultiArray, '/hope/runner/operator_echo_hdu_flat', qos)
        self._challenge_sub = node.create_subscription(Float64MultiArray,
            '/hope/runner/operator_challenge_hdu_flat', self._challenge_cb, qos)

    def _challenge_cb(self, msg):
        values = list(msg.data)
        if (not self._session or len(values) != 4 or values[0] != 1 or
                not all(math.isfinite(x) and x == int(x) and 0 < x <= (1 << 52) for x in values) or
                values[2] != session_fingerprint(self._session)):
            return
        identity = tuple(values[1:])
        if self._last is not None and identity[:2] == self._last[:2] and identity[2] <= self._last[2]:
            return
        self._last = identity
        self._pub.publish(Float64MultiArray(data=values + [2.]))
