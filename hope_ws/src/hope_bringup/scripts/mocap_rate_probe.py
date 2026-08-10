#!/usr/bin/env python3
"""One-shot mocap topic rate probe (bring-up preflight).

Counts messages on one topic for a fixed window and PASSes iff the measured
rate reaches --min-hz. Message type is resolved from the live graph, so it
works for /P1/pose (PoseStamped), /ball/point (PointStamped), /poses
(PoseArray) and /optitrack/poses (NamedPoseArray) alike. When messages carry a
header, it also reports the max header-stamp gap (timestamp-jitter check).

A single short-lived node that exits — suitable for scripted preflight gates
on resource-constrained onboard computers, where long-running `ros2 topic
echo/hz` sessions are unwelcome. Particularly useful for the OptiTrack
backend: NatNet is UDP, so unlike the VRPN TCP port there is nothing to
connect() to before launch — mocap liveness can only be proven by data.

For the raw NatNet ``/optitrack/poses`` topic, this probe measures adapter and
transport liveness only: the competition adapter intentionally publishes empty
array heartbeats when no ``Ball``/``P1``/``P2`` body is valid. Use the managed
bringup's downstream ``/P1/pose`` probe, or inspect array contents, to qualify
competition-body tracking.

Exit codes: 0 = rate OK, 1 = below threshold / topic never appeared.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--min-hz', type=float, required=True)
    parser.add_argument('--window', type=float, default=5.0,
                        help='measurement window in seconds')
    parser.add_argument('--discover-timeout', type=float, default=10.0,
                        help='max wait for the topic to appear in the graph')
    args = parser.parse_args()

    rclpy.init()
    node = Node('mocap_rate_probe')
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    try:
        # Resolve the message type from the live graph.
        msg_type_name = None
        deadline = time.monotonic() + args.discover_timeout
        while time.monotonic() < deadline and msg_type_name is None:
            for topic, types in node.get_topic_names_and_types():
                if topic == args.topic and types:
                    msg_type_name = types[0]
                    break
            if msg_type_name is None:
                rclpy.spin_once(node, timeout_sec=0.2)
        if msg_type_name is None:
            print(
                f'RATE FAIL: {args.topic} never appeared within '
                f'{args.discover_timeout:.1f} s', file=sys.stderr)
            return 1
        msg_type = get_message(msg_type_name)

        counts = {'n': 0, 'stamps': []}

        def callback(msg) -> None:
            counts['n'] += 1
            header = getattr(msg, 'header', None)
            if header is not None:
                counts['stamps'].append(header.stamp.sec + header.stamp.nanosec * 1e-9)

        node.create_subscription(msg_type, args.topic, callback, qos)
        end = time.monotonic() + args.window
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        hz = counts['n'] / args.window
        stamps = counts['stamps']
        gap_note = ''
        if len(stamps) >= 2:
            gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
            if gaps:
                gap_note = (f' header-gap mean={sum(gaps) / len(gaps) * 1e3:.1f}ms'
                            f' max={max(gaps) * 1e3:.1f}ms')

        if hz < args.min_hz:
            print(
                f'RATE FAIL: {args.topic} ({msg_type_name}) {hz:.1f} Hz < '
                f'{args.min_hz:.1f} Hz over {args.window:.1f} s{gap_note}',
                file=sys.stderr)
            return 1
        print(
            f'RATE PASS: {args.topic} ({msg_type_name}) {hz:.1f} Hz >= '
            f'{args.min_hz:.1f} Hz over {args.window:.1f} s{gap_note}')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
