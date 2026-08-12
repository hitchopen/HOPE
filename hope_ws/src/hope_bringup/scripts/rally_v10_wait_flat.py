#!/usr/bin/env python3
"""Wait for one valid RallyV10 flat and verify its side and fixed plane."""

import argparse
import math
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
from std_msgs.msg import Float64MultiArray


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--expected-sign', choices=('-1', '+1', 'any'), required=True)
    parser.add_argument('--x-hit', type=float, required=True)
    parser.add_argument('--x-tolerance', type=float, default=0.02)
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('rally_v10_flat_gate')
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    result = {'message': None, 'error': None}

    def callback(msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 12:
            result['error'] = f'malformed flat length {len(data)} < 12'
            return
        if not all(math.isfinite(value) for value in data[:12]):
            result['error'] = 'non-finite value in flat'
            return
        if abs(data[0] - 1.0) > 1.0e-9:
            result['error'] = f'unsupported flat schema {data[0]}'
            return
        if data[1] < 0.5:
            return

        expected_sign = None if args.expected_sign == 'any' else float(args.expected_sign)
        if expected_sign is not None and abs(data[2] - expected_sign) > 1.0e-9:
            result['error'] = (
                f'valid flat has swing_sign={data[2]:+.1f}; expected {expected_sign:+.1f}'
            )
            return
        if abs(data[3] - args.x_hit) > args.x_tolerance:
            result['error'] = (
                f'valid flat px={data[3]:.4f} differs from frozen x_hit={args.x_hit:.4f} '
                f'by more than {args.x_tolerance:.3f} m'
            )
            return
        result['message'] = (
            f'FLAT PASS: valid=1 swing_sign={data[2]:+.1f} '
            f'px={data[3]:.4f} x_hit={args.x_hit:.4f}'
        )

    node.create_subscription(Float64MultiArray, '/racket/command_flat', callback, qos)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and result['message'] is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if result['error'] is not None:
                error = result['error']
                print(f'FLAT FAIL: {error}', file=sys.stderr)
                return 1
        if result['message'] is None:
            print(
                f'FLAT FAIL: no valid /racket/command_flat within {args.timeout:.1f} s',
                file=sys.stderr,
            )
            return 1
        print(result['message'])
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
