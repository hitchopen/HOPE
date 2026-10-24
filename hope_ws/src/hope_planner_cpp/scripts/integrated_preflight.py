#!/usr/bin/env python3
"""Read-only integrated input readiness and discovered QoS report."""
import argparse
import json
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from hope_msgs.msg import BallObservationV2
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=3.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('integrated_read_only_preflight')
    statuses, stamps = [], []
    node.create_subscription(DiagnosticArray, '/hope/shadow/integrated_planner/status',
        lambda m: statuses.extend(m.status), 10)
    node.create_subscription(BallObservationV2, '/hope/ball/observation_v2',
        lambda m: stamps.append(m.source_time_ns), qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        topics = {}
        for topic in ['/hope/ball/observation_v2', '/a3/base_pose_flat',
                      '/hope/integrated_planner/game_context']:
            endpoints = node.get_publishers_info_by_topic(topic)
            topics[topic] = [{'node': i.node_name, 'type': i.topic_type,
                             'qos': str(i.qos_profile)} for i in endpoints]
        latest = statuses[-1] if statuses else None
        state = {kv.key: kv.value for kv in latest.values} if latest else {}
        result = {'status': latest.message if latest else 'PLANNER_UNAVAILABLE',
                  'node_status': state, 'publishers': topics,
                  'optional_game_context_required': False,
                  'observed_ball_rate_hz': ((len(stamps)-1)*1e9/(stamps[-1]-stamps[0]))
                  if len(stamps) > 1 and stamps[-1] > stamps[0] else None}
        print(json.dumps(result, indent=2))
        return 0 if latest else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
