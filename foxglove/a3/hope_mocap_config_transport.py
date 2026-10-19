#!/usr/bin/env python3
"""Forward supervisor snapshots through a fresh HDU network participant.

The supervisor may predate a USB network adapter. Keep its active session and
hardware-operation lock intact; only this read-only transport restarts when
interfaces change. Distinct topics prevent loops. No cached timer republication:
loss of the supervisor must still expire the Laptop's configuration freshness.
"""
import json
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def network_addresses():
    result = subprocess.run(['ip', '-j', '-4', 'address', 'show'],
                            check=True, capture_output=True, text=True)
    return sorted((item['ifname'], address['local'])
                  for item in json.loads(result.stdout)
                  for address in item.get('addr_info', [])
                  if item['ifname'] != 'lo')


class MocapConfigTransport(Node):
    def __init__(self):
        super().__init__('hope_mocap_config_transport', start_parameter_services=False)
        self.publisher = self.create_publisher(String, '/hope/lifecycle/config/mocap_transport', 10)
        self.subscription = self.create_subscription(
            String, '/hope/lifecycle/config/mocap', self.publisher.publish, 10)
        self.addresses = network_addresses()
        self.create_timer(1.0, self.check_network)
        self.get_logger().info('Forwarding original mocap snapshots; no configuration changes or cached heartbeats')

    def check_network(self):
        if network_addresses() != self.addresses:
            raise RuntimeError('Network addresses changed; recreate config transport DDS')


def main():
    rclpy.init()
    node = MocapConfigTransport()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
