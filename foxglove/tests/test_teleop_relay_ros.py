#!/usr/bin/env python3
"""Local-only DDS test; never run in a robot's ROS domain.

ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 python3 foxglove/tests/test_teleop_relay_ros.py
"""

import importlib.util
import os
import time
import unittest
from pathlib import Path


@unittest.skipUnless(
    os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    and os.environ.get("ROS_DOMAIN_ID") == "219",
    "requires explicit isolated localhost domain 219",
)
class TeleopRelayTest(unittest.TestCase):
    def test_payload_identity_and_no_timer_replay(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Float64MultiArray

        path = Path(__file__).parents[1] / "a3/hope_runner_transport_relay.py"
        spec = importlib.util.spec_from_file_location("teleop_relay", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rclpy.init(args=[])
        relay = module.HopeRunnerTransportRelay()
        client = rclpy.create_node("isolated_teleop_relay_test")
        executor = SingleThreadedExecutor()
        executor.add_node(relay)
        executor.add_node(client)
        received_input, received_state = [], []
        qos = module._reliable_qos()
        pubs = [
            client.create_publisher(Float64MultiArray, topic, qos)
            for topic in (
                "/hope/runner/teleop_input_hdu_flat",
                "/hope/runner/teleop_state_flat",
            )
        ]
        [
            client.create_subscription(
                Float64MultiArray,
                topic,
                lambda msg, dst=dst: dst.append(list(msg.data)),
                qos,
            )
            for topic, dst in (
                ("/hope/runner/teleop_input_flat", received_input),
                ("/hope/runner/teleop_state_hdu_flat", received_state),
            )
        ]

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)

        try:
            spin(1)
            # Deliberately old timestamp: relay must leave it old for Runner rejection.
            payloads = [
                [1.0, 42.0, 7.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [1.0, 42.0, 1.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
            for pub, payload in zip(pubs, payloads):
                pub.publish(Float64MultiArray(data=payload))
            spin(0.5)
            self.assertEqual(received_input, [payloads[0]])
            self.assertEqual(received_state, [payloads[1]])
            spin(0.5)
            self.assertEqual(len(received_input), 1)
            self.assertEqual(len(received_state), 1)
        finally:
            executor.shutdown()
            client.destroy_node()
            relay.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
