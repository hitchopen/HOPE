#!/usr/bin/env python3
"""Exercise the supported native Runner gateway on isolated localhost DDS.

ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 HOPE_NATIVE_BIN_DIR=/tmp/hope-native \
  python3 foxglove/tests/test_teleop_relay_ros.py
"""

import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time
import unittest


@unittest.skipUnless(
    os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    and os.environ.get("ROS_DOMAIN_ID") == "219"
    and os.environ.get("HOPE_NATIVE_BIN_DIR"),
    "requires isolated localhost domain 219 and candidate native binaries",
)
class TeleopRelayTest(unittest.TestCase):
    def test_payload_identity_and_no_timer_replay(self):
        import rclpy
        from std_msgs.msg import Float64MultiArray, MultiArrayDimension

        def fields(message):
            return (message.layout.data_offset,
                    tuple((d.label, d.size, d.stride) for d in message.layout.dim),
                    struct.pack('<' + 'd' * len(message.data), *message.data))

        rclpy.init(args=[])
        client = rclpy.create_node("isolated_native_teleop_test")
        names = (
            "remote_state_topic", "local_state_topic", "local_control_topic", "remote_control_topic",
            "local_teleop_input_topic", "remote_teleop_input_topic",
            "remote_teleop_state_topic", "local_teleop_state_topic",
            "remote_session_home_topic", "local_session_home_topic",
        )
        topics = {name: f"/cpu_native_test/teleop/{name}" for name in names}
        args = [str(Path(os.environ["HOPE_NATIVE_BIN_DIR"]) / "hope-transport-gateway"), "--ros-args"]
        for name, topic in topics.items():
            args += ["-p", f"{name}:={topic}"]
        received = [[], [], []]
        routes = (
            ("local_teleop_input_topic", "remote_teleop_input_topic"),
            ("remote_teleop_state_topic", "local_teleop_state_topic"),
            ("remote_session_home_topic", "local_session_home_topic"),
        )
        pubs, subs = [], []
        for index, (source, target) in enumerate(routes):
            pubs.append(client.create_publisher(Float64MultiArray, topics[source], 10))
            subs.append(client.create_subscription(Float64MultiArray, topics[target],
                lambda msg, dst=received[index]: dst.append(fields(msg)), 10))

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(client, timeout_sec=0.01)

        process = None
        with tempfile.TemporaryFile(mode="w+") as output:
            try:
                process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 8
                while not all(pub.get_subscription_count() for pub in pubs) and time.monotonic() < deadline:
                    self.assertIsNone(process.poll(), "native gateway exited before discovery")
                    spin(.05)
                self.assertTrue(all(pub.get_subscription_count() for pub in pubs))
                spin(.3)
                self.assertEqual(received, [[], [], []], "startup must not invent input or HOME")
                # Deliberately old timestamps remain old for Runner validation.
                payloads = (
                    [1., 42., 7., 1., 1., 1., 0., -0., 0., 0., 1.],
                    [1., 42., 1., 0., .1, 0., 0., -0., 0., 0., 0.],
                    [1., 42., 17., -0., -3.],
                )
                expected = []
                for index, (publisher, payload) in enumerate(zip(pubs, payloads)):
                    message = Float64MultiArray(data=payload)
                    message.layout.data_offset = index
                    message.layout.dim = [MultiArrayDimension(label="original", size=len(payload), stride=len(payload))]
                    expected.append([fields(message)])
                    publisher.publish(message)
                spin(.5)
                self.assertEqual(received, expected)
                spin(.7)
                self.assertEqual(received, expected, "stopped source must not be replayed")
            finally:
                if process is not None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
                client.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
