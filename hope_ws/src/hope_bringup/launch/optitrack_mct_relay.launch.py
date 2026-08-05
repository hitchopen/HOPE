"""Relay-only launch for the OptiTrack backend (bag replay / bench debugging).

Feeds a live or recorded /optitrack/poses (NamedPoseArray) stream through
optitrack_mct_relay without starting the NatNet driver or the world frame.
Pairs with ``fake_optitrack_publisher`` for a no-hardware smoke test. The
independent NatNet2ROS2 workspace supplies the live stream.
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = Path(__file__).resolve().parent.parent / "config" / "optitrack_relay.yaml"

    return LaunchDescription([
        Node(
            package="hope_bringup",
            executable="optitrack_mct_relay",
            name="optitrack_mct_relay",
            output="screen",
            parameters=[str(config_path)],
        )
    ])
