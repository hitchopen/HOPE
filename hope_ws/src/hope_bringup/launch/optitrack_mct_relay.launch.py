"""Relay-only launch for the OptiTrack backend (bag replay / bench debugging).

Feeds a live or recorded /optitrack/poses (NamedPoseArrayV2) stream through
optitrack_mct_relay without starting the NatNet driver or the world frame.
Sibling of avatar_pro_vrpn_relay.launch.py.
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
