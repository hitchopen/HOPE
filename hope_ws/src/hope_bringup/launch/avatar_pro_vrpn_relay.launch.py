from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = Path(__file__).resolve().parent.parent / "config" / "avatar_pro_vrpn.yaml"

    return LaunchDescription([
        Node(
            package="hope_bringup",
            executable="avatar_pro_vrpn_relay",
            name="avatar_pro_vrpn_relay",
            output="screen",
            parameters=[str(config_path)],
        )
    ])
