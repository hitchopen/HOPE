"""Launch the HOPE planner node with the default config."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("hope_planner")) / "config" / "hope_planner.yaml"
    return LaunchDescription([
        Node(
            package="hope_planner",
            executable="hope_planner_node",
            name="hope_planner",
            output="screen",
            parameters=[str(config)],
        ),
    ])
