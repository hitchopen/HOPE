"""Launch the fake HITTER planner (planner_imitate) for sim-to-real bring-up.

This is a SEPARATE command source from the real mocap planner
(hope_planner.launch.py). It never runs alongside the real planner - pick one.

Examples
--------
Dry-run (publishes NOTHING, logs only), level 1 slow forehand:
    ros2 launch hope_planner planner_imitate.launch.py dry_run:=true level:=1

Stand-only bring-up (publishes a stand command), level 0:
    ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=0

Alternating forehand/backhand:
    ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=5
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("hope_planner")) / "config" / "planner_imitate.yaml"

    args = [
        DeclareLaunchArgument("level", default_value="0",
                              description="bring-up level 0..5 (0=stand,1=fh slow,2=fh,3=bh slow,4=bh,5=alt)"),
        DeclareLaunchArgument("frame_id", default_value="base_link",
                              description="'base_link' (robot-relative, default) or 'world'"),
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true=log only (no publish); false=publish /racket/command"),
        DeclareLaunchArgument("backhand_disabled", default_value="false",
                              description="hard-disable backhand swings (extra bring-up safety)"),
        DeclareLaunchArgument("strike_period_s", default_value="3.0",
                              description="seconds per fake strike (2-4 s recommended for bring-up)"),
    ]

    node = Node(
        package="hope_planner",
        executable="planner_imitate_node",
        name="planner_imitate",
        output="screen",
        parameters=[
            str(config),
            {
                "level": LaunchConfiguration("level"),
                "frame_id": LaunchConfiguration("frame_id"),
                "dry_run": LaunchConfiguration("dry_run"),
                "backhand_disabled": LaunchConfiguration("backhand_disabled"),
                "strike_period_s": LaunchConfiguration("strike_period_s"),
            },
        ],
    )
    return LaunchDescription(args + [node])
