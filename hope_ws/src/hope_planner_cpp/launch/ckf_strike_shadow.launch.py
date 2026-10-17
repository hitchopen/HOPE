"""Explicit shadow-only launch: two new nodes, no runner or production planner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('hope_planner_cpp')) / 'config' / 'ckf_strike_shadow.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='hope_planner_cpp', executable='hope_ball_event_forecaster_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
        Node(package='hope_planner_cpp', executable='hope_strike_target_selector_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
