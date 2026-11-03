"""Launch the explicitly selected v2 shadow predictor without a robot runner."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('hope_planner_cpp')) /
                 'config' / 'flight_prediction_v2_shadow.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='hope_planner_cpp', executable='hope_flight_prediction_v2_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
