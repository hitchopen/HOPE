"""One planner, streaming qualified inputs, non-actuating outputs."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default = PathJoinSubstitution([
        FindPackageShare('hope_planner_cpp'), 'config', 'integrated_planner.shadow.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        Node(package='hope_planner_cpp', executable='hope_planner_cpp_node',
             name='hope_planner', parameters=[LaunchConfiguration('params_file')],
             output='screen'),
    ])
