"""Shared incoming owner and existing packet commitment, with shadow outputs."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Require explicit registration and launch only protected shadow outputs."""
    share = FindPackageShare('hope_planner_cpp')

    def config(name):
        return PathJoinSubstitution([share, 'config', name])

    def text(name):
        return ParameterValue(LaunchConfiguration(name), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument('session_id', description='Unique capture session identity'),
            DeclareLaunchArgument('frame_id', description='Registered table coordinate frame'),
            DeclareLaunchArgument(
                'calibration_id', description='Qualified ball/table registration'
            ),
            DeclareLaunchArgument('base_calibration_id', description='Numeric base calibration'),
            DeclareLaunchArgument('base_world_id', description='Numeric base world identity'),
            DeclareLaunchArgument('base_topic', default_value='/a3/base_pose_flat'),
            DeclareLaunchArgument(
                'home_topic', default_value='/hope/runner/session_home_hdu_flat'
            ),
            DeclareLaunchArgument(
                'plane_mode',
                default_value='fixed',
                choices=['fixed', 'per_ball', 'stable_per_ball'],
            ),
            Node(
                package='hope_planner_cpp',
                executable='hope_planner_cpp_node',
                name='hope_planner',
                output='screen',
                parameters=[
                    config('model21800_hardware.yaml'),
                    config('v2_incoming.shadow.yaml'),
                    {
                        'v2_incoming.session_id': text('session_id'),
                        'v2_incoming.frame_id': text('frame_id'),
                        'v2_incoming.calibration_id': text('calibration_id'),
                        'v2_incoming.base_calibration_id': ParameterValue(
                            LaunchConfiguration('base_calibration_id'), value_type=int
                        ),
                        'v2_incoming.base_world_id': ParameterValue(
                            LaunchConfiguration('base_world_id'), value_type=int
                        ),
                        'base_pose_flat_input_topic': text('base_topic'),
                        'session_home_input_topic': text('home_topic'),
                        'plane_reference_mode': text('plane_mode'),
                    },
                ],
            ),
            Node(
                package='hope_planner_cpp',
                executable='hope_ball_flight_packetizer',
                name='hope_ball_flight_packetizer',
                output='screen',
                parameters=[
                    config('model21800_flight_packetizer.yaml'),
                    {
                        'session_id': text('session_id'),
                        'producer_instance_id': 'v2-incoming-shadow-packetizer',
                        'flight_packet_topic': '/hope/shadow/v2_incoming/flight_packet',
                        'adaptive_hit_plane_topic': '/hope/shadow/v2_incoming/hit_plane',
                    },
                ],
            ),
        ]
    )
