"""Laptop-side OptiTrack adapter, calibration, and HOPE pose bridge.

Chain:  NatNet2ROS2 /optitrack/poses  -->  optitrack_mct_relay
        --> /poses, /tf, /ball/point, /{P1,P2}/pose  --> hope_planner

When launched directly on the external computer, this owns NatNet2ROS2, the
per-run P1 calibration service, the laptop-local calibration JSON, the base
pose relay, and the static HOPE arena frames. ``hope_bringup.launch.py`` also
includes this file in relay-only mode for its legacy independently launched
NatNet workflow.

NatNet2ROS2 owns the load-bearing driver namespace/remaps: its raw
``NamedPoseArray`` stays on ``/optitrack/poses`` rather than colliding with the
HOPE ``geometry_msgs/PoseArray`` contract on ``/poses``. NatNet2ROS2 publishes
no raw TF, so this relay remains the HOPE TF authority.

Before running against a live rig (see docs/OPTITRACK.md), verify
``/optitrack/poses`` is live and that PREPARE replaces the laptop-local
``calibration/p1_to_pelvis.json`` before policy entry.
Rigid-body names are mapped in config/optitrack_relay.yaml: P1/P2 and the
strict 6-DOF ball rigid body named Ball.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    relay_config_path = launch_dir.parent / "config" / "optitrack_relay.yaml"

    start_world = LaunchConfiguration("start_world")
    start_natnet = LaunchConfiguration("start_natnet")
    start_calibration = LaunchConfiguration("start_calibration")
    motive_hostname = LaunchConfiguration("motive_hostname")
    position_scale = LaunchConfiguration("position_scale")
    p1_calibration_file = LaunchConfiguration("p1_calibration_file")

    return LaunchDescription([
        DeclareLaunchArgument("start_world", default_value="true"),
        DeclareLaunchArgument("start_natnet", default_value="true"),
        DeclareLaunchArgument("start_calibration", default_value="true"),
        DeclareLaunchArgument(
            "motive_hostname",
            default_value="192.168.100.111",
            description="Motive/NatNet server IPv4 address",
        ),
        DeclareLaunchArgument(
            "p1_calibration_file",
            default_value="calibration/p1_to_pelvis.json",
            description=(
                "laptop-local P1 calibration JSON, relative to the launch "
                "working directory unless absolute"
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("motion_capture_tracking"),
                    "launch",
                    "natnet2ros2.launch.py",
                ])
            ),
            condition=IfCondition(start_natnet),
            launch_arguments={
                "hostname": motive_hostname,
                # Each PREPARE performs one fit. Keeping the publisher alive
                # does not trigger recalibration during policy execution.
                "publish_p1_markers": "true",
            }.items(),
        ),
        DeclareLaunchArgument(
            "position_scale", default_value="1.0",
            description="Uniform position conversion applied by the relay. Motive "
                        "streams metres -> 1.0 (use 0.001 only for a millimetre feed).",
        ),
        # Static HOPE arena-landmark frames. P1/P2-to-robot-root transforms
        # are separate calibrated authorities.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "hope_world.launch.py")),
            condition=IfCondition(start_world),
            launch_arguments={
                "p1_calibration_file": p1_calibration_file,
            }.items(),
        ),

        # Internal laptop service. Foxglove exposes only the five robot-facing
        # controls; PREPARE calls this service after PD_STAND is settled.
        Node(
            package="hope_bringup",
            executable="p1_marker_cad_calibration_server",
            name="p1_marker_cad_calibration_server",
            output="screen",
            condition=IfCondition(start_calibration),
            parameters=[{
                "calibration_file": p1_calibration_file,
            }],
        ),

        # Relay: external /optitrack/poses -> HOPE-standard topics (the relay is the
        # only /tf authority for Ball/P1/P2.
        Node(
            package="hope_bringup",
            executable="optitrack_mct_relay",
            name="optitrack_mct_relay",
            output="screen",
            parameters=[
                str(relay_config_path),
                {
                    "position_scale": ParameterValue(position_scale, value_type=float),
                },
            ],
        ),
    ])
