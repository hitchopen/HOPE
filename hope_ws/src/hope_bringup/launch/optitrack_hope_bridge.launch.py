"""Laptop-side OptiTrack adapter, calibration, and HOPE pose bridge.

Chain:  NatNet2ROS2 /optitrack/poses  -->  optitrack_mct_relay
        --> /poses, /tf, /ball/point, /{P1,P2}/pose
        --> hope_ball_flight_packetizer --> /ball/flight_packet --> hope_planner

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

import ipaddress
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate_natnet_interface(context):
    start_natnet = LaunchConfiguration("start_natnet").perform(context).lower()
    if start_natnet not in {"1", "true", "yes", "on"}:
        return []

    interface_ip = LaunchConfiguration("motive_interface_ip").perform(context)
    try:
        address = ipaddress.IPv4Address(interface_ip)
    except ipaddress.AddressValueError as exc:
        raise RuntimeError(
            "start_natnet:=true requires "
            "motive_interface_ip:=<LOCAL_MOTIVE_NIC_IPV4>"
        ) from exc
    if address.is_unspecified or address.is_multicast:
        raise RuntimeError(
            "motive_interface_ip must be an explicit local unicast IPv4 address"
        )
    return []


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    relay_config_path = launch_dir.parent / "config" / "optitrack_relay.yaml"

    start_world = LaunchConfiguration("start_world")
    start_natnet = LaunchConfiguration("start_natnet")
    start_calibration = LaunchConfiguration("start_calibration")
    motive_hostname = LaunchConfiguration("motive_hostname")
    motive_interface_ip = LaunchConfiguration("motive_interface_ip")
    position_scale = LaunchConfiguration("position_scale")
    p1_calibration_file = LaunchConfiguration("p1_calibration_file")
    base_pose_output_topic = LaunchConfiguration("base_pose_output_topic")
    debug_csv_path = LaunchConfiguration("debug_csv_path")
    debug_session_id = LaunchConfiguration("debug_session_id")
    start_flight_packetizer = LaunchConfiguration("start_flight_packetizer")
    flight_packet_topic = LaunchConfiguration("flight_packet_topic")
    flight_packet_debug_csv_path = LaunchConfiguration(
        "flight_packet_debug_csv_path"
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_world", default_value="true"),
        DeclareLaunchArgument("start_natnet", default_value="true"),
        DeclareLaunchArgument("start_calibration", default_value="true"),
        DeclareLaunchArgument("start_flight_packetizer", default_value="true"),
        DeclareLaunchArgument(
            "motive_hostname",
            default_value="192.168.100.111",
            description="Motive/NatNet server IPv4 address",
        ),
        DeclareLaunchArgument(
            "motive_interface_ip",
            default_value="",
            description=(
                "Local wired-NIC IPv4 used for NatNet multicast. Required "
                "when start_natnet is true."
            ),
        ),
        DeclareLaunchArgument(
            "p1_calibration_file",
            default_value="calibration/p1_to_pelvis.json",
            description=(
                "laptop-local P1 calibration JSON, relative to the launch "
                "working directory unless absolute"
            ),
        ),
        DeclareLaunchArgument(
            "debug_csv_path",
            default_value="",
            description="Optional per-frame mocap audit CSV path.",
        ),
        DeclareLaunchArgument(
            "debug_session_id",
            default_value="",
            description="Session identifier copied into mocap and packet audit rows.",
        ),
        DeclareLaunchArgument(
            "flight_packet_topic",
            default_value="/ball/flight_packet",
        ),
        DeclareLaunchArgument(
            "base_pose_output_topic",
            default_value="/a3/base_pose_flat",
        ),
        DeclareLaunchArgument(
            "flight_packet_debug_csv_path",
            default_value="",
            description="Optional Laptop flight-packet audit CSV path.",
        ),
        OpaqueFunction(function=_validate_natnet_interface),
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
                "interface_ip": motive_interface_ip,
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
                "base_pose_output_topic": base_pose_output_topic,
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
                    "debug_csv_path": debug_csv_path,
                    "debug_session_id": debug_session_id,
                },
            ],
        ),

        # Foxglove lifecycle data adapter. It observes the same /poses stream,
        # freezes one complete incoming flight, and publishes the immutable
        # transport packet consumed by the HDU Planner. The downstream
        # build_1 estimator, bounce, target and schema-2 logic remain unchanged.
        Node(
            package="hope_planner_cpp",
            executable="hope_ball_flight_packetizer",
            name="hope_ball_flight_packetizer",
            output="screen",
            condition=IfCondition(start_flight_packetizer),
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("hope_planner_cpp"),
                    "config",
                    "model21800_flight_packetizer.yaml",
                ]),
                {
                    "session_id": debug_session_id,
                    "flight_packet_topic": flight_packet_topic,
                    "debug_csv_path": flight_packet_debug_csv_path,
                },
            ],
        ),
    ])
