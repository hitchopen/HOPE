"""Generic HOPE bringup: motion capture -> flight packetizer -> C++ Planner.

Both mocap backends feed the same ``/poses`` contract with the ball at index
zero.  The packetizer freezes each flight once on ``/ball/flight_packet``;
the production Planner consumes that immutable packet rather than refitting a
moving sample window independently.

Examples::

    ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=vrpn
    ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack \
        motive_hostname:=192.168.100.111
    ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mocap_backend = LaunchConfiguration("mocap_backend")
    use_fake_ball = LaunchConfiguration("use_fake_ball")
    ball_pose_topic = LaunchConfiguration("ball_pose_topic")
    motive_hostname = LaunchConfiguration("motive_hostname")
    flight_window_s = LaunchConfiguration("flight_window_s")

    planner_config = (
        Path(get_package_share_directory("hope_planner_cpp"))
        / "config"
        / "model21800_hardware.yaml"
    )
    packetizer_config = (
        Path(get_package_share_directory("hope_planner_cpp"))
        / "config"
        / "model21800_flight_packetizer.yaml"
    )

    fake_ball_off = ["'", use_fake_ball, "'.lower() not in ('true', '1')"]
    vrpn_selected = IfCondition(
        PythonExpression(["'", mocap_backend, "' == 'vrpn' and "] + fake_ball_off)
    )
    optitrack_selected = IfCondition(
        PythonExpression(
            ["'", mocap_backend, "' == 'optitrack' and "] + fake_ball_off
        )
    )

    # VRPN2ROS2 publishes one PoseStamped topic per tracker.  This adapter
    # preserves the triggering sample stamp in the common PoseArray contract.
    pose_adapter = Node(
        package="hope_bringup",
        executable="pose_to_posearray",
        name="pose_to_posearray",
        output="screen",
        parameters=[{"input_topics": [[ball_pose_topic]], "trigger_index": 0}],
        condition=vrpn_selected,
    )

    # HOPE uses the hardened NamedPoseArrayV2 NatNet driver.  Disable
    # the nested world/calibration/packetizer nodes because this top-level
    # launch owns the single packetizer and does not own a robot calibration.
    optitrack_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hope_bringup"),
                    "launch",
                    "optitrack_hope_bridge.launch.py",
                ]
            )
        ),
        launch_arguments={
            "hostname": motive_hostname,
            "start_world": "false",
            "start_mocap_node": "true",
            "start_calibration": "false",
            "start_flight_packetizer": "false",
        }.items(),
        condition=optitrack_selected,
    )

    fake_ball = Node(
        package="hope_bringup",
        executable="fake_ball_publisher",
        name="fake_ball_publisher",
        output="screen",
        condition=IfCondition(use_fake_ball),
    )

    packetizer = Node(
        package="hope_planner_cpp",
        executable="hope_ball_flight_packetizer",
        name="hope_ball_flight_packetizer",
        output="screen",
        parameters=[
            str(packetizer_config),
            {"flight_window_s": ParameterValue(flight_window_s, value_type=float)},
        ],
    )

    planner = Node(
        package="hope_planner_cpp",
        executable="hope_planner_cpp_node",
        name="hope_planner",
        output="screen",
        parameters=[str(planner_config)],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mocap_backend",
                default_value="vrpn",
                choices=["vrpn", "optitrack"],
                description="Mocap source; both backends publish /poses.",
            ),
            DeclareLaunchArgument(
                "use_fake_ball",
                default_value="false",
                description="Use a synthetic /poses stream instead of mocap.",
            ),
            DeclareLaunchArgument(
                "ball_pose_topic",
                default_value="/vrpn_mocap/Ball/pose_id_0",
                description="VRPN ball PoseStamped topic aggregated into /poses.",
            ),
            DeclareLaunchArgument(
                "motive_hostname",
                default_value="192.168.100.111",
                description="OptiTrack Motive/NatNet server address.",
            ),
            DeclareLaunchArgument(
                "flight_window_s",
                default_value="0.18",
                description="Time window retained by the C++ flight packetizer.",
            ),
            pose_adapter,
            optitrack_bridge,
            fake_ball,
            packetizer,
            planner,
        ]
    )
