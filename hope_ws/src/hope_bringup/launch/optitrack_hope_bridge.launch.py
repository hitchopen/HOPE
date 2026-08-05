"""Bridge an externally launched NatNet2ROS2 adapter into HOPE topics.

Chain:  NatNet2ROS2 /optitrack/poses  -->  optitrack_mct_relay
        --> /poses, /tf, /ball/point, /{P1,P2}/pose  --> hope_planner

OptiTrack sibling of the VRPN path (``vrpn_mocap`` + ``pose_to_posearray``,
wired by ``hope_bringup.launch.py``); both feed the identical ``/poses``
contract (ball at index 0), so everything downstream is unchanged. This launch
starts the HOPE relay side only — launch NatNet2ROS2 independently first, then
run the planner via ``hope_bringup.launch.py``
(``mocap_backend:=optitrack`` includes this file), or separately.
Also starts the static HOPE arena-landmark frames. Robot marker-to-root
transforms are loaded separately from their calibration records.

NatNet2ROS2 owns the load-bearing driver namespace/remaps: its raw
``NamedPoseArray`` stays on ``/optitrack/poses`` rather than colliding with the
HOPE ``geometry_msgs/PoseArray`` contract on ``/poses``. Its raw TF is likewise
kept under ``/optitrack/tf`` so this relay remains the HOPE TF authority.

Before running against a live rig (see docs/OPTITRACK.md), start the independent
adapter workspace with ``ros2 launch motion_capture_tracking
natnet2ros2.launch.py hostname:=<MOTIVE_PC_IP>`` and verify
``/optitrack/poses`` is live.
Rigid-body names are mapped in config/optitrack_relay.yaml: P1/P2/Table assets
and the strict 6-DOF ball rigid body named Ball.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    relay_config_path = launch_dir.parent / "config" / "optitrack_relay.yaml"

    start_world = LaunchConfiguration("start_world")
    position_scale = LaunchConfiguration("position_scale")
    publish_table = LaunchConfiguration("publish_table")

    return LaunchDescription([
        DeclareLaunchArgument("start_world", default_value="true"),
        DeclareLaunchArgument(
            "position_scale", default_value="1.0",
            description="Uniform position conversion applied by the relay. Motive "
                        "streams metres -> 1.0 (use 0.001 only for a millimetre feed).",
        ),
        DeclareLaunchArgument(
            "publish_table", default_value="false",
            description="Setup/recording only: relay the Table asset to /table/pose, "
                        "/poses, and TF. Must remain false in competition.",
        ),
        # Static HOPE arena-landmark frames. P1/P2-to-robot-root transforms
        # are separate calibrated authorities.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "hope_world.launch.py")),
            condition=IfCondition(start_world),
        ),

        # Relay: external /optitrack/poses -> HOPE-standard topics (the relay is the
        # only /tf authority for Ball/P1/P2. Table output is opt-in for a
        # separate setup/recording session and is off by default.
        Node(
            package="hope_bringup",
            executable="optitrack_mct_relay",
            name="optitrack_mct_relay",
            output="screen",
            parameters=[
                str(relay_config_path),
                {
                    "position_scale": ParameterValue(position_scale, value_type=float),
                    "publish_table": ParameterValue(publish_table, value_type=bool),
                },
            ],
        ),
    ])
