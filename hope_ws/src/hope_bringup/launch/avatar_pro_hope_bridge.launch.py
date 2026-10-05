"""Full Avatar-Pro (Chingmu VRPN) -> HOPE mocap bringup.

Chain:  Avatar-Pro / CMTracker (MCServer)  --VRPN-->  vrpn_mocap client_node
        --> /vrpn_mocap/<object>/pose  --> avatar_pro_vrpn_relay
        --> /poses, /tf, /ball/point, /{UCB_P1,UCB_P2}/pose --> planner / WBC

Also starts the static HOPE world frame (table landmarks + mocap->base_link).

TODO before running on hardware (see reimplement.md "Values you must fill in"):
  * server   -> the Avatar-Pro / CMTracker (MCServer) PC IP on the arena LAN.
  * port     -> the Chingmu VRPN port (default 3883).
  * update_freq -> match the mocap camera rate (>= 240-360 Hz for the ball).
  * object names + mocap->base_link offsets are set in the config files.
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
    config_path = launch_dir.parent / "config" / "avatar_pro_vrpn.yaml"

    server = LaunchConfiguration("server")
    port = LaunchConfiguration("port")
    update_freq = LaunchConfiguration("update_freq")
    start_vrpn_client = LaunchConfiguration("start_vrpn_client")
    start_world = LaunchConfiguration("start_world")
    ball_tracking_mode = LaunchConfiguration("ball_tracking_mode")
    ball_object = LaunchConfiguration("ball_object")
    position_scale = LaunchConfiguration("position_scale")

    return LaunchDescription([
        DeclareLaunchArgument(
            "server", default_value="192.168.1.100",
            description="TODO: Avatar-Pro / Chingmu VRPN server (MCServer PC) IP on the arena LAN.",
        ),
        DeclareLaunchArgument("port", default_value="3883", description="Chingmu VRPN port."),
        DeclareLaunchArgument(
            "update_freq", default_value="300.0",
            description="VRPN client poll rate (Hz); match the mocap camera rate (arena "
                        "cameras run 300 Hz; the planner's velocity polyfit window is sized "
                        "for a >=240 Hz ball stream — 180 Hz doubles its time window).",
        ),
        DeclareLaunchArgument("start_vrpn_client", default_value="true"),
        DeclareLaunchArgument("start_world", default_value="true"),
        DeclareLaunchArgument(
            "ball_tracking_mode", default_value="rigid_body",
            description="How to track the ball: 'rigid_body' (default — the ball is the "
                        "STANDARDIZED rigid body named in ball_object) or 'auto' "
                        "(motion-based detection, fallback for setups without a ball body).",
        ),
        DeclareLaunchArgument(
            "ball_object", default_value="Ball",
            description="Ball rigid-body name in CMTracker. STANDARDIZED 2026-07-03: every "
                        "AvatarPro setup must name the ball 'Ball' (robots: UCB_P1/UCB_P2).",
        ),
        DeclareLaunchArgument(
            "position_scale", default_value="0.001",
            description="Uniform position conversion applied by the relay. Use 0.001 for a "
                        "millimetre VRPN feed and 1.0 when CMTracker already streams metres.",
        ),

        # Static HOPE world frame: table landmarks + mocap->base_link offsets.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "hope_world.launch.py")),
            condition=IfCondition(start_world),
        ),

        # VRPN client: Avatar-Pro server -> /vrpn_mocap/<object>/pose (+ /tf).
        Node(
            package="vrpn_mocap",
            executable="client_node",
            namespace="vrpn_mocap",
            name="client_node",
            output="screen",
            condition=IfCondition(start_vrpn_client),
            parameters=[{
                "server": server,
                "port": ParameterValue(port, value_type=int),
                "frame_id": "world",
                "update_freq": ParameterValue(update_freq, value_type=float),
                # multi_sensor:=true exposes EVERY VRPN sensor channel as its own
                # topic (/vrpn_mocap/<sender>/<sensor>/pose). The ball is a single
                # marker, not a nameable rigid body, so it usually only appears as
                # a sensor channel rather than a named /vrpn_mocap/ball/pose. The
                # relay matches objects on the sender name, so the extra
                # sensor-index segment is harmless for PPT/UCB_P1/UCB_P2.
                "multi_sensor": True,
            }],
        ),

        # Relay: /vrpn_mocap/* -> HOPE-standard topics. The two ball launch args
        # override the matching keys in the yaml (later entries win), so the ball
        # tracking method can be chosen on the command line without editing config.
        Node(
            package="hope_bringup",
            executable="avatar_pro_vrpn_relay",
            name="avatar_pro_vrpn_relay",
            output="screen",
            parameters=[
                str(config_path),
                {
                    "ball_tracking_mode": ball_tracking_mode,
                    "ball_object": ball_object,
                    "position_scale": ParameterValue(position_scale, value_type=float),
                },
            ],
        ),
    ])
