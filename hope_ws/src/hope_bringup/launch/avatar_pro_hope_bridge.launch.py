"""CMTracker VRPN -> selected pelvis ground truth + ball transport.

Import config/pants_profiles/a3_{v2,v4}_{P1,P2}_ground_truth.json first.
VRPN has no marker ModelDef: geometry cannot be verified over this protocol.
The base wire remains invalid until the asset and table-world setup are
explicitly confirmed. This launch never starts the robot Runner.
"""
import hashlib
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Written by the stopped-only Foxglove ball selector; late override wins.
    ball_profile = Path.home() / ".config/hope-foxglove/ball_physics.json"
    ball_overrides = [str(ball_profile)] if ball_profile.exists() else []
    config = Path(__file__).resolve().parent.parent / "config/avatar_pro_vrpn.yaml"
    arg = LaunchConfiguration
    typed = lambda name, kind: ParameterValue(arg(name), value_type=kind)
    # A nominal contract ID, activated ONLY by world_frame_confirmed. Never
    # reuse the Motive +90-degree venue receipt for a different VRPN world.
    world_sha = hashlib.sha256(
        b"hope.vrpn.operator_confirmed.canonical_table_world.v1;X=P1_to_P2;Z=up;"
        b"table_bounds_x=0,2.74;y=-1.525,0;surface_z=0"
    ).hexdigest()
    declarations = [
        ("server", "192.168.1.100", "CMTracker / AvatarPro VRPN server IP."),
        ("port", "3883", "VRPN TCP/UDP port."),
        ("update_freq", "300.0", "VRPN polling frequency, Hz."),
        ("start_vrpn_client", "true", "Start the VRPN client."),
        ("start_world", "false", "Compatibility argument; legacy Motive world is not loaded."),
        ("start_base_pose", "true", "Start guarded schema-2 localization adapter."),
        ("debug_session_id", "", "Flight packet session ID when packetization is enabled."),
        ("ball_tracking_mode", "rigid_body", "rigid_body (Ball) or auto."),
        ("ball_object", "Ball", "Case-sensitive ball sender name."),
        ("position_scale", "0.001", "0.001 for millimetres; 1.0 for metres."),
        ("pants_profile", "v2", "v2 or v4; initial standalone selection."),
        ("table_side", "P1", "P1 or P2; applies to ball AND robot."),
        ("follow_lifecycle_config", "true", "Follow Foxglove CONFIRM CONFIG; wait if unavailable."),
        ("pelvis_asset_confirmed", "false", "Operator verified imported selected asset retains pelvis pivot/axes."),
        ("world_frame_confirmed", "false", "Operator verified VRPN world matches canonical table surface frame."),
        ("start_flight_packetizer", arg("world_frame_confirmed"),
         "Enable Planner ball packets; defaults to world_frame_confirmed. Set false only for pose-only diagnostics."),
        ("base_pose_output_topic", "/a3/base_pose_laptop_flat", "HDU relay input; do not bypass HDU clock stamping."),
    ]
    actions = [DeclareLaunchArgument(name, default_value=value, description=description)
               for name, value, description in declarations]
    actions += [
        Node(package="vrpn_mocap", executable="client_node", namespace="vrpn_mocap",
             name="client_node", output="screen", condition=IfCondition(arg("start_vrpn_client")),
             parameters=[{"server": arg("server"), "port": typed("port", int),
                          "frame_id": "world", "update_freq": typed("update_freq", float),
                          "multi_sensor": True}]),
        Node(package="hope_bringup", executable="avatar_pro_vrpn_relay",
             name="avatar_pro_vrpn_relay", output="screen", parameters=[str(config), {
                 "server": arg("server"), "port": typed("port", int),
                 "ball_tracking_mode": arg("ball_tracking_mode"), "ball_object": arg("ball_object"),
                 "position_scale": typed("position_scale", float), "pants_profile": arg("pants_profile"),
                 "table_side": arg("table_side"), "follow_lifecycle_config": typed("follow_lifecycle_config", bool),
             }]),
        Node(package="hope_bringup", executable="hope_base_pose_flat_relay",
             name="hope_base_pose_flat_relay", output="screen", condition=IfCondition(arg("start_base_pose")),
             parameters=[{
                 "localization_source": "vrpn_ground_truth",
                 "vrpn_pelvis_asset_confirmed": typed("pelvis_asset_confirmed", bool),
                 "world_frame_calibrated": typed("world_frame_confirmed", bool),
                 "world_frame_sha256": world_sha, "source_stamp_mode": "input_header",
                 "output_topic": arg("base_pose_output_topic"), "policy_z_offset": 0.76,
             }]),
        Node(package="hope_planner_cpp", executable="hope_ball_flight_packetizer",
             name="hope_ball_flight_packetizer", output="screen",
             condition=IfCondition(arg("start_flight_packetizer")), parameters=[
                 PathJoinSubstitution([FindPackageShare("hope_planner_cpp"), "config",
                                       "model21800_flight_packetizer.yaml"]),
                 *ball_overrides,
                 {"session_id": arg("debug_session_id"), "flight_packet_topic": "/ball/flight_packet"},
             ]),
    ]
    return LaunchDescription(actions)
