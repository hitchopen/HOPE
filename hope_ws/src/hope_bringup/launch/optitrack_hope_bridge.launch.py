"""Laptop OptiTrack -> calibrated base pose and immutable flight packets.

Chain:  Motive (NatNet UDP, cmd port 1510)  -->  motion_capture_tracking_node
        (vendored, namespace /optitrack; poses/tf remapped to /optitrack/*)
        --> optitrack_mct_relay --> /poses, /UCB_P1/pose, /UCB_P2/pose
        --> hope_ball_flight_packetizer --> /ball/flight_packet --> C++ Planner

Sibling of avatar_pro_hope_bridge.launch.py (ChingMu/VRPN backend); both emit
the identical HOPE topic contract, so everything downstream is unchanged.
It also starts the per-run 12-station v3 sticker or legacy V2/V3 ball
marker-carrier calibration services and the
calibrated schema-2 base-pose relay. In P2 table-side mode the relay rotates all
world poses 180 degrees about the table centre before Planner/base consumers.
The HDU transport relay republishes that Laptop-owned pose on the authoritative
Runner topic without recomputing calibration.

The remaps are LOAD-BEARING, not cosmetic:
  * the driver's `poses` topic is motion_capture_tracking_interfaces/
    NamedPoseArray -- on the bare /poses name it would collide with the HOPE
    /poses contract (geometry_msgs/PoseArray) as a DDS type mismatch;
  * the driver broadcasts /tf with raw body names (UCB_P1/PPT/...) which would
    fight the relay's world->UCB_P1/PPT transforms and the hope_world statics.

TODO before running on hardware (see docs/operations/run_mocap.md):
  * hostname -> the Motive PC IP on the arena LAN.
  * Motive streaming pane per mocap reference §6.3 (Z-up, Rigid Bodies ON,
    NatNet enabled; Multicast and Unicast are both supported and discovered
    from NAT_SERVERINFO). Labeled Markers stays OFF for normal runtime and
    must be ON only for the selected UCB_P1/UCB_P2 capture described
    by publish_ucb_markers.
  * rigid-body names + ball tracker are set in config/optitrack_mct.yaml and
    config/optitrack_relay.yaml (UCB_P1/UCB_P2/PPT assets; upstream ball
    'Ball' or 'ball', canonical downstream name 'Ball').
    The selected ``tracked_marker_frame`` overrides the marker aggregate's
    default UCB_P1 asset so P2 calibration receives UCB_P2 marker messages.
  * after importing the pelvis-aligned 12-station sticker asset, name it for
    the selected UCB frame, restart NatNet to refresh MODELDEF, and generate a
    fresh calibration/ucb_robot_to_pelvis.json receipt before policy entry.
"""

import ipaddress
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def validate_mocap_interface(context):
    if not IfCondition(LaunchConfiguration("start_mocap_node")).evaluate(context):
        return []
    if LaunchConfiguration("mocap_type").perform(context) != "optitrack":
        return []
    value = LaunchConfiguration("interface_ip").perform(context)
    if value:
        try:
            address = ipaddress.IPv4Address(value)
            if address.is_multicast or int(address) == 0xffffffff:
                raise ValueError("expected a local unicast IPv4 address")
        except ValueError as exc:
            raise RuntimeError(f"Invalid interface_ip {value!r}: {exc}") from exc
    return []


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    mct_config_path = launch_dir.parent / "config" / "optitrack_mct.yaml"
    relay_config_path = launch_dir.parent / "config" / "optitrack_relay.yaml"

    hostname = LaunchConfiguration("hostname")
    mocap_type = LaunchConfiguration("mocap_type")
    start_mocap_node = LaunchConfiguration("start_mocap_node")
    start_world = LaunchConfiguration("start_world")
    start_calibration = LaunchConfiguration("start_calibration")
    start_flight_packetizer = LaunchConfiguration("start_flight_packetizer")
    position_scale = LaunchConfiguration("position_scale")
    ucb_calibration_file = LaunchConfiguration("ucb_calibration_file")
    base_pose_output_topic = LaunchConfiguration("base_pose_output_topic")
    debug_csv_path = LaunchConfiguration("debug_csv_path")
    debug_session_id = LaunchConfiguration("debug_session_id")
    flight_packet_topic = LaunchConfiguration("flight_packet_topic")
    flight_packet_debug_csv_path = LaunchConfiguration(
        "flight_packet_debug_csv_path"
    )
    publish_ucb_markers = LaunchConfiguration("publish_ucb_markers")
    ucb_marker_match_max_distance_m = LaunchConfiguration(
        "ucb_marker_match_max_distance_m"
    )
    ucb_max_live_max_mm = LaunchConfiguration("ucb_max_live_max_mm")
    ucb_v2_max_registration_rms_mm = LaunchConfiguration(
        "ucb_v2_max_registration_rms_mm"
    )
    ucb_v3_max_registration_rms_mm = LaunchConfiguration(
        "ucb_v3_max_registration_rms_mm"
    )
    table_side = LaunchConfiguration("table_side")
    tracked_marker_frame = LaunchConfiguration("tracked_marker_frame")
    tracked_pose_topic = LaunchConfiguration("tracked_pose_topic")
    table_length_m = LaunchConfiguration("table_length_m")
    table_width_m = LaunchConfiguration("table_width_m")

    return LaunchDescription([
        DeclareLaunchArgument(
            "interface_ip", default_value="",
            description="Local Motive NIC IPv4; empty or 0.0.0.0 selects the "
                        "source address of the route to the Motive server.",
        ),
        DeclareLaunchArgument(
            "hostname",
            description="REQUIRED (no default by operator policy — venue values are "
                        "passed explicitly): Motive PC IP (NatNet server), e.g. "
                        "hostname:=192.168.50.1. Transmission mode, data port, and "
                        "multicast group are discovered from NAT_SERVERINFO; "
                        "Multicast and Unicast share this launch path.",
        ),
        DeclareLaunchArgument(
            "mocap_type", default_value="optitrack",
            description="libmotioncapture backend: 'optitrack' (live Motive) or "
                        "'mock' (no-hardware smoke test; streams the yaml "
                        "rigid_bodies statically).",
        ),
        DeclareLaunchArgument(
            "start_mocap_node", default_value="true",
            description="Set false when replaying a recorded /optitrack/poses bag.",
        ),
        DeclareLaunchArgument("start_world", default_value="true"),
        DeclareLaunchArgument("start_calibration", default_value="true"),
        DeclareLaunchArgument("start_flight_packetizer", default_value="true"),
        DeclareLaunchArgument(
            "table_side",
            default_value="P1",
            description=(
                "P1 consumes Motive world directly; P2 rotates every relayed "
                "pose 180 degrees about the table centre."
            ),
        ),
        DeclareLaunchArgument("tracked_marker_frame", default_value="UCB_P1"),
        DeclareLaunchArgument("tracked_pose_topic", default_value="/UCB_P1/pose"),
        DeclareLaunchArgument("table_length_m", default_value="2.74"),
        DeclareLaunchArgument("table_width_m", default_value="1.525"),
        DeclareLaunchArgument(
            "ucb_calibration_file",
            default_value="calibration/ucb_robot_to_pelvis.json",
            description="Laptop-local active marker-frame -> pelvis_link receipt.",
        ),
        DeclareLaunchArgument(
            "base_pose_output_topic", default_value="/a3/base_pose_flat"
        ),
        DeclareLaunchArgument(
            "position_scale", default_value="1.0",
            description="Uniform position conversion applied by the relay. Motive "
                        "streams metres -> 1.0 (use 0.001 only for a millimetre feed).",
        ),
        DeclareLaunchArgument(
            "debug_csv_path", default_value="",
            description="Optional per-frame raw/normalized mocap CSV path.",
        ),
        DeclareLaunchArgument(
            "debug_session_id", default_value="",
            description="Session identifier copied into every mocap CSV row.",
        ),
        DeclareLaunchArgument(
            "flight_packet_topic", default_value="/ball/flight_packet"
        ),
        DeclareLaunchArgument(
            "flight_packet_debug_csv_path", default_value="",
            description="Optional Laptop flight-packet audit CSV path.",
        ),
        DeclareLaunchArgument(
            "publish_ucb_markers", default_value="false",
            description=(
                "Calibration only: publish an atomic /optitrack/"
                "rigid_body_markers message containing the selected UCB robot "
                "asset's NatNet ModelDef "
                "and same-frame labeled-marker samples. Motive Labeled "
                "Markers must also be ON. Keep false during normal play."
            ),
        ),
        DeclareLaunchArgument(
            "ucb_marker_match_max_distance_m",
            default_value="0.008",
            description=(
                "Calibration-only nearest-sample association gate. New-robot "
                "commissioning currently uses 8 mm while the independent "
                "mapping-margin and fit-quality gates remain strict."
            ),
        ),
        DeclareLaunchArgument(
            "ucb_max_live_max_mm",
            default_value="8.0",
            description=(
                "Calibration receipt maximum per-marker live-to-ModelDef error. "
                "New-robot commissioning currently uses 8 mm; RMS and "
                "registration quality gates remain unchanged."
            ),
        ),
        DeclareLaunchArgument(
            "ucb_v2_max_registration_rms_mm",
            default_value="3.25",
            description=(
                "V2 CAD-registration RMS gate in millimetres. The explicit "
                "3.25 mm new-pants gate covers the field-measured 3.0819 mm "
                "fit while registration-max, pairwise, live-error, and "
                "mapping-margin gates remain unchanged."
            ),
        ),
        DeclareLaunchArgument(
            "ucb_v3_max_registration_rms_mm",
            default_value="3.0",
            description=(
                "V3 CAD-registration RMS gate in millimetres; unchanged from "
                "the strict profile contract."
            ),
        ),

        # Reject invalid configuration before any nodes or respawn loops start.
        OpaqueFunction(function=validate_mocap_interface),

        # Static HOPE world frame: table landmarks + mocap->base_link offsets.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "hope_world.launch.py")),
            condition=IfCondition(start_world),
            launch_arguments={
                "ucb_calibration_file": ucb_calibration_file,
                "base_pose_output_topic": base_pose_output_topic,
                "table_side": table_side,
                "tracked_marker_frame": tracked_marker_frame,
                "tracked_pose_topic": tracked_pose_topic,
                "table_length_m": table_length_m,
                "table_width_m": table_width_m,
                "require_marker_identity": publish_ucb_markers,
            }.items(),
        ),

        # Two explicit marker-carrier profiles share one active receipt path.
        # A successful request atomically replaces it; the command proxy
        # serializes requests and waits for the matching base receipt.
        Node(
            package="hope_bringup",
            executable="p1_marker_cad_calibration_server",
            name="ucb_robot_cad_calibration_server_stickers_v3",
            output="screen",
            condition=IfCondition(start_calibration),
            parameters=[
                {
                    "calibration_file": ucb_calibration_file,
                    "asset_name": tracked_marker_frame,
                    "marker_layout": "stickers_v3",
                    "marker_names": ",".join(f"S{i:02d}" for i in range(1,25)),
                    "allow_nominal_markers": False,
                    "service_name": "/a3/calibration/recompute_p1",
                    "table_side": table_side,
                    "table_length_m": ParameterValue(
                        table_length_m, value_type=float
                    ),
                    "table_width_m": ParameterValue(
                        table_width_m, value_type=float
                    ),
                    "max_live_max_mm": ParameterValue(
                        ucb_max_live_max_mm, value_type=float
                    ),
                    "max_registration_rms_mm": 3.0,
                }
            ],
        ),
        Node(
            package="hope_bringup",
            executable="p1_marker_cad_calibration_server",
            name="ucb_robot_cad_calibration_server_v3",
            output="screen",
            condition=IfCondition(start_calibration),
            parameters=[
                {
                    "calibration_file": ucb_calibration_file,
                    "asset_name": tracked_marker_frame,
                    "marker_layout": "v3",
                    "marker_names": "f1,f2,f3,f4,f5,b1,b2,b3,b4,b5",
                    "allow_nominal_markers": False,
                    "service_name": "/a3/calibration/recompute_ucb_robot_v3",
                    "table_side": table_side,
                    "table_length_m": ParameterValue(
                        table_length_m, value_type=float
                    ),
                    "table_width_m": ParameterValue(
                        table_width_m, value_type=float
                    ),
                    "max_live_max_mm": ParameterValue(
                        ucb_max_live_max_mm, value_type=float
                    ),
                    "max_registration_rms_mm": ParameterValue(
                        ucb_v3_max_registration_rms_mm, value_type=float
                    ),
                }
            ],
        ),

        Node(
            package="hope_bringup",
            executable="p1_marker_cad_calibration_server",
            name="ucb_robot_cad_calibration_server_v2",
            output="screen",
            condition=IfCondition(start_calibration),
            parameters=[
                {
                    "calibration_file": ucb_calibration_file,
                    "asset_name": tracked_marker_frame,
                    "marker_layout": "v2",
                    "marker_names": "f1,f2,f3,f4,f5,b1,b2,b3,b4,b5",
                    "allow_nominal_markers": True,
                    "service_name": "/a3/calibration/recompute_ucb_robot_v2",
                    "table_side": table_side,
                    "table_length_m": ParameterValue(
                        table_length_m, value_type=float
                    ),
                    "table_width_m": ParameterValue(
                        table_width_m, value_type=float
                    ),
                    "max_live_max_mm": ParameterValue(
                        ucb_max_live_max_mm, value_type=float
                    ),
                    "max_registration_rms_mm": ParameterValue(
                        ucb_v2_max_registration_rms_mm, value_type=float
                    ),
                }
            ],
        ),


        # Vendored NatNet driver: Motive -> /optitrack/poses (NamedPoseArrayV2),
        # /optitrack/tf, /optitrack/pointCloud. Motive-native rigid bodies
        # (UCB_P1/UCB_P2/PPT) and the ball (upstream 'Ball' or 'ball',
        # canonicalized to 'Ball' by optitrack_mct.yaml) arrive in the same
        # NamedPoseArrayV2.
        Node(
            package="motion_capture_tracking",
            executable="motion_capture_tracking_node",
            namespace="optitrack",
            name="motion_capture_tracking_node",
            output="screen",
            # A disconnected Motive cable makes the vendored constructor exit
            # after its bounded NAT_SERVERINFO retries. Keep the launch alive
            # and retry the driver so restoring the physical link repairs
            # Cali V2/V3 without requiring a complete lifecycle restart.
            # Calibration still fails closed until fresh marker frames arrive.
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(start_mocap_node),
            parameters=[
                str(mct_config_path),
                {
                    "hostname": hostname,
                    "interface_ip": LaunchConfiguration("interface_ip"),
                    "type": mocap_type,
                    "topics.rigid_body_markers.enabled": ParameterValue(
                        publish_ucb_markers, value_type=bool
                    ),
                    "topics.rigid_body_markers.asset_name":
                        tracked_marker_frame,
                    "topics.rigid_body_markers.geometric_match_max_distance_m":
                        ParameterValue(
                            ucb_marker_match_max_distance_m, value_type=float
                        ),
                },
            ],
            remappings=[
                # `poses`/`pointCloud` are relative and already resolve under
                # /optitrack; /tf and /tf_static are absolute and must be
                # remapped explicitly (see module docstring).
                ("/tf", "/optitrack/tf"),
                ("/tf_static", "/optitrack/tf_static"),
            ],
        ),

        # Relay: /optitrack/poses -> HOPE-standard topics (the relay is the
        # only /tf authority for ball/PPT/UCB_P1/UCB_P2, matching AvatarPro).
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
                    "table_side": table_side,
                    "table_length_m": ParameterValue(
                        table_length_m, value_type=float
                    ),
                    "table_width_m": ParameterValue(
                        table_width_m, value_type=float
                    ),
                },
            ],
        ),

        # Freeze each incoming flight once on the Laptop. The HDU Planner
        # consumes this immutable packet and deduplicates retransmissions.
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
