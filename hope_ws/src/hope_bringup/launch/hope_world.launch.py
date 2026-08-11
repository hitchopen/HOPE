from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_world_config():
    config_path = Path(__file__).resolve().parent.parent / "config" / "hope_world_frame.yaml"
    with config_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)["hope_world"]


def _static_tf(parent_frame, child_frame, xyz, rpy):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{child_frame}_static_tf".replace("/", "_"),
        arguments=[
            "--x", str(xyz[0]),
            "--y", str(xyz[1]),
            "--z", str(xyz[2]),
            "--roll", str(rpy[0]),
            "--pitch", str(rpy[1]),
            "--yaw", str(rpy[2]),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )


def _static_tf_quat(parent_frame, child_frame, xyz, quat_wxyz):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{child_frame}_static_tf".replace("/", "_"),
        arguments=[
            "--x", str(xyz[0]),
            "--y", str(xyz[1]),
            "--z", str(xyz[2]),
            "--qx", str(quat_wxyz[1]),
            "--qy", str(quat_wxyz[2]),
            "--qz", str(quat_wxyz[3]),
            "--qw", str(quat_wxyz[0]),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )


def _calibrated_marker_tf(frames, offsets, robot):
    entry = offsets[robot]
    if not bool(entry.get("calibrated", False)):
        return LogInfo(
            msg=(
                f"[hope_world] {robot.upper()} marker->base TF NOT published: "
                "calibration receipt is missing (fail closed)"
            )
        )
    receipt = str(entry.get("calibration_sha256", ""))
    if len(receipt) != 64 or any(c not in "0123456789abcdefABCDEF" for c in receipt):
        raise RuntimeError(
            f"{robot} calibrated=true requires a 64-hex calibration_sha256 receipt"
        )
    return _static_tf_quat(
        frames[f"{robot}_mocap"],
        frames[f"{robot}_base_link"],
        entry["xyz_m"],
        entry["quaternion_wxyz"],
    )


def generate_launch_description():
    config = _load_world_config()
    frames = config["frames"]
    landmarks = config["landmarks_m"]
    offsets = config["mocap_to_base_link"]
    contract = config["contract"]
    x_hit = config["planner"]["x_hit"]
    p1_calibration_file = LaunchConfiguration("p1_calibration_file")
    base_pose_output_topic = LaunchConfiguration("base_pose_output_topic")

    nodes = [
        _static_tf(frames["world"], frames["table_center"], landmarks["table_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["p1_half_center"], landmarks["p1_half_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["p2_half_center"], landmarks["p2_half_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["net_center"], landmarks["net_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["floor_origin"], landmarks["floor_origin"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["virtual_hit_plane"], [x_hit, 0.0, 0.0], [0.0, 0.0, 0.0]),
        # Do not publish P1 -> pelvis_link into the vendor TF tree: the vendor
        # stack already owns pelvis_link below odom.  The relay publishes the
        # independently composed world pose on /a3/mocap/pelvis_pose instead.
        _calibrated_marker_tf(frames, offsets, "p2"),
        # Independent high-rate transport for the native runner and planner.
        # It publishes explicit schema-2 valid=0 packets until BOTH the Motive
        # world frame and marker->pelvis receipts are present.
        Node(
            package="hope_planner",
            executable="hope_base_pose_flat_relay",
            name="hope_base_pose_flat_relay",
            output="screen",
            parameters=[{
                "input_topic": f"/{frames['p1_mocap']}/pose",
                "output_topic": base_pose_output_topic,
                "expected_input_frame": frames["world"],
                "expected_marker_frame": frames["p1_mocap"],
                "pelvis_frame": "pelvis_link",
                "pelvis_pose_topic": "/a3/mocap/pelvis_pose",
                "calibration_file": p1_calibration_file,
                "policy_z_offset": config["planner"]["policy_z_offset"],
                "world_frame_calibrated": bool(
                    contract.get("venue_calibrated", False)
                ),
                "world_frame_sha256": str(
                    contract.get("calibration_sha256", "")
                ),
                # Production launches this relay next to NatNet on the laptop.
                # The input header is already mapped into that host's
                # disciplined ROS system-time epoch.
                "source_stamp_mode": "input_header",
            }],
        ),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "p1_calibration_file",
                default_value="calibration/p1_to_pelvis.json",
                description=(
                    "laptop-local approved P1 -> pelvis_link calibration receipt"
                ),
            ),
            DeclareLaunchArgument(
                "base_pose_output_topic",
                default_value="/a3/base_pose_flat",
                description="schema-2 base-pose output topic",
            ),
            *nodes,
        ]
    )
