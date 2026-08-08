from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo
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
    p1 = offsets["p1"]

    nodes = [
        _static_tf(frames["world"], frames["table_center"], landmarks["table_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["p1_half_center"], landmarks["p1_half_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["p2_half_center"], landmarks["p2_half_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["net_center"], landmarks["net_center"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["floor_origin"], landmarks["floor_origin"], [0.0, 0.0, 0.0]),
        _static_tf(frames["world"], frames["virtual_hit_plane"], [x_hit, 0.0, 0.0], [0.0, 0.0, 0.0]),
        _calibrated_marker_tf(frames, offsets, "p1"),
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
                "output_topic": "/a3/base_pose_flat",
                "expected_input_frame": frames["world"],
                "marker_to_base_xyz": p1["xyz_m"],
                "marker_to_base_quaternion_wxyz": p1["quaternion_wxyz"],
                "policy_z_offset": config["planner"]["policy_z_offset"],
                "extrinsic_calibrated": bool(p1.get("calibrated", False)),
                "world_frame_calibrated": bool(
                    contract.get("venue_calibrated", False)
                ),
                "calibration_sha256": str(
                    p1.get("calibration_sha256", "")
                ),
                "world_frame_sha256": str(
                    contract.get("calibration_sha256", "")
                ),
                # Production launches this relay on HDU.  /P1/pose originates
                # on the Laptop, whose ROS clock is not a valid MDU freshness
                # clock; schema-2 therefore carries the HDU ROS receipt stamp.
                "source_stamp_mode": "local_receipt",
            }],
        ),
    ]
    return LaunchDescription(nodes)
