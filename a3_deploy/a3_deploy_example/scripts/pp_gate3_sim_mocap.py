#!/usr/bin/env python3
"""Expose MuJoCo through the public raw NamedPoseArray boundary as OptiTrack."""

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
from mujoco_sim_msgs.msg import Gate3BallState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from pp_gate3_core import (
    TABLE_HEIGHT_M,
    base_pose_to_marker_pose,
    calibrated_p1_marker_contract,
    world_to_table_position,
)


def translated_pose(world_pose: Pose, table_height_m: float) -> Pose:
    result = Pose()
    (
        result.position.x,
        result.position.y,
        result.position.z,
    ) = world_to_table_position(
        (world_pose.position.x, world_pose.position.y, world_pose.position.z),
        table_height_m,
    )
    result.orientation = world_pose.orientation
    return result


class Gate3SimMocap(Node):
    def __init__(
        self,
        table_height_m: float,
        marker_to_base_xyz: tuple[float, float, float],
        marker_to_base_quaternion_wxyz: tuple[float, float, float, float],
    ) -> None:
        super().__init__("gate3_sim_mocap")
        self._table_height_m = float(table_height_m)
        self._marker_to_base_xyz = marker_to_base_xyz
        self._marker_to_base_quaternion_wxyz = (
            marker_to_base_quaternion_wxyz
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(
            NamedPoseArray, "/optitrack/poses", sensor_qos
        )
        self._pelvis: PoseStamped | None = None
        self._frames = 0
        self._frames_without_pelvis = 0
        self.create_subscription(
            PoseStamped, "/sim/a3/pelvis_pose", self._pelvis_cb, reliable_qos
        )
        self.create_subscription(
            Gate3BallState,
            "/sim/gate3/ball_state",
            self._ball_cb,
            reliable_qos,
        )
        self.create_timer(1.0, self._health)

    def _pelvis_cb(self, msg: PoseStamped) -> None:
        self._pelvis = msg

    @staticmethod
    def _named(name: str, pose: Pose) -> NamedPose:
        result = NamedPose()
        result.name = name
        result.pose = pose
        return result

    def _ball_cb(self, msg: Gate3BallState) -> None:
        if self._pelvis is None:
            self._frames_without_pelvis += 1
            return
        # The public NatNet relay consumes the stable NamedPoseArray contract.
        # build_1 briefly used NamedPoseArrayV2 here while its corresponding
        # relay lived in a separate workspace; publishing V2 to the public
        # relay's V1 subscription leaves the topic visible in the ROS graph but
        # delivers zero samples.
        frame = NamedPoseArray()
        frame.header = msg.header
        frame.header.frame_id = "world"

        pelvis_table = translated_pose(self._pelvis.pose, self._table_height_m)
        marker_position, marker_quaternion = base_pose_to_marker_pose(
            (
                pelvis_table.position.x,
                pelvis_table.position.y,
                pelvis_table.position.z,
            ),
            (
                pelvis_table.orientation.w,
                pelvis_table.orientation.x,
                pelvis_table.orientation.y,
                pelvis_table.orientation.z,
            ),
            self._marker_to_base_xyz,
            self._marker_to_base_quaternion_wxyz,
        )
        p1 = Pose()
        p1.position.x, p1.position.y, p1.position.z = marker_position
        (
            p1.orientation.w,
            p1.orientation.x,
            p1.orientation.y,
            p1.orientation.z,
        ) = marker_quaternion
        table = Pose()
        table.position.x = 1.370
        table.position.y = -0.7625
        table.orientation.w = 1.0
        entries = [self._named("P1", p1), self._named("PPT", table)]
        if msg.active:
            ball = Pose()
            (
                ball.position.x,
                ball.position.y,
                ball.position.z,
            ) = world_to_table_position(
                (msg.position.x, msg.position.y, msg.position.z),
                self._table_height_m,
            )
            ball.orientation.w = 1.0
            entries.insert(0, self._named("Ball", ball))
        frame.poses = entries
        self._pub.publish(frame)
        self._frames += 1

    def _health(self) -> None:
        self.get_logger().info(
            "GATE3 RAW MOCAP frames=%d dropped_without_pelvis=%d ball/P1/PPT "
            "positions are table-surface-frame; P1 is the calibrated marker "
            "(not the pelvis)"
            % (self._frames, self._frames_without_pelvis)
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--table-height-m", type=float, default=TABLE_HEIGHT_M)
    parser.add_argument(
        "--world-config",
        type=Path,
        required=True,
        help="calibrated hope_world_frame.yaml used by the production relay",
    )
    args, ros_args = parser.parse_known_args()
    with args.world_config.open("r", encoding="utf-8") as handle:
        marker_to_base_xyz, marker_to_base_quaternion = (
            calibrated_p1_marker_contract(yaml.safe_load(handle))
        )
    rclpy.init(args=ros_args)
    node = Gate3SimMocap(
        args.table_height_m,
        marker_to_base_xyz,
        marker_to_base_quaternion,
    )
    try:
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except Exception:
        if rclpy.ok():
            raise
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
