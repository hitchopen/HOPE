#!/usr/bin/env python3
"""Fail-closed static check for the live HOPE mocap/policy vertical contract.

Run after the mocap bridge is streaming and while the ball is motionless on
the table.  The probe never publishes a control command.  It verifies the two
measurements that make a vertical miss diagnosable before the planner starts:

* the tracked Ball origin is its centre, near one ball radius above table z=0;
* the P1 marker pose plus marker->base and table->floor offsets produces a
  plausible policy-frame pelvis height.

The competition adapters deliberately publish no live Table/PPT topic. The
surveyed X/Y origin and axis directions are setup-time evidence pinned by
``hope_world_frame.yaml`` and its calibration receipt; this probe neither
requires that asset nor claims to detect post-calibration table/world drift.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(frozen=True)
class PositionSample:
    receipt_s: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class PoseSample(PositionSample):
    qw: float
    qx: float
    qy: float
    qz: float


def _median(values: Iterable[float]) -> float:
    return float(statistics.median(values))


def _median_position(samples: Sequence[PositionSample]) -> list[float]:
    return [
        _median(sample.x for sample in samples),
        _median(sample.y for sample in samples),
        _median(sample.z for sample in samples),
    ]


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(float(value) for value in values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def _speed_p95(samples: Sequence[PositionSample]) -> float:
    speeds: list[float] = []
    for previous, current in zip(samples, samples[1:]):
        dt = current.receipt_s - previous.receipt_s
        if dt <= 1.0e-6:
            continue
        distance = math.sqrt(
            (current.x - previous.x) ** 2
            + (current.y - previous.y) ** 2
            + (current.z - previous.z) ** 2
        )
        speeds.append(distance / dt)
    return _percentile(speeds, 0.95)


def _rotate_vector(q: Sequence[float], vector: Sequence[float]) -> list[float]:
    qw, qx, qy, qz = (float(value) for value in q)
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return [float(value) for value in vector]
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    vx, vy, vz = (float(value) for value in vector)
    return [
        (1 - 2 * (qy * qy + qz * qz)) * vx
        + 2 * (qx * qy - qw * qz) * vy
        + 2 * (qx * qz + qw * qy) * vz,
        2 * (qx * qy + qw * qz) * vx
        + (1 - 2 * (qx * qx + qz * qz)) * vy
        + 2 * (qy * qz - qw * qx) * vz,
        2 * (qx * qz - qw * qy) * vx
        + 2 * (qy * qz + qw * qx) * vy
        + (1 - 2 * (qx * qx + qy * qy)) * vz,
    ]


def evaluate_frame_samples(
    ball_samples: Sequence[PositionSample],
    robot_samples: Sequence[PoseSample],
    *,
    min_samples: int,
    ball_radius_m: float,
    ball_z_tolerance_m: float,
    ball_speed_p95_max_mps: float,
    marker_to_base_xyz: Sequence[float],
    policy_z_offset_m: float,
    base_policy_z_min_m: float,
    base_policy_z_max_m: float,
) -> dict:
    """Evaluate already-collected samples; kept pure for host-side tests."""

    errors: list[str] = []
    warnings: list[str] = []
    checks: dict[str, dict] = {}

    if len(ball_samples) < min_samples:
        errors.append(f"ball samples {len(ball_samples)} < {min_samples}")
    if len(robot_samples) < min_samples:
        errors.append(f"P1 samples {len(robot_samples)} < {min_samples}")

    if ball_samples:
        ball_median = _median_position(ball_samples)
        ball_speed = _speed_p95(ball_samples)
        ball_z_error = ball_median[2] - ball_radius_m
        ball_ok = (
            len(ball_samples) >= min_samples
            and math.isfinite(ball_speed)
            and ball_speed <= ball_speed_p95_max_mps
            and abs(ball_z_error) <= ball_z_tolerance_m
        )
        checks["ball_on_table"] = {
            "pass": ball_ok,
            "samples": len(ball_samples),
            "median_xyz_m": ball_median,
            "expected_center_z_m": ball_radius_m,
            "z_error_m": ball_z_error,
            "speed_p95_mps": ball_speed,
        }
        if math.isfinite(ball_speed) and ball_speed > ball_speed_p95_max_mps:
            errors.append(
                f"ball is not stationary: speed p95 {ball_speed:.3f} m/s > "
                f"{ball_speed_p95_max_mps:.3f} m/s"
            )
        if abs(ball_z_error) > ball_z_tolerance_m:
            errors.append(
                f"Ball centre z={ball_median[2]:.4f} m, expected "
                f"{ball_radius_m:.4f}+/-{ball_z_tolerance_m:.4f} m; "
                "check Motive world z=0 and Ball pivot"
            )

    if robot_samples:
        robot_median = _median_position(robot_samples)
        mid = robot_samples[len(robot_samples) // 2]
        marker_world = _rotate_vector(
            [mid.qw, mid.qx, mid.qy, mid.qz], marker_to_base_xyz
        )
        base_policy = [
            robot_median[0] + marker_world[0],
            robot_median[1] + marker_world[1],
            robot_median[2] + marker_world[2] + policy_z_offset_m,
        ]
        base_ok = base_policy_z_min_m <= base_policy[2] <= base_policy_z_max_m
        checks["policy_base_height"] = {
            "pass": base_ok and len(robot_samples) >= min_samples,
            "samples": len(robot_samples),
            "marker_median_xyz_m": robot_median,
            "marker_to_base_world_xyz_m": marker_world,
            "policy_z_offset_m": policy_z_offset_m,
            "base_policy_xyz_m": base_policy,
            "allowed_z_m": [base_policy_z_min_m, base_policy_z_max_m],
        }
        if not base_ok:
            errors.append(
                f"corrected policy base z={base_policy[2]:.4f} m outside "
                f"[{base_policy_z_min_m:.3f}, {base_policy_z_max_m:.3f}]; "
                "check P1 pivot, marker_to_base and policy_z_offset"
            )

    return {
        "schema_version": 2,
        "pass": not errors,
        "errors": errors,
        "warnings": warnings,
        "checks": checks,
        "scope": {
            "live_table_pose_required": False,
            "verified_live": ["Ball centre height/stationarity", "P1-derived policy base height"],
            "setup_authority": (
                "surveyed world origin/axes pinned by hope_world_frame.yaml "
                "and its calibration receipt"
            ),
            "not_verified_live": "post-calibration table/world origin or axis drift",
        },
    }


def _parse_xyz(value: str) -> list[float]:
    parts = value.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected X,Y,Z")
    try:
        result = [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected numeric X,Y,Z") from exc
    if not all(math.isfinite(item) for item in result):
        raise argparse.ArgumentTypeError("X,Y,Z must be finite")
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate HOPE mocap frame while the ball rests motionless on the table"
    )
    parser.add_argument("--ball-topic", default="/ball/point")
    parser.add_argument("--robot-topic", default="/P1/pose")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--discover-timeout", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--ball-radius", type=float, default=0.02)
    parser.add_argument("--ball-z-tolerance", type=float, default=0.015)
    parser.add_argument("--ball-speed-p95-max", type=float, default=0.10)
    parser.add_argument("--marker-to-base", type=_parse_xyz, default=[0.0, 0.0, 0.0])
    parser.add_argument("--policy-z-offset", type=float, default=0.76)
    parser.add_argument("--base-policy-z-min", type=float, default=0.70)
    parser.add_argument("--base-policy-z-max", type=float, default=1.20)
    parser.add_argument("--json", default="")
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    if args.duration <= 0.0 or args.discover_timeout <= 0.0 or args.min_samples < 2:
        raise SystemExit("duration/discover-timeout must be positive and min-samples >= 2")

    import rclpy
    from geometry_msgs.msg import PointStamped, PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class Collector(Node):
        def __init__(self):
            super().__init__("hope_mocap_frame_probe")
            self.ball: list[PositionSample] = []
            self.robot: list[PoseSample] = []
            self.create_subscription(
                PointStamped, args.ball_topic, self._ball, qos_profile_sensor_data
            )
            self.create_subscription(
                PoseStamped, args.robot_topic, self._robot, qos_profile_sensor_data
            )

        def _ball(self, msg):
            self.ball.append(PositionSample(time.monotonic(), msg.point.x, msg.point.y, msg.point.z))

        @staticmethod
        def _pose(msg, receipt_s):
            p, q = msg.pose.position, msg.pose.orientation
            return PoseSample(receipt_s, p.x, p.y, p.z, q.w, q.x, q.y, q.z)

        def _robot(self, msg):
            self.robot.append(self._pose(msg, time.monotonic()))

    rclpy.init(args=None)
    node = Collector()
    try:
        discovery_deadline = time.monotonic() + args.discover_timeout
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ball and node.robot:
                break
        collection_deadline = time.monotonic() + args.duration
        while time.monotonic() < collection_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        report = evaluate_frame_samples(
            node.ball,
            node.robot,
            min_samples=args.min_samples,
            ball_radius_m=args.ball_radius,
            ball_z_tolerance_m=args.ball_z_tolerance,
            ball_speed_p95_max_mps=args.ball_speed_p95_max,
            marker_to_base_xyz=args.marker_to_base,
            policy_z_offset_m=args.policy_z_offset,
            base_policy_z_min_m=args.base_policy_z_min,
            base_policy_z_max_m=args.base_policy_z_max,
        )
        report["generated_wall_time_ns"] = time.time_ns()
        report["topics"] = {
            "ball": args.ball_topic,
            "robot": args.robot_topic,
        }
        rendered = json.dumps(report, indent=2, sort_keys=True)
        print(rendered)
        if args.json:
            path = Path(args.json)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(rendered + "\n", encoding="utf-8")
        return 0 if report["pass"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
