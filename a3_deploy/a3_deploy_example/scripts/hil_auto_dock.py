#!/usr/bin/env python3
"""Drive seeded auto-dock trials against the MuJoCo plant and the MDU Runner.

The Runner is already running on the robot computer. This process only resets
the simulated body and sends PD_STAND / ENTER_DOCK on the isolated ROS domain.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_spin001_serve_mujoco as sim
from runner_sim_profile import apply_hope_contact_profile

REPO = Path(__file__).resolve().parents[3]
DEFAULT_DOCK = REPO / "a3_deploy/a3_deploy_example/assets/a3_runtime/dock/dock.yaml"
DEFAULT_MODEL = sim.DEFAULT_MODEL
DOCK_REASONS = {
    0: "none",
    1: "timeout",
    2: "mocap_stale",
    3: "mocap_invalid",
    4: "wrong_frame",
    5: "start_too_far",
    6: "attempts_exhausted",
    7: "stopped",
}
DOCK_RESULTS = {0: "none", 1: "running", 2: "done", 3: "failed"}
MODE_PD_STAND = 1
ACTION_ENTER_PD_STAND = 3
ACTION_ENTER_DOCK = 14
TARGET = (-0.50, -0.7625, 0.0)


def _yaw(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = quat_wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _tilt(quat_wxyz: np.ndarray) -> float:
    _, x, y, _ = (float(value) for value in quat_wxyz)
    return math.acos(min(1.0, max(-1.0, 1.0 - 2.0 * (x * x + y * y))))


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = np.sort(np.asarray(values, dtype=np.float64))
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


def _spawn(rng: np.random.Generator, x_range, y_range, yaw_range) -> np.ndarray:
    low = np.array([x_range[0], y_range[0], yaw_range[0]])
    high = np.array([x_range[1], y_range[1], yaw_range[1]])
    if np.any(high <= low):
        raise RuntimeError("spawn ranges must be low < high")
    for _ in range(10000):
        pose = rng.uniform(low, high)
        if pose[0] < 0.0:
            return pose
    raise RuntimeError("every spawn landed on the table")


def _intersects_table(model, data) -> bool:
    table = int(model.geom("gate3_table_collision").id)
    mujoco.mj_forward(model, data)
    return any(table in (int(contact.geom1), int(contact.geom2)) for contact in data.contact[: data.ncon])


def draw_start(rng, model, data, qa, stand_q, x_range, y_range, yaw_range):
    for _ in range(10000):
        candidate = _spawn(rng, x_range, y_range, yaw_range)
        mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
        data.qpos[0] = candidate[0]
        data.qpos[1] = candidate[1]
        half = candidate[2] / 2.0
        data.qpos[3:7] = [math.cos(half), 0.0, 0.0, math.sin(half)]
        data.qpos[qa] = stand_q
        if not _intersects_table(model, data):
            return candidate
    raise RuntimeError("every spawn intersected the table")


class Plant:
    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from mujoco_sim_msgs.msg import SimReset
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._rclpy = rclpy
        self._SimReset = SimReset
        self._JointState = JointState
        self._Float = Float64MultiArray
        if not rclpy.ok():
            rclpy.init()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        class _Node(Node):
            def __init__(self) -> None:
                super().__init__("hil_auto_dock")
                self.pelvis: PoseStamped | None = None
                self.pelvis_stamp = 0.0
                self.state: list[float] | None = None
                self.dock: list[float] | None = None
                self.base: list[float] | None = None
                self.base_receipt = 0.0
                self.create_subscription(PoseStamped, "/sim/a3/pelvis_pose", self._pelvis, qos)
                self.create_subscription(Float64MultiArray, "/hope/runner/state_flat", self._state, qos)
                self.create_subscription(Float64MultiArray, "/hope/runner/dock_state_flat", self._dock, qos)
                self.create_subscription(Float64MultiArray, "/a3/base_pose_flat", self._base, qos)

            def _pelvis(self, msg: PoseStamped) -> None:
                self.pelvis = msg
                self.pelvis_stamp = time.monotonic()

            def _state(self, msg: Float64MultiArray) -> None:
                self.state = [float(value) for value in msg.data]

            def _dock(self, msg: Float64MultiArray) -> None:
                self.dock = [float(value) for value in msg.data]

            def _base(self, msg: Float64MultiArray) -> None:
                self.base = [float(value) for value in msg.data]
                self.base_receipt = time.monotonic()

        self.node = _Node()
        self.reset_pub = self.node.create_publisher(SimReset, "/sim/a3/reset", qos)
        self.control_pub = self.node.create_publisher(Float64MultiArray, "/hope/runner/control_request_flat", qos)
        self.request_id = 1

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait(self, predicate, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def reset(self, pose: np.ndarray, height: float, joints: list[str], stand_q: np.ndarray) -> None:
        msg = self._SimReset()
        msg.mode = self._SimReset.MODE_KEYFRAME
        msg.keyframe_id = 0
        msg.set_base = True
        msg.pelvis_pose.position.x = float(pose[0])
        msg.pelvis_pose.position.y = float(pose[1])
        msg.pelvis_pose.position.z = float(height)
        half = float(pose[2]) / 2.0
        msg.pelvis_pose.orientation.w = math.cos(half)
        msg.pelvis_pose.orientation.z = math.sin(half)
        msg.zero_all_velocities = True
        msg.clear_ctrl = True
        msg.set_joints = True
        msg.joint_state = self._JointState()
        msg.joint_state.name = list(joints)
        msg.joint_state.position = [float(value) for value in stand_q]
        self.reset_pub.publish(msg)

    def command(self, action: int) -> int:
        msg = self._Float()
        request = self.request_id
        self.request_id += 1
        msg.data = [2.0, float(request), float(action), 0.0]
        self.control_pub.publish(msg)
        return request

    def close(self) -> None:
        self.node.destroy_node()


def _mode(state: list[float] | None) -> int | None:
    if state is None or len(state) < 5:
        return None
    return int(state[3])


def _fallen(plant: Plant) -> tuple[bool, float, float]:
    pelvis = plant.node.pelvis
    if pelvis is None:
        return False, 0.0, 0.0
    quat = np.array([
        pelvis.pose.orientation.w,
        pelvis.pose.orientation.x,
        pelvis.pose.orientation.y,
        pelvis.pose.orientation.z,
    ])
    height = float(pelvis.pose.position.z)
    tilt = _tilt(quat)
    return tilt > 0.8 or height < 0.55, tilt, height


def run_trial(plant: Plant, pose: np.ndarray, height: float, joints, stand_q, timeout: float) -> dict:
    plant.node.dock = None
    plant.reset(pose, height, joints, stand_q)
    plant.spin(0.2)
    plant.reset(pose, height, joints, stand_q)
    arrived = plant.wait(
        lambda: plant.node.pelvis is not None
        and abs(plant.node.pelvis.pose.position.x - pose[0]) < 0.05
        and abs(plant.node.pelvis.pose.position.y - pose[1]) < 0.05,
        3.0,
    )
    if not arrived:
        raise RuntimeError("plant did not reach the reset pose")
    plant.command(ACTION_ENTER_PD_STAND)
    if not plant.wait(lambda: _mode(plant.node.state) == MODE_PD_STAND, 8.0):
        raise RuntimeError(f"runner did not enter PD_STAND (state={plant.node.state})")
    plant.spin(1.0)
    plant.node.dock = None
    plant.command(ACTION_ENTER_DOCK)
    started = time.monotonic()
    fell = False
    max_tilt = 0.0
    min_height = float(plant.node.pelvis.pose.position.z)
    pose_ages: list[float] = []
    terminal = None
    while time.monotonic() - started < timeout:
        plant.spin(0.05)
        down, tilt, height_now = _fallen(plant)
        max_tilt = max(max_tilt, tilt)
        min_height = min(min_height, height_now)
        dock = plant.node.dock
        if dock is not None and len(dock) >= 15 and math.isfinite(dock[14]) and dock[14] < 1.0e8:
            pose_ages.append(float(dock[14]))
        if down:
            fell = True
            break
        if dock is not None and len(dock) >= 11 and int(dock[9]) in (2, 3):
            terminal = list(dock)
            break
    elapsed = None if terminal is None else float(terminal[8])
    result = "fell" if fell else (DOCK_RESULTS.get(int(terminal[9]), "unknown") if terminal else "timeout")
    reason = DOCK_REASONS.get(int(terminal[10]), "unknown") if terminal else "none"
    pelvis = plant.node.pelvis
    quat = np.array([
        pelvis.pose.orientation.w,
        pelvis.pose.orientation.x,
        pelvis.pose.orientation.y,
        pelvis.pose.orientation.z,
    ])
    final_yaw = _yaw(quat)
    position_error = float(np.hypot(pelvis.pose.position.x - TARGET[0], pelvis.pose.position.y - TARGET[1]))
    report = {
        "initial": {"x": float(pose[0]), "y": float(pose[1]), "yaw_deg": math.degrees(float(pose[2]))},
        "target": {"x": TARGET[0], "y": TARGET[1], "yaw_deg": 0.0},
        "final": {
            "x": float(pelvis.pose.position.x),
            "y": float(pelvis.pose.position.y),
            "yaw_deg": math.degrees(final_yaw),
        },
        "position_error_m": position_error,
        "yaw_error_deg": math.degrees(abs(_wrap(TARGET[2] - final_yaw))),
        "docked_position_error_m": None if terminal is None else float(math.hypot(terminal[4], terminal[5])),
        "docked_yaw_error_deg": None if terminal is None else math.degrees(abs(_wrap(float(terminal[6])))),
        "time_s": elapsed,
        "attempts": 0 if terminal is None else int(terminal[3]),
        "fell": fell,
        "max_tilt_rad": max_tilt,
        "min_pelvis_height_m": min_height,
        "result": result,
        "reason": reason,
        "pose_age_s_max": None if not pose_ages else max(pose_ages),
        "command_fault": bool(plant.node.state and len(plant.node.state) > 6 and plant.node.state[6] >= 0.5),
        "publishing": None if not plant.node.state or len(plant.node.state) < 5 else plant.node.state[4],
    }
    report["success"] = (not fell) and result == "done" and elapsed is not None and elapsed <= 45.0
    plant.command(ACTION_ENTER_PD_STAND)
    plant.wait(lambda: _mode(plant.node.state) == MODE_PD_STAND, 8.0)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=50.0)
    parser.add_argument("--json-out", type=Path, default=REPO / "out" / "auto_dock_hil.json")
    parser.add_argument("--round-trip-ms", type=float, default=None)
    parser.add_argument("--runner-rate-hz", type=float, default=None)
    parser.add_argument("--sync-miss", type=int, default=None)
    parser.add_argument("--halts", type=int, default=None)
    args = parser.parse_args()
    model, bindings = sim._compile_model(DEFAULT_MODEL)
    apply_hope_contact_profile(model)
    stand_q, _, _ = sim._official_stand_vectors()
    qa = np.array(bindings)[:, 0]
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    height = float(data.qpos[2])
    x_range = [TARGET[0] - 2.0, TARGET[0] + 2.0]
    y_range = [TARGET[1] - 2.0, TARGET[1] + 2.0]
    yaw_range = [math.radians(-180.0), math.radians(180.0)]
    rng = np.random.default_rng(args.seed)
    draws = []
    for _ in range(args.trials):
        trial_seed = int(rng.integers(0, 2**31 - 1))
        pose = draw_start(
            np.random.default_rng(trial_seed), model, data, qa, stand_q, x_range, y_range, yaw_range
        )
        draws.append((trial_seed, pose.copy()))
    plant = Plant()
    reports = []
    try:
        if not plant.wait(lambda: plant.node.pelvis is not None and plant.node.state is not None, 15.0):
            raise RuntimeError("pelvis pose or runner state is not arriving")
        for trial_seed, pose in draws:
            report = run_trial(plant, pose, height, sim.SDK_ORDER, stand_q, args.timeout)
            report["seed"] = trial_seed
            reports.append(report)
            print(json.dumps(report), flush=True)
    finally:
        plant.close()
    times = [item["time_s"] for item in reports if item["success"] and item["time_s"] is not None]
    summary = {
        "count": len(reports),
        "successes": sum(item["success"] for item in reports),
        "falls": sum(item["fell"] for item in reports),
        "success_rate": 0.0 if not reports else sum(item["success"] for item in reports) / len(reports),
        "time_s_p50": _percentile(times, 0.50),
        "time_s_p95": _percentile(times, 0.95),
        "docked_position_error_m_p95": _percentile(
            [item["docked_position_error_m"] for item in reports if item["success"] and item["docked_position_error_m"] is not None],
            0.95,
        ),
        "docked_yaw_error_deg_p95": _percentile(
            [item["docked_yaw_error_deg"] for item in reports if item["success"] and item["docked_yaw_error_deg"] is not None],
            0.95,
        ),
        "runner_rate_hz": args.runner_rate_hz,
        "sync_miss": args.sync_miss,
        "halts": args.halts,
        "base_pose_receipt_age_s_max": _percentile(
            [item["pose_age_s_max"] for item in reports if item["pose_age_s_max"] is not None], 1.0
        ),
        "round_trip_ms": args.round_trip_ms,
        "command_fault": any(item["command_fault"] for item in reports),
    }
    payload = {"trials": reports, "summary": summary}
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary), flush=True)
    return 0 if reports else 1


if __name__ == "__main__":
    raise SystemExit(main())
