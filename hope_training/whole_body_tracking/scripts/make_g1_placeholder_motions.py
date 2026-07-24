#!/usr/bin/env python3
"""Generate G1 PLACEHOLDER forehand/backhand motion clips for the HOPE task.

The HOPE task needs reference clips whose ``joint_pos`` is 29-wide (G1 canonical order) and whose
14 tracked-body world poses are kinematically consistent with those joint angles. The shipped A3
placeholders are 31-DOF and cannot be reused, so this script synthesizes G1-shaped clips.

It is deliberately dependency-light — numpy + the standard-library XML parser only, NO Isaac Lab —
so you can regenerate the clips anywhere. It implements a small URDF forward-kinematics pass over
the G1 URDF to place the tracked bodies, then writes the npz schema ``MotionLoader`` expects:

    fps            : scalar
    joint_pos      : float32 [T, 29]   (G1 canonical / native joint order)
    joint_vel      : float32 [T, 29]
    body_pos_w     : float32 [T, 14, 3]
    body_quat_w    : float32 [T, 14, 4]  (w, x, y, z)
    body_lin_vel_w : float32 [T, 14, 3]
    body_ang_vel_w : float32 [T, 14, 3]

These are NON-PERFORMANT placeholders (a gentle synthetic arm swing about the ready stance) that
only exist so training imports and shape checks pass. Replace them with real retargeted G1 swings
before training a policy you intend to deploy (see docs/REPLACE_MOTIONS.md).

Usage (from the repo root):
    python3 hope_training/whole_body_tracking/scripts/make_g1_placeholder_motions.py
    python3 .../make_g1_placeholder_motions.py --urdf /path/to/g1.urdf --out-dir /path/to/preprocessed
"""

from __future__ import annotations

import argparse
import os
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

# G1 canonical (Isaac-native) joint order — must match joint_order_unitree_g1.yaml / robots/g1.py.
G1_JOINT_NAMES = [
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
]

# 14 tracked bodies (must match robots/g1.py G1_TRACKED_BODIES, in this order).
G1_TRACKED_BODIES = [
    "pelvis",
    "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
    "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link",
    "torso_link",
    "left_shoulder_roll_link", "left_elbow_link", "left_wrist_yaw_link",
    "right_shoulder_roll_link", "right_elbow_link", "right_wrist_yaw_link",
]

# Ready-stance default pose (matches robots/g1.py init_state and the shared action_adapter.yaml).
DEFAULT_POSE = {
    "left_hip_pitch_joint": -0.15, "right_hip_pitch_joint": -0.15,
    "left_knee_joint": 0.30, "right_knee_joint": 0.30,
    "left_ankle_pitch_joint": -0.15, "right_ankle_pitch_joint": -0.15,
    "left_shoulder_pitch_joint": 0.20, "left_shoulder_roll_joint": 0.30, "left_elbow_joint": 0.50,
    "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": -0.10, "right_elbow_joint": 0.50,
}

PELVIS_HEIGHT = 0.79
FPS = 50.0
NUM_FRAMES = 80

_DEFAULT_URDF = (
    "/home/TTRL-ICRA2026/legged_lab/assets/unitree_description/urdf/g1/"
    "g1_with_racket_adapter_short_ball_throwing.urdf"
)


# --------------------------------------------------------------------------------------------------
# Minimal URDF forward kinematics (numpy).
# --------------------------------------------------------------------------------------------------
def _rpy_to_mat(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx  # URDF fixed-axis rpy = Rz*Ry*Rx


def _axis_angle_to_mat(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.eye(3)
    x, y, z = axis / n
    c, s, C = np.cos(angle), np.sin(angle), 1.0 - np.cos(angle)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _mat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> (w, x, y, z)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


class UrdfFK:
    """Forward kinematics over a URDF's revolute/fixed joint tree (root placed at PELVIS_HEIGHT)."""

    def __init__(self, urdf_path: str):
        root = ET.parse(urdf_path).getroot()
        self.links = {ln.get("name") for ln in root.findall("link")}
        self.child_joint = {}  # child_link -> joint dict
        for j in root.findall("joint"):
            child = j.find("child").get("link")
            parent = j.find("parent").get("link")
            origin = j.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0").split() if origin is not None else [0, 0, 0])]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0").split() if origin is not None else [0, 0, 0])]
            axis_el = j.find("axis")
            axis = [float(v) for v in (axis_el.get("xyz", "1 0 0").split() if axis_el is not None else [1, 0, 0])]
            self.child_joint[child] = {
                "name": j.get("name"), "parent": parent, "type": j.get("type"),
                "T_origin": self._make_T(xyz, rpy), "axis": np.array(axis),
            }
        # Root link = the one that is never a child.
        children = set(self.child_joint)
        roots = [ln for ln in self.links if ln not in children]
        self.root = "pelvis" if "pelvis" in roots else roots[0]

    @staticmethod
    def _make_T(xyz, rpy) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = _rpy_to_mat(*rpy)
        T[:3, 3] = xyz
        return T

    def link_pose(self, link: str, angles: dict) -> np.ndarray:
        """World 4x4 transform of ``link`` given a {joint_name: angle} dict."""
        T = np.eye(4)
        T[:3, 3] = [0.0, 0.0, PELVIS_HEIGHT]  # place the root (pelvis) in the world
        chain = []
        cur = link
        while cur in self.child_joint:
            j = self.child_joint[cur]
            chain.append(j)
            cur = j["parent"]
        for j in reversed(chain):
            Tj = j["T_origin"].copy()
            if j["type"] in ("revolute", "continuous", "prismatic"):
                ang = float(angles.get(j["name"], 0.0))
                if j["type"] == "prismatic":
                    Tm = np.eye(4)
                    Tm[:3, 3] = j["axis"] / (np.linalg.norm(j["axis"]) + 1e-9) * ang
                else:
                    Tm = np.eye(4)
                    Tm[:3, :3] = _axis_angle_to_mat(j["axis"], ang)
                Tj = Tj @ Tm
            T = T @ Tj
        return T


# --------------------------------------------------------------------------------------------------
# Clip synthesis.
# --------------------------------------------------------------------------------------------------
def _swing_trajectory(side: str) -> np.ndarray:
    """[T, 29] joint trajectory: default ready stance + a gentle right-arm swing."""
    q = np.tile(np.array([DEFAULT_POSE.get(n, 0.0) for n in G1_JOINT_NAMES], dtype=np.float64),
                (NUM_FRAMES, 1))
    idx = {n: i for i, n in enumerate(G1_JOINT_NAMES)}
    phase = np.sin(np.linspace(0.0, np.pi, NUM_FRAMES))  # 0 -> 1 -> 0 (windup..strike..follow)
    sign = 1.0 if side == "forehand" else -1.0
    # Small, safe amplitudes on the paddle (right) arm; opposite face for backhand.
    q[:, idx["right_shoulder_pitch_joint"]] += sign * 0.35 * phase
    q[:, idx["right_shoulder_roll_joint"]] += -0.30 * phase
    q[:, idx["right_elbow_joint"]] += 0.30 * phase
    q[:, idx["right_wrist_pitch_joint"]] += sign * 0.20 * phase
    q[:, idx["waist_yaw_joint"]] += sign * 0.12 * phase
    return q


def _angular_velocity(R0: np.ndarray, R1: np.ndarray, dt: float) -> np.ndarray:
    """World-frame angular velocity from consecutive rotation matrices."""
    dR = R1 @ R0.T
    ang = np.arccos(np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0))
    if ang < 1e-8:
        return np.zeros(3)
    axis = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
    axis = axis / (2.0 * np.sin(ang))
    return axis * (ang / dt)


def build_clip(fk: UrdfFK, side: str) -> dict:
    dt = 1.0 / FPS
    joint_pos = _swing_trajectory(side).astype(np.float64)
    joint_vel = np.gradient(joint_pos, dt, axis=0)

    T, B = NUM_FRAMES, len(G1_TRACKED_BODIES)
    body_pos = np.zeros((T, B, 3))
    body_quat = np.zeros((T, B, 4))
    body_R = np.zeros((T, B, 3, 3))
    for t in range(T):
        angles = {n: joint_pos[t, i] for i, n in enumerate(G1_JOINT_NAMES)}
        for b, name in enumerate(G1_TRACKED_BODIES):
            M = fk.link_pose(name, angles)
            body_pos[t, b] = M[:3, 3]
            body_R[t, b] = M[:3, :3]
            body_quat[t, b] = _mat_to_quat(M[:3, :3])

    body_lin_vel = np.gradient(body_pos, dt, axis=0)
    body_ang_vel = np.zeros((T, B, 3))
    for b in range(B):
        for t in range(T):
            t0, t1 = max(0, t - 1), min(T - 1, t + 1)
            span = (t1 - t0) * dt if t1 > t0 else dt
            body_ang_vel[t, b] = _angular_velocity(body_R[t0, b], body_R[t1, b], span)

    return {
        "fps": np.float32(FPS),
        "joint_pos": joint_pos.astype(np.float32),
        "joint_vel": joint_vel.astype(np.float32),
        "body_pos_w": body_pos.astype(np.float32),
        "body_quat_w": body_quat.astype(np.float32),
        "body_lin_vel_w": body_lin_vel.astype(np.float32),
        "body_ang_vel_w": body_ang_vel.astype(np.float32),
    }


def _default_out_dir() -> pathlib.Path:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "hope_training" / "motions" / "preprocessed"
        if cand.parent.parent.is_dir():
            return cand
    return here.parents[3] / "hope_training" / "motions" / "preprocessed"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=_DEFAULT_URDF, help="G1 URDF used for forward kinematics.")
    ap.add_argument("--out-dir", type=pathlib.Path, default=_default_out_dir(),
                    help="Where to write hope_g1_forehand.npz / hope_g1_backhand.npz.")
    args = ap.parse_args()

    if not os.path.isfile(args.urdf):
        raise FileNotFoundError(f"URDF not found: {args.urdf} (pass --urdf).")
    fk = UrdfFK(args.urdf)
    missing = [b for b in G1_TRACKED_BODIES if b not in fk.links]
    if missing:
        raise ValueError(f"URDF is missing tracked bodies: {missing}")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for side, fname in (("forehand", "hope_g1_forehand.npz"), ("backhand", "hope_g1_backhand.npz")):
        clip = build_clip(fk, side)
        out = args.out_dir / fname
        np.savez(out, **clip)
        print(f"[make_g1_placeholder_motions] wrote {out}  "
              f"(joint_pos {clip['joint_pos'].shape}, bodies {clip['body_pos_w'].shape[1]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
