# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""Canonical Unitree G1 joint order (29 controllable DOF).

This is the single joint order shared by training, the exported ONNX policy, this
reference runner, and the planner. The observation ``joint_pos``/``joint_vel``
slices, the ONNX ``raw_action[29]`` output, and the joint-position targets written
to the robot all use this exact index order.

It is the Isaac-native articulation enumeration of the G1 racket USD (breadth-first
by kinematic-tree depth, left/right interleaved with the waist chain), matching
``hope_training/config/joint_order_unitree_g1.yaml``. The racket is mounted on the
right wrist. The G1 has NO head/neck joints, so — unlike the A3 — there are no
passive-at-deploy columns (``HEAD_INDICES`` is empty).
"""

from __future__ import annotations

JOINT_NAMES: tuple[str, ...] = (
    "left_hip_pitch_joint",       # 0
    "right_hip_pitch_joint",      # 1
    "waist_yaw_joint",            # 2
    "left_hip_roll_joint",        # 3
    "right_hip_roll_joint",       # 4
    "waist_roll_joint",           # 5
    "left_hip_yaw_joint",         # 6
    "right_hip_yaw_joint",        # 7
    "waist_pitch_joint",          # 8
    "left_knee_joint",            # 9
    "right_knee_joint",           # 10
    "left_shoulder_pitch_joint",  # 11
    "right_shoulder_pitch_joint", # 12
    "left_ankle_pitch_joint",     # 13
    "right_ankle_pitch_joint",    # 14
    "left_shoulder_roll_joint",   # 15
    "right_shoulder_roll_joint",  # 16
    "left_ankle_roll_joint",      # 17
    "right_ankle_roll_joint",     # 18
    "left_shoulder_yaw_joint",    # 19
    "right_shoulder_yaw_joint",   # 20
    "left_elbow_joint",           # 21
    "right_elbow_joint",          # 22
    "left_wrist_roll_joint",      # 23
    "right_wrist_roll_joint",     # 24
    "left_wrist_pitch_joint",     # 25
    "right_wrist_pitch_joint",    # 26
    "left_wrist_yaw_joint",       # 27
    "right_wrist_yaw_joint",      # 28
)

NUM_JOINTS: int = 29

# The G1 has no passive neck: no columns are held at their default. Kept (empty) so the runner's
# passive-column handling is a no-op with zero code change relative to the A3 runner.
HEAD_INDICES: tuple[int, ...] = ()

NAME_TO_INDEX: dict[str, int] = {name: i for i, name in enumerate(JOINT_NAMES)}

assert len(JOINT_NAMES) == NUM_JOINTS, "joint order must contain exactly 29 names"
