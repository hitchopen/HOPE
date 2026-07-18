# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""Canonical Agibot A3 joint order (31 controllable DOF).

This is the single joint order shared by training, the exported ONNX policy, this
reference runner, and the planner. The observation ``joint_pos``/``joint_vel``
slices, the ONNX ``raw_action[31]`` output, and the joint-position targets written
to the robot all use this exact index order.

The racket is mounted on the right wrist. ``head_yaw`` / ``head_pitch`` (indices
3 and 4) are passive at deploy (held at their default angle) but still occupy their
action columns, so the vector is always length 31.
"""

from __future__ import annotations

JOINT_NAMES: tuple[str, ...] = (
    "waist_yaw_joint",          # 0
    "waist_roll_joint",         # 1
    "waist_pitch_joint",        # 2
    "head_yaw_joint",           # 3  passive at deploy (held at default)
    "head_pitch_joint",         # 4  passive at deploy (held at default)
    "left_shoulder_pitch_joint",   # 5
    "left_shoulder_roll_joint",    # 6
    "left_shoulder_yaw_joint",     # 7
    "left_elbow_joint",            # 8
    "left_wrist_roll_joint",       # 9
    "left_wrist_pitch_joint",      # 10
    "left_wrist_yaw_joint",        # 11
    "right_shoulder_pitch_joint",  # 12
    "right_shoulder_roll_joint",   # 13
    "right_shoulder_yaw_joint",    # 14
    "right_elbow_joint",           # 15
    "right_wrist_roll_joint",      # 16
    "right_wrist_pitch_joint",     # 17
    "right_wrist_yaw_joint",       # 18
    "left_hip_pitch_joint",     # 19
    "left_hip_roll_joint",      # 20
    "left_hip_yaw_joint",       # 21
    "left_knee_joint",          # 22
    "left_ankle_pitch_joint",   # 23
    "left_ankle_roll_joint",    # 24
    "right_hip_pitch_joint",    # 25
    "right_hip_roll_joint",     # 26
    "right_hip_yaw_joint",      # 27
    "right_knee_joint",         # 28
    "right_ankle_pitch_joint",  # 29
    "right_ankle_roll_joint",   # 30
)

NUM_JOINTS: int = 31

# Passive-at-deploy neck columns (still present in the 31-D action vector).
HEAD_INDICES: tuple[int, ...] = (3, 4)

NAME_TO_INDEX: dict[str, int] = {name: i for i, name in enumerate(JOINT_NAMES)}

assert len(JOINT_NAMES) == NUM_JOINTS, "joint order must contain exactly 31 names"
