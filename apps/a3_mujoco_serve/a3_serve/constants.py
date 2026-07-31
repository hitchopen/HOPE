"""Shared A3, CSV, and repository contracts.

The SDK CSV stores angles in degrees.  MuJoCo, the IK implementation, and the
A3 high-level ROS 2 interface use radians.  Conversions happen only at the CSV
boundary so an angle cannot silently change units inside the planner.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]

DEFAULT_MODEL_XML = (
    REPO_ROOT
    / "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/models/bin/cfg/model"
    / "a3_pingpong/a3_pingpong.xml"
)

ROOT_COLUMNS = (
    "root_translateX",
    "root_translateY",
    "root_translateZ",
    "root_rotateX",
    "root_rotateY",
    "root_rotateZ",
)

JOINT_NAMES = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

CSV_COLUMNS = ROOT_COLUMNS + JOINT_NAMES

LEFT_ARM_JOINTS = JOINT_NAMES[5:12]
RIGHT_ARM_JOINTS = JOINT_NAMES[12:19]
ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

# A3 v3 high-level motion-control limits, not the broader low-level MJCF limits.
HIGH_LEVEL_ARM_LO = np.array(
    [-2.967, -1.588, -2.793, -1.047, -0.576, -1.623, -2.793] * 2,
    dtype=np.float64,
)
HIGH_LEVEL_ARM_HI = np.array(
    [2.967, 1.588, 2.793, 2.444, 0.576, 1.623, 2.793] * 2,
    dtype=np.float64,
)

RIGHT_ARM_LO = HIGH_LEVEL_ARM_LO[7:].copy()
RIGHT_ARM_HI = HIGH_LEVEL_ARM_HI[7:].copy()

VALIDATED_SOURCE_HZ = 200.0
VALIDATED_COMMAND_HZ = 100.0
VALIDATED_FRAME_COUNT = 3878
VALIDATED_READY_FRAME = 1600
VALIDATED_STROKE_START_FRAME = 1848
VALIDATED_STRIKE_FRAME = 1860
VALIDATED_LAST_STROKE_FRAME = 3876

