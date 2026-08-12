// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "robot_io/a3_layout_extra.hpp"

// Implements the 31-DOF A3 layout (notes/a3_dof_orderings.md).
//
// This layout matches the ORDER produced by `mujoco.MjModel.njnt` iteration
// for `a3_t2d5.xml` (the authoritative source, cross-checked by the training
// side's `training_scripts/generate_a3_order_mapping.py`). A3 is actuated
// on all 31 DOFs (waist[0..2], neck[3..4], arms[5..18], legs[19..30]), so
// there are no virtual slots here — the layout IS the full MuJoCo DOF order.
//
// ⚠️ This is NOT the same order as A3's 29-DOF flat layout. A3's "legs →
// waist → arms" grouping happens to coincide with Unitree SDK motor ordering
// and MuJoCo for A3; A3 has no such coincidence. See notes/a3_dof_orderings.md
// for the full discussion.

namespace robot_io {

const JointLayout& MakeA3Layout31() {
  static const JointLayout layout{{
      // waist [0..2]
      "waist_yaw_joint",
      "waist_roll_joint",
      "waist_pitch_joint",
      // neck  [3..4] — excluded from the 29-DOF policy view, kept at
      //                position 0 by deploy-time PD command.
      "head_yaw_joint",
      "head_pitch_joint",
      // left arm [5..11]
      "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",
      "left_elbow_joint",
      "left_wrist_roll_joint",
      "left_wrist_pitch_joint",
      "left_wrist_yaw_joint",
      // right arm [12..18]
      "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint",
      "right_elbow_joint",
      "right_wrist_roll_joint",
      "right_wrist_pitch_joint",
      "right_wrist_yaw_joint",
      // left leg [19..24]
      "left_hip_pitch_joint",
      "left_hip_roll_joint",
      "left_hip_yaw_joint",
      "left_knee_joint",
      "left_ankle_pitch_joint",
      "left_ankle_roll_joint",
      // right leg [25..30]
      "right_hip_pitch_joint",
      "right_hip_roll_joint",
      "right_hip_yaw_joint",
      "right_knee_joint",
      "right_ankle_pitch_joint",
      "right_ankle_roll_joint",
  }};
  return layout;
}

// Mapping between the A3 neck group's sub-index (0 or 1 within neck_sample)
// and the 31-DOF layout's neck slots [3..4]. Currently identity; kept
// explicit so a future neck topic reshuffle only needs edits here.
const std::array<int, kA3NeckCount> kFlatToA3NeckIdx = {0, 1};
const std::array<int, kA3NeckCount> kA3NeckToFlatIdx = {0, 1};

// Neck joint names in the A3 topic's internal order (same as MuJoCo slots
// [3..4]).
const std::array<std::string, kA3NeckCount> kA3NeckJointNames = {
    "head_yaw_joint",
    "head_pitch_joint",
};

}  // namespace robot_io
