// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "robot_io/layouts.hpp"

namespace robot_io {

namespace {

// The 29 flat-layout joint names, shared by A3 and A3.
// Order follows PolicyJointIndex in
// src/a3/a3_deploy_onnx_ref/include/robot_parameters.hpp:
//   legs (12) → waist (3) → arms (14)
const JointLayout& FlatLayout() {
  static const JointLayout layout{{
      // legs [0..11]
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
      // waist [12..14]
      "waist_yaw_joint",
      "waist_roll_joint",
      "waist_pitch_joint",
      // arms [15..28]
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
  }};
  return layout;
}

}  // namespace

const JointLayout& MakeFlatPolicyLayout() { return FlatLayout(); }
const JointLayout& MakeA3Layout() { return FlatLayout(); }

// ---------------------------------------------------------------------------
// A3 ↔ A3 group mappings.
//
// The flat 29-DOF layout groups joints as legs(12) + waist(3) + arms(14).
// The A3 topics use the same internal order within each group (verified
// against ~/aimde/mujoco_sim/src/models/bin/cfg/a3_t2d5_cfg.yaml):
//   waist topic: waist_yaw_joint, waist_roll_joint, waist_pitch_joint
//   leg   topic: left_hip_pitch, …, left_ankle_roll, right_hip_pitch, …,
//                right_ankle_roll
//   arm   topic: left_shoulder_pitch, …, left_wrist_yaw,
//                right_shoulder_pitch, …, right_wrist_yaw
// Therefore the flat-layout sub-index within each group equals the A3-topic
// index within the same group: the mappings below are identity. They are
// declared explicitly so that any future divergence (e.g. an A3 revision
// re-orders the waist topic) only needs to be edited in this file.
// ---------------------------------------------------------------------------
const std::array<int, kWaistCount> kPolicyToA3WaistIdx = {0, 1, 2};
const std::array<int, kLegCount>   kPolicyToA3LegIdx   = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const std::array<int, kArmCount>   kPolicyToA3ArmIdx   = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

const std::array<int, kWaistCount> kA3WaistToPolicyIdx = {0, 1, 2};
const std::array<int, kLegCount>   kA3LegToPolicyIdx   = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const std::array<int, kArmCount>   kA3ArmToPolicyIdx   = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

const std::array<std::string, kWaistCount> kA3WaistJointNames = {
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
};

const std::array<std::string, kLegCount> kA3LegJointNames = {
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
};

const std::array<std::string, kArmCount> kA3ArmJointNames = {
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
};

}  // namespace robot_io
