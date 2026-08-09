// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#pragma once

#include "robot_io/robot_io_backend.hpp"

#include <array>
#include <string>

namespace robot_io {

// Flat 29-DOF layout shared by A3 and A3 backends.
//
// Order (matches PolicyJointIndex in
// src/a3/a3_deploy_onnx_ref/include/robot_parameters.hpp):
//   [0..11]   legs  (left 6 + right 6)
//   [12..14]  waist (yaw, roll, pitch)
//   [15..28]  arms  (left 7 + right 7)

constexpr int kDof        = 29;
constexpr int kLegStart   = 0;
constexpr int kLegCount   = 12;
constexpr int kWaistStart = 12;
constexpr int kWaistCount = 3;
constexpr int kArmStart   = 15;
constexpr int kArmCount   = 14;

// Layout accessors. Both return the same 29 joint names in the same order.
// Kept as separate functions so the two layouts can diverge later without
// touching the interface.
const JointLayout& MakeFlatPolicyLayout();
const JointLayout& MakeA3Layout();

// A3 backend topic-group mappings.
//
// A3 publishes / subscribes its joints on three separate topics (see
// ~/aimde/mujoco_sim/src/models/bin/cfg/a3_t2d5_cfg.yaml):
//   /body_drive/waist_joint_{state,command}  (3 DOF)
//   /body_drive/leg_joint_{state,command}    (12 DOF)
//   /body_drive/arm_joint_{state,command}    (14 DOF)
//
// Each topic has its own internal joint order. The tables below describe
// how the flat 29-DOF layout's group sub-indices map to the A3 topic's
// internal indices.
//   kPolicyToA3WaistIdx[i] == j  ⇒  flat layout sub-index `i` within the waist
//                               group (i.e. flat index kWaistStart + i) lands
//                               at position `j` inside the A3 waist topic.
// Reverse tables kA3{Waist,Leg,Arm}ToA3Idx do the opposite direction.
extern const std::array<int, kWaistCount> kPolicyToA3WaistIdx;
extern const std::array<int, kLegCount>   kPolicyToA3LegIdx;
extern const std::array<int, kArmCount>   kPolicyToA3ArmIdx;

extern const std::array<int, kWaistCount> kA3WaistToPolicyIdx;
extern const std::array<int, kLegCount>   kA3LegToPolicyIdx;
extern const std::array<int, kArmCount>   kA3ArmToPolicyIdx;

// A3 joint names in A3-native topic order.
// Used when the A3 backend populates joint_msgs::msg::JointCommand.name
// and when matching joint_msgs::msg::JointState entries by name.
extern const std::array<std::string, kWaistCount> kA3WaistJointNames;
extern const std::array<std::string, kLegCount>   kA3LegJointNames;
extern const std::array<std::string, kArmCount>   kA3ArmJointNames;

}  // namespace robot_io
