// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 6
//
// Shared pure-function helpers used by the 4 A3 JointCommand publishers.
// Mirrors the role of a3_io/convert_helpers.hpp on the subscriber side.
//
// - BuildJointCmdRow<DOF> is always available; it extracts a single joint
//   row {name, sequence, pos, vel, eff, kp, kd} from a 31-DOF RobotCommand
//   slice. This allows unit tests to verify per-joint packing *without*
//   linking against any ROS2 message type.
//
// - FillJointCommandMsg<DOF> is only available when HAS_A3_ROS_MSGS is
//   defined; it populates a joint_msgs::msg::JointCommand with all DOF
//   rows and a std_msgs::msg::Header stamp converted from ns.

#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "robot_io/robot_io_backend.hpp"

#ifdef HAS_A3_ROS_MSGS
#include "joint_msgs/msg/joint_command.hpp"
#endif

namespace a3_io {

// Plain, ROS-free per-joint row used for unit testing and as an
// intermediate form. Layout matches joint_msgs::msg::Command
// (name, sequence, position, velocity, effort, stiffness, damping).
struct JointCmdRow {
  std::string name;
  std::uint32_t sequence{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double stiffness{0.0};
  double damping{0.0};
};

// Extract a single joint row from a 31-DOF RobotCommand.
//
//   flat_start_idx — starting index of the group inside the 31-DOF flat
//                    layout (e.g. kArmStart=15 for the arm group).
//   sub_idx        — 0-based offset within the group ([0, DOF)).
//   joint_names    — canonical A3 name table for the group.
//   seq            — caller-supplied monotonic sequence number, copied
//                    into every row.
//
// Throws std::out_of_range on sub_idx >= DOF or if the RobotCommand
// vectors do not have at least flat_start_idx + DOF elements. This is a
// programming error; the caller (a specific *CmdPub) is responsible for
// passing a well-sized RobotCommand and a valid sub_idx.
template <std::size_t DOF>
JointCmdRow BuildJointCmdRow(const robot_io::RobotCommand& cmd_31,
                             int flat_start_idx,
                             int sub_idx,
                             const std::array<std::string, DOF>& joint_names,
                             std::uint32_t seq) {
  if (sub_idx < 0 || static_cast<std::size_t>(sub_idx) >= DOF) {
    throw std::out_of_range("BuildJointCmdRow: sub_idx out of range");
  }
  const int flat = flat_start_idx + sub_idx;
  if (cmd_31.q_des.size() < flat + 1 || cmd_31.dq_des.size() < flat + 1 ||
      cmd_31.tau_ff.size() < flat + 1 || cmd_31.kp.size() < flat + 1 ||
      cmd_31.kd.size() < flat + 1) {
    throw std::out_of_range(
        "BuildJointCmdRow: RobotCommand vector too short for flat index");
  }

  JointCmdRow row;
  row.name      = joint_names[sub_idx];
  row.sequence  = seq;
  row.position  = cmd_31.q_des[flat];
  row.velocity  = cmd_31.dq_des[flat];
  row.effort    = cmd_31.tau_ff[flat];
  row.stiffness = cmd_31.kp[flat];
  row.damping   = cmd_31.kd[flat];
  return row;
}

#ifdef HAS_A3_ROS_MSGS
// Populate a joint_msgs::msg::JointCommand with DOF entries from a
// 31-DOF RobotCommand slice. Overwrites `out.joints` (resizes to DOF)
// and sets `out.header.stamp` from stamp_ns.
template <std::size_t DOF>
void FillJointCommandMsg(const robot_io::RobotCommand& cmd_31,
                         int flat_start_idx,
                         const std::array<std::string, DOF>& joint_names,
                         std::int64_t stamp_ns,
                         std::uint32_t seq,
                         joint_msgs::msg::JointCommand& out) {
  out.header.stamp.sec =
      static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  out.header.stamp.nanosec =
      static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);

  out.joints.clear();
  out.joints.resize(DOF);
  for (std::size_t i = 0; i < DOF; ++i) {
    const JointCmdRow row =
        BuildJointCmdRow<DOF>(cmd_31, flat_start_idx, static_cast<int>(i),
                              joint_names, seq);
    auto& j = out.joints[i];
    j.name      = row.name;
    j.sequence  = row.sequence;
    j.position  = row.position;
    j.velocity  = row.velocity;
    j.effort    = row.effort;
    j.stiffness = row.stiffness;
    j.damping   = row.damping;
  }
}
#endif  // HAS_A3_ROS_MSGS

}  // namespace a3_io
