// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 6
#pragma once

#include <cstdint>
#include <functional>

#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/layouts.hpp"
#include "robot_io/robot_io_backend.hpp"

#ifdef HAS_A3_ROS_MSGS
#include "joint_msgs/msg/joint_command.hpp"
#endif

namespace a3_io {

// 3-DOF waist command publisher.
// Source slice: RobotCommand.[q|dq|tau|kp|kd][0..2] (kA3WaistStart=0) in
// MuJoCo real 31-DOF SDK layout.
class WaistCmdPub {
 public:
  static constexpr std::size_t kDof = robot_io::kA3WaistCount;  // 3

#ifdef HAS_A3_ROS_MSGS
  using PublishFn = std::function<void(const joint_msgs::msg::JointCommand&)>;

  WaistCmdPub() = default;
  explicit WaistCmdPub(PublishFn publish) : publish_fn_(std::move(publish)) {}

  // Build a JointCommand message from cmd_31 and invoke the injected
  // callback. No-op if publish_fn_ is empty.
  void Publish(std::int64_t stamp_ns, std::uint32_t seq,
               const robot_io::RobotCommand& cmd_31);

  // Pure helper — fills `out` in-place.
  static void BuildMessage(const robot_io::RobotCommand& cmd_31,
                           std::int64_t stamp_ns, std::uint32_t seq,
                           joint_msgs::msg::JointCommand& out);
#endif

 private:
#ifdef HAS_A3_ROS_MSGS
  PublishFn publish_fn_;
#endif
};

}  // namespace a3_io
