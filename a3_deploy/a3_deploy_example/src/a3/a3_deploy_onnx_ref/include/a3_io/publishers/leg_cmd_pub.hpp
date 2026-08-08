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

// 12-DOF leg command publisher.
// Source slice: RobotCommand.[...][19..30] (kA3LegStart=19) in MuJoCo real
// 31-DOF SDK layout.
class LegCmdPub {
 public:
  static constexpr std::size_t kDof = robot_io::kA3LegCount;  // 12

#ifdef HAS_A3_ROS_MSGS
  using PublishFn = std::function<void(const joint_msgs::msg::JointCommand&)>;

  LegCmdPub() = default;
  explicit LegCmdPub(PublishFn publish) : publish_fn_(std::move(publish)) {}

  void Publish(std::int64_t stamp_ns, std::uint32_t seq,
               const robot_io::RobotCommand& cmd_31);

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
