// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 6
#include "a3_io/publishers/neck_cmd_pub.hpp"

#include "a3_io/publish_helpers.hpp"

namespace a3_io {

#ifdef HAS_A3_ROS_MSGS

void NeckCmdPub::BuildMessage(const robot_io::RobotCommand& cmd_31,
                              std::int64_t stamp_ns, std::uint32_t seq,
                              joint_msgs::msg::JointCommand& out) {
  FillJointCommandMsg<kDof>(cmd_31, robot_io::kA3NeckStart,
                            robot_io::kA3NeckJointNames, stamp_ns, seq, out);
}

void NeckCmdPub::Publish(std::int64_t stamp_ns, std::uint32_t seq,
                         const robot_io::RobotCommand& cmd_31) {
  if (!publish_fn_) return;
  joint_msgs::msg::JointCommand msg;
  BuildMessage(cmd_31, stamp_ns, seq, msg);
  publish_fn_(msg);
}

#endif  // HAS_A3_ROS_MSGS

}  // namespace a3_io
