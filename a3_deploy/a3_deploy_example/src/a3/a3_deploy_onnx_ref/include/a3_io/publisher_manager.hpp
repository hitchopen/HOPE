// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 6
//
// A3PublisherManager owns the 4 concrete JointCommand publishers
// (waist/leg/arm/neck) and fans out a single 31-DOF robot_io::RobotCommand
// to all 4 topics. Like A3SubscriberManager, AimRT channel registration is
// deferred to PR 7 — here the publish path is abstracted behind injected
// std::function<void(...)> callbacks.

#pragma once

#include <cstdint>

#include "a3_io/publishers/arm_cmd_pub.hpp"
#include "a3_io/publishers/leg_cmd_pub.hpp"
#include "a3_io/publishers/neck_cmd_pub.hpp"
#include "a3_io/publishers/waist_cmd_pub.hpp"
#include "robot_io/robot_io_backend.hpp"

namespace a3_io {

class A3PublisherManager {
 public:
#ifdef HAS_A3_ROS_MSGS
  struct Options {
    WaistCmdPub::PublishFn waist_publish_fn;
    LegCmdPub::PublishFn   leg_publish_fn;
    ArmCmdPub::PublishFn   arm_publish_fn;
    NeckCmdPub::PublishFn  neck_publish_fn;
  };

  explicit A3PublisherManager(Options opt);

  // Publish all 4 command topics from a single 31-DOF RobotCommand.
  // stamp_ns typically = source RobotState.timestamp_ns; seq is a
  // monotonically increasing counter managed by the caller (backend).
  void PublishAll(std::int64_t stamp_ns, std::uint32_t seq,
                  const robot_io::RobotCommand& cmd_31);

  WaistCmdPub& waist() { return waist_; }
  LegCmdPub&   leg()   { return leg_; }
  ArmCmdPub&   arm()   { return arm_; }
  NeckCmdPub&  neck()  { return neck_; }
#endif

 private:
#ifdef HAS_A3_ROS_MSGS
  WaistCmdPub waist_;
  LegCmdPub   leg_;
  ArmCmdPub   arm_;
  NeckCmdPub  neck_;
#endif
};

}  // namespace a3_io
