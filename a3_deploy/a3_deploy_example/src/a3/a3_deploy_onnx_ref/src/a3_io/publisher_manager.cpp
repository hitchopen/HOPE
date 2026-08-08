// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 6
#include "a3_io/publisher_manager.hpp"

namespace a3_io {

#ifdef HAS_A3_ROS_MSGS

A3PublisherManager::A3PublisherManager(Options opt)
    : waist_(std::move(opt.waist_publish_fn)),
      leg_(std::move(opt.leg_publish_fn)),
      arm_(std::move(opt.arm_publish_fn)),
      neck_(std::move(opt.neck_publish_fn)) {}

void A3PublisherManager::PublishAll(std::int64_t stamp_ns, std::uint32_t seq,
                                    const robot_io::RobotCommand& cmd_31) {
  waist_.Publish(stamp_ns, seq, cmd_31);
  leg_.Publish(stamp_ns, seq, cmd_31);
  arm_.Publish(stamp_ns, seq, cmd_31);
  neck_.Publish(stamp_ns, seq, cmd_31);
}

#endif  // HAS_A3_ROS_MSGS

}  // namespace a3_io
