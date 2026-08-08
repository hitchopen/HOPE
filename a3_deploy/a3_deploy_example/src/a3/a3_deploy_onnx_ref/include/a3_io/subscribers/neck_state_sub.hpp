// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
#pragma once

#include "a3_io/subscriber_base.hpp"
#include "a3_sync/a3_ring_buffer.hpp"
#include "a3_sync/a3_sync_types.hpp"
#include "robot_io/a3_layout_extra.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#ifdef HAS_A3_ROS_MSGS
#include "joint_msgs/msg/joint_state.hpp"
#endif

namespace a3_io {

class NeckStateSub : public SubscriberBase<a3_sync::NeckSample> {
 public:
  static constexpr std::size_t kDof = robot_io::kA3NeckCount;  // 2

  explicit NeckStateSub(
      a3_sync::RingBuffer<a3_sync::NeckSample>& ring,
      SampleCallback sample_cb = {})
      : SubscriberBase(std::move(sample_cb)), ring_(ring) {}

  static bool ConvertMessage(const std::array<std::string, kDof>& names_in,
                             const std::array<double, kDof>& pos_in,
                             const std::array<double, kDof>& vel_in,
                             const std::array<double, kDof>& eff_in,
                             std::int64_t stamp_ns,
                             a3_sync::NeckSample& out);

#ifdef HAS_A3_ROS_MSGS
  void OnMessage(const std::shared_ptr<const joint_msgs::msg::JointState>& msg);
#endif

 private:
  a3_sync::RingBuffer<a3_sync::NeckSample>& ring_;
};

}  // namespace a3_io
