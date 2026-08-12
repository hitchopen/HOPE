// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
//
// sensor_msgs::msg::Imu subscriber for /body_drive/pelvis_imu/data. Quat is
// converted from the ROS2 XYZW convention to the a3_sync WXYZ convention
// inside OnMessage / ConvertMessage.
#pragma once

#include "a3_io/subscriber_base.hpp"
#include "a3_sync/a3_ring_buffer.hpp"
#include "a3_sync/a3_sync_types.hpp"

#include <cstdint>
#include <memory>

#ifdef HAS_A3_ROS_MSGS
#include "sensor_msgs/msg/imu.hpp"
#endif

namespace a3_io {

class PelvisImuSub : public SubscriberBase<a3_sync::ImuSample> {
 public:
  explicit PelvisImuSub(
      a3_sync::RingBuffer<a3_sync::ImuSample>& ring,
      SampleCallback sample_cb = {})
      : SubscriberBase(std::move(sample_cb)), ring_(ring) {}

  // Pure conversion. Inputs mirror sensor_msgs::msg::Imu field layout.
  // quat_xyzw is in ROS2 order (x, y, z, w); gyro = angular_velocity
  // (xyz), acc = linear_acceleration (xyz). stamp_ns from header.stamp.
  // Always succeeds — an Imu message is always well-formed (no joint
  // name matching needed).
  static void ConvertMessage(double qx, double qy, double qz, double qw,
                             double gx, double gy, double gz,
                             double ax, double ay, double az,
                             std::int64_t stamp_ns,
                             a3_sync::ImuSample& out);

#ifdef HAS_A3_ROS_MSGS
  void OnMessage(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg);
#endif

 private:
  a3_sync::RingBuffer<a3_sync::ImuSample>& ring_;
};

}  // namespace a3_io
