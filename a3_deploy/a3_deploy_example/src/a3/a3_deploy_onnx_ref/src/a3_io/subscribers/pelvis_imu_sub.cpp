// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
#include "a3_io/subscribers/pelvis_imu_sub.hpp"

#include "a3_io/convert_helpers.hpp"

namespace a3_io {

void PelvisImuSub::ConvertMessage(double qx, double qy, double qz, double qw,
                                  double gx, double gy, double gz,
                                  double ax, double ay, double az,
                                  std::int64_t stamp_ns,
                                  a3_sync::ImuSample& out) {
  out.quat_wxyz = QuatXyzwToWxyz(qx, qy, qz, qw);
  out.gyro      = {gx, gy, gz};
  out.acc       = {ax, ay, az};
  out.stamp.value = stamp_ns;
}

#ifdef HAS_A3_ROS_MSGS
void PelvisImuSub::OnMessage(
    const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
  if (!msg) return;

  const std::int64_t stamp_ns =
      StampToNs(msg->header.stamp.sec, msg->header.stamp.nanosec);

  a3_sync::ImuSample sample{};
  ConvertMessage(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                 msg->orientation.w,
                 msg->angular_velocity.x, msg->angular_velocity.y,
                 msg->angular_velocity.z,
                 msg->linear_acceleration.x, msg->linear_acceleration.y,
                 msg->linear_acceleration.z,
                 stamp_ns, sample);
  StampReceiptTime(sample);
  ring_.write([&](a3_sync::ImuSample& slot) { slot = sample; });
  MarkReady();
  NotifySample(sample);
}
#endif

}  // namespace a3_io
