// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/gate3_ball_state_ros2_publisher.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "mujoco_sim_module/common/gate3_ball_layout.h"
#include "mujoco_sim_module/publisher/utils.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {
namespace gate3 = common::gate3_ball;

void Gate3BallStateRos2Publisher::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
  const int joint_id = mj_name2id(m_, mjOBJ_JOINT, gate3::kJointName);
  AIMRT_CHECK_ERROR_THROW(
      joint_id >= 0 && m_->jnt_type[joint_id] == mjJNT_FREE,
      "Gate3 ball free joint '{}' is missing.", gate3::kJointName);
  AIMRT_CHECK_ERROR_THROW(
      m_->nuserdata >= static_cast<int>(gate3::kRequiredUserData),
      "Gate3 ball state requires at least {} userdata values.",
      gate3::kRequiredUserData);
  qpos_addr_ = m_->jnt_qposadr[joint_id];
  dof_addr_ = m_->jnt_dofadr[joint_id];
}

void Gate3BallStateRos2Publisher::Initialize(YAML::Node /*options_node*/) {
  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);
  AIMRT_CHECK_ERROR_THROW(
      aimrt::channel::RegisterPublishType<mujoco_sim_msgs::msg::Gate3BallState>(
          publisher_),
      "Register Gate3 ball state publish type failed.");
}

void Gate3BallStateRos2Publisher::PublishSensorData() {
  static constexpr uint32_t kCounterWrap = 1024 * 1024;
  if (counter_++ < avg_interval_) return;

  auto msg = std::make_unique<mujoco_sim_msgs::msg::Gate3BallState>();
  const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
  msg->header.stamp.sec = static_cast<int32_t>(timestamp / 1000000000);
  msg->header.stamp.nanosec =
      static_cast<uint32_t>(timestamp % 1000000000);
  msg->header.frame_id = "world";
  msg->shot_id = static_cast<std::uint64_t>(
      std::max<mjtNum>(0.0, d_->userdata[gate3::kShotId]));
  msg->active = d_->userdata[gate3::kActive] > 0.5;
  msg->position.x = d_->qpos[qpos_addr_ + 0];
  msg->position.y = d_->qpos[qpos_addr_ + 1];
  msg->position.z = d_->qpos[qpos_addr_ + 2];
  msg->linear_velocity.x = d_->qvel[dof_addr_ + 0];
  msg->linear_velocity.y = d_->qvel[dof_addr_ + 1];
  msg->linear_velocity.z = d_->qvel[dof_addr_ + 2];
  msg->contact_bits = static_cast<std::uint8_t>(
      std::clamp<mjtNum>(d_->userdata[gate3::kContactBits], 0.0, 255.0));
  msg->racket_contact_count = static_cast<std::uint32_t>(
      std::max<mjtNum>(0.0, d_->userdata[gate3::kRacketContactCount]));
  msg->table_contact_count = static_cast<std::uint32_t>(
      std::max<mjtNum>(0.0, d_->userdata[gate3::kTableContactCount]));
  msg->net_contact_count = static_cast<std::uint32_t>(
      std::max<mjtNum>(0.0, d_->userdata[gate3::kNetContactCount]));
  msg->racket_normal_force_n =
      d_->userdata[gate3::kRacketNormalForce];

  executor_.Execute([this, msg = std::move(msg)]() {
    aimrt::channel::Publish(publisher_, *msg);
  });

  avg_interval_ += avg_interval_base_;
  if (counter_ > kCounterWrap) {
    avg_interval_ -= kCounterWrap;
    counter_ -= kCounterWrap;
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher

