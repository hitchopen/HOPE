// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/subscriber/gate3_ball_command_ros2_subscriber.h"

#include <algorithm>
#include <cmath>
#include <functional>

#include "mujoco_sim_module/common/gate3_ball_layout.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {
namespace gate3 = common::gate3_ball;

void Gate3BallCommandRos2Subscriber::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
  ball_joint_id_ = mj_name2id(m_, mjOBJ_JOINT, gate3::kJointName);
  AIMRT_CHECK_ERROR_THROW(
      ball_joint_id_ >= 0 && m_->jnt_type[ball_joint_id_] == mjJNT_FREE,
      "Gate3 ball free joint '{}' is missing.", gate3::kJointName);
  AIMRT_CHECK_ERROR_THROW(
      m_->nuserdata >= static_cast<int>(gate3::kRequiredUserData),
      "Gate3 ball requires at least {} userdata values, model has {}.",
      gate3::kRequiredUserData, m_->nuserdata);
  qpos_addr_ = m_->jnt_qposadr[ball_joint_id_];
  dof_addr_ = m_->jnt_dofadr[ball_joint_id_];
}

void Gate3BallCommandRos2Subscriber::Initialize(YAML::Node /*options_node*/) {
  AIMRT_CHECK_ERROR_THROW(
      aimrt::channel::Subscribe<BallCommand>(
          subscriber_,
          std::bind(&Gate3BallCommandRos2Subscriber::EventHandle, this,
                    std::placeholders::_1)),
      "Subscribe Gate3 ball command failed.");
}

void Gate3BallCommandRos2Subscriber::EventHandle(
    const std::shared_ptr<const BallCommand>& msg) {
  if (stop_flag_ || !msg) [[unlikely]]
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  pending_msg_ = *msg;
}

void Gate3BallCommandRos2Subscriber::ResetTelemetry(
    std::uint64_t shot_id, bool active) {
  d_->userdata[gate3::kShotId] = static_cast<mjtNum>(shot_id);
  d_->userdata[gate3::kActive] = active ? 1.0 : 0.0;
  d_->userdata[gate3::kContactBits] = 0.0;
  d_->userdata[gate3::kRacketContactCount] = 0.0;
  d_->userdata[gate3::kTableContactCount] = 0.0;
  d_->userdata[gate3::kNetContactCount] = 0.0;
  d_->userdata[gate3::kRacketNormalForce] = 0.0;
}

void Gate3BallCommandRos2Subscriber::ParkBall(std::uint64_t shot_id) {
  d_->qpos[qpos_addr_ + 0] = 100.0;
  d_->qpos[qpos_addr_ + 1] = 0.0;
  d_->qpos[qpos_addr_ + 2] = -10.0;
  d_->qpos[qpos_addr_ + 3] = 1.0;
  d_->qpos[qpos_addr_ + 4] = 0.0;
  d_->qpos[qpos_addr_ + 5] = 0.0;
  d_->qpos[qpos_addr_ + 6] = 0.0;
  std::fill(d_->qvel + dof_addr_, d_->qvel + dof_addr_ + 6, 0.0);
  ResetTelemetry(shot_id, false);
}

void Gate3BallCommandRos2Subscriber::ApplyCtrlData() {
  std::optional<BallCommand> msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_msg_.has_value()) return;
    msg = std::move(pending_msg_);
    pending_msg_.reset();
  }
  if (!m_ || !d_ || qpos_addr_ < 0 || dof_addr_ < 0) [[unlikely]]
    return;

  const auto current_shot_id = static_cast<std::uint64_t>(
      std::max<mjtNum>(0.0, d_->userdata[gate3::kShotId]));
  const bool current_active = d_->userdata[gate3::kActive] > 0.5;
  if (msg->shot_id == 0) {
    AIMRT_WARN("Reject Gate3 ball command with shot_id=0.");
    return;
  }

  // ROS reliability or the launcher's acknowledgement loop may deliver the
  // same command more than once. A duplicate must acknowledge the current
  // state without relaunching the body or erasing 1 kHz edge counters.
  if (msg->shot_id == current_shot_id &&
      msg->active == current_active) {
    return;
  }

  if (!msg->active) {
    if (msg->shot_id != current_shot_id || !current_active) {
      AIMRT_WARN(
          "Reject out-of-order Gate3 park shot_id={} while current "
          "shot_id={} active={}.",
          msg->shot_id, current_shot_id, current_active);
      return;
    }
    ParkBall(msg->shot_id);
    mj_forward(m_, d_);
    return;
  }

  if (current_active || msg->shot_id != current_shot_id + 1) {
    AIMRT_WARN(
        "Reject out-of-order Gate3 launch shot_id={} while current "
        "shot_id={} active={}; each shot must be parked before the next "
        "monotonic ID.",
        msg->shot_id, current_shot_id, current_active);
    return;
  }

  const double values[] = {
      msg->position.x, msg->position.y, msg->position.z,
      msg->linear_velocity.x, msg->linear_velocity.y,
      msg->linear_velocity.z};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](double value) { return std::isfinite(value); })) {
    AIMRT_WARN("Reject non-finite Gate3 ball launch shot_id={}.", msg->shot_id);
    ParkBall(msg->shot_id);
    mj_forward(m_, d_);
    return;
  }

  d_->qpos[qpos_addr_ + 0] = msg->position.x;
  d_->qpos[qpos_addr_ + 1] = msg->position.y;
  d_->qpos[qpos_addr_ + 2] = msg->position.z;
  d_->qpos[qpos_addr_ + 3] = 1.0;
  d_->qpos[qpos_addr_ + 4] = 0.0;
  d_->qpos[qpos_addr_ + 5] = 0.0;
  d_->qpos[qpos_addr_ + 6] = 0.0;
  d_->qvel[dof_addr_ + 0] = msg->linear_velocity.x;
  d_->qvel[dof_addr_ + 1] = msg->linear_velocity.y;
  d_->qvel[dof_addr_ + 2] = msg->linear_velocity.z;
  d_->qvel[dof_addr_ + 3] = 0.0;
  d_->qvel[dof_addr_ + 4] = 0.0;
  d_->qvel[dof_addr_ + 5] = 0.0;
  ResetTelemetry(msg->shot_id, true);
  mj_forward(m_, d_);
  AIMRT_INFO(
      "Gate3 ball launch shot_id={} p=[{:.3f},{:.3f},{:.3f}] "
      "v=[{:.3f},{:.3f},{:.3f}].",
      msg->shot_id, msg->position.x, msg->position.y, msg->position.z,
      msg->linear_velocity.x, msg->linear_velocity.y,
      msg->linear_velocity.z);
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
