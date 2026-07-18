// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/subscriber/sim_reset_ros2_subscriber.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {

void SimResetRos2Subscriber::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void SimResetRos2Subscriber::Initialize(YAML::Node /*options_node*/) {
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::Subscribe<SimResetMsg>(
                              subscriber_,
                              std::bind(&SimResetRos2Subscriber::EventHandle, this, std::placeholders::_1)),
                          "Subscribe failed.");
}

void SimResetRos2Subscriber::EventHandle(const std::shared_ptr<const SimResetMsg>& msg) {
  if (stop_flag_) [[unlikely]]
    return;
  if (!msg) [[unlikely]]
    return;

  std::lock_guard<std::mutex> lock(mutex_);
  pending_msg_ = *msg;
}

void SimResetRos2Subscriber::ApplyCtrlData() {
  std::optional<SimResetMsg> msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_msg_.has_value()) return;
    msg = std::move(pending_msg_);
    pending_msg_.reset();
  }

  if (!m_ || !d_) [[unlikely]] {
    AIMRT_WARN("Ignore sim reset because mujoco model/data is not ready.");
    return;
  }

  if (msg->mode == SimResetMsg::MODE_KEYFRAME) {
    ApplyKeyframe(*msg);
  } else if (msg->mode != SimResetMsg::MODE_ABSOLUTE) {
    AIMRT_WARN("Unknown sim reset mode {}, treating it as MODE_ABSOLUTE.", msg->mode);
  }

  if (msg->zero_all_velocities) {
    std::fill(d_->qvel, d_->qvel + m_->nv, 0.0);
  }

  if (msg->set_base) {
    ApplyBasePose(*msg);
  }

  if (msg->set_base_twist) {
    ApplyBaseTwist(*msg);
  }

  if (msg->set_joints) {
    ApplyJointState(*msg);
  }

  if (msg->clear_ctrl) {
    std::fill(d_->ctrl, d_->ctrl + m_->nu, 0.0);
  }

  mj_forward(m_, d_);
}

void SimResetRos2Subscriber::ApplyKeyframe(const SimResetMsg& msg) {
  if (m_->nkey <= 0) {
    AIMRT_WARN("Ignore keyframe reset because model has no keyframes.");
    return;
  }

  const int keyframe_id = std::clamp(static_cast<int>(msg.keyframe_id), 0, m_->nkey - 1);
  mj_resetDataKeyframe(m_, d_, keyframe_id);
}

void SimResetRos2Subscriber::ApplyBasePose(const SimResetMsg& msg) {
  if (m_->nq < 7 || m_->njnt <= 0 || m_->jnt_type[0] != mjJNT_FREE) {
    AIMRT_WARN("Ignore base pose reset because the model root is not a free joint.");
    return;
  }

  d_->qpos[0] = msg.pelvis_pose.position.x;
  d_->qpos[1] = msg.pelvis_pose.position.y;
  d_->qpos[2] = msg.pelvis_pose.position.z;
  d_->qpos[3] = msg.pelvis_pose.orientation.w;
  d_->qpos[4] = msg.pelvis_pose.orientation.x;
  d_->qpos[5] = msg.pelvis_pose.orientation.y;
  d_->qpos[6] = msg.pelvis_pose.orientation.z;

  const double quat_norm = std::sqrt(d_->qpos[3] * d_->qpos[3] +
                                     d_->qpos[4] * d_->qpos[4] +
                                     d_->qpos[5] * d_->qpos[5] +
                                     d_->qpos[6] * d_->qpos[6]);
  if (quat_norm < 1e-9) {
    d_->qpos[3] = 1.0;
    d_->qpos[4] = 0.0;
    d_->qpos[5] = 0.0;
    d_->qpos[6] = 0.0;
  } else {
    mju_normalize4(d_->qpos + 3);
  }
}

void SimResetRos2Subscriber::ApplyBaseTwist(const SimResetMsg& msg) {
  if (m_->nv < 6) {
    AIMRT_WARN("Ignore base twist reset because the model has less than 6 dofs.");
    return;
  }

  d_->qvel[0] = msg.pelvis_twist.linear.x;
  d_->qvel[1] = msg.pelvis_twist.linear.y;
  d_->qvel[2] = msg.pelvis_twist.linear.z;
  d_->qvel[3] = msg.pelvis_twist.angular.x;
  d_->qvel[4] = msg.pelvis_twist.angular.y;
  d_->qvel[5] = msg.pelvis_twist.angular.z;
}

void SimResetRos2Subscriber::ApplyJointState(const SimResetMsg& msg) {
  const auto& joint_state = msg.joint_state;
  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    const auto& joint_name = joint_state.name[i];
    const int joint_id = mj_name2id(m_, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id < 0) {
      AIMRT_WARN("Ignore reset for unknown joint '{}'.", joint_name);
      continue;
    }

    const auto joint_type = m_->jnt_type[joint_id];
    if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
      AIMRT_WARN("Ignore reset for unsupported joint '{}' type {}.", joint_name, joint_type);
      continue;
    }

    if (i < joint_state.position.size()) {
      d_->qpos[m_->jnt_qposadr[joint_id]] = joint_state.position[i];
    }

    if (i < joint_state.velocity.size()) {
      d_->qvel[m_->jnt_dofadr[joint_id]] = joint_state.velocity[i];
    }
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
