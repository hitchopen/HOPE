// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/subscriber/joint_actuator_subscriber.h"

#include <algorithm>
#include <cstddef>
#include "mujoco_sim_module/common/xmodel_reader.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::subscriber::JointActuatorSubscriberBase::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::subscriber::JointActuatorSubscriberBase::Options;
  static Node encode(const Options& rhs) {
    Node node;

    node["joints"] = YAML::Node();
    for (const auto& joint : rhs.joints) {
      Node joint_node;
      joint_node["name"] = joint.name;
      joint_node["bind_joint"] = joint.bind_joint;
      node["joints"].push_back(joint_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["joints"] && node["joints"].IsSequence()) {
      for (const auto& joint_node : node["joints"]) {
        auto joint_node_options = Options::Joint{
            .name = joint_node["name"].as<std::string>(),
            .bind_joint = joint_node["bind_joint"].as<std::string>()};

        rhs.joints.emplace_back(std::move(joint_node_options));
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {
void JointActuatorSubscriberBase::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void JointActuatorSubscriberBase::ApplyCtrlData() {
  auto* current_command_array = command_array_.exchange(nullptr);
  if (current_command_array != nullptr) {
    ApplyCtrlValues(current_command_array, joint_num_);
    delete[] current_command_array;
  }
}

void JointActuatorSubscriberBase::ApplyCtrlValues(const double* ctrl_values, size_t ctrl_num) {
  if (joint_num_ == 0) return;

  if (ctrl_values == nullptr || ctrl_num != joint_num_) [[unlikely]] {
    AIMRT_WARN("Invalid ctrl data for topic '{}', expected {} joints, got {}.",
               subscriber_.GetTopic(), joint_num_, ctrl_num);
    return;
  }

  for (size_t i = 0; i < joint_num_; ++i) {
    d_->ctrl[actuator_addr_vec_[i]] = ctrl_values[i];
  }
}

void JointActuatorSubscriberBase::RegisterActuatorAddr() {
  for (auto const& joint : options_.joints) {
    int32_t actuator_id = common::GetJointActIdByJointName(m_, joint.bind_joint).value_or(-1);

    actuator_addr_vec_.emplace_back(actuator_id);
    joint_names_vec_.emplace_back(joint.name);
    joint_actuator_type_vec_.emplace_back(common::GetJointActTypeByJointName(m_, joint.bind_joint).value_or(""));

    auto joint_id = m_->actuator_trnid[static_cast<int32_t>(actuator_id * 2)];
    actuator_bind_joint_sensor_addr_vec_.emplace_back(ActuatorBindJointSensorAddr{
        .pos_addr = m_->jnt_qposadr[joint_id],
        .vel_addr = m_->jnt_dofadr[joint_id],
    });
  }

  joint_num_ = actuator_addr_vec_.size();
}

void JointActuatorSubscriberBase::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  RegisterActuatorAddr();

  options_node = options_;
}

void JointActuatorSubscriber::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::Subscribe<aimrt::protocols::sensor::JointCommandArray>(
                              subscriber_,
                              std::bind(&JointActuatorSubscriber::EventHandle, this, std::placeholders::_1)),
                          "Subscribe failed.");
}

void JointActuatorSubscriber::EventHandle(const std::shared_ptr<const aimrt::protocols::sensor::JointCommandArray>& commands) {
  if (stop_flag_) [[unlikely]]
    return;

  size_t joint_num_real = commands->joints_size();

  if (joint_num_ != joint_num_real) [[unlikely]] {
    AIMRT_WARN("The number of joints (topci: {}, expected: {}, actual: {}) in the options and the received message are different.",
               subscriber_.GetTopic(), joint_num_, joint_num_real);
    joint_num_real = std::min(joint_num_, joint_num_real);
  }

  auto* new_command_array = new double[joint_num_]();

  for (size_t ii = 0; ii < joint_num_real; ++ii) {
    const auto& joint_options = options_.joints[ii];
    const auto& command = commands->joints()[ii];

    auto itr = std::ranges::find(joint_names_vec_, command.name());
    if (itr == joint_names_vec_.end()) [[unlikely]] {
      AIMRT_WARN("Invalid msg for topic '{}', msg: {}, maybe the joint name : {} is not matched.",
                 subscriber_.GetTopic(), aimrt::Pb2CompactJson(*commands), command.name());

      delete[] new_command_array;
      return;
    }
    uint32_t joint_idx = std::distance(joint_names_vec_.begin(), itr);

    if (joint_actuator_type_vec_[joint_idx] == "position") {
      new_command_array[joint_idx] = command.position();
    } else if (joint_actuator_type_vec_[joint_idx] == "velocity") {
      new_command_array[joint_idx] = command.velocity();
    } else if (joint_actuator_type_vec_[joint_idx] == "motor") {
      // motor
      double state_posiotin = d_->qpos[actuator_bind_joint_sensor_addr_vec_[joint_idx].pos_addr];
      double state_velocity = d_->qvel[actuator_bind_joint_sensor_addr_vec_[joint_idx].vel_addr];

      new_command_array[joint_idx] = command.effort() +
                                     command.stiffness() * (command.position() - state_posiotin) +
                                     command.damping() * (command.velocity() - state_velocity);
    } else {
      AIMRT_WARN("Invalid joint actuator type '{}'.", joint_actuator_type_vec_[joint_idx]);
    }
  }

  auto* old_command_array = command_array_.exchange(new_command_array);
  delete[] old_command_array;
}

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
void JointActuatorRos2Subscriber::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::Subscribe<aimrt_msgs::msg::JointCommandArray>(
                              subscriber_,
                              std::bind(&JointActuatorRos2Subscriber::EventHandle, this, std::placeholders::_1)),
                          "Subscribe failed.");
}
void JointActuatorRos2Subscriber::EventHandle(const std::shared_ptr<const aimrt_msgs::msg::JointCommandArray>& commands) {
  if (stop_flag_) [[unlikely]]
    return;

  size_t joint_num_real = commands->joints.size();

  if (joint_num_ != joint_num_real) [[unlikely]] {
    AIMRT_WARN("The number of joints (topci: {}, expected: {}, actual: {}) in the options and the received message are different.",
               subscriber_.GetTopic(), joint_num_, joint_num_real);
    joint_num_real = std::min(joint_num_, joint_num_real);
  }

  auto* new_command_array = new double[joint_num_]();

  for (size_t ii = 0; ii < joint_num_real; ++ii) {
    const auto& joint_options = options_.joints[ii];
    const auto command = commands->joints[ii];

    auto itr = std::ranges::find(joint_names_vec_, command.name);
    if (itr == joint_names_vec_.end()) [[unlikely]] {
      AIMRT_WARN("Invalid msg for topic '{}', msg: {}, Joint name '{}' is not matched.",
                 subscriber_.GetTopic(), aimrt_msgs::msg::to_yaml(*commands), command.name);

      delete[] new_command_array;
      return;
    }
    uint32_t joint_idx = std::distance(joint_names_vec_.begin(), itr);

    if (joint_actuator_type_vec_[joint_idx] == "position") {
      new_command_array[joint_idx] = command.position;
    } else if (joint_actuator_type_vec_[joint_idx] == "velocity") {
      new_command_array[joint_idx] = command.velocity;
    } else if (joint_actuator_type_vec_[joint_idx] == "motor") {
      // motor
      double state_posiotin = d_->qpos[actuator_bind_joint_sensor_addr_vec_[joint_idx].pos_addr];
      double state_velocity = d_->qvel[actuator_bind_joint_sensor_addr_vec_[joint_idx].vel_addr];

      new_command_array[joint_idx] = command.effort +
                                     command.stiffness * (command.position - state_posiotin) +
                                     command.damping * (command.velocity - state_velocity);
    } else {
      AIMRT_WARN("Invalid joint actuator type '{}'.", joint_actuator_type_vec_[joint_idx]);
    }
  }

  auto* old_command_array = command_array_.exchange(new_command_array);
  delete[] old_command_array;
}

void BodyDriveJointActuatorSubscriber::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);
  latest_targets_.resize(joint_num_);
  ctrl_buffer_.resize(joint_num_, 0.0);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::Subscribe<joint_msgs::msg::JointCommand>(
                              subscriber_,
                              std::bind(&BodyDriveJointActuatorSubscriber::EventHandle, this, std::placeholders::_1)),
                          "Subscribe failed.");
}

void BodyDriveJointActuatorSubscriber::EventHandle(const std::shared_ptr<const joint_msgs::msg::JointCommand>& commands) {
  if (stop_flag_) [[unlikely]]
    return;

  if (!commands || commands->joints.size() < joint_num_) [[unlikely]] {
    AIMRT_WARN("Invalid msg for topic '{}', expected at least {} joints, got {}.",
               subscriber_.GetTopic(), joint_num_, commands ? commands->joints.size() : 0);
    return;
  }

  std::vector<JointCommandTarget> next_targets(joint_num_);
  for (size_t ii = 0; ii < joint_num_; ++ii) {
    const auto& command = commands->joints[ii];
    auto itr = std::ranges::find(joint_names_vec_, command.name);
    if (itr == joint_names_vec_.end()) [[unlikely]] {
      AIMRT_WARN("Invalid msg for topic '{}', msg: {}, Joint name '{}' is not matched.",
                 subscriber_.GetTopic(), joint_msgs::msg::to_yaml(*commands), command.name);
      return;
    }

    const uint32_t joint_idx = std::distance(joint_names_vec_.begin(), itr);
    next_targets[joint_idx] = JointCommandTarget{
        .position = command.position,
        .velocity = command.velocity,
        .effort = command.effort,
        .stiffness = command.stiffness,
        .damping = command.damping,
    };
  }

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    latest_targets_ = std::move(next_targets);
    has_targets_ = true;
  }
}

void BodyDriveJointActuatorSubscriber::ApplyCtrlData() {
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (!has_targets_ || latest_targets_.size() != joint_num_) return;

  if (ctrl_buffer_.size() != joint_num_) ctrl_buffer_.resize(joint_num_, 0.0);
  std::fill(ctrl_buffer_.begin(), ctrl_buffer_.end(), 0.0);

  for (size_t joint_idx = 0; joint_idx < joint_num_; ++joint_idx) {
    const auto& command = latest_targets_[joint_idx];

    if (joint_actuator_type_vec_[joint_idx] == "position") {
      ctrl_buffer_[joint_idx] = command.position;
    } else if (joint_actuator_type_vec_[joint_idx] == "velocity") {
      ctrl_buffer_[joint_idx] = command.velocity;
    } else if (joint_actuator_type_vec_[joint_idx] == "motor") {
      const double state_position = d_->qpos[actuator_bind_joint_sensor_addr_vec_[joint_idx].pos_addr];
      const double state_velocity = d_->qvel[actuator_bind_joint_sensor_addr_vec_[joint_idx].vel_addr];

      ctrl_buffer_[joint_idx] = command.effort +
                                command.stiffness * (command.position - state_position) +
                                command.damping * (command.velocity - state_velocity);
    } else {
      AIMRT_WARN("Invalid joint actuator type '{}'.", joint_actuator_type_vec_[joint_idx]);
    }
  }

  ApplyCtrlValues(ctrl_buffer_.data(), ctrl_buffer_.size());
}
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
