// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/subscriber/joint_actuator_subscriber.h"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <string_view>
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
  implicit_pd_ = std::string_view(std::getenv("A3_MUJOCO_PD_MODE")
                                      ? std::getenv("A3_MUJOCO_PD_MODE") : "explicit") == "implicit";
  base_passive_damping_.resize(joint_num_, 0.0);
  for (size_t joint_idx = 0; joint_idx < joint_num_; ++joint_idx) {
    const int dof = actuator_bind_joint_sensor_addr_vec_[joint_idx].vel_addr;
    base_passive_damping_[joint_idx] = m_->dof_damping[dof];
  }

  const char* vendor_profile =
      std::getenv("A3_MUJOCO_HUMANLIKE_VENDOR_PROFILE");
  const std::string_view topic = subscriber_.GetTopic();
  const bool humanlike_owned_topic =
      topic.find("/leg_joint_command") != std::string_view::npos ||
      topic.find("/waist_joint_command") != std::string_view::npos;
  humanlike_vendor_profile_ = vendor_profile != nullptr &&
                              std::string_view(vendor_profile) != "0" &&
                              humanlike_owned_topic;
  explicit_pd_decimation_ = humanlike_vendor_profile_ ? 2u : 1u;
  torque_limits_.assign(joint_num_, std::numeric_limits<double>::infinity());
  if (humanlike_vendor_profile_) {
    // Physical A3 HumanLike runs MOTION at 2 ms.  It recomputes the
    // torque-feasible q_des interval from fresh q/dq on every base tick, while
    // the 100 Hz actor output is held for five ticks.  Computing and clipping
    // the equivalent explicit torque every other 1 ms MuJoCo step reproduces
    // that actuator contract without requiring duplicate command publishers.
    const auto vendor_limit = [](std::string_view name) {
      if (name == "waist_yaw_joint") return 220.0;
      if (name == "waist_roll_joint") return 45.0;
      if (name == "waist_pitch_joint") return 110.0;
      if (name.find("hip_") != std::string_view::npos) return 220.0;
      if (name.find("knee") != std::string_view::npos) return 320.0;
      if (name.find("ankle_pitch") != std::string_view::npos) return 110.0;
      if (name.find("ankle_roll") != std::string_view::npos) return 45.0;
      if (name.find("shoulder_pitch") != std::string_view::npos ||
          name.find("shoulder_roll") != std::string_view::npos)
        return 60.0;
      if (name.find("shoulder_yaw") != std::string_view::npos ||
          name.find("elbow") != std::string_view::npos ||
          name.find("wrist_roll") != std::string_view::npos)
        return 24.0;
      if (name.find("wrist_pitch") != std::string_view::npos ||
          name.find("wrist_yaw") != std::string_view::npos)
        return 6.0;
      return std::numeric_limits<double>::infinity();
    };
    for (size_t joint_idx = 0; joint_idx < joint_num_; ++joint_idx)
      torque_limits_[joint_idx] = vendor_limit(joint_names_vec_[joint_idx]);
    AIMRT_INFO(
        "HumanLike vendor actuator profile on '{}': explicit PD=500 Hz, "
        "default.yaml torque limits enabled.",
        subscriber_.GetTopic());
  }

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

  if (!implicit_pd_ && explicit_pd_decimation_ > 1) {
    const bool recompute =
        (explicit_pd_tick_ % explicit_pd_decimation_) == 0;
    ++explicit_pd_tick_;
    if (!recompute) return;  // hold the previous d_->ctrl for the second 1 ms step
  }

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
      const int dof = actuator_bind_joint_sensor_addr_vec_[joint_idx].vel_addr;
      const double state_velocity = d_->qvel[dof];

      if (implicit_pd_) {
        // Isaac-faithful diagnostic: preserve the MJCF plant damping and add the runner's
        // message kd to MuJoCo's implicit damping solve. The normal AGI path remains the
        // explicit torque formula below. dq_des is zero in the policy runner, but retain
        // its feed-forward kd*dq_des term for a complete command contract.
        m_->dof_damping[dof] = base_passive_damping_[joint_idx] + command.damping;
        ctrl_buffer_[joint_idx] = command.effort +
                                  command.stiffness * (command.position - state_position) +
                                  command.damping * command.velocity;
      } else {
        ctrl_buffer_[joint_idx] = command.effort +
                                  command.stiffness * (command.position - state_position) +
                                  command.damping * (command.velocity - state_velocity);
        const double limit = torque_limits_[joint_idx];
        ctrl_buffer_[joint_idx] =
            std::clamp(ctrl_buffer_[joint_idx], -limit, limit);
      }
    } else {
      AIMRT_WARN("Invalid joint actuator type '{}'.", joint_actuator_type_vec_[joint_idx]);
    }
  }

  ApplyCtrlValues(ctrl_buffer_.data(), ctrl_buffer_.size());
}
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
