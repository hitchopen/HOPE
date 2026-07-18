// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#pragma once

#include <mutex>
#include <optional>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/subscriber/subscriber_base.h"
#include "mujoco_sim_msgs/msg/sim_reset.hpp"

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {

class SimResetRos2Subscriber : public SubscriberBase {
 public:
  SimResetRos2Subscriber() = default;
  ~SimResetRos2Subscriber() override = default;

  void Initialize(YAML::Node options_node) override;
  void Start() override { stop_flag_ = false; }
  void Shutdown() override { stop_flag_ = true; }

  std::string_view Type() const noexcept override { return "sim_reset_ros2"; }

  void SetMj(mjModel* m, mjData* d) override;
  void SetSubscriberHandle(aimrt::channel::SubscriberRef subscriber_handle) override { subscriber_ = subscriber_handle; }

  void ApplyCtrlData() override;

 private:
  using SimResetMsg = mujoco_sim_msgs::msg::SimReset;

  void EventHandle(const std::shared_ptr<const SimResetMsg>& msg);
  void ApplyKeyframe(const SimResetMsg& msg);
  void ApplyBasePose(const SimResetMsg& msg);
  void ApplyBaseTwist(const SimResetMsg& msg);
  void ApplyJointState(const SimResetMsg& msg);

  bool stop_flag_ = true;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  aimrt::channel::SubscriberRef subscriber_;
  std::mutex mutex_;
  std::optional<SimResetMsg> pending_msg_;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
