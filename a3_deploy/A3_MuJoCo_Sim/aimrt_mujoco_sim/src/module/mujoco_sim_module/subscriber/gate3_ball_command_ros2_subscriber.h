// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#pragma once

#include <mutex>
#include <optional>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "mujoco_sim_msgs/msg/gate3_ball_command.hpp"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/subscriber/subscriber_base.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {

class Gate3BallCommandRos2Subscriber : public SubscriberBase {
 public:
  Gate3BallCommandRos2Subscriber() = default;
  ~Gate3BallCommandRos2Subscriber() override = default;

  void Initialize(YAML::Node options_node) override;
  void Start() override { stop_flag_ = false; }
  void Shutdown() override { stop_flag_ = true; }

  std::string_view Type() const noexcept override {
    return "gate3_ball_command_ros2";
  }

  void SetMj(mjModel* m, mjData* d) override;
  void SetSubscriberHandle(aimrt::channel::SubscriberRef subscriber_handle) override {
    subscriber_ = subscriber_handle;
  }
  void ApplyCtrlData() override;

 private:
  using BallCommand = mujoco_sim_msgs::msg::Gate3BallCommand;

  void EventHandle(const std::shared_ptr<const BallCommand>& msg);
  void ParkBall(std::uint64_t shot_id);
  void ResetTelemetry(std::uint64_t shot_id, bool active);

  bool stop_flag_ = true;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int ball_joint_id_ = -1;
  int qpos_addr_ = -1;
  int dof_addr_ = -1;
  aimrt::channel::SubscriberRef subscriber_;
  std::mutex mutex_;
  std::optional<BallCommand> pending_msg_;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
