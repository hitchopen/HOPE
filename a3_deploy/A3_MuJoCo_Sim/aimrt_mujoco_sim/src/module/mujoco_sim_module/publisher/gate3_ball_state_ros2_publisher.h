// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "mujoco_sim_msgs/msg/gate3_ball_state.hpp"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

class Gate3BallStateRos2Publisher : public PublisherBase {
 public:
  Gate3BallStateRos2Publisher() = default;
  ~Gate3BallStateRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override {
    return "gate3_ball_state_ros2";
  }
  void PublishSensorData() override;
  void Start() override {}
  void Shutdown() override {}

  void SetMj(mjModel* m, mjData* d) override;
  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override {
    publisher_ = publisher_handle;
  }
  void SetExecutor(aimrt::executor::ExecutorRef executor) override {
    executor_ = executor;
  }
  void SetFreq(uint32_t freq) override { channel_frq_ = freq; }

 private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int qpos_addr_ = -1;
  int dof_addr_ = -1;
  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;
  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0.0;
  uint32_t counter_ = 0;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
