// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/module_base.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "pid_control_module/pid_control_algorithm.h"
#include "pid_control_module/pid_control_service.h"

#include "joint_command.pb.h"
#include "joint_state.pb.h"
#include "pid_control.aimrt_rpc.pb.h"

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

class PidControlModule : public aimrt::ModuleBase {
 public:
  PidControlModule() = default;
  ~PidControlModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "PidControlModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  void EventHandle(const std::shared_ptr<const aimrt::protocols::sensor::JointStateArray>& data);

 private:
  aimrt::CoreRef core_;

  std::unique_ptr<aimrt::channel::PublisherProxy<aimrt::protocols::sensor::JointCommandArray>> publisher_proxy_;
  std::unique_ptr<PidControlServiceImpl> service_ptr_;

  PIDController controller_;
};

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module
