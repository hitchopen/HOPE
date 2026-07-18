// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <utility>

#include "pid_control.aimrt_rpc.pb.h"
#include "pid_control_module/pid_control_algorithm.h"

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

class PidControlServiceImpl : public aimrt_mujoco_sim::protocols::examples::inverted_pendulum::PidControlCoService {
 public:
  explicit PidControlServiceImpl(PIDController& controller) : controller_(controller) {}
  ~PidControlServiceImpl() override = default;

  aimrt::co::Task<aimrt::rpc::Status> SetPid(
      aimrt::rpc::ContextRef ctx,
      const aimrt_mujoco_sim::protocols::examples::inverted_pendulum::SetPidReq& req,
      aimrt_mujoco_sim::protocols::examples::inverted_pendulum::SetPidRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> GetPid(
      aimrt::rpc::ContextRef ctx,
      const aimrt_mujoco_sim::protocols::examples::inverted_pendulum::GetPidReq& req,
      aimrt_mujoco_sim::protocols::examples::inverted_pendulum::GetPidRsp& rsp) override;

 private:
  PIDController& controller_;
};

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module
