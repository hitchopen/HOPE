// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "pid_control_module/pid_control_service.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "pid_control_module/global.h"

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

aimrt::co::Task<aimrt::rpc::Status> PidControlServiceImpl::SetPid(
    aimrt::rpc::ContextRef ctx,
    const aimrt_mujoco_sim::protocols::examples::inverted_pendulum::SetPidReq& req,
    aimrt_mujoco_sim::protocols::examples::inverted_pendulum::SetPidRsp& rsp) {
  // set pid params
  const auto& pid_value = req.value();
  controller_.SetPIDParm(PIDParm{
      .Kp = pid_value.p(),
      .Ki = pid_value.i(),
      .Kd = pid_value.d()});

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> PidControlServiceImpl::GetPid(
    aimrt::rpc::ContextRef ctx,
    const aimrt_mujoco_sim::protocols::examples::inverted_pendulum::GetPidReq& req,
    aimrt_mujoco_sim::protocols::examples::inverted_pendulum::GetPidRsp& rsp) {
  auto pid_parm = controller_.GetPIDParm();
  rsp.mutable_value()->set_p(pid_parm.Kp);
  rsp.mutable_value()->set_i(pid_parm.Ki);
  rsp.mutable_value()->set_d(pid_parm.Kd);

  co_return aimrt::rpc::Status();
}

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module