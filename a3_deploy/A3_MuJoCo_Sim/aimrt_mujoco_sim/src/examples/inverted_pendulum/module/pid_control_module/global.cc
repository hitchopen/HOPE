// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "pid_control_module/global.h"

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

aimrt::logger::LoggerRef global_logger;
void SetLogger(aimrt::logger::LoggerRef logger) { global_logger = logger; }
aimrt::logger::LoggerRef GetLogger() { return global_logger; }

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module
