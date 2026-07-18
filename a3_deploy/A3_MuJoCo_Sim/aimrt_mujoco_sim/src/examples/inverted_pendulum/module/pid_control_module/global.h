// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

void SetLogger(aimrt::logger::LoggerRef);
aimrt::logger::LoggerRef GetLogger();

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module
