// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "pid_control_module/pid_control_module.h"

using namespace aimrt_mujoco_sim::example::inverted_pendulum;

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"PidControlModule", []() -> aimrt::ModuleBase* { return new pid_control_module::PidControlModule(); }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
