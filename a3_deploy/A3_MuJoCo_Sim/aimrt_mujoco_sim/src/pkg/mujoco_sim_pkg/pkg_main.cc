// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "mujoco_sim_module/mujoco_sim_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"MujocoSimModule", []() -> aimrt::ModuleBase* {
       return new aimrt_mujoco_sim::mujoco_sim_module::MujocoSimModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
