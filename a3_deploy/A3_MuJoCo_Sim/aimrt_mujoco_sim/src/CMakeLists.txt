# Copyright (c) 2023, AgiBot Inc.
# All rights reserved

set_root_namespace("aimrt_mujoco_sim")

add_subdirectory(protocols)

add_subdirectory(module/mujoco_sim_module)

add_subdirectory(pkg/mujoco_sim_pkg)

# models = the a3_pingpong sim (what HOPE needs); build it always. It is
# self-contained (copies its own src/models/bin, depends only on aimrt::runtime::main).
add_subdirectory(models)

# examples/ (hardware, inverted_pendulum) copy a prebuilt install/linux/bin that is
# NOT in this repo, so their build_all copy step fails. We never run them; keep gated.
if(AIMRT_MUJOCO_SIM_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
