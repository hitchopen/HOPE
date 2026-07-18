# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

if(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
  add_subdirectory(joint_msgs)
  add_subdirectory(mujoco_sim_msgs)
endif()

if(AIMRT_MUJOCO_SIM_BUILD_EXAMPLES)
  add_subdirectory(examples/inverted_pendulum)
endif()
