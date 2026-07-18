# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc)
file(GLOB_RECURSE with_ros2_files ${CMAKE_CURRENT_SOURCE_DIR}/*ros2*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*ros2*.h)
list(REMOVE_ITEM src ${test_files})

# Add target
add_library(${CUR_TARGET_NAME} STATIC)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# deal with ROS2 related files and dependencies
if(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
  find_package(geometry_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)

  set(ADDITIONAL_ROS2_DEPS
      aimrt::interface::aimrt_module_ros2_interface #
      aimrt_msgs::aimrt_msgs__rosidl_generator_cpp #
      aimrt_msgs::aimrt_msgs__rosidl_typesupport_cpp #
      aimrt_msgs::aimrt_msgs__rosidl_typesupport_fastrtps_cpp #
      aimrt_msgs::aimrt_msgs__rosidl_typesupport_introspection_cpp #
      geometry_msgs::geometry_msgs__rosidl_generator_cpp #
      geometry_msgs::geometry_msgs__rosidl_typesupport_cpp #
      geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_cpp #
      geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp #
      joint_msgs::joint_msgs__rosidl_generator_cpp #
      joint_msgs::joint_msgs__rosidl_typesupport_cpp #
      joint_msgs::joint_msgs__rosidl_typesupport_fastrtps_cpp #
      joint_msgs::joint_msgs__rosidl_typesupport_introspection_cpp #
      mujoco_sim_msgs::mujoco_sim_msgs__rosidl_generator_cpp #
      mujoco_sim_msgs::mujoco_sim_msgs__rosidl_typesupport_cpp #
      mujoco_sim_msgs::mujoco_sim_msgs__rosidl_typesupport_fastrtps_cpp #
      mujoco_sim_msgs::mujoco_sim_msgs__rosidl_typesupport_introspection_cpp #
      sensor_msgs::sensor_msgs__rosidl_generator_cpp #
      sensor_msgs::sensor_msgs__rosidl_typesupport_cpp #
      sensor_msgs::sensor_msgs__rosidl_typesupport_fastrtps_cpp #
      sensor_msgs::sensor_msgs__rosidl_typesupport_introspection_cpp #
      tf2_msgs::tf2_msgs__rosidl_generator_cpp #
      tf2_msgs::tf2_msgs__rosidl_typesupport_cpp #
      tf2_msgs::tf2_msgs__rosidl_typesupport_fastrtps_cpp #
      tf2_msgs::tf2_msgs__rosidl_typesupport_introspection_cpp)
  target_compile_definitions(${CUR_TARGET_NAME} PRIVATE AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
else()
  # remove ROS2 related files and dependencies
  list(REMOVE_ITEM src ${with_ros2_files})
  list(REMOVE_ITEM head_files ${with_ros2_files})
  set(ADDITIONAL_ROS2_DEPS "")
endif()

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE yaml-cpp::yaml-cpp
  PUBLIC aimrt::interface::aimrt_module_cpp_interface
         aimrt::protocols::sensor_pb_gencode
         aimrt::protocols::actuator_pb_gencode
         mujoco::libsimulate
         mujoco::mujoco
         glfw
         ${ADDITIONAL_ROS2_DEPS})

# Set test of target
if(AIMRT_MUJOCO_SIM_BUILD_TESTS AND test_files)
  add_gtest_target(TEST_TARGET ${CUR_TARGET_NAME} TEST_SRC ${test_files})
endif()
