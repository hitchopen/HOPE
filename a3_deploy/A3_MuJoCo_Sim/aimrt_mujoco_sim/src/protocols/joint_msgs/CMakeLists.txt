# Copyright (c) 2026, AgiBot Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.8)

project(joint_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(CUR_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Command.msg"
  "msg/JointCommand.msg"
  "msg/JointState.msg"
  "msg/State.msg"
  DEPENDENCIES
  std_msgs)

set(BUILD_SHARED_LIBS ${CUR_BUILD_SHARED_LIBS})

foreach(_target_suffix
        __rosidl_generator_cpp
        __rosidl_typesupport_cpp
        __rosidl_typesupport_fastrtps_cpp
        __rosidl_typesupport_introspection_cpp)
  if(TARGET ${PROJECT_NAME}${_target_suffix} AND NOT TARGET ${PROJECT_NAME}::${PROJECT_NAME}${_target_suffix})
    add_library(${PROJECT_NAME}::${PROJECT_NAME}${_target_suffix} ALIAS ${PROJECT_NAME}${_target_suffix})
  endif()
endforeach()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
