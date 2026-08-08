// Copyright (c) 2026, AgiBot Inc. All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"
#include "aimrt_module_ros2_interface/util/ros2_type_support.h"

#include "joint_msgs/msg/joint_command.hpp"
#include "joint_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetRos2MessageTypeSupport<joint_msgs::msg::JointState>(),
    aimrt::GetRos2MessageTypeSupport<joint_msgs::msg::JointCommand>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Imu>(),
};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}

}  // extern "C"
