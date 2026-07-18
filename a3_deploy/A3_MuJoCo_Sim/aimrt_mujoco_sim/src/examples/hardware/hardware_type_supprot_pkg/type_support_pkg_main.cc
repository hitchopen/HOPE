// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_protobuf_interface/util/protobuf_type_support.h"
#include "imu.pb.h"
#include "joint_command.pb.h"
#include "joint_state.pb.h"
#include "touch_sensor_state.pb.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include <sensor_msgs/msg/imu.hpp>
  #include "aimrt_module_ros2_interface/util/ros2_type_support.h"
  #include "aimrt_msgs/msg/joint_command_array.hpp"
  #include "aimrt_msgs/msg/joint_state_array.hpp"
  #include "aimrt_msgs/msg/touch_sensor_state_array.hpp"
#endif

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::ImuState>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::JointStateArray>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::JointCommandArray>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::TouchSensorStateArray>(),

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Imu>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::JointStateArray>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::JointCommandArray>(),
    aimrt::GetRos2MessageTypeSupport<aimrt_msgs::msg::TouchSensorStateArray>(),
#endif
};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}
}