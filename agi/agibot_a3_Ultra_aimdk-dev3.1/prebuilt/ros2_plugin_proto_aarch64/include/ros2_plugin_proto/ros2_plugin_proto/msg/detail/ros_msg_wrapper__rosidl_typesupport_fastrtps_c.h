// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from ros2_plugin_proto:msg/RosMsgWrapper.idl
// generated code does not contain a copyright notice
#ifndef ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2_plugin_proto/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2_plugin_proto/msg/detail/ros_msg_wrapper__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
bool cdr_serialize_ros2_plugin_proto__msg__RosMsgWrapper(
  const ros2_plugin_proto__msg__RosMsgWrapper * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
bool cdr_deserialize_ros2_plugin_proto__msg__RosMsgWrapper(
  eprosima::fastcdr::Cdr &,
  ros2_plugin_proto__msg__RosMsgWrapper * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
size_t get_serialized_size_ros2_plugin_proto__msg__RosMsgWrapper(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
size_t max_serialized_size_ros2_plugin_proto__msg__RosMsgWrapper(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
bool cdr_serialize_key_ros2_plugin_proto__msg__RosMsgWrapper(
  const ros2_plugin_proto__msg__RosMsgWrapper * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
size_t get_serialized_size_key_ros2_plugin_proto__msg__RosMsgWrapper(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
size_t max_serialized_size_key_ros2_plugin_proto__msg__RosMsgWrapper(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_plugin_proto
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_plugin_proto, msg, RosMsgWrapper)();

#ifdef __cplusplus
}
#endif

#endif  // ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
