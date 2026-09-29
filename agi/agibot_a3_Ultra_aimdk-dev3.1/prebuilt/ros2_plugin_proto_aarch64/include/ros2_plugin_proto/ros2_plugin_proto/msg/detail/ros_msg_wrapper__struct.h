// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_plugin_proto:msg/RosMsgWrapper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_plugin_proto/msg/ros_msg_wrapper.h"


#ifndef ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__STRUCT_H_
#define ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'serialization_type'
// Member 'context'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RosMsgWrapper in the package ros2_plugin_proto.
typedef struct ros2_plugin_proto__msg__RosMsgWrapper
{
  rosidl_runtime_c__String serialization_type;
  rosidl_runtime_c__String__Sequence context;
  rosidl_runtime_c__octet__Sequence data;
} ros2_plugin_proto__msg__RosMsgWrapper;

// Struct for a sequence of ros2_plugin_proto__msg__RosMsgWrapper.
typedef struct ros2_plugin_proto__msg__RosMsgWrapper__Sequence
{
  ros2_plugin_proto__msg__RosMsgWrapper * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_plugin_proto__msg__RosMsgWrapper__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__STRUCT_H_
