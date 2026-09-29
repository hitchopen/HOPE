// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_msgs:msg/FocusRequester.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/focus_requester.h"


#ifndef AUDIO_MSGS__MSG__DETAIL__FOCUS_REQUESTER__STRUCT_H_
#define AUDIO_MSGS__MSG__DETAIL__FOCUS_REQUESTER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'pkg_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FocusRequester in the package audio_msgs.
/**
  * 必要项，标识调用方来源
 */
typedef struct audio_msgs__msg__FocusRequester
{
  rosidl_runtime_c__String pkg_name;
  /// 必要项，优先级(1~10, 默认6)
  uint32_t priority;
  /// 可选项，(1~100) priority + priority_weight% is final priority
  uint32_t priority_weight;
} audio_msgs__msg__FocusRequester;

// Struct for a sequence of audio_msgs__msg__FocusRequester.
typedef struct audio_msgs__msg__FocusRequester__Sequence
{
  audio_msgs__msg__FocusRequester * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__msg__FocusRequester__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__MSG__DETAIL__FOCUS_REQUESTER__STRUCT_H_
