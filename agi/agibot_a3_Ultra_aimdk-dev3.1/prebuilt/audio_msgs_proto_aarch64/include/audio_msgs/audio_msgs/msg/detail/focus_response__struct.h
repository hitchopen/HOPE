// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_msgs:msg/FocusResponse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/focus_response.h"


#ifndef AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__STRUCT_H_
#define AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__STRUCT_H_

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

/// Struct defined in msg/FocusResponse in the package audio_msgs.
/**
  * 必要项，标识调用方来源
 */
typedef struct audio_msgs__msg__FocusResponse
{
  rosidl_runtime_c__String pkg_name;
  /// 必要项，焦点结果
  /// true: 获取到焦点
  /// false: 丢失焦点
  bool focus_gain;
} audio_msgs__msg__FocusResponse;

// Struct for a sequence of audio_msgs__msg__FocusResponse.
typedef struct audio_msgs__msg__FocusResponse__Sequence
{
  audio_msgs__msg__FocusResponse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__msg__FocusResponse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__STRUCT_H_
