// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_msgs:msg/AudioPlayback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/audio_playback.h"


#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__STRUCT_H_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamps'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'info'
#include "audio_msgs/msg/detail/audio_info__struct.h"
// Member 'data'
#include "audio_msgs/msg/detail/audio_data__struct.h"
// Member 'pkg_name'
// Member 'token_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/AudioPlayback in the package audio_msgs.
/**
  * 必要项，时间戳
 */
typedef struct audio_msgs__msg__AudioPlayback
{
  builtin_interfaces__msg__Time stamps;
  /// 必要项，音频格式
  audio_msgs__msg__AudioInfo info;
  /// 必要项，音频数据
  audio_msgs__msg__AudioData data;
  /// 必要项，标识发送方来源
  rosidl_runtime_c__String pkg_name;
  /// 可选项，token_id有变化时会清理播放缓存（用于打断当前播放）
  rosidl_runtime_c__String token_id;
} audio_msgs__msg__AudioPlayback;

// Struct for a sequence of audio_msgs__msg__AudioPlayback.
typedef struct audio_msgs__msg__AudioPlayback__Sequence
{
  audio_msgs__msg__AudioPlayback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__msg__AudioPlayback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__STRUCT_H_
