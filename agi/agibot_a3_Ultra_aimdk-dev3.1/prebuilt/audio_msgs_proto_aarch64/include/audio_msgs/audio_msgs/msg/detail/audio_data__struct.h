// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_msgs:msg/AudioData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/audio_data.h"


#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_DATA__STRUCT_H_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AudioData in the package audio_msgs.
typedef struct audio_msgs__msg__AudioData
{
  rosidl_runtime_c__uint8__Sequence data;
} audio_msgs__msg__AudioData;

// Struct for a sequence of audio_msgs__msg__AudioData.
typedef struct audio_msgs__msg__AudioData__Sequence
{
  audio_msgs__msg__AudioData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__msg__AudioData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_DATA__STRUCT_H_
