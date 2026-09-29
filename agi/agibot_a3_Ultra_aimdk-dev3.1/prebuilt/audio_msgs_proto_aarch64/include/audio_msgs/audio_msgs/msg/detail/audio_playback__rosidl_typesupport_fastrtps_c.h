// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from audio_msgs:msg/AudioPlayback.idl
// generated code does not contain a copyright notice
#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "audio_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "audio_msgs/msg/detail/audio_playback__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
bool cdr_serialize_audio_msgs__msg__AudioPlayback(
  const audio_msgs__msg__AudioPlayback * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
bool cdr_deserialize_audio_msgs__msg__AudioPlayback(
  eprosima::fastcdr::Cdr &,
  audio_msgs__msg__AudioPlayback * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
size_t get_serialized_size_audio_msgs__msg__AudioPlayback(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
size_t max_serialized_size_audio_msgs__msg__AudioPlayback(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
bool cdr_serialize_key_audio_msgs__msg__AudioPlayback(
  const audio_msgs__msg__AudioPlayback * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
size_t get_serialized_size_key_audio_msgs__msg__AudioPlayback(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
size_t max_serialized_size_key_audio_msgs__msg__AudioPlayback(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, audio_msgs, msg, AudioPlayback)();

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
