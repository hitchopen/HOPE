// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_msgs:srv/RequestAudioFocus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/srv/request_audio_focus.h"


#ifndef AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__STRUCT_H_
#define AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'focus_requester'
#include "audio_msgs/msg/detail/focus_requester__struct.h"

/// Struct defined in srv/RequestAudioFocus in the package audio_msgs.
typedef struct audio_msgs__srv__RequestAudioFocus_Request
{
  /// 请求数据头
  std_msgs__msg__Header header;
  /// 请求焦点信息
  audio_msgs__msg__FocusRequester focus_requester;
} audio_msgs__srv__RequestAudioFocus_Request;

// Struct for a sequence of audio_msgs__srv__RequestAudioFocus_Request.
typedef struct audio_msgs__srv__RequestAudioFocus_Request__Sequence
{
  audio_msgs__srv__RequestAudioFocus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__srv__RequestAudioFocus_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'header'
// already included above
// #include "std_msgs/msg/detail/header__struct.h"
// Member 'focus_response'
#include "audio_msgs/msg/detail/focus_response__struct.h"

/// Struct defined in srv/RequestAudioFocus in the package audio_msgs.
typedef struct audio_msgs__srv__RequestAudioFocus_Response
{
  /// ----------------------------------------------------
  /// response
  /// 响应数据头
  std_msgs__msg__Header header;
  /// 请求结果
  audio_msgs__msg__FocusResponse focus_response;
} audio_msgs__srv__RequestAudioFocus_Response;

// Struct for a sequence of audio_msgs__srv__RequestAudioFocus_Response.
typedef struct audio_msgs__srv__RequestAudioFocus_Response__Sequence
{
  audio_msgs__srv__RequestAudioFocus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__srv__RequestAudioFocus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  audio_msgs__srv__RequestAudioFocus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  audio_msgs__srv__RequestAudioFocus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/RequestAudioFocus in the package audio_msgs.
typedef struct audio_msgs__srv__RequestAudioFocus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  audio_msgs__srv__RequestAudioFocus_Request__Sequence request;
  audio_msgs__srv__RequestAudioFocus_Response__Sequence response;
} audio_msgs__srv__RequestAudioFocus_Event;

// Struct for a sequence of audio_msgs__srv__RequestAudioFocus_Event.
typedef struct audio_msgs__srv__RequestAudioFocus_Event__Sequence
{
  audio_msgs__srv__RequestAudioFocus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__srv__RequestAudioFocus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__STRUCT_H_
