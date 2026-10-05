// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from audio_msgs:srv/AbandonAudioFocus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "audio_msgs/srv/detail/abandon_audio_focus__rosidl_typesupport_introspection_c.h"
#include "audio_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "audio_msgs/srv/detail/abandon_audio_focus__functions.h"
#include "audio_msgs/srv/detail/abandon_audio_focus__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `focus_requester`
#include "audio_msgs/msg/focus_requester.h"
// Member `focus_requester`
#include "audio_msgs/msg/detail/focus_requester__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  audio_msgs__srv__AbandonAudioFocus_Request__init(message_memory);
}

void audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_fini_function(void * message_memory)
{
  audio_msgs__srv__AbandonAudioFocus_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__srv__AbandonAudioFocus_Request, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "focus_requester",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__srv__AbandonAudioFocus_Request, focus_requester),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_members = {
  "audio_msgs__srv",  // message namespace
  "AbandonAudioFocus_Request",  // message name
  2,  // number of fields
  sizeof(audio_msgs__srv__AbandonAudioFocus_Request),
  false,  // has_any_key_member_
  audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_member_array,  // message members
  audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_type_support_handle = {
  0,
  &audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_members,
  get_message_typesupport_handle_function,
  &audio_msgs__srv__AbandonAudioFocus_Request__get_type_hash,
  &audio_msgs__srv__AbandonAudioFocus_Request__get_type_description,
  &audio_msgs__srv__AbandonAudioFocus_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_audio_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Request)() {
  audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, msg, FocusRequester)();
  if (!audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_type_support_handle.typesupport_identifier) {
    audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__rosidl_typesupport_introspection_c.h"
// already included above
// #include "audio_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__functions.h"
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__struct.h"


// Include directives for member types
// Member `header`
// already included above
// #include "std_msgs/msg/header.h"
// Member `header`
// already included above
// #include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `focus_response`
#include "audio_msgs/msg/focus_response.h"
// Member `focus_response`
#include "audio_msgs/msg/detail/focus_response__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  audio_msgs__srv__AbandonAudioFocus_Response__init(message_memory);
}

void audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_fini_function(void * message_memory)
{
  audio_msgs__srv__AbandonAudioFocus_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__srv__AbandonAudioFocus_Response, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "focus_response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__srv__AbandonAudioFocus_Response, focus_response),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_members = {
  "audio_msgs__srv",  // message namespace
  "AbandonAudioFocus_Response",  // message name
  2,  // number of fields
  sizeof(audio_msgs__srv__AbandonAudioFocus_Response),
  false,  // has_any_key_member_
  audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_member_array,  // message members
  audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_type_support_handle = {
  0,
  &audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_members,
  get_message_typesupport_handle_function,
  &audio_msgs__srv__AbandonAudioFocus_Response__get_type_hash,
  &audio_msgs__srv__AbandonAudioFocus_Response__get_type_description,
  &audio_msgs__srv__AbandonAudioFocus_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_audio_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Response)() {
  audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, msg, FocusResponse)();
  if (!audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_type_support_handle.typesupport_identifier) {
    audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__rosidl_typesupport_introspection_c.h"
// already included above
// #include "audio_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__functions.h"
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "audio_msgs/srv/abandon_audio_focus.h"
// Member `request`
// Member `response`
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  audio_msgs__srv__AbandonAudioFocus_Event__init(message_memory);
}

void audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_fini_function(void * message_memory)
{
  audio_msgs__srv__AbandonAudioFocus_Event__fini(message_memory);
}

size_t audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__size_function__AbandonAudioFocus_Event__request(
  const void * untyped_member)
{
  const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * member =
    (const audio_msgs__srv__AbandonAudioFocus_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_const_function__AbandonAudioFocus_Event__request(
  const void * untyped_member, size_t index)
{
  const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * member =
    (const audio_msgs__srv__AbandonAudioFocus_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_function__AbandonAudioFocus_Event__request(
  void * untyped_member, size_t index)
{
  audio_msgs__srv__AbandonAudioFocus_Request__Sequence * member =
    (audio_msgs__srv__AbandonAudioFocus_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__fetch_function__AbandonAudioFocus_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const audio_msgs__srv__AbandonAudioFocus_Request * item =
    ((const audio_msgs__srv__AbandonAudioFocus_Request *)
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_const_function__AbandonAudioFocus_Event__request(untyped_member, index));
  audio_msgs__srv__AbandonAudioFocus_Request * value =
    (audio_msgs__srv__AbandonAudioFocus_Request *)(untyped_value);
  *value = *item;
}

void audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__assign_function__AbandonAudioFocus_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  audio_msgs__srv__AbandonAudioFocus_Request * item =
    ((audio_msgs__srv__AbandonAudioFocus_Request *)
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_function__AbandonAudioFocus_Event__request(untyped_member, index));
  const audio_msgs__srv__AbandonAudioFocus_Request * value =
    (const audio_msgs__srv__AbandonAudioFocus_Request *)(untyped_value);
  *item = *value;
}

bool audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__resize_function__AbandonAudioFocus_Event__request(
  void * untyped_member, size_t size)
{
  audio_msgs__srv__AbandonAudioFocus_Request__Sequence * member =
    (audio_msgs__srv__AbandonAudioFocus_Request__Sequence *)(untyped_member);
  audio_msgs__srv__AbandonAudioFocus_Request__Sequence__fini(member);
  return audio_msgs__srv__AbandonAudioFocus_Request__Sequence__init(member, size);
}

size_t audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__size_function__AbandonAudioFocus_Event__response(
  const void * untyped_member)
{
  const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * member =
    (const audio_msgs__srv__AbandonAudioFocus_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_const_function__AbandonAudioFocus_Event__response(
  const void * untyped_member, size_t index)
{
  const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * member =
    (const audio_msgs__srv__AbandonAudioFocus_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_function__AbandonAudioFocus_Event__response(
  void * untyped_member, size_t index)
{
  audio_msgs__srv__AbandonAudioFocus_Response__Sequence * member =
    (audio_msgs__srv__AbandonAudioFocus_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__fetch_function__AbandonAudioFocus_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const audio_msgs__srv__AbandonAudioFocus_Response * item =
    ((const audio_msgs__srv__AbandonAudioFocus_Response *)
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_const_function__AbandonAudioFocus_Event__response(untyped_member, index));
  audio_msgs__srv__AbandonAudioFocus_Response * value =
    (audio_msgs__srv__AbandonAudioFocus_Response *)(untyped_value);
  *value = *item;
}

void audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__assign_function__AbandonAudioFocus_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  audio_msgs__srv__AbandonAudioFocus_Response * item =
    ((audio_msgs__srv__AbandonAudioFocus_Response *)
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_function__AbandonAudioFocus_Event__response(untyped_member, index));
  const audio_msgs__srv__AbandonAudioFocus_Response * value =
    (const audio_msgs__srv__AbandonAudioFocus_Response *)(untyped_value);
  *item = *value;
}

bool audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__resize_function__AbandonAudioFocus_Event__response(
  void * untyped_member, size_t size)
{
  audio_msgs__srv__AbandonAudioFocus_Response__Sequence * member =
    (audio_msgs__srv__AbandonAudioFocus_Response__Sequence *)(untyped_member);
  audio_msgs__srv__AbandonAudioFocus_Response__Sequence__fini(member);
  return audio_msgs__srv__AbandonAudioFocus_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__srv__AbandonAudioFocus_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(audio_msgs__srv__AbandonAudioFocus_Event, request),  // bytes offset in struct
    NULL,  // default value
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__size_function__AbandonAudioFocus_Event__request,  // size() function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_const_function__AbandonAudioFocus_Event__request,  // get_const(index) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_function__AbandonAudioFocus_Event__request,  // get(index) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__fetch_function__AbandonAudioFocus_Event__request,  // fetch(index, &value) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__assign_function__AbandonAudioFocus_Event__request,  // assign(index, value) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__resize_function__AbandonAudioFocus_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(audio_msgs__srv__AbandonAudioFocus_Event, response),  // bytes offset in struct
    NULL,  // default value
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__size_function__AbandonAudioFocus_Event__response,  // size() function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_const_function__AbandonAudioFocus_Event__response,  // get_const(index) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__get_function__AbandonAudioFocus_Event__response,  // get(index) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__fetch_function__AbandonAudioFocus_Event__response,  // fetch(index, &value) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__assign_function__AbandonAudioFocus_Event__response,  // assign(index, value) function pointer
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__resize_function__AbandonAudioFocus_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_members = {
  "audio_msgs__srv",  // message namespace
  "AbandonAudioFocus_Event",  // message name
  3,  // number of fields
  sizeof(audio_msgs__srv__AbandonAudioFocus_Event),
  false,  // has_any_key_member_
  audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_member_array,  // message members
  audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_type_support_handle = {
  0,
  &audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_members,
  get_message_typesupport_handle_function,
  &audio_msgs__srv__AbandonAudioFocus_Event__get_type_hash,
  &audio_msgs__srv__AbandonAudioFocus_Event__get_type_description,
  &audio_msgs__srv__AbandonAudioFocus_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_audio_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Event)() {
  audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Request)();
  audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Response)();
  if (!audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_type_support_handle.typesupport_identifier) {
    audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "audio_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_service_members = {
  "audio_msgs__srv",  // service namespace
  "AbandonAudioFocus",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_type_support_handle,
  NULL,  // response message
  // audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_type_support_handle
  NULL  // event_message
  // audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_type_support_handle
};


static rosidl_service_type_support_t audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_service_type_support_handle = {
  0,
  &audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_service_members,
  get_service_typesupport_handle_function,
  &audio_msgs__srv__AbandonAudioFocus_Request__rosidl_typesupport_introspection_c__AbandonAudioFocus_Request_message_type_support_handle,
  &audio_msgs__srv__AbandonAudioFocus_Response__rosidl_typesupport_introspection_c__AbandonAudioFocus_Response_message_type_support_handle,
  &audio_msgs__srv__AbandonAudioFocus_Event__rosidl_typesupport_introspection_c__AbandonAudioFocus_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    audio_msgs,
    srv,
    AbandonAudioFocus
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    audio_msgs,
    srv,
    AbandonAudioFocus
  ),
  &audio_msgs__srv__AbandonAudioFocus__get_type_hash,
  &audio_msgs__srv__AbandonAudioFocus__get_type_description,
  &audio_msgs__srv__AbandonAudioFocus__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_audio_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus)(void) {
  if (!audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_service_type_support_handle.typesupport_identifier) {
    audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, srv, AbandonAudioFocus_Event)()->data;
  }

  return &audio_msgs__srv__detail__abandon_audio_focus__rosidl_typesupport_introspection_c__AbandonAudioFocus_service_type_support_handle;
}
