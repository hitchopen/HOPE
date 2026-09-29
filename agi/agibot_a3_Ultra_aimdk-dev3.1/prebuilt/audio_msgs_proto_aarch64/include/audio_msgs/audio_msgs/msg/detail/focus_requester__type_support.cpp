// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from audio_msgs:msg/FocusRequester.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "audio_msgs/msg/detail/focus_requester__functions.h"
#include "audio_msgs/msg/detail/focus_requester__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace audio_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FocusRequester_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) audio_msgs::msg::FocusRequester(_init);
}

void FocusRequester_fini_function(void * message_memory)
{
  auto typed_message = static_cast<audio_msgs::msg::FocusRequester *>(message_memory);
  typed_message->~FocusRequester();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FocusRequester_message_member_array[3] = {
  {
    "pkg_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs::msg::FocusRequester, pkg_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "priority",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs::msg::FocusRequester, priority),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "priority_weight",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs::msg::FocusRequester, priority_weight),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FocusRequester_message_members = {
  "audio_msgs::msg",  // message namespace
  "FocusRequester",  // message name
  3,  // number of fields
  sizeof(audio_msgs::msg::FocusRequester),
  false,  // has_any_key_member_
  FocusRequester_message_member_array,  // message members
  FocusRequester_init_function,  // function to initialize message memory (memory has to be allocated)
  FocusRequester_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FocusRequester_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FocusRequester_message_members,
  get_message_typesupport_handle_function,
  &audio_msgs__msg__FocusRequester__get_type_hash,
  &audio_msgs__msg__FocusRequester__get_type_description,
  &audio_msgs__msg__FocusRequester__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace audio_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<audio_msgs::msg::FocusRequester>()
{
  return &::audio_msgs::msg::rosidl_typesupport_introspection_cpp::FocusRequester_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, audio_msgs, msg, FocusRequester)() {
  return &::audio_msgs::msg::rosidl_typesupport_introspection_cpp::FocusRequester_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
