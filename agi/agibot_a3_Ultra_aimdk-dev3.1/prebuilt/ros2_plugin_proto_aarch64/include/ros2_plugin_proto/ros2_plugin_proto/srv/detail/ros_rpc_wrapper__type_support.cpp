// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros2_plugin_proto:srv/RosRpcWrapper.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"
#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_plugin_proto
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RosRpcWrapper_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_plugin_proto::srv::RosRpcWrapper_Request(_init);
}

void RosRpcWrapper_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_plugin_proto::srv::RosRpcWrapper_Request *>(message_memory);
  typed_message->~RosRpcWrapper_Request();
}

size_t size_function__RosRpcWrapper_Request__context(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RosRpcWrapper_Request__context(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RosRpcWrapper_Request__context(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RosRpcWrapper_Request__context(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RosRpcWrapper_Request__context(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RosRpcWrapper_Request__context(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RosRpcWrapper_Request__context(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RosRpcWrapper_Request__context(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RosRpcWrapper_Request__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<unsigned char> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RosRpcWrapper_Request__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<unsigned char> *>(untyped_member);
  return &member[index];
}

void * get_function__RosRpcWrapper_Request__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<unsigned char> *>(untyped_member);
  return &member[index];
}

void fetch_function__RosRpcWrapper_Request__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const unsigned char *>(
    get_const_function__RosRpcWrapper_Request__data(untyped_member, index));
  auto & value = *reinterpret_cast<unsigned char *>(untyped_value);
  value = item;
}

void assign_function__RosRpcWrapper_Request__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<unsigned char *>(
    get_function__RosRpcWrapper_Request__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const unsigned char *>(untyped_value);
  item = value;
}

void resize_function__RosRpcWrapper_Request__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<unsigned char> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RosRpcWrapper_Request_message_member_array[3] = {
  {
    "serialization_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Request, serialization_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "context",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Request, context),  // bytes offset in struct
    nullptr,  // default value
    size_function__RosRpcWrapper_Request__context,  // size() function pointer
    get_const_function__RosRpcWrapper_Request__context,  // get_const(index) function pointer
    get_function__RosRpcWrapper_Request__context,  // get(index) function pointer
    fetch_function__RosRpcWrapper_Request__context,  // fetch(index, &value) function pointer
    assign_function__RosRpcWrapper_Request__context,  // assign(index, value) function pointer
    resize_function__RosRpcWrapper_Request__context  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Request, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__RosRpcWrapper_Request__data,  // size() function pointer
    get_const_function__RosRpcWrapper_Request__data,  // get_const(index) function pointer
    get_function__RosRpcWrapper_Request__data,  // get(index) function pointer
    fetch_function__RosRpcWrapper_Request__data,  // fetch(index, &value) function pointer
    assign_function__RosRpcWrapper_Request__data,  // assign(index, value) function pointer
    resize_function__RosRpcWrapper_Request__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RosRpcWrapper_Request_message_members = {
  "ros2_plugin_proto::srv",  // message namespace
  "RosRpcWrapper_Request",  // message name
  3,  // number of fields
  sizeof(ros2_plugin_proto::srv::RosRpcWrapper_Request),
  false,  // has_any_key_member_
  RosRpcWrapper_Request_message_member_array,  // message members
  RosRpcWrapper_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  RosRpcWrapper_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RosRpcWrapper_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RosRpcWrapper_Request_message_members,
  get_message_typesupport_handle_function,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros2_plugin_proto


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Request>()
{
  return &::ros2_plugin_proto::srv::rosidl_typesupport_introspection_cpp::RosRpcWrapper_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_plugin_proto, srv, RosRpcWrapper_Request)() {
  return &::ros2_plugin_proto::srv::rosidl_typesupport_introspection_cpp::RosRpcWrapper_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_plugin_proto
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RosRpcWrapper_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_plugin_proto::srv::RosRpcWrapper_Response(_init);
}

void RosRpcWrapper_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_plugin_proto::srv::RosRpcWrapper_Response *>(message_memory);
  typed_message->~RosRpcWrapper_Response();
}

size_t size_function__RosRpcWrapper_Response__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<unsigned char> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RosRpcWrapper_Response__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<unsigned char> *>(untyped_member);
  return &member[index];
}

void * get_function__RosRpcWrapper_Response__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<unsigned char> *>(untyped_member);
  return &member[index];
}

void fetch_function__RosRpcWrapper_Response__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const unsigned char *>(
    get_const_function__RosRpcWrapper_Response__data(untyped_member, index));
  auto & value = *reinterpret_cast<unsigned char *>(untyped_value);
  value = item;
}

void assign_function__RosRpcWrapper_Response__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<unsigned char *>(
    get_function__RosRpcWrapper_Response__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const unsigned char *>(untyped_value);
  item = value;
}

void resize_function__RosRpcWrapper_Response__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<unsigned char> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RosRpcWrapper_Response_message_member_array[3] = {
  {
    "code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Response, code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "serialization_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Response, serialization_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Response, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__RosRpcWrapper_Response__data,  // size() function pointer
    get_const_function__RosRpcWrapper_Response__data,  // get_const(index) function pointer
    get_function__RosRpcWrapper_Response__data,  // get(index) function pointer
    fetch_function__RosRpcWrapper_Response__data,  // fetch(index, &value) function pointer
    assign_function__RosRpcWrapper_Response__data,  // assign(index, value) function pointer
    resize_function__RosRpcWrapper_Response__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RosRpcWrapper_Response_message_members = {
  "ros2_plugin_proto::srv",  // message namespace
  "RosRpcWrapper_Response",  // message name
  3,  // number of fields
  sizeof(ros2_plugin_proto::srv::RosRpcWrapper_Response),
  false,  // has_any_key_member_
  RosRpcWrapper_Response_message_member_array,  // message members
  RosRpcWrapper_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  RosRpcWrapper_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RosRpcWrapper_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RosRpcWrapper_Response_message_members,
  get_message_typesupport_handle_function,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros2_plugin_proto


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Response>()
{
  return &::ros2_plugin_proto::srv::rosidl_typesupport_introspection_cpp::RosRpcWrapper_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_plugin_proto, srv, RosRpcWrapper_Response)() {
  return &::ros2_plugin_proto::srv::rosidl_typesupport_introspection_cpp::RosRpcWrapper_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_plugin_proto
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RosRpcWrapper_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_plugin_proto::srv::RosRpcWrapper_Event(_init);
}

void RosRpcWrapper_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_plugin_proto::srv::RosRpcWrapper_Event *>(message_memory);
  typed_message->~RosRpcWrapper_Event();
}

size_t size_function__RosRpcWrapper_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RosRpcWrapper_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__RosRpcWrapper_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__RosRpcWrapper_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros2_plugin_proto::srv::RosRpcWrapper_Request *>(
    get_const_function__RosRpcWrapper_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<ros2_plugin_proto::srv::RosRpcWrapper_Request *>(untyped_value);
  value = item;
}

void assign_function__RosRpcWrapper_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros2_plugin_proto::srv::RosRpcWrapper_Request *>(
    get_function__RosRpcWrapper_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros2_plugin_proto::srv::RosRpcWrapper_Request *>(untyped_value);
  item = value;
}

void resize_function__RosRpcWrapper_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RosRpcWrapper_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RosRpcWrapper_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__RosRpcWrapper_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__RosRpcWrapper_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros2_plugin_proto::srv::RosRpcWrapper_Response *>(
    get_const_function__RosRpcWrapper_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<ros2_plugin_proto::srv::RosRpcWrapper_Response *>(untyped_value);
  value = item;
}

void assign_function__RosRpcWrapper_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros2_plugin_proto::srv::RosRpcWrapper_Response *>(
    get_function__RosRpcWrapper_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros2_plugin_proto::srv::RosRpcWrapper_Response *>(untyped_value);
  item = value;
}

void resize_function__RosRpcWrapper_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros2_plugin_proto::srv::RosRpcWrapper_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RosRpcWrapper_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__RosRpcWrapper_Event__request,  // size() function pointer
    get_const_function__RosRpcWrapper_Event__request,  // get_const(index) function pointer
    get_function__RosRpcWrapper_Event__request,  // get(index) function pointer
    fetch_function__RosRpcWrapper_Event__request,  // fetch(index, &value) function pointer
    assign_function__RosRpcWrapper_Event__request,  // assign(index, value) function pointer
    resize_function__RosRpcWrapper_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(ros2_plugin_proto::srv::RosRpcWrapper_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__RosRpcWrapper_Event__response,  // size() function pointer
    get_const_function__RosRpcWrapper_Event__response,  // get_const(index) function pointer
    get_function__RosRpcWrapper_Event__response,  // get(index) function pointer
    fetch_function__RosRpcWrapper_Event__response,  // fetch(index, &value) function pointer
    assign_function__RosRpcWrapper_Event__response,  // assign(index, value) function pointer
    resize_function__RosRpcWrapper_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RosRpcWrapper_Event_message_members = {
  "ros2_plugin_proto::srv",  // message namespace
  "RosRpcWrapper_Event",  // message name
  3,  // number of fields
  sizeof(ros2_plugin_proto::srv::RosRpcWrapper_Event),
  false,  // has_any_key_member_
  RosRpcWrapper_Event_message_member_array,  // message members
  RosRpcWrapper_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  RosRpcWrapper_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RosRpcWrapper_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RosRpcWrapper_Event_message_members,
  get_message_typesupport_handle_function,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros2_plugin_proto


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Event>()
{
  return &::ros2_plugin_proto::srv::rosidl_typesupport_introspection_cpp::RosRpcWrapper_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_plugin_proto, srv, RosRpcWrapper_Event)() {
  return &::ros2_plugin_proto::srv::rosidl_typesupport_introspection_cpp::RosRpcWrapper_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace ros2_plugin_proto
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers RosRpcWrapper_service_members = {
  "ros2_plugin_proto::srv",  // service namespace
  "RosRpcWrapper",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t RosRpcWrapper_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RosRpcWrapper_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<ros2_plugin_proto::srv::RosRpcWrapper>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<ros2_plugin_proto::srv::RosRpcWrapper>,
  &ros2_plugin_proto__srv__RosRpcWrapper__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros2_plugin_proto


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::ros2_plugin_proto::srv::rosidl_typesupport_introspection_cpp::RosRpcWrapper_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros2_plugin_proto::srv::RosRpcWrapper_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros2_plugin_proto::srv::RosRpcWrapper_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros2_plugin_proto::srv::RosRpcWrapper_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_plugin_proto, srv, RosRpcWrapper)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper>();
}

#ifdef __cplusplus
}
#endif
