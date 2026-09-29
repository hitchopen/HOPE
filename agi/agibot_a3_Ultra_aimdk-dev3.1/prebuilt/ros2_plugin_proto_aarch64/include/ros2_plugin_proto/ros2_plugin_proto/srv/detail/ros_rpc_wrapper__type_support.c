// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_plugin_proto:srv/RosRpcWrapper.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__rosidl_typesupport_introspection_c.h"
#include "ros2_plugin_proto/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"
#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.h"


// Include directives for member types
// Member `serialization_type`
// Member `context`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_plugin_proto__srv__RosRpcWrapper_Request__init(message_memory);
}

void ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_fini_function(void * message_memory)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Request__fini(message_memory);
}

size_t ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Request__context(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Request__context(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Request__context(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Request__context(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Request__context(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Request__context(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Request__context(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Request__context(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Request__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__octet__Sequence * member =
    (const rosidl_runtime_c__octet__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Request__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__octet__Sequence * member =
    (const rosidl_runtime_c__octet__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Request__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__octet__Sequence * member =
    (rosidl_runtime_c__octet__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Request__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Request__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Request__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Request__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Request__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__octet__Sequence * member =
    (rosidl_runtime_c__octet__Sequence *)(untyped_member);
  rosidl_runtime_c__octet__Sequence__fini(member);
  return rosidl_runtime_c__octet__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_member_array[3] = {
  {
    "serialization_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Request, serialization_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "context",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Request, context),  // bytes offset in struct
    NULL,  // default value
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Request__context,  // size() function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Request__context,  // get_const(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Request__context,  // get(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Request__context,  // fetch(index, &value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Request__context,  // assign(index, value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Request__context  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Request, data),  // bytes offset in struct
    NULL,  // default value
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Request__data,  // size() function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Request__data,  // get_const(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Request__data,  // get(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Request__data,  // fetch(index, &value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Request__data,  // assign(index, value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Request__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_members = {
  "ros2_plugin_proto__srv",  // message namespace
  "RosRpcWrapper_Request",  // message name
  3,  // number of fields
  sizeof(ros2_plugin_proto__srv__RosRpcWrapper_Request),
  false,  // has_any_key_member_
  ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_member_array,  // message members
  ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_type_support_handle = {
  0,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_members,
  get_message_typesupport_handle_function,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_plugin_proto
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Request)() {
  if (!ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_type_support_handle.typesupport_identifier) {
    ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros2_plugin_proto/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.h"


// Include directives for member types
// Member `serialization_type`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `data`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_plugin_proto__srv__RosRpcWrapper_Response__init(message_memory);
}

void ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_fini_function(void * message_memory)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Response__fini(message_memory);
}

size_t ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Response__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__octet__Sequence * member =
    (const rosidl_runtime_c__octet__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Response__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__octet__Sequence * member =
    (const rosidl_runtime_c__octet__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Response__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__octet__Sequence * member =
    (rosidl_runtime_c__octet__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Response__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Response__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Response__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Response__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Response__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__octet__Sequence * member =
    (rosidl_runtime_c__octet__Sequence *)(untyped_member);
  rosidl_runtime_c__octet__Sequence__fini(member);
  return rosidl_runtime_c__octet__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_member_array[3] = {
  {
    "code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Response, code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "serialization_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Response, serialization_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Response, data),  // bytes offset in struct
    NULL,  // default value
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Response__data,  // size() function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Response__data,  // get_const(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Response__data,  // get(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Response__data,  // fetch(index, &value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Response__data,  // assign(index, value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Response__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_members = {
  "ros2_plugin_proto__srv",  // message namespace
  "RosRpcWrapper_Response",  // message name
  3,  // number of fields
  sizeof(ros2_plugin_proto__srv__RosRpcWrapper_Response),
  false,  // has_any_key_member_
  ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_member_array,  // message members
  ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_type_support_handle = {
  0,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_members,
  get_message_typesupport_handle_function,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_plugin_proto
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Response)() {
  if (!ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_type_support_handle.typesupport_identifier) {
    ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros2_plugin_proto/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "ros2_plugin_proto/srv/ros_rpc_wrapper.h"
// Member `request`
// Member `response`
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_plugin_proto__srv__RosRpcWrapper_Event__init(message_memory);
}

void ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_fini_function(void * message_memory)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Event__fini(message_memory);
}

size_t ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Event__request(
  const void * untyped_member)
{
  const ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence * member =
    (const ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Event__request(
  const void * untyped_member, size_t index)
{
  const ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence * member =
    (const ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Event__request(
  void * untyped_member, size_t index)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence * member =
    (ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_plugin_proto__srv__RosRpcWrapper_Request * item =
    ((const ros2_plugin_proto__srv__RosRpcWrapper_Request *)
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Event__request(untyped_member, index));
  ros2_plugin_proto__srv__RosRpcWrapper_Request * value =
    (ros2_plugin_proto__srv__RosRpcWrapper_Request *)(untyped_value);
  *value = *item;
}

void ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Request * item =
    ((ros2_plugin_proto__srv__RosRpcWrapper_Request *)
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Event__request(untyped_member, index));
  const ros2_plugin_proto__srv__RosRpcWrapper_Request * value =
    (const ros2_plugin_proto__srv__RosRpcWrapper_Request *)(untyped_value);
  *item = *value;
}

bool ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Event__request(
  void * untyped_member, size_t size)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence * member =
    (ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence *)(untyped_member);
  ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence__fini(member);
  return ros2_plugin_proto__srv__RosRpcWrapper_Request__Sequence__init(member, size);
}

size_t ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Event__response(
  const void * untyped_member)
{
  const ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence * member =
    (const ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Event__response(
  const void * untyped_member, size_t index)
{
  const ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence * member =
    (const ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Event__response(
  void * untyped_member, size_t index)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence * member =
    (ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_plugin_proto__srv__RosRpcWrapper_Response * item =
    ((const ros2_plugin_proto__srv__RosRpcWrapper_Response *)
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Event__response(untyped_member, index));
  ros2_plugin_proto__srv__RosRpcWrapper_Response * value =
    (ros2_plugin_proto__srv__RosRpcWrapper_Response *)(untyped_value);
  *value = *item;
}

void ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Response * item =
    ((ros2_plugin_proto__srv__RosRpcWrapper_Response *)
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Event__response(untyped_member, index));
  const ros2_plugin_proto__srv__RosRpcWrapper_Response * value =
    (const ros2_plugin_proto__srv__RosRpcWrapper_Response *)(untyped_value);
  *item = *value;
}

bool ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Event__response(
  void * untyped_member, size_t size)
{
  ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence * member =
    (ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence *)(untyped_member);
  ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence__fini(member);
  return ros2_plugin_proto__srv__RosRpcWrapper_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Event, info),  // bytes offset in struct
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
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Event, request),  // bytes offset in struct
    NULL,  // default value
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Event__request,  // size() function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Event__request,  // get_const(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Event__request,  // get(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Event__request,  // fetch(index, &value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Event__request,  // assign(index, value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Event__request  // resize(index) function pointer
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
    offsetof(ros2_plugin_proto__srv__RosRpcWrapper_Event, response),  // bytes offset in struct
    NULL,  // default value
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__size_function__RosRpcWrapper_Event__response,  // size() function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_const_function__RosRpcWrapper_Event__response,  // get_const(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__get_function__RosRpcWrapper_Event__response,  // get(index) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__fetch_function__RosRpcWrapper_Event__response,  // fetch(index, &value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__assign_function__RosRpcWrapper_Event__response,  // assign(index, value) function pointer
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__resize_function__RosRpcWrapper_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_members = {
  "ros2_plugin_proto__srv",  // message namespace
  "RosRpcWrapper_Event",  // message name
  3,  // number of fields
  sizeof(ros2_plugin_proto__srv__RosRpcWrapper_Event),
  false,  // has_any_key_member_
  ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_member_array,  // message members
  ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_type_support_handle = {
  0,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_members,
  get_message_typesupport_handle_function,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_plugin_proto
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Event)() {
  ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Request)();
  ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Response)();
  if (!ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_type_support_handle.typesupport_identifier) {
    ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ros2_plugin_proto/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_service_members = {
  "ros2_plugin_proto__srv",  // service namespace
  "RosRpcWrapper",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_type_support_handle,
  NULL,  // response message
  // ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_type_support_handle
  NULL  // event_message
  // ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_type_support_handle
};


static rosidl_service_type_support_t ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_service_type_support_handle = {
  0,
  &ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_service_members,
  get_service_typesupport_handle_function,
  &ros2_plugin_proto__srv__RosRpcWrapper_Request__rosidl_typesupport_introspection_c__RosRpcWrapper_Request_message_type_support_handle,
  &ros2_plugin_proto__srv__RosRpcWrapper_Response__rosidl_typesupport_introspection_c__RosRpcWrapper_Response_message_type_support_handle,
  &ros2_plugin_proto__srv__RosRpcWrapper_Event__rosidl_typesupport_introspection_c__RosRpcWrapper_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2_plugin_proto,
    srv,
    RosRpcWrapper
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2_plugin_proto,
    srv,
    RosRpcWrapper
  ),
  &ros2_plugin_proto__srv__RosRpcWrapper__get_type_hash,
  &ros2_plugin_proto__srv__RosRpcWrapper__get_type_description,
  &ros2_plugin_proto__srv__RosRpcWrapper__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_plugin_proto
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper)(void) {
  if (!ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_service_type_support_handle.typesupport_identifier) {
    ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_plugin_proto, srv, RosRpcWrapper_Event)()->data;
  }

  return &ros2_plugin_proto__srv__detail__ros_rpc_wrapper__rosidl_typesupport_introspection_c__RosRpcWrapper_service_type_support_handle;
}
