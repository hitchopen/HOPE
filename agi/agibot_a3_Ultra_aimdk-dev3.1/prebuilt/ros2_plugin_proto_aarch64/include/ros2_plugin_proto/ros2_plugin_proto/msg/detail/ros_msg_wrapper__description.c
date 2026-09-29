// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros2_plugin_proto:msg/RosMsgWrapper.idl
// generated code does not contain a copyright notice

#include "ros2_plugin_proto/msg/detail/ros_msg_wrapper__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros2_plugin_proto
const rosidl_type_hash_t *
ros2_plugin_proto__msg__RosMsgWrapper__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0xb7, 0xbb, 0x85, 0x14, 0x05, 0x1c, 0x40,
      0xb8, 0xff, 0xf8, 0xaa, 0x66, 0xf4, 0xb1, 0xb1,
      0x56, 0x64, 0xc4, 0x68, 0x54, 0xb7, 0x2c, 0x83,
      0xce, 0xa2, 0x7b, 0xdb, 0x57, 0x88, 0x5c, 0x8f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros2_plugin_proto__msg__RosMsgWrapper__TYPE_NAME[] = "ros2_plugin_proto/msg/RosMsgWrapper";

// Define type names, field names, and default values
static char ros2_plugin_proto__msg__RosMsgWrapper__FIELD_NAME__serialization_type[] = "serialization_type";
static char ros2_plugin_proto__msg__RosMsgWrapper__FIELD_NAME__context[] = "context";
static char ros2_plugin_proto__msg__RosMsgWrapper__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field ros2_plugin_proto__msg__RosMsgWrapper__FIELDS[] = {
  {
    {ros2_plugin_proto__msg__RosMsgWrapper__FIELD_NAME__serialization_type, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__msg__RosMsgWrapper__FIELD_NAME__context, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__msg__RosMsgWrapper__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2_plugin_proto__msg__RosMsgWrapper__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2_plugin_proto__msg__RosMsgWrapper__TYPE_NAME, 35, 35},
      {ros2_plugin_proto__msg__RosMsgWrapper__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string  serialization_type\n"
  "string[]  context\n"
  "byte[]  data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros2_plugin_proto__msg__RosMsgWrapper__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2_plugin_proto__msg__RosMsgWrapper__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 58, 58},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2_plugin_proto__msg__RosMsgWrapper__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2_plugin_proto__msg__RosMsgWrapper__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
