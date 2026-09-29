// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_msgs:msg/FocusResponse.idl
// generated code does not contain a copyright notice

#include "audio_msgs/msg/detail/focus_response__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__msg__FocusResponse__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5d, 0x6b, 0xd7, 0x18, 0x01, 0x80, 0xaa, 0xa5,
      0x9f, 0x77, 0x1e, 0xe6, 0x12, 0x95, 0xcb, 0xa7,
      0xbf, 0xe7, 0x65, 0xe4, 0x86, 0xe7, 0x57, 0x18,
      0x5f, 0xe8, 0x2a, 0x85, 0x59, 0x1f, 0x98, 0x8b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char audio_msgs__msg__FocusResponse__TYPE_NAME[] = "audio_msgs/msg/FocusResponse";

// Define type names, field names, and default values
static char audio_msgs__msg__FocusResponse__FIELD_NAME__pkg_name[] = "pkg_name";
static char audio_msgs__msg__FocusResponse__FIELD_NAME__focus_gain[] = "focus_gain";

static rosidl_runtime_c__type_description__Field audio_msgs__msg__FocusResponse__FIELDS[] = {
  {
    {audio_msgs__msg__FocusResponse__FIELD_NAME__pkg_name, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__FocusResponse__FIELD_NAME__focus_gain, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__msg__FocusResponse__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__msg__FocusResponse__TYPE_NAME, 28, 28},
      {audio_msgs__msg__FocusResponse__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe5\\xbf\\x85\\xe8\\xa6\\x81\\xe9\\xa1\\xb9\\xef\\xbc\\x8c\\xe6\\xa0\\x87\\xe8\\xaf\\x86\\xe8\\xb0\\x83\\xe7\\x94\\xa8\\xe6\\x96\\xb9\\xe6\\x9d\\xa5\\xe6\\xba\\x90\n"
  "string pkg_name\n"
  "\n"
  "# \\xe5\\xbf\\x85\\xe8\\xa6\\x81\\xe9\\xa1\\xb9\\xef\\xbc\\x8c\\xe7\\x84\\xa6\\xe7\\x82\\xb9\\xe7\\xbb\\x93\\xe6\\x9e\\x9c\n"
  "# true: \\xe8\\x8e\\xb7\\xe5\\x8f\\x96\\xe5\\x88\\xb0\\xe7\\x84\\xa6\\xe7\\x82\\xb9\n"
  "# false: \\xe4\\xb8\\xa2\\xe5\\xa4\\xb1\\xe7\\x84\\xa6\\xe7\\x82\\xb9\n"
  "bool focus_gain";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__msg__FocusResponse__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__msg__FocusResponse__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 85, 85},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__msg__FocusResponse__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__msg__FocusResponse__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
