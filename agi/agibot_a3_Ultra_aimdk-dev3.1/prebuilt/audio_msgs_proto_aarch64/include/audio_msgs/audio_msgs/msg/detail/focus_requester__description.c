// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_msgs:msg/FocusRequester.idl
// generated code does not contain a copyright notice

#include "audio_msgs/msg/detail/focus_requester__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__msg__FocusRequester__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x12, 0x2a, 0x06, 0xff, 0x96, 0x16, 0x54, 0x37,
      0xf9, 0x4a, 0x8b, 0x7b, 0xac, 0xe9, 0x45, 0x14,
      0x91, 0x85, 0x44, 0xec, 0x72, 0x89, 0x7c, 0xe1,
      0x09, 0xb3, 0xa8, 0x8d, 0x25, 0x3c, 0x3f, 0xd2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char audio_msgs__msg__FocusRequester__TYPE_NAME[] = "audio_msgs/msg/FocusRequester";

// Define type names, field names, and default values
static char audio_msgs__msg__FocusRequester__FIELD_NAME__pkg_name[] = "pkg_name";
static char audio_msgs__msg__FocusRequester__FIELD_NAME__priority[] = "priority";
static char audio_msgs__msg__FocusRequester__FIELD_NAME__priority_weight[] = "priority_weight";

static rosidl_runtime_c__type_description__Field audio_msgs__msg__FocusRequester__FIELDS[] = {
  {
    {audio_msgs__msg__FocusRequester__FIELD_NAME__pkg_name, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__FocusRequester__FIELD_NAME__priority, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__FocusRequester__FIELD_NAME__priority_weight, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__msg__FocusRequester__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__msg__FocusRequester__TYPE_NAME, 29, 29},
      {audio_msgs__msg__FocusRequester__FIELDS, 3, 3},
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
  "# \\xe5\\xbf\\x85\\xe8\\xa6\\x81\\xe9\\xa1\\xb9\\xef\\xbc\\x8c\\xe4\\xbc\\x98\\xe5\\x85\\x88\\xe7\\xba\\xa7(1~10, \\xe9\\xbb\\x98\\xe8\\xae\\xa46)\n"
  "uint32 priority\n"
  "\n"
  "# \\xe5\\x8f\\xaf\\xe9\\x80\\x89\\xe9\\xa1\\xb9\\xef\\xbc\\x8c(1~100) priority + priority_weight% is final priority\n"
  "uint32 priority_weight";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__msg__FocusRequester__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__msg__FocusRequester__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 151, 151},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__msg__FocusRequester__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__msg__FocusRequester__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
