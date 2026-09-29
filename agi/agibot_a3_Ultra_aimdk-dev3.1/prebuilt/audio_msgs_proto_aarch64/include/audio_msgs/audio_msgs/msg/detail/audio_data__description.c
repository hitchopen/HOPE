// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_msgs:msg/AudioData.idl
// generated code does not contain a copyright notice

#include "audio_msgs/msg/detail/audio_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__msg__AudioData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5b, 0x65, 0x35, 0x95, 0xec, 0x94, 0xc2, 0x29,
      0x7a, 0x67, 0x6f, 0x73, 0x89, 0x66, 0x5f, 0xf0,
      0xed, 0xda, 0x0b, 0x2f, 0x3a, 0x81, 0x85, 0x81,
      0xc1, 0x87, 0x4c, 0x55, 0xe9, 0x87, 0x33, 0xc9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char audio_msgs__msg__AudioData__TYPE_NAME[] = "audio_msgs/msg/AudioData";

// Define type names, field names, and default values
static char audio_msgs__msg__AudioData__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field audio_msgs__msg__AudioData__FIELDS[] = {
  {
    {audio_msgs__msg__AudioData__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__msg__AudioData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__msg__AudioData__TYPE_NAME, 24, 24},
      {audio_msgs__msg__AudioData__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8[] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__msg__AudioData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__msg__AudioData__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 12, 12},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__msg__AudioData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__msg__AudioData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
