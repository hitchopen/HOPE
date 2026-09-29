// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_msgs:msg/AudioInfo.idl
// generated code does not contain a copyright notice

#include "audio_msgs/msg/detail/audio_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__msg__AudioInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x68, 0xd8, 0x57, 0x36, 0x3a, 0x52, 0x73, 0x93,
      0xa3, 0x21, 0x6a, 0xd8, 0x92, 0x8c, 0x4a, 0x64,
      0x61, 0x4d, 0xf8, 0x31, 0xeb, 0xb5, 0x8f, 0xa2,
      0xc1, 0x4e, 0x28, 0xa4, 0x2a, 0x90, 0x77, 0x05,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char audio_msgs__msg__AudioInfo__TYPE_NAME[] = "audio_msgs/msg/AudioInfo";

// Define type names, field names, and default values
static char audio_msgs__msg__AudioInfo__FIELD_NAME__channels[] = "channels";
static char audio_msgs__msg__AudioInfo__FIELD_NAME__sample_rate[] = "sample_rate";
static char audio_msgs__msg__AudioInfo__FIELD_NAME__sample_format[] = "sample_format";
static char audio_msgs__msg__AudioInfo__FIELD_NAME__coding_format[] = "coding_format";

static rosidl_runtime_c__type_description__Field audio_msgs__msg__AudioInfo__FIELDS[] = {
  {
    {audio_msgs__msg__AudioInfo__FIELD_NAME__channels, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioInfo__FIELD_NAME__sample_rate, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioInfo__FIELD_NAME__sample_format, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioInfo__FIELD_NAME__coding_format, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__msg__AudioInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__msg__AudioInfo__TYPE_NAME, 24, 24},
      {audio_msgs__msg__AudioInfo__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Number of channels\n"
  "uint8 channels\n"
  "# Sampling rate [Hz]\n"
  "uint32 sample_rate\n"
  "# Audio format (e.g. S16LE)\n"
  "string sample_format\n"
  "# Audio coding format (e.g. pcm, wave, opus)\n"
  "string coding_format";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__msg__AudioInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__msg__AudioInfo__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 190, 190},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__msg__AudioInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__msg__AudioInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
