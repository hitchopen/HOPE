// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_msgs:msg/AudioPlayback.idl
// generated code does not contain a copyright notice

#include "audio_msgs/msg/detail/audio_playback__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__msg__AudioPlayback__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd4, 0x7c, 0x13, 0x35, 0x50, 0x37, 0xd2, 0x75,
      0xc4, 0x99, 0xe3, 0xad, 0xf2, 0x06, 0xfc, 0xfb,
      0x30, 0xd6, 0xa0, 0x3e, 0x57, 0x1e, 0x79, 0xbc,
      0x1c, 0x38, 0x74, 0x02, 0xcd, 0x67, 0xb0, 0xd7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "audio_msgs/msg/detail/audio_data__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "audio_msgs/msg/detail/audio_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t audio_msgs__msg__AudioData__EXPECTED_HASH = {1, {
    0x5b, 0x65, 0x35, 0x95, 0xec, 0x94, 0xc2, 0x29,
    0x7a, 0x67, 0x6f, 0x73, 0x89, 0x66, 0x5f, 0xf0,
    0xed, 0xda, 0x0b, 0x2f, 0x3a, 0x81, 0x85, 0x81,
    0xc1, 0x87, 0x4c, 0x55, 0xe9, 0x87, 0x33, 0xc9,
  }};
static const rosidl_type_hash_t audio_msgs__msg__AudioInfo__EXPECTED_HASH = {1, {
    0x68, 0xd8, 0x57, 0x36, 0x3a, 0x52, 0x73, 0x93,
    0xa3, 0x21, 0x6a, 0xd8, 0x92, 0x8c, 0x4a, 0x64,
    0x61, 0x4d, 0xf8, 0x31, 0xeb, 0xb5, 0x8f, 0xa2,
    0xc1, 0x4e, 0x28, 0xa4, 0x2a, 0x90, 0x77, 0x05,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char audio_msgs__msg__AudioPlayback__TYPE_NAME[] = "audio_msgs/msg/AudioPlayback";
static char audio_msgs__msg__AudioData__TYPE_NAME[] = "audio_msgs/msg/AudioData";
static char audio_msgs__msg__AudioInfo__TYPE_NAME[] = "audio_msgs/msg/AudioInfo";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char audio_msgs__msg__AudioPlayback__FIELD_NAME__stamps[] = "stamps";
static char audio_msgs__msg__AudioPlayback__FIELD_NAME__info[] = "info";
static char audio_msgs__msg__AudioPlayback__FIELD_NAME__data[] = "data";
static char audio_msgs__msg__AudioPlayback__FIELD_NAME__pkg_name[] = "pkg_name";
static char audio_msgs__msg__AudioPlayback__FIELD_NAME__token_id[] = "token_id";

static rosidl_runtime_c__type_description__Field audio_msgs__msg__AudioPlayback__FIELDS[] = {
  {
    {audio_msgs__msg__AudioPlayback__FIELD_NAME__stamps, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioPlayback__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_msgs__msg__AudioInfo__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioPlayback__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_msgs__msg__AudioData__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioPlayback__FIELD_NAME__pkg_name, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioPlayback__FIELD_NAME__token_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription audio_msgs__msg__AudioPlayback__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {audio_msgs__msg__AudioData__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__AudioInfo__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__msg__AudioPlayback__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__msg__AudioPlayback__TYPE_NAME, 28, 28},
      {audio_msgs__msg__AudioPlayback__FIELDS, 5, 5},
    },
    {audio_msgs__msg__AudioPlayback__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&audio_msgs__msg__AudioData__EXPECTED_HASH, audio_msgs__msg__AudioData__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = audio_msgs__msg__AudioData__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&audio_msgs__msg__AudioInfo__EXPECTED_HASH, audio_msgs__msg__AudioInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = audio_msgs__msg__AudioInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe5\\xbf\\x85\\xe8\\xa6\\x81\\xe9\\xa1\\xb9\\xef\\xbc\\x8c\\xe6\\x97\\xb6\\xe9\\x97\\xb4\\xe6\\x88\\xb3\n"
  "builtin_interfaces/Time stamps\n"
  "\n"
  "# \\xe5\\xbf\\x85\\xe8\\xa6\\x81\\xe9\\xa1\\xb9\\xef\\xbc\\x8c\\xe9\\x9f\\xb3\\xe9\\xa2\\x91\\xe6\\xa0\\xbc\\xe5\\xbc\\x8f\n"
  "AudioInfo info\n"
  "\n"
  "# \\xe5\\xbf\\x85\\xe8\\xa6\\x81\\xe9\\xa1\\xb9\\xef\\xbc\\x8c\\xe9\\x9f\\xb3\\xe9\\xa2\\x91\\xe6\\x95\\xb0\\xe6\\x8d\\xae\n"
  "AudioData data\n"
  "\n"
  "# \\xe5\\xbf\\x85\\xe8\\xa6\\x81\\xe9\\xa1\\xb9\\xef\\xbc\\x8c\\xe6\\xa0\\x87\\xe8\\xaf\\x86\\xe5\\x8f\\x91\\xe9\\x80\\x81\\xe6\\x96\\xb9\\xe6\\x9d\\xa5\\xe6\\xba\\x90\n"
  "string pkg_name\n"
  "\n"
  "# \\xe5\\x8f\\xaf\\xe9\\x80\\x89\\xe9\\xa1\\xb9\\xef\\xbc\\x8ctoken_id\\xe6\\x9c\\x89\\xe5\\x8f\\x98\\xe5\\x8c\\x96\\xe6\\x97\\xb6\\xe4\\xbc\\x9a\\xe6\\xb8\\x85\\xe7\\x90\\x86\\xe6\\x92\\xad\\xe6\\x94\\xbe\\xe7\\xbc\\x93\\xe5\\xad\\x98\\xef\\xbc\\x88\\xe7\\x94\\xa8\\xe4\\xba\\x8e\\xe6\\x89\\x93\\xe6\\x96\\xad\\xe5\\xbd\\x93\\xe5\\x89\\x8d\\xe6\\x92\\xad\\xe6\\x94\\xbe\\xef\\xbc\\x89\n"
  "string token_id";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__msg__AudioPlayback__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__msg__AudioPlayback__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 178, 178},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__msg__AudioPlayback__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__msg__AudioPlayback__get_individual_type_description_source(NULL),
    sources[1] = *audio_msgs__msg__AudioData__get_individual_type_description_source(NULL);
    sources[2] = *audio_msgs__msg__AudioInfo__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
