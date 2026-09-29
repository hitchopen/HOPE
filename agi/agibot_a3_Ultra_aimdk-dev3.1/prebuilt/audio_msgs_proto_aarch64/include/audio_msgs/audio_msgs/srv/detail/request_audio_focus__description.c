// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_msgs:srv/RequestAudioFocus.idl
// generated code does not contain a copyright notice

#include "audio_msgs/srv/detail/request_audio_focus__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__RequestAudioFocus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x96, 0x4c, 0x28, 0xb4, 0x81, 0xe8, 0x15, 0x1b,
      0x74, 0xde, 0x5f, 0x68, 0x2b, 0xe8, 0xcb, 0xc4,
      0x26, 0x7e, 0x82, 0x8e, 0x5f, 0x1a, 0x3e, 0xe7,
      0x45, 0x4e, 0x2f, 0xcb, 0x51, 0xc1, 0x89, 0x1b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__RequestAudioFocus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf0, 0x4b, 0x22, 0x03, 0x6c, 0xde, 0xd8, 0x11,
      0x46, 0xd1, 0x25, 0x9e, 0xb1, 0x09, 0x81, 0xc6,
      0xe0, 0x74, 0xc8, 0x14, 0x8b, 0xef, 0x90, 0x3b,
      0xd1, 0x8b, 0x5a, 0xbd, 0xb0, 0x1c, 0x5c, 0xa9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__RequestAudioFocus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd0, 0x06, 0xfd, 0x1f, 0xc1, 0xd6, 0xb1, 0x0c,
      0xbe, 0x1e, 0x5e, 0x39, 0x6f, 0x8e, 0xca, 0xae,
      0x1f, 0x03, 0x2c, 0x7d, 0xfb, 0x93, 0x46, 0x23,
      0x14, 0x85, 0xc7, 0x1c, 0xa1, 0x83, 0x8b, 0x54,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__RequestAudioFocus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x19, 0xe4, 0x7e, 0xfb, 0xfd, 0xac, 0x2e, 0xfb,
      0x6b, 0xcd, 0x1d, 0x54, 0x2d, 0x3d, 0x35, 0x90,
      0x4c, 0xcd, 0xc5, 0x9d, 0xec, 0xcc, 0x50, 0x19,
      0xf5, 0x28, 0x3c, 0x05, 0xf8, 0xad, 0x4d, 0xc3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "audio_msgs/msg/detail/focus_requester__functions.h"
#include "audio_msgs/msg/detail/focus_response__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t audio_msgs__msg__FocusRequester__EXPECTED_HASH = {1, {
    0x12, 0x2a, 0x06, 0xff, 0x96, 0x16, 0x54, 0x37,
    0xf9, 0x4a, 0x8b, 0x7b, 0xac, 0xe9, 0x45, 0x14,
    0x91, 0x85, 0x44, 0xec, 0x72, 0x89, 0x7c, 0xe1,
    0x09, 0xb3, 0xa8, 0x8d, 0x25, 0x3c, 0x3f, 0xd2,
  }};
static const rosidl_type_hash_t audio_msgs__msg__FocusResponse__EXPECTED_HASH = {1, {
    0x5d, 0x6b, 0xd7, 0x18, 0x01, 0x80, 0xaa, 0xa5,
    0x9f, 0x77, 0x1e, 0xe6, 0x12, 0x95, 0xcb, 0xa7,
    0xbf, 0xe7, 0x65, 0xe4, 0x86, 0xe7, 0x57, 0x18,
    0x5f, 0xe8, 0x2a, 0x85, 0x59, 0x1f, 0x98, 0x8b,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char audio_msgs__srv__RequestAudioFocus__TYPE_NAME[] = "audio_msgs/srv/RequestAudioFocus";
static char audio_msgs__msg__FocusRequester__TYPE_NAME[] = "audio_msgs/msg/FocusRequester";
static char audio_msgs__msg__FocusResponse__TYPE_NAME[] = "audio_msgs/msg/FocusResponse";
static char audio_msgs__srv__RequestAudioFocus_Event__TYPE_NAME[] = "audio_msgs/srv/RequestAudioFocus_Event";
static char audio_msgs__srv__RequestAudioFocus_Request__TYPE_NAME[] = "audio_msgs/srv/RequestAudioFocus_Request";
static char audio_msgs__srv__RequestAudioFocus_Response__TYPE_NAME[] = "audio_msgs/srv/RequestAudioFocus_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char audio_msgs__srv__RequestAudioFocus__FIELD_NAME__request_message[] = "request_message";
static char audio_msgs__srv__RequestAudioFocus__FIELD_NAME__response_message[] = "response_message";
static char audio_msgs__srv__RequestAudioFocus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field audio_msgs__srv__RequestAudioFocus__FIELDS[] = {
  {
    {audio_msgs__srv__RequestAudioFocus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_msgs__srv__RequestAudioFocus_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_msgs__srv__RequestAudioFocus_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_msgs__srv__RequestAudioFocus_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription audio_msgs__srv__RequestAudioFocus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {audio_msgs__msg__FocusRequester__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__FocusResponse__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__RequestAudioFocus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__srv__RequestAudioFocus__TYPE_NAME, 32, 32},
      {audio_msgs__srv__RequestAudioFocus__FIELDS, 3, 3},
    },
    {audio_msgs__srv__RequestAudioFocus__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&audio_msgs__msg__FocusRequester__EXPECTED_HASH, audio_msgs__msg__FocusRequester__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = audio_msgs__msg__FocusRequester__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&audio_msgs__msg__FocusResponse__EXPECTED_HASH, audio_msgs__msg__FocusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = audio_msgs__msg__FocusResponse__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = audio_msgs__srv__RequestAudioFocus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = audio_msgs__srv__RequestAudioFocus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = audio_msgs__srv__RequestAudioFocus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char audio_msgs__srv__RequestAudioFocus_Request__FIELD_NAME__header[] = "header";
static char audio_msgs__srv__RequestAudioFocus_Request__FIELD_NAME__focus_requester[] = "focus_requester";

static rosidl_runtime_c__type_description__Field audio_msgs__srv__RequestAudioFocus_Request__FIELDS[] = {
  {
    {audio_msgs__srv__RequestAudioFocus_Request__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Request__FIELD_NAME__focus_requester, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_msgs__msg__FocusRequester__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription audio_msgs__srv__RequestAudioFocus_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {audio_msgs__msg__FocusRequester__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__RequestAudioFocus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__srv__RequestAudioFocus_Request__TYPE_NAME, 40, 40},
      {audio_msgs__srv__RequestAudioFocus_Request__FIELDS, 2, 2},
    },
    {audio_msgs__srv__RequestAudioFocus_Request__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&audio_msgs__msg__FocusRequester__EXPECTED_HASH, audio_msgs__msg__FocusRequester__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = audio_msgs__msg__FocusRequester__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char audio_msgs__srv__RequestAudioFocus_Response__FIELD_NAME__header[] = "header";
static char audio_msgs__srv__RequestAudioFocus_Response__FIELD_NAME__focus_response[] = "focus_response";

static rosidl_runtime_c__type_description__Field audio_msgs__srv__RequestAudioFocus_Response__FIELDS[] = {
  {
    {audio_msgs__srv__RequestAudioFocus_Response__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Response__FIELD_NAME__focus_response, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_msgs__msg__FocusResponse__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription audio_msgs__srv__RequestAudioFocus_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {audio_msgs__msg__FocusResponse__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__RequestAudioFocus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__srv__RequestAudioFocus_Response__TYPE_NAME, 41, 41},
      {audio_msgs__srv__RequestAudioFocus_Response__FIELDS, 2, 2},
    },
    {audio_msgs__srv__RequestAudioFocus_Response__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&audio_msgs__msg__FocusResponse__EXPECTED_HASH, audio_msgs__msg__FocusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = audio_msgs__msg__FocusResponse__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char audio_msgs__srv__RequestAudioFocus_Event__FIELD_NAME__info[] = "info";
static char audio_msgs__srv__RequestAudioFocus_Event__FIELD_NAME__request[] = "request";
static char audio_msgs__srv__RequestAudioFocus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field audio_msgs__srv__RequestAudioFocus_Event__FIELDS[] = {
  {
    {audio_msgs__srv__RequestAudioFocus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {audio_msgs__srv__RequestAudioFocus_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {audio_msgs__srv__RequestAudioFocus_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription audio_msgs__srv__RequestAudioFocus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {audio_msgs__msg__FocusRequester__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__msg__FocusResponse__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {audio_msgs__srv__RequestAudioFocus_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__RequestAudioFocus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_msgs__srv__RequestAudioFocus_Event__TYPE_NAME, 38, 38},
      {audio_msgs__srv__RequestAudioFocus_Event__FIELDS, 3, 3},
    },
    {audio_msgs__srv__RequestAudioFocus_Event__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&audio_msgs__msg__FocusRequester__EXPECTED_HASH, audio_msgs__msg__FocusRequester__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = audio_msgs__msg__FocusRequester__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&audio_msgs__msg__FocusResponse__EXPECTED_HASH, audio_msgs__msg__FocusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = audio_msgs__msg__FocusResponse__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = audio_msgs__srv__RequestAudioFocus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = audio_msgs__srv__RequestAudioFocus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ----------------------------------------------------\n"
  "# request\n"
  "\n"
  "# \\xe8\\xaf\\xb7\\xe6\\xb1\\x82\\xe6\\x95\\xb0\\xe6\\x8d\\xae\\xe5\\xa4\\xb4\n"
  "std_msgs/Header header\n"
  "\n"
  "# \\xe8\\xaf\\xb7\\xe6\\xb1\\x82\\xe7\\x84\\xa6\\xe7\\x82\\xb9\\xe4\\xbf\\xa1\\xe6\\x81\\xaf\n"
  "FocusRequester focus_requester\n"
  "\n"
  "---\n"
  "\n"
  "# ----------------------------------------------------\n"
  "# response\n"
  "\n"
  "# \\xe5\\x93\\x8d\\xe5\\xba\\x94\\xe6\\x95\\xb0\\xe6\\x8d\\xae\\xe5\\xa4\\xb4\n"
  "std_msgs/Header header\n"
  "\n"
  "# \\xe8\\xaf\\xb7\\xe6\\xb1\\x82\\xe7\\xbb\\x93\\xe6\\x9e\\x9c\n"
  "FocusResponse focus_response\n"
  "";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__RequestAudioFocus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__srv__RequestAudioFocus__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 280, 280},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__RequestAudioFocus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__srv__RequestAudioFocus_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__RequestAudioFocus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__srv__RequestAudioFocus_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__RequestAudioFocus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_msgs__srv__RequestAudioFocus_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__RequestAudioFocus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__srv__RequestAudioFocus__get_individual_type_description_source(NULL),
    sources[1] = *audio_msgs__msg__FocusRequester__get_individual_type_description_source(NULL);
    sources[2] = *audio_msgs__msg__FocusResponse__get_individual_type_description_source(NULL);
    sources[3] = *audio_msgs__srv__RequestAudioFocus_Event__get_individual_type_description_source(NULL);
    sources[4] = *audio_msgs__srv__RequestAudioFocus_Request__get_individual_type_description_source(NULL);
    sources[5] = *audio_msgs__srv__RequestAudioFocus_Response__get_individual_type_description_source(NULL);
    sources[6] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__RequestAudioFocus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__srv__RequestAudioFocus_Request__get_individual_type_description_source(NULL),
    sources[1] = *audio_msgs__msg__FocusRequester__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__RequestAudioFocus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__srv__RequestAudioFocus_Response__get_individual_type_description_source(NULL),
    sources[1] = *audio_msgs__msg__FocusResponse__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__RequestAudioFocus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_msgs__srv__RequestAudioFocus_Event__get_individual_type_description_source(NULL),
    sources[1] = *audio_msgs__msg__FocusRequester__get_individual_type_description_source(NULL);
    sources[2] = *audio_msgs__msg__FocusResponse__get_individual_type_description_source(NULL);
    sources[3] = *audio_msgs__srv__RequestAudioFocus_Request__get_individual_type_description_source(NULL);
    sources[4] = *audio_msgs__srv__RequestAudioFocus_Response__get_individual_type_description_source(NULL);
    sources[5] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
