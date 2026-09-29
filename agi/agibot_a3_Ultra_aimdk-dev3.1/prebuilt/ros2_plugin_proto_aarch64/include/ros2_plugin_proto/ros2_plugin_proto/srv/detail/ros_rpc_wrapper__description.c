// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros2_plugin_proto:srv/RosRpcWrapper.idl
// generated code does not contain a copyright notice

#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros2_plugin_proto
const rosidl_type_hash_t *
ros2_plugin_proto__srv__RosRpcWrapper__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8d, 0x4e, 0x9e, 0x28, 0x8c, 0x4f, 0x6e, 0x95,
      0xff, 0x87, 0xd0, 0xd7, 0xdf, 0x18, 0xb2, 0xb1,
      0x51, 0x24, 0x26, 0x2f, 0xa1, 0x57, 0x5d, 0x35,
      0x0d, 0xc2, 0x1a, 0xf4, 0x9a, 0x8b, 0x17, 0x71,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros2_plugin_proto
const rosidl_type_hash_t *
ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc2, 0xf5, 0x61, 0x61, 0xc2, 0x47, 0x9a, 0xb3,
      0xfa, 0x41, 0xab, 0xb9, 0xc6, 0x91, 0xd2, 0xef,
      0xcd, 0x07, 0x20, 0x83, 0x07, 0x26, 0x3a, 0x22,
      0xbf, 0xb4, 0x7d, 0x22, 0xbf, 0xeb, 0xc7, 0xaf,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros2_plugin_proto
const rosidl_type_hash_t *
ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x14, 0x6d, 0x7b, 0x6e, 0xf5, 0x16, 0x29, 0xb2,
      0x9b, 0x69, 0xfc, 0x6f, 0xff, 0x69, 0x9d, 0xa1,
      0xb4, 0xad, 0xfe, 0xe3, 0x42, 0x3d, 0x9f, 0x2c,
      0x03, 0x72, 0x5d, 0x41, 0xd0, 0xb3, 0xf5, 0x2c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros2_plugin_proto
const rosidl_type_hash_t *
ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7b, 0x06, 0x7b, 0x7e, 0x3a, 0xea, 0x11, 0x84,
      0x13, 0x12, 0x11, 0xd7, 0xb9, 0xeb, 0xbb, 0x40,
      0x28, 0x85, 0x0c, 0xce, 0xd4, 0x6d, 0xf3, 0x06,
      0x05, 0x6f, 0x7e, 0xf0, 0xa2, 0xaa, 0x03, 0x47,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
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
#endif

static char ros2_plugin_proto__srv__RosRpcWrapper__TYPE_NAME[] = "ros2_plugin_proto/srv/RosRpcWrapper";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char ros2_plugin_proto__srv__RosRpcWrapper_Event__TYPE_NAME[] = "ros2_plugin_proto/srv/RosRpcWrapper_Event";
static char ros2_plugin_proto__srv__RosRpcWrapper_Request__TYPE_NAME[] = "ros2_plugin_proto/srv/RosRpcWrapper_Request";
static char ros2_plugin_proto__srv__RosRpcWrapper_Response__TYPE_NAME[] = "ros2_plugin_proto/srv/RosRpcWrapper_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char ros2_plugin_proto__srv__RosRpcWrapper__FIELD_NAME__request_message[] = "request_message";
static char ros2_plugin_proto__srv__RosRpcWrapper__FIELD_NAME__response_message[] = "response_message";
static char ros2_plugin_proto__srv__RosRpcWrapper__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field ros2_plugin_proto__srv__RosRpcWrapper__FIELDS[] = {
  {
    {ros2_plugin_proto__srv__RosRpcWrapper__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2_plugin_proto__srv__RosRpcWrapper_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2_plugin_proto__srv__RosRpcWrapper_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2_plugin_proto__srv__RosRpcWrapper_Event__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros2_plugin_proto__srv__RosRpcWrapper__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Event__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Response__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2_plugin_proto__srv__RosRpcWrapper__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2_plugin_proto__srv__RosRpcWrapper__TYPE_NAME, 35, 35},
      {ros2_plugin_proto__srv__RosRpcWrapper__FIELDS, 3, 3},
    },
    {ros2_plugin_proto__srv__RosRpcWrapper__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELD_NAME__serialization_type[] = "serialization_type";
static char ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELD_NAME__context[] = "context";
static char ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELDS[] = {
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELD_NAME__serialization_type, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELD_NAME__context, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELD_NAME__data, 4, 4},
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
ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2_plugin_proto__srv__RosRpcWrapper_Request__TYPE_NAME, 43, 43},
      {ros2_plugin_proto__srv__RosRpcWrapper_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELD_NAME__code[] = "code";
static char ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELD_NAME__serialization_type[] = "serialization_type";
static char ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELDS[] = {
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELD_NAME__code, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELD_NAME__serialization_type, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELD_NAME__data, 4, 4},
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
ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2_plugin_proto__srv__RosRpcWrapper_Response__TYPE_NAME, 44, 44},
      {ros2_plugin_proto__srv__RosRpcWrapper_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELD_NAME__info[] = "info";
static char ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELD_NAME__request[] = "request";
static char ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELDS[] = {
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {ros2_plugin_proto__srv__RosRpcWrapper_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {ros2_plugin_proto__srv__RosRpcWrapper_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros2_plugin_proto__srv__RosRpcWrapper_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {ros2_plugin_proto__srv__RosRpcWrapper_Response__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2_plugin_proto__srv__RosRpcWrapper_Event__TYPE_NAME, 41, 41},
      {ros2_plugin_proto__srv__RosRpcWrapper_Event__FIELDS, 3, 3},
    },
    {ros2_plugin_proto__srv__RosRpcWrapper_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string  serialization_type\n"
  "string[]  context\n"
  "byte[]  data\n"
  "---\n"
  "int64   code\n"
  "string  serialization_type\n"
  "byte[]  data";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros2_plugin_proto__srv__RosRpcWrapper__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2_plugin_proto__srv__RosRpcWrapper__TYPE_NAME, 35, 35},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 115, 115},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros2_plugin_proto__srv__RosRpcWrapper_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2_plugin_proto__srv__RosRpcWrapper_Request__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros2_plugin_proto__srv__RosRpcWrapper_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2_plugin_proto__srv__RosRpcWrapper_Response__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros2_plugin_proto__srv__RosRpcWrapper_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2_plugin_proto__srv__RosRpcWrapper_Event__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2_plugin_proto__srv__RosRpcWrapper__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2_plugin_proto__srv__RosRpcWrapper__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *ros2_plugin_proto__srv__RosRpcWrapper_Event__get_individual_type_description_source(NULL);
    sources[3] = *ros2_plugin_proto__srv__RosRpcWrapper_Request__get_individual_type_description_source(NULL);
    sources[4] = *ros2_plugin_proto__srv__RosRpcWrapper_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2_plugin_proto__srv__RosRpcWrapper_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2_plugin_proto__srv__RosRpcWrapper_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2_plugin_proto__srv__RosRpcWrapper_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2_plugin_proto__srv__RosRpcWrapper_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2_plugin_proto__srv__RosRpcWrapper_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2_plugin_proto__srv__RosRpcWrapper_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *ros2_plugin_proto__srv__RosRpcWrapper_Request__get_individual_type_description_source(NULL);
    sources[3] = *ros2_plugin_proto__srv__RosRpcWrapper_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
