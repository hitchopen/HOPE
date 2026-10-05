// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from audio_msgs:msg/AudioPlayback.idl
// generated code does not contain a copyright notice
#include "audio_msgs/msg/detail/audio_playback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamps`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `info`
#include "audio_msgs/msg/detail/audio_info__functions.h"
// Member `data`
#include "audio_msgs/msg/detail/audio_data__functions.h"
// Member `pkg_name`
// Member `token_id`
#include "rosidl_runtime_c/string_functions.h"

bool
audio_msgs__msg__AudioPlayback__init(audio_msgs__msg__AudioPlayback * msg)
{
  if (!msg) {
    return false;
  }
  // stamps
  if (!builtin_interfaces__msg__Time__init(&msg->stamps)) {
    audio_msgs__msg__AudioPlayback__fini(msg);
    return false;
  }
  // info
  if (!audio_msgs__msg__AudioInfo__init(&msg->info)) {
    audio_msgs__msg__AudioPlayback__fini(msg);
    return false;
  }
  // data
  if (!audio_msgs__msg__AudioData__init(&msg->data)) {
    audio_msgs__msg__AudioPlayback__fini(msg);
    return false;
  }
  // pkg_name
  if (!rosidl_runtime_c__String__init(&msg->pkg_name)) {
    audio_msgs__msg__AudioPlayback__fini(msg);
    return false;
  }
  // token_id
  if (!rosidl_runtime_c__String__init(&msg->token_id)) {
    audio_msgs__msg__AudioPlayback__fini(msg);
    return false;
  }
  return true;
}

void
audio_msgs__msg__AudioPlayback__fini(audio_msgs__msg__AudioPlayback * msg)
{
  if (!msg) {
    return;
  }
  // stamps
  builtin_interfaces__msg__Time__fini(&msg->stamps);
  // info
  audio_msgs__msg__AudioInfo__fini(&msg->info);
  // data
  audio_msgs__msg__AudioData__fini(&msg->data);
  // pkg_name
  rosidl_runtime_c__String__fini(&msg->pkg_name);
  // token_id
  rosidl_runtime_c__String__fini(&msg->token_id);
}

bool
audio_msgs__msg__AudioPlayback__are_equal(const audio_msgs__msg__AudioPlayback * lhs, const audio_msgs__msg__AudioPlayback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamps
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamps), &(rhs->stamps)))
  {
    return false;
  }
  // info
  if (!audio_msgs__msg__AudioInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // data
  if (!audio_msgs__msg__AudioData__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // pkg_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->pkg_name), &(rhs->pkg_name)))
  {
    return false;
  }
  // token_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->token_id), &(rhs->token_id)))
  {
    return false;
  }
  return true;
}

bool
audio_msgs__msg__AudioPlayback__copy(
  const audio_msgs__msg__AudioPlayback * input,
  audio_msgs__msg__AudioPlayback * output)
{
  if (!input || !output) {
    return false;
  }
  // stamps
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamps), &(output->stamps)))
  {
    return false;
  }
  // info
  if (!audio_msgs__msg__AudioInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // data
  if (!audio_msgs__msg__AudioData__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // pkg_name
  if (!rosidl_runtime_c__String__copy(
      &(input->pkg_name), &(output->pkg_name)))
  {
    return false;
  }
  // token_id
  if (!rosidl_runtime_c__String__copy(
      &(input->token_id), &(output->token_id)))
  {
    return false;
  }
  return true;
}

audio_msgs__msg__AudioPlayback *
audio_msgs__msg__AudioPlayback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__AudioPlayback * msg = (audio_msgs__msg__AudioPlayback *)allocator.allocate(sizeof(audio_msgs__msg__AudioPlayback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(audio_msgs__msg__AudioPlayback));
  bool success = audio_msgs__msg__AudioPlayback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
audio_msgs__msg__AudioPlayback__destroy(audio_msgs__msg__AudioPlayback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    audio_msgs__msg__AudioPlayback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
audio_msgs__msg__AudioPlayback__Sequence__init(audio_msgs__msg__AudioPlayback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__AudioPlayback * data = NULL;

  if (size) {
    data = (audio_msgs__msg__AudioPlayback *)allocator.zero_allocate(size, sizeof(audio_msgs__msg__AudioPlayback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = audio_msgs__msg__AudioPlayback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        audio_msgs__msg__AudioPlayback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
audio_msgs__msg__AudioPlayback__Sequence__fini(audio_msgs__msg__AudioPlayback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      audio_msgs__msg__AudioPlayback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

audio_msgs__msg__AudioPlayback__Sequence *
audio_msgs__msg__AudioPlayback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__AudioPlayback__Sequence * array = (audio_msgs__msg__AudioPlayback__Sequence *)allocator.allocate(sizeof(audio_msgs__msg__AudioPlayback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = audio_msgs__msg__AudioPlayback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
audio_msgs__msg__AudioPlayback__Sequence__destroy(audio_msgs__msg__AudioPlayback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    audio_msgs__msg__AudioPlayback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
audio_msgs__msg__AudioPlayback__Sequence__are_equal(const audio_msgs__msg__AudioPlayback__Sequence * lhs, const audio_msgs__msg__AudioPlayback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!audio_msgs__msg__AudioPlayback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
audio_msgs__msg__AudioPlayback__Sequence__copy(
  const audio_msgs__msg__AudioPlayback__Sequence * input,
  audio_msgs__msg__AudioPlayback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(audio_msgs__msg__AudioPlayback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    audio_msgs__msg__AudioPlayback * data =
      (audio_msgs__msg__AudioPlayback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!audio_msgs__msg__AudioPlayback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          audio_msgs__msg__AudioPlayback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!audio_msgs__msg__AudioPlayback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
