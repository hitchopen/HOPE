// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from audio_msgs:msg/AudioInfo.idl
// generated code does not contain a copyright notice
#include "audio_msgs/msg/detail/audio_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sample_format`
// Member `coding_format`
#include "rosidl_runtime_c/string_functions.h"

bool
audio_msgs__msg__AudioInfo__init(audio_msgs__msg__AudioInfo * msg)
{
  if (!msg) {
    return false;
  }
  // channels
  // sample_rate
  // sample_format
  if (!rosidl_runtime_c__String__init(&msg->sample_format)) {
    audio_msgs__msg__AudioInfo__fini(msg);
    return false;
  }
  // coding_format
  if (!rosidl_runtime_c__String__init(&msg->coding_format)) {
    audio_msgs__msg__AudioInfo__fini(msg);
    return false;
  }
  return true;
}

void
audio_msgs__msg__AudioInfo__fini(audio_msgs__msg__AudioInfo * msg)
{
  if (!msg) {
    return;
  }
  // channels
  // sample_rate
  // sample_format
  rosidl_runtime_c__String__fini(&msg->sample_format);
  // coding_format
  rosidl_runtime_c__String__fini(&msg->coding_format);
}

bool
audio_msgs__msg__AudioInfo__are_equal(const audio_msgs__msg__AudioInfo * lhs, const audio_msgs__msg__AudioInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // channels
  if (lhs->channels != rhs->channels) {
    return false;
  }
  // sample_rate
  if (lhs->sample_rate != rhs->sample_rate) {
    return false;
  }
  // sample_format
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sample_format), &(rhs->sample_format)))
  {
    return false;
  }
  // coding_format
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->coding_format), &(rhs->coding_format)))
  {
    return false;
  }
  return true;
}

bool
audio_msgs__msg__AudioInfo__copy(
  const audio_msgs__msg__AudioInfo * input,
  audio_msgs__msg__AudioInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // channels
  output->channels = input->channels;
  // sample_rate
  output->sample_rate = input->sample_rate;
  // sample_format
  if (!rosidl_runtime_c__String__copy(
      &(input->sample_format), &(output->sample_format)))
  {
    return false;
  }
  // coding_format
  if (!rosidl_runtime_c__String__copy(
      &(input->coding_format), &(output->coding_format)))
  {
    return false;
  }
  return true;
}

audio_msgs__msg__AudioInfo *
audio_msgs__msg__AudioInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__AudioInfo * msg = (audio_msgs__msg__AudioInfo *)allocator.allocate(sizeof(audio_msgs__msg__AudioInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(audio_msgs__msg__AudioInfo));
  bool success = audio_msgs__msg__AudioInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
audio_msgs__msg__AudioInfo__destroy(audio_msgs__msg__AudioInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    audio_msgs__msg__AudioInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
audio_msgs__msg__AudioInfo__Sequence__init(audio_msgs__msg__AudioInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__AudioInfo * data = NULL;

  if (size) {
    data = (audio_msgs__msg__AudioInfo *)allocator.zero_allocate(size, sizeof(audio_msgs__msg__AudioInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = audio_msgs__msg__AudioInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        audio_msgs__msg__AudioInfo__fini(&data[i - 1]);
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
audio_msgs__msg__AudioInfo__Sequence__fini(audio_msgs__msg__AudioInfo__Sequence * array)
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
      audio_msgs__msg__AudioInfo__fini(&array->data[i]);
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

audio_msgs__msg__AudioInfo__Sequence *
audio_msgs__msg__AudioInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__AudioInfo__Sequence * array = (audio_msgs__msg__AudioInfo__Sequence *)allocator.allocate(sizeof(audio_msgs__msg__AudioInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = audio_msgs__msg__AudioInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
audio_msgs__msg__AudioInfo__Sequence__destroy(audio_msgs__msg__AudioInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    audio_msgs__msg__AudioInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
audio_msgs__msg__AudioInfo__Sequence__are_equal(const audio_msgs__msg__AudioInfo__Sequence * lhs, const audio_msgs__msg__AudioInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!audio_msgs__msg__AudioInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
audio_msgs__msg__AudioInfo__Sequence__copy(
  const audio_msgs__msg__AudioInfo__Sequence * input,
  audio_msgs__msg__AudioInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(audio_msgs__msg__AudioInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    audio_msgs__msg__AudioInfo * data =
      (audio_msgs__msg__AudioInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!audio_msgs__msg__AudioInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          audio_msgs__msg__AudioInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!audio_msgs__msg__AudioInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
