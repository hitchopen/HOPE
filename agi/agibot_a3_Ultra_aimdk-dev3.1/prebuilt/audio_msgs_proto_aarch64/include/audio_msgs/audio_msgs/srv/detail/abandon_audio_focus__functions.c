// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from audio_msgs:srv/AbandonAudioFocus.idl
// generated code does not contain a copyright notice
#include "audio_msgs/srv/detail/abandon_audio_focus__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `focus_requester`
#include "audio_msgs/msg/detail/focus_requester__functions.h"

bool
audio_msgs__srv__AbandonAudioFocus_Request__init(audio_msgs__srv__AbandonAudioFocus_Request * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    audio_msgs__srv__AbandonAudioFocus_Request__fini(msg);
    return false;
  }
  // focus_requester
  if (!audio_msgs__msg__FocusRequester__init(&msg->focus_requester)) {
    audio_msgs__srv__AbandonAudioFocus_Request__fini(msg);
    return false;
  }
  return true;
}

void
audio_msgs__srv__AbandonAudioFocus_Request__fini(audio_msgs__srv__AbandonAudioFocus_Request * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // focus_requester
  audio_msgs__msg__FocusRequester__fini(&msg->focus_requester);
}

bool
audio_msgs__srv__AbandonAudioFocus_Request__are_equal(const audio_msgs__srv__AbandonAudioFocus_Request * lhs, const audio_msgs__srv__AbandonAudioFocus_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // focus_requester
  if (!audio_msgs__msg__FocusRequester__are_equal(
      &(lhs->focus_requester), &(rhs->focus_requester)))
  {
    return false;
  }
  return true;
}

bool
audio_msgs__srv__AbandonAudioFocus_Request__copy(
  const audio_msgs__srv__AbandonAudioFocus_Request * input,
  audio_msgs__srv__AbandonAudioFocus_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // focus_requester
  if (!audio_msgs__msg__FocusRequester__copy(
      &(input->focus_requester), &(output->focus_requester)))
  {
    return false;
  }
  return true;
}

audio_msgs__srv__AbandonAudioFocus_Request *
audio_msgs__srv__AbandonAudioFocus_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Request * msg = (audio_msgs__srv__AbandonAudioFocus_Request *)allocator.allocate(sizeof(audio_msgs__srv__AbandonAudioFocus_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(audio_msgs__srv__AbandonAudioFocus_Request));
  bool success = audio_msgs__srv__AbandonAudioFocus_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
audio_msgs__srv__AbandonAudioFocus_Request__destroy(audio_msgs__srv__AbandonAudioFocus_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    audio_msgs__srv__AbandonAudioFocus_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__init(audio_msgs__srv__AbandonAudioFocus_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Request * data = NULL;

  if (size) {
    data = (audio_msgs__srv__AbandonAudioFocus_Request *)allocator.zero_allocate(size, sizeof(audio_msgs__srv__AbandonAudioFocus_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = audio_msgs__srv__AbandonAudioFocus_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        audio_msgs__srv__AbandonAudioFocus_Request__fini(&data[i - 1]);
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
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__fini(audio_msgs__srv__AbandonAudioFocus_Request__Sequence * array)
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
      audio_msgs__srv__AbandonAudioFocus_Request__fini(&array->data[i]);
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

audio_msgs__srv__AbandonAudioFocus_Request__Sequence *
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Request__Sequence * array = (audio_msgs__srv__AbandonAudioFocus_Request__Sequence *)allocator.allocate(sizeof(audio_msgs__srv__AbandonAudioFocus_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = audio_msgs__srv__AbandonAudioFocus_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__destroy(audio_msgs__srv__AbandonAudioFocus_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    audio_msgs__srv__AbandonAudioFocus_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__are_equal(const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * lhs, const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!audio_msgs__srv__AbandonAudioFocus_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__copy(
  const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * input,
  audio_msgs__srv__AbandonAudioFocus_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(audio_msgs__srv__AbandonAudioFocus_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    audio_msgs__srv__AbandonAudioFocus_Request * data =
      (audio_msgs__srv__AbandonAudioFocus_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!audio_msgs__srv__AbandonAudioFocus_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          audio_msgs__srv__AbandonAudioFocus_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!audio_msgs__srv__AbandonAudioFocus_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `header`
// already included above
// #include "std_msgs/msg/detail/header__functions.h"
// Member `focus_response`
#include "audio_msgs/msg/detail/focus_response__functions.h"

bool
audio_msgs__srv__AbandonAudioFocus_Response__init(audio_msgs__srv__AbandonAudioFocus_Response * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    audio_msgs__srv__AbandonAudioFocus_Response__fini(msg);
    return false;
  }
  // focus_response
  if (!audio_msgs__msg__FocusResponse__init(&msg->focus_response)) {
    audio_msgs__srv__AbandonAudioFocus_Response__fini(msg);
    return false;
  }
  return true;
}

void
audio_msgs__srv__AbandonAudioFocus_Response__fini(audio_msgs__srv__AbandonAudioFocus_Response * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // focus_response
  audio_msgs__msg__FocusResponse__fini(&msg->focus_response);
}

bool
audio_msgs__srv__AbandonAudioFocus_Response__are_equal(const audio_msgs__srv__AbandonAudioFocus_Response * lhs, const audio_msgs__srv__AbandonAudioFocus_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // focus_response
  if (!audio_msgs__msg__FocusResponse__are_equal(
      &(lhs->focus_response), &(rhs->focus_response)))
  {
    return false;
  }
  return true;
}

bool
audio_msgs__srv__AbandonAudioFocus_Response__copy(
  const audio_msgs__srv__AbandonAudioFocus_Response * input,
  audio_msgs__srv__AbandonAudioFocus_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // focus_response
  if (!audio_msgs__msg__FocusResponse__copy(
      &(input->focus_response), &(output->focus_response)))
  {
    return false;
  }
  return true;
}

audio_msgs__srv__AbandonAudioFocus_Response *
audio_msgs__srv__AbandonAudioFocus_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Response * msg = (audio_msgs__srv__AbandonAudioFocus_Response *)allocator.allocate(sizeof(audio_msgs__srv__AbandonAudioFocus_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(audio_msgs__srv__AbandonAudioFocus_Response));
  bool success = audio_msgs__srv__AbandonAudioFocus_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
audio_msgs__srv__AbandonAudioFocus_Response__destroy(audio_msgs__srv__AbandonAudioFocus_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    audio_msgs__srv__AbandonAudioFocus_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__init(audio_msgs__srv__AbandonAudioFocus_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Response * data = NULL;

  if (size) {
    data = (audio_msgs__srv__AbandonAudioFocus_Response *)allocator.zero_allocate(size, sizeof(audio_msgs__srv__AbandonAudioFocus_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = audio_msgs__srv__AbandonAudioFocus_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        audio_msgs__srv__AbandonAudioFocus_Response__fini(&data[i - 1]);
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
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__fini(audio_msgs__srv__AbandonAudioFocus_Response__Sequence * array)
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
      audio_msgs__srv__AbandonAudioFocus_Response__fini(&array->data[i]);
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

audio_msgs__srv__AbandonAudioFocus_Response__Sequence *
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Response__Sequence * array = (audio_msgs__srv__AbandonAudioFocus_Response__Sequence *)allocator.allocate(sizeof(audio_msgs__srv__AbandonAudioFocus_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = audio_msgs__srv__AbandonAudioFocus_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__destroy(audio_msgs__srv__AbandonAudioFocus_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    audio_msgs__srv__AbandonAudioFocus_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__are_equal(const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * lhs, const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!audio_msgs__srv__AbandonAudioFocus_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__copy(
  const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * input,
  audio_msgs__srv__AbandonAudioFocus_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(audio_msgs__srv__AbandonAudioFocus_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    audio_msgs__srv__AbandonAudioFocus_Response * data =
      (audio_msgs__srv__AbandonAudioFocus_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!audio_msgs__srv__AbandonAudioFocus_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          audio_msgs__srv__AbandonAudioFocus_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!audio_msgs__srv__AbandonAudioFocus_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "audio_msgs/srv/detail/abandon_audio_focus__functions.h"

bool
audio_msgs__srv__AbandonAudioFocus_Event__init(audio_msgs__srv__AbandonAudioFocus_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    audio_msgs__srv__AbandonAudioFocus_Event__fini(msg);
    return false;
  }
  // request
  if (!audio_msgs__srv__AbandonAudioFocus_Request__Sequence__init(&msg->request, 0)) {
    audio_msgs__srv__AbandonAudioFocus_Event__fini(msg);
    return false;
  }
  // response
  if (!audio_msgs__srv__AbandonAudioFocus_Response__Sequence__init(&msg->response, 0)) {
    audio_msgs__srv__AbandonAudioFocus_Event__fini(msg);
    return false;
  }
  return true;
}

void
audio_msgs__srv__AbandonAudioFocus_Event__fini(audio_msgs__srv__AbandonAudioFocus_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  audio_msgs__srv__AbandonAudioFocus_Request__Sequence__fini(&msg->request);
  // response
  audio_msgs__srv__AbandonAudioFocus_Response__Sequence__fini(&msg->response);
}

bool
audio_msgs__srv__AbandonAudioFocus_Event__are_equal(const audio_msgs__srv__AbandonAudioFocus_Event * lhs, const audio_msgs__srv__AbandonAudioFocus_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!audio_msgs__srv__AbandonAudioFocus_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!audio_msgs__srv__AbandonAudioFocus_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
audio_msgs__srv__AbandonAudioFocus_Event__copy(
  const audio_msgs__srv__AbandonAudioFocus_Event * input,
  audio_msgs__srv__AbandonAudioFocus_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!audio_msgs__srv__AbandonAudioFocus_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!audio_msgs__srv__AbandonAudioFocus_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

audio_msgs__srv__AbandonAudioFocus_Event *
audio_msgs__srv__AbandonAudioFocus_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Event * msg = (audio_msgs__srv__AbandonAudioFocus_Event *)allocator.allocate(sizeof(audio_msgs__srv__AbandonAudioFocus_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(audio_msgs__srv__AbandonAudioFocus_Event));
  bool success = audio_msgs__srv__AbandonAudioFocus_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
audio_msgs__srv__AbandonAudioFocus_Event__destroy(audio_msgs__srv__AbandonAudioFocus_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    audio_msgs__srv__AbandonAudioFocus_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__init(audio_msgs__srv__AbandonAudioFocus_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Event * data = NULL;

  if (size) {
    data = (audio_msgs__srv__AbandonAudioFocus_Event *)allocator.zero_allocate(size, sizeof(audio_msgs__srv__AbandonAudioFocus_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = audio_msgs__srv__AbandonAudioFocus_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        audio_msgs__srv__AbandonAudioFocus_Event__fini(&data[i - 1]);
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
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__fini(audio_msgs__srv__AbandonAudioFocus_Event__Sequence * array)
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
      audio_msgs__srv__AbandonAudioFocus_Event__fini(&array->data[i]);
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

audio_msgs__srv__AbandonAudioFocus_Event__Sequence *
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__srv__AbandonAudioFocus_Event__Sequence * array = (audio_msgs__srv__AbandonAudioFocus_Event__Sequence *)allocator.allocate(sizeof(audio_msgs__srv__AbandonAudioFocus_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = audio_msgs__srv__AbandonAudioFocus_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__destroy(audio_msgs__srv__AbandonAudioFocus_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    audio_msgs__srv__AbandonAudioFocus_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__are_equal(const audio_msgs__srv__AbandonAudioFocus_Event__Sequence * lhs, const audio_msgs__srv__AbandonAudioFocus_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!audio_msgs__srv__AbandonAudioFocus_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__copy(
  const audio_msgs__srv__AbandonAudioFocus_Event__Sequence * input,
  audio_msgs__srv__AbandonAudioFocus_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(audio_msgs__srv__AbandonAudioFocus_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    audio_msgs__srv__AbandonAudioFocus_Event * data =
      (audio_msgs__srv__AbandonAudioFocus_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!audio_msgs__srv__AbandonAudioFocus_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          audio_msgs__srv__AbandonAudioFocus_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!audio_msgs__srv__AbandonAudioFocus_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
