// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_plugin_proto:msg/RosMsgWrapper.idl
// generated code does not contain a copyright notice
#include "ros2_plugin_proto/msg/detail/ros_msg_wrapper__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `serialization_type`
// Member `context`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2_plugin_proto__msg__RosMsgWrapper__init(ros2_plugin_proto__msg__RosMsgWrapper * msg)
{
  if (!msg) {
    return false;
  }
  // serialization_type
  if (!rosidl_runtime_c__String__init(&msg->serialization_type)) {
    ros2_plugin_proto__msg__RosMsgWrapper__fini(msg);
    return false;
  }
  // context
  if (!rosidl_runtime_c__String__Sequence__init(&msg->context, 0)) {
    ros2_plugin_proto__msg__RosMsgWrapper__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__octet__Sequence__init(&msg->data, 0)) {
    ros2_plugin_proto__msg__RosMsgWrapper__fini(msg);
    return false;
  }
  return true;
}

void
ros2_plugin_proto__msg__RosMsgWrapper__fini(ros2_plugin_proto__msg__RosMsgWrapper * msg)
{
  if (!msg) {
    return;
  }
  // serialization_type
  rosidl_runtime_c__String__fini(&msg->serialization_type);
  // context
  rosidl_runtime_c__String__Sequence__fini(&msg->context);
  // data
  rosidl_runtime_c__octet__Sequence__fini(&msg->data);
}

bool
ros2_plugin_proto__msg__RosMsgWrapper__are_equal(const ros2_plugin_proto__msg__RosMsgWrapper * lhs, const ros2_plugin_proto__msg__RosMsgWrapper * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // serialization_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->serialization_type), &(rhs->serialization_type)))
  {
    return false;
  }
  // context
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->context), &(rhs->context)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__octet__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
ros2_plugin_proto__msg__RosMsgWrapper__copy(
  const ros2_plugin_proto__msg__RosMsgWrapper * input,
  ros2_plugin_proto__msg__RosMsgWrapper * output)
{
  if (!input || !output) {
    return false;
  }
  // serialization_type
  if (!rosidl_runtime_c__String__copy(
      &(input->serialization_type), &(output->serialization_type)))
  {
    return false;
  }
  // context
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->context), &(output->context)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__octet__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

ros2_plugin_proto__msg__RosMsgWrapper *
ros2_plugin_proto__msg__RosMsgWrapper__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_plugin_proto__msg__RosMsgWrapper * msg = (ros2_plugin_proto__msg__RosMsgWrapper *)allocator.allocate(sizeof(ros2_plugin_proto__msg__RosMsgWrapper), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_plugin_proto__msg__RosMsgWrapper));
  bool success = ros2_plugin_proto__msg__RosMsgWrapper__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_plugin_proto__msg__RosMsgWrapper__destroy(ros2_plugin_proto__msg__RosMsgWrapper * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_plugin_proto__msg__RosMsgWrapper__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_plugin_proto__msg__RosMsgWrapper__Sequence__init(ros2_plugin_proto__msg__RosMsgWrapper__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_plugin_proto__msg__RosMsgWrapper * data = NULL;

  if (size) {
    data = (ros2_plugin_proto__msg__RosMsgWrapper *)allocator.zero_allocate(size, sizeof(ros2_plugin_proto__msg__RosMsgWrapper), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_plugin_proto__msg__RosMsgWrapper__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_plugin_proto__msg__RosMsgWrapper__fini(&data[i - 1]);
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
ros2_plugin_proto__msg__RosMsgWrapper__Sequence__fini(ros2_plugin_proto__msg__RosMsgWrapper__Sequence * array)
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
      ros2_plugin_proto__msg__RosMsgWrapper__fini(&array->data[i]);
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

ros2_plugin_proto__msg__RosMsgWrapper__Sequence *
ros2_plugin_proto__msg__RosMsgWrapper__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_plugin_proto__msg__RosMsgWrapper__Sequence * array = (ros2_plugin_proto__msg__RosMsgWrapper__Sequence *)allocator.allocate(sizeof(ros2_plugin_proto__msg__RosMsgWrapper__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_plugin_proto__msg__RosMsgWrapper__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_plugin_proto__msg__RosMsgWrapper__Sequence__destroy(ros2_plugin_proto__msg__RosMsgWrapper__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_plugin_proto__msg__RosMsgWrapper__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_plugin_proto__msg__RosMsgWrapper__Sequence__are_equal(const ros2_plugin_proto__msg__RosMsgWrapper__Sequence * lhs, const ros2_plugin_proto__msg__RosMsgWrapper__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_plugin_proto__msg__RosMsgWrapper__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_plugin_proto__msg__RosMsgWrapper__Sequence__copy(
  const ros2_plugin_proto__msg__RosMsgWrapper__Sequence * input,
  ros2_plugin_proto__msg__RosMsgWrapper__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_plugin_proto__msg__RosMsgWrapper);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_plugin_proto__msg__RosMsgWrapper * data =
      (ros2_plugin_proto__msg__RosMsgWrapper *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_plugin_proto__msg__RosMsgWrapper__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_plugin_proto__msg__RosMsgWrapper__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_plugin_proto__msg__RosMsgWrapper__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
