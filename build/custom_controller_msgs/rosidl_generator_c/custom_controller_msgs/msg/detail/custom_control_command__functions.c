// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_controller_msgs:msg/CustomControlCommand.idl
// generated code does not contain a copyright notice
#include "custom_controller_msgs/msg/detail/custom_control_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `effort`
// Member `kp`
// Member `kd`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `time_from_start`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
custom_controller_msgs__msg__CustomControlCommand__init(custom_controller_msgs__msg__CustomControlCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->joint_names, 0)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->effort, 0)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  // kp
  if (!rosidl_runtime_c__double__Sequence__init(&msg->kp, 0)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  // kd
  if (!rosidl_runtime_c__double__Sequence__init(&msg->kd, 0)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  // time_from_start
  if (!builtin_interfaces__msg__Duration__init(&msg->time_from_start)) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
    return false;
  }
  return true;
}

void
custom_controller_msgs__msg__CustomControlCommand__fini(custom_controller_msgs__msg__CustomControlCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // joint_names
  rosidl_runtime_c__String__Sequence__fini(&msg->joint_names);
  // position
  rosidl_runtime_c__double__Sequence__fini(&msg->position);
  // velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->velocity);
  // effort
  rosidl_runtime_c__double__Sequence__fini(&msg->effort);
  // kp
  rosidl_runtime_c__double__Sequence__fini(&msg->kp);
  // kd
  rosidl_runtime_c__double__Sequence__fini(&msg->kd);
  // time_from_start
  builtin_interfaces__msg__Duration__fini(&msg->time_from_start);
}

bool
custom_controller_msgs__msg__CustomControlCommand__are_equal(const custom_controller_msgs__msg__CustomControlCommand * lhs, const custom_controller_msgs__msg__CustomControlCommand * rhs)
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
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->joint_names), &(rhs->joint_names)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->effort), &(rhs->effort)))
  {
    return false;
  }
  // kp
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->kp), &(rhs->kp)))
  {
    return false;
  }
  // kd
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->kd), &(rhs->kd)))
  {
    return false;
  }
  // time_from_start
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->time_from_start), &(rhs->time_from_start)))
  {
    return false;
  }
  return true;
}

bool
custom_controller_msgs__msg__CustomControlCommand__copy(
  const custom_controller_msgs__msg__CustomControlCommand * input,
  custom_controller_msgs__msg__CustomControlCommand * output)
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
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->joint_names), &(output->joint_names)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->effort), &(output->effort)))
  {
    return false;
  }
  // kp
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->kp), &(output->kp)))
  {
    return false;
  }
  // kd
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->kd), &(output->kd)))
  {
    return false;
  }
  // time_from_start
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->time_from_start), &(output->time_from_start)))
  {
    return false;
  }
  return true;
}

custom_controller_msgs__msg__CustomControlCommand *
custom_controller_msgs__msg__CustomControlCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_controller_msgs__msg__CustomControlCommand * msg = (custom_controller_msgs__msg__CustomControlCommand *)allocator.allocate(sizeof(custom_controller_msgs__msg__CustomControlCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_controller_msgs__msg__CustomControlCommand));
  bool success = custom_controller_msgs__msg__CustomControlCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_controller_msgs__msg__CustomControlCommand__destroy(custom_controller_msgs__msg__CustomControlCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_controller_msgs__msg__CustomControlCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_controller_msgs__msg__CustomControlCommand__Sequence__init(custom_controller_msgs__msg__CustomControlCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_controller_msgs__msg__CustomControlCommand * data = NULL;

  if (size) {
    data = (custom_controller_msgs__msg__CustomControlCommand *)allocator.zero_allocate(size, sizeof(custom_controller_msgs__msg__CustomControlCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_controller_msgs__msg__CustomControlCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_controller_msgs__msg__CustomControlCommand__fini(&data[i - 1]);
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
custom_controller_msgs__msg__CustomControlCommand__Sequence__fini(custom_controller_msgs__msg__CustomControlCommand__Sequence * array)
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
      custom_controller_msgs__msg__CustomControlCommand__fini(&array->data[i]);
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

custom_controller_msgs__msg__CustomControlCommand__Sequence *
custom_controller_msgs__msg__CustomControlCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_controller_msgs__msg__CustomControlCommand__Sequence * array = (custom_controller_msgs__msg__CustomControlCommand__Sequence *)allocator.allocate(sizeof(custom_controller_msgs__msg__CustomControlCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_controller_msgs__msg__CustomControlCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_controller_msgs__msg__CustomControlCommand__Sequence__destroy(custom_controller_msgs__msg__CustomControlCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_controller_msgs__msg__CustomControlCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_controller_msgs__msg__CustomControlCommand__Sequence__are_equal(const custom_controller_msgs__msg__CustomControlCommand__Sequence * lhs, const custom_controller_msgs__msg__CustomControlCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_controller_msgs__msg__CustomControlCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_controller_msgs__msg__CustomControlCommand__Sequence__copy(
  const custom_controller_msgs__msg__CustomControlCommand__Sequence * input,
  custom_controller_msgs__msg__CustomControlCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_controller_msgs__msg__CustomControlCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_controller_msgs__msg__CustomControlCommand * data =
      (custom_controller_msgs__msg__CustomControlCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_controller_msgs__msg__CustomControlCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_controller_msgs__msg__CustomControlCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_controller_msgs__msg__CustomControlCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
