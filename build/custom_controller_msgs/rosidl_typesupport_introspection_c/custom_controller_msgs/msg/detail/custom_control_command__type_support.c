// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_controller_msgs:msg/CustomControlCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_controller_msgs/msg/detail/custom_control_command__rosidl_typesupport_introspection_c.h"
#include "custom_controller_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_controller_msgs/msg/detail/custom_control_command__functions.h"
#include "custom_controller_msgs/msg/detail/custom_control_command__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `effort`
// Member `kp`
// Member `kd`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `time_from_start`
#include "builtin_interfaces/msg/duration.h"
// Member `time_from_start`
#include "builtin_interfaces/msg/detail/duration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_controller_msgs__msg__CustomControlCommand__init(message_memory);
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_fini_function(void * message_memory)
{
  custom_controller_msgs__msg__CustomControlCommand__fini(message_memory);
}

size_t custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__joint_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__joint_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__joint_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__joint_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__joint_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__joint_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__joint_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__joint_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__kp(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__kp(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__kp(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__kp(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__kp(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__kp(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__kd(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__kd(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__kd(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__kd(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__kd(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__kd(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, joint_names),  // bytes offset in struct
    NULL,  // default value
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__joint_names,  // size() function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__joint_names,  // get_const(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__joint_names,  // get(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__joint_names,  // fetch(index, &value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__joint_names,  // assign(index, value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__joint_names  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, position),  // bytes offset in struct
    NULL,  // default value
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__position,  // size() function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__position,  // get_const(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__position,  // get(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__position,  // fetch(index, &value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__position,  // assign(index, value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, velocity),  // bytes offset in struct
    NULL,  // default value
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__velocity,  // size() function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__velocity,  // get_const(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__velocity,  // get(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__velocity,  // fetch(index, &value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__velocity,  // assign(index, value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__velocity  // resize(index) function pointer
  },
  {
    "effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, effort),  // bytes offset in struct
    NULL,  // default value
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__effort,  // size() function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__effort,  // get_const(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__effort,  // get(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__effort,  // fetch(index, &value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__effort,  // assign(index, value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__effort  // resize(index) function pointer
  },
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, kp),  // bytes offset in struct
    NULL,  // default value
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__kp,  // size() function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__kp,  // get_const(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__kp,  // get(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__kp,  // fetch(index, &value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__kp,  // assign(index, value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__kp  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, kd),  // bytes offset in struct
    NULL,  // default value
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__size_function__CustomControlCommand__kd,  // size() function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_const_function__CustomControlCommand__kd,  // get_const(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__get_function__CustomControlCommand__kd,  // get(index) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__fetch_function__CustomControlCommand__kd,  // fetch(index, &value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__assign_function__CustomControlCommand__kd,  // assign(index, value) function pointer
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__resize_function__CustomControlCommand__kd  // resize(index) function pointer
  },
  {
    "time_from_start",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_controller_msgs__msg__CustomControlCommand, time_from_start),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_members = {
  "custom_controller_msgs__msg",  // message namespace
  "CustomControlCommand",  // message name
  8,  // number of fields
  sizeof(custom_controller_msgs__msg__CustomControlCommand),
  custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_member_array,  // message members
  custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_type_support_handle = {
  0,
  &custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_controller_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_controller_msgs, msg, CustomControlCommand)() {
  custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  if (!custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_type_support_handle.typesupport_identifier) {
    custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_controller_msgs__msg__CustomControlCommand__rosidl_typesupport_introspection_c__CustomControlCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
