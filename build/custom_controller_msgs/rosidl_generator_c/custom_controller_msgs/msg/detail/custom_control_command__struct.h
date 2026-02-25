// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_controller_msgs:msg/CustomControlCommand.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__STRUCT_H_
#define CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'joint_names'
#include "rosidl_runtime_c/string.h"
// Member 'position'
// Member 'velocity'
// Member 'effort'
// Member 'kp'
// Member 'kd'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'time_from_start'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/CustomControlCommand in the package custom_controller_msgs.
/**
  * CustomControlCommand.msg
 */
typedef struct custom_controller_msgs__msg__CustomControlCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence joint_names;
  /// 每个关节的控制命令
  /// 位置命令
  rosidl_runtime_c__double__Sequence position;
  /// 速度命令
  rosidl_runtime_c__double__Sequence velocity;
  /// 力矩命令
  rosidl_runtime_c__double__Sequence effort;
  /// 比例增益
  rosidl_runtime_c__double__Sequence kp;
  /// 微分增益
  rosidl_runtime_c__double__Sequence kd;
  /// 可选：时间戳或执行时间
  builtin_interfaces__msg__Duration time_from_start;
} custom_controller_msgs__msg__CustomControlCommand;

// Struct for a sequence of custom_controller_msgs__msg__CustomControlCommand.
typedef struct custom_controller_msgs__msg__CustomControlCommand__Sequence
{
  custom_controller_msgs__msg__CustomControlCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_controller_msgs__msg__CustomControlCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__STRUCT_H_
