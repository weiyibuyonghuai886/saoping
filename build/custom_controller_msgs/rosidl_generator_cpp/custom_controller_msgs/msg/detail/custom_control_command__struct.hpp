// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_controller_msgs:msg/CustomControlCommand.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__STRUCT_HPP_
#define CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'time_from_start'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_controller_msgs__msg__CustomControlCommand __attribute__((deprecated))
#else
# define DEPRECATED__custom_controller_msgs__msg__CustomControlCommand __declspec(deprecated)
#endif

namespace custom_controller_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CustomControlCommand_
{
  using Type = CustomControlCommand_<ContainerAllocator>;

  explicit CustomControlCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    time_from_start(_init)
  {
    (void)_init;
  }

  explicit CustomControlCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    time_from_start(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _joint_names_type joint_names;
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _velocity_type velocity;
  using _effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _effort_type effort;
  using _kp_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _kp_type kp;
  using _kd_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _kd_type kd;
  using _time_from_start_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_from_start_type time_from_start;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__kp(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__time_from_start(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_from_start = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_controller_msgs__msg__CustomControlCommand
    std::shared_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_controller_msgs__msg__CustomControlCommand
    std::shared_ptr<custom_controller_msgs::msg::CustomControlCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomControlCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_names != other.joint_names) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->time_from_start != other.time_from_start) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomControlCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomControlCommand_

// alias to use template instance with default allocator
using CustomControlCommand =
  custom_controller_msgs::msg::CustomControlCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_controller_msgs

#endif  // CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__STRUCT_HPP_
