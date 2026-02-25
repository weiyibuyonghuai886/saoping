// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_controller_msgs:msg/CustomControlCommand.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__BUILDER_HPP_
#define CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_controller_msgs/msg/detail/custom_control_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_controller_msgs
{

namespace msg
{

namespace builder
{

class Init_CustomControlCommand_time_from_start
{
public:
  explicit Init_CustomControlCommand_time_from_start(::custom_controller_msgs::msg::CustomControlCommand & msg)
  : msg_(msg)
  {}
  ::custom_controller_msgs::msg::CustomControlCommand time_from_start(::custom_controller_msgs::msg::CustomControlCommand::_time_from_start_type arg)
  {
    msg_.time_from_start = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

class Init_CustomControlCommand_kd
{
public:
  explicit Init_CustomControlCommand_kd(::custom_controller_msgs::msg::CustomControlCommand & msg)
  : msg_(msg)
  {}
  Init_CustomControlCommand_time_from_start kd(::custom_controller_msgs::msg::CustomControlCommand::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_CustomControlCommand_time_from_start(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

class Init_CustomControlCommand_kp
{
public:
  explicit Init_CustomControlCommand_kp(::custom_controller_msgs::msg::CustomControlCommand & msg)
  : msg_(msg)
  {}
  Init_CustomControlCommand_kd kp(::custom_controller_msgs::msg::CustomControlCommand::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_CustomControlCommand_kd(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

class Init_CustomControlCommand_effort
{
public:
  explicit Init_CustomControlCommand_effort(::custom_controller_msgs::msg::CustomControlCommand & msg)
  : msg_(msg)
  {}
  Init_CustomControlCommand_kp effort(::custom_controller_msgs::msg::CustomControlCommand::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_CustomControlCommand_kp(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

class Init_CustomControlCommand_velocity
{
public:
  explicit Init_CustomControlCommand_velocity(::custom_controller_msgs::msg::CustomControlCommand & msg)
  : msg_(msg)
  {}
  Init_CustomControlCommand_effort velocity(::custom_controller_msgs::msg::CustomControlCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_CustomControlCommand_effort(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

class Init_CustomControlCommand_position
{
public:
  explicit Init_CustomControlCommand_position(::custom_controller_msgs::msg::CustomControlCommand & msg)
  : msg_(msg)
  {}
  Init_CustomControlCommand_velocity position(::custom_controller_msgs::msg::CustomControlCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_CustomControlCommand_velocity(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

class Init_CustomControlCommand_joint_names
{
public:
  explicit Init_CustomControlCommand_joint_names(::custom_controller_msgs::msg::CustomControlCommand & msg)
  : msg_(msg)
  {}
  Init_CustomControlCommand_position joint_names(::custom_controller_msgs::msg::CustomControlCommand::_joint_names_type arg)
  {
    msg_.joint_names = std::move(arg);
    return Init_CustomControlCommand_position(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

class Init_CustomControlCommand_header
{
public:
  Init_CustomControlCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomControlCommand_joint_names header(::custom_controller_msgs::msg::CustomControlCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CustomControlCommand_joint_names(msg_);
  }

private:
  ::custom_controller_msgs::msg::CustomControlCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_controller_msgs::msg::CustomControlCommand>()
{
  return custom_controller_msgs::msg::builder::Init_CustomControlCommand_header();
}

}  // namespace custom_controller_msgs

#endif  // CUSTOM_CONTROLLER_MSGS__MSG__DETAIL__CUSTOM_CONTROL_COMMAND__BUILDER_HPP_
