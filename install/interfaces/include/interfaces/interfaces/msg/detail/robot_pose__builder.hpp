// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/RobotPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/robot_pose.hpp"


#ifndef INTERFACES__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/robot_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotPose_pose
{
public:
  explicit Init_RobotPose_pose(::interfaces::msg::RobotPose & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::RobotPose pose(::interfaces::msg::RobotPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::RobotPose msg_;
};

class Init_RobotPose_status
{
public:
  Init_RobotPose_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotPose_pose status(::interfaces::msg::RobotPose::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RobotPose_pose(msg_);
  }

private:
  ::interfaces::msg::RobotPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::RobotPose>()
{
  return interfaces::msg::builder::Init_RobotPose_status();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_
