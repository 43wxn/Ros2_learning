// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/MoveRobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/srv/move_robot.hpp"


#ifndef INTERFACES__SRV__DETAIL__MOVE_ROBOT__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__MOVE_ROBOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/move_robot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveRobot_Request_distance
{
public:
  Init_MoveRobot_Request_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::MoveRobot_Request distance(::interfaces::srv::MoveRobot_Request::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::MoveRobot_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::MoveRobot_Request>()
{
  return interfaces::srv::builder::Init_MoveRobot_Request_distance();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveRobot_Response_pose
{
public:
  Init_MoveRobot_Response_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::MoveRobot_Response pose(::interfaces::srv::MoveRobot_Response::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::MoveRobot_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::MoveRobot_Response>()
{
  return interfaces::srv::builder::Init_MoveRobot_Response_pose();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveRobot_Event_response
{
public:
  explicit Init_MoveRobot_Event_response(::interfaces::srv::MoveRobot_Event & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::MoveRobot_Event response(::interfaces::srv::MoveRobot_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::MoveRobot_Event msg_;
};

class Init_MoveRobot_Event_request
{
public:
  explicit Init_MoveRobot_Event_request(::interfaces::srv::MoveRobot_Event & msg)
  : msg_(msg)
  {}
  Init_MoveRobot_Event_response request(::interfaces::srv::MoveRobot_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveRobot_Event_response(msg_);
  }

private:
  ::interfaces::srv::MoveRobot_Event msg_;
};

class Init_MoveRobot_Event_info
{
public:
  Init_MoveRobot_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveRobot_Event_request info(::interfaces::srv::MoveRobot_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveRobot_Event_request(msg_);
  }

private:
  ::interfaces::srv::MoveRobot_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::MoveRobot_Event>()
{
  return interfaces::srv::builder::Init_MoveRobot_Event_info();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__MOVE_ROBOT__BUILDER_HPP_
