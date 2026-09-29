// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_plugin_proto:msg/RosMsgWrapper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_plugin_proto/msg/ros_msg_wrapper.hpp"


#ifndef ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__BUILDER_HPP_
#define ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_plugin_proto/msg/detail/ros_msg_wrapper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_plugin_proto
{

namespace msg
{

namespace builder
{

class Init_RosMsgWrapper_data
{
public:
  explicit Init_RosMsgWrapper_data(::ros2_plugin_proto::msg::RosMsgWrapper & msg)
  : msg_(msg)
  {}
  ::ros2_plugin_proto::msg::RosMsgWrapper data(::ros2_plugin_proto::msg::RosMsgWrapper::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_plugin_proto::msg::RosMsgWrapper msg_;
};

class Init_RosMsgWrapper_context
{
public:
  explicit Init_RosMsgWrapper_context(::ros2_plugin_proto::msg::RosMsgWrapper & msg)
  : msg_(msg)
  {}
  Init_RosMsgWrapper_data context(::ros2_plugin_proto::msg::RosMsgWrapper::_context_type arg)
  {
    msg_.context = std::move(arg);
    return Init_RosMsgWrapper_data(msg_);
  }

private:
  ::ros2_plugin_proto::msg::RosMsgWrapper msg_;
};

class Init_RosMsgWrapper_serialization_type
{
public:
  Init_RosMsgWrapper_serialization_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RosMsgWrapper_context serialization_type(::ros2_plugin_proto::msg::RosMsgWrapper::_serialization_type_type arg)
  {
    msg_.serialization_type = std::move(arg);
    return Init_RosMsgWrapper_context(msg_);
  }

private:
  ::ros2_plugin_proto::msg::RosMsgWrapper msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_plugin_proto::msg::RosMsgWrapper>()
{
  return ros2_plugin_proto::msg::builder::Init_RosMsgWrapper_serialization_type();
}

}  // namespace ros2_plugin_proto

#endif  // ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__BUILDER_HPP_
