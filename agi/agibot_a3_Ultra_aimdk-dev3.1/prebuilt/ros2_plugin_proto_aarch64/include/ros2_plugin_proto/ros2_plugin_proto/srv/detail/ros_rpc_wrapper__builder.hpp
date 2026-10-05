// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_plugin_proto:srv/RosRpcWrapper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_plugin_proto/srv/ros_rpc_wrapper.hpp"


#ifndef ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__BUILDER_HPP_
#define ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_plugin_proto
{

namespace srv
{

namespace builder
{

class Init_RosRpcWrapper_Request_data
{
public:
  explicit Init_RosRpcWrapper_Request_data(::ros2_plugin_proto::srv::RosRpcWrapper_Request & msg)
  : msg_(msg)
  {}
  ::ros2_plugin_proto::srv::RosRpcWrapper_Request data(::ros2_plugin_proto::srv::RosRpcWrapper_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Request msg_;
};

class Init_RosRpcWrapper_Request_context
{
public:
  explicit Init_RosRpcWrapper_Request_context(::ros2_plugin_proto::srv::RosRpcWrapper_Request & msg)
  : msg_(msg)
  {}
  Init_RosRpcWrapper_Request_data context(::ros2_plugin_proto::srv::RosRpcWrapper_Request::_context_type arg)
  {
    msg_.context = std::move(arg);
    return Init_RosRpcWrapper_Request_data(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Request msg_;
};

class Init_RosRpcWrapper_Request_serialization_type
{
public:
  Init_RosRpcWrapper_Request_serialization_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RosRpcWrapper_Request_context serialization_type(::ros2_plugin_proto::srv::RosRpcWrapper_Request::_serialization_type_type arg)
  {
    msg_.serialization_type = std::move(arg);
    return Init_RosRpcWrapper_Request_context(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_plugin_proto::srv::RosRpcWrapper_Request>()
{
  return ros2_plugin_proto::srv::builder::Init_RosRpcWrapper_Request_serialization_type();
}

}  // namespace ros2_plugin_proto


namespace ros2_plugin_proto
{

namespace srv
{

namespace builder
{

class Init_RosRpcWrapper_Response_data
{
public:
  explicit Init_RosRpcWrapper_Response_data(::ros2_plugin_proto::srv::RosRpcWrapper_Response & msg)
  : msg_(msg)
  {}
  ::ros2_plugin_proto::srv::RosRpcWrapper_Response data(::ros2_plugin_proto::srv::RosRpcWrapper_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Response msg_;
};

class Init_RosRpcWrapper_Response_serialization_type
{
public:
  explicit Init_RosRpcWrapper_Response_serialization_type(::ros2_plugin_proto::srv::RosRpcWrapper_Response & msg)
  : msg_(msg)
  {}
  Init_RosRpcWrapper_Response_data serialization_type(::ros2_plugin_proto::srv::RosRpcWrapper_Response::_serialization_type_type arg)
  {
    msg_.serialization_type = std::move(arg);
    return Init_RosRpcWrapper_Response_data(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Response msg_;
};

class Init_RosRpcWrapper_Response_code
{
public:
  Init_RosRpcWrapper_Response_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RosRpcWrapper_Response_serialization_type code(::ros2_plugin_proto::srv::RosRpcWrapper_Response::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_RosRpcWrapper_Response_serialization_type(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_plugin_proto::srv::RosRpcWrapper_Response>()
{
  return ros2_plugin_proto::srv::builder::Init_RosRpcWrapper_Response_code();
}

}  // namespace ros2_plugin_proto


namespace ros2_plugin_proto
{

namespace srv
{

namespace builder
{

class Init_RosRpcWrapper_Event_response
{
public:
  explicit Init_RosRpcWrapper_Event_response(::ros2_plugin_proto::srv::RosRpcWrapper_Event & msg)
  : msg_(msg)
  {}
  ::ros2_plugin_proto::srv::RosRpcWrapper_Event response(::ros2_plugin_proto::srv::RosRpcWrapper_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Event msg_;
};

class Init_RosRpcWrapper_Event_request
{
public:
  explicit Init_RosRpcWrapper_Event_request(::ros2_plugin_proto::srv::RosRpcWrapper_Event & msg)
  : msg_(msg)
  {}
  Init_RosRpcWrapper_Event_response request(::ros2_plugin_proto::srv::RosRpcWrapper_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RosRpcWrapper_Event_response(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Event msg_;
};

class Init_RosRpcWrapper_Event_info
{
public:
  Init_RosRpcWrapper_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RosRpcWrapper_Event_request info(::ros2_plugin_proto::srv::RosRpcWrapper_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RosRpcWrapper_Event_request(msg_);
  }

private:
  ::ros2_plugin_proto::srv::RosRpcWrapper_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_plugin_proto::srv::RosRpcWrapper_Event>()
{
  return ros2_plugin_proto::srv::builder::Init_RosRpcWrapper_Event_info();
}

}  // namespace ros2_plugin_proto

#endif  // ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__BUILDER_HPP_
