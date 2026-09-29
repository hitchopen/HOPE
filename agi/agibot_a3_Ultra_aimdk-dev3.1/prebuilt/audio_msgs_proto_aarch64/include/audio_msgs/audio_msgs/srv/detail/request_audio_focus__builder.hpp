// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from audio_msgs:srv/RequestAudioFocus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/srv/request_audio_focus.hpp"


#ifndef AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__BUILDER_HPP_
#define AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "audio_msgs/srv/detail/request_audio_focus__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace audio_msgs
{

namespace srv
{

namespace builder
{

class Init_RequestAudioFocus_Request_focus_requester
{
public:
  explicit Init_RequestAudioFocus_Request_focus_requester(::audio_msgs::srv::RequestAudioFocus_Request & msg)
  : msg_(msg)
  {}
  ::audio_msgs::srv::RequestAudioFocus_Request focus_requester(::audio_msgs::srv::RequestAudioFocus_Request::_focus_requester_type arg)
  {
    msg_.focus_requester = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_msgs::srv::RequestAudioFocus_Request msg_;
};

class Init_RequestAudioFocus_Request_header
{
public:
  Init_RequestAudioFocus_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestAudioFocus_Request_focus_requester header(::audio_msgs::srv::RequestAudioFocus_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RequestAudioFocus_Request_focus_requester(msg_);
  }

private:
  ::audio_msgs::srv::RequestAudioFocus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_msgs::srv::RequestAudioFocus_Request>()
{
  return audio_msgs::srv::builder::Init_RequestAudioFocus_Request_header();
}

}  // namespace audio_msgs


namespace audio_msgs
{

namespace srv
{

namespace builder
{

class Init_RequestAudioFocus_Response_focus_response
{
public:
  explicit Init_RequestAudioFocus_Response_focus_response(::audio_msgs::srv::RequestAudioFocus_Response & msg)
  : msg_(msg)
  {}
  ::audio_msgs::srv::RequestAudioFocus_Response focus_response(::audio_msgs::srv::RequestAudioFocus_Response::_focus_response_type arg)
  {
    msg_.focus_response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_msgs::srv::RequestAudioFocus_Response msg_;
};

class Init_RequestAudioFocus_Response_header
{
public:
  Init_RequestAudioFocus_Response_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestAudioFocus_Response_focus_response header(::audio_msgs::srv::RequestAudioFocus_Response::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RequestAudioFocus_Response_focus_response(msg_);
  }

private:
  ::audio_msgs::srv::RequestAudioFocus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_msgs::srv::RequestAudioFocus_Response>()
{
  return audio_msgs::srv::builder::Init_RequestAudioFocus_Response_header();
}

}  // namespace audio_msgs


namespace audio_msgs
{

namespace srv
{

namespace builder
{

class Init_RequestAudioFocus_Event_response
{
public:
  explicit Init_RequestAudioFocus_Event_response(::audio_msgs::srv::RequestAudioFocus_Event & msg)
  : msg_(msg)
  {}
  ::audio_msgs::srv::RequestAudioFocus_Event response(::audio_msgs::srv::RequestAudioFocus_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_msgs::srv::RequestAudioFocus_Event msg_;
};

class Init_RequestAudioFocus_Event_request
{
public:
  explicit Init_RequestAudioFocus_Event_request(::audio_msgs::srv::RequestAudioFocus_Event & msg)
  : msg_(msg)
  {}
  Init_RequestAudioFocus_Event_response request(::audio_msgs::srv::RequestAudioFocus_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RequestAudioFocus_Event_response(msg_);
  }

private:
  ::audio_msgs::srv::RequestAudioFocus_Event msg_;
};

class Init_RequestAudioFocus_Event_info
{
public:
  Init_RequestAudioFocus_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestAudioFocus_Event_request info(::audio_msgs::srv::RequestAudioFocus_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RequestAudioFocus_Event_request(msg_);
  }

private:
  ::audio_msgs::srv::RequestAudioFocus_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_msgs::srv::RequestAudioFocus_Event>()
{
  return audio_msgs::srv::builder::Init_RequestAudioFocus_Event_info();
}

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__BUILDER_HPP_
