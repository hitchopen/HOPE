// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from audio_msgs:msg/FocusResponse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/focus_response.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__BUILDER_HPP_
#define AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "audio_msgs/msg/detail/focus_response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace audio_msgs
{

namespace msg
{

namespace builder
{

class Init_FocusResponse_focus_gain
{
public:
  explicit Init_FocusResponse_focus_gain(::audio_msgs::msg::FocusResponse & msg)
  : msg_(msg)
  {}
  ::audio_msgs::msg::FocusResponse focus_gain(::audio_msgs::msg::FocusResponse::_focus_gain_type arg)
  {
    msg_.focus_gain = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_msgs::msg::FocusResponse msg_;
};

class Init_FocusResponse_pkg_name
{
public:
  Init_FocusResponse_pkg_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FocusResponse_focus_gain pkg_name(::audio_msgs::msg::FocusResponse::_pkg_name_type arg)
  {
    msg_.pkg_name = std::move(arg);
    return Init_FocusResponse_focus_gain(msg_);
  }

private:
  ::audio_msgs::msg::FocusResponse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_msgs::msg::FocusResponse>()
{
  return audio_msgs::msg::builder::Init_FocusResponse_pkg_name();
}

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__BUILDER_HPP_
