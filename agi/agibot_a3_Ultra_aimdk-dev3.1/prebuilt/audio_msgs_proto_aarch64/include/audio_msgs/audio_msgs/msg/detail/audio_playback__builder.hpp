// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from audio_msgs:msg/AudioPlayback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/audio_playback.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__BUILDER_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "audio_msgs/msg/detail/audio_playback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace audio_msgs
{

namespace msg
{

namespace builder
{

class Init_AudioPlayback_token_id
{
public:
  explicit Init_AudioPlayback_token_id(::audio_msgs::msg::AudioPlayback & msg)
  : msg_(msg)
  {}
  ::audio_msgs::msg::AudioPlayback token_id(::audio_msgs::msg::AudioPlayback::_token_id_type arg)
  {
    msg_.token_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_msgs::msg::AudioPlayback msg_;
};

class Init_AudioPlayback_pkg_name
{
public:
  explicit Init_AudioPlayback_pkg_name(::audio_msgs::msg::AudioPlayback & msg)
  : msg_(msg)
  {}
  Init_AudioPlayback_token_id pkg_name(::audio_msgs::msg::AudioPlayback::_pkg_name_type arg)
  {
    msg_.pkg_name = std::move(arg);
    return Init_AudioPlayback_token_id(msg_);
  }

private:
  ::audio_msgs::msg::AudioPlayback msg_;
};

class Init_AudioPlayback_data
{
public:
  explicit Init_AudioPlayback_data(::audio_msgs::msg::AudioPlayback & msg)
  : msg_(msg)
  {}
  Init_AudioPlayback_pkg_name data(::audio_msgs::msg::AudioPlayback::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_AudioPlayback_pkg_name(msg_);
  }

private:
  ::audio_msgs::msg::AudioPlayback msg_;
};

class Init_AudioPlayback_info
{
public:
  explicit Init_AudioPlayback_info(::audio_msgs::msg::AudioPlayback & msg)
  : msg_(msg)
  {}
  Init_AudioPlayback_data info(::audio_msgs::msg::AudioPlayback::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_AudioPlayback_data(msg_);
  }

private:
  ::audio_msgs::msg::AudioPlayback msg_;
};

class Init_AudioPlayback_stamps
{
public:
  Init_AudioPlayback_stamps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AudioPlayback_info stamps(::audio_msgs::msg::AudioPlayback::_stamps_type arg)
  {
    msg_.stamps = std::move(arg);
    return Init_AudioPlayback_info(msg_);
  }

private:
  ::audio_msgs::msg::AudioPlayback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_msgs::msg::AudioPlayback>()
{
  return audio_msgs::msg::builder::Init_AudioPlayback_stamps();
}

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__BUILDER_HPP_
