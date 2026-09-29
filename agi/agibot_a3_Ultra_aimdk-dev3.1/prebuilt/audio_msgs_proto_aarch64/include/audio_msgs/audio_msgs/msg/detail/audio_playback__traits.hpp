// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from audio_msgs:msg/AudioPlayback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/audio_playback.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__TRAITS_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "audio_msgs/msg/detail/audio_playback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamps'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'info'
#include "audio_msgs/msg/detail/audio_info__traits.hpp"
// Member 'data'
#include "audio_msgs/msg/detail/audio_data__traits.hpp"

namespace audio_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AudioPlayback & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamps
  {
    out << "stamps: ";
    to_flow_style_yaml(msg.stamps, out);
    out << ", ";
  }

  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: data
  {
    out << "data: ";
    to_flow_style_yaml(msg.data, out);
    out << ", ";
  }

  // member: pkg_name
  {
    out << "pkg_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pkg_name, out);
    out << ", ";
  }

  // member: token_id
  {
    out << "token_id: ";
    rosidl_generator_traits::value_to_yaml(msg.token_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AudioPlayback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamps:\n";
    to_block_style_yaml(msg.stamps, out, indentation + 2);
  }

  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data:\n";
    to_block_style_yaml(msg.data, out, indentation + 2);
  }

  // member: pkg_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pkg_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pkg_name, out);
    out << "\n";
  }

  // member: token_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "token_id: ";
    rosidl_generator_traits::value_to_yaml(msg.token_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AudioPlayback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace audio_msgs

namespace rosidl_generator_traits
{

[[deprecated("use audio_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const audio_msgs::msg::AudioPlayback & msg,
  std::ostream & out, size_t indentation = 0)
{
  audio_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use audio_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const audio_msgs::msg::AudioPlayback & msg)
{
  return audio_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<audio_msgs::msg::AudioPlayback>()
{
  return "audio_msgs::msg::AudioPlayback";
}

template<>
inline const char * name<audio_msgs::msg::AudioPlayback>()
{
  return "audio_msgs/msg/AudioPlayback";
}

template<>
struct has_fixed_size<audio_msgs::msg::AudioPlayback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<audio_msgs::msg::AudioPlayback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<audio_msgs::msg::AudioPlayback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__TRAITS_HPP_
