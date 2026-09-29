// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from audio_msgs:msg/FocusResponse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/focus_response.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__TRAITS_HPP_
#define AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "audio_msgs/msg/detail/focus_response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace audio_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FocusResponse & msg,
  std::ostream & out)
{
  out << "{";
  // member: pkg_name
  {
    out << "pkg_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pkg_name, out);
    out << ", ";
  }

  // member: focus_gain
  {
    out << "focus_gain: ";
    rosidl_generator_traits::value_to_yaml(msg.focus_gain, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FocusResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pkg_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pkg_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pkg_name, out);
    out << "\n";
  }

  // member: focus_gain
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "focus_gain: ";
    rosidl_generator_traits::value_to_yaml(msg.focus_gain, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FocusResponse & msg, bool use_flow_style = false)
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
  const audio_msgs::msg::FocusResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  audio_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use audio_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const audio_msgs::msg::FocusResponse & msg)
{
  return audio_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<audio_msgs::msg::FocusResponse>()
{
  return "audio_msgs::msg::FocusResponse";
}

template<>
inline const char * name<audio_msgs::msg::FocusResponse>()
{
  return "audio_msgs/msg/FocusResponse";
}

template<>
struct has_fixed_size<audio_msgs::msg::FocusResponse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<audio_msgs::msg::FocusResponse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<audio_msgs::msg::FocusResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUDIO_MSGS__MSG__DETAIL__FOCUS_RESPONSE__TRAITS_HPP_
