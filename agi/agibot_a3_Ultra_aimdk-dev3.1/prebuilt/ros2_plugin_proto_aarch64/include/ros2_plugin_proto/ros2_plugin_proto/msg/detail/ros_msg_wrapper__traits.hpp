// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_plugin_proto:msg/RosMsgWrapper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_plugin_proto/msg/ros_msg_wrapper.hpp"


#ifndef ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__TRAITS_HPP_
#define ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_plugin_proto/msg/detail/ros_msg_wrapper__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_plugin_proto
{

namespace msg
{

inline void to_flow_style_yaml(
  const RosMsgWrapper & msg,
  std::ostream & out)
{
  out << "{";
  // member: serialization_type
  {
    out << "serialization_type: ";
    rosidl_generator_traits::value_to_yaml(msg.serialization_type, out);
    out << ", ";
  }

  // member: context
  {
    if (msg.context.size() == 0) {
      out << "context: []";
    } else {
      out << "context: [";
      size_t pending_items = msg.context.size();
      for (auto item : msg.context) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::character_value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RosMsgWrapper & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: serialization_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serialization_type: ";
    rosidl_generator_traits::value_to_yaml(msg.serialization_type, out);
    out << "\n";
  }

  // member: context
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.context.size() == 0) {
      out << "context: []\n";
    } else {
      out << "context:\n";
      for (auto item : msg.context) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::character_value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RosMsgWrapper & msg, bool use_flow_style = false)
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

}  // namespace ros2_plugin_proto

namespace rosidl_generator_traits
{

[[deprecated("use ros2_plugin_proto::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_plugin_proto::msg::RosMsgWrapper & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_plugin_proto::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_plugin_proto::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_plugin_proto::msg::RosMsgWrapper & msg)
{
  return ros2_plugin_proto::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_plugin_proto::msg::RosMsgWrapper>()
{
  return "ros2_plugin_proto::msg::RosMsgWrapper";
}

template<>
inline const char * name<ros2_plugin_proto::msg::RosMsgWrapper>()
{
  return "ros2_plugin_proto/msg/RosMsgWrapper";
}

template<>
struct has_fixed_size<ros2_plugin_proto::msg::RosMsgWrapper>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_plugin_proto::msg::RosMsgWrapper>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_plugin_proto::msg::RosMsgWrapper>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__TRAITS_HPP_
