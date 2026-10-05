// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_plugin_proto:srv/RosRpcWrapper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_plugin_proto/srv/ros_rpc_wrapper.hpp"


#ifndef ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__TRAITS_HPP_
#define ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_plugin_proto/srv/detail/ros_rpc_wrapper__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_plugin_proto
{

namespace srv
{

inline void to_flow_style_yaml(
  const RosRpcWrapper_Request & msg,
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
  const RosRpcWrapper_Request & msg,
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

inline std::string to_yaml(const RosRpcWrapper_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2_plugin_proto

namespace rosidl_generator_traits
{

[[deprecated("use ros2_plugin_proto::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_plugin_proto::srv::RosRpcWrapper_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_plugin_proto::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_plugin_proto::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_plugin_proto::srv::RosRpcWrapper_Request & msg)
{
  return ros2_plugin_proto::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_plugin_proto::srv::RosRpcWrapper_Request>()
{
  return "ros2_plugin_proto::srv::RosRpcWrapper_Request";
}

template<>
inline const char * name<ros2_plugin_proto::srv::RosRpcWrapper_Request>()
{
  return "ros2_plugin_proto/srv/RosRpcWrapper_Request";
}

template<>
struct has_fixed_size<ros2_plugin_proto::srv::RosRpcWrapper_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_plugin_proto::srv::RosRpcWrapper_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2_plugin_proto
{

namespace srv
{

inline void to_flow_style_yaml(
  const RosRpcWrapper_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: code
  {
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << ", ";
  }

  // member: serialization_type
  {
    out << "serialization_type: ";
    rosidl_generator_traits::value_to_yaml(msg.serialization_type, out);
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
  const RosRpcWrapper_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << "\n";
  }

  // member: serialization_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serialization_type: ";
    rosidl_generator_traits::value_to_yaml(msg.serialization_type, out);
    out << "\n";
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

inline std::string to_yaml(const RosRpcWrapper_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2_plugin_proto

namespace rosidl_generator_traits
{

[[deprecated("use ros2_plugin_proto::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_plugin_proto::srv::RosRpcWrapper_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_plugin_proto::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_plugin_proto::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_plugin_proto::srv::RosRpcWrapper_Response & msg)
{
  return ros2_plugin_proto::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_plugin_proto::srv::RosRpcWrapper_Response>()
{
  return "ros2_plugin_proto::srv::RosRpcWrapper_Response";
}

template<>
inline const char * name<ros2_plugin_proto::srv::RosRpcWrapper_Response>()
{
  return "ros2_plugin_proto/srv/RosRpcWrapper_Response";
}

template<>
struct has_fixed_size<ros2_plugin_proto::srv::RosRpcWrapper_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_plugin_proto::srv::RosRpcWrapper_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace ros2_plugin_proto
{

namespace srv
{

inline void to_flow_style_yaml(
  const RosRpcWrapper_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
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
  const RosRpcWrapper_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RosRpcWrapper_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2_plugin_proto

namespace rosidl_generator_traits
{

[[deprecated("use ros2_plugin_proto::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_plugin_proto::srv::RosRpcWrapper_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_plugin_proto::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_plugin_proto::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_plugin_proto::srv::RosRpcWrapper_Event & msg)
{
  return ros2_plugin_proto::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_plugin_proto::srv::RosRpcWrapper_Event>()
{
  return "ros2_plugin_proto::srv::RosRpcWrapper_Event";
}

template<>
inline const char * name<ros2_plugin_proto::srv::RosRpcWrapper_Event>()
{
  return "ros2_plugin_proto/srv/RosRpcWrapper_Event";
}

template<>
struct has_fixed_size<ros2_plugin_proto::srv::RosRpcWrapper_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper_Event>
  : std::integral_constant<bool, has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper_Request>::value && has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<ros2_plugin_proto::srv::RosRpcWrapper_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_plugin_proto::srv::RosRpcWrapper>()
{
  return "ros2_plugin_proto::srv::RosRpcWrapper";
}

template<>
inline const char * name<ros2_plugin_proto::srv::RosRpcWrapper>()
{
  return "ros2_plugin_proto/srv/RosRpcWrapper";
}

template<>
struct has_fixed_size<ros2_plugin_proto::srv::RosRpcWrapper>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_plugin_proto::srv::RosRpcWrapper_Request>::value &&
    has_fixed_size<ros2_plugin_proto::srv::RosRpcWrapper_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper_Request>::value &&
    has_bounded_size<ros2_plugin_proto::srv::RosRpcWrapper_Response>::value
  >
{
};

template<>
struct is_service<ros2_plugin_proto::srv::RosRpcWrapper>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_plugin_proto::srv::RosRpcWrapper_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_plugin_proto::srv::RosRpcWrapper_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__TRAITS_HPP_
