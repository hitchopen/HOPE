// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from audio_msgs:srv/RequestAudioFocus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/srv/request_audio_focus.hpp"


#ifndef AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__TRAITS_HPP_
#define AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "audio_msgs/srv/detail/request_audio_focus__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'focus_requester'
#include "audio_msgs/msg/detail/focus_requester__traits.hpp"

namespace audio_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestAudioFocus_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: focus_requester
  {
    out << "focus_requester: ";
    to_flow_style_yaml(msg.focus_requester, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RequestAudioFocus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: focus_requester
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "focus_requester:\n";
    to_block_style_yaml(msg.focus_requester, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RequestAudioFocus_Request & msg, bool use_flow_style = false)
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

}  // namespace audio_msgs

namespace rosidl_generator_traits
{

[[deprecated("use audio_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const audio_msgs::srv::RequestAudioFocus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  audio_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use audio_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const audio_msgs::srv::RequestAudioFocus_Request & msg)
{
  return audio_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<audio_msgs::srv::RequestAudioFocus_Request>()
{
  return "audio_msgs::srv::RequestAudioFocus_Request";
}

template<>
inline const char * name<audio_msgs::srv::RequestAudioFocus_Request>()
{
  return "audio_msgs/srv/RequestAudioFocus_Request";
}

template<>
struct has_fixed_size<audio_msgs::srv::RequestAudioFocus_Request>
  : std::integral_constant<bool, has_fixed_size<audio_msgs::msg::FocusRequester>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<audio_msgs::srv::RequestAudioFocus_Request>
  : std::integral_constant<bool, has_bounded_size<audio_msgs::msg::FocusRequester>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<audio_msgs::srv::RequestAudioFocus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'header'
// already included above
// #include "std_msgs/msg/detail/header__traits.hpp"
// Member 'focus_response'
#include "audio_msgs/msg/detail/focus_response__traits.hpp"

namespace audio_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestAudioFocus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: focus_response
  {
    out << "focus_response: ";
    to_flow_style_yaml(msg.focus_response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RequestAudioFocus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: focus_response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "focus_response:\n";
    to_block_style_yaml(msg.focus_response, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RequestAudioFocus_Response & msg, bool use_flow_style = false)
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

}  // namespace audio_msgs

namespace rosidl_generator_traits
{

[[deprecated("use audio_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const audio_msgs::srv::RequestAudioFocus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  audio_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use audio_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const audio_msgs::srv::RequestAudioFocus_Response & msg)
{
  return audio_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<audio_msgs::srv::RequestAudioFocus_Response>()
{
  return "audio_msgs::srv::RequestAudioFocus_Response";
}

template<>
inline const char * name<audio_msgs::srv::RequestAudioFocus_Response>()
{
  return "audio_msgs/srv/RequestAudioFocus_Response";
}

template<>
struct has_fixed_size<audio_msgs::srv::RequestAudioFocus_Response>
  : std::integral_constant<bool, has_fixed_size<audio_msgs::msg::FocusResponse>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<audio_msgs::srv::RequestAudioFocus_Response>
  : std::integral_constant<bool, has_bounded_size<audio_msgs::msg::FocusResponse>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<audio_msgs::srv::RequestAudioFocus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace audio_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestAudioFocus_Event & msg,
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
  const RequestAudioFocus_Event & msg,
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

inline std::string to_yaml(const RequestAudioFocus_Event & msg, bool use_flow_style = false)
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

}  // namespace audio_msgs

namespace rosidl_generator_traits
{

[[deprecated("use audio_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const audio_msgs::srv::RequestAudioFocus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  audio_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use audio_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const audio_msgs::srv::RequestAudioFocus_Event & msg)
{
  return audio_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<audio_msgs::srv::RequestAudioFocus_Event>()
{
  return "audio_msgs::srv::RequestAudioFocus_Event";
}

template<>
inline const char * name<audio_msgs::srv::RequestAudioFocus_Event>()
{
  return "audio_msgs/srv/RequestAudioFocus_Event";
}

template<>
struct has_fixed_size<audio_msgs::srv::RequestAudioFocus_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<audio_msgs::srv::RequestAudioFocus_Event>
  : std::integral_constant<bool, has_bounded_size<audio_msgs::srv::RequestAudioFocus_Request>::value && has_bounded_size<audio_msgs::srv::RequestAudioFocus_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<audio_msgs::srv::RequestAudioFocus_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<audio_msgs::srv::RequestAudioFocus>()
{
  return "audio_msgs::srv::RequestAudioFocus";
}

template<>
inline const char * name<audio_msgs::srv::RequestAudioFocus>()
{
  return "audio_msgs/srv/RequestAudioFocus";
}

template<>
struct has_fixed_size<audio_msgs::srv::RequestAudioFocus>
  : std::integral_constant<
    bool,
    has_fixed_size<audio_msgs::srv::RequestAudioFocus_Request>::value &&
    has_fixed_size<audio_msgs::srv::RequestAudioFocus_Response>::value
  >
{
};

template<>
struct has_bounded_size<audio_msgs::srv::RequestAudioFocus>
  : std::integral_constant<
    bool,
    has_bounded_size<audio_msgs::srv::RequestAudioFocus_Request>::value &&
    has_bounded_size<audio_msgs::srv::RequestAudioFocus_Response>::value
  >
{
};

template<>
struct is_service<audio_msgs::srv::RequestAudioFocus>
  : std::true_type
{
};

template<>
struct is_service_request<audio_msgs::srv::RequestAudioFocus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<audio_msgs::srv::RequestAudioFocus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUDIO_MSGS__SRV__DETAIL__REQUEST_AUDIO_FOCUS__TRAITS_HPP_
