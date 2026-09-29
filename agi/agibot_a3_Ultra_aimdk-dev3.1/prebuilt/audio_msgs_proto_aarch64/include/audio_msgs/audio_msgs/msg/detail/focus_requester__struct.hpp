// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from audio_msgs:msg/FocusRequester.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/focus_requester.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__FOCUS_REQUESTER__STRUCT_HPP_
#define AUDIO_MSGS__MSG__DETAIL__FOCUS_REQUESTER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__audio_msgs__msg__FocusRequester __attribute__((deprecated))
#else
# define DEPRECATED__audio_msgs__msg__FocusRequester __declspec(deprecated)
#endif

namespace audio_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FocusRequester_
{
  using Type = FocusRequester_<ContainerAllocator>;

  explicit FocusRequester_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pkg_name = "";
      this->priority = 0ul;
      this->priority_weight = 0ul;
    }
  }

  explicit FocusRequester_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pkg_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pkg_name = "";
      this->priority = 0ul;
      this->priority_weight = 0ul;
    }
  }

  // field types and members
  using _pkg_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pkg_name_type pkg_name;
  using _priority_type =
    uint32_t;
  _priority_type priority;
  using _priority_weight_type =
    uint32_t;
  _priority_weight_type priority_weight;

  // setters for named parameter idiom
  Type & set__pkg_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pkg_name = _arg;
    return *this;
  }
  Type & set__priority(
    const uint32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__priority_weight(
    const uint32_t & _arg)
  {
    this->priority_weight = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    audio_msgs::msg::FocusRequester_<ContainerAllocator> *;
  using ConstRawPtr =
    const audio_msgs::msg::FocusRequester_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::FocusRequester_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::FocusRequester_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__audio_msgs__msg__FocusRequester
    std::shared_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__audio_msgs__msg__FocusRequester
    std::shared_ptr<audio_msgs::msg::FocusRequester_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FocusRequester_ & other) const
  {
    if (this->pkg_name != other.pkg_name) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    if (this->priority_weight != other.priority_weight) {
      return false;
    }
    return true;
  }
  bool operator!=(const FocusRequester_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FocusRequester_

// alias to use template instance with default allocator
using FocusRequester =
  audio_msgs::msg::FocusRequester_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__FOCUS_REQUESTER__STRUCT_HPP_
