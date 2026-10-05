// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from audio_msgs:msg/AudioInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/audio_info.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_INFO__STRUCT_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__audio_msgs__msg__AudioInfo __attribute__((deprecated))
#else
# define DEPRECATED__audio_msgs__msg__AudioInfo __declspec(deprecated)
#endif

namespace audio_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AudioInfo_
{
  using Type = AudioInfo_<ContainerAllocator>;

  explicit AudioInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channels = 0;
      this->sample_rate = 0ul;
      this->sample_format = "";
      this->coding_format = "";
    }
  }

  explicit AudioInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sample_format(_alloc),
    coding_format(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channels = 0;
      this->sample_rate = 0ul;
      this->sample_format = "";
      this->coding_format = "";
    }
  }

  // field types and members
  using _channels_type =
    uint8_t;
  _channels_type channels;
  using _sample_rate_type =
    uint32_t;
  _sample_rate_type sample_rate;
  using _sample_format_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sample_format_type sample_format;
  using _coding_format_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _coding_format_type coding_format;

  // setters for named parameter idiom
  Type & set__channels(
    const uint8_t & _arg)
  {
    this->channels = _arg;
    return *this;
  }
  Type & set__sample_rate(
    const uint32_t & _arg)
  {
    this->sample_rate = _arg;
    return *this;
  }
  Type & set__sample_format(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sample_format = _arg;
    return *this;
  }
  Type & set__coding_format(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->coding_format = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    audio_msgs::msg::AudioInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const audio_msgs::msg::AudioInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::AudioInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::AudioInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__audio_msgs__msg__AudioInfo
    std::shared_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__audio_msgs__msg__AudioInfo
    std::shared_ptr<audio_msgs::msg::AudioInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AudioInfo_ & other) const
  {
    if (this->channels != other.channels) {
      return false;
    }
    if (this->sample_rate != other.sample_rate) {
      return false;
    }
    if (this->sample_format != other.sample_format) {
      return false;
    }
    if (this->coding_format != other.coding_format) {
      return false;
    }
    return true;
  }
  bool operator!=(const AudioInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AudioInfo_

// alias to use template instance with default allocator
using AudioInfo =
  audio_msgs::msg::AudioInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_INFO__STRUCT_HPP_
