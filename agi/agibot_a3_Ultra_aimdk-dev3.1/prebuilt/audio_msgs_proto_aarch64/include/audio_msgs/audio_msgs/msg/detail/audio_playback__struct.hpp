// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from audio_msgs:msg/AudioPlayback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/audio_playback.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__STRUCT_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamps'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'info'
#include "audio_msgs/msg/detail/audio_info__struct.hpp"
// Member 'data'
#include "audio_msgs/msg/detail/audio_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__audio_msgs__msg__AudioPlayback __attribute__((deprecated))
#else
# define DEPRECATED__audio_msgs__msg__AudioPlayback __declspec(deprecated)
#endif

namespace audio_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AudioPlayback_
{
  using Type = AudioPlayback_<ContainerAllocator>;

  explicit AudioPlayback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamps(_init),
    info(_init),
    data(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pkg_name = "";
      this->token_id = "";
    }
  }

  explicit AudioPlayback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamps(_alloc, _init),
    info(_alloc, _init),
    data(_alloc, _init),
    pkg_name(_alloc),
    token_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pkg_name = "";
      this->token_id = "";
    }
  }

  // field types and members
  using _stamps_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamps_type stamps;
  using _info_type =
    audio_msgs::msg::AudioInfo_<ContainerAllocator>;
  _info_type info;
  using _data_type =
    audio_msgs::msg::AudioData_<ContainerAllocator>;
  _data_type data;
  using _pkg_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pkg_name_type pkg_name;
  using _token_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _token_id_type token_id;

  // setters for named parameter idiom
  Type & set__stamps(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamps = _arg;
    return *this;
  }
  Type & set__info(
    const audio_msgs::msg::AudioInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__data(
    const audio_msgs::msg::AudioData_<ContainerAllocator> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__pkg_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pkg_name = _arg;
    return *this;
  }
  Type & set__token_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->token_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    audio_msgs::msg::AudioPlayback_<ContainerAllocator> *;
  using ConstRawPtr =
    const audio_msgs::msg::AudioPlayback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::AudioPlayback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::AudioPlayback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__audio_msgs__msg__AudioPlayback
    std::shared_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__audio_msgs__msg__AudioPlayback
    std::shared_ptr<audio_msgs::msg::AudioPlayback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AudioPlayback_ & other) const
  {
    if (this->stamps != other.stamps) {
      return false;
    }
    if (this->info != other.info) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->pkg_name != other.pkg_name) {
      return false;
    }
    if (this->token_id != other.token_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const AudioPlayback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AudioPlayback_

// alias to use template instance with default allocator
using AudioPlayback =
  audio_msgs::msg::AudioPlayback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_PLAYBACK__STRUCT_HPP_
