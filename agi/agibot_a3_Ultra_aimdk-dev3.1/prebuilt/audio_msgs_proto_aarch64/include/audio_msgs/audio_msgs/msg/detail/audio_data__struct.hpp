// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from audio_msgs:msg/AudioData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/msg/audio_data.hpp"


#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__audio_msgs__msg__AudioData __attribute__((deprecated))
#else
# define DEPRECATED__audio_msgs__msg__AudioData __declspec(deprecated)
#endif

namespace audio_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AudioData_
{
  using Type = AudioData_<ContainerAllocator>;

  explicit AudioData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit AudioData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    audio_msgs::msg::AudioData_<ContainerAllocator> *;
  using ConstRawPtr =
    const audio_msgs::msg::AudioData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<audio_msgs::msg::AudioData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<audio_msgs::msg::AudioData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::AudioData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::AudioData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::AudioData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::AudioData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<audio_msgs::msg::AudioData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<audio_msgs::msg::AudioData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__audio_msgs__msg__AudioData
    std::shared_ptr<audio_msgs::msg::AudioData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__audio_msgs__msg__AudioData
    std::shared_ptr<audio_msgs::msg::AudioData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AudioData_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const AudioData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AudioData_

// alias to use template instance with default allocator
using AudioData =
  audio_msgs::msg::AudioData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_
