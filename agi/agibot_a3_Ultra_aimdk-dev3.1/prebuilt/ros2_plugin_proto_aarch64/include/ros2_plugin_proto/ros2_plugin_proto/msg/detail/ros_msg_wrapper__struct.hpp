// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_plugin_proto:msg/RosMsgWrapper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_plugin_proto/msg/ros_msg_wrapper.hpp"


#ifndef ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__STRUCT_HPP_
#define ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_plugin_proto__msg__RosMsgWrapper __attribute__((deprecated))
#else
# define DEPRECATED__ros2_plugin_proto__msg__RosMsgWrapper __declspec(deprecated)
#endif

namespace ros2_plugin_proto
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RosMsgWrapper_
{
  using Type = RosMsgWrapper_<ContainerAllocator>;

  explicit RosMsgWrapper_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->serialization_type = "";
    }
  }

  explicit RosMsgWrapper_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : serialization_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->serialization_type = "";
    }
  }

  // field types and members
  using _serialization_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _serialization_type_type serialization_type;
  using _context_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _context_type context;
  using _data_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__serialization_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->serialization_type = _arg;
    return *this;
  }
  Type & set__context(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->context = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_plugin_proto__msg__RosMsgWrapper
    std::shared_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_plugin_proto__msg__RosMsgWrapper
    std::shared_ptr<ros2_plugin_proto::msg::RosMsgWrapper_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RosMsgWrapper_ & other) const
  {
    if (this->serialization_type != other.serialization_type) {
      return false;
    }
    if (this->context != other.context) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RosMsgWrapper_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RosMsgWrapper_

// alias to use template instance with default allocator
using RosMsgWrapper =
  ros2_plugin_proto::msg::RosMsgWrapper_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_plugin_proto

#endif  // ROS2_PLUGIN_PROTO__MSG__DETAIL__ROS_MSG_WRAPPER__STRUCT_HPP_
