// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_plugin_proto:srv/RosRpcWrapper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_plugin_proto/srv/ros_rpc_wrapper.hpp"


#ifndef ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__STRUCT_HPP_
#define ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Request __declspec(deprecated)
#endif

namespace ros2_plugin_proto
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RosRpcWrapper_Request_
{
  using Type = RosRpcWrapper_Request_<ContainerAllocator>;

  explicit RosRpcWrapper_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->serialization_type = "";
    }
  }

  explicit RosRpcWrapper_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Request
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Request
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RosRpcWrapper_Request_ & other) const
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
  bool operator!=(const RosRpcWrapper_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RosRpcWrapper_Request_

// alias to use template instance with default allocator
using RosRpcWrapper_Request =
  ros2_plugin_proto::srv::RosRpcWrapper_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_plugin_proto


#ifndef _WIN32
# define DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Response __declspec(deprecated)
#endif

namespace ros2_plugin_proto
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RosRpcWrapper_Response_
{
  using Type = RosRpcWrapper_Response_<ContainerAllocator>;

  explicit RosRpcWrapper_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->code = 0ll;
      this->serialization_type = "";
    }
  }

  explicit RosRpcWrapper_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : serialization_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->code = 0ll;
      this->serialization_type = "";
    }
  }

  // field types and members
  using _code_type =
    int64_t;
  _code_type code;
  using _serialization_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _serialization_type_type serialization_type;
  using _data_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__code(
    const int64_t & _arg)
  {
    this->code = _arg;
    return *this;
  }
  Type & set__serialization_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->serialization_type = _arg;
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
    ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Response
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Response
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RosRpcWrapper_Response_ & other) const
  {
    if (this->code != other.code) {
      return false;
    }
    if (this->serialization_type != other.serialization_type) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RosRpcWrapper_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RosRpcWrapper_Response_

// alias to use template instance with default allocator
using RosRpcWrapper_Response =
  ros2_plugin_proto::srv::RosRpcWrapper_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_plugin_proto


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Event __attribute__((deprecated))
#else
# define DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Event __declspec(deprecated)
#endif

namespace ros2_plugin_proto
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RosRpcWrapper_Event_
{
  using Type = RosRpcWrapper_Event_<ContainerAllocator>;

  explicit RosRpcWrapper_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit RosRpcWrapper_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_plugin_proto::srv::RosRpcWrapper_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_plugin_proto::srv::RosRpcWrapper_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Event
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_plugin_proto__srv__RosRpcWrapper_Event
    std::shared_ptr<ros2_plugin_proto::srv::RosRpcWrapper_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RosRpcWrapper_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const RosRpcWrapper_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RosRpcWrapper_Event_

// alias to use template instance with default allocator
using RosRpcWrapper_Event =
  ros2_plugin_proto::srv::RosRpcWrapper_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_plugin_proto

namespace ros2_plugin_proto
{

namespace srv
{

struct RosRpcWrapper
{
  using Request = ros2_plugin_proto::srv::RosRpcWrapper_Request;
  using Response = ros2_plugin_proto::srv::RosRpcWrapper_Response;
  using Event = ros2_plugin_proto::srv::RosRpcWrapper_Event;
};

}  // namespace srv

}  // namespace ros2_plugin_proto

#endif  // ROS2_PLUGIN_PROTO__SRV__DETAIL__ROS_RPC_WRAPPER__STRUCT_HPP_
