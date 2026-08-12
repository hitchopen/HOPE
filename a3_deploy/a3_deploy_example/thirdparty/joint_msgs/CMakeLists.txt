# ROS2 msg package name
cmake_minimum_required(VERSION 3.8)

project("joint_msgs")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 MSG related dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
include(GSRosidl OPTIONAL)
if(COMMAND gs_disable_rosidl_generator_py_if_requested)
  gs_disable_rosidl_generator_py_if_requested()
endif()

# ROS2 MSG only supports shared libraries, set to build shared library mode
set(CUR_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)

# generate msg
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Command.msg"
  "msg/JointCommand.msg"
  "msg/JointState.msg"
  "msg/State.msg"
  "msg/FootSensor.msg"
  "msg/FootSensorState.msg"
  DEPENDENCIES
  std_msgs)

set(BUILD_SHARED_LIBS ${CUR_BUILD_SHARED_LIBS})

# add namespace to msg target
if(NOT TARGET ${PROJECT_NAME}::${PROJECT_NAME}__rosidl_typesupport_cpp)
  add_library(${PROJECT_NAME}::${PROJECT_NAME}__rosidl_typesupport_cpp ALIAS ${PROJECT_NAME}__rosidl_typesupport_cpp)

endif()

if(NOT TARGET ${PROJECT_NAME}::${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp)
  add_library(${PROJECT_NAME}::${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp ALIAS ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp)
  # target_link_libraries( ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
  # ${std_msgs_LIBRARIES})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
