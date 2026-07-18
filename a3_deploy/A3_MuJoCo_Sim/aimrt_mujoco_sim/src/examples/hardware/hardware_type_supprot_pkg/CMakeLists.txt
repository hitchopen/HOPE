# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# Add target
add_library(${CUR_TARGET_NAME} SHARED)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set link libraries of target
set(BASE_DEPENDENCIES
    aimrt::interface::aimrt_type_support_pkg_c_interface #
    aimrt::interface::aimrt_module_protobuf_interface #
    aimrt::protocols::sensor_pb_gencode# 
    aimrt::protocols::actuator_pb_gencode)

if(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  target_compile_definitions(${CUR_TARGET_NAME} PRIVATE AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)

  target_link_libraries(
    ${CUR_TARGET_NAME}
    PRIVATE ${BASE_DEPENDENCIES} #
            aimrt::interface::aimrt_module_ros2_interface #
            sensor_msgs::sensor_msgs__rosidl_generator_cpp #
            sensor_msgs::sensor_msgs__rosidl_typesupport_cpp #
            aimrt_msgs::aimrt_msgs__rosidl_generator_cpp #
            aimrt_msgs::aimrt_msgs__rosidl_typesupport_cpp #
            aimrt_msgs::aimrt_msgs__rosidl_typesupport_fastrtps_cpp #
            aimrt_msgs::aimrt_msgs__rosidl_typesupport_introspection_cpp)
else()
  target_link_libraries(${CUR_TARGET_NAME} PRIVATE ${BASE_DEPENDENCIES})
endif()

# Set misc of target
set_target_properties(${CUR_TARGET_NAME} PROPERTIES OUTPUT_NAME ${CUR_DIR})
