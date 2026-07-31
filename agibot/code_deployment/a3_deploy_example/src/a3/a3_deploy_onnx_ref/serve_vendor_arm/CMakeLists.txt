cmake_minimum_required(VERSION 3.16)
project(a3_serve_vendor_arm LANGUAGES CXX)

find_package(Threads REQUIRED)
find_package(rclcpp CONFIG REQUIRED)
find_package(sensor_msgs CONFIG REQUIRED)

set(
  SERVE_VENDOR_ARM_RUNTIME_OUTPUT_DIR
  "${CMAKE_BINARY_DIR}"
  CACHE PATH
  "Output directory for the fixed vendor-arm serve runner"
)

add_executable(
  a3_serve_vendor_arm_runner
  ../src/a3_deploy/a3_serve_vendor_arm_main.cpp
)
target_link_libraries(
  a3_serve_vendor_arm_runner
  PRIVATE
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    Threads::Threads
)
set_target_properties(
  a3_serve_vendor_arm_runner
  PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY "${SERVE_VENDOR_ARM_RUNTIME_OUTPUT_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
    INSTALL_RPATH "$ORIGIN;/opt/ros/jazzy/lib;/opt/ros/humble/lib"
)
