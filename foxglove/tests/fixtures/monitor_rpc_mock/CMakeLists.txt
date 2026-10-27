# Isolated-test interface only. Never install this package on a robot.
cmake_minimum_required(VERSION 3.16)
project(ros2_plugin_proto LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/RosRpcWrapper.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
