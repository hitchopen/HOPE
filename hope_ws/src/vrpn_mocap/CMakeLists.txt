cmake_minimum_required(VERSION 3.5)
project(vrpn_mocap)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(VRPN REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(vrpn_mocap src/client.cpp src/tracker.cpp)
target_include_directories(vrpn_mocap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vrpn_mocap rclcpp VRPN std_msgs geometry_msgs Eigen3)

install(TARGETS vrpn_mocap
        DESTINATION lib/${PROJECT_NAME})

add_executable(client_node src/client_node.cpp)
target_include_directories(client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(client_node vrpn_mocap)

install(TARGETS client_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # add rclcpp include dir to ament_cppcheck
  get_target_property(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS
    rclcpp::rclcpp INTERFACE_INCLUDE_DIRECTORIES)
  ament_lint_auto_find_test_dependencies()
endif()

# install launch directory
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

#install config directory
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_package()
