cmake_minimum_required(VERSION 3.8)
project(hope_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallObservationV2.msg"
  "msg/FlightContextV2.msg"
  "msg/FlightEventV2.msg"
  "msg/FlightWaypointV2.msg"
  "msg/FlightPredictionV2.msg"
  "msg/StrikeCandidateV1.msg"
  "msg/FlightPacketV2.msg"

  "msg/BallFlightSample.msg"
  "msg/BallFlightPacket.msg"
  "msg/RacketCommand.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
