cmake_minimum_required(VERSION 3.16)
project(hope_planner_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hope_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(hope_planner_core
  src/batch_physics_estimator.cpp
  src/spin_estimator.cpp
  src/trajectory_predictor.cpp
  src/racket_target_planner.cpp
  src/schema2_packer.cpp
  src/audit_logger.cpp
  src/post_net_one_shot.cpp
  src/incoming_trajectory.cpp
)
target_include_directories(hope_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(hope_planner_core PUBLIC Eigen3::Eigen)

add_executable(hope_planner_cpp_node
  src/planner_node.cpp
  src/main.cpp
)
target_include_directories(hope_planner_cpp_node PRIVATE include)
target_link_libraries(hope_planner_cpp_node PRIVATE
  hope_planner_core
  Eigen3::Eigen
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${hope_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

add_executable(hope_planner_cpp_replay src/replay_main.cpp)
target_include_directories(hope_planner_cpp_replay PRIVATE include)
target_link_libraries(hope_planner_cpp_replay PRIVATE hope_planner_core Eigen3::Eigen)

install(TARGETS
  hope_planner_core
  hope_planner_cpp_node
  hope_planner_cpp_replay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/audit_bounce_transition.py
  scripts/audit_model21800_replay.py
  scripts/audit_spin_observability.py
  scripts/compare_cpp_python_planner.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_batch_physics_estimator test/test_batch_physics_estimator.cpp)
  target_link_libraries(test_batch_physics_estimator hope_planner_core Eigen3::Eigen)
  target_include_directories(test_batch_physics_estimator PRIVATE include)

  ament_add_gtest(test_trajectory_and_stage3 test/test_trajectory_and_stage3.cpp)
  target_link_libraries(test_trajectory_and_stage3 hope_planner_core Eigen3::Eigen)
  target_include_directories(test_trajectory_and_stage3 PRIVATE include)

  ament_add_gtest(test_schema2 test/test_schema2.cpp)
  target_link_libraries(test_schema2 hope_planner_core Eigen3::Eigen)
  target_include_directories(test_schema2 PRIVATE include)

  ament_add_gtest(test_spin_estimator test/test_spin_estimator.cpp)
  target_link_libraries(test_spin_estimator hope_planner_core Eigen3::Eigen)
  target_include_directories(test_spin_estimator PRIVATE include)

  ament_add_gtest(test_audit_logger test/test_audit_logger.cpp)
  target_link_libraries(test_audit_logger hope_planner_core Eigen3::Eigen)
  target_include_directories(test_audit_logger PRIVATE include)

  ament_add_gtest(test_post_net_one_shot test/test_post_net_one_shot.cpp)
  target_link_libraries(test_post_net_one_shot hope_planner_core Eigen3::Eigen)
  target_include_directories(test_post_net_one_shot PRIVATE include)

  ament_add_gtest(test_incoming_trajectory test/test_incoming_trajectory.cpp)
  target_link_libraries(test_incoming_trajectory hope_planner_core Eigen3::Eigen)
  target_include_directories(test_incoming_trajectory PRIVATE include)

  ament_add_gtest(test_planner_node_one_shot
    test/test_planner_node_one_shot.cpp
    src/planner_node.cpp
  )
  target_include_directories(test_planner_node_one_shot PRIVATE include)
  target_link_libraries(test_planner_node_one_shot
    hope_planner_core
    Eigen3::Eigen
    rclcpp::rclcpp
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${hope_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(hope_planner_core)
ament_export_dependencies(Eigen3)
ament_package()
