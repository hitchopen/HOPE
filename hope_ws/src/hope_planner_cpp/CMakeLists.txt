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
  src/common_ball_physics.cpp
  src/batch_physics_estimator.cpp
  src/spin_estimator.cpp
  src/trajectory_predictor.cpp
  src/racket_target_planner.cpp
  src/reach_permission.cpp
  src/schema2_packer.cpp
  src/schema3_packer.cpp
  src/audit_logger.cpp
  src/post_net_one_shot.cpp
  src/incoming_trajectory.cpp
  src/adaptive_flight_commit.cpp
  src/flight_packet.cpp
  src/question_bank_fixture.cpp
)
find_path(HOPE_RECORDING_INCLUDE_DIR a3_deploy/recording_file.hpp
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include"
  REQUIRED)
target_include_directories(hope_planner_core PRIVATE "${HOPE_RECORDING_INCLUDE_DIR}")
target_include_directories(hope_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(hope_planner_core PUBLIC
  Eigen3::Eigen
  ${hope_msgs_TARGETS}
)

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

add_executable(hope_ball_flight_packetizer
  src/flight_packetizer_node.cpp
  src/flight_packetizer_main.cpp
)
target_include_directories(hope_ball_flight_packetizer PRIVATE include)
target_link_libraries(hope_ball_flight_packetizer PRIVATE
  hope_planner_core
  Eigen3::Eigen
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${hope_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS
  hope_planner_core
  hope_planner_cpp_node
  hope_planner_cpp_replay
  hope_ball_flight_packetizer
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
  ament_add_gtest(test_spin_physics test/test_spin_physics.cpp)
  target_link_libraries(test_spin_physics hope_planner_core Eigen3::Eigen)
  ament_add_gtest(test_future_contact_predictor test/test_future_contact_predictor.cpp)
  target_link_libraries(test_future_contact_predictor hope_planner_core Eigen3::Eigen)
  ament_add_gtest(test_batch_physics_estimator test/test_batch_physics_estimator.cpp)
  target_link_libraries(test_batch_physics_estimator hope_planner_core Eigen3::Eigen)
  target_include_directories(test_batch_physics_estimator PRIVATE include)

  ament_add_gtest(test_trajectory_and_stage3 test/test_trajectory_and_stage3.cpp)
  target_link_libraries(test_trajectory_and_stage3 hope_planner_core Eigen3::Eigen)
  target_include_directories(test_trajectory_and_stage3 PRIVATE include)

  ament_add_gtest(test_target_turn_guard test/test_target_turn_guard.cpp)
  target_link_libraries(test_target_turn_guard hope_planner_core Eigen3::Eigen)
  target_include_directories(test_target_turn_guard PRIVATE include)

  ament_add_gtest(test_prediction_event_order test/test_prediction_event_order.cpp)
  target_link_libraries(test_prediction_event_order hope_planner_core Eigen3::Eigen)
  target_include_directories(test_prediction_event_order PRIVATE include)

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

  ament_add_gtest(test_adaptive_flight_commit test/test_adaptive_flight_commit.cpp)
  target_link_libraries(test_adaptive_flight_commit hope_planner_core Eigen3::Eigen)
  target_include_directories(test_adaptive_flight_commit PRIVATE include)

  ament_add_gtest(test_flight_packet test/test_flight_packet.cpp)
  target_link_libraries(test_flight_packet hope_planner_core Eigen3::Eigen)
  target_include_directories(test_flight_packet PRIVATE include)

  ament_add_gtest(test_question_bank_fixture test/test_question_bank_fixture.cpp)
  target_link_libraries(test_question_bank_fixture hope_planner_core Eigen3::Eigen)
  target_include_directories(test_question_bank_fixture PRIVATE include)

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

  ament_add_gtest(test_planner_node_turn_guard
    test/test_planner_node_turn_guard.cpp
    src/planner_node.cpp
  )
  target_include_directories(test_planner_node_turn_guard PRIVATE include)
  target_link_libraries(test_planner_node_turn_guard
    hope_planner_core
    Eigen3::Eigen
    rclcpp::rclcpp
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${hope_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
  )

  ament_add_gtest(test_flight_packet_pipeline
    test/test_flight_packet_pipeline.cpp
    src/planner_node.cpp
    src/flight_packetizer_node.cpp
  )
  target_include_directories(test_flight_packet_pipeline PRIVATE include)
  target_link_libraries(test_flight_packet_pipeline
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


# Independent, opt-in-at-launch shadow pipeline. Public planner links remain unchanged.
add_library(hope_forecast_core
  src/probabilistic_physics.cpp src/probabilistic_ball_tracker.cpp
  src/nominal_event_forecast.cpp src/fixed_lag_types.cpp src/return_region.cpp
  src/ball_flight_state_machine.cpp src/hybrid_trajectory_coordinator.cpp
  src/hit_plane_manager.cpp src/flight_state_v2.cpp)
target_include_directories(hope_forecast_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(hope_forecast_core PUBLIC hope_planner_core Eigen3::Eigen)
add_library(hope_strike_selector_core src/strike_flight_detector.cpp src/strike_target_selector.cpp)
target_link_libraries(hope_strike_selector_core PUBLIC hope_forecast_core ${hope_msgs_TARGETS})
add_executable(hope_ball_event_forecaster_node src/hybrid_trajectory_node.cpp src/hybrid_trajectory_main.cpp)
target_link_libraries(hope_ball_event_forecaster_node PRIVATE hope_forecast_core rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS})
add_executable(hope_strike_target_selector_node src/strike_target_selector_node.cpp)
target_link_libraries(hope_strike_target_selector_node PRIVATE hope_strike_selector_core
  rclcpp::rclcpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})
install(TARGETS hope_forecast_core hope_strike_selector_core hope_ball_event_forecaster_node
  hope_strike_target_selector_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_strike_flight_detector test/test_strike_flight_detector.cpp)
  target_link_libraries(test_strike_flight_detector hope_strike_selector_core)
  ament_add_gtest(test_strike_target_selector test/test_strike_target_selector.cpp)
  target_link_libraries(test_strike_target_selector hope_strike_selector_core)
  add_executable(test_shadow_runner_input test/test_shadow_runner_input.cpp)
  target_compile_features(test_shadow_runner_input PRIVATE cxx_std_20)
  target_include_directories(test_shadow_runner_input PRIVATE "${HOPE_RECORDING_INCLUDE_DIR}")
  target_link_libraries(test_shadow_runner_input PRIVATE hope_strike_selector_core)
  add_test(NAME test_shadow_runner_input COMMAND test_shadow_runner_input)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ckf_strike_shadow_ros COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/ckf_strike_shadow_smoke.py
    --forecaster $<TARGET_FILE:hope_ball_event_forecaster_node>
    --selector $<TARGET_FILE:hope_strike_target_selector_node>
    --parser $<TARGET_FILE:test_shadow_runner_input>)
  set_tests_properties(ckf_strike_shadow_ros PROPERTIES TIMEOUT 60)
endif()
install(FILES docs/ckf_strike_shadow.md DESTINATION share/${PROJECT_NAME}/docs)

ament_export_include_directories(include)
ament_export_libraries(hope_planner_core)
ament_export_dependencies(Eigen3 hope_msgs)
if(BUILD_TESTING)
  ament_add_gtest(test_flight_prediction_v2 test/test_flight_prediction_v2.cpp)
  target_link_libraries(test_flight_prediction_v2 hope_forecast_core Eigen3::Eigen)
endif()
ament_package()
