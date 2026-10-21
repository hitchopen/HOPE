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

# Application integration is intentionally outside the forward-only graph.
add_library(hope_integrated_return
  src/return_eligibility.cpp src/validated_return_planner.cpp)
target_include_directories(hope_integrated_return PUBLIC include)
target_link_libraries(hope_integrated_return PUBLIC hope_v2_forecast_core)
add_library(hope_integrated_runtime src/integrated_planner_runtime.cpp)
target_include_directories(hope_integrated_runtime PUBLIC include)
target_link_libraries(hope_integrated_runtime PUBLIC hope_integrated_return
  hope_v2_transport hope_planner_core rclcpp::rclcpp ${std_msgs_TARGETS}
  ${std_srvs_TARGETS} ${diagnostic_msgs_TARGETS})
target_link_libraries(hope_planner_cpp_node PRIVATE hope_integrated_runtime)
install(TARGETS hope_integrated_return hope_integrated_runtime
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
add_executable(hope_integrated_runner_probe test/integrated_runner_probe.cpp)
set_target_properties(hope_integrated_runner_probe PROPERTIES CXX_STANDARD 20)
target_include_directories(hope_integrated_runner_probe PRIVATE "${HOPE_RECORDING_INCLUDE_DIR}")
target_link_libraries(hope_integrated_runner_probe PRIVATE Eigen3::Eigen rclcpp::rclcpp ${std_msgs_TARGETS})
install(TARGETS hope_integrated_runner_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
add_executable(hope_integrated_return_sweep test/integrated_return_sweep.cpp)
target_link_libraries(hope_integrated_return_sweep PRIVATE hope_integrated_return)
install(TARGETS hope_integrated_return_sweep RUNTIME DESTINATION lib/${PROJECT_NAME})

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
  scripts/integrated_preflight.py
  scripts/audit_bounce_transition.py
  scripts/audit_model21800_replay.py
  scripts/audit_spin_observability.py
  scripts/compare_cpp_python_planner.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_integrated_return test/test_integrated_return.cpp)
  ament_add_gtest(test_v2_pr64_review_regressions test/test_v2_pr64_review_regressions.cpp)
  target_link_libraries(test_v2_pr64_review_regressions hope_integrated_return hope_planner_core)
  set_target_properties(test_integrated_return PROPERTIES CXX_STANDARD 20)
  target_include_directories(test_integrated_return PRIVATE "${HOPE_RECORDING_INCLUDE_DIR}")
  target_link_libraries(test_integrated_return hope_integrated_return hope_planner_core)
  add_test(NAME integrated_outgoing_reference COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/flight_prediction_validation/integrated_numerical_gate.py
    --executable $<TARGET_FILE:hope_integrated_return_sweep>
    --output ${CMAKE_CURRENT_BINARY_DIR}/integrated_outgoing_reference.json)
  foreach(rate 150 300)
    foreach(policy FOLLOW_POSE ON_REFRESH)
      add_test(NAME integrated_chain_${rate}_${policy} COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_producer_chain.py
        --planner $<TARGET_FILE:hope_planner_cpp_node>
        --decoder $<TARGET_FILE:hope_integrated_runner_probe>
        --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
        --rate ${rate} --plane-policy ${policy}
        --output ${CMAKE_CURRENT_BINARY_DIR}/integrated_chain_${rate}_${policy}.json)
      set_tests_properties(integrated_chain_${rate}_${policy} PROPERTIES
        RUN_SERIAL TRUE TIMEOUT 60 ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
    endforeach()
  endforeach()
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
  target_link_libraries(test_planner_node_one_shot hope_integrated_runtime)
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
  target_link_libraries(test_planner_node_turn_guard hope_integrated_runtime)
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
  target_link_libraries(test_flight_packet_pipeline hope_integrated_runtime)
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

ament_export_include_directories(include)
ament_export_libraries(hope_planner_core)
ament_export_dependencies(Eigen3 hope_msgs)

# V2's forward-only graph is separate from every legacy production object.
add_library(hope_v2_forward_physics
  src/common_ball_physics.cpp src/flight_spin_estimator.cpp)
target_include_directories(hope_v2_forward_physics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(hope_v2_forward_physics PUBLIC Eigen3::Eigen)
add_library(hope_v2_forecast_core
  src/probabilistic_physics.cpp src/probabilistic_ball_tracker.cpp
  src/nominal_event_forecast.cpp src/fixed_lag_types.cpp src/return_region.cpp
  src/hit_plane_manager.cpp src/flight_state_v2.cpp)
target_link_libraries(hope_v2_forecast_core PUBLIC hope_v2_forward_physics)
add_library(hope_v2_transport src/flight_transport_v2.cpp)
target_link_libraries(hope_v2_transport PUBLIC hope_v2_forecast_core
  rclcpp::rclcpp ${hope_msgs_TARGETS})
add_executable(hope_flight_prediction_v2_node src/flight_prediction_v2_node.cpp)
target_link_libraries(hope_flight_prediction_v2_node PRIVATE hope_v2_transport ${std_msgs_TARGETS})
add_executable(hope_flight_prediction_v2_replay src/flight_prediction_v2_replay.cpp)
target_link_libraries(hope_flight_prediction_v2_replay PRIVATE hope_v2_forecast_core)
# Evaluation exporter links only the original Stage 1/2 objects, never Stage 3.
add_library(hope_legacy_flight_evaluation STATIC
  src/batch_physics_estimator.cpp src/spin_estimator.cpp
  src/trajectory_predictor.cpp src/incoming_trajectory.cpp src/adaptive_flight_commit.cpp)
target_include_directories(hope_legacy_flight_evaluation PUBLIC include)
target_link_libraries(hope_legacy_flight_evaluation PUBLIC Eigen3::Eigen)
add_executable(hope_ball_flight_baseline_replay src/ball_flight_baseline_replay.cpp)
target_link_libraries(hope_ball_flight_baseline_replay PRIVATE
  hope_legacy_flight_evaluation hope_v2_forecast_core)
install(TARGETS hope_v2_forward_physics hope_v2_forecast_core hope_v2_transport
  EXPORT hope_v2_targets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS hope_flight_prediction_v2_node hope_flight_prediction_v2_replay
  hope_ball_flight_baseline_replay RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_targets(hope_v2_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
install(DIRECTORY scripts/flight_prediction_validation DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_pytest REQUIRED)
  add_test(NAME integrated_context_noop COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_context_noop.py
    --executable $<TARGET_FILE:hope_flight_prediction_v2_node>)
  set_tests_properties(integrated_context_noop PROPERTIES RUN_SERIAL TRUE TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=195;ROS_LOCALHOST_ONLY=1")
  add_test(NAME flight_v2_installed_api COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/installed_forward_api.py --prefix ${CMAKE_INSTALL_PREFIX})
  ament_add_gtest(test_flight_prediction_v2 test/test_flight_prediction_v2.cpp)
  target_link_libraries(test_flight_prediction_v2 hope_v2_forecast_core)
  ament_add_gtest(test_flight_transport_v2 test/test_flight_transport_v2.cpp)
  target_link_libraries(test_flight_transport_v2 hope_v2_transport)
  foreach(pair "numerical;numerical_gate.py" "causal_replay;causal_replay_gate.py")
    list(GET pair 0 label)
    list(GET pair 1 script)
    add_test(NAME flight_v2_${label} COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/flight_prediction_validation/${script}
      --executable $<TARGET_FILE:hope_flight_prediction_v2_replay>)
  endforeach()
  foreach(pair "scoring;test_flight_scoring.py" "accuracy_tooling;test_scoped_accuracy.py" "scope;test_forecast_scope.py")
    list(GET pair 0 label)
    list(GET pair 1 script)
    ament_add_pytest_test(flight_v2_${label} test/${script})
  endforeach()
  foreach(pair "ros_transport;flight_prediction_v2_smoke.py" "raw_scenarios;flight_prediction_v2_scenarios.py" "binary_scope;forecast_binary_scope.py")
    list(GET pair 0 label)
    list(GET pair 1 script)
    add_test(NAME flight_v2_${label} COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/${script}
      --executable $<TARGET_FILE:hope_flight_prediction_v2_node>)
    set_tests_properties(flight_v2_${label} PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
  endforeach()
  foreach(pair "matched_replay_tooling;matched_replay_gate.py" "baseline_epochs;baseline_replay_gate.py")
    list(GET pair 0 label)
    list(GET pair 1 script)
    add_test(NAME flight_v2_${label} COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/${script}
      --baseline $<TARGET_FILE:hope_ball_flight_baseline_replay>
      --v2 $<TARGET_FILE:hope_flight_prediction_v2_replay>)
  endforeach()
endif()

install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs
  FILES_MATCHING PATTERN "flight_prediction*.md" PATTERN "flight_prediction*.json"
  PATTERN "integrated_planner*.md" PATTERN "integrated_planner*.json"
  PATTERN "flight_prediction_accuracy" PATTERN "NOT_RUN.json" PATTERN "README.md")
ament_package()
