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
find_package(rcl_interfaces REQUIRED)
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
target_include_directories(hope_planner_core PUBLIC "$<BUILD_INTERFACE:${HOPE_RECORDING_INCLUDE_DIR}>")
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
    ${rcl_interfaces_TARGETS}
)

# Application integration is intentionally outside the forward-only graph.
add_library(hope_integrated_return
  src/return_eligibility.cpp src/validated_return_planner.cpp)
target_include_directories(hope_integrated_return PUBLIC include)
target_link_libraries(hope_integrated_return PUBLIC hope_v2_forecast_core)
add_library(hope_v2_incoming_core src/v2_strike_adapter.cpp src/v2_incoming_coordinator.cpp)
target_include_directories(hope_v2_incoming_core PUBLIC include)
target_link_libraries(hope_v2_incoming_core PUBLIC hope_integrated_return hope_planner_core)
install(TARGETS hope_v2_incoming_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
add_library(hope_integrated_runtime src/integrated_planner_runtime.cpp src/v2_incoming_runtime.cpp)
target_include_directories(hope_integrated_runtime PUBLIC include)
target_link_libraries(hope_integrated_runtime PUBLIC hope_integrated_return
  hope_v2_transport hope_planner_core rclcpp::rclcpp ${std_msgs_TARGETS}
  ${std_srvs_TARGETS} ${diagnostic_msgs_TARGETS})
target_link_libraries(hope_integrated_runtime PUBLIC hope_v2_incoming_core)
target_link_libraries(hope_planner_cpp_node PRIVATE hope_integrated_runtime)
install(TARGETS hope_integrated_return hope_integrated_runtime
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
add_executable(hope_integrated_runner_probe test/integrated_runner_probe.cpp)
set_target_properties(hope_integrated_runner_probe PROPERTIES CXX_STANDARD 20)
target_include_directories(hope_integrated_runner_probe PRIVATE include "${HOPE_RECORDING_INCLUDE_DIR}")
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
install(FILES "${HOPE_RECORDING_INCLUDE_DIR}/a3_deploy/session_home_contract.hpp" DESTINATION include/a3_deploy)
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
  foreach(rate 150 300 360)
    foreach(mode fixed per_ball stable_per_ball)
      foreach(spin unknown zero)
        set(profile incoming_chain_${rate}_${mode}_${spin})
        add_test(NAME ${profile} COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/v2_incoming_producer_chain.py
          --repo ${CMAKE_CURRENT_SOURCE_DIR}/../../..
          --build ${CMAKE_CURRENT_BINARY_DIR}
          --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
          --rate ${rate} --mode ${mode} --spin ${spin}
          --output ${CMAKE_CURRENT_BINARY_DIR}/${profile})
        set_tests_properties(${profile} PROPERTIES RUN_SERIAL TRUE TIMEOUT 60
          ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
      endforeach()
    endforeach()
  endforeach()
  ament_add_gtest(test_reacquisition_counterexamples test/reacquisition_counterexamples.cpp)
  target_compile_definitions(test_reacquisition_counterexamples PRIVATE HOPE_REARM_CONTINUATION=1)
  target_link_libraries(test_reacquisition_counterexamples hope_integrated_return)
  ament_add_gtest(test_reacquisition_spin test/test_reacquisition_spin.cpp)
  target_link_libraries(test_reacquisition_spin hope_integrated_return hope_planner_core)
  ament_add_gtest(test_base_pose_contract test/test_base_pose_contract.cpp)
  set_target_properties(test_base_pose_contract PROPERTIES CXX_STANDARD 20)
  target_include_directories(test_base_pose_contract PRIVATE "${HOPE_RECORDING_INCLUDE_DIR}")
  target_link_libraries(test_base_pose_contract hope_integrated_return hope_planner_core)
  add_executable(hope_base_pose_contract_probe test/base_pose_contract_probe.cpp)
  target_link_libraries(hope_base_pose_contract_probe hope_v2_forecast_core)
  add_test(NAME base_pose_producer_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/base_pose_producer_contract.py
    --probe $<TARGET_FILE:hope_base_pose_contract_probe>
    --output ${CMAKE_CURRENT_BINARY_DIR}/base_pose_producer_contract.json)
  foreach(rate 150 300)
    foreach(policy FOLLOW_POSE ON_REFRESH)
      foreach(spin unknown quaternion_only zero)
        add_test(NAME reacquisition_chain_${rate}_${policy}_${spin} COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/reacquisition_producer_chain.py
          --planner $<TARGET_FILE:hope_planner_cpp_node>
          --decoder $<TARGET_FILE:hope_integrated_runner_probe>
          --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
          --rate ${rate} --policy ${policy} --spin ${spin}
          --output ${CMAKE_CURRENT_BINARY_DIR}/reacquisition_chain_${rate}_${policy}_${spin}.json)
        set_tests_properties(reacquisition_chain_${rate}_${policy}_${spin} PROPERTIES RUN_SERIAL TRUE TIMEOUT 75
          ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
      endforeach()
    endforeach()
    add_test(NAME reacquisition_delayed_${rate} COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/reacquisition_producer_chain.py
      --planner $<TARGET_FILE:hope_planner_cpp_node> --decoder $<TARGET_FILE:hope_integrated_runner_probe>
      --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
      --rate ${rate} --policy FOLLOW_POSE --spin unknown --first-worker-delay-ms 750
      --output ${CMAKE_CURRENT_BINARY_DIR}/reacquisition_delayed_${rate}.json)
    set_tests_properties(reacquisition_delayed_${rate} PROPERTIES RUN_SERIAL TRUE TIMEOUT 75
      ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
    foreach(yaw 0 30 -30 50)
      add_test(NAME base_yaw_chain_${rate}_${yaw} COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_producer_chain.py
        --planner $<TARGET_FILE:hope_planner_cpp_node> --decoder $<TARGET_FILE:hope_integrated_runner_probe>
        --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
        --rate ${rate} --base-yaw ${yaw}
        --output ${CMAKE_CURRENT_BINARY_DIR}/base_yaw_chain_${rate}_${yaw}.json)
      set_tests_properties(base_yaw_chain_${rate}_${yaw} PROPERTIES RUN_SERIAL TRUE TIMEOUT 60
        ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
    endforeach()
    foreach(spec 10_unknown 12_known)
      string(REPLACE "_" ";" parts ${spec})
      list(GET parts 0 speed)
      list(GET parts 1 spin)
      add_test(NAME review_refresh_${rate}_${spec} COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_producer_chain.py
        --planner $<TARGET_FILE:hope_planner_cpp_node> --decoder $<TARGET_FILE:hope_integrated_runner_probe>
        --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
        --rate ${rate} --plane-policy ON_REFRESH --speed ${speed} --spin ${spin}
        --output ${CMAKE_CURRENT_BINARY_DIR}/review_refresh_${rate}_${spec}.json)
      set_tests_properties(review_refresh_${rate}_${spec} PROPERTIES RUN_SERIAL TRUE TIMEOUT 60
        ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
    endforeach()
    add_test(NAME review_worker_expiry_${rate} COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_producer_chain.py
      --planner $<TARGET_FILE:hope_planner_cpp_node> --decoder $<TARGET_FILE:hope_integrated_runner_probe>
      --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
      --rate ${rate} --worker-delay-ms 40
      --output ${CMAKE_CURRENT_BINARY_DIR}/review_worker_expiry_${rate}.json)
    set_tests_properties(review_worker_expiry_${rate} PROPERTIES RUN_SERIAL TRUE TIMEOUT 60
      ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
  endforeach()
  add_test(NAME reacquisition_source_cutoff_parity COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reacquisition_cutoff_parity.py
    --node $<TARGET_FILE:hope_flight_prediction_v2_node>
    --replay $<TARGET_FILE:hope_flight_prediction_v2_replay>
    --output ${CMAKE_CURRENT_BINARY_DIR}/reacquisition_source_cutoff_parity.json)
  set_tests_properties(reacquisition_source_cutoff_parity PROPERTIES RUN_SERIAL TRUE TIMEOUT 120
    ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
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
    foreach(scenario net_clear net_collision uncertain_contact held)
      add_test(NAME review_geometry_${rate}_${scenario} COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_producer_chain.py
        --planner $<TARGET_FILE:hope_planner_cpp_node>
        --decoder $<TARGET_FILE:hope_integrated_runner_probe>
        --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
        --rate ${rate} --scenario ${scenario}
        --output ${CMAKE_CURRENT_BINARY_DIR}/review_geometry_${rate}_${scenario}.json)
      set_tests_properties(review_geometry_${rate}_${scenario} PROPERTIES
        RUN_SERIAL TRUE TIMEOUT 60 ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
    endforeach()
    foreach(speed 10 12 20)
      foreach(spin unknown zero known)
        add_test(NAME review_chain_${rate}_${speed}_${spin} COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_producer_chain.py
          --planner $<TARGET_FILE:hope_planner_cpp_node>
          --decoder $<TARGET_FILE:hope_integrated_runner_probe>
          --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
          --rate ${rate} --speed ${speed} --spin ${spin}
          --output ${CMAKE_CURRENT_BINARY_DIR}/review_chain_${rate}_${speed}_${spin}.json)
        set_tests_properties(review_chain_${rate}_${speed}_${spin} PROPERTIES
          RUN_SERIAL TRUE TIMEOUT 60 ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
      endforeach()
    endforeach()
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

  ament_add_gtest(test_runtime_plane_reference test/test_runtime_plane_reference.cpp)
  target_link_libraries(test_runtime_plane_reference hope_planner_core Eigen3::Eigen)
  ament_add_gtest(test_v2_strike_adapter test/test_v2_strike_adapter.cpp)
  set_target_properties(test_v2_strike_adapter PROPERTIES CXX_STANDARD 20)
  target_link_libraries(test_v2_strike_adapter hope_v2_incoming_core)
  ament_add_gtest(test_runner_preengagement_revocation test/test_runner_preengagement_revocation.cpp)
  set_target_properties(test_runner_preengagement_revocation PROPERTIES CXX_STANDARD 20)
  target_include_directories(test_runner_preengagement_revocation PRIVATE "${HOPE_RECORDING_INCLUDE_DIR}")
  target_link_libraries(test_runner_preengagement_revocation Eigen3::Eigen)
  ament_add_gtest(test_v2_incoming_coordinator test/test_v2_incoming_coordinator.cpp)
  target_link_libraries(test_v2_incoming_coordinator hope_v2_incoming_core)
  ament_add_gtest(test_planner_runtime_plane test/test_planner_runtime_plane.cpp src/planner_node.cpp)
  ament_add_gtest(test_v2_incoming_ros test/test_v2_incoming_ros.cpp src/planner_node.cpp)
  set_target_properties(test_v2_incoming_ros PROPERTIES CXX_STANDARD 20)
  target_link_libraries(test_v2_incoming_ros hope_integrated_runtime rclcpp::rclcpp
    ${geometry_msgs_TARGETS} ${hope_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS}
    ${rcl_interfaces_TARGETS} ${diagnostic_msgs_TARGETS})
  set_tests_properties(test_v2_incoming_ros PROPERTIES RUN_SERIAL TRUE TIMEOUT 120)
  target_link_libraries(test_planner_runtime_plane hope_integrated_runtime)
  target_link_libraries(test_planner_runtime_plane hope_planner_core Eigen3::Eigen rclcpp::rclcpp
    ${diagnostic_msgs_TARGETS} ${geometry_msgs_TARGETS} ${hope_msgs_TARGETS}
    ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${rcl_interfaces_TARGETS})

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
    ${rcl_interfaces_TARGETS}
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
    ${rcl_interfaces_TARGETS}
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
    ${rcl_interfaces_TARGETS}
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
target_link_libraries(hope_flight_prediction_v2_node PRIVATE hope_v2_transport ${std_msgs_TARGETS} ${diagnostic_msgs_TARGETS})
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
  ament_add_pytest_test(qualification_accounting test/test_qualification_accounting.py)
  ament_add_pytest_test(lifecycle_observer test/test_lifecycle_observer.py)
  ament_add_pytest_test(coverage_outcome_audit test/test_coverage_outcome_audit.py)
  ament_add_pytest_test(qualification_relay_shutdown test/test_relay_shutdown.py)
  set_tests_properties(qualification_relay_shutdown PROPERTIES RUN_SERIAL TRUE
    ENVIRONMENT "ROS_DOMAIN_ID=194;ROS_LOCALHOST_ONLY=1")
  ament_add_pytest_test(qualification_relay_backlog test/test_relay_ingress_backlog.py TIMEOUT 45)
  set_tests_properties(qualification_relay_backlog PROPERTIES RUN_SERIAL TRUE
    ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_ros_context_guard test/test_ros_context_guard.cpp)
  target_include_directories(test_ros_context_guard PRIVATE include)
  ament_target_dependencies(test_ros_context_guard rclcpp)
  add_test(NAME qualification_failure_receipts COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/failure_receipt_gate.py
    --planner $<TARGET_FILE:hope_planner_cpp_node>
    --decoder $<TARGET_FILE:hope_integrated_runner_probe>
    --bringup ${CMAKE_INSTALL_PREFIX}/../hope_bringup/lib/hope_bringup
    --output ${CMAKE_CURRENT_BINARY_DIR}/qualification_failure_receipts.json)
  set_tests_properties(qualification_failure_receipts PROPERTIES RUN_SERIAL TRUE TIMEOUT 50
    ENVIRONMENT "ROS_DOMAIN_ID=193;ROS_LOCALHOST_ONLY=1")
  add_test(NAME integrated_context_noop COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_context_noop.py
    --executable $<TARGET_FILE:hope_flight_prediction_v2_node>)
  set_tests_properties(integrated_context_noop PROPERTIES RUN_SERIAL TRUE TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=195;ROS_LOCALHOST_ONLY=1")
  add_test(NAME review_context_worker_race COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/integrated_context_noop.py
    --executable $<TARGET_FILE:hope_flight_prediction_v2_node> --worker-delay-ms 12)
  set_tests_properties(review_context_worker_race PROPERTIES RUN_SERIAL TRUE TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=195;ROS_LOCALHOST_ONLY=1")
  add_test(NAME review_source_cutoff_parity COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/review_cutoff_parity.py
    --node $<TARGET_FILE:hope_flight_prediction_v2_node>
    --replay $<TARGET_FILE:hope_flight_prediction_v2_replay>
    --output ${CMAKE_CURRENT_BINARY_DIR}/review_source_cutoff_parity.json)
  set_tests_properties(review_source_cutoff_parity PROPERTIES RUN_SERIAL TRUE TIMEOUT 45
    ENVIRONMENT "ROS_DOMAIN_ID=196;ROS_LOCALHOST_ONLY=1")
  add_test(NAME review_startup_guards COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/review_startup_guards.py
    --node $<TARGET_FILE:hope_planner_cpp_node>)
  set_tests_properties(review_startup_guards PROPERTIES RUN_SERIAL TRUE TIMEOUT 40
    ENVIRONMENT "ROS_DOMAIN_ID=197;ROS_LOCALHOST_ONLY=1")
  add_test(NAME flight_v2_installed_api COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/installed_forward_api.py --prefix ${CMAKE_INSTALL_PREFIX})
  ament_add_gtest(test_flight_prediction_v2 test/test_flight_prediction_v2.cpp)
  target_link_libraries(test_flight_prediction_v2 hope_v2_forecast_core)
  ament_add_gtest(test_v2_contact_corroboration test/test_v2_contact_corroboration.cpp)
  target_link_libraries(test_v2_contact_corroboration hope_v2_forecast_core)
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
