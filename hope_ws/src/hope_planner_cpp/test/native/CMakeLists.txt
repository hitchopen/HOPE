cmake_minimum_required(VERSION 3.16)
project(hope_flight_native LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)
enable_testing()
get_filename_component(PLANNER ../.. ABSOLUTE)
add_library(hope_native_core
  ${PLANNER}/src/common_ball_physics.cpp ${PLANNER}/src/flight_spin_estimator.cpp
  ${PLANNER}/src/flight_state_v2.cpp
  ${PLANNER}/src/spin_estimator.cpp ${PLANNER}/src/batch_physics_estimator.cpp
  ${PLANNER}/src/trajectory_predictor.cpp ${PLANNER}/src/racket_target_planner.cpp
  ${PLANNER}/src/reach_permission.cpp ${PLANNER}/src/schema2_packer.cpp
  ${PLANNER}/src/schema3_packer.cpp ${PLANNER}/src/audit_logger.cpp
  ${PLANNER}/src/post_net_one_shot.cpp ${PLANNER}/src/incoming_trajectory.cpp
  ${PLANNER}/src/adaptive_flight_commit.cpp
  ${PLANNER}/src/probabilistic_physics.cpp ${PLANNER}/src/probabilistic_ball_tracker.cpp
  ${PLANNER}/src/nominal_event_forecast.cpp ${PLANNER}/src/fixed_lag_types.cpp
  ${PLANNER}/src/return_region.cpp ${PLANNER}/src/hit_plane_manager.cpp)
target_include_directories(hope_native_core PUBLIC ${PLANNER}/include)
target_include_directories(hope_native_core PRIVATE ${PLANNER}/../../../a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include)
target_link_libraries(hope_native_core PUBLIC Eigen3::Eigen)
target_compile_options(hope_native_core PRIVATE -Wall -Wextra -Wpedantic)
foreach(test flight_prediction_v2 spin_physics future_contact_predictor batch_physics_estimator
    trajectory_and_stage3 target_turn_guard prediction_event_order schema2
    spin_estimator audit_logger post_net_one_shot incoming_trajectory adaptive_flight_commit)
  add_executable(test_${test} ${PLANNER}/test/test_${test}.cpp)
  target_link_libraries(test_${test} PRIVATE hope_native_core GTest::gtest_main)
  add_test(NAME ${test} COMMAND test_${test})
endforeach()

add_executable(hope_flight_prediction_v2_replay ${PLANNER}/src/flight_prediction_v2_replay.cpp)
target_link_libraries(hope_flight_prediction_v2_replay PRIVATE hope_native_core)

add_library(hope_integrated_native
  ${PLANNER}/src/return_eligibility.cpp ${PLANNER}/src/validated_return_planner.cpp)
target_link_libraries(hope_integrated_native PUBLIC hope_native_core)
add_executable(test_integrated_return ${PLANNER}/test/test_integrated_return.cpp)
set_target_properties(test_integrated_return PROPERTIES CXX_STANDARD 20)
target_include_directories(test_integrated_return PRIVATE
  ${PLANNER}/../../../a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include)
target_link_libraries(test_integrated_return PRIVATE hope_integrated_native GTest::gtest_main)
add_test(NAME integrated_return COMMAND test_integrated_return)
add_executable(test_v2_pr64_review_regressions ${PLANNER}/test/test_v2_pr64_review_regressions.cpp)
target_link_libraries(test_v2_pr64_review_regressions PRIVATE hope_integrated_native GTest::gtest_main)
add_test(NAME v2_pr64_review_regressions COMMAND test_v2_pr64_review_regressions)
add_executable(hope_integrated_return_sweep ${PLANNER}/test/integrated_return_sweep.cpp)
target_link_libraries(hope_integrated_return_sweep PRIVATE hope_integrated_native)

add_executable(test_reacquisition_counterexamples ${PLANNER}/test/reacquisition_counterexamples.cpp)
target_link_libraries(test_reacquisition_counterexamples PRIVATE hope_native_core GTest::gtest_main)
add_test(NAME reacquisition_counterexamples COMMAND test_reacquisition_counterexamples)
