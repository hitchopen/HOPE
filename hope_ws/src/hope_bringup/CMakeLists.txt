cmake_minimum_required(VERSION 3.8)
project(hope_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_ball_publisher
  scripts/fake_optitrack_publisher
  scripts/mocap_rate_probe.py
  scripts/natnet_preflight.py
  scripts/p1_pelvis_calibrator
  scripts/p1_pelvis_tf_publisher
  scripts/optitrack_mct_relay
  scripts/pose_to_posearray
  scripts/with_fastdds_unicast.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/p1_pelvis_calibration_impl.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the copyright linter until per-file headers are added.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pose_to_posearray_test test/test_pose_to_posearray.py
    TIMEOUT 60
  )
  ament_add_pytest_test(p1_pelvis_calibrator_test test/test_p1_pelvis_calibrator.py)
  ament_add_pytest_test(p1_pelvis_tf_publisher_test test/test_p1_pelvis_tf_publisher.py)
endif()

ament_package()
