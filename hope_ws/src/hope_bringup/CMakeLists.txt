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
  scripts/agibot_mdu_timesync_supervised.sh
  scripts/avatar_pro_vrpn_relay
  scripts/fake_ball_publisher
  scripts/fake_optitrack_publisher
  scripts/mocap_frame_probe.py
  scripts/mocap_rate_probe.py
  scripts/natnet_preflight.py
  scripts/p1_marker_cad_calibrator
  scripts/p1_marker_cad_calibration_server
  scripts/hope_base_pose_flat_relay
  scripts/p1_pelvis_calibrator
  scripts/p1_pelvis_tf_publisher
  scripts/optitrack_mct_relay
  scripts/pose_to_posearray
  scripts/rally_v10_wait_flat.py
  scripts/run_rally_v10_hdu.sh
  scripts/with_fastdds_unicast.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/base_pose_contract.py
  scripts/p1_calibration.py
  scripts/source_time_rate_gate.py
  scripts/table_side_transform.py
  scripts/p1_marker_layout.py
  scripts/p1_marker_cad_registration_impl.py
  scripts/p1_pelvis_calibration_impl.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the canonical hardware table, not a hand-maintained numeric twin.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../agibot/mocap_sticker_shell_v3/documents/marker_transforms_pelvis_link_stickers.csv
  DESTINATION share/${PROJECT_NAME}/config/marker_layouts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the copyright linter until per-file headers are added.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(table_side_transform_test test/test_table_side_transform.py)
  ament_add_pytest_test(source_time_rate_gate_test test/test_source_time_rate_gate.py)
  ament_add_pytest_test(natnet_preflight_test test/test_natnet_preflight.py)
  ament_add_pytest_test(rally_mocap_interface_test test/test_rally_mocap_interface.py)
  ament_add_pytest_test(natnet_launch_interfaces_test test/test_natnet_launch_interfaces.py)
  ament_add_pytest_test(pose_to_posearray_test test/test_pose_to_posearray.py
    TIMEOUT 60
  )
  ament_add_pytest_test(p1_pelvis_calibrator_test test/test_p1_pelvis_calibrator.py)
  ament_add_pytest_test(p1_pelvis_tf_publisher_test test/test_p1_pelvis_tf_publisher.py)
  ament_add_pytest_test(
    natnet_preflight_clock_sync_test
    test/test_natnet_preflight_clock_sync.py
  )
  ament_add_pytest_test(
    p1_marker_cad_registration_test
    test/test_p1_marker_cad_registration.py
  )
  ament_add_pytest_test(mocap_frame_probe_test test/test_mocap_frame_probe.py)
  ament_add_pytest_test(base_pose_contract_test test/test_base_pose_contract.py)
  ament_add_pytest_test(p1_calibration_test test/test_p1_calibration.py)
  ament_add_pytest_test(public_sticker_integration_test test/test_public_sticker_integration.py)
  ament_add_pytest_test(p1_marker_layout_test test/test_p1_marker_layout.py)
  ament_add_pytest_test(v3_optitrack_asset_test test/test_v3_optitrack_asset.py)
  ament_add_pytest_test(p1_marker_cad_calibration_server_test test/test_p1_marker_cad_calibration_server.py)
endif()

ament_package()
