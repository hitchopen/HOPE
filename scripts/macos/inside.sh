#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ -d /opt/hope/ros ]]; then
  for resource in src calibration_receipts; do
    if [[ -d "/work/hope_ws/$resource" && ! -e "/opt/hope/ros/$resource" ]]; then
      ln -s "/work/hope_ws/$resource" "/opt/hope/ros/$resource"
    fi
  done
fi
if [[ -f /opt/hope/ros/install/local_setup.bash ]]; then
  source /opt/hope/ros/install/local_setup.bash
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${HOPE_JOBS:-2}"
export MAKEFLAGS="-j${HOPE_JOBS:-2}"

ros_source_paths() {
  source_paths=(src)
  # Discover both capture backends for build and test, without moving sources.
  # Older layouts may already keep them under hope_ws/src.
  for workspace in NatNet2ROS2 VRPN2ROS2; do
    if [[ -d "${1:-/work}/$workspace/src" ]]; then
      source_paths+=("../$workspace/src")
    fi
  done
}

operation="$1"
shift
case "$operation" in
  shell|exec)
    if [[ "$operation" == shell ]]; then set -- bash; fi
    if [[ "${HOPE_VIRTUAL_DISPLAY:-0}" == 1 ]]; then
      exec xvfb-run -a -s '-screen 0 1280x720x24' "$@"
    fi
    exec "$@"
    ;;
  build-ros)
    mkdir -p /opt/hope/ros
    if [[ ! -e /opt/hope/ros/src ]]; then
      ln -s /work/hope_ws/src /opt/hope/ros/src
    fi
    if [[ -d /work/hope_ws/calibration_receipts && ! -e /opt/hope/ros/calibration_receipts ]]; then
      ln -s /work/hope_ws/calibration_receipts /opt/hope/ros/calibration_receipts
    fi
    cd /work/hope_ws
    cache_args=()
    ros_source_paths
    if [[ "${HOPE_ROS_CLEAN_CACHE:-0}" == 1 ]]; then
      cache_args+=(--cmake-clean-cache)
    fi
    colcon --log-base /opt/hope/ros/log build --symlink-install \
      --base-paths "${source_paths[@]}" \
      --build-base /opt/hope/ros/build --install-base /opt/hope/ros/install \
      --executor sequential "${cache_args[@]}" \
      --cmake-args -DBUILD_TESTING="${HOPE_ROS_TESTING:-ON}" \
      -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
    ;;
  test-ros)
    cd /work/hope_ws
    ros_source_paths
    test_packages=(hope_planner_cpp motion_capture_tracking)
    if [[ -d /opt/hope/ros/build/vrpn_mocap ]]; then
      test_packages+=(vrpn_mocap)
    fi
    ctest_args=()
    if [[ "${HOPE_ROS_LINT:-0}" != 1 ]]; then
      printf 'Running functional ROS regression tests; inherited ament linters are separate (--with-lint).\n'
      ctest_args=(--ctest-args -LE linter)
    fi
    colcon --log-base /opt/hope/ros/log test \
      --base-paths "${source_paths[@]}" \
      --build-base /opt/hope/ros/build --install-base /opt/hope/ros/install \
      --packages-select "${test_packages[@]}" --executor sequential \
      --return-code-on-test-failure "${ctest_args[@]}"
    if [[ "${HOPE_ROS_LINT:-0}" == 1 ]]; then
      colcon --log-base /opt/hope/ros/log test-result --test-result-base /opt/hope/ros/build --verbose
    fi
    PYTHONPATH=/work/hope_ws/src/hope_planner${PYTHONPATH:+:$PYTHONPATH} \
      /usr/bin/python3 -m pytest /work/hope_ws/src/hope_planner/test -q
    /usr/bin/python3 -m pytest -q \
      /work/foxglove/tests/test_laptop_runtime_selection.py \
      /work/foxglove/tests/test_lifecycle_core.py \
      /work/foxglove/tests/test_macos_build_entry.py \
      /work/foxglove/tests/test_macos_operator_contract.py \
      /work/foxglove/tests/test_macos_operator_dispatch.py \
      /work/foxglove/tests/test_operator_profiles.py \
      /work/foxglove/tests/test_mocap_setup.py \
      /work/hope_ws/src/hope_bringup/test/test_vrpn_pants_contract.py \
      /work/hope_ws/src/hope_bringup/test/test_vrpn_pants_relay.py \
      /work/a3_deploy/a3_deploy_example/tests/test_a3p_gripper_bridge.py
    ROS_DOMAIN_ID=229 ROS_LOCALHOST_ONLY=1 \
      /usr/bin/python3 -m pytest -q /work/foxglove/tests/test_mocap_config_transport_ros.py
    ;;
  build-runner)
    cd "${HOPE_DEPLOY_DIR:?missing deployment source path}"
    mkdir -p thirdparty/onnxruntime
    source setup_a3_env.sh
    # Newer package scripts honor this workaround for GCC 13 LTO jobserver
    # failures under x86 emulation. Older checkouts may still require it to be
    # implemented in their package builder; this entry does not patch sources.
    export A3_TBB_ENABLE_IPO=OFF
    options=(--arch x86_64 --jobs "${HOPE_JOBS:-2}")
    if [[ "${HOPE_BUILD_ONLY:-0}" == 1 ]]; then
      options+=(--build-only)
    else
      : "${HOPE_POLICY_DIR:?provide --policy-dir or --build-only}"
      options+=(--policy-dir "$HOPE_POLICY_DIR" --runtime-cfg \
        src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml)
      if [[ -n "${HOPE_TELEOP_DIR:-}" ]]; then
        options+=(--teleop-policy-dir "$HOPE_TELEOP_DIR")
      fi
    fi
    bash scripts/build_a3_deploy_pkg.sh "${options[@]}"
    ;;
  build-sim)
    sim="${HOPE_SIM_DIR:?missing simulator source path}"
    cmake -S "$sim" -B /opt/hope/sim/build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/hope/sim/install \
      -DAIMRT_MUJOCO_SIM_INSTALL=ON -DAIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=ON \
      -DAIMRT_MUJOCO_SIM_BUILD_TESTS=OFF -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=OFF \
      -DTBB_ENABLE_IPO=OFF \
      -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
    cmake --build /opt/hope/sim/build --target install --parallel "${HOPE_JOBS:-2}"
    # Upstream's models target copies to build/install independently of the
    # chosen CMAKE_INSTALL_PREFIX; stage those runtime resources here as well.
    cmake -E copy_directory "$sim/src/models/bin" /opt/hope/sim/install/bin
    ;;
  inspect-policy)
    : "${HOPE_POLICY_DIR:?provide --policy-dir}"
    dist="${HOPE_DEPLOY_DIR:?missing deployment source path}/dist/a3_deploy_x86_64"
    LD_LIBRARY_PATH="$dist:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}" \
      "$dist/a3_deploy_onnx_ref_pingpong" --inspect-policy --policy-dir "$HOPE_POLICY_DIR"
    ;;
  *) echo "Unknown operation: $operation" >&2; exit 64 ;;
esac
