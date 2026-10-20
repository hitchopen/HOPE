#!/usr/bin/env bash
# Run inside the HOPE Jazzy distrobox; all products are kept outside the source tree.
set -eo pipefail
repo=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
validation_root=${1:?Usage: verify.sh /absolute/empty/validation-output-directory}
if [ -e "$validation_root" ]; then
  echo "Validation directory must not already exist: $validation_root" >&2
  exit 2
fi
mkdir -p "$validation_root"
# Keep private/stale workspace overlays out of the clean public build.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-187}
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
cd "$repo/hope_ws"
colcon --log-base "$validation_root/log" build \
  --base-paths src ../NatNet2ROS2/src/motion_capture_tracking_interfaces \
  --build-base "$validation_root/build" --install-base "$validation_root/install" \
  --packages-up-to hope_planner_cpp hope_bringup motion_capture_tracking_interfaces \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$validation_root/install/setup.bash"
colcon --log-base "$validation_root/log" test \
  --build-base "$validation_root/build" --install-base "$validation_root/install" \
  --packages-select hope_msgs hope_planner_cpp hope_bringup --event-handlers console_direct+
# Deliberately preserve the complete gate's failure status, including existing lint failures.
colcon test-result --test-result-base "$validation_root/build" --verbose
