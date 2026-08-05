#!/bin/bash
# Build the pinned ROS 2 foxglove_bridge release against the A3's Jazzy tree.
# Run on the A3 as user agi. The source release and downloaded SDK archive are
# both pinned; CMake verifies the SDK archive's upstream SHA-256.
set -Eeuo pipefail

source /opt/ros/jazzy/setup.bash

readonly FOXGLOVE_SDK_REPOSITORY="https://github.com/foxglove/foxglove-sdk.git"
readonly FOXGLOVE_ROS_RELEASE="ros-v3.4.3"
readonly FOXGLOVE_ROS_COMMIT="05f27efc7e535d9c30c6b0cb4f6aa89de7243870"
readonly FOXGLOVE_WORKSPACE="${HOME}/hope_foxglove_ws"
readonly FOXGLOVE_SOURCE="${FOXGLOVE_WORKSPACE}/foxglove-sdk"

mkdir -p "${FOXGLOVE_WORKSPACE}"
if [[ ! -d "${FOXGLOVE_SOURCE}/.git" ]]; then
    git clone --branch "${FOXGLOVE_ROS_RELEASE}" --depth 1 \
        "${FOXGLOVE_SDK_REPOSITORY}" "${FOXGLOVE_SOURCE}"
fi

actual_commit=$(git -C "${FOXGLOVE_SOURCE}" rev-parse HEAD)
if [[ "${actual_commit}" != "${FOXGLOVE_ROS_COMMIT}" ]]; then
    echo "ERROR: ${FOXGLOVE_SOURCE} is ${actual_commit}, expected ${FOXGLOVE_ROS_COMMIT}." >&2
    echo "Remove only that dedicated source directory and rerun this script." >&2
    exit 1
fi
if [[ -n "$(git -C "${FOXGLOVE_SOURCE}" status --porcelain --untracked-files=normal)" ]]; then
    echo "ERROR: ${FOXGLOVE_SOURCE} has local or untracked changes." >&2
    echo "Use a clean checkout of the pinned release before building." >&2
    exit 1
fi

cd "${FOXGLOVE_SOURCE}/ros"
colcon build --packages-select foxglove_bridge \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DFOXGLOVE_BRIDGE_REMOTE_ACCESS=OFF

echo "Built pinned ${FOXGLOVE_ROS_RELEASE}: ${FOXGLOVE_SOURCE}/ros/install/foxglove_bridge"
echo "Next: install the systemd units per foxglove/README.md"
