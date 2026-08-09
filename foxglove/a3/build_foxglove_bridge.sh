#!/bin/bash
# Build the pinned ROS 2 foxglove_bridge release against the A3's Jazzy tree.
# Run on the A3 as user agi. Both source repositories and the two compatibility
# patches are pinned and verified before the build starts.
set -Eeo pipefail

# The vendor setup script reads unset colcon variables, so source it before
# enabling nounset. All HOPE build logic below still runs with `set -u`.
source /opt/ros/jazzy/setup.bash
set -u

readonly FOXGLOVE_SDK_REPOSITORY="https://github.com/foxglove/foxglove-sdk.git"
readonly FOXGLOVE_ROS_RELEASE="ros-v3.4.3"
readonly FOXGLOVE_ROS_COMMIT="05f27efc7e535d9c30c6b0cb4f6aa89de7243870"
readonly ROSX_REPOSITORY="https://github.com/facontidavide/rosx_introspection.git"
readonly ROSX_COMMIT="ab747a0d3970d3297a5652b82e7645ab1d11feb9"
readonly FOXGLOVE_WORKSPACE="${HOME}/hope_foxglove_ws"
readonly FOXGLOVE_SOURCE="${FOXGLOVE_WORKSPACE}/foxglove-sdk"
readonly ROSX_SOURCE="${FOXGLOVE_WORKSPACE}/rosx_introspection"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly FOXGLOVE_PATCH="${SCRIPT_DIR}/patches/foxglove_bridge-ament-index-1.8.patch"
readonly ROSX_PATCH="${SCRIPT_DIR}/patches/rosx_introspection-ament-index-1.8.patch"
readonly FOXGLOVE_PATCH_SHA256="1c6f40f6af4fe0186f196f65fb04c4d79b585ae16df448f16c72e09887d58828"
readonly ROSX_PATCH_SHA256="bd6541d663b57505cc083b7d67aa5593c6297928adee04ed72e64ae35f6e4da5"
readonly FOXGLOVE_PATCHED_FILE="ros/src/foxglove_bridge/src/message_definition_cache.cpp"
readonly ROSX_PATCHED_FILE="src/ros_utils/message_definition_cache.cpp"
readonly FOXGLOVE_PATCHED_FILE_SHA256="ff879cd712a4d167169c5d229a0f67e2c112f42b65ecb62404d6dc51cb44a8f1"
readonly ROSX_PATCHED_FILE_SHA256="c3100994dea0fdc6dc2b87614ed35b1582d95819cd3f714a062910da64e36d16"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

checkout_pinned_source() {
    local repository="$1"
    local commit="$2"
    local destination="$3"
    local label="$4"

    if [[ ! -d "${destination}/.git" ]]; then
        if [[ -e "${destination}" ]] && [[ -n "$(ls -A "${destination}" 2>/dev/null)" ]]; then
            die "${destination} exists but is not a ${label} Git checkout."
        fi
        mkdir -p "${destination}"
        git -C "${destination}" init --quiet
        git -C "${destination}" remote add origin "${repository}"
        git -C "${destination}" fetch --depth 1 origin "${commit}"
        git -C "${destination}" checkout --quiet --detach FETCH_HEAD
    fi

    local actual_commit
    actual_commit="$(git -C "${destination}" rev-parse HEAD 2>/dev/null)" || \
        die "${destination} is an incomplete ${label} checkout."
    if [[ "${actual_commit}" != "${commit}" ]]; then
        die "${destination} is ${actual_commit}, expected ${commit}. Remove only that dedicated source directory and rerun."
    fi
}

apply_verified_patch() {
    local repository="$1"
    local patch="$2"
    local patch_sha256="$3"
    local patched_file="$4"
    local patched_file_sha256="$5"
    local label="$6"

    [[ -f "${patch}" ]] || die "missing ${label} compatibility patch: ${patch}"
    echo "${patch_sha256}  ${patch}" | sha256sum --check --status || \
        die "${label} compatibility patch checksum mismatch."

    local status
    status="$(git -C "${repository}" status --porcelain --untracked-files=normal)"
    if [[ -z "${status}" ]]; then
        git -C "${repository}" apply --check "${patch}" || \
            die "${label} compatibility patch does not apply to the pinned source."
        git -C "${repository}" apply "${patch}"
    elif [[ "${status}" == " M ${patched_file}" ]]; then
        git -C "${repository}" apply --reverse --check "${patch}" || \
            die "${label} source is modified, but not by the expected compatibility patch."
    else
        die "${label} source has unexpected local or untracked changes: ${status}"
    fi

    echo "${patched_file_sha256}  ${repository}/${patched_file}" | \
        sha256sum --check --status || \
        die "${label} patched source checksum mismatch."
}

mkdir -p "${FOXGLOVE_WORKSPACE}"
checkout_pinned_source \
    "${FOXGLOVE_SDK_REPOSITORY}" "${FOXGLOVE_ROS_COMMIT}" \
    "${FOXGLOVE_SOURCE}" "Foxglove SDK ${FOXGLOVE_ROS_RELEASE}"
checkout_pinned_source \
    "${ROSX_REPOSITORY}" "${ROSX_COMMIT}" \
    "${ROSX_SOURCE}" "rosx_introspection 3.1.1"

apply_verified_patch \
    "${FOXGLOVE_SOURCE}" "${FOXGLOVE_PATCH}" "${FOXGLOVE_PATCH_SHA256}" \
    "${FOXGLOVE_PATCHED_FILE}" "${FOXGLOVE_PATCHED_FILE_SHA256}" \
    "Foxglove SDK"
apply_verified_patch \
    "${ROSX_SOURCE}" "${ROSX_PATCH}" "${ROSX_PATCH_SHA256}" \
    "${ROSX_PATCHED_FILE}" "${ROSX_PATCHED_FILE_SHA256}" \
    "rosx_introspection"

cd "${FOXGLOVE_SOURCE}/ros"
colcon build \
    --base-paths "${FOXGLOVE_SOURCE}/ros/src" "${ROSX_SOURCE}" \
    --packages-up-to foxglove_bridge \
    --cmake-clean-cache \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DFOXGLOVE_BRIDGE_REMOTE_ACCESS=OFF

echo "Built pinned ${FOXGLOVE_ROS_RELEASE}: ${FOXGLOVE_SOURCE}/ros/install/foxglove_bridge"
sha256sum "${FOXGLOVE_SOURCE}/ros/install/foxglove_bridge/lib/foxglove_bridge/foxglove_bridge"
echo "Next: install the systemd units per foxglove/README.md"
