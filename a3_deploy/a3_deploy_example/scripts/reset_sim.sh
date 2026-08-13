#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
A3_DEPLOY_ROOT="$(cd -- "${DEPLOY_ROOT}/.." && pwd)"

source_if_exists() {
  local setup_file=$1
  if [[ -f "${setup_file}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
  fi
}

find_source_sim_install() {
  local candidate
  local -a candidates=()
  if [[ -n "${A3_SIM_INSTALL:-}" ]]; then
    candidates+=("${A3_SIM_INSTALL}")
  fi
  candidates+=(
    "${A3_DEPLOY_ROOT}/A3_MuJoCo_Sim/aimrt_mujoco_sim/build/install"
    "${A3_DEPLOY_ROOT}/aimrt_mujoco_sim_source/aimrt_mujoco_sim/build/install"
  )
  for candidate in "${candidates[@]}"; do
    if [[ -d "${candidate}/share/mujoco_sim_msgs" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

if [[ "${1:-}" == "--print-paths" ]]; then
  echo "source_sim_install=$(find_source_sim_install || echo '<missing>')"
  exit 0
fi

INSTALL="$(find_source_sim_install || true)"
if [[ -z "${INSTALL}" ]]; then
  echo "reset_sim.sh requires the source-built A3_MuJoCo_Sim install (for /sim/a3/reset)" >&2
  exit 66
fi

source_if_exists /opt/ros/jazzy/setup.bash
source_if_exists "${INSTALL}/share/ros2_plugin_proto/local_setup.bash"
source_if_exists "${INSTALL}/share/aimrt_msgs/local_setup.bash"
source_if_exists "${INSTALL}/share/joint_msgs/local_setup.bash"
source_if_exists "${INSTALL}/share/mujoco_sim_msgs/local_setup.bash"

export AMENT_PREFIX_PATH="${INSTALL}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${INSTALL}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${INSTALL}/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

echo "[reset_sim] install=${INSTALL} topic=/sim/a3/reset"
exec ros2 topic pub --once /sim/a3/reset mujoco_sim_msgs/msg/SimReset \
  "{mode: 1, keyframe_id: 0, set_base: false, set_base_twist: false, set_joints: false, zero_all_velocities: true, clear_ctrl: false}"
