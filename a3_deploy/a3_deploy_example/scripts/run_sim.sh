#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
A3_DEPLOY_ROOT="$(cd -- "${DEPLOY_ROOT}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./run_sim.sh [--print-paths] [cfg.yaml]

Environment:
  A3_SIM_FLAVOR=auto|source|standalone
      auto       prefer the source-built A3_MuJoCo_Sim package when present,
                 otherwise fall back to mujoco_sim_standalone
      source     use the source-built A3_MuJoCo_Sim install package
      standalone use the tracked mujoco_sim_standalone package
  A3_SIM_INSTALL=/abs/path/to/.../build/install
      override the source-sim install root
  A3_SIM_CFG=a3_pingpong_iceoryx_cfg.yaml
      source-sim cfg name/path; default keeps the shared /body_drive/* loop

Notes:
  - Both source and standalone sims publish the same /body_drive/* interface.
  - Only the source sim publishes /sim/a3/* oracle/reset topics.
EOF
}

source_if_exists() {
  local setup_file=$1
  if [[ -f "${setup_file}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
  fi
}

start_iox_roudi_if_needed() {
  local sim_bin=$1
  local roudi_log="${A3_IOX_ROUDI_LOG:-/tmp/a3_pingpong_iox_roudi.log}"
  if pgrep -x iox-roudi >/dev/null 2>&1; then
    echo "[run_sim] iox-roudi already running"
    return
  fi

  local roudi_cmd=""
  if [[ -x "${sim_bin}/iox-roudi" ]]; then
    roudi_cmd="${sim_bin}/iox-roudi"
  elif command -v iox-roudi >/dev/null 2>&1; then
    roudi_cmd="$(command -v iox-roudi)"
  else
    echo "iox-roudi not found; please start it before running the iceoryx sim" >&2
    exit 66
  fi

  if command -v setsid >/dev/null 2>&1; then
    setsid "${roudi_cmd}" >"${roudi_log}" 2>&1 </dev/null &
  else
    nohup "${roudi_cmd}" >"${roudi_log}" 2>&1 </dev/null &
  fi
  echo "[run_sim] iox-roudi log=${roudi_log}"
  disown "$!" 2>/dev/null || true
  sleep 1
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
    if [[ -x "${candidate}/bin/aimrt_main" && -d "${candidate}/bin/cfg" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

resolve_source_cfg() {
  local sim_bin=$1
  local requested=${2:-}
  local cfg_dir="${sim_bin}/cfg"
  if [[ -z "${requested}" ]]; then
    requested="${A3_SIM_CFG:-a3_pingpong_iceoryx_cfg.yaml}"
  fi
  if [[ -f "${requested}" ]]; then
    realpath "${requested}"
    return 0
  fi
  if [[ -f "${cfg_dir}/${requested}" ]]; then
    realpath "${cfg_dir}/${requested}"
    return 0
  fi
  echo "source sim cfg not found: ${requested}" >&2
  exit 66
}

start_source_sim() {
  local sim_install=$1
  local cfg_request=${2:-}
  local sim_bin="${sim_install}/bin"
  local cfg_file
  cfg_file="$(resolve_source_cfg "${sim_bin}" "${cfg_request}")"

  # Source the host ROS 2 underlay so libaimrt_ros2_plugin.so can resolve
  # librclcpp.so et al. The distro dir is not always 'jazzy' (this host ships
  # 'lyrical'), so honor A3_ROS_DISTRO / ROS_DISTRO, else auto-pick the first
  # /opt/ros/*/setup.bash found.
  local ros_setup=""
  for d in "${A3_ROS_DISTRO:-}" "${ROS_DISTRO:-}"; do
    if [[ -n "${d}" && -f "/opt/ros/${d}/setup.bash" ]]; then
      ros_setup="/opt/ros/${d}/setup.bash"
      break
    fi
  done
  if [[ -z "${ros_setup}" ]]; then
    for f in /opt/ros/*/setup.bash; do
      if [[ -f "${f}" ]]; then
        ros_setup="${f}"
        break
      fi
    done
  fi
  if [[ -n "${ros_setup}" ]]; then
    echo "[run_sim] sourcing ROS underlay: ${ros_setup}"
    source_if_exists "${ros_setup}"
  else
    echo "[run_sim] WARN: no /opt/ros/*/setup.bash found; ros2 plugin may fail to load librclcpp.so" >&2
  fi
  source_if_exists "${sim_install}/share/ros2_plugin_proto/local_setup.bash"
  source_if_exists "${sim_install}/share/aimrt_msgs/local_setup.bash"
  source_if_exists "${sim_install}/share/joint_msgs/local_setup.bash"
  source_if_exists "${sim_install}/share/mujoco_sim_msgs/local_setup.bash"

  export LD_LIBRARY_PATH="${sim_bin}:${sim_install}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  start_iox_roudi_if_needed "${sim_bin}"

  echo "[run_sim] flavor=source install=${sim_install} cfg=${cfg_file}"
  cd "${sim_bin}"
  exec ./aimrt_main --cfg_file_path="${cfg_file}"
}

find_standalone_dir() {
  local candidate="${DEPLOY_ROOT}/mujoco_sim_standalone"
  if [[ -x "${candidate}/run.sh" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi
  return 1
}

PRINT_PATHS=0
case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
  --print-paths)
    PRINT_PATHS=1
    shift
    ;;
esac

SIM_FLAVOR="${A3_SIM_FLAVOR:-auto}"
SOURCE_SIM_INSTALL="$(find_source_sim_install || true)"
STANDALONE_DIR="$(find_standalone_dir || true)"

if [[ "${PRINT_PATHS}" == "1" ]]; then
  echo "source_sim_install=${SOURCE_SIM_INSTALL:-<missing>}"
  echo "standalone_dir=${STANDALONE_DIR:-<missing>}"
  echo "selected_flavor=${SIM_FLAVOR}"
  exit 0
fi

case "${SIM_FLAVOR}" in
  auto)
    if [[ -n "${SOURCE_SIM_INSTALL}" ]]; then
      SIM_FLAVOR="source"
    elif [[ -n "${STANDALONE_DIR}" ]]; then
      SIM_FLAVOR="standalone"
    else
      echo "no shared-interface sim package found under ${AGI_ROOT} or ${DEPLOY_ROOT}" >&2
      exit 66
    fi
    ;;
  source|standalone)
    ;;
  *)
    echo "invalid A3_SIM_FLAVOR='${SIM_FLAVOR}'; expected auto, source, or standalone" >&2
    exit 64
    ;;
esac

case "${SIM_FLAVOR}" in
  source)
    if [[ -z "${SOURCE_SIM_INSTALL}" ]]; then
      echo "A3_SIM_FLAVOR=source requested, but no source sim install was found" >&2
      exit 66
    fi
    start_source_sim "${SOURCE_SIM_INSTALL}" "${1:-}"
    ;;
  standalone)
    if [[ -z "${STANDALONE_DIR}" ]]; then
      echo "A3_SIM_FLAVOR=standalone requested, but mujoco_sim_standalone is missing" >&2
      exit 66
    fi
    echo "[run_sim] flavor=standalone package=${STANDALONE_DIR}"
    exec "${STANDALONE_DIR}/run.sh" "$@"
    ;;
esac
