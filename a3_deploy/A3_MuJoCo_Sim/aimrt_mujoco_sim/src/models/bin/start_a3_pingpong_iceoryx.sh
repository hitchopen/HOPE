#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

start_iox_roudi() {
  if pgrep -x iox-roudi >/dev/null; then
    echo "iox-roudi is already running"
    return
  fi

  local roudi_cmd=""
  if [ -x ./iox-roudi ]; then
    roudi_cmd="./iox-roudi"
  elif command -v iox-roudi >/dev/null 2>&1; then
    roudi_cmd="$(command -v iox-roudi)"
  else
    echo "iox-roudi not found; please start it before running iceoryx body_drive topics" >&2
    return
  fi

  if command -v setsid >/dev/null 2>&1; then
    setsid "${roudi_cmd}" >/tmp/a3_pingpong_iox_roudi.log 2>&1 </dev/null &
  else
    nohup "${roudi_cmd}" >/tmp/a3_pingpong_iox_roudi.log 2>&1 </dev/null &
  fi
  disown "$!" 2>/dev/null || true

  sleep 1
}

source_if_exists() {
  if [ -f "$1" ]; then
    # shellcheck source=/dev/null
    source "$1"
  fi
}

start_iox_roudi

source_if_exists ../share/ros2_plugin_proto/local_setup.bash
source_if_exists ../share/aimrt_msgs/local_setup.bash
source_if_exists ../share/joint_msgs/local_setup.bash
source_if_exists ../share/mujoco_sim_msgs/local_setup.bash

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:$(pwd):$(pwd)/../lib"

./aimrt_main --cfg_file_path=./cfg/a3_pingpong_iceoryx_cfg.yaml
