#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

TRANSPORT="iceoryx"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros2)
      TRANSPORT="ros2"
      shift
      ;;
    --iceoryx)
      TRANSPORT="iceoryx"
      shift
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  ./run_a3_body_drive_debug_record.sh          # iceoryx, default
  ./run_a3_body_drive_debug_record.sh --ros2   # ROS2

Records raw /body_drive topics until Ctrl+C. The raw bag keeps the latest
approximately 256MB by AimRT rotation (256MB x 1 file).
USAGE
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 64
      ;;
  esac
done

case "${TRANSPORT}" in
  iceoryx|ros2) ;;
  *)
    echo "invalid transport: ${TRANSPORT}" >&2
    exit 64
    ;;
esac

if [[ ! -x "${SCRIPT_DIR}/a3_body_drive_debug_record" ]]; then
  echo "missing executable: ${SCRIPT_DIR}/a3_body_drive_debug_record" >&2
  echo "rebuild the A3 deploy package with ENABLE_A3_AIMRT_BACKEND=ON and ENABLE_A3_ROS_MSGS=ON" >&2
  exit 66
fi

CFG_TEMPLATE="${SCRIPT_DIR}/config/a3_body_drive_debug_record.${TRANSPORT}.yaml"
if [[ ! -f "${CFG_TEMPLATE}" ]]; then
  echo "missing config template: ${CFG_TEMPLATE}" >&2
  exit 66
fi

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"
if [[ -f "${SCRIPT_DIR}/config/fastrtps_profile.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/config/fastrtps_profile.xml"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${SCRIPT_DIR}/bags/a3_body_drive_debug/${STAMP}"
RAW_DIR="${SESSION_DIR}/raw"
mkdir -p "${RAW_DIR}"

CFG_FILE="${RAW_DIR}/a3_body_drive_debug_record.${TRANSPORT}.yaml"
python3 - "${CFG_TEMPLATE}" "${CFG_FILE}" "${RAW_DIR}" <<'PY'
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
bag = Path(sys.argv[3]).resolve()
text = src.read_text(encoding="utf-8").replace("__BAG_PATH__", str(bag))
dst.write_text(text, encoding="utf-8")
PY

cat > "${SESSION_DIR}/README.txt" <<EOF
A3 body-drive debug recording
transport: ${TRANSPORT}
raw: ${RAW_DIR}
config: ${CFG_FILE}

Convert manually:
  ./run_a3_body_drive_debug_convert.sh "${RAW_DIR}"
EOF

echo "[a3-debug-record] transport=${TRANSPORT}"
echo "[a3-debug-record] raw=${RAW_DIR}"
echo "[a3-debug-record] Ctrl+C to stop"

set +e
"${SCRIPT_DIR}/a3_body_drive_debug_record" --cfg_file_path "${CFG_FILE}"
RC=$?
set -e

echo "[a3-debug-record] stopped"
echo "[a3-debug-record] raw=${RAW_DIR}"
echo "[a3-debug-record] convert with:"
echo "  ${SCRIPT_DIR}/run_a3_body_drive_debug_convert.sh \"${RAW_DIR}\""
exit "${RC}"
