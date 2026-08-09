# Source me (inside `hope`) before building: `source setup_a3_env.sh`
_A3_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || \
  { [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash; }

_a3_download() {
  _a3_download_url="$1"
  _a3_download_path="$2"
  if command -v wget >/dev/null 2>&1; then
    wget -c --tries=5 --timeout=30 --retry-connrefused \
      -O "${_a3_download_path}" "${_a3_download_url}"
  elif command -v curl >/dev/null 2>&1; then
    curl --fail --location --retry 5 --retry-all-errors --continue-at - \
      --output "${_a3_download_path}" "${_a3_download_url}"
  else
    echo "[a3-env] wget or curl is required to fetch public build dependencies" >&2
    return 1
  fi
}

_UNITREE_VERSION="2.0.0"
_UNITREE_DIR="${_A3_DIR}/thirdparty/unitree_sdk2"
_UNITREE_TEMPLATE="${_A3_DIR}/../../agibot/code_deployment/a3_deploy_example/thirdparty/unitree_sdk2"
_UNITREE_ARCH="$(uname -m)"
case "${_UNITREE_ARCH}" in
  amd64) _UNITREE_ARCH="x86_64" ;;
  arm64) _UNITREE_ARCH="aarch64" ;;
  x86_64|aarch64) ;;
  *)
    echo "[a3-env] unsupported Unitree SDK architecture: ${_UNITREE_ARCH}" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac

if [ ! -f "${_UNITREE_DIR}/CMakeLists.txt" ]; then
  if [ -e "${_UNITREE_DIR}" ]; then
    echo "[a3-env] incomplete Unitree SDK source directory: ${_UNITREE_DIR}" >&2
    return 1 2>/dev/null || exit 1
  fi
  if [ ! -f "${_UNITREE_TEMPLATE}/CMakeLists.txt" ]; then
    echo "[a3-env] tracked Unitree SDK source template is missing:" >&2
    echo "[a3-env] ${_UNITREE_TEMPLATE}" >&2
    return 1 2>/dev/null || exit 1
  fi
  mkdir -p "${_UNITREE_DIR}/thirdparty"
  cp -a \
    "${_UNITREE_TEMPLATE}/CMakeLists.txt" \
    "${_UNITREE_TEMPLATE}/LICENSE" \
    "${_UNITREE_TEMPLATE}/README.md" \
    "${_UNITREE_TEMPLATE}/cmake" \
    "${_UNITREE_TEMPLATE}/include" \
    "${_UNITREE_DIR}/"
  cp -a \
    "${_UNITREE_TEMPLATE}/thirdparty/CMakeLists.txt" \
    "${_UNITREE_TEMPLATE}/thirdparty/include" \
    "${_UNITREE_DIR}/thirdparty/"
fi

_UNITREE_URL_ROOT="https://raw.githubusercontent.com/unitreerobotics"
_UNITREE_URL_ROOT="${_UNITREE_URL_ROOT}/unitree_sdk2/${_UNITREE_VERSION}"
for _UNITREE_REL in \
  "lib/${_UNITREE_ARCH}/libunitree_sdk2.a" \
  "thirdparty/lib/${_UNITREE_ARCH}/libddsc.so" \
  "thirdparty/lib/${_UNITREE_ARCH}/libddscxx.so"; do
  _UNITREE_DEST="${_UNITREE_DIR}/${_UNITREE_REL}"
  if [ ! -f "${_UNITREE_DEST}" ]; then
    mkdir -p "$(dirname "${_UNITREE_DEST}")"
    if ! _a3_download "${_UNITREE_URL_ROOT}/${_UNITREE_REL}" "${_UNITREE_DEST}.part"; then
      echo "[a3-env] failed to fetch Unitree SDK ${_UNITREE_VERSION}: ${_UNITREE_REL}" >&2
      return 1 2>/dev/null || exit 1
    fi
    mv -- "${_UNITREE_DEST}.part" "${_UNITREE_DEST}"
  fi
done
ln -sfn libddsc.so "${_UNITREE_DIR}/thirdparty/lib/${_UNITREE_ARCH}/libddsc.so.0"
ln -sfn libddscxx.so "${_UNITREE_DIR}/thirdparty/lib/${_UNITREE_ARCH}/libddscxx.so.0"

_ORT="onnxruntime-linux-x64-1.19.2"
_ORT_BASE="${_A3_DIR}/thirdparty/onnxruntime"
_ORT_DIR="${_ORT_BASE}/${_ORT}"
if [ ! -f "${_ORT_DIR}/include/onnxruntime_cxx_api.h" ]; then
  _ORT_ARCHIVE="${_ORT_BASE}/${_ORT}.tgz"
  mkdir -p "${_ORT_BASE}"
  if ! _a3_download \
      "https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/${_ORT}.tgz" \
      "${_ORT_ARCHIVE}" || ! tar -xzf "${_ORT_ARCHIVE}" -C "${_ORT_BASE}"; then
    echo "[a3-env] failed to install public ONNX Runtime 1.19.2" >&2
    return 1 2>/dev/null || exit 1
  fi
  rm -f -- "${_ORT_ARCHIVE}"
fi
export onnxruntime_ROOT="${_ORT_DIR}"
echo "[a3-env] ready: unitree_sdk2=${_UNITREE_DIR}  onnxruntime_ROOT=${onnxruntime_ROOT}  ROS_DISTRO=${ROS_DISTRO:-none}"
