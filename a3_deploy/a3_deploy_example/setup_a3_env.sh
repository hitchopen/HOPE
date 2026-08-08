# Source me (inside `hope`) before building: `source setup_a3_env.sh`
_A3_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || \
  { [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash; }
_ORT="onnxruntime-linux-x64-1.19.2"
_ORT_DIR="${_A3_DIR}/thirdparty/onnxruntime/${_ORT}"
if [ ! -f "${_ORT_DIR}/include/onnxruntime_cxx_api.h" ]; then
  ( cd "${_A3_DIR}/thirdparty/onnxruntime" &&
    { wget -c "https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/${_ORT}.tgz" ||
      curl -L -O "https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/${_ORT}.tgz"; } &&
    tar xzf "${_ORT}.tgz" )
fi
export onnxruntime_ROOT="${_ORT_DIR}"
echo "[a3-env] ready: onnxruntime_ROOT=${onnxruntime_ROOT}  ROS_DISTRO=${ROS_DISTRO:-none}"
