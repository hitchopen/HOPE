#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/build_a3_deploy_pkg.sh --arch x86_64 [--runtime-cfg PATH]
  scripts/build_a3_deploy_pkg.sh --arch rockchip [--runtime-cfg PATH]
  scripts/build_a3_deploy_pkg.sh --arch thor [--runtime-cfg PATH]

Options:
  --arch ARCH          x86_64, rockchip, or thor.
  --runtime-cfg PATH   Source A3 runtime YAML. Defaults to the repo A3 config.
  --smpl-zmq-host HOST Override packaged smpl_zmq.host. By default x86_64 uses
                       localhost, while rockchip and thor use the fixed HDU
                       address <hdu_ip> when the source config still has a
                       localhost-style host.
  --jobs N             Parallel build jobs. Defaults to nproc.
  --inside-docker      Internal flag used by the arm64 builder containers.
  --build-only         Internal flag: compile binaries/libs but skip asset staging.

Proxy:
  Existing http_proxy/https_proxy/all_proxy environment variables are passed
  through to Docker build/run. If they point at localhost/127.0.0.1, Docker
  uses --network host so the container can reach the host proxy.

Thor sysroot:
  Thor builds require thirdparty/thor_sysroot/thor-1.0-aarch64-sysroot.tar.gz.
  Generate it once with scripts/export_thor_sysroot.sh.

Rockchip sysroot:
  Rockchip builds require
  thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz.
  Generate it once with scripts/export_rockchip_sysroot.sh.
  The x86_64 cross-builder base defaults to debian:bookworm, matching the
  original Rockchip image OS family. Override with A3_ROCKCHIP_BUILDER_BASE.
USAGE
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="${GEAR_ROOT}"

ARCH=""
RUNTIME_CFG="${GEAR_ROOT}/src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.yaml"
SMPL_ZMQ_HOST_OVERRIDE=""
INSIDE_DOCKER=0
BUILD_ONLY=0
JOBS="$(nproc)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arch)
      ARCH="${2:-}"
      shift 2
      ;;
    --runtime-cfg)
      RUNTIME_CFG="${2:-}"
      shift 2
      ;;
    --smpl-zmq-host)
      SMPL_ZMQ_HOST_OVERRIDE="${2:-}"
      shift 2
      ;;
    --jobs)
      JOBS="${2:-}"
      shift 2
      ;;
    --inside-docker)
      INSIDE_DOCKER=1
      shift
      ;;
    --build-only)
      BUILD_ONLY=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 64
      ;;
  esac
done

if [[ -z "${ARCH}" ]]; then
  echo "--arch is required" >&2
  usage >&2
  exit 64
fi
if [[ ! "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--jobs must be a positive integer; got '${JOBS}'" >&2
  exit 64
fi

case "${ARCH}" in
  x86_64)
    PACKAGE_NAME="a3_deploy_x86_64"
    BUILD_DIR="${GEAR_ROOT}/build/a3_pkg_x86_64"
    ;;
  rockchip)
    PACKAGE_NAME="a3_deploy_rockchip"
    BUILD_DIR="${GEAR_ROOT}/build/a3_pkg_rockchip"
    ;;
  thor)
    PACKAGE_NAME="a3_deploy_thor"
    BUILD_DIR="${GEAR_ROOT}/build/a3_pkg_thor"
    ;;
  *)
    echo "--arch must be one of: x86_64, rockchip, thor" >&2
    exit 64
    ;;
esac

PKG_DIR="${GEAR_ROOT}/dist/${PACKAGE_NAME}"
ROCKCHIP_SYSROOT_TARBALL_REL="${A3_ROCKCHIP_SYSROOT_TARBALL_REL:-thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz}"
ROCKCHIP_SYSROOT_TARBALL="${GEAR_ROOT}/${ROCKCHIP_SYSROOT_TARBALL_REL}"
ROCKCHIP_BUILDER_BASE="${A3_ROCKCHIP_BUILDER_BASE:-debian:bookworm}"
THOR_SYSROOT_TARBALL_REL="${A3_THOR_SYSROOT_TARBALL_REL:-thirdparty/thor_sysroot/thor-1.0-aarch64-sysroot.tar.gz}"
THOR_SYSROOT_TARBALL="${GEAR_ROOT}/${THOR_SYSROOT_TARBALL_REL}"
PROXY_ENV_NAMES=(
  http_proxy
  https_proxy
  all_proxy
  HTTP_PROXY
  HTTPS_PROXY
  ALL_PROXY
)

append_proxy_build_args() {
  local -n out_args="$1"
  local name
  for name in "${PROXY_ENV_NAMES[@]}"; do
    if [[ -n "${!name:-}" ]]; then
      out_args+=(--build-arg "${name}=${!name}")
    fi
  done
}

append_proxy_run_env_args() {
  local -n out_args="$1"
  local name
  for name in "${PROXY_ENV_NAMES[@]}"; do
    if [[ -n "${!name:-}" ]]; then
      out_args+=(-e "${name}=${!name}")
    fi
  done
}

proxy_requires_host_network() {
  local name value
  for name in "${PROXY_ENV_NAMES[@]}"; do
    value="${!name:-}"
    if [[ "${value}" =~ (^|://)(localhost|127\.0\.0\.1)(:|/|$) ]]; then
      return 0
    fi
  done
  return 1
}

is_native_arm64() {
  local machine
  machine="$(uname -m)"
  [[ "${machine}" == "aarch64" || "${machine}" == "arm64" ]]
}

clear_stale_cross_cmake_cache_if_needed() {
  local has_cross_cache=1
  local cached_version=""
  local current_version=""
  if [[ ! "${ARCH}" =~ ^(rockchip|thor)$ || ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
    return 0
  fi
  if is_native_arm64; then
    return 0
  fi
  if grep -Fq "cmake/toolchains/aarch64-linux-gnu.cmake" "${BUILD_DIR}/CMakeCache.txt"; then
    has_cross_cache=0
  elif [[ -d "${BUILD_DIR}/CMakeFiles" ]] &&
      find "${BUILD_DIR}/CMakeFiles" -path '*/CMakeCXXCompiler.cmake' -type f \
        -exec grep -Fq "aarch64-linux-gnu-g++" {} \; -print -quit | grep -q .; then
    has_cross_cache=0
  fi
  if [[ "${has_cross_cache}" -ne 0 ]]; then
    echo "clearing stale ${ARCH} CMake build dir before cross-compiling" >&2
    rm -rf "${BUILD_DIR}"
    return 0
  fi

  cached_version="$(sed -n 's/^CMAKE_CXX_COMPILER_VERSION:INTERNAL=//p' \
    "${BUILD_DIR}/CMakeCache.txt" | head -n 1)"
  current_version="$(aarch64-linux-gnu-g++ -dumpfullversion -dumpversion 2>/dev/null | head -n 1 || true)"
  if [[ -n "${cached_version}" && -n "${current_version}" &&
      "${cached_version}" != "${current_version}" ]]; then
    echo "clearing stale ${ARCH} CMake build dir after compiler version changed: ${cached_version} -> ${current_version}" >&2
    rm -rf "${BUILD_DIR}"
  fi
}

source_ros_if_available() {
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
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    export HAS_ROS2=1
  fi
}

apply_existing_aimrt_patches() {
  local aimrt_src="${BUILD_DIR}/_deps/aimrt-src"
  local patch_script="${GEAR_ROOT}/cmake/aimrt_patches/ApplyAimRTPatches.cmake"
  if [[ -f "${aimrt_src}/CMakeLists.txt" && -f "${patch_script}" ]]; then
    (
      cd "${aimrt_src}"
      cmake -P "${patch_script}"
    )
  fi
}

refresh_aimrt_source_if_stale() {
  local aimrt_src="${BUILD_DIR}/_deps/aimrt-src"
  local version_file="${aimrt_src}/VERSION"
  local events_executor_cmake="${aimrt_src}/cmake/GetEventsExecutor.cmake"
  local upstream_events_executor_url="https://github.com/irobot-ros/events-executor/archive/b2ef94e1ecee3aa3369c680343df46035998ddc0.tar.gz"
  if [[ -f "${events_executor_cmake}" ]] &&
     ! grep -Fq "${upstream_events_executor_url}" "${events_executor_cmake}"; then
    echo "refreshing AimRT FetchContent source to restore upstream GitHub URLs" >&2
    rm -rf \
      "${BUILD_DIR}/_deps/aimrt-src" \
      "${BUILD_DIR}/_deps/aimrt-build" \
      "${BUILD_DIR}/_deps/aimrt-subbuild" \
      "${BUILD_DIR}/_deps/irobot_events_executor-src" \
      "${BUILD_DIR}/_deps/irobot_events_executor-build" \
      "${BUILD_DIR}/_deps/irobot_events_executor-subbuild"
  fi
  if [[ -d "${BUILD_DIR}/_deps/irobot_events_executor-subbuild" ]] &&
     [[ ! -d "${BUILD_DIR}/_deps/irobot_events_executor-src/irobot_lock_free_events_queue" ]]; then
    echo "clearing incomplete irobot_events_executor FetchContent download" >&2
    rm -rf \
      "${BUILD_DIR}/_deps/irobot_events_executor-src" \
      "${BUILD_DIR}/_deps/irobot_events_executor-build" \
      "${BUILD_DIR}/_deps/irobot_events_executor-subbuild"
  fi
  if [[ ! -f "${version_file}" && -d "${BUILD_DIR}/_deps/aimrt-subbuild" ]]; then
    echo "clearing incomplete AimRT FetchContent download" >&2
    rm -rf "${BUILD_DIR}/_deps/aimrt-subbuild"
  fi
  if [[ -f "${version_file}" ]] && [[ "$(tr -d '[:space:]' < "${version_file}")" != "1.6.0" ]]; then
    echo "refreshing AimRT FetchContent source for v1.6.0" >&2
    rm -rf \
      "${BUILD_DIR}/_deps/aimrt-src" \
      "${BUILD_DIR}/_deps/aimrt-build" \
      "${BUILD_DIR}/_deps/aimrt-subbuild"
  fi
}

require_rockchip_sysroot_bundle() {
  if [[ -f "${ROCKCHIP_SYSROOT_TARBALL}" ]]; then
    return 0
  fi

  cat >&2 <<EOF
missing Rockchip sysroot bundle: ${ROCKCHIP_SYSROOT_TARBALL}

Generate it once from the Rockchip target image:
  scripts/export_rockchip_sysroot.sh

Or set A3_ROCKCHIP_SYSROOT_TARBALL_REL to another path relative to this repository.
EOF
  exit 66
}

prepare_rockchip_docker_context() {
  local context_dir
  context_dir="$(mktemp -d "${TMPDIR:-/tmp}/a3-rockchip-builder-context.XXXXXX")"

  mkdir -p \
    "${context_dir}/docker" \
    "${context_dir}/$(dirname "${ROCKCHIP_SYSROOT_TARBALL_REL}")"

  cp -f \
    "${GEAR_ROOT}/docker/Dockerfile.a3-rockchip-builder" \
    "${context_dir}/docker/Dockerfile.a3-rockchip-builder"

  if ! ln "${ROCKCHIP_SYSROOT_TARBALL}" \
      "${context_dir}/${ROCKCHIP_SYSROOT_TARBALL_REL}" 2>/dev/null; then
    cp -f "${ROCKCHIP_SYSROOT_TARBALL}" \
      "${context_dir}/${ROCKCHIP_SYSROOT_TARBALL_REL}"
  fi

  echo "${context_dir}"
}

require_thor_sysroot_bundle() {
  if [[ -f "${THOR_SYSROOT_TARBALL}" ]]; then
    return 0
  fi

  cat >&2 <<EOF
missing Thor sysroot bundle: ${THOR_SYSROOT_TARBALL}

Generate it once from the Thor target image:
  scripts/export_thor_sysroot.sh

Or set A3_THOR_SYSROOT_TARBALL_REL to another path relative to this repository.
EOF
  exit 66
}

prepare_thor_docker_context() {
  local context_dir
  context_dir="$(mktemp -d "${TMPDIR:-/tmp}/a3-thor-builder-context.XXXXXX")"

  mkdir -p \
    "${context_dir}/docker" \
    "${context_dir}/$(dirname "${THOR_SYSROOT_TARBALL_REL}")"

  cp -f \
    "${GEAR_ROOT}/docker/Dockerfile.a3-thor-builder" \
    "${context_dir}/docker/Dockerfile.a3-thor-builder"

  if ! ln "${THOR_SYSROOT_TARBALL}" \
      "${context_dir}/${THOR_SYSROOT_TARBALL_REL}" 2>/dev/null; then
    cp -f "${THOR_SYSROOT_TARBALL}" \
      "${context_dir}/${THOR_SYSROOT_TARBALL_REL}"
  fi

  echo "${context_dir}"
}

build_rockchip_in_docker() {
  local image="a3-rockchip-builder:1.0"
  local docker_build_args=()
  local docker_run_args=()
  local docker_context=""
  local status=0
  require_rockchip_sysroot_bundle
  docker_context="$(prepare_rockchip_docker_context)"

  append_proxy_build_args docker_build_args
  append_proxy_run_env_args docker_run_args
  docker_build_args+=(--build-arg "ROCKCHIP_BUILDER_BASE=${ROCKCHIP_BUILDER_BASE}")
  docker_build_args+=(--build-arg "ROCKCHIP_SYSROOT_TARBALL=${ROCKCHIP_SYSROOT_TARBALL_REL}")
  if proxy_requires_host_network; then
    docker_build_args+=(--network host)
    docker_run_args+=(--network host)
  fi

  docker build --platform linux/amd64 \
    "${docker_build_args[@]}" \
    -f "${docker_context}/docker/Dockerfile.a3-rockchip-builder" \
    -t "${image}" \
    "${docker_context}" || status=$?

  if [[ "${status}" -eq 0 ]]; then
    docker run --rm --platform linux/amd64 \
      "${docker_run_args[@]}" \
      --user "$(id -u):$(id -g)" \
      -e HOME=/tmp \
      -e HAS_ROS2=1 \
      -v "${REPO_ROOT}:/work" \
      -w /work \
      "${image}" \
      bash -lc "source /opt/ros/jazzy/setup.bash && scripts/build_a3_deploy_pkg.sh --arch rockchip --inside-docker --build-only --jobs '${JOBS}'" || status=$?
  fi

  rm -rf "${docker_context}"
  return "${status}"
}

build_thor_in_docker() {
  local image="a3-thor-builder:1.0"
  local docker_build_args=()
  local docker_run_args=()
  local inner_build_only_arg=""
  local docker_context=""
  local status=0
  if [[ "${BUILD_ONLY}" -eq 1 ]]; then
    inner_build_only_arg=" --build-only"
  fi
  require_thor_sysroot_bundle
  docker_context="$(prepare_thor_docker_context)"

  append_proxy_build_args docker_build_args
  append_proxy_run_env_args docker_run_args
  docker_build_args+=(--build-arg "THOR_SYSROOT_TARBALL=${THOR_SYSROOT_TARBALL_REL}")
  if proxy_requires_host_network; then
    docker_build_args+=(--network host)
    docker_run_args+=(--network host)
  fi

  docker build --platform linux/amd64 \
    "${docker_build_args[@]}" \
    -f "${docker_context}/docker/Dockerfile.a3-thor-builder" \
    -t "${image}" \
    "${docker_context}" || status=$?

  if [[ "${status}" -eq 0 ]]; then
    docker run --rm --platform linux/amd64 \
      "${docker_run_args[@]}" \
      --user "$(id -u):$(id -g)" \
      -e HOME=/tmp \
      -e HAS_ROS2=1 \
      -e HOST_REPO_ROOT="${REPO_ROOT}" \
      -v "${REPO_ROOT}:/work" \
      -w /work \
      "${image}" \
      bash -lc "source /opt/ros/jazzy/setup.bash && scripts/build_a3_deploy_pkg.sh --arch thor --inside-docker${inner_build_only_arg} --jobs '${JOBS}'" || status=$?
  fi

  rm -rf "${docker_context}"
  return "${status}"
}

cmake_target_exists() {
  local build_dir="$1"
  local target="$2"
  local help_file
  help_file="$(mktemp)"
  if ! cmake --build "${build_dir}" --target help >"${help_file}" 2>/dev/null; then
    rm -f "${help_file}"
    return 1
  fi
  if grep -Eq "(^|[[:space:]])${target}([[:space:]]|$)" "${help_file}"; then
    rm -f "${help_file}"
    return 0
  fi
  rm -f "${help_file}"
  return 1
}

configure_and_build() {
  rm -rf "${PKG_DIR}"
  mkdir -p "${PKG_DIR}"

  clear_stale_cross_cmake_cache_if_needed
  source_ros_if_available
  refresh_aimrt_source_if_stale
  apply_existing_aimrt_patches

  local enable_trt_inference="OFF"
  if [[ "${ARCH}" == "thor" ]]; then
    enable_trt_inference="ON"
  fi
  local cmake_python="${A3_CMAKE_PYTHON:-/usr/bin/python3}"

  local cmake_args=(
    -S "${GEAR_ROOT}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SRCS=ON
    -DENABLE_TRT_INFERENCE="${enable_trt_inference}"
    -DENABLE_A3_ROS_MSGS=ON
    -DENABLE_A3_AIMRT_BACKEND=ON
    -DGS_PACKAGE_ARCH_NAME="${ARCH}"
    -DGS_RUNTIME_OUTPUT_DIR="${PKG_DIR}"
  )
  if [[ -x "${cmake_python}" ]]; then
    cmake_args+=(
      -DPython3_EXECUTABLE="${cmake_python}"
      -DPYTHON_EXECUTABLE="${cmake_python}"
    )
  fi

  if [[ "${ARCH}" == "rockchip" ]]; then
    cmake_args+=(
      -DUSE_BUNDLED_ONNXRUNTIME_AARCH64=ON
      -DENABLE_RKNN_INFERENCE=ON
    )
    if is_native_arm64; then
      cmake_args+=(
        -DCMAKE_C_COMPILER=/opt/gcc-13.3/bin/gcc
        -DCMAKE_CXX_COMPILER=/opt/gcc-13.3/bin/g++
      )
    else
      cmake_args+=(
        -DCMAKE_TOOLCHAIN_FILE="${GEAR_ROOT}/cmake/toolchains/aarch64-linux-gnu.cmake"
        -DGS_SKIP_ROSIDL_GENERATOR_PY=ON
      )
    fi
  elif [[ "${ARCH}" == "thor" ]]; then
    cmake_args+=(
      -DUSE_BUNDLED_ONNXRUNTIME_AARCH64=ON
      -DTensorRT_ROOT=/usr
      -DCUDAToolkit_ROOT=/usr/local/cuda
    )
    if ! is_native_arm64; then
      cmake_args+=(
        -DCMAKE_TOOLCHAIN_FILE="${GEAR_ROOT}/cmake/toolchains/aarch64-linux-gnu.cmake"
      )
    fi
  fi

  cmake "${cmake_args[@]}"
  cmake --build "${BUILD_DIR}" --target a3_deploy_onnx_ref -j"${JOBS}"
  cmake --build "${BUILD_DIR}" --target a3_policy_runtime_probe -j"${JOBS}"
  if cmake_target_exists "${BUILD_DIR}" "a3_body_drive_debug_record"; then
    cmake --build "${BUILD_DIR}" --target a3_body_drive_debug_record -j"${JOBS}"
  else
    echo "missing CMake target: a3_body_drive_debug_record" >&2
    exit 1
  fi
  if cmake_target_exists "${BUILD_DIR}" "joint_msgs_s__rosidl_typesupport_c"; then
    cmake --build "${BUILD_DIR}" --target joint_msgs_s__rosidl_typesupport_c -j"${JOBS}"
  fi
}

copy_found_libs() {
  local search_dir="$1"
  local pattern="$2"
  [[ -d "${search_dir}" ]] || return 0
  while IFS= read -r -d '' lib; do
    local dst="${PKG_DIR}/$(basename "${lib}")"
    rm -f "${dst}"
    cp -a "${lib}" "${dst}"
  done < <(find "${search_dir}" \( -type f -o -type l \) -name "${pattern}" -print0 2>/dev/null)
}

stage_runtime_config_and_assets() {
  python3 - "${REPO_ROOT}" "${GEAR_ROOT}" "${RUNTIME_CFG}" "${PKG_DIR}" "${ARCH}" "${SMPL_ZMQ_HOST_OVERRIDE}" <<'PY'
import shutil
import sys
import os
from pathlib import Path

import yaml

repo_root = Path(sys.argv[1]).resolve()
gear_root = Path(sys.argv[2]).resolve()
runtime_cfg = Path(sys.argv[3]).resolve()
pkg_dir = Path(sys.argv[4]).resolve()
arch = sys.argv[5]
smpl_zmq_host_override = sys.argv[6]
host_repo_root_raw = os.environ.get("HOST_REPO_ROOT", "")
host_repo_root = Path(host_repo_root_raw).expanduser() if host_repo_root_raw else None

if not runtime_cfg.exists():
    raise SystemExit(f"runtime config does not exist: {runtime_cfg}")

with runtime_cfg.open("r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}

def nested(mapping, path):
    cur = mapping
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur

def set_nested(mapping, path, value):
    cur = mapping
    for key in path[:-1]:
        cur = cur.setdefault(key, {})
    cur[path[-1]] = value

def is_localhost(value):
    return str(value or "").strip() in ("", "localhost", "127.0.0.1", "::1")

def path_relative_to(path, root):
    if root is None:
        return None
    try:
        return path.relative_to(root)
    except ValueError:
        return None

def existing_repo_suffix_candidate(path):
    parts = path.parts
    for idx in range(1, len(parts) - 1):
        candidate = repo_root.joinpath(*parts[idx + 1:])
        if candidate.exists():
            return candidate
    return None

def resolve_path(raw, label, required=True):
    if raw is None:
        if required:
            raise SystemExit(f"missing required path: {label}")
        return None
    raw = str(raw)
    p = Path(raw).expanduser()
    if p.is_absolute():
        candidates = [p]
        host_rel = path_relative_to(p, host_repo_root)
        if host_rel is not None:
            candidates.append(repo_root / host_rel)
        repo_suffix_candidate = existing_repo_suffix_candidate(p)
        if repo_suffix_candidate is not None and repo_suffix_candidate not in candidates:
            candidates.append(repo_suffix_candidate)
    else:
        candidates = [
            repo_root / p,
            gear_root / p,
            runtime_cfg.parent / p,
        ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    if required:
        tried = ", ".join(str(c) for c in candidates)
        raise SystemExit(f"{label} does not exist; tried: {tried}")
    return None

def copy_into(src, rel_dir, dst_name=None):
    dst_dir = pkg_dir / rel_dir
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / (dst_name or src.name)
    shutil.copy2(src, dst)
    return str(Path(rel_dir) / dst.name)

def unique_existing(paths):
    seen = set()
    out = []
    for path in paths:
        if path is None:
            continue
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        if path.exists():
            out.append(path.resolve())
    return out

def auto_rknn_candidates(onnx_src):
    if onnx_src is None:
        return []
    rknn_name = onnx_src.with_suffix(".rknn").name
    return unique_existing([
        gear_root / "assets" / "a3_runtime" / "rknn_models" / rknn_name,
        repo_root / "assets" / "a3_runtime" / "rknn_models" / rknn_name,
        onnx_src.with_suffix(".rknn"),
        onnx_src.parent / "rknn_models" / rknn_name,
    ])

def resolve_rknn_model(raw, label, onnx_src, required=False):
    explicit = resolve_path(raw, label, required=False)
    if explicit is not None:
        return explicit
    candidates = auto_rknn_candidates(onnx_src)
    if candidates:
        print(f"auto-selected {label}: {candidates[0]}")
        return candidates[0]
    if required:
        rknn_name = onnx_src.with_suffix(".rknn").name if onnx_src is not None else "<model>.rknn"
        raise SystemExit(
            f"missing {label}; expected {rknn_name} under "
            "assets/a3_runtime/rknn_models/ or next to the ONNX. "
            "Run scripts/convert_a3_onnx_to_rknn.py first."
        )
    return None

def copy_motion_dir_csvs(selected_motion):
    motion_dir = selected_motion.parent
    copied = []
    for src in sorted(motion_dir.glob("*.csv")):
        copied.append(copy_into(src, "motions"))
    if str(Path("motions") / selected_motion.name) not in copied:
        copied.append(copy_into(selected_motion, "motions"))
    return copied

def copy_motion_folder_csvs(motion_dir, rel_dir="motions"):
    copied = []
    for src in sorted(motion_dir.glob("*.csv")):
        copied.append(copy_into(src, rel_dir))
    return copied

pkg_dir.mkdir(parents=True, exist_ok=True)
(pkg_dir / "config").mkdir(parents=True, exist_ok=True)

onnx_cfg = nested(cfg, ["onnx"]) or {}
onnx_mode = str(onnx_cfg.get("mode") or "monolithic")
onnx_mode_norm = onnx_mode.strip().lower().replace("-", "_")
encoder_decoder_mode = onnx_mode_norm in ("encoder_decoder", "encoderdecoder", "split")
onnx_backend_norm = str(onnx_cfg.get("backend") or "ort_cpu").strip().lower().replace("-", "_")
explicit_rknn_backend = onnx_backend_norm in ("rknn", "rk_npu", "rockchip_npu")
rockchip_auto_rknn_backend = (
    arch == "rockchip"
    and not encoder_decoder_mode
    and onnx_backend_norm in ("", "auto", "ort_cpu", "cpu", "ort", "onnxruntime", "onnxruntime_cpu")
)
rknn_backend = explicit_rknn_backend or rockchip_auto_rknn_backend
if rknn_backend and encoder_decoder_mode:
    raise SystemExit("onnx.backend=rknn does not support onnx.mode=encoder_decoder")
model_src = resolve_path(
    nested(cfg, ["onnx", "model_path"]),
    "onnx.model_path",
    required=(not encoder_decoder_mode),
)
smpl_model_src = resolve_path(
    nested(cfg, ["onnx", "smpl_model_path"]) if not encoder_decoder_mode else None,
    "onnx.smpl_model_path",
    required=False,
)
a3_fast_model_src = resolve_path(
    nested(cfg, ["onnx", "a3_fast_model_path"]) if not encoder_decoder_mode else None,
    "onnx.a3_fast_model_path",
    required=False,
)
rknn_model_src = resolve_path(
    nested(cfg, ["onnx", "rknn_model_path"]) if not encoder_decoder_mode else None,
    "onnx.rknn_model_path",
    required=False,
)
smpl_rknn_model_src = resolve_path(
    nested(cfg, ["onnx", "smpl_rknn_model_path"]) if not encoder_decoder_mode else None,
    "onnx.smpl_rknn_model_path",
    required=False,
)
a3_fast_rknn_model_src = resolve_path(
    nested(cfg, ["onnx", "a3_fast_rknn_model_path"]) if not encoder_decoder_mode else None,
    "onnx.a3_fast_rknn_model_path",
    required=False,
)
if rknn_backend and not encoder_decoder_mode:
    rknn_model_src = resolve_rknn_model(
        nested(cfg, ["onnx", "rknn_model_path"]),
        "onnx.rknn_model_path",
        model_src,
        required=True,
    )
    if smpl_model_src is not None:
        smpl_rknn_model_src = resolve_rknn_model(
            nested(cfg, ["onnx", "smpl_rknn_model_path"]),
            "onnx.smpl_rknn_model_path",
            smpl_model_src,
            required=True,
        )
    if a3_fast_model_src is not None:
        a3_fast_rknn_model_src = resolve_rknn_model(
            nested(cfg, ["onnx", "a3_fast_rknn_model_path"]),
            "onnx.a3_fast_rknn_model_path",
            a3_fast_model_src,
            required=True,
        )
encoder_model_src = resolve_path(
    nested(cfg, ["onnx", "encoder_model_path"]) if encoder_decoder_mode else None,
    "onnx.encoder_model_path",
    required=encoder_decoder_mode,
)
decoder_model_src = resolve_path(
    nested(cfg, ["onnx", "decoder_model_path"]) if encoder_decoder_mode else None,
    "onnx.decoder_model_path",
    required=encoder_decoder_mode,
)
motion_dir_src = resolve_path(
    nested(cfg, ["reference_motion", "motion_dir"]),
    "reference_motion.motion_dir",
    required=False,
)
remote_motion_dir_srcs = []
remote_motion_dir_raw = nested(cfg, ["reference_motion", "remote_motion_dir"])
if remote_motion_dir_raw is not None:
    remote_motion_dir_srcs.append(resolve_path(
        remote_motion_dir_raw,
        "reference_motion.remote_motion_dir",
        required=True,
    ))
remote_motion_dirs_raw = nested(cfg, ["reference_motion", "remote_motion_dirs"])
if remote_motion_dirs_raw is not None:
    if not isinstance(remote_motion_dirs_raw, list):
        raise SystemExit("reference_motion.remote_motion_dirs must be a list")
    for i, raw_remote_dir in enumerate(remote_motion_dirs_raw):
        remote_motion_dir_srcs.append(resolve_path(
            raw_remote_dir,
            f"reference_motion.remote_motion_dirs[{i}]",
            required=True,
        ))
legacy_extra_motion_dirs_raw = nested(cfg, ["reference_motion", "extra_motion_dirs"])
if legacy_extra_motion_dirs_raw is not None:
    if not isinstance(legacy_extra_motion_dirs_raw, list):
        raise SystemExit("reference_motion.extra_motion_dirs must be a list")
    for i, raw_extra_dir in enumerate(legacy_extra_motion_dirs_raw):
        remote_motion_dir_srcs.append(resolve_path(
            raw_extra_dir,
            f"reference_motion.extra_motion_dirs[{i}]",
            required=True,
        ))
motion_src = None
used_motion_dir = False
motion_idle_src = resolve_path(
    nested(cfg, ["reference_motion", "idle_csv_path"]),
    "reference_motion.idle_csv_path",
    required=False,
)
teleop_fallback_src = None
teleop_fallback_cfg = nested(cfg, ["teleop", "fallback_reference"])
if isinstance(teleop_fallback_cfg, dict):
    teleop_fallback_enabled = bool(teleop_fallback_cfg.get("enabled", False))
    teleop_fallback_raw = teleop_fallback_cfg.get("csv_path")
    if teleop_fallback_enabled:
        teleop_fallback_src = resolve_path(
            teleop_fallback_raw,
            "teleop.fallback_reference.csv_path",
        )
    elif teleop_fallback_raw is not None:
        teleop_fallback_src = resolve_path(
            teleop_fallback_raw,
            "teleop.fallback_reference.csv_path",
            required=False,
        )
aimrt_src = resolve_path(
    nested(cfg, ["backend", "aimrt_cfg_path"]),
    "backend.aimrt_cfg_path",
)

if model_src is not None:
    set_nested(cfg, ["onnx", "model_path"], copy_into(model_src, "models"))
if smpl_model_src is not None:
    set_nested(cfg, ["onnx", "smpl_model_path"], copy_into(smpl_model_src, "models"))
if a3_fast_model_src is not None:
    set_nested(cfg, ["onnx", "a3_fast_model_path"], copy_into(a3_fast_model_src, "models"))
if rknn_model_src is not None:
    set_nested(cfg, ["onnx", "rknn_model_path"], copy_into(rknn_model_src, "models"))
if smpl_rknn_model_src is not None:
    set_nested(cfg, ["onnx", "smpl_rknn_model_path"], copy_into(smpl_rknn_model_src, "models"))
if a3_fast_rknn_model_src is not None:
    set_nested(cfg, ["onnx", "a3_fast_rknn_model_path"], copy_into(a3_fast_rknn_model_src, "models"))
if rknn_backend:
    set_nested(cfg, ["onnx", "backend"], "rknn")
if encoder_model_src is not None:
    set_nested(cfg, ["onnx", "encoder_model_path"], copy_into(encoder_model_src, "models"))
if decoder_model_src is not None:
    set_nested(cfg, ["onnx", "decoder_model_path"], copy_into(decoder_model_src, "models"))
if motion_dir_src is not None:
    if not motion_dir_src.is_dir():
        raise SystemExit(f"reference_motion.motion_dir is not a directory: {motion_dir_src}")
    motion_rel_paths = copy_motion_folder_csvs(motion_dir_src)
    if motion_rel_paths:
        used_motion_dir = True
        initial_index = int(nested(cfg, ["reference_motion", "initial_index"]) or 0)
        if initial_index < 0 or initial_index >= len(motion_rel_paths):
            raise SystemExit(
                f"reference_motion.initial_index={initial_index} out of range "
                f"for {len(motion_rel_paths)} packaged motions"
            )
        set_nested(cfg, ["reference_motion", "motion_dir"], "motions")
        set_nested(cfg, ["reference_motion", "csv_path"], motion_rel_paths[initial_index])
    else:
        print(f"reference_motion.motion_dir has no CSV files; falling back to csv_path: {motion_dir_src}")
        cfg.get("reference_motion", {}).pop("motion_dir", None)
        motion_src = resolve_path(
            nested(cfg, ["reference_motion", "csv_path"]),
            "reference_motion.csv_path",
        )
        motion_rel_paths = copy_motion_dir_csvs(motion_src)
        set_nested(cfg, ["reference_motion", "csv_path"], str(Path("motions") / motion_src.name))
else:
    motion_src = resolve_path(
        nested(cfg, ["reference_motion", "csv_path"]),
        "reference_motion.csv_path",
    )
    motion_rel_paths = copy_motion_dir_csvs(motion_src)
    set_nested(cfg, ["reference_motion", "csv_path"], str(Path("motions") / motion_src.name))

remote_motion_rel_dirs = []
for i, remote_motion_dir_src in enumerate(remote_motion_dir_srcs):
    if not remote_motion_dir_src.is_dir():
        raise SystemExit(f"reference_motion.remote_motion_dirs[{i}] is not a directory: {remote_motion_dir_src}")
    remote_rel_dir = "remote_motions" if i == 0 else f"remote_motions_{i + 1}"
    remote_rel_paths = copy_motion_folder_csvs(remote_motion_dir_src, remote_rel_dir)
    if remote_rel_paths:
        remote_motion_rel_dirs.append(remote_rel_dir)
    else:
        print(f"reference_motion.remote_motion_dirs[{i}] has no CSV files: {remote_motion_dir_src}")
if remote_motion_dir_srcs:
    ref_cfg = cfg.get("reference_motion", {})
    ref_cfg.pop("extra_motion_dirs", None)
    ref_cfg.pop("remote_motion_dir", None)
    ref_cfg.pop("remote_motion_dirs", None)
    if len(remote_motion_rel_dirs) == 1:
        set_nested(cfg, ["reference_motion", "remote_motion_dir"], remote_motion_rel_dirs[0])
    elif len(remote_motion_rel_dirs) > 1:
        set_nested(cfg, ["reference_motion", "remote_motion_dirs"], remote_motion_rel_dirs)

if teleop_fallback_src is not None:
    teleop_fallback_rel_dir = "teleop_motions" if used_motion_dir else "motions"
    set_nested(
        cfg,
        ["teleop", "fallback_reference", "csv_path"],
        copy_into(teleop_fallback_src, teleop_fallback_rel_dir),
    )
if motion_idle_src is not None:
    set_nested(
        cfg,
        ["reference_motion", "idle_csv_path"],
        copy_into(motion_idle_src, "teleop_motions"),
    )
set_nested(cfg, ["backend", "aimrt_cfg_path"], copy_into(aimrt_src, "config", "a3_aimrt_config.yaml"))
for aimrt_name in ("a3_aimrt_config.iceoryx.yaml", "a3_aimrt_config.ros2.yaml"):
    aimrt_variant = aimrt_src.parent / aimrt_name
    if aimrt_variant.exists():
        copy_into(aimrt_variant, "config", aimrt_name)

if nested(cfg, ["logging", "dump_path"]) is not None:
    set_nested(cfg, ["logging", "dump_path"], "logs/" + Path(str(nested(cfg, ["logging", "dump_path"]))).name)

if arch == "thor":
    set_nested(cfg, ["onnx", "backend"], "trt")
    set_nested(cfg, ["logging", "artificial_infer_delay_ms"], 0.0)
    set_nested(cfg, ["logging", "artificial_infer_delay_jitter_ms"], 0.0)

smpl_zmq_cfg = nested(cfg, ["smpl_zmq"])
if isinstance(smpl_zmq_cfg, dict):
    if smpl_zmq_host_override:
        set_nested(cfg, ["smpl_zmq", "host"], smpl_zmq_host_override)
        print(f"packaged smpl_zmq.host override: {smpl_zmq_host_override}")
    elif arch in ("rockchip", "thor") and is_localhost(smpl_zmq_cfg.get("host")):
        # Arm deploy targets connect to the Pico sender on the HDU via the
        # robot's fixed internal HDU address.
        set_nested(cfg, ["smpl_zmq", "host"], "<hdu_ip>")
        print(f"packaged smpl_zmq.host default for {arch}: <hdu_ip>")
    elif arch == "x86_64" and is_localhost(smpl_zmq_cfg.get("host")):
        set_nested(cfg, ["smpl_zmq", "host"], "localhost")

with (pkg_dir / "config" / "a3_runtime_config.yaml").open("w", encoding="utf-8") as f:
    yaml.safe_dump(cfg, f, sort_keys=False)

if model_src is not None:
    print(f"packaged model: {model_src}")
if smpl_model_src is not None:
    print(f"packaged smpl model: {smpl_model_src}")
if a3_fast_model_src is not None:
    print(f"packaged a3_fast model: {a3_fast_model_src}")
if rknn_model_src is not None:
    print(f"packaged RKNN model: {rknn_model_src}")
if smpl_rknn_model_src is not None:
    print(f"packaged SMPL RKNN model: {smpl_rknn_model_src}")
if a3_fast_rknn_model_src is not None:
    print(f"packaged A3-fast RKNN model: {a3_fast_rknn_model_src}")
if encoder_model_src is not None:
    print(f"packaged encoder model: {encoder_model_src}")
if decoder_model_src is not None:
    print(f"packaged decoder model: {decoder_model_src}")
if used_motion_dir:
    print(f"packaged motion dir: {motion_dir_src} ({len(motion_rel_paths)} csv files)")
else:
    print(f"packaged motion: {motion_src}")
    print(f"packaged motion dir: {motion_src.parent} ({len(motion_rel_paths)} csv files)")
for src, rel_dir in zip(remote_motion_dir_srcs, remote_motion_rel_dirs):
    print(f"packaged remote motion dir: {src} -> {rel_dir}")
if teleop_fallback_src is not None:
    print(f"packaged teleop fallback motion: {teleop_fallback_src}")
print(f"packaged aimrt cfg: {aimrt_src}")
PY

  local fastrtps_src="${GEAR_ROOT}/src/a3/a3_deploy_onnx_ref/config/fastrtps_profile.xml"
  if [[ -f "${fastrtps_src}" ]]; then
    cp -f "${fastrtps_src}" "${PKG_DIR}/config/fastrtps_profile.xml"
  fi
}

stage_body_drive_debug_files() {
  local debug_src="${GEAR_ROOT}/src/a3/a3_deploy_onnx_ref"

  mkdir -p \
    "${PKG_DIR}/config" \
    "${PKG_DIR}/tools"

  cp -f \
    "${debug_src}/config/a3_body_drive_debug_record.iceoryx.yaml" \
    "${debug_src}/config/a3_body_drive_debug_record.ros2.yaml" \
    "${debug_src}/config/a3_body_drive_debug.layout.json" \
    "${PKG_DIR}/config/"

  install -m 0755 \
    "${debug_src}/scripts/run_a3_body_drive_debug_record.sh" \
    "${debug_src}/scripts/run_a3_body_drive_debug_convert.sh" \
    "${PKG_DIR}/"

  install -m 0755 \
    "${debug_src}/scripts/tools/a3_body_drive_debug_convert.py" \
    "${debug_src}/scripts/tools/serve_foxglove_assets.py" \
    "${PKG_DIR}/tools/"
}

stage_run_scripts() {
  local default_transport="iceoryx"
  if [[ "${ARCH}" == "thor" ]]; then
    default_transport="ros2"
  fi
  local source_robot_env_default="0"
  if [[ "${ARCH}" == "rockchip" || "${ARCH}" == "thor" ]]; then
    source_robot_env_default="1"
  fi

  cat > "${PKG_DIR}/run_a3.sh" <<'RUN_A3'
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

A3_SOURCE_ROBOT_ENV_DEFAULT="__A3_SOURCE_ROBOT_ENV_DEFAULT__"
A3_ROBOT_ENV="${A3_ROBOT_ENV:-/agibot/software/v0/entry/env/env.sh}"
A3_SOURCE_ROBOT_ENV_ENABLED="${A3_SOURCE_ROBOT_ENV:-${A3_SOURCE_ROBOT_ENV_DEFAULT}}"
if [[ "${A3_SOURCE_ROBOT_ENV_ENABLED}" != "0" ]]; then
  if [[ ! -f "${A3_ROBOT_ENV}" ]]; then
    echo "required robot env not found: ${A3_ROBOT_ENV} (set A3_SOURCE_ROBOT_ENV=0 to skip)" >&2
    exit 66
  fi
  set +u
  # shellcheck disable=SC1090
  source "${A3_ROBOT_ENV}"
  set -u
  echo "[a3] sourced robot env: ${A3_ROBOT_ENV}"
fi

DEFAULT_A3_TRANSPORT="__DEFAULT_A3_TRANSPORT__"
A3_TRANSPORT="${A3_TRANSPORT:-${DEFAULT_A3_TRANSPORT}}"
case "${A3_TRANSPORT}" in
  iceoryx)
    A3_AIMRT_CFG="${SCRIPT_DIR}/config/a3_aimrt_config.iceoryx.yaml"
    ;;
  ros2)
    A3_AIMRT_CFG="${SCRIPT_DIR}/config/a3_aimrt_config.ros2.yaml"
    ;;
  *)
    echo "invalid A3_TRANSPORT='${A3_TRANSPORT}'; expected one of: iceoryx, ros2" >&2
    exit 64
    ;;
esac
if [[ ! -f "${A3_AIMRT_CFG}" ]]; then
  echo "selected AimRT config does not exist: ${A3_AIMRT_CFG}" >&2
  exit 66
fi

mkdir -p logs
export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/config/fastrtps_profile.xml"

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

echo "[a3] transport=${A3_TRANSPORT} aimrt_cfg=${A3_AIMRT_CFG}"
exec "${SCRIPT_DIR}/a3_deploy_onnx_ref" \
  --runtime-cfg="${SCRIPT_DIR}/config/a3_runtime_config.yaml" \
  --aimrt-cfg="${A3_AIMRT_CFG}" \
  "$@"
RUN_A3
  sed -i "s/__DEFAULT_A3_TRANSPORT__/${default_transport}/g" "${PKG_DIR}/run_a3.sh"
  sed -i "s/__A3_SOURCE_ROBOT_ENV_DEFAULT__/${source_robot_env_default}/g" "${PKG_DIR}/run_a3.sh"

  cat > "${PKG_DIR}/run_a3_probe.sh" <<'RUN_A3_PROBE'
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export A3_LATENCY_LOG="${A3_LATENCY_LOG:-${A3_PROBE_LATENCY_LOG:-verbose}}"
exec "${SCRIPT_DIR}/run_a3.sh" \
  --probe \
  --probe-source "${A3_PROBE_SOURCE:-both}" \
  --frame-log-interval "${A3_FRAME_LOG_INTERVAL:-50}" \
  "$@"
RUN_A3_PROBE

  chmod +x "${PKG_DIR}/run_a3.sh" "${PKG_DIR}/run_a3_probe.sh"
}

stage_extra_libs() {
  copy_found_libs "${BUILD_DIR}" "libtbb.so*"
  copy_found_libs "${BUILD_DIR}" "libirobot_events_executor.so*"
  copy_found_libs "${BUILD_DIR}" "libmcap.so*"
  copy_found_libs "${BUILD_DIR}" "libaimrt_record_playback_plugin.so*"
  copy_found_libs "${BUILD_DIR}" "liba3_body_drive_debug_ros2_ts.so*"

  if [[ "${ARCH}" == "rockchip" || "${ARCH}" == "thor" ]]; then
    copy_found_libs "${GEAR_ROOT}/thirdparty/unitree_sdk2/thirdparty/lib/aarch64" "libddsc.so*"
    copy_found_libs "${GEAR_ROOT}/thirdparty/unitree_sdk2/thirdparty/lib/aarch64" "libddscxx.so*"
  else
    copy_found_libs "${GEAR_ROOT}/thirdparty/unitree_sdk2/thirdparty/lib/x86_64" "libddsc.so*"
    copy_found_libs "${GEAR_ROOT}/thirdparty/unitree_sdk2/thirdparty/lib/x86_64" "libddscxx.so*"
  fi

  if [[ "${ARCH}" == "rockchip" ]]; then
    copy_found_libs "${GEAR_ROOT}/thirdparty/rknn_runtime/2.3.2/lib/aarch64" "librknnrt.so*"
  fi

  if [[ "${ARCH}" == "thor" ]]; then
    copy_found_libs "/usr/lib/aarch64-linux-gnu" "libnvinfer*.so*"
    copy_found_libs "/usr/lib/aarch64-linux-gnu" "libnvonnxparser*.so*"
    copy_found_libs "/usr/local/cuda/targets/aarch64-linux/lib" "libcudart.so*"
  fi
}

stage_ros2_plugin_proto_prefix() {
  # ros2 bag play needs ros2_plugin_proto/msg/RosMsgWrapper type support to
  # replay AimRT protobuf wrapper topics. The deploy executable can load the
  # copied libraries from the package root, but ROS2 CLI tools discover message
  # packages through an ament prefix with resource_index + lib/.
  local ament_src="${BUILD_DIR}/ament_cmake_index/share/ament_index/resource_index"
  if [[ ! -d "${ament_src}" ]]; then
    return 0
  fi

  local copied_index=0
  while IFS= read -r -d '' resource; do
    local rel="${resource#${ament_src}/}"
    mkdir -p "${PKG_DIR}/share/ament_index/resource_index/$(dirname "${rel}")"
    cp -f "${resource}" "${PKG_DIR}/share/ament_index/resource_index/${rel}"
    copied_index=1
  done < <(find "${ament_src}" -type f -name ros2_plugin_proto -print0 2>/dev/null)

  if [[ "${copied_index}" -eq 1 ]]; then
    mkdir -p "${PKG_DIR}/lib"
    find "${PKG_DIR}" -maxdepth 1 -type f -name 'libros2_plugin_proto__*.so' \
      -exec cp -f {} "${PKG_DIR}/lib/" \;
  fi
}

stage_joint_msgs_ros2_cli_overlay() {
  # The deploy binary only needs the copied C/C++ type-support .so files in the
  # package root. ROS2 CLI tools (`ros2 topic hz/echo`) additionally import the
  # Python message package, so stage a tiny source-able overlay for joint_msgs.
  local joint_build="${BUILD_DIR}/src/a3/a3_deploy_onnx_ref/joint_msgs_build"
  local joint_py_src="${joint_build}/rosidl_generator_py/joint_msgs"
  local joint_py_ext="${joint_py_src}/joint_msgs_s__rosidl_typesupport_c.so"
  local joint_py_lib="${joint_build}/libjoint_msgs__rosidl_generator_py.so"
  if [[ ! -d "${joint_py_src}" || ! -f "${joint_py_ext}" || ! -f "${joint_py_lib}" ]]; then
    return 0
  fi

  local py_site="${PKG_DIR}/lib/python/site-packages"
  mkdir -p "${py_site}"
  rm -rf "${py_site}/joint_msgs"
  cp -a "${joint_py_src}" "${py_site}/joint_msgs"
  cp -f "${joint_py_lib}" "${py_site}/joint_msgs/"

  local ament_src="${BUILD_DIR}/ament_cmake_index/share/ament_index/resource_index"
  if [[ -d "${ament_src}" ]]; then
    while IFS= read -r -d '' resource; do
      local rel="${resource#${ament_src}/}"
      mkdir -p "${PKG_DIR}/share/ament_index/resource_index/$(dirname "${rel}")"
      cp -f "${resource}" "${PKG_DIR}/share/ament_index/resource_index/${rel}"
    done < <(find "${ament_src}" -type f -name joint_msgs -print0 2>/dev/null)
  fi

  mkdir -p "${PKG_DIR}/share/joint_msgs"
  cp -f "${GEAR_ROOT}/thirdparty/joint_msgs/package.xml" \
    "${PKG_DIR}/share/joint_msgs/package.xml"
  rm -rf "${PKG_DIR}/share/joint_msgs/msg"
  cp -a "${GEAR_ROOT}/thirdparty/joint_msgs/msg" \
    "${PKG_DIR}/share/joint_msgs/msg"

cat > "${PKG_DIR}/setup_ros2_msgs.bash" <<'SETUP_ROS2_MSGS'
#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

_a3_source_ros_setup() {
  local setup_file="$1"
  local had_nounset=0
  case "$-" in
    *u*)
      had_nounset=1
      set +u
      ;;
  esac
  # shellcheck disable=SC1090
  source "${setup_file}"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    _a3_source_ros_setup /opt/ros/jazzy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    _a3_source_ros_setup /opt/ros/humble/setup.bash
  fi
elif [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  _a3_source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

PY_SITE_DIR=""
for candidate in "${SCRIPT_DIR}"/lib/python*/site-packages; do
  if [[ -d "${candidate}/joint_msgs" ]]; then
    PY_SITE_DIR="${candidate}"
    break
  fi
done

if [[ -n "${PY_SITE_DIR}" ]]; then
  export PYTHONPATH="${PY_SITE_DIR}:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="${PY_SITE_DIR}/joint_msgs:${LD_LIBRARY_PATH:-}"
fi

export AMENT_PREFIX_PATH="${SCRIPT_DIR}:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"

unset -f _a3_source_ros_setup
SETUP_ROS2_MSGS
  chmod +x "${PKG_DIR}/setup_ros2_msgs.bash"
}

verify_package() {
  if [[ ! -x "${PKG_DIR}/a3_deploy_onnx_ref" ]]; then
    echo "missing executable: ${PKG_DIR}/a3_deploy_onnx_ref" >&2
    exit 1
  fi
  if [[ ! -x "${PKG_DIR}/a3_policy_runtime_probe" ]]; then
    echo "missing executable: ${PKG_DIR}/a3_policy_runtime_probe" >&2
    exit 1
  fi
  if [[ "${ARCH}" == "rockchip" && ! -e "${PKG_DIR}/librknnrt.so" ]]; then
    echo "missing Rockchip RKNN runtime library: librknnrt.so" >&2
    exit 1
  fi
  local debug_required=(
    "a3_body_drive_debug_record"
    "liba3_body_drive_debug_ros2_ts.so"
    "libaimrt_record_playback_plugin.so"
    "libaimrt_iceoryx_plugin.so"
    "libaimrt_ros2_plugin.so"
    "config/a3_body_drive_debug_record.iceoryx.yaml"
    "config/a3_body_drive_debug_record.ros2.yaml"
    "config/a3_body_drive_debug.layout.json"
    "run_a3_body_drive_debug_record.sh"
    "run_a3_body_drive_debug_convert.sh"
    "tools/a3_body_drive_debug_convert.py"
    "tools/serve_foxglove_assets.py"
  )
  local rel
  for rel in "${debug_required[@]}"; do
    if [[ ! -e "${PKG_DIR}/${rel}" ]]; then
      echo "missing A3 body-drive debug package file: ${rel}" >&2
      exit 1
    fi
  done
  for rel in \
      "a3_body_drive_debug_record" \
      "run_a3_body_drive_debug_record.sh" \
      "run_a3_body_drive_debug_convert.sh" \
      "tools/a3_body_drive_debug_convert.py" \
      "tools/serve_foxglove_assets.py"; do
    if [[ ! -x "${PKG_DIR}/${rel}" ]]; then
      echo "A3 body-drive debug file is not executable: ${rel}" >&2
      exit 1
    fi
  done
  python3 - "${PKG_DIR}/config/a3_runtime_config.yaml" "${ARCH}" <<'PY'
import os
import sys
from pathlib import Path

import yaml

cfg_path = Path(sys.argv[1])
arch = sys.argv[2]
cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
onnx_cfg = cfg.get("onnx", {})
onnx_mode = str(onnx_cfg.get("mode") or "monolithic")
onnx_mode_norm = onnx_mode.strip().lower().replace("-", "_")
encoder_decoder_mode = onnx_mode_norm in ("encoder_decoder", "encoderdecoder", "split")
onnx_backend_norm = str(onnx_cfg.get("backend") or "ort_cpu").strip().lower().replace("-", "_")
rknn_backend = onnx_backend_norm in ("rknn", "rk_npu", "rockchip_npu")
if rknn_backend and encoder_decoder_mode:
    raise SystemExit("onnx.backend=rknn does not support onnx.mode=encoder_decoder")
checks = [("backend.aimrt_cfg_path", cfg.get("backend", {}).get("aimrt_cfg_path"))]
if onnx_cfg.get("model_path"):
    checks.append(("onnx.model_path", onnx_cfg.get("model_path")))
elif not encoder_decoder_mode:
    raise SystemExit("missing packaged config value: onnx.model_path")
if onnx_cfg.get("smpl_model_path"):
    checks.append(("onnx.smpl_model_path", onnx_cfg.get("smpl_model_path")))
if onnx_cfg.get("a3_fast_model_path"):
    checks.append(("onnx.a3_fast_model_path", onnx_cfg.get("a3_fast_model_path")))
if onnx_cfg.get("rknn_model_path"):
    checks.append(("onnx.rknn_model_path", onnx_cfg.get("rknn_model_path")))
elif rknn_backend:
    raise SystemExit("missing packaged config value: onnx.rknn_model_path")
if onnx_cfg.get("smpl_rknn_model_path"):
    checks.append(("onnx.smpl_rknn_model_path", onnx_cfg.get("smpl_rknn_model_path")))
if onnx_cfg.get("a3_fast_rknn_model_path"):
    checks.append(("onnx.a3_fast_rknn_model_path", onnx_cfg.get("a3_fast_rknn_model_path")))
if encoder_decoder_mode:
    checks.extend([
        ("onnx.encoder_model_path", onnx_cfg.get("encoder_model_path")),
        ("onnx.decoder_model_path", onnx_cfg.get("decoder_model_path")),
    ])
ref_motion = cfg.get("reference_motion", {})
if ref_motion.get("motion_dir"):
    checks.append(("reference_motion.motion_dir", ref_motion.get("motion_dir")))
else:
    checks.append(("reference_motion.csv_path", ref_motion.get("csv_path")))
if ref_motion.get("remote_motion_dir"):
    checks.append(("reference_motion.remote_motion_dir", ref_motion.get("remote_motion_dir")))
for i, remote_dir in enumerate(ref_motion.get("remote_motion_dirs") or []):
    checks.append((f"reference_motion.remote_motion_dirs[{i}]", remote_dir))
if ref_motion.get("idle_csv_path"):
    checks.append(("reference_motion.idle_csv_path", ref_motion.get("idle_csv_path")))
teleop_fallback = cfg.get("teleop", {}).get("fallback_reference", {})
if isinstance(teleop_fallback, dict) and teleop_fallback.get("enabled", False):
    checks.append(
        ("teleop.fallback_reference.csv_path", teleop_fallback.get("csv_path"))
    )
for label, value in checks:
    if not value:
        raise SystemExit(f"missing packaged config value: {label}")
    if os.path.isabs(str(value)):
        raise SystemExit(f"packaged config path must be relative: {label}={value}")
    if not (cfg_path.parent.parent / str(value)).exists():
        raise SystemExit(f"packaged config path does not exist: {label}={value}")
for aimrt_name in ("a3_aimrt_config.iceoryx.yaml", "a3_aimrt_config.ros2.yaml"):
    if not (cfg_path.parent / aimrt_name).exists():
        raise SystemExit(f"missing packaged AimRT transport config: {aimrt_name}")
if arch == "thor":
    logging_cfg = cfg.get("logging", {})
    for key in ("artificial_infer_delay_ms", "artificial_infer_delay_jitter_ms"):
        if float(logging_cfg.get(key, -1.0)) != 0.0:
            raise SystemExit(f"Thor package must set logging.{key}=0.0")
PY
}

if [[ "${INSIDE_DOCKER}" -eq 0 ]]; then
  case "${ARCH}" in
    rockchip)
      build_rockchip_in_docker
      ;;
    thor)
      build_thor_in_docker
      if [[ "${BUILD_ONLY}" -eq 0 ]]; then
        exit 0
      fi
      ;;
    *)
      configure_and_build
      ;;
  esac
else
  configure_and_build
fi

if [[ "${BUILD_ONLY}" -eq 0 ]]; then
  stage_runtime_config_and_assets
  stage_body_drive_debug_files
  stage_extra_libs
  stage_ros2_plugin_proto_prefix
  stage_joint_msgs_ros2_cli_overlay
  stage_run_scripts
  verify_package
  echo "A3 deploy package ready: ${PKG_DIR}"
fi
