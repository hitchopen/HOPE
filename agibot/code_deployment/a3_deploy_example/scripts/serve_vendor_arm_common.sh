#!/usr/bin/env bash

# Fail-closed package checks shared by the fixed vendor-arm serve wrapper.
# In the built package this file, the runner and the wrapper all live at the
# package root.

SERVE_SCRIPT_DEPLOY_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd
)"
SERVE_VENDOR_ARM_RUNNER="${SERVE_SCRIPT_DEPLOY_DIR}/a3_serve_vendor_arm_runner"
SERVE_VENDOR_ARM_WRAPPER="${SERVE_SCRIPT_DEPLOY_DIR}/run_serve_vendor_arm.sh"
SERVE_SCRIPT_MOTION="${SERVE_SCRIPT_DEPLOY_DIR}/motions/serve_policy.csv"
SERVE_VENDOR_ARM_MANIFEST="${SERVE_SCRIPT_DEPLOY_DIR}/config/serve_vendor_arm_manifest.json"
SERVE_VENDOR_ARM_BUILD_BINDINGS="${SERVE_SCRIPT_DEPLOY_DIR}/config/serve_vendor_arm_build.env"
SERVE_VENDOR_ARM_PACKAGE_MANIFEST="${SERVE_SCRIPT_DEPLOY_DIR}/config/serve_vendor_arm_package.sha256"

serve_script_die() {
  echo "[serve-vendor-arm] ERROR: $*" >&2
  exit 1
}

serve_script_sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    serve_script_die "sha256sum or shasum is required"
  fi
}

serve_script_active_runner_pids() {
  local executable=""
  local proc_dir=""
  for proc_dir in /proc/[0-9]*; do
    [[ -d "${proc_dir}" ]] || continue
    executable="$(readlink "${proc_dir}/exe" 2>/dev/null || true)"
    executable="${executable% (deleted)}"
    [[ "${executable##*/}" == "a3_serve_vendor_arm_runner" ]] || continue
    printf '%s\n' "${proc_dir#/proc/}"
  done
}

serve_vendor_arm_verify_package_manifest() {
  local actual=""
  local expected=""
  local relative=""
  local path=""
  local -A seen=()
  local -a required=(
    "a3_serve_vendor_arm_runner"
    "run_serve_vendor_arm.sh"
    "serve_vendor_arm_common.sh"
    "motions/serve_policy.csv"
    "config/serve_vendor_arm_manifest.json"
    "config/serve_vendor_arm_build.env"
  )

  [[ -f "${SERVE_VENDOR_ARM_PACKAGE_MANIFEST}" ]] ||
    serve_script_die \
      "package manifest is missing: ${SERVE_VENDOR_ARM_PACKAGE_MANIFEST}"

  while read -r expected relative || [[ -n "${expected}${relative}" ]]; do
    [[ "${expected}" =~ ^[0-9a-f]{64}$ ]] ||
      serve_script_die "invalid package-manifest digest"
    [[ "${relative}" =~ ^[A-Za-z0-9._+/-]+$ ]] ||
      serve_script_die "invalid package-manifest path: ${relative}"
    [[ "${relative}" != /* && "${relative}" != *".."* ]] ||
      serve_script_die "unsafe package-manifest path: ${relative}"
    [[ -z "${seen[${relative}]+x}" ]] ||
      serve_script_die "duplicate package-manifest path: ${relative}"
    path="${SERVE_SCRIPT_DEPLOY_DIR}/${relative}"
    [[ -f "${path}" ]] ||
      serve_script_die "package file is missing: ${relative}"
    actual="$(serve_script_sha256 "${path}")"
    [[ "${actual}" == "${expected}" ]] ||
      serve_script_die "package SHA mismatch: ${relative}"
    seen["${relative}"]=1
  done <"${SERVE_VENDOR_ARM_PACKAGE_MANIFEST}"

  for relative in "${required[@]}"; do
    [[ -n "${seen[${relative}]+x}" ]] ||
      serve_script_die "package manifest omits required file: ${relative}"
  done

  while IFS= read -r -d '' path; do
    relative="${path#"${SERVE_SCRIPT_DEPLOY_DIR}/"}"
    [[ "${relative}" == "config/serve_vendor_arm_package.sha256" ]] &&
      continue
    [[ -n "${seen[${relative}]+x}" ]] ||
      serve_script_die "unmanifested package file: ${relative}"
  done < <(find "${SERVE_SCRIPT_DEPLOY_DIR}" -type f -print0)
}

serve_script_prepare_mdu() {
  local active_runner_pids=""
  local elf_description=""
  local inference_artifact=""
  local ldd_output=""
  local machine=""
  local motion_sha=""

  machine="$(uname -m)"
  [[ "${machine}" == "aarch64" || "${machine}" == "arm64" ]] ||
    serve_script_die \
      "MDU must be arm64/aarch64; uname -m returned '${machine}'"
  [[ -f /agibot/software/v0/entry/env/env.sh ]] ||
    serve_script_die \
      "vendor environment is missing: /agibot/software/v0/entry/env/env.sh"
  command -v taskset >/dev/null 2>&1 ||
    serve_script_die "taskset is unavailable"
  command -v file >/dev/null 2>&1 ||
    serve_script_die "file is required for ELF verification"
  command -v ldd >/dev/null 2>&1 ||
    serve_script_die "ldd is required for dependency verification"
  [[ -x "${SERVE_VENDOR_ARM_RUNNER}" ]] ||
    serve_script_die "vendor-arm runner is missing or not executable"
  [[ -x "${SERVE_VENDOR_ARM_WRAPPER}" ]] ||
    serve_script_die "vendor-arm wrapper is missing or not executable"
  [[ -f "${SERVE_SCRIPT_MOTION}" ]] ||
    serve_script_die "serve CSV is missing: ${SERVE_SCRIPT_MOTION}"
  [[ -f "${SERVE_VENDOR_ARM_MANIFEST}" ]] ||
    serve_script_die "serve manifest is missing: ${SERVE_VENDOR_ARM_MANIFEST}"
  [[ -f "${SERVE_VENDOR_ARM_BUILD_BINDINGS}" ]] ||
    serve_script_die "build identity is missing: ${SERVE_VENDOR_ARM_BUILD_BINDINGS}"

  active_runner_pids="$(serve_script_active_runner_pids)"
  if [[ -n "${active_runner_pids}" ]]; then
    active_runner_pids="${active_runner_pids//$'\n'/,}"
    serve_script_die \
      "another vendor-arm runner is active; pids=${active_runner_pids}"
  fi

  motion_sha="$(serve_script_sha256 "${SERVE_SCRIPT_MOTION}")"
  [[ "${motion_sha}" == "${SERVE_VENDOR_ARM_EXPECTED_MOTION_SHA256}" ]] ||
    serve_script_die \
      "serve CSV SHA mismatch: expected ${SERVE_VENDOR_ARM_EXPECTED_MOTION_SHA256}, got ${motion_sha}"
  serve_vendor_arm_verify_package_manifest

  inference_artifact="$(
    find "${SERVE_SCRIPT_DEPLOY_DIR}" -type f \
      \( -name '*.onnx' -o -name '*.rknn' -o -name '*.engine' \) \
      -print -quit
  )"
  if [[ -n "${inference_artifact}" ]]; then
    serve_script_die "inference artifacts are not allowed in this fixed-motion package"
  fi

  set +u
  # shellcheck disable=SC1091
  source /agibot/software/v0/entry/env/env.sh
  set -u
  export LD_LIBRARY_PATH="${SERVE_SCRIPT_DEPLOY_DIR}:${LD_LIBRARY_PATH:-}"

  elf_description="$(LC_ALL=C LANG=C file "${SERVE_VENDOR_ARM_RUNNER}")"
  [[ "${elf_description}" == *"ELF 64-bit"* &&
     ( "${elf_description}" == *"ARM aarch64"* ||
       "${elf_description}" == *"aarch64"* ) ]] ||
    serve_script_die \
      "vendor-arm runner is not an AArch64 ELF: ${elf_description}"
  ldd_output="$(LC_ALL=C LANG=C ldd "${SERVE_VENDOR_ARM_RUNNER}" 2>&1)" ||
    serve_script_die "ldd could not inspect the vendor-arm runner"
  [[ "${ldd_output}" != *"not found"* ]] ||
    serve_script_die "vendor-arm runner has unresolved dependencies"

  cd "${SERVE_SCRIPT_DEPLOY_DIR}"
  echo \
    "[serve-vendor-arm] PACKAGE PASS: runner_sha=$(serve_script_sha256 "${SERVE_VENDOR_ARM_RUNNER}") csv_sha=${motion_sha}"
}
