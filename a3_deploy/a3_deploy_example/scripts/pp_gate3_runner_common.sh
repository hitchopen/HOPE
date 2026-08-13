#!/usr/bin/env bash

# Shared policy-native argv contract for the model_21800 Gate3 engine.  The
# string is intentionally shell-safe: every token is free of whitespace and
# the conductor splits it before adding observability-only arguments.
GATE3_POLICY_NATIVE_CORE_ARGS="--planner --policy-native --start passive --official-stand"
RALLY_POLICY_NATIVE_CORE_ARGS="$GATE3_POLICY_NATIVE_CORE_ARGS"

gate3_runner_die() {
  echo "[gate3-runner] ERROR: $*" >&2
  return 1
}

rally_assert_policy_native_runner_args() {
  local token required found
  local -a argv=("$@")
  for required in --planner --policy-native --start passive --official-stand; do
    found=0
    for token in "${argv[@]}"; do
      if [[ "$token" == "$required" ]]; then
        found=1
        break
      fi
    done
    [[ "$found" -eq 1 ]] ||
      gate3_runner_die "runner command is missing shared contract token '$required'" || return 1
  done

  # These options change policy decisions, timing, or recovery behavior.  They
  # are commissioning tools, never part of autonomous Gate3.
  for token in "${argv[@]}"; do
    case "$token" in
      --demo|--side|--side=*|--hold-recover|--hold-recover=*|\
      --gate-x-max|--gate-x-max=*|--ready-x-max|--ready-x-max=*|\
      --ready-y-max|--ready-y-max=*|--ready-speed-max|--ready-speed-max=*|\
      --ready-dwell|--ready-dwell=*|--swing-rest|--swing-rest=*)
        gate3_runner_die "runner command contains Gate3-forbidden override '$token'" || return 1
        ;;
    esac
  done
}
