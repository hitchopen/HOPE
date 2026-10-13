#!/usr/bin/env bash
# Resolve once, before starting either the planner or the physical simulator.
pp_load_ball_physics() {
  local params_file="$1" profile="${2:-}" receipt="${3:-}" exports
  local helper_dir
  helper_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  local -a args=(--params-file "$params_file" --shell)
  [[ -z "$profile" ]] || args+=(--params-file "$profile")
  [[ -z "$receipt" ]] || args+=(--receipt "$receipt")
  exports="$(python3 "$helper_dir/pp_ball_physics.py" "${args[@]}")" || return 2
  # Output contains only fixed names and validated, quoted finite numbers.
  eval "$exports"
  echo "[ball-physics] source=$params_file drag=$PP_BALL_DRAG_K horizontal=$PP_BALL_RESTITUTION_H vertical=$PP_BALL_RESTITUTION_V future_gain=$PP_BALL_FUTURE_BOUNCE_GAIN"
}
