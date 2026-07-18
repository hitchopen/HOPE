# =============================================================================
# HOPE PingPong training environment.
#
# SOURCE this (do not execute) inside your GPU/Isaac shell before running the
# training / play / export scripts:
#
#   cd hope_training/whole_body_tracking
#   source setup_train_env.sh
#
# It (1) puts the working-tree package source first on PYTHONPATH (so local edits
# win over any installed copy) and (2) defines an `isaac_py` launcher that runs
# your Isaac Sim Python with that PYTHONPATH. There is no external logging or
# experiment-tracking setup — training writes local checkpoints only.
#
# Point ISAAC_PYTHON / ISAACLAB_ROOT at your install if the defaults do not match
# (e.g. export them in setup_train_env.local.sh, which is auto-sourced if present).
# Safe to re-source; idempotent.
# =============================================================================

# Directory of this script (.../hope_training/whole_body_tracking), so sourcing works from any cwd.
_WBT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Optional machine-specific override (git-ignored).
if [ -f "${_WBT_DIR}/setup_train_env.local.sh" ]; then
  # shellcheck disable=SC1091
  source "${_WBT_DIR}/setup_train_env.local.sh"
fi

# Isaac Sim python launcher: honor a valid preset, else fall back to a common default.
if [ -n "${ISAAC_PYTHON:-}" ] && [ ! -x "${ISAAC_PYTHON}" ]; then
  echo "[hope] ISAAC_PYTHON='${ISAAC_PYTHON}' is not executable here -> re-probing."
  unset ISAAC_PYTHON
fi
if [ -z "${ISAAC_PYTHON:-}" ]; then
  if [ -x "${HOME}/isaacsim/python.sh" ]; then
    ISAAC_PYTHON="${HOME}/isaacsim/python.sh"
  elif command -v python >/dev/null 2>&1; then
    ISAAC_PYTHON="$(command -v python)"
  else
    ISAAC_PYTHON="/opt/isaacsim/python.sh"
  fi
fi
export ISAAC_PYTHON

# Isaac Lab source root (used only to extend PYTHONPATH when Isaac Lab is a source checkout rather
# than an installed package). Leave ISAACLAB_ROOT unset if Isaac Lab is pip-installed.
_extra_paths=""
if [ -n "${ISAACLAB_ROOT:-}" ] && [ -d "${ISAACLAB_ROOT}/source" ]; then
  _il="${ISAACLAB_ROOT}/source"
  _extra_paths=":${_il}/isaaclab:${_il}/isaaclab_tasks:${_il}/isaaclab_assets:${_il}/isaaclab_rl"
fi

# Working-tree package source FIRST so local edits win over any installed copy.
export HOPE_PYTHONPATH="${_WBT_DIR}/source/whole_body_tracking${_extra_paths}"

# Run Isaac's python with the training PYTHONPATH.
isaac_py () {
  PYTHONPATH="${HOPE_PYTHONPATH}${PYTHONPATH:+:$PYTHONPATH}" "${ISAAC_PYTHON}" "$@"
}

unset _WBT_DIR _il _extra_paths

echo "[hope] training env ready."
echo "[hope]   isaac_py -> ${ISAAC_PYTHON}"
echo "[hope]   run:  isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true"
if [ ! -x "${ISAAC_PYTHON}" ]; then
  echo "[hope]   WARNING: ISAAC_PYTHON='${ISAAC_PYTHON}' is not executable — set it in setup_train_env.local.sh."
fi
