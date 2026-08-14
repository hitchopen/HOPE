# =============================================================================
# HOPE training environment.
#
# SOURCE this (do not execute) inside your GPU/Isaac shell before running the
# training / play / export scripts:
#
#   cd hope_training/whole_body_tracking
#   source setup_train_env.sh
#
# It (1) puts the working-tree package source first on PYTHONPATH (so local edits
# win over any installed copy) and (2) defines a `hope_isaac_py` launcher that
# runs your Isaac Sim Python with that PYTHONPATH. There is no external logging
# or experiment-tracking setup — training writes local checkpoints only.
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

# Re-sourcing after entering a container must not retain a host-only path.
if [ -n "${ISAAC_PYTHON:-}" ] && [ ! -x "${ISAAC_PYTHON}" ]; then
  echo "[hope] inherited ISAAC_PYTHON='${ISAAC_PYTHON}' is not executable here -> re-probing."
  unset ISAAC_PYTHON
fi
if [ -n "${ISAACLAB_ROOT:-}" ] && [ ! -d "${ISAACLAB_ROOT}/source" ]; then
  echo "[hope] inherited ISAACLAB_ROOT='${ISAACLAB_ROOT}' has no source/ here -> re-probing."
  unset ISAACLAB_ROOT
fi
if [ -n "${ISAAC_VENV_SITE:-}" ] && [ ! -d "${ISAAC_VENV_SITE}" ]; then
  echo "[hope] inherited ISAAC_VENV_SITE='${ISAAC_VENV_SITE}' does not exist here -> re-probing."
  unset ISAAC_VENV_SITE
fi

# Prefer Isaac's launcher over an arbitrary `python` on PATH.  In common
# Omnidrones/Distrobox images `python` is a minimal Conda interpreter which has
# neither Isaac Lab nor torch, while /workspace/isaacsim/python.sh is the real
# simulator interpreter.
if [ -z "${ISAAC_PYTHON:-}" ]; then
  for _candidate in \
    /workspace/isaacsim/python.sh \
    "${HOME}/isaacsim/python.sh" \
    /workspace/hope_isaac_venv/bin/python \
    /opt/isaacsim/python.sh; do
    if [ -x "${_candidate}" ]; then
      ISAAC_PYTHON="${_candidate}"
      break
    fi
  done
fi

# A plain Python is accepted only when Isaac Lab is already installed in it.
if [ -z "${ISAAC_PYTHON:-}" ]; then
  for _candidate in "$(command -v python 2>/dev/null)" "$(command -v python3 2>/dev/null)"; do
    if [ -n "${_candidate}" ] && "${_candidate}" -c \
      'import hydra, omegaconf, torch; import importlib.util; assert importlib.util.find_spec("isaaclab")' \
      >/dev/null 2>&1; then
      ISAAC_PYTHON="${_candidate}"
      break
    fi
  done
fi

# Source-checkout layouts used by the documented Isaac Lab installation and
# by the HOPE development container.  Leave unset for a pip-installed Isaac Lab.
if [ -z "${ISAACLAB_ROOT:-}" ]; then
  for _candidate in \
    /workspace/omni_drones/third_party/IsaacLab \
    /workspace/IsaacLab \
    "${HOME}/IsaacLab" \
    /opt/IsaacLab; do
    if [ -d "${_candidate}/source" ]; then
      ISAACLAB_ROOT="${_candidate}"
      break
    fi
  done
fi

# Some Isaac container images keep Hydra/torch in a companion venv while
# python.sh supplies the simulator runtime.
if [ -z "${ISAAC_VENV_SITE:-}" ]; then
  for _candidate in \
    /opt/drone_venv/lib/python3.11/site-packages \
    /opt/drone_venv/lib/python3.10/site-packages; do
    if [ -d "${_candidate}" ]; then
      ISAAC_VENV_SITE="${_candidate}"
      break
    fi
  done
fi
export ISAAC_PYTHON ISAACLAB_ROOT ISAAC_VENV_SITE

# Working-tree package source FIRST so local edits win over an installed copy.
HOPE_PYTHONPATH="${_WBT_DIR}/source/whole_body_tracking"
if [ -n "${ISAAC_VENV_SITE:-}" ]; then
  HOPE_PYTHONPATH="${HOPE_PYTHONPATH}:${ISAAC_VENV_SITE}"
fi
if [ -n "${ISAACLAB_ROOT:-}" ]; then
  _il="${ISAACLAB_ROOT}/source"
  HOPE_PYTHONPATH="${HOPE_PYTHONPATH}:${_il}/isaaclab:${_il}/isaaclab_tasks:${_il}/isaaclab_assets:${_il}/isaaclab_rl"
fi
export HOPE_PYTHONPATH

# Run Isaac's python with the training PYTHONPATH.
hope_isaac_py () {
  if [ -z "${ISAAC_PYTHON:-}" ] || [ ! -x "${ISAAC_PYTHON}" ]; then
    echo "[hope] ERROR: no usable Isaac Sim Python was found." >&2
    echo "[hope] Enter the GPU/Isaac shell, then re-source setup_train_env.sh," >&2
    echo "[hope] or set ISAAC_PYTHON in setup_train_env.local.sh." >&2
    return 127
  fi
  PYTHONPATH="${HOPE_PYTHONPATH}${PYTHONPATH:+:$PYTHONPATH}" "${ISAAC_PYTHON}" "$@"
}

# Backward-compatible short alias retained for early public-branch users.
isaac_py () {
  hope_isaac_py "$@"
}

unset _WBT_DIR _il _candidate

if [ -n "${ISAAC_PYTHON:-}" ] && [ -x "${ISAAC_PYTHON}" ]; then
  echo "[hope] training env ready."
  echo "[hope]   hope_isaac_py -> ${ISAAC_PYTHON}"
  if [ -n "${ISAACLAB_ROOT:-}" ]; then
    echo "[hope]   Isaac Lab source -> ${ISAACLAB_ROOT}"
  fi
else
  echo "[hope] training env NOT ready: no usable Isaac Sim Python was found."
fi
echo "[hope]   run:  hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true"
