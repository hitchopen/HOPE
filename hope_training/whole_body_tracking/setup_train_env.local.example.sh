# Local Isaac paths for HOPE PingPong. Copy this file to
# setup_train_env.local.sh and edit the values for your machine. The copied file
# is git-ignored and is auto-sourced by setup_train_env.sh if present.
#
# Both variables are optional: setup_train_env.sh probes for a usable Isaac Sim
# Python on its own, and Isaac Lab may simply be pip-installed. Set them only
# when the probe picks the wrong interpreter or your Isaac Lab is a source
# checkout.

# The Isaac Sim Python interpreter used by the `isaac_py` launcher.
export ISAAC_PYTHON=/absolute/path/to/isaacsim/python.sh

# An Isaac Lab *source checkout* root (the directory containing `source/`).
# Leave unset if Isaac Lab is pip-installed.
export ISAACLAB_ROOT=/absolute/path/to/IsaacLab
