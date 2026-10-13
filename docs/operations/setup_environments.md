# Environment setup

The canonical installation guide is
[DISTROBOX_SETUP.md](../DISTROBOX_SETUP.md). This file keeps existing operation
links working; update installation commands in the canonical guide.

| Work | Environment |
| --- | --- |
| Isaac train/play/evaluate/export | NVIDIA-enabled `grasping`; deactivate Conda, source `setup_train_env.sh`, use `hope_isaac_py` |
| ROS 2 Jazzy, C++ Runner/Planner and native MuJoCo Gate 3 | Ubuntu 24.04 `hope`; source ROS and the intended workspace overlay |
| Independent offline MuJoCo experiments | Separate Python 3.12 venv with that tool's dependencies |

The host's Python 3.14 is not the documented Isaac or ROS interpreter.
Source setup scripts after entering the appropriate container in each new
terminal. Do not share Isaac, ROS and experimental Python package installs.
