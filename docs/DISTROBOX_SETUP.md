# Distrobox environments

Run every command in this document on the **Laptop host**, not inside an
existing container.

## Install Distrobox on a new Ubuntu machine

```bash
sudo apt update
sudo apt install -y podman curl

if ! command -v distrobox >/dev/null 2>&1; then
  if ! sudo apt install -y distrobox; then
    curl -s \
      https://raw.githubusercontent.com/89luca89/distrobox/main/install |
      sh -s -- --prefix "$HOME/.local"

    grep -qxF 'export PATH="$HOME/.local/bin:$PATH"' "$HOME/.bashrc" ||
      echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
    export PATH="$HOME/.local/bin:$PATH"
  fi
fi

podman --version
distrobox --version
```

## `hope`: ROS 2 Jazzy, Planner, Runner and Gate3

An empty Ubuntu container is not enough: `hope` also needs ROS 2 Jazzy and the
native build dependencies. Follow
[`MODEL_21800.md`, section 0](MODEL_21800.md#0-one-time-hope-distrobox-setup-on-a-new-machine)
once, then return to the runbook that sent you here.

Check the completed environment from the host:

```bash
distrobox list
distrobox enter hope -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  test "$ROS_DISTRO" = jazzy
  ros2 --help >/dev/null
  echo "hope: READY"
'
```

## `grasping`: Isaac Sim and Isaac Lab

`grasping` is the conventional name for a separately provisioned NVIDIA Isaac
environment. This repository does not create or download that container.
Follow [`QUICKSTART_A3_ISAAC.md`](../QUICKSTART_A3_ISAAC.md) to install a
compatible Isaac Sim/Isaac Lab environment. If your team supplies a prepared
`grasping` container, enter it before sourcing `setup_train_env.sh`; otherwise
run the same setup script from the host's working Isaac shell.

Do not create a plain Ubuntu container named `grasping` and expect Isaac Sim,
Isaac Lab, CUDA or `rsl_rl` to be present.
