# Reproduce the verified HOPE workstation environments

This is the canonical new-machine setup for the public HOPE repository. It is
based on the workstation that currently runs the HOPE Isaac, ROS 2 and native
build workflows successfully. Do not assemble an environment by independently
upgrading Isaac Sim, Isaac Lab, PyTorch and `rsl_rl`: the training code is
validated against the pinned `grasping` image below.

Unless a block explicitly says **inside `grasping`** or **inside `hope`**, run
it on the Laptop host. Setup never launches PPO training, a Runner, HAL, or any
robot process.

## Which environment is used for what

| Work | Environment | GPU |
| --- | --- | --- |
| Isaac scene smoke, policy train/play/evaluate/export | `grasping` Distrobox | required |
| ROS 2 Jazzy, C++ Planner, native Runner build, Gate 3 and Foxglove Laptop nodes | `hope` Distrobox | not required |
| Plain MuJoCo reference runner and offline analysis/generation tools | host Python virtual environment | not required |

The two containers deliberately share the clone through Distrobox's normal
host-home mount, but they do not share Python packages. Never source ROS 2 into
the Isaac shell and never run Isaac entrypoints with host, ROS, or bare Conda
Python.

## Known-working reference snapshot

This snapshot was read from the working machine on 2026-09-01. It is a
reproduction reference, not a claim that every value is a minimum requirement.

| Layer | Verified value |
| --- | --- |
| Host | Ubuntu 26.04.1 LTS, x86_64, kernel 7.0, system Python 3.14.4 |
| CPU / RAM | Intel Core Ultra 9 275HX, 24 logical CPUs, 30 GiB RAM |
| GPU | NVIDIA GeForce RTX 5070 Ti Laptop GPU, 12,227 MiB VRAM |
| Host driver | 580.173.02; `nvidia-smi` reports CUDA Version 13.0 |
| Container tools | rootless Podman 5.7.0, Distrobox 1.8.2.4 |
| NVIDIA container integration | NVIDIA Container Toolkit 1.20.0, CDI device `nvidia.com/gpu=all` |
| `grasping` image | `docker.io/danielmunicio/omnidrones@sha256:825c2da0a5ff581a7009c4f2e0ac44b15e83490242de17b13093f872fe569db2` |
| `grasping` userspace | Ubuntu 22.04.5, Isaac Sim 5.1.0 (`5.1.0-rc.19+release.26219.9c81211b.gl`), Python 3.11.13 |
| Training Python packages | Isaac Lab 0.54.2, PyTorch 2.7.0+cu128, Hydra 1.3.2, OmegaConf 2.3.0, `rsl-rl-lib` 3.1.2, Gymnasium 1.2.1, NumPy 1.26.4 |
| `hope` userspace | Ubuntu 24.04.4, ROS 2 Jazzy, Python 3.12.3, GCC 13.3, CMake 3.28.3 |

The pinned training image is about 64 GB before writable container data and
Isaac caches. Reserve at least 100 GB of free storage for a new installation.
The image contains the listed Isaac/Python stack before any HOPE-specific
changes, so recreating `grasping` does not depend on unrecorded `pip install`
commands from this workstation. It is a third-party image and Distrobox exposes
the host home directory to it: the digest prevents silent image drift, but it
does not replace a source/security review or an organization-managed mirror.

## 1. Prepare a new Ubuntu host

The exact verified host is Ubuntu 26.04.1 LTS. Ubuntu 24.04 can also host these
containers, but the host must be x86_64, use a recent NVIDIA driver, and expose
the GPU to rootless Podman through CDI. ROS 2 Jazzy stays in the Ubuntu 24.04
`hope` container because Jazzy's official Ubuntu packages target Noble.

### 1.1 Install and verify the NVIDIA driver

Install a production NVIDIA driver through Ubuntu's driver tool or the
[official NVIDIA Ubuntu driver guide](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/latest/ubuntu.html),
then reboot. The current machine uses the open 580 branch; do not replace an
already working driver merely to make the version string identical.

On a fresh Ubuntu desktop, the distribution-managed path is:

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

Continue only after the rebooted host can see the GPU:

```bash
uname -m
nvidia-smi
```

`uname -m` must print `x86_64`. No host CUDA Toolkit installation is required:
the pinned container carries PyTorch's CUDA 12.8 runtime, while the host only
provides the compatible NVIDIA driver.

### 1.2 Install Podman, Distrobox, Git and Git LFS

```bash
sudo apt update
sudo apt install -y \
  ca-certificates curl distrobox git git-lfs gnupg podman \
  python3-pip python3-pytest python3-venv python3-yaml

podman --version
distrobox --version
git lfs install
```

The verified versions are recorded above. A newer Distrobox is acceptable if
`distrobox create --help` still lists `--nvidia`. See the official
[`distrobox create` reference](https://distrobox.it/usage/distrobox-create/)
if Ubuntu does not package Distrobox on the selected host release.

### 1.3 Install NVIDIA Container Toolkit and verify CDI

Podman uses CDI rather than the Docker runtime configuration. These commands
follow NVIDIA's
[Container Toolkit install guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
and [CDI guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/cdi-support.html):

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes \
      -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L \
  https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

sudo apt update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.20.0-1
sudo apt install -y \
  "libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}" \
  "libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION}" \
  "nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION}" \
  "nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION}"
sudo systemctl enable --now nvidia-cdi-refresh.path
sudo systemctl restart nvidia-cdi-refresh.service

nvidia-ctk --version
nvidia-ctk cdi list
podman run --rm \
  --device nvidia.com/gpu=all \
  --security-opt=label=disable \
  docker.io/library/ubuntu:24.04 nvidia-smi -L
```

The last two commands must list the same physical GPU as host `nvidia-smi -L`.
Do not create `grasping` until this works; otherwise Distrobox can be created
successfully while Isaac remains unable to use CUDA.

### 1.4 Clone HOPE and materialize Git LFS content

Distrobox shares the host home directory, so clone under `$HOME`. The rest of
the documentation uses `$HOME/workspace/HOPE` as the example location; another
path under `$HOME` is valid if commands are adjusted consistently.

```bash
mkdir -p "$HOME/workspace"
git clone https://github.com/hitchopen/HOPE.git "$HOME/workspace/HOPE"
cd "$HOME/workspace/HOPE"

git lfs install
git lfs pull
test "$(stat -c %s hope_training/whole_body_tracking/checkpoints/model_21800.pt)" -gt 1000000
```

Without Git LFS, `model_21800.pt` is a small text pointer. Training from scratch
does not require that checkpoint, but a complete clone and published-model
playback do.

## 2. Create the verified `grasping` training environment

Run this block on the host. The digest is intentional: the moving `:eecs` tag
must not silently replace the known-working Isaac/PyTorch combination.

```bash
export HOPE_GRASPING_IMAGE='docker.io/danielmunicio/omnidrones@sha256:825c2da0a5ff581a7009c4f2e0ac44b15e83490242de17b13093f872fe569db2'

distrobox create \
  --name grasping \
  --image "$HOPE_GRASPING_IMAGE" \
  --nvidia \
  --yes

distrobox enter grasping
```

The first pull is large. Distrobox's `--nvidia` option should resolve to the
CDI device `nvidia.com/gpu=all`; no extra CUDA volume or host CUDA installation
is needed.

### 2.1 Initialize every training shell

Run this block **inside `grasping`**. Its interactive shell may activate Conda
`base`; deactivate it, then always use `hope_isaac_py`. Bare `python` in this
image is not the Isaac interpreter even after the Conda prompt disappears.

```bash
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  conda deactivate
fi

cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh
```

A successful source reports these paths:

```text
hope_isaac_py -> /workspace/isaacsim/python.sh
Isaac Lab source -> /workspace/omni_drones/third_party/IsaacLab
```

`setup_train_env.sh` adds the checked-out HOPE package to `PYTHONPATH`, so the
pinned path does not require an editable `pip install`. This also guarantees
that working-tree edits win over any installed package. Do not import
`whole_body_tracking` directly before Kit starts; its task modules require
`omni.*`. The entrypoints start `AppLauncher` first.

### 2.2 Verify the pinned environment without starting training

Run inside the initialized `grasping` shell:

```bash
nvidia-smi -L
cat /workspace/isaacsim/VERSION

hope_isaac_py - <<'PY'
from importlib import metadata, util
import sys
import torch

for name in (
    "isaaclab",
    "torch",
    "hydra-core",
    "omegaconf",
    "rsl-rl-lib",
    "gymnasium",
    "numpy",
):
    print(f"{name}={metadata.version(name)}")

print(f"python={sys.version.split()[0]}")
print(f"torch_cuda={torch.version.cuda}")
print(f"cuda_available={torch.cuda.is_available()}")
print(f"whole_body_tracking={util.find_spec('whole_body_tracking').origin}")
assert torch.cuda.is_available()
assert util.find_spec("isaaclab") is not None
assert util.find_spec("whole_body_tracking") is not None
PY
```

Prepare the generated A3 Isaac asset, then run a bounded scene smoke. This
loads and steps Isaac but does not load a policy or start PPO:

```bash
hope_isaac_py scripts/prepare_a3_isaac_asset.py --force
hope_isaac_py scripts/prepare_a3_isaac_asset.py --check
hope_isaac_py scripts/play_table_tennis.py --headless --steps 2
```

The first Kit launch builds caches and can take several minutes. Once this
passes, continue with [`QUICKSTART_A3_ISAAC.md`](../QUICKSTART_A3_ISAAC.md).
Training remains an explicit operator command.

### 2.3 Custom Isaac installations

The pinned Distrobox is the supported reproduction path. If it cannot be used,
install an equivalent Isaac Sim 5.1/Python 3.11/Isaac Lab stack and place a
git-ignored `setup_train_env.local.sh` next to `setup_train_env.sh`:

```bash
export ISAAC_PYTHON=/absolute/path/to/isaacsim/python.sh
export ISAACLAB_ROOT=/absolute/path/to/IsaacLab
```

That alternate path is not the workstation baseline until the version probe,
asset preparation and bounded scene smoke above pass. Current upstream Isaac
Lab documentation may recommend newer Isaac releases; upgrading the simulator
and training libraries is a migration task, not a new-machine setup shortcut.

## 3. Create the `hope` ROS 2 and native-build environment

The `hope` container intentionally uses Ubuntu 24.04 because ROS 2 Jazzy binary
packages target that release. Run this on the host:

```bash
distrobox create \
  --name hope \
  --image quay.io/toolbx/ubuntu-toolbox:24.04 \
  --yes

distrobox enter hope
```

Run the remaining installation block **inside `hope`**:

```bash
sudo apt update
sudo apt install -y \
  ca-certificates curl locales software-properties-common

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository -y universe
sudo apt update

export ROS_APT_SOURCE_VERSION="$({
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest
} | sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p')"
test -n "$ROS_APT_SOURCE_VERSION"

curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

sudo apt install -y \
  ros-jazzy-desktop ros-dev-tools \
  build-essential cmake coreutils git git-lfs pkg-config procps tar \
  util-linux wget \
  python3-colcon-common-extensions python3-numpy python3-pip \
  python3-rosdep python3-vcstool python3-yaml \
  cppzmq-dev libacl1-dev libboost-all-dev libeigen3-dev libglm-dev \
  libgl1-mesa-dev libglfw3-dev libgtest-dev libssl-dev libmsgpack-cxx-dev \
  libx11-dev libxcursor-dev libxi-dev libxinerama-dev \
  libxrandr-dev libxxf86vm-dev libyaml-cpp-dev libzmq3-dev rapidjson-dev \
  x11-utils zlib1g-dev

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

source /opt/ros/jazzy/setup.bash
test "$ROS_DISTRO" = jazzy
ros2 --help >/dev/null
cmake --version
exit
```

Back on the host, verify the completed environment:

```bash
distrobox enter hope -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  test "$ROS_DISTRO" = jazzy
  test "$(. /etc/os-release && echo "$VERSION_ID")" = 24.04
  ros2 --help >/dev/null
  cmake --version
  echo "hope: READY"
'
```

For every later ROS/native-build shell:

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd "$HOME/workspace/HOPE"
```

Do not enable `set -u` while sourcing `/opt/ros/jazzy/setup.bash`. Continue with
[`hope_ws/SMOKE_TEST.md`](../hope_ws/SMOKE_TEST.md) for the process-only C++
Planner smoke or [`MODEL_21800.md`](MODEL_21800.md) for the full native build
and Gate 3 workflow.

## Troubleshooting

- `nvidia-ctk cdi list` has no `nvidia.com/gpu=all`: fix the host driver and
  `nvidia-cdi-refresh` before recreating the container.
- `torch.cuda.is_available()` is false inside `grasping`: confirm the container
  was created with `--nvidia`, then compare host and container `nvidia-smi -L`.
- `training env NOT ready` or `ModuleNotFoundError: hydra`: enter `grasping`,
  deactivate the active Conda environment, source `setup_train_env.sh`, and use
  `hope_isaac_py`; do not call `/workspace/isaacsim/python.sh` or bare Python
  directly.
- `ModuleNotFoundError: omni.timeline` from `python -c 'import
  whole_body_tracking'`: this import occurred before Kit startup. Use the
  `find_spec` probe above or a shipped Isaac entrypoint.
- `model_21800.pt` is about 130 bytes: install Git LFS on the host and run
  `git lfs pull` from the repository root.
- `/opt/ros/jazzy/setup.bash` is missing: the `hope` container is incomplete or
  was created from Ubuntu 22.04. Recreate it from the documented 24.04 image.
- The Isaac image fills the disk during creation: check `podman system df`; do
  not delete active containers or training data as an installation shortcut.
