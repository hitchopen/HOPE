# Build and test

Updated for the public runtime on 2026-10-13. Install dependencies through
[DISTROBOX_SETUP.md](../DISTROBOX_SETUP.md). Current ROS and native build
commands run inside Ubuntu 24.04 / ROS 2 Jazzy `hope`. Substitute the actual
clone path for `$HOME/workspace/HOPE`.

For full policy packaging, simulator installation, Gate 3 checks/runs/reports
and Rockchip staging, follow [deploy_and_gate3.md](deploy_and_gate3.md).
The compile-only example below does not prepare a complete runtime package.

## ROS workspace

From a host terminal:

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd "$HOME/workspace/HOPE/hope_ws"
rosdep install --from-paths src ../NatNet2ROS2/src --ignore-src --rosdistro jazzy -r -y
colcon build --base-paths src ../NatNet2ROS2/src --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
```

OptiTrack message/driver builds require `libpcl-dev`, `libfmt-dev`,
`libboost-program-options-dev`, `libeigen3-dev`, `python3-empy` and
`python3-lark`, included in the environment guide. A uv/Conda shim on `PATH`
can hide system ROS modules; the explicit CMake Python flags prevent that.
Deactivate unrelated venvs before building. The workspace overlay belongs to
the clone you built, even when another worktree shares the same repository.

For package-local planner tests, from the repository root in `hope`:

```bash
cd "$HOME/workspace/HOPE"
source /opt/ros/jazzy/setup.bash
PYTHONPATH="hope_ws/src/hope_planner${PYTHONPATH:+:$PYTHONPATH}" \
  /usr/bin/python3 -m pytest hope_ws/src/hope_planner/test -q
```

Select package tests for the change under review. For C++ Planner tests, build
with `-DBUILD_TESTING=ON` and run `ctest --test-dir hope_ws/build/hope_planner_cpp
--output-on-failure` from the repository root. Compilation and unit tests do
not certify a policy or hardware session.

## Native x86 Runner and simulator

The active deployment project is
[a3_deploy/a3_deploy_example](../../a3_deploy/a3_deploy_example/README.md). The preserved
`agi/code_deployment/` source is a vendor reference, not this build entry.

In a separate initialized `hope` shell:

```bash
cd "$HOME/workspace/HOPE/a3_deploy/a3_deploy_example"
source setup_a3_env.sh
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --build-only --jobs 8
```

The setup script can download ONNX Runtime 1.19.2 into ignored `thirdparty/`.
The builder uses CMake FetchContent for native dependencies and selects
`/usr/bin/python3` by default. `--build-only` compiles but skips full runtime
asset staging; packaging additionally requires the selected policy and motions.
The builder recreates its generated package directory, so use an isolated
checkout when retaining an existing package for a run.

Build the source AimRT simulator using
[its build instructions](../../a3_deploy/A3_MuJoCo_Sim/README.md#build); the source
project is `a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/`. A vendor standalone binary
linked against ROS Humble needs its own matching environment. Sourcing Jazzy
or installing Python `mujoco` does not rebuild that binary.

After preparing a Runner package, instrumented simulator install and ROS
workspace, configure their actual paths and inspect the supplied policy:

```bash
cd "$HOME/workspace/HOPE"
./run_gate3.sh --configure \
  --runner-dist /absolute/path/to/runner_package \
  --sim-install /absolute/path/to/mujoco/install \
  --hope-ws "$PWD/hope_ws"
./run_gate3.sh --policy-dir /absolute/path/to/policy --check
```

The two-file guide documents
[machine-local runtime configuration and reports](gate3_two_file_runner.md).
`--check` uses the native loader without starting simulation/control. A full
Gate 3 run and hardware qualification are separate later steps.
