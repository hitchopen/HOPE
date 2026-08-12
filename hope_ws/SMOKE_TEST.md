# HOPE bring-up DRY-RUN smoke test

Verifies the new ROS 2 packages **build, register, launch, and log** — with **no
hardware command ever published**. Run the script, or the manual checklist below.

```bash
cd hope_ws
./smoke_test_dry_run.sh /abs/path/to/exported/policy.onnx
# or rely on the default ONNX path (the re-exported both-clip model_15200):
./smoke_test_dry_run.sh
```

## Why it is hardware-safe

- `wbc_runner` runs **`mode:=dry_run`** → it **never** creates a joint-command
  publisher and **never** publishes `joint_msgs/JointCommand`. Nothing can move.
- `hardware_enable` stays **false**; estop state is irrelevant in dry_run.
- `planner_imitate` is launched **`dry_run:=false`** *only* so it **publishes
  `/racket/command`** (a target topic, not a hardware command) — needed for steps
  6–7. The planner never drives hardware. **Safety comes from the runner's
  `mode:=dry_run`, not the planner's flag.**

## Manual checklist

| # | Step | Expected |
|---|---|---|
| 0 | `source /opt/ros/<distro>/setup.bash` | `ROS_DISTRO` set |
| 1 | `colcon build --packages-up-to hope_planner hope_wbc_runner --symlink-install` | builds incl. `hope_msgs` |
| 2 | `source install/local_setup.bash` | overlay sourced |
| 3 | `ros2 pkg list \| grep hope_wbc_runner` | listed; `ros2 pkg executables hope_wbc_runner` → `wbc_runner_node` |
| 4 | `ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=1` | "planner_imitate started" |
| 5 | `ros2 launch hope_wbc_runner wbc_runner.launch.py mode:=dry_run onnx_path:=.../policy.onnx csv_path:=/tmp/wbc_runner.csv` | "wbc_runner started", "WILL NOT PUBLISH" |
| 6 | `ros2 topic echo --once /racket/command` | a `RacketCommand` with `frame_id: base_link` |
| 7 | `cat /tmp/wbc_runner.csv` | header has `obs_norm`, `action_norm`, `target_q_0..30`; active `forehand` rows with non-zero norms |
| 8 | `cd src/hope_wbc_runner && PYTHONPATH=. python3 -m pytest test -q` (and same for `hope_planner`) | 12 + 35 passed |

> Use **step 1 with `--packages-up-to`** (not `--packages-select hope_planner
> hope_wbc_runner`): the leaf packages depend on `hope_msgs`, and building them
> against a stale `hope_msgs` causes a typesupport/python skew at launch.

## Pre-verified in this repo (build side)

- ✅ `colcon build` of both packages succeeds (pure ament_python).
- ✅ `ros2 pkg executables`: `wbc_runner_node`, `planner_imitate_node`, `hope_planner_node`.
- ✅ Unit tests: **12** (`hope_wbc_runner`) + **35** (`hope_planner`) pass via `pytest`.
- ⏳ Live launch / topic-echo / CSV and the ONNX path are run by **you** (needs your
  ROS python + `onnxruntime` + the model).

## Troubleshooting

- **`Could not import 'rosidl_typesupport_c' for package 'hope_msgs'`** /
  **`ImportError: libpython3.X.so: cannot open shared object file`** →
  `hope_msgs` was built under a different python than the ROS you sourced.
  Rebuild the whole workspace under ONE ROS: `rm -rf build install && colcon build`.
- **`colcon test` says "Ran 0 tests"** → on some distros colcon picks the unittest
  runner, which doesn't collect pytest-style functions. Use direct `pytest`
  (step 8) — that's what the script does.
- **`ModuleNotFoundError: onnxruntime`** → `python3 -m pip install onnxruntime` in
  the ROS python. The runner needs it even in dry_run (it still runs the ONNX).
- **CSV has only `stand` rows** → `planner_imitate` is not publishing; launch it
  with `dry_run:=false` (see safety note above).

## NOT done by this smoke (by design)

No `mode:=hardware`, no `hardware_enable:=true`, no `joint_msgs/JointCommand`
published, no robot motion. Hardware bring-up is a separate, later step.
