# HOPE ping-pong bring-up tutorial (beginner, copy-paste)

A safe, **dry-run only** walkthrough. **No real hardware moves.** You will:
1. build + smoke-test the ROS bring-up in the **`hope`** container,
2. run `planner_imitate` + `wbc_runner` by hand across a few terminals,
3. **see the A3 robot swing in MuJoCo** in the **`grasping`** container.

## Which container does what?

| Task | Container | Why |
|---|---|---|
| ROS bring-up (planner_imitate, wbc_runner, smoke test) | **`hope`** | it's the ROS 2 Jazzy workspace |
| MuJoCo visual swing (`mujoco_eval_onnx.py --viewer`) | **`grasping`** | it has the `hope-motion-py310` conda env with `mujoco` + `onnxruntime` |

> The ROS pipeline (Sections 1–2) and the MuJoCo picture (Section 3) are **separate**
> today. `wbc_runner` builds the observation and runs the policy but only **logs** the
> joint targets — nothing feeds them into a live MuJoCo robot yet (see Section 3's
> "Current limitation"). The MuJoCo viewer runs the *same* policy on its own.

Safety in one line: `wbc_runner` runs `mode:=dry_run` → it **never** publishes a joint
command and **cannot** move hardware. We never use `mode:=hardware`.

---

## Section 1 — Build + smoke test (in `hope`)

### 1.1 Open a terminal on the host and enter the container
```bash
distrobox enter hope -- bash
```
- **Does:** drops you into the `hope` ROS 2 container; your prompt changes.
- **Success:** prompt shows you're inside (e.g. `user@hope:~$`).
- **Error `No such container: hope`** → list them: `distrobox list`. Use the ROS one
  (the image with `ros2-jazzy`). If it's stopped, `distrobox enter hope` starts it.

### 1.2 Find your ROS distro, then source it
```bash
ls /opt/ros/
```
- **Does:** shows the installed ROS 2 distro folder name.
- **Success:** prints one name, e.g. `jazzy`.
- Then source it (replace `jazzy` if the folder above was different):
```bash
source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
```
- **Success:** `ROS_DISTRO=jazzy`.
- **Error `No such file or directory`** → you used the wrong name; re-run `ls /opt/ros/`
  and source the exact folder shown.

### 1.3 Build the two packages (and their message dependency)
```bash
export HOPE_REPO=/path/to/your/clone   # root of this repository checkout
cd $HOPE_REPO/hope_ws
colcon build --packages-up-to hope_planner hope_wbc_runner --symlink-install
```
- **Does:** compiles `hope_planner`, `hope_wbc_runner`, and `hope_msgs` (the
  `/racket/command` message) with the **same** ROS/python — `--packages-up-to` pulls in
  `hope_msgs` so you don't get a version mismatch later.
- **Success:** ends with `Summary: 3 packages finished`.
- **Error: `Could not find Python3 ... Development`** or a `hope_msgs` failure → your
  container is missing python dev headers; install them:
  `sudo apt update && sudo apt install -y python3-dev`, then rebuild.
- **Warning `Unknown distribution option: 'tests_require'`** → harmless, ignore.

### 1.4 Source your freshly-built workspace
```bash
source install/local_setup.bash
```
- **Does:** makes `ros2` aware of your new packages.
- **Success:** no output (that's fine).
- **Tip:** every NEW terminal needs **both** `source /opt/ros/jazzy/setup.bash` **and**
  `source $HOPE_REPO/hope_ws/install/local_setup.bash`.

### 1.5 Confirm the packages are registered
```bash
ros2 pkg list | grep hope_wbc_runner
ros2 pkg executables hope_wbc_runner
ros2 pkg executables hope_planner | grep planner_imitate
```
- **Success:** you see `hope_wbc_runner`, then `hope_wbc_runner wbc_runner_node`, then
  `hope_planner planner_imitate_node`.
- **Error: nothing prints** → you forgot step 1.4 (source the overlay), or the build
  failed. Re-source / re-build.

### 1.6 Check (and install) onnxruntime
The runner runs the ONNX model even in dry-run, so it needs `onnxruntime`.
```bash
python3 -c "import onnxruntime; print('onnxruntime', onnxruntime.__version__)"
```
- **Success:** prints a version like `onnxruntime 1.18.0`.
- **Error: `ModuleNotFoundError: No module named 'onnxruntime'`** → install it into the
  ROS python:
  ```bash
  python3 -m pip install onnxruntime
  ```
  Re-run the check.

### 1.7 Locate the CORRECT ONNX (2-clip, ~1.24 MB)
```bash
# list every exported ONNX with size, newest last:
find $HOPE_REPO/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope/ \
  -name policy.onnx -printf '%s  %p\n' | sort -n
```
- **Does:** prints `bytes  path` for each exported policy.
- **The one you want** is the **unified `model_15200`** export, ~**1,243,884 bytes (≈1.24 MB)**:
  ```
  $HOPE_REPO/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch/exported/policy.onnx
  ```
- Save it in a variable (used everywhere below):
  ```bash
  export ONNX=$HOPE_REPO/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch/exported/policy.onnx
  ls -l "$ONNX"
  ```
- **Verify the size is right** (this is the #1 gotcha):
  ```bash
  stat -c '%s bytes' "$ONNX"
  ```
  - **≈ 1243884 (1.24 MB) → GOOD** (both clips baked in).
  - **≈ 1141378 (1.14 MB) → BAD** = the old forehand-only export; the backhand will be
    dead. Re-export with both clips before continuing.

### 1.8 Run the dry-run smoke test
```bash
cd $HOPE_REPO/hope_ws
./smoke_test_dry_run.sh "$ONNX"
```
- **Does:** builds, sources, checks the packages, **publishes fake `/racket/command`**,
  runs the runner in **dry-run** (logs only, no hardware), and verifies the CSV. It cleans
  up its background processes at the end.
- **Success:** ends with `SMOKE SUMMARY: N passed, 0 failed` and
  `DRY-RUN SMOKE OK (no hardware command was ever published).`
- **`[SKIP] onnxruntime not in the ROS python`** → do step 1.6, re-run.
- **`Could not import 'rosidl_typesupport_c' for package 'hope_msgs'`** or
  **`libpython3.X.so: cannot open shared object file`** → `hope_msgs` was built with a
  different python. Fix: `cd $HOPE_REPO/hope_ws && rm -rf build install && colcon build`,
  then re-source (1.4) and re-run.
- **`Permission denied`** → `chmod +x smoke_test_dry_run.sh` then re-run.

If the smoke passes, the ROS bring-up works. Section 2 runs it by hand.

---

## Section 2 — Run planner_imitate + wbc_runner manually (4 terminals, in `hope`)

Open **four** host terminals. In **each** one, run this same prologue first:
```bash
distrobox enter hope -- bash
export HOPE_REPO=/path/to/your/clone   # root of this repository checkout
source /opt/ros/jazzy/setup.bash
source $HOPE_REPO/hope_ws/install/local_setup.bash
export ONNX=$HOPE_REPO/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch/exported/policy.onnx
```

### Terminal A — the fake planner (publishes `/racket/command`)
```bash
ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=1
```
- **Does:** publishes a fake forehand strike target every few seconds. `level:=1` is a
  slow forehand (safest first). Levels: 0=stand, 1=fh slow, 2=fh, 3=bh slow, 4=bh, 5=alternate.
- **Why `dry_run:=false` is still safe:** for the planner, `dry_run:=false` only means
  "actually publish the `/racket/command` topic" — a target message, **not** a hardware
  command. Hardware safety comes from the **runner** being `mode:=dry_run` (Terminal B).
- **Success:** logs `planner_imitate started ... PUBLISHING to /racket/command` and a
  `[seq N] L1:forehand_slow ...` line each new swing.
- **Error: `package 'hope_planner' not found`** → re-source the overlay (prologue).

### Terminal B — the WBC runner in DRY-RUN (builds obs, runs model, logs)
```bash
ros2 launch hope_wbc_runner wbc_runner.launch.py mode:=dry_run \
  onnx_path:="$ONNX" csv_path:=/tmp/wbc_runner.csv
```
- **Does:** subscribes `/racket/command`, builds the 180-D observation, runs `model_15200`
  deterministically, and **logs** the action / joint targets to `/tmp/wbc_runner.csv`.
  **Publishes no joint commands.**
- **Success:** logs `wbc_runner started | mode=dry_run ...`,
  `WILL NOT PUBLISH (log only)`, then `[log-only/dry_run] forehand ts=34 phase=0.36 ...`.
- **Error: `onnx_path is required`** → `$ONNX` wasn't set in this terminal; re-run the
  prologue. **Error: `No module named 'onnxruntime'`** → step 1.6.

### Terminal C — watch the planner messages
```bash
ros2 topic echo --qos-reliability best_effort /racket/command
```
- **Does:** prints each fake `RacketCommand` (position, velocity, normal, time_to_strike…).
- **IMPORTANT — `--qos-reliability best_effort` is required.** `planner_imitate` publishes
  with **best-effort** QoS, but `ros2 topic echo` defaults to **reliable**, which is
  *incompatible* with a best-effort publisher — so a plain `ros2 topic echo` shows **nothing**
  even though the topic is publishing fine.
- **Success:** a message scrolls by with `frame_id: base_link` and a `position:` near
  `x: 0.4`. Forehand has **negative** `y`; backhand **positive** `y`.
- **Error: nothing prints / `does not appear to be published`** → either you dropped the
  `--qos-reliability best_effort` flag, or Terminal A isn't running with `dry_run:=false`.
- **Error: `xmlrpc.client.ResponseError: unknown tag 'rclpy.endpoint_info.TopicEndpointInfo'`** → a
  stale `ros2` daemon (left by a different ROS/python) is corrupting introspection. Fix:
  `ros2 daemon stop` (it restarts clean), then retry — or bypass it with
  `ros2 topic echo --no-daemon --qos-reliability best_effort /racket/command`.
- Quieter alternative (also proves data flow, no QoS flag needed): `ros2 topic hz /racket/command`.

### Terminal D — inspect the runner's CSV log
```bash
# header (the columns), then the most recent rows, nicely aligned:
head -1 /tmp/wbc_runner.csv | tr ',' '\n' | head -20
echo "--- last rows ---"
tail -5 /tmp/wbc_runner.csv | cut -c1-140
echo "--- count of ACTIVE forehand rows (non-idle) ---"
awk -F, 'NR>1 && $4=="forehand" && $9=="1"' /tmp/wbc_runner.csv | wc -l
```
- **Does:** shows the logged 180-D-obs summary + action + `target_q_0..30` (31 joint
  position targets), and counts real swing rows.
- **Success:** header includes `obs_norm`, `action_norm`, `target_q_0 … target_q_30`; the
  active-forehand count is > 0; `action_norm` and `obs_norm` are non-zero on swing rows.
- **Error: only `stand` rows / count is 0** → Terminal A is not publishing; make sure it
  runs with `dry_run:=false`.

**Stop everything:** press `Ctrl-C` in Terminals A and B.

---

## Section 3 — See the A3 robot swing in MuJoCo (in `grasping`)

The ROS runner only logs numbers. To **watch the A3 swing**, run the standalone MuJoCo
viewer, which loads the same `model_15200` ONNX and the official A3 ping-pong model.

### 3.1 Current limitation (read this)
- `planner_imitate → /racket/command → wbc_runner` is a **log-only** loop. `wbc_runner`
  in dry-run/shadow never publishes joint commands; even in hardware mode it would publish
  `joint_msgs/JointCommand`, which **no MuJoCo sim currently subscribes to**.
- The official `a3_pingpong` MuJoCo sim is driven by the Agibot **C++ deploy over iceoryx**,
  not by ROS `/racket/command` — so it can't be driven by `wbc_runner` yet.
- **What's missing** to connect them: a "shadow bridge" node that subscribes
  `/racket/command`, steps a live MuJoCo A3, and publishes `joint_states` back to the runner.
  That doesn't exist yet. Until then, use the viewer below — it runs the **same policy**,
  generating its own targets internally, so you genuinely see model_15200 swing.

### 3.2 Run the MuJoCo viewer (the closest "see it move" test)
Open a host terminal:
```bash
distrobox enter grasping -- bash
conda activate hope-motion-py310
export HOPE_REPO=/path/to/your/clone   # root of this repository checkout
cd $HOPE_REPO/hope_training/whole_body_tracking
RUN=logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch
python scripts/mujoco_eval_onnx.py --viewer --pd-mode implicit \
  --noise-scales 0.0 --steps 4000 \
  --onnx  $RUN/exported/policy.onnx \
  --std   $RUN/exported/learned_std.npy
```
- **Does:** opens a MuJoCo window and runs `model_15200` **deterministically** (`0.0` = no
  dither) on the A3 ping-pong model. You should see the robot do forehand/backhand swings
  toward a green target marker (red marker = the racket).
- **Success:** a 3D window opens; the A3 stands and swings; the terminal prints strike
  stats (forehand/backhand composite) when it finishes.
- **No strike-phase flag needed:** the script default is already forehand 0.36 / backhand
  0.50 (matches model_15200).
- **Want it to run longer / let backhands finish without the training cutoff?** add
  `--ee-term-z 100` (relaxes a training-only tracking guard).

### 3.3 If the window does NOT open (X11 / display troubleshooting)
MuJoCo's viewer needs a display + OpenGL. Distrobox usually shares the host display, but:

1. **On the HOST (outside any container)**, allow local containers to use your screen:
   ```bash
   echo "DISPLAY on host: $DISPLAY"      # expect something like :0 or :1
   xhost +local:                          # allow local apps to open windows
   ```
2. **Inside `grasping`**, check the display is visible and GL works:
   ```bash
   echo "DISPLAY in container: $DISPLAY"  # should match the host (e.g. :0)
   python3 -c "import mujoco; print('mujoco', mujoco.__version__)"
   ```
3. **Common errors → fixes:**
   - `GLFWError ... X11: Failed to open display` / `Failed to initialize GLFW` → `$DISPLAY`
     is empty or not allowed. Set it to the host value and re-run `xhost +local:` on the host:
     `export DISPLAY=:0` (use the host's value), then retry.
   - `libGL error` / `failed to load driver` → software GL fallback:
     `export LIBGL_ALWAYS_SOFTWARE=1` then retry.
   - SSH with no screen at all → you can't open a window; use 3.4 instead.

### 3.4 No display? Headless fallbacks
- **Headless numbers only** (no window) — drop `--viewer`; you still get strike stats +
  CSVs but no picture:
  ```bash
  python scripts/mujoco_eval_onnx.py --pd-mode implicit --noise-scales 0.0 --steps 4000 \
    --onnx $RUN/exported/policy.onnx --std $RUN/exported/learned_std.npy
  ```
- **A recorded video** (Isaac, needs no live display) — in `grasping`, the Isaac play
  script can render an MP4 of the swing:
  ```bash
  export HOPE_REPO=/path/to/your/clone   # root of this repository checkout
  cd $HOPE_REPO/hope_training/whole_body_tracking
  source setup_train_env.sh
  hope_isaac_py scripts/play.py task=HOPEPingPong algo=ppo headless=true video=true num_envs=2 \
    checkpoint=$RUN/model_15200.pt \
    'motion_file=[logs/rsl_rl/eval_motion/fh.npz, logs/rsl_rl/eval_motion/bh.npz]'
  # -> writes videos/play/play.mp4 under the run dir; copy it out and play it on the host.
  ```

---

## Copy-paste quickstart (all in order)

```bash
# ===== A) ROS bring-up smoke (container: hope) =====
distrobox enter hope -- bash
export HOPE_REPO=/path/to/your/clone           # root of this repository checkout
ls /opt/ros/                                   # note the distro name (e.g. jazzy)
source /opt/ros/jazzy/setup.bash               # use the name you just saw
cd $HOPE_REPO/hope_ws
colcon build --packages-up-to hope_planner hope_wbc_runner --symlink-install
source install/local_setup.bash
python3 -c "import onnxruntime" || python3 -m pip install onnxruntime
export ONNX=$HOPE_REPO/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch/exported/policy.onnx
stat -c '%s bytes (want ~1243884)' "$ONNX"
./smoke_test_dry_run.sh "$ONNX"

# ===== B) Manual ROS run (container: hope; 4 terminals, prologue in each) =====
# prologue (every terminal):
#   distrobox enter hope -- bash
#   export HOPE_REPO=/path/to/your/clone   # root of this repository checkout
#   source /opt/ros/jazzy/setup.bash
#   source $HOPE_REPO/hope_ws/install/local_setup.bash
#   export ONNX=$HOPE_REPO/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch/exported/policy.onnx
# Terminal A:
ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=1
# Terminal B:
ros2 launch hope_wbc_runner wbc_runner.launch.py mode:=dry_run onnx_path:="$ONNX" csv_path:=/tmp/wbc_runner.csv
# Terminal C (best_effort REQUIRED — planner publishes best-effort; plain echo shows nothing):
ros2 topic echo --qos-reliability best_effort /racket/command
# Terminal D:
tail -5 /tmp/wbc_runner.csv | cut -c1-140

# ===== C) See it swing in MuJoCo (container: grasping) =====
distrobox enter grasping -- bash
conda activate hope-motion-py310
export HOPE_REPO=/path/to/your/clone           # root of this repository checkout
cd $HOPE_REPO/hope_training/whole_body_tracking
RUN=logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch
# if the window fails: run `xhost +local:` on the HOST first, then retry.
python scripts/mujoco_eval_onnx.py --viewer --pd-mode implicit --noise-scales 0.0 --steps 4000 \
  --onnx $RUN/exported/policy.onnx --std $RUN/exported/learned_std.npy
```

**Safety recap:** every step above is dry-run / log-only / standalone-sim. No
`mode:=hardware`, no `hardware_enable`, no joint commands published, no real robot motion.
