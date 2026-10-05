# Runner, Xbox and field operation

The public runtime uses **model_21800**, including its original ONNX, deployment
parameters and 110-input/31-action contract. The training and checkpoint workflow
remains described in [MODEL_21800](../MODEL_21800.md). Runtime features do not
require a newer checkpoint. No unpublished checkpoints, training configurations,
experiment results or robot-specific calibration receipts are distributed here.

## Controls and ownership

One Runner owns every body command, including Stand, Serve, receive and optional
HumanLike locomotion. Foxglove and the controller request state changes; they do
not write motor commands. The gripper bridge runs asynchronously and does not
own body motion.

| Input | Action |
| --- | --- |
| A | Select server and prepare **Start to Serve**; raise the loading hand |
| B | Play the selected serve; Kernel returns slowly to **Stand**, Normal Play enters **Ready / MOTION** |
| X | Select receiver and enter **Ready / MOTION** |
| Y | Enter **Teleop** only when already in **Stand**; other modes require a new press after returning to Stand |
| Hold LT + left stick | Body-frame forward/backward and lateral velocity |
| Hold LT + right stick | Turn |
| Release LT or lose fresh input | Decelerate to zero; leaving Teleop waits for foot settling |
| LB + RB together | Assert the software emergency stop |

A/B/X/Y do not require LT. After entering Teleop, wait for ACTIVE, center the
sticks, release LT and hold it again. The input latch prevents a held stick from
starting locomotion on entry. Xbox status has a fixed display area and does not
resize the panel whenever an input packet becomes stale.

**Reset Software E-stop** stays visible, including after an Xbox stop. It clears
the software latch; it neither releases a hardware/vendor emergency stop nor
starts motion. Hardware recovery remains an operator action.

## Normal Play and Kernel Mode

| | Normal Play | Kernel Mode |
| --- | --- | --- |
| Connection | Laptop, HDU, MDU; Motive stream for localization | Laptop, HDU, MDU; OptiTrack optional |
| Receive localization | Calibrated, timestamped world pelvis pose | Local frame established from IMU heading; stance-foot odometry |
| Incoming ball | Planner's live trajectory and receive command | No live-ball gate when testing mode entry |
| A/B/X/Y and transition controller | Shared | Shared |
| Interpretation | Full receive workflow with valid field geometry | Local actor/transition testing; local odometry is not a venue calibration |

Kernel Mode permits X to enter the selected receive actor without an OptiTrack
calibration. Serve completion instead lowers the arms slowly and returns to Stand. It must not publish a fabricated
world calibration receipt or make Normal Play consume its local pose as mocap.
Both pelvis and torso IMU acquisition remain part of the hardware state contract.

A receive command requires an incoming-flight packet in Normal Play. Seeing a
ball in Foxglove does not by itself schedule a swing. Kernel Mode has no real
ball trajectory when OptiTrack is absent.

## Serve loop and transitions

Stand/Teleop-to-Serve transitions carry position, velocity, feed-forward torque and
PD gains together. The last delivered command seeds the transition. Loading-arm
motion uses bounded pitch support; entry does not add an operator settlement gate.
In Kernel mode, the selected CSV's validated final stance sets the matching Stand width.

In Kernel mode the sequence is **Start to Serve → Serve → slow arm lowering → Stand**.
Normal Play retains the receive-policy handoff. Repeated A requests prepare the next
serve. Y never queues a Stand-to-Teleop sequence during a serve or another mode.
Reconnecting a controller reopens the device; held face buttons require release
and a new press. Neither reconnecting nor finishing a serve selects locomotion.

The Console enables **Enter Teleop** from fresh Stand and connected, neutral Xbox
input with LT released. Its separate Teleop phase/input-age display can reconnect
independently and does not lock the request button. Runner still validates the
loaded policy, current Stand mode and neutral input within its 200 ms watchdog
when processing every request; rejection is shown in the request receipt.

The default serve is:

```text
a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/a3p_op3_serve025_smooth_center_v14.csv
```

This is a named SDK 31-joint CSV with 468 frames at 100 Hz. The default handoff
starts after the final frame (467). Kernel mode uses a 2.5-second full-command return, then measured
settling into Stand. Normal Play uses a one-second return and the existing receive
blend. `--serve-handoff-frame` and `--serve-return-sec` remain explicit options.
`--serve-only` retains complete CSV playback for isolated tests.

**v14 is the attended Kernel-mode serve clip.** Select Kernel before starting it.
Normal Play still requires a CSV whose final pose matches its nominal Stand; the
wide-stance clip fails that preflight. The wide-stance Normal Play/receive loop is
not qualified: a separate simulation stalled during settling, and a trial with a
longer return fell. Those trials are not included in the passing Kernel results.

The v14 clip uses a continuous right-arm spline while preserving the other joints
and release events. Its 70 primary MuJoCo cases (0–80 ms release delay, tested
ball offsets and hold durations) passed; three additional cases at 85 ms failed.
This is not a calibrated hardware hit-rate claim. v13 is rejected due to observed
hardware jitter; historical CSVs remain available for comparison.
The packaged Rockchip profile sends the release command at frame 47; physical jaw
opening and ball detachment are not confirmed by the software publish receipt.

The [gripper URDF](../../a3_deploy/a3_deploy_example/assets/a3_runtime/robots/A3PingPong-with-gripper/README.md)
includes relative mesh paths and documents missing collision-mesh fallbacks.

The field panel can upload a replacement CSV. The MDU validates its complete
schema with the actual Runner before storing it by SHA-256. Load the desired
file while stopped; the next Runner session uses that file. Calibration JSONs
are similarly archived and identified by hash.

## Public package

Use the environment setup in [RUN_ON_AGIBOT](../RUN_ON_AGIBOT.md). Build from
the HOPE deployment directory, not the separate vendor example:

```bash
export HOPE_ROOT="$(git rev-parse --show-toplevel)"
cd "$HOPE_ROOT/a3_deploy/a3_deploy_example"
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh --arch x86_64
# MDU package, after provisioning the documented Rockchip sysroot:
bash scripts/build_a3_deploy_pkg.sh --arch rockchip
```

Both commands select `models/model_21800/policy` and the ping-pong runtime YAML
by default. `--policy-dir` is an explicit operator override. Model and parameters
are copied together to `policy/`; packaged paths are relative. Do not substitute
a development checkpoint when building a public release.

The public runtime includes the matching HumanLike inference bundle at
`assets/a3_runtime/teleop_humanlike/`. The builder includes its `humanlike.yaml`,
`policy.onnx` and `lin_vel_encoder.onnx` by default under `teleop_humanlike/`.
The lifecycle discovers this directory and passes it to Runner, so the standard
public package has the files required for **Enter Teleop**. Runtime readiness
and operator input requirements still apply.

Use `--teleop-policy-dir /absolute/path/to/humanlike` to select another compatible
bundle. `--without-teleop` deliberately builds a Stand/Serve/receive package
without learned locomotion. These vendor-origin inference assets are separate
from the unchanged HOPE model_21800 receive policy and private HOPE training.

The gripper bridge includes declarative E-link command presets in
`scripts/serve_gripper_presets.py`; it does not import a private vendor `grip.py`.
The current lifecycle hardware profile is **A3_T3D0** with E-link hand control.
Other A3 variants require their own checked hardware configuration. Missing
gripper transport is reported separately from arm control.

The lifecycle default MDU slot is `/agibot/a3_deploy_model21800`.
`HOPE_MDU_DEPLOY` in the robot account's
`~/.config/hope-foxglove/lifecycle.env` selects another immutable slot. Copy the
complete package and retain the previous slot before switching. This document
and the package builder do not start a robot automatically.

## Laptop and Foxglove update

Follow the SSH keys, HDU services and network setup in
[the first hardware deployment guide](foxglove_first_hardware_test.md). Replace
site addresses and usernames with your own. Keep `HOPE_ROOT` in
`~/.config/hope-foxglove/lifecycle.env`; no workstation home path is required.

Build the public standalone NatNet packages together with the HOPE workspace:

```bash
cd "$HOPE_ROOT/hope_ws"
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src ../NatNet2ROS2/src \
  --packages-up-to motion_capture_tracking hope_bringup hope_planner_cpp \
  --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
```

The optional AvatarPro path uses `../VRPN2ROS2/src` as an additional base path.
There is one copy of each driver package; do not copy NatNet packages into
`hope_ws/src`.

Update both helpers on the Laptop and the lifecycle helper on each robot host:

```bash
sudo install -Dm755 "$HOPE_ROOT/foxglove/helpers/hope-lifecycle" \
  /usr/local/libexec/hope-lifecycle
sudo install -Dm755 "$HOPE_ROOT/foxglove/helpers/hope-laptop-container-runtime" \
  /usr/local/libexec/hope-laptop-container-runtime
install -d "$HOME/.local/share/hope-foxglove"
install -m755 "$HOPE_ROOT/foxglove/laptop/hope_marker_monitor.py" \
  "$HOME/.local/share/hope-foxglove/"
install -m644 "$HOPE_ROOT/foxglove/laptop/hope_marker_monitor_core.py" \
  "$HOPE_ROOT/foxglove/laptop/marker_monitor.yaml" \
  "$HOME/.local/share/hope-foxglove/"
```

For the HDU control-plane update, install all `foxglove/a3/hope_*.py` modules
into `/usr/local/bin` together, including field assets, field operations, time
calibration. Runner transport and IMU clock telemetry now use the native C++ binaries
from `foxglove/a3/native`; install them using the native-build commands in
[the Foxglove installation guide](../../foxglove/README.md#per-robot-installation-operator-action-changes-that-robot).
Also install `hope-imu-telemetry.service` alongside the updated monitor unit and
keep both bridge YAMLs at `num_threads: 2`. The observer suppresses unchanged
display-status messages with a bounded heartbeat; Runner liveness, mode and fault
status retain their existing rate. Install `hope_field_assets.py` additionally on the MDU at
`/usr/local/lib/hope-foxglove/hope_field_assets.py`. The Laptop helper reads it
and the calibration validators directly from `HOPE_ROOT`.
Update the fixed command/monitor/lifecycle service files and bridge YAMLs as a
set, then reload systemd while the lifecycle is stopped. The telemetry recorder
also needs `hope-pingpong-telemetry-record` and its matching service file.
Do not mix an old proxy or transport relay with the new console.

Xbox input runs in the Laptop ROS environment:

```bash
python3 -m venv --system-site-packages "$HOME/.local/share/hope-xbox/venv"
"$HOME/.local/share/hope-xbox/venv/bin/pip" install evdev
export HOPE_XBOX_HDU_IP='<hdu-ip>'
bash "$HOPE_ROOT/foxglove/laptop/run_xbox_input.sh"
```

For automatic startup, install `foxglove/laptop/hope-xbox-input.service` as a user
service. It reads the same `lifecycle.env`; set `HOPE_XBOX_HDU_IP` in a service
drop-in. The gamepad must be visible inside the `hope` container. `LT`, axes and
A/B/X/Y are read locally, so a Foxglove browser gamepad preview is not required.

Install the ready-made console on the Laptop:

1. Download [hopeopen.hope-a3-console-1.8.10.foxe](../../foxglove/extensions/hope-a3-console/hopeopen.hope-a3-console-1.8.10.foxe)
   from this checkout. On GitHub, use the file's download button.
2. Open the `.foxe` in Foxglove Desktop's Extensions screen to install it.
3. Import `foxglove/layouts/model21800_console.json` and connect to the HDU control
   bridge on port 8766. Confirm the field addresses, mode and P1/P2 selection.

No Node.js build is required to install the committed `.foxe`. To rebuild it:

```bash
cd "$HOPE_ROOT/foxglove/extensions/hope-a3-console"
npm ci
npm run package
```

When publishing a rebuilt console, commit its installer with the matching source
and update download links and the exact `.foxe` filename allowed by the extension's
`.gitignore`. Keep `node_modules/`, `dist/` and superseded installers out of Git.

## Calibration and clocks

The default public marker carrier remains the **24-sticker S01–S24 shell**.
**Cali 24 stickers** validates the canonical optical-center table and its receipt
metadata. **Cali V2** and **Cali V3** are separate legacy ten-ball layouts; select
the actual carrier, never reuse a ten-ball receipt as a 24-sticker receipt.
The marker counter uses the selected body's ModelDef size and counts only live,
finite, non-occluded, point-cloud-solved samples. It supports 24 and 10 markers.

The standalone NatNet adapter keeps its public Ball/P1/P2 output defaults and
200 Hz output limit. HOPE bringup explicitly aliases `P1`/`P2` to
`UCB_P1`/`UCB_P2` and `ball` to `Ball`, retaining exact canonical names when both
exist. Set `HOPE_MOTIVE_INTERFACE_IP` in Laptop `lifecycle.env` to the wired NIC
address. NatNet's transport is independent of Fast DDS transport.

P2 applies the table-side transform consistently to the ball, robot and saved
calibration. Saved JSON must identify the selected side and rigid body. Changing
field mode or table side is a stopped-session operation.

Keep main's [supervised clock package](../../agibot/ntp_sync/README.md).
HDU Chrony and the dedicated HDU/MDU PTP services retain clock ownership.
**TIME CALIBRATION** is an explicit stopped-session operation: it coordinates
NTP/MDU synchronization and restores clock services. Starting a normal session
does not silently run time calibration. Clock diagnostics remain visible.

## Reproducible transition checks

The native simulation probe links the production controllers without HAL or DDS:

```bash
cd "$HOPE_ROOT/a3_deploy/a3_deploy_example"
bash scripts/build_runner_transition_probe.sh /tmp/hope-transition.so
python scripts/validate_serve_loop_mujoco.py \
  --library /tmp/hope-transition.so \
  --policy-dir models/model_21800/policy \
  --output /tmp/model21800-serve-loop.json
```

Use a Python environment with MuJoCo and NumPy. The probe includes both IMUs,
actual command-delivery feedback and the production frame-110/one-second return
settings. It checks command continuity and floating-base stability separately;
it sends no actuator commands and does not qualify real-ball performance.
See [public runtime validation](runtime_public_validation.md) for the results
and outstanding qualification limits of this update.

## Supplied AimDK reference bundle

The A3 Ultra AimDK 3.1 protocols, examples, wheel and AArch64 protocol libraries
are available in [the consolidated SDK directory](../../agi/agibot_a3_Ultra_aimdk-dev3.1/HOPE_INTEGRATION.md).
Main's Runner and declarative gripper presets retain their existing build paths.
