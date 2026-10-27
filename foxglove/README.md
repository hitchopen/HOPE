# Current Runner workflow

ELink startup checks detect known process/IPC and startup-log failures; runtime
health observations are diagnostic-only. Missing telemetry audit/decoders, clock
qualification, or observer updates do not add motion gates. Existing vendor and
Runner protections remain in force. See the
[operational scope and validation limits](../docs/operations/foxglove_lifecycle.md#elink-startup-checks-and-runtime-diagnostics).

For direct installation, use the committed
[Console 1.8.18 installer](extensions/hope-a3-console/hopeopen.hope-a3-console-1.8.18.foxe).
Node.js is needed
only to rebuild the extension, not to install this package.

Normal play verifies P1/P2 from raw mocap positions at START and locks the side
until STOP. A 100 ms loss of the live base stream requests controlled PD_STAND
recovery in native Runner. A marked 20 ms last-pose hold cannot renew real input
freshness or shorten mode admission to 20 ms. Separate challenge echoes expire
after 1 s for the Console and 100 ms for the operator relay in normal play.
Offline Kernel/Pure Serve and Teleop remain exempt. Install
the matching Laptop relays, HDU modules and rebuilt MDU Runner with the panel;
see [deployment safety](../docs/operations/foxglove_lifecycle.md#player-assignment-and-mocap-loss).

The current deployment uses **model17400 + PPQ004** in receive-only mode. See
[current model installation](extensions/hope-a3-console/README.md#model17400-receive-policy)
and the [managed lifecycle](../docs/operations/foxglove_lifecycle.md).
Model17400 does not enable serving or Kernel mode.

Current 17-sticker trousers use a pelvis-aligned Motive import and automatic
asset matching. Console 1.8.13 replaces Venue Calibration with live asset status;
see [the current import/setup procedure](extensions/hope-a3-console/README.md).
The V2/V3 capture descriptions below apply only to legacy calibration mode.

Current Runner/serve behavior, v12 assets, PR status and validation limits:
[Runner, Xbox and serving](../docs/operations/runtime_xbox.md).
The public [Console 1.8.18 installer](extensions/hope-a3-console/hopeopen.hope-a3-console-1.8.18.foxe)
includes Mac/Linux, gripper and NatNet/VRPN selectors and matches the checked-in source.
Combined lifecycle schema 6 preserves existing field schema-5 settings; source
and image builds do not overwrite or restart the live HDU backend.

# HOPE A3 Foxglove Operator Interface

> **Model17400 + PPQ004 operator stack.** This directory combines monitoring,
> OptiTrack/marker, pelvis TF and E-stop support with the native Runner contract.
> The obsolete 8-double TTY adapter and
> `/hope/control/*` process-control path have been removed; they must not be
> restored alongside the native Runner. Use the opt-in `8766` bridge, the services under `/hope/runner/*`,
> and `layouts/model21800_console.json`; the exact mapping and deployment
> boundary are in `docs/operations/foxglove_runner_integration.md`.
> The copy/paste deployment procedure is
> `docs/operations/foxglove_first_hardware_test.md`.

For a Mac operator alongside an existing Linux laptop, see
[macOS operator setup and explicit profiles](../docs/operations/foxglove_lifecycle.md#macos-operator)
and [local build commands](../docs/operations/macos_deployment.md). Any normal Mac
login can operate it; no personal or administrator account is hardcoded. The Mac
container and HDU service identities are configured independently. This does not
change robot scheduling or motion assets.

Rules and procedures for monitoring **any** Agibot A3 unit from a laptop on
the same subnet: NTP offset, aggregate CPU utilization, ROS-message timestamp
latency, vendor process and pelvis-TF readiness. The operator layouts contain
no robot URDF or robot TF-tree rendering; only the static table URDF and the
sanitized Pelvis link are shown.
The fleet endpoint exposes one audited action:
assert E-stop. In the integrated Runner deployment the E-stop requests the vendor
software latch and independently requests native Runner PASSIVE, but cannot
release either latch. All other Runner actions use the separate attended
control endpoint documented in `foxglove/README.md`. Motive/NatNet conversion,
v3 17-sticker calibration, the calibration JSON, and
HOPE base-pose reconstruction all stay on the external computer. The A3 never
connects to or probes the Motive host and never stores, reads, or receives the
JSON; it consumes the computer's `/a3/base_pose_flat` output.

Use the current pelvis-aligned **P1** asset and retained station set in the
[v3 setup procedure](../agibot/README.md#v3-sticker-p1-to-pelvis-calibration).
Its intended identity correction still requires fresh live verification;
old receipts must not be reused. See [calibration receipts](../calibration/README.md).

On the attended `8766` console, `/hope/calibrate` and
`/hope/refresh_x_hit` are deliberately separate. Calibration atomically
replaces the Laptop JSON with the fixed `P1 -> pelvis_link` result plus a
stationary `world -> pelvis_link` audit snapshot, then waits for the matching
live base receipt. Refresh x_hit touches only the Planner request/status files.

This document is site-neutral. The operator must provide and verify both
distinct Wi-Fi addresses whenever the robot, Laptop, or venue changes:

1. Shell sessions use the HDU address in `A3_HOST`.
2. `fastdds_bridge_profile.xml` uses UDPv4 without shared memory but does not
   pin a local IPv4 address. Runner supplies validated peers at startup.
3. `/etc/hope-foxglove/network.env` supplies the Laptop peer selected for the
   current venue.
4. The Foxglove connection dialog takes the HDU address as
   `ws://<HDU-IP>:8765` or, for the attended console, port `8766`.

> **Nothing in this folder is installed on a robot by itself.** The `a3/`
> directory contains staged deployment assets; a robot changes only when an
> operator runs the per-robot installation procedure below.

## Rules

- **R1 — One bridge per robot, on the robot.** The Foxglove bridge runs on the
  robot and serves a plain-TCP WebSocket over Wi-Fi. Separately, the external
  computer is an explicit FastDDS peer: it owns the calibration service and
  publishes `/a3/base_pose_flat`. Foxglove client-topic publishing remains
  disabled; it is not the base-pose transport.
- **R2 — Least-privilege control.** Client publishing, parameters, and remote
  assets remain disabled. The fleet service allowlist contains exactly one
  `std_srvs/Trigger` endpoint: the assert-only vendor E-stop proxy. The monitor advertises the E-stop proxy
  only while the live vendor `HalEmergencyService/SetEmergencyCommand` RPC is
  matched. It can only encode `software_emergency_stop=true`; it contains no
  reset/release path.
- **R3 — Additive only.** Two dedicated fleet-monitoring HDU systemd units; no
  vendor runner file is modified. The Runner observer, command proxy and 8766
  bridge are separately staged opt-in units. Disabling the units removes their runtime effect; the documented
  removal procedure separately cleans installed and build artifacts.
- **R4 — Bandwidth is opt-in.** The topic whitelist excludes H.265 camera
  and lidar streams by default. Add them per-robot, deliberately.
- **R5 — Verify per unit before trusting.** Fleet defaults below were
  verified live on one A3 (2026-08-04). Robots on different vendor software
  versions must pass the per-unit verification checklist before the layout
  is trusted.
- **R6 — Mocap and optional marker-CAD calibration stay computer-side.** The
  external computer receives NatNet from Motive or VRPN from ChingMu and sends
  ROS2 to the HDU; the MDU needs no mocap-server route. NatNet alone supplies
  `/optitrack/rigid_body_markers`; VRPN uses the explicitly confirmed pelvis/world
  setup in [mocap setup](../docs/operations/foxglove_mocap_setup.md), not a fabricated
  live marker audit. If an approved NatNet setup procedure runs the
  optional P1 marker-CAD calibration, only that computer writes
  `calibration/ucb_robot_to_pelvis.json`. The receipt includes the fixed extrinsic and
  a stationary world-pelvis audit snapshot; the latter is not a static runtime
  TF. Its base-pose relay reads the fixed extrinsic and
  publishes `/a3/base_pose_flat`; the A3 does not consume the marker stream or
  JSON and never attempts to reach the mocap network.
- **R7 — PM state is not TF readiness.** `/hope/vendor/agibot_pm_active`
  reports the HDU's local systemd unit literally. On split HDU/MDU deployments,
  the HDU unit can be active while MDU-owned joint/TF/localization processes are
  absent. `/hope/vendor/tf_ready` is the authoritative layout gate for the 3D
  pelvis indicator.

## Fleet defaults (verify per unit)

| Item | Fleet default | Verify with |
|---|---|---|
| ROS 2 distribution | Jazzy at `/opt/ros/jazzy` | `ls /opt/ros` |
| DDS domain | `ROS_DOMAIN_ID=232` | `grep -r ROS_DOMAIN_ID /opt/agibot/entry/env/` |
| DDS interfaces | HDU-MDU internal link plus the unit's Wi-Fi interface for the explicit computer peer | staged FastDDS profile and vendor `ros_dds_configuration.xml` |
| TF topics | `/tf`, `/tf_static` (`tf2_msgs/TFMessage`) | `ros2 topic type /tf` |
| Odometry | `/agivslam/localization/odometry` (`nav_msgs/Odometry`) | `ros2 topic list` |
| chrony | installed per `agibot/ntp_sync` | `chronyc tracking` |
| Software E-stop service | live `HalEmergencyService/SetEmergencyCommand`, conditionally wrapped by `/hope/safety/trigger_estop` | `/hope/safety/estop_ready`; do not test by firing it |

```text
A3 unit                                          External computer
  AimRT apps ⇄ iceoryx (local SHM)
            ⇄ ROS2/FastDDS domain 232  ──┐
  hope-monitor C++ (health, E-stop, scene) ┤
  fleet foxglove_bridge ─────────────────┴── ws://<robot-ip>:8765 ──► monitoring / E-stop
  Runner observer + command proxy ◄──► native Runner request/state
  attended control bridge ─────────────── ws://<robot-ip>:8766 ──► A3 Console

Motive ── NatNet ──► adapter ── v3 17-sticker calibration ──► calibration/ucb_robot_to_pelvis.json
```

## Folder contents

```text
foxglove/
|-- README.md                        this rulebook
|-- layouts/
|   `-- a3_monitor.json              Foxglove layout (import on the laptop)
|-- assets/
|   `-- hope_ping_pong_table.urdf    static table visualization asset
|-- laptop/                          Laptop marker and local-asset services
|-- helpers/
|   `-- hope-lifecycle               fixed three-machine lifecycle helper
|-- tests/                           maintained regressions; not installed on robots
`-- a3/                              staged robot-side assets (NOT installed)
    |-- build_foxglove_bridge.sh     pinned ROS 2 bridge build vs /opt/ros/jazzy
    |-- bridge_params.yaml           read-only fleet endpoint on 8765
    |-- bridge_params_control.yaml   attended control endpoint on 8766
    |-- fastdds_bridge_profile.xml   UDPv4-only, address-agnostic host profile
    |-- network.env.example          one-place Laptop static-peer configuration
    |-- native/monitor.cpp           native health, scene and assert-only E-stop monitor
    |-- native/mocap_config_transport.cpp read-only native configuration snapshot relay
    |-- hope_observer.py             normalized Runner/operator state publisher
    |-- hope_command_proxy.py        fixed Runner and calibration service proxy
    |-- hope_lifecycle_supervisor.py fixed process lifecycle authority
    |-- hope_player_positions.py    raw player-end evidence for managed START
    |-- hope_elink_health.py         passive ELink process/listener observation
    |-- hope_elink_fault.py          startup-log checks and legacy fault diagnostics
    |-- hope_elink_telemetry.py      source-progress/freshness contract and tracker
    |-- hope_elink_telemetry_ros.py  passive vendor telemetry subscriptions
    |-- hope_elink_clock.py          boot-bound original chrony-probe freshness
    |-- hope-foxglove-bridge.service read-only fleet bridge
    |-- hope-foxglove-control-bridge.service
    |-- hope-monitor.service
    |-- hope-observer.service
    |-- hope-command-proxy.service
    |-- hope-lifecycle-supervisor.service
    `-- patches/                     pinned A3 ament-index compatibility patches
```

## ELink startup checks and diagnostics

Managed startup performs passive process/listener checks,
rejects known failure signatures in current-session startup logs, and verifies
vendor restoration after cleanup. A failed check rejects that attempt with a
retryable error; it does not create a permanent motion lockout or veto KILL.
Runtime observations are diagnostic-only: missing, stale or faulty observations
do not change Runner modes, cancel swings, require an extra serve-continuation
action, or block normal commands. There is no ELink permit transport.
`OBSERVED` is **not** proof of fresh hand/BMS/E-stop telemetry or safe native HAL
recovery. An optional audited telemetry contract improves diagnostic evidence;
an absent/unaudited contract or unavailable decoder does not block START.
See [ELink operational scope](../docs/operations/foxglove_lifecycle.md#elink-startup-checks-and-runtime-diagnostics)
for matching diagnostic modules, commissioning and validation limits.

Battery health remains A3's responsibility: BMS voltage, pack-presence and
reported battery faults are diagnostic only, not HOPE motion vetoes or durable
ELink faults. Freshness and source progression remain diagnostic checks, with
strictly advancing source timestamps supported for the vendor's unset BMS
sequence (`seq=0`). This policy does not override vendor emergency protection
or existing Runner command-fault handling. Legacy fault records are read-only
diagnostics, not admission authority, and are not automatically erased.

Only the documented runtime modules and helpers are installed on HDU/MDU;
the regression tests stay in the source checkout. Generated review reports,
live captures and scratch build outputs are not deployment assets.

## Startup background cleanup

Every managed START (including KERNEL mode) completes read-only preflight, then
runs the mutating HDU `BACKGROUND` step before launching Planner, HAL or Runner.
HOPE deliberately stops only `hal_hdu_camera` and exact matched camera-process
leftovers. These are application-side shutdowns, not proof of vendor crashes.
`agent`, `hal_audio`, their watchdogs, `hal_remote` and `agibot_ui.service` are
not cleanup targets, and their presence does not reject START. Earlier cleanup
stopped startup/audio/UI dependencies; a missing `agent` was reproduced as a
vendor startup rollback. Preserve the control/emergency dependencies and vendor
automatic-mode policy rather than disabling them as a CPU optimization.

The guard calls the local vendor process manager's StopApp API for the camera
only. It sends SIGTERM to exact camera command/config matches
left outside the manager, checking process identity with pidfds. It requires a
continuous two-second quiet window; RPC errors, failed stops or respawning
allowlisted processes reject START with
`UNRELATED_BACKGROUND_NOT_STOPPED`. The HDU background-step timeout is 180 seconds to
cover the bounded vendor calls. Drive HAL, process manager, emergency-stop,
local networking, clock and recording processes are outside the cleanup allowlist.
The optional Wi-Fi-only profile additionally stops the cellular watchdog; see
[robot Wi-Fi/runtime setup](../docs/operations/robot_runtime_policy.md).

Deploy `helpers/hope_catch_background.py` beside the HDU's installed
`/usr/local/libexec/hope-lifecycle`, together with the matching helper and
`a3/hope_lifecycle_supervisor.py`. Preserve each robot's existing hardware,
path and network configuration when updating the helper. The guard requires
Linux/Python pidfd support and access to the existing localhost vendor process
manager on port 50081. An active vendor UI does not reject START, and normal
cleanup neither probes nor stops it. The base cleanup does not install sudo
rules or change vendor launch defaults. Existing disabled UI units, HOPE
boot-blocking conditions and missing vendor app registrations require a
separate, backed-up repair; installing this helper does not repair them.
See [handheld preservation and upgrade checks](../docs/operations/robot_runtime_policy.md#vendor-handheld-and-status-display).
The former UI-stop-only `70-upstream-startup-sudo.conf` override is no longer
needed; follow [supervisor hardening recovery](../docs/operations/robot_runtime_policy.md#restoring-supervisor-hardening)
to remove only that backed-up override and verify effective/running
`NoNewPrivileges` protection. The optional cellular stop already supports it.
Physical E-stop continuity during managed HAL handover and clock maintenance
is not established by this fix. Until qualified during HOPE play, credit only
the verified wired E-stop. The [required live START/STOP and clock-restore test](../docs/operations/robot_runtime_policy.md#required-live-validation-before-deployment)
must also check whether restoring vendor services automatically activates
`Manual`/native `mc` or joint control after STOP; HOPE `STOPPED` is not proof
that the drives are disabled.
The optional Wi-Fi-only installer permanently disables cellular and adds a narrow
polkit stop rule. HDU preflight also requires the installed Wi-Fi policy helper;
install its service/configuration together with the updated lifecycle helper.

Each attempt writes `/tmp/hope_background_<session_id>.json`, including failure
details. Successful session preparation copies it to
`/tmp/hope_real/<session_id>/hdu/background_cleanup.json` alongside the normal
session evidence. Failed background-cleanup receipts remain at their original path.
The cleanup runs again on the next START so vendor defaults can remain intact.

Local validation (all process/RPC operations mocked):

```bash
python3 -m pytest -q foxglove/tests/test_catch_background.py foxglove/tests/test_lifecycle_core.py
bash -n foxglove/helpers/hope-lifecycle
```

## Per-robot installation (operator action; changes that robot)

**Preserve the installed control-plane layout.** These service/module examples
are not a migration recipe for an existing consolidated field installation.
Inspect the supervisor, observer and command-proxy units and their actual
processes first. If `hope-lifecycle-supervisor.service` owns the Python
`/usr/local/bin/hope_operations.py` process, update its matching field bundle;
do not replace it with split sample units/modules. Loaded-inactive
observer/proxy compatibility stubs may be intentional and must not be enabled
just to satisfy a split-layout checklist. See the
[deployment-layout boundary](../docs/operations/foxglove_lifecycle.md#install-the-hdu-supervisor)
and [topology-aware clock workflow](../docs/operations/foxglove_lifecycle.md#pre-operation-clock-correction).

Set the target once per session; every command below uses it:

```bash
export A3_HOST='<robot-ip-or-hostname>'
```

### 1. Stage and build foxglove_bridge (once per robot)

> **Confirmed Stage 1 installation (2026-08-06).** One A3 running the vendor
> Debian 12 / ROS 2 Jazzy image was staged, built, and smoke-tested successfully.
> The installed bridge started as Foxglove Bridge `3.4.3`, listened on
> `0.0.0.0:8765`, and shut down cleanly on `SIGINT`. The build used
> `FOXGLOVE_BRIDGE_REMOTE_ACCESS=OFF`. This is a unit-specific confirmation,
> not a fleet-wide compatibility claim.

> **Revalidation status.** The compatibility work from the confirmed unit is
> now captured in `build_foxglove_bridge.sh` and its checked-in `patches/`.
> Patch application and resulting source hashes are tested locally against both
> pinned upstream commits. The revised one-command path still requires the
> normal per-unit verification below before it is trusted on another robot; it
> is not an unattended fleet rollout mechanism.

No compatible Jazzy apt repository is assumed on the vendor Debian 12 image.
The script is intended to build the official ROS 2 implementation from the
pinned Foxglove `ros-v3.4.3` release
(`05f27efc7e535d9c30c6b0cb4f6aa89de7243870`) with remote access disabled. The
robot must already carry `g++`, `cmake`, `colcon`, and the Jazzy development
tree:

```bash
ssh "agi@$A3_HOST" 'mkdir -p ~/foxglove_a3'
scp -r foxglove/a3/. "agi@$A3_HOST:~/foxglove_a3/"
ssh "agi@$A3_HOST" 'sudo apt-get install --no-install-recommends -y \
  git ca-certificates libssl-dev zlib1g-dev rapidjson-dev && \
  bash ~/foxglove_a3/build_foxglove_bridge.sh'
```

Result on the robot:
`~/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/`.
The build stops if either existing source checkout does not match its pinned
commit. It never silently replaces or updates a checkout, accepts only the two
expected compatibility modifications, and rejects untracked or additional
changes.

The confirmed installation has these identities:

| Item | Confirmed value |
|---|---|
| Foxglove source | `ros-v3.4.3` at `05f27efc7e535d9c30c6b0cb4f6aa89de7243870` |
| `rosx_introspection` source dependency | `3.1.1` at `ab747a0d3970d3297a5652b82e7645ab1d11feb9` |
| Foxglove compatibility patch | SHA-256 `1c6f40f6af4fe0186f196f65fb04c4d79b585ae16df448f16c72e09887d58828` |
| `rosx_introspection` compatibility patch | SHA-256 `bd6541d663b57505cc083b7d67aa5593c6297928adee04ed72e64ae35f6e4da5` |
| Patched Foxglove source file | SHA-256 `ff879cd712a4d167169c5d229a0f67e2c112f42b65ecb62404d6dc51cb44a8f1` |
| Patched `rosx_introspection` source file | SHA-256 `c3100994dea0fdc6dc2b87614ed35b1582d95819cd3f714a062910da64e36d16` |
| Installed bridge | `~/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/` |
| Bridge executable SHA-256 | `c9aeed8fb5ae95d06927ad1ef9e87b0a5f95c7ec7611ef90f6120f42c80411a1` |
| CMake remote-access setting | `FOXGLOVE_BRIDGE_REMOTE_ACCESS:BOOL=OFF` |
| Smoke test | Server started on port `8765`; timed `SIGINT` shutdown was clean |
| Stage 2 state | Installed and verified separately below |

Verify the installed result without starting a persistent service:

```bash
ssh "agi@$A3_HOST" '
  source /opt/ros/jazzy/setup.bash
  source ~/hope_foxglove_ws/foxglove-sdk/ros/install/setup.bash
  ros2 pkg prefix foxglove_bridge
  ros2 pkg executables foxglove_bridge
  sha256sum \
    ~/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/lib/foxglove_bridge/foxglove_bridge
  grep "^FOXGLOVE_BRIDGE_REMOTE_ACCESS:BOOL=" \
    ~/hope_foxglove_ws/foxglove-sdk/ros/build/foxglove_bridge/CMakeCache.txt
'
```

#### Captured vendor-image compatibility

The A3 vendor Jazzy tree supplies `ament_index_cpp` `1.8.1`, which does not
contain `ament_index_cpp/version.h`. The pinned Foxglove Bridge and
`rosx_introspection` sources otherwise select newer APIs using that header.
`build_foxglove_bridge.sh` now closes all three gaps found during the first
installation:

1. It sources `/opt/ros/jazzy/setup.bash` before enabling Bash nounset mode.
2. It fetches `rosx_introspection` at the exact confirmed commit and builds it
   in the same colcon invocation as Foxglove Bridge.
3. It checksum-verifies and applies narrow patches that unconditionally select
   the `ament_index_cpp` 1.8-compatible share-directory and resource APIs. It
   then verifies the complete patched source-file hashes before building.

The first unit's executable SHA-256 above remains the confirmed reference. Do
not treat a binary mismatch on another compiler/sysroot as sufficient proof of
a functional difference; verify source identities, CMake settings, startup,
shutdown, topics, and service restrictions using this checklist.

### 2. Install the bridge and monitor services

The calibration JSON and base-pose relay are computer-side components. Do not
create a calibration directory or install a JSON-reading world service on the
robot.

```bash
ssh "agi@$A3_HOST"
# Build native telemetry, transport and monitor (or use a target-matched build).
# This step does not start a process or issue a robot command.
source /opt/ros/jazzy/setup.bash
# Use the real installed vendor SDK, never the isolated-test mock package.
export CMAKE_PREFIX_PATH="/opt/agibot/share/ros2_package/aimrt_protocol_ros2_package:${CMAKE_PREFIX_PATH}"
cmake -S ~/foxglove_a3/native -B ~/hope_field_native_build \
  -DCMAKE_BUILD_TYPE=Release -DHOPE_MONITOR_REQUIRE_VENDOR_RPC=ON -DBUILD_TESTING=OFF
cmake --build ~/hope_field_native_build -j2
# Qualify from a full checkout using the isolated test instructions below.
# This deployment build contains only runtime executables, not test tools.
sudo install -D -o root -g root -m 0755 ~/hope_field_native_build/hope-monitor \
  /usr/local/libexec/hope-monitor
sudo install -D -o root -g root -m 0755 ~/hope_field_native_build/hope-imu-telemetry \
  /usr/local/libexec/hope-imu-telemetry
sudo install -D -o root -g root -m 0755 ~/hope_field_native_build/hope-runner-transport-relay-native \
  /usr/local/libexec/hope-runner-transport-relay
sudo install -D -o root -g root -m 0755 ~/hope_field_native_build/hope-mocap-config-transport \
  /usr/local/libexec/hope-mocap-config-transport
# The matching lifecycle helper executes the relay directly (without python3).
# Deploy that helper together with the native relay; preserve venue configuration.
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/bridge_params.yaml \
  /etc/hope-foxglove/bridge_params.yaml
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/fastdds_bridge_profile.xml \
  /etc/hope-foxglove/fastdds_bridge_profile.xml
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/network.env.example \
  /etc/hope-foxglove/network.env
# Edit this one file and set ROS_STATIC_PEERS to the Laptop Wi-Fi address.
# Re-check it after every venue or network change.
sudoedit /etc/hope-foxglove/network.env
# Other Python services still import the pure shared helpers. Keep this module;
# hope_monitor.py itself is a regression/reference implementation, not launched.
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_monitor_core.py \
  /usr/local/lib/hope-foxglove/hope_monitor_core.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope-foxglove-bridge.service \
  /etc/systemd/system/hope-foxglove-bridge.service
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope-monitor.service \
  /etc/systemd/system/hope-monitor.service
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope-imu-telemetry.service \
  /etc/systemd/system/hope-imu-telemetry.service

# If this unit's TF tree differs from the fleet defaults, edit only the
# pelvis_frame and reference_frame values in the installed monitor unit.
# No Motive/mocap IP is configured on the A3.

sudo systemctl daemon-reload
sudo systemctl enable --now hope-imu-telemetry.service hope-monitor.service hope-foxglove-bridge.service
systemctl status hope-monitor hope-foxglove-bridge --no-pager
```

High-rate IMU reception, clock display publication, and Runner/teleop transport
run in C++. A fixed native wait set receives IMU data independently of the 20 Hz
display timer. Original HDU `CLOCK_MONOTONIC` receipt time determines freshness;
the three `/hope/clock` display topics have one native publisher. The native
`hope-monitor` service owns the independent safety endpoints and slower health checks. A lost
IMU makes freshness false after 0.5 s; a lost native process makes the Console's
latency stream stale after its existing 0.5 s receipt timeout. The relay starts after Runner, retains
the original payload, and never replays commands on a timer. Both bridges use two
executor threads. The laptop samples Xbox input at 50 Hz; TELEOP, enabled input,
and mode transitions retain that rate, while inactive heartbeats and GUI preview
use 10 Hz with immediate state edges. Runner's 200 ms watchdog is unchanged.

### Native monitor migration and verification

`hope-monitor.service` launches `/usr/local/libexec/hope-monitor`, built from
`a3/native/monitor.cpp`. Its ROS node name, topic types, six Trigger services,
configured frames, publication periods and `/hope/ntp/qualification` schema are
preserved, with the explicit configuration/status differences below.
Chrony/systemd/CPU probes run on four persistent, single-job workers outside ROS
callbacks with bounded timeouts; probe exceptions (including failure to create a
worker thread) report unavailable health rather than restarting the safety service.
The independent emergency callback can still request both the vendor
assert-only RPC and Runner emergency-PASSIVE, including when another probe is
slow. No startup motion, service-manager handover, or automatic E-stop reset is
introduced. `native_clock_topics=true` still leaves the clock topics exclusively
with `hope-imu-telemetry`.

An accepted E-stop immediately inhibits normal mode publication. A separate
atomic assertion flag refuses duplicate triggers, **not** triggers arriving during
reset: those wait for the running reset and then assert both available stop paths.
Pending assertions keep mode publication inhibited even as the preceding reset
clears its old latch. Reset during an assertion (including one waiting for reset)
is refused. Assertion/reset stay serialized through persistence and RPC completion.
An already-running reset/unlink can delay the new assertion; this is not a bounded
physical-stop latency guarantee.

After any preceding reset finishes, the monitor attempts the available stop send
calls **before its own** shared-state cleanup, status/log publication or latch
file/directory sync. A separate short dispatch lock orders an already-started mode
**send call** before stop **send calls**; no health probe, filesystem operation or
RPC wait holds that lock. Topic and service requests use separate DDS writers,
so this does **not** guarantee their arrival/execution order at the Runner. The
Runner's own emergency inhibit remains necessary; these legacy `/hope/control/*`
services are not exposed by either bridge. Vendor discovery grace never delays an
available Runner request. Slow assertion-latch sync can delay the Trigger response
but not the preceding stop send calls. Persistence failure retains the in-memory
inhibit. No software response independently proves actuator stop.

The HDU status dot defaults to `hope-lifecycle-supervisor.service`, matching the
consolidated field deployment. A split installation should explicitly set
`hdu_runtime_unit:=hope-observer.service`; this differs from the repository Python
reference default, not the backed-up field configuration. Network-change
detection retains Python's **global-scope IPv4** behavior (kernel scope, not an
address-prefix guess); link-scope-only changes do not restart the monitor.
The monitor and mocap-config relay now use one bounded reader in
`native/network_addresses.hpp`: the monitor projects global-scope
index/name/address identity; the relay projects all-scope name/address pairs.
A disappearing interface name makes a dump incomplete instead of fabricating an
address removal: the monitor's periodic poll warns/retries, while the relay keeps
its existing clean-exit/restart-on-probe-failure behavior. Both projections have
comparison tests against `ip -j -4 address show`; scope filtering also has native
regression coverage.
Backend full/partial/unavailable transitions, assert/reassert outcomes and
calibration start/success-SHA/failure are journalled. Malformed inputs and failed
chrony/CPU probes produce throttled warnings. Error codes and missing-detail
fallbacks are retained; descriptive labels do not name obsolete models or marker
layouts.

Runtime edits of clock limits, freshness/calibration timeouts, frames and monitored
systemd unit names take effect on subsequent observations. Topics/client names,
latch path, native-clock ownership, TF-fallback choice and timer periods/rates are
startup-only **read-only ROS parameters**: changing them requires a stopped-state
monitor restart, rather than an acknowledged but ignored parameter update.

For an existing robot, first build against its **real** vendor SDK with
`HOPE_MONITOR_REQUIRE_VENDOR_RPC=ON` and `BUILD_TESTING=OFF` (the defaults).
Back up the installed monitor unit, its drop-ins,
old executable/source and hashes. Do not replace venue configuration or the
consolidated operations supervisor with sample units. Preserve every installed
monitor parameter/environment setting; replace only the old Python invocation
with `exec /usr/local/libexec/hope-monitor` and install the matching binary.
Keep the old unit/source as a rollback copy rather than deleting it first.

Switch only during an attended, stopped maintenance interval: fresh lifecycle
`STOPPED`, lifecycle/time-calibration not busy, no HOPE Runner, and a mechanically
supported/stopped robot with an operator at an independently verified wired
E-stop. Recheck immediately before the monitor-only restart. This briefly removes
the monitor's software endpoint; do not perform it during play or while an
emergency request is outstanding. Do not restart vendor PM, HAL, operations,
controller, or clock services as part of this change. Never clear
`/var/lib/hope-monitor/estop-latched` to make a performance test pass. Run only
one production monitor; any side-by-side candidate must isolate **all** outputs,
control services and command clients from the robot graph.

After the switch, verify the live executable, zero unexpected restarts, exactly
one publisher per monitor topic, qualified clock evidence with advancing original
observation times, correct stale/future-pose rejection, persistent latch state and
the advertised partial/full backend distinction. Compare CPU over matched
intervals, not while building. If compatibility or backend readiness regresses,
restore only the backed-up monitor unit/executable and restart that unit while
still stopped. Process presence/RPC readiness does not prove physical E-stop
operation; this port does not resolve or qualify the existing handheld path
through the MDU PM handover.

Maintained offline checks (no robot access): CTest exercises native parsing,
assert-only protobuf bytes, malformed replies, bounded probes, persistence and
persistent-worker reuse/error recovery/bounded admission.
`tests/test_native_monitor_ros.py` checks the ROS graph, high-rate pose input,
source timestamps/staleness, legacy calibration sequencing, mock dual stop paths,
reassertion, persistence/restart, explicit reset, Runner-only handover, vendor and
Runner timeouts/rejections, concurrent assertions/reset, runtime parameters and
health continuity under busy callbacks and deliberately slow storage. A test-only
preload library injects slow `fsync`, paused reset/unlink and CPU-read failure; it is never installed
or linked into the runtime. These ROS tests refuse to run unless localhost-only
domain **219** is explicitly selected. The unit-file test is separate and runs
without ROS. Native core tests live under `tests/native`; all test drivers,
fault injection and mock RPC sources stay under `tests`, outside the deployment
sources. `BUILD_TESTING=ON` explicitly opts into these tools and requires the full
matching checkout. To test in an isolated ROS development environment:

```bash
task_dir="$(mktemp -d /tmp/hope-monitor-test.XXXXXX)"
source /opt/ros/jazzy/setup.bash
colcon --log-base "$task_dir/log" build \
  --base-paths foxglove/tests/fixtures/monitor_rpc_mock \
  --build-base "$task_dir/mock-build" --install-base "$task_dir/mock-install"
source "$task_dir/mock-install/setup.bash"
cmake -S foxglove/a3/native -B "$task_dir/native" \
  -DCMAKE_BUILD_TYPE=Release -DHOPE_MONITOR_REQUIRE_VENDOR_RPC=ON -DBUILD_TESTING=ON
cmake --build "$task_dir/native" -j2
ctest --test-dir "$task_dir/native" --output-on-failure
env -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 HOPE_NATIVE_BIN_DIR="$task_dir/native" \
  python3 -m pytest -c /dev/null -p no:cacheprovider \
  foxglove/tests/test_native_monitor_ros.py foxglove/tests/test_native_transport_ros.py
env -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=229 HOPE_NATIVE_BIN_DIR="$task_dir/native" \
  python3 -m pytest -c /dev/null -p no:cacheprovider \
  foxglove/tests/test_mocap_config_transport_ros.py
```

The mock package is **test-only**: never install it or its linked test binaries
or the fault-injection library on A3. Production must use the actual vendor SDK.
An explicit `-DHOPE_MONITOR_REQUIRE_VENDOR_RPC=OFF` allows a degraded development
build without that SDK; it must not replace a production monitor.
`hope_monitor.py` remains a non-launched compatibility/reference fixture;
`hope_monitor_core.py` is still
imported by other Python services and must remain installed. Port shared parsing,
wire-schema and health-rule fixes to the C++ and Python helpers with matching
regression tests in the same change.

**Latest stopped-state verification, 2026-10-27:** validation passed 21 local
native monitor/transport tests, seven config
relay tests and 225 selected compatibility regressions; A3's real-SDK build passed
19 isolated monitor tests, seven relay tests and ten consecutive core-test runs.
After explicit deployment/support/stopped/operator confirmation, only the two
binaries were replaced and those two services restarted. Their units, peer
drop-in and venue settings were byte-identical to the backups. Latest installed
SHA-256 values are:

- `hope-monitor`: `375f27503878e6f3caf59a3648a8a69c62e7b531091869d36571d61a7dabc60f`
- `hope-mocap-config-transport`: `f15f7dd30d3f64a540f2216cc8158716c63ea4ecc80b84fd6b279763a2a5f84e`

The preceding binaries/units are recoverable under
`/var/backups/hope-native-review-followup.w209qK` on HDU. Read-only post-switch
checks confirmed STOPPED/idle, zero unexpected service restarts, unique
publishers, advancing qualified NTP evidence and unchanged readiness/latch
verdicts. The config relay forwarded 31/31 original snapshots in 15 seconds with
the same payload hash as before. Operations, IMU, both vendor PMs and handheld
service PIDs were unchanged; the Python transport remains absent from deployment.
No live E-stop assertion/reset, motion or START/STOP qualification was performed.
Send-call ordering is not a cross-writer DDS receive-order or physical-stop proof.
The reset/trigger regression was reproduced against the old binary in isolation
and verified fixed, including five focused repetitions. Both network projections
were compared with `ip -j -4` on Linux and A3. Physical network disconnect,
long-duration operation and active-play qualification remain separate.
Short monitor CPU samples were 12.7% of one core for Python and approximately
8–10% for C++; differing workloads and short windows do not establish an endurance
or active-play speedup.

### Native mocap configuration transport

The optional `hope-mocap-config-transport.service` now launches
`/usr/local/libexec/hope-mocap-config-transport`, built with the other native tools
above. It remains a **read-only** late-started HDU DDS participant, not a new
lifecycle/configuration authority. It forwards each `std_msgs/String` from
`/hope/lifecycle/config/mocap` to `/hope/lifecycle/config/mocap_transport` unchanged,
with reliable/volatile depth-10 QoS. JSON, timestamps, revisions, whitespace and
even malformed input are not decoded or rewritten. There is no startup snapshot,
timer replay, retained config, or parameter/control service. A framework-only ROS
type-description query may be present.

Every second it compares sorted non-loopback `(interface name, IPv4 address)`
pairs, including all address scopes, matching the replaced implementation.
The native implementation uses a bounded kernel address dump, not an `ip`
subprocess per poll. An address change or failed probe exits cleanly with status
1; the existing `Restart=on-failure` / `RestartSec=2` recreates **only this relay**.
It never restarts the supervisor or changes its hardware lock/session. Input loss
still expires the Laptop receiver's existing three-second configuration freshness
check; the relay must never manufacture freshness. Input/output remapping to the
same topic is rejected to prevent a self-loop.

This service remains optional: do not enable it automatically on installations
that do not use it. For an existing deployment, back up its unit, **all drop-ins**,
old script/binary and hashes. Preserve the installed environment, peer wrapper,
ROS domain and network configuration. Replace only the Python invocation in the
effective ExecStart (including any drop-in override) with the native executable;
do not overwrite venue peer settings with the example unit. Switch only with
fresh lifecycle STOPPED/idle, no Runner, and supported/stopped A3 with an operator
at a verified wired E-stop. Run `daemon-reload` and restart only this transport
unit. Other monitoring, operations, vendor and controller services must remain
untouched.

Verify exactly one native relay child (the unit's MainPID may be its waiting
network wrapper), unique source/output publishers and byte-identical snapshots
at the original rate. Do not confirm a new config or publish synthetic snapshots
into the production domain as a migration test. After successful verification,
move `/usr/local/libexec/hope_mocap_config_transport.py` out of the deployment path
into the rollback backup; no active unit/drop-in should still reference it. The
obsolete implementation is removed from the repository. To roll back while still
stopped, restore the saved script and matching unit/drop-ins, reload systemd and
restart only this relay.

`tests/test_mocap_config_transport_ros.py` exercises the production executable's
payload/order/QoS behavior, silence/late-subscriber/restart non-replay, loop
rejection and kernel address parity with the old `ip -j -4` projection. A
**non-installed** test driver links the same node/timer implementation and injects
address changes/probe errors without changing real networking. Run with
`ROS_DOMAIN_ID=229 ROS_LOCALHOST_ONLY=1 HOPE_NATIVE_BIN_DIR=<native build>`;
the unit-file test also runs without ROS. Mac `test-ros` builds the relay and its
test driver before running this suite. Neither the driver nor test fixtures are
installed on robots.

Final build identities and stopped-state results are recorded in the monitor
verification section above. The Python transport script/unit/drop-ins remain
recoverable under `/var/backups/hope-mocap-config-native.Wlds6a` on HDU; the script
is no longer deployed. Short relay-child CPU samples were 3.7% for Python and
3.6% for C++ (one core): no meaningful reduction was established. Measure the
actual child, not its waiting network wrapper. Laptop end-to-end recovery,
physical network interruption and active play still need separate qualification.

The read-only observer still evaluates every source and freshness gate at 5 Hz.
Its 34 non-liveness Bool/String display statuses publish on every changed value;
unchanged values keep a 0.4 s heartbeat, quantized to the existing 0.2 s snapshot
ticks (normally 0.4–0.6 s). Liveness, Runner mode/session matching, fault status,
numeric traces and ball markers retain their original publication cadence. This
reduces duplicate display traffic without changing the IMU or command streams.

> **Confirmed Stage 2 installation (2026-08-06).** On the same A3, both units
> were installed, enabled, and observed active with zero restarts. The bridge
> listened on `0.0.0.0:8765`, and a TCP connection from the external laptop
> succeeded. The NTP gate published `true`. The monitor unit contained no
> Motive IP or mocap-host parameter; laptop-side NatNet handling was confirmed
> as the intended architecture.

That confirmation describes the installed monitoring baseline before the
E-stop-proxy and timestamp-latency revision in this branch. Redeploy the staged
`bridge_params.yaml`, both fleet service units, and the monitor files
to activate the revised layout. The vendor E-stop request encoding was checked
against the installed A3 protobuf schemas; **no live E-stop was fired as part
of development or verification.**

The verification also found that the HDU `agibot_pm.service` was active while
the MDU vendor manager had been intentionally stopped for a custom application.
The vendor joint-state, localization, TF, and emergency-service endpoints were
therefore absent from a direct live ROS graph query. Consequently
`odom -> pelvis_link` was unavailable, and the E-stop proxy could not safely be
exposed. This is why the revised UI has
separate **HDU agibot_pm**, **LIVE TF READY**, and **E-STOP BACKEND READY**
indicators. A stale `ros2cli` daemon can retain old service names; the monitor
uses live client matching rather than trusting that cache.

Runtime removal disables the added processes and removes their installed
configuration. It does not claim to reverse apt packages that may have existed
before this procedure:

```bash
sudo systemctl disable --now hope-foxglove-bridge hope-monitor
sudo rm -f /etc/systemd/system/hope-foxglove-bridge.service \
  /etc/systemd/system/hope-monitor.service \
  /usr/local/libexec/hope-monitor
sudo rmdir --ignore-fail-on-non-empty /usr/local/lib/hope-foxglove
sudo rm -rf /etc/hope-foxglove
sudo systemctl daemon-reload
```

After verifying that the following dedicated paths contain only files created
by this procedure, the operator may also remove staged/build artifacts:

```bash
rm -rf "$HOME/hope_foxglove_ws" "$HOME/foxglove_a3"
```

### 3. Per-unit verification checklist

```bash
ssh "agi@$A3_HOST" \
  'systemctl is-active hope-monitor hope-foxglove-bridge'
```

Then from the laptop, after connecting (next section): `/tf` frames appear in
the 3D panel when the vendor TF publisher is active;
`/hope/ntp/offset_ms` plots live values; the time indicator is green;
`/hope/clock/message_latency_ms` plots the A3 ROS clock minus the pelvis-IMU
header timestamp; `/hope/system/cpu_load_percent` plots aggregate A3 CPU
utilization from 0% to 100%; `/hope/vendor/agibot_pm_active` reports the local
HDU unit; and `/hope/vendor/tf_ready` becomes green only when the configured
live TF path exists. `/hope/safety/estop_ready` is true while at least one of
the vendor or native Runner emergency paths is callable, so the assert button
is not lost when managed operation intentionally stops `agibot_pm`.
`/hope/safety/estop_full_ready` is true only when both paths are live; a
one-path state is visibly `PARTIAL ONLY`. Native Runner acknowledgments and role/serve state are published by
the Runner observer on `/hope/runner/**` when the attended layer is installed.
The same observer converts `/poses[0]` into a 4 cm orange
`visualization_msgs/Marker` on `/hope/ball/marker`; its 0.2 s lifetime removes
the ball promptly when tracking is lost.
Camera topics, arbitrary publish controls, parameters, and all other services
remain unavailable.

On the A3, verify discovery without invoking the safety action:

```bash
ros2 topic echo /hope/vendor/agibot_pm_active --once
ros2 topic echo /hope/vendor/tf_ready --once
ros2 topic echo /hope/clock/message_latency_ms --once
ros2 topic echo /hope/system/cpu_load_percent --once
ros2 topic echo /hope/safety/estop_ready --once
ros2 topic echo /hope/safety/estop_full_ready --once
ros2 topic info /hope/pelvis/marker
ros2 topic info /hope/ball/marker
# Run the next line only after estop_ready reports true:
ros2 service type /hope/safety/trigger_estop
```

Do **not** invoke this service as a deployment smoke test. E-stop is a real
safety action.

> **Confirmed Stage 3 connection (2026-08-06).** Foxglove Desktop `2.58.0` on
> the external laptop completed a `foxglove.sdk.v1` WebSocket upgrade and held
> an established connection to the A3. The bridge registered the client and
> created subscriptions for `/tf` and `/tf_static`. Both robot services stayed
> active with zero restarts. That installed baseline exposed no client
> publishing, services, parameters, camera, or lidar topics; the current branch
> intentionally changes only the service portion by exposing the E-stop-only
> proxy. The local layout file remains a GUI import action; Foxglove deep links
> can select a remotely saved `layoutId` but cannot import this repository's
> local JSON file.

On that verification run, upstream vendor joint-state, TF, and localization
publishers were inactive. Therefore the connection baseline passed, while the
3D robot pose, joint freshness, and pelvis-pose data checks correctly remained
unavailable.

## Laptop setup — the robot IP is typed into the Foxglove UI

1. Install [Foxglove Desktop](https://foxglove.dev/download).
2. **Open connection → Foxglove WebSocket**, and type the robot's address
   as text in the URL field:

   ```text
   ws://<robot-ip>:8765
   ```

   This is the only place the laptop needs the robot's IP. To switch to a
   different A3, open a new connection with that robot's IP — the layout is
   fleet-generic and needs no editing.
3. Layout menu → **Import from file** → `foxglove/layouts/a3_monitor.json`.

Foxglove Desktop uses one active data source per window. This A3 layout connects
to `ws://<robot-ip>:8765`; to view laptop-side NatNet/ROS 2 mocap data at the
same time, open a second Foxglove window and connect that window to the laptop
mocap data source.

## Pelvis and ball in 3D

The operator layouts contain no URDF and do not expose
`/hope/robot_description`, `/joint_states`, raw `/tf`, or raw `/tf_static`
through the Foxglove bridges. `/hope/pelvis/tf` contains exactly one sanitized
`world -> pelvis_link` transform. The 3D panel shows only a `world` grid, that
Pelvis link and marker, and the ball marker.
The native `hope-monitor` validates the configured `world -> pelvis_link` pose, publishes
the structured/text status and standard visualization markers, and broadcasts
only the root transform needed to place the pelvis status in the HOPE world.
The Pelvis point and text markers expire after 0.5 seconds without a fresh
authoritative pose/TF, so the panel never presents a stale world position as live.

The live ball is a separate `/hope/ball/marker` topic in the same 3D panel.
It is visible only while `/poses` contains a finite pose at index 0; absence of
the ball is therefore represented by the marker expiring, not by retaining a
stale last position.

The grid follows the repository's HOPE world convention:

- `world=(0,0,0)` is the near-side left corner of the **playing surface** from
  P1's view;
- +X points along the table toward P2, so the table spans `x=[0, 2.740]` m;
- +Y points left from P1, so the table spans `y=[-1.525, 0]` m;
- +Z points up, the playing surface is `z=0`, and the floor is `z=-0.760` m.

The visual includes a red/green/blue +X/+Y/+Z triad at the origin. It is an
illustration only: no collision or planning geometry is provided.

The Pelvis indicator appears on the world grid only when the authoritative mocap
pose or TF provides a connected, fresh `world -> pelvis_link` path. Until then,
**LIVE TF READY** stays false and the 3D viewer intentionally shows only the
grid without a Pelvis label. Do not add an identity transform unless calibration
actually establishes that identity.

**Pelvis position and rotation in text:** the native `hope-monitor` looks up the
transform `reference_frame -> pelvis_frame` (layout defaults `world ->
pelvis_link`, both settable in `hope-monitor.service`) and publishes:

- `/hope/pelvis/pose` (`geometry_msgs/PoseStamped`) — structured values for
  optional inspection.
- `/hope/pelvis/text` (`std_msgs/String`) — one human-readable line:
  position in meters, quaternion, and roll/pitch/yaw in degrees. If the TF
  lookup fails, this topic carries the error text instead, so a wrong frame
  name is visible directly in the UI.

The structured pose is published only for a fresh authoritative mocap pose or
a fresh TF fallback. Consumers must still use `/hope/vendor/tf_ready` as the
validity bit because ROS topics retain the last pose a subscriber received.
The default freshness limit is 0.5 seconds and is configurable as
`tf_stale_after_s`.

During the managed session the authoritative root pose is
`/a3/mocap/pelvis_pose`, composed on the Laptop from the approved marker
calibration. The monitor validates its `world` frame and timestamp, republishes
it as `/hope/pelvis/pose`, and broadcasts `world -> pelvis_link`. Existing live
vendor TF remains the fallback when the authoritative mocap pose is absent.

To discover a unit's actual frame names, open the 3D panel's frame list, or
run on the robot: `ros2 topic echo /tf_static --once | grep frame_id`.

## CPU-load semantics

The CPU plot is the A3's aggregate Linux CPU utilization, sampled every second
from the first `cpu` row in `/proc/stat`:

```text
cpu_load_percent = 100 * (delta_total - delta(idle + iowait)) / delta_total
```

The first sample establishes the baseline, so the numeric topic begins on the
second poll. `/hope/system/cpu_load_percent` is clamped to the physical range
0%–100%, and the Foxglove Y-axis is fixed to the same range. This is not Linux
load average and it is not a sum of per-core percentages: 100% means the A3's
CPUs were collectively busy throughout the sampling interval. Sustained values
near 100% are evidence of compute saturation, but temperature throttling,
memory pressure, and scheduling latency require separate diagnostics. The
sampling interval is configurable as `cpu_publish_period_s` in
`hope-monitor.service`.

At the same one-second cadence the monitor reads the CPU tick counters in each
readable `/proc/<pid>/stat` and publishes the largest delta on
`/hope/system/cpu_top_process`. The text reports both `core=...%` (the familiar
one-core `top` scale) and `system=...%` (share of total machine CPU time). This
is diagnostic attribution; it does not change Runner, TF, joint, or latency
rates automatically.

## Timestamp latency and process-state semantics

The latency plot is not a ping test. The native `hope-imu-telemetry` process
subscribes to `/ros2/body_drive/pelvis_imu/data` (`sensor_msgs/Imu`) using
sensor-data QoS and computes at receipt:

```text
latency_ms = (A3 ROS system clock now - message.header.stamp) / 1e6
```

It republishes the latest finite value at a bounded 20 Hz and marks the source
stale after 0.5 s. A negative or implausibly large value is useful evidence of
an epoch/clock problem; it is not clamped away. Change
the native `imu_topic` parameter only to another `sensor_msgs/Imu` topic unless
the node is extended to support a different message type. The Python monitor's
direct subscription is a compatibility path when `native_clock_topics=false`.

The green/red `agibot_pm` tile is the local **HDU** systemd state. It is not a
claim that the MDU vendor manager or TF publisher is ready. The adjacent
`LIVE TF READY` tile checks the configured `world -> pelvis_link` lookup and is
the display gate that matters for the combined robot/table scene. The 3D panel
follows `world`, not `pelvis_link`, so the table and HOPE axes remain usable
while the vendor tree is absent; this also avoids treating a missing
`pelvis_link` as the panel's required coordinate frame.

## Foxglove E-stop button

The red button calls `/hope/safety/trigger_estop` with `{}`. The monitor
immediately latches out its legacy command path, requests the configured native
Runner `emergency_passive` service when available, and independently adds a
fresh timestamp and trace ID, encodes the vendor
`aimdk.protocol.EmergencyCommandReq`, and calls the live A3
`HalEmergencyService/SetEmergencyCommand` endpoint with only
`software_emergency_stop=true`. The assert-only HOPE proxy remains callable even
when neither backend is matched, so an assertion can still establish the local
inhibit and report unavailable backends. `/hope/safety/estop_ready` means at least
one explicit stop path is callable; `/hope/safety/estop_full_ready` is true only
when both independent paths are callable. A one-path state is always shown as
`PARTIAL ONLY` and requires the physical E-stop.
The independent managed Runner emergency service is attempted whenever it is
available, but its absence never suppresses the primary vendor stop; that case
returns an explicit `PARTIAL E-STOP` result.

Full dual-path acceptance requires both independent paths: the managed runner must be confirmed
stopped, AimRT's `RosRpcWrapper.code` must be zero, and the protobuf
`EmergencyCommandRsp.header.code` inside the returned `data` must decode to
zero. `Trigger.success` can also be true for one accepted path, with an explicit
`PARTIAL E-STOP` result; it must not be interpreted as full-path or physical-stop
proof. A nonzero application code, unexpected serialization,
missing header, or malformed payload fails that backend; the vendor
message is included when present. The service callback does not hold the
readiness lock while waiting up to 2 seconds for the vendor RPC, so NTP, CPU,
TF, and other monitor timers continue publishing during the call. A concurrent
second E-stop request is rejected while the first remains in progress. An E-stop
arriving during a reset is inhibited immediately, waits for that reset, then
asserts; reset during an assertion is refused. The
Runner emergency action bypasses the ordinary operator-action lock, so a long
calibration cannot prevent the independent command-source removal request.
The in-memory inhibit is set before dispatch. The monitor attempts both available
stop send calls, then durably stores the assert-only latch at
`/var/lib/hope-monitor/estop-latched`, without waiting for RPC responses first.
A backend may already have completed by the time storage finishes. A storage
failure retains the in-memory inhibit, forces a failed Trigger response and
requires physical E-stop/recovery; persistence across a monitor crash cannot then
be assumed. When persistence succeeds, `/hope/safety/estop_latched` remains true
across monitor restarts, and the Console follows this authoritative state rather
than a browser-local flag. Local dispatch order does not guarantee receive order
across the separate DDS topic/service writers. Even when both requests succeed, this
repository has no independent actuator-state
feedback, so the response explicitly requires checking the physical E-stop.

- The button is a real safety action, not a test widget.
- The fleet bridge allowlists only the assert-only E-stop service; the attended
  bridge has a separate fixed Runner allowlist. Arbitrary services, parameters,
  and client-published topics remain blocked.
- The attended bridge exposes **software-inhibit reset**, not vendor or hardware
  E-stop release. It never sends a false vendor emergency bit or starts motion.
  Inspect the supported/stopped robot and follow Agibot's approved physical/vendor
  recovery procedure before explicitly resetting the HOPE software inhibit.
  Do not delete the latch file to bypass a fault or a pending assertion.
- If the vendor wrapper package or emergency service is absent, the proxy is
  removed and the Foxglove control is unavailable. If the vendor endpoint drops
  during a call or its response cannot be validated, the callback fails closed
  and reports that success is unconfirmed; inspect the physical robot and its
  emergency state before any further action.

After the physical inspection and vendor emergency reset are complete, the
local operator must verify native Runner state and follow the approved local
recovery procedure. This recovery is intentionally unavailable through
Foxglove.

## What the layout shows

- **3D (left):** Pelvis link/world-pose marker and live ball on a HOPE `world`
  grid; no URDF, non-Pelvis robot links, or joint rendering.
- **NTP offset (upper right):** A3 system-clock offset and root dispersion in
  milliseconds.
- **ROS timestamp latency (upper right):** A3 ROS clock minus the pelvis-IMU
  message header timestamp, in milliseconds.
- **CPU utilization (upper right):** aggregate A3 CPU busy percentage over
  time, displayed on a fixed 0%–100% scale, plus the process with the largest
  per-sample CPU delta.
- **Status tiles:** HDU `agibot_pm`, connected TF readiness, runner readiness,
  calibration readiness, E-stop backend, NTP gate, and timestamp freshness use
  green/red/orange backgrounds.
- **A3 EMERGENCY STOP:** backed by the assert-only proxy described above and
  enabled when the live primary vendor backend exists and no prior latch is
  active; the detail text reports whether the Runner path is also ready.
- The integrated A3 Console on port 8766 provides the fixed native Runner actions
  documented in `docs/operations/foxglove_runner_integration.md`; the fleet
  layout itself has no legacy TTY Runner controls.

## Notes and limits

- **Security:** the fleet WebSocket has no authentication and carries the real
  E-stop service. The attended 8766 endpoint carries the fixed native Runner
  actions. Serve either only on the trusted lab subnet; do not
  port-forward it, and do not leave an untrusted Foxglove client connected.
- **Mocap network:** Motive access, NatNet reception, and mocap diagnostics are
  computer-side responsibilities. The A3 monitor has no mocap host parameter
  or network probe. It requests one computer-side calibration after settled
  PD_STAND and consumes the resulting `/a3/base_pose_flat`, not marker samples
  or JSON. Do not route the Motive network through A3.
- **Battery:** the BMS topic is protobuf-wrapped
  (`ros2_plugin_proto/msg/RosMsgWrapper`). The ELink decoder exposes
  bounded battery diagnostics in the lifecycle ELink status; the dedicated
  battery display is not implemented in this layout. Battery readings/status
  do not independently stop or inhibit HOPE. Source freshness is still checked
  for ELink monitoring, and A3's own battery/emergency protections are unchanged.
- **Clock context:** `/hope/ntp/utc_qualified` uses the same Leap Normal plus
  selected-source rule as `timesync.sh --preflight`.
  `/hope/ntp/gate_pass` additionally applies the documented offset and skew
  limits (see
  [agibot/ntp_sync/README.md](../agibot/ntp_sync/README.md)). ELink diagnostics use
  `/hope/ntp/qualification`, a boot/monitor-bound snapshot of the original
  chrony probe start and completed-probe sequence, not the Bool's receipt age.
  Delayed or replayed qualification cannot refresh diagnostic evidence; install
  the matching monitor and `hope_elink_clock.py` together. Clock qualification
  does not add a runtime motion veto. An attended, stopped Time Calibration is
  the explicit clock-correction workflow, not an automatic step during motion.
  The monitor never
  modifies clock state. Robot-state mutations occur only through the explicitly
  allowlisted Trigger services on their respective endpoints.
