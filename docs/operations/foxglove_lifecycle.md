# Foxglove Runner three-machine lifecycle

> Runtime update: use [Runner, Xbox and field operation](runtime_xbox.md) for
> current Kernel Mode, Xbox, Serve-to-Ready, calibration-layout and package
> instructions. The public checkpoint remains model_21800. The foundational
> host/SSH setup below still applies; older UI labels are superseded by that guide.

> Deployment commands reviewed 2026-10-11: the current package build,
> native Gate 3 and immutable-slot staging entry is
> [deploy_and_gate3.md](deploy_and_gate3.md). The package is selected separately
> from the machine's lifecycle configuration; build/staging alone does not
> switch the MDU slot or start Runner.

This layer replaces the repeated terminal work in hardware-runbook STEP
0/1/2A/2B/4/5 with a fixed HDU-resident supervisor. It does not expose a
remote shell. Foxglove can only:

1. confirm three RFC1918 transport addresses, an optional Motive IPv4 address, the robot's P1/P2 table
   side, and the Mac/Linux operator profile while the stack is stopped;
2. start the fixed Laptop -> HDU -> MDU sequence;
3. kill every lifecycle-managed session in reverse order, restore `agibot_pm`,
   and collect the current session logs.

The native MDU Runner remains the authority for PASSIVE, PD_STAND, MOTION,
role, serve state, action admission and action acknowledgements. The lifecycle
supervisor owns process state only.

## macOS operator

Apple Silicon Macs can run Foxglove Desktop while the fixed Laptop lifecycle
helpers run in a Linux Docker container. This imports only the Mac operator
support; it does not change Runner, Planner, ELink priorities, motion CSVs or
the robot's hardware profile. The normal Linux Distrobox path stays unchanged.

### Separate Mac, Linux and robot identities

The persistent `operator_profile` selects `mac` or `linux`, independently of
Wi-Fi address. All Laptop startup, collection, recording and asset calls use
this selection. Mac calls go to the stable SSH alias `hope-mac-operator`; Linux
calls go to its configured Laptop IP. The IP remains DDS/session metadata.

The Mac host login may be **any normal user**, not an administrator and not
necessarily the container account. Configuration lives in that user's
`~/.config/hope-macos/`. No named personal account, `/Users/...` path, native SSH
server or Mac `sudo` is required by the operator commands. Docker Desktop must
already be installed and usable by that user; organization-managed installation
may separately require an administrator. Foxglove may be installed globally or
per-user; the launcher finds it through macOS Launch Services.

The HDU explicitly specifies the remote SSH username and helper, so SSH `User`
settings alone cannot override them. Match the two **Mac-profile** settings:

| Mac `operator.json` field | Existing HDU service setting | Default if unset |
|---|---|---|
| `laptop_user` | `HOPE_MAC_USER` | `operator` |
| `laptop_helper` | `HOPE_MAC_LIFECYCLE_HELPER` | `/home/operator/.local/libexec/hope-lifecycle` |

The Linux profile independently retains `HOPE_LAPTOP_USER` (default `operator`)
and `HOPE_LAPTOP_LIFECYCLE_HELPER` (default `/usr/local/libexec/hope-lifecycle`).
Never change the Linux settings just to accommodate a Mac. The container creates
the configured non-root Linux account and translates the advertised helper path
to its fixed local executable. Other paths and robot-side actions are rejected.
Container initialization runs as root **inside Docker**, not as Mac root.

`bind` and `select-mac` read the running HDU supervisor's `MainPID` environment
and compare only these two fields before changing anything. They fail if the
identity differs, the service is stopped, or its environment cannot be read.
Use the HDU supervisor's service account for `ssh_user` (normally `agi`); do not
grant broad sudo access to bypass this check. No full environment or credentials
are returned by the check. `select-linux` remains available independently of
the Mac container's health.

### Prepare the Mac backend

Docker Desktop must be installed and running. From this checkout:

```bash
docker build --platform linux/arm64 -t hope-dev:jazzy-arm64 \
  -f scripts/macos/Dockerfile.ros scripts/macos
docker build --platform linux/arm64 -t hope-operator:jazzy-arm64 \
  -f scripts/macos/Dockerfile.operator scripts/macos
# Build the recording overlay into the volume used by the operator container.
python3 scripts/macos/hope.py build-ros --arch arm64 --jobs 2
```

Create `~/.config/hope-macos/operator.json` using
[`operator.example.json`](../../scripts/macos/operator.example.json). Replace
the sample addresses, absolute Mac checkout path and identity with your actual
setup. A `~` checkout path expands to the current Mac user's home; paths with
spaces are supported. `ssh_user` is the HDU account; `laptop_user` is the
container account selected by the supervisor's Mac profile. Neither needs to
match the Mac login. `layout_id` is only
needed by the optional `open_foxglove.command` launcher.

Keep these files outside the repository in the same private configuration directory:

- `client_key`: the existing authorized SSH private key for Mac/container access
  to the HDU and, through the HDU, the MDU. Keep permissions `0600`.
- `known_hosts`: independently verified HDU/MDU host keys; strict checking stays enabled.
- `hdu.pub`: the public key used by the HDU supervisor for its outbound Laptop SSH calls.

The configuration directory should be `0700`. The Mac does not need a native
account named `laptop_user` or macOS Remote Login. SSH is published only on
Mac localhost and forwarded to HDU localhost. Incoming container SSH accepts
only the fixed Laptop actions, with no shell, PTY or port forwarding.

While the lifecycle is **STOPPED**, run:

```bash
python3 scripts/macos/operator.py up
python3 scripts/macos/operator.py bind
python3 scripts/macos/operator.py select-mac
python3 scripts/macos/operator.py status
```

`up` starts only the local helper container and SSH tunnel. `bind` backs up and
updates the HDU's Mac SSH route/host key and adds the Mac DDS peer without
restarting services. It preserves existing Linux entries. `select-mac` only
changes the selected Laptop address and `operator_profile` together through the
configuration service; it does not press START or send motion commands. Both computers may view the
same Foxglove endpoint at `ws://<HDU-IP>:8766`, but select only one session Laptop.
Container initialization is asynchronous; if `bind` runs before its host key is
ready, wait for initialization and retry. The existing `bind` DDS update needs
administrator authorization on the HDU; if its noninteractive sudo is denied,
have the administrator update the peer configuration instead of granting
unrestricted Python sudo. No sudo policy is installed by this Mac patch.

For logs/recording, the `hope-macos-arm64` Docker volume must already contain a
compatible built ROS workspace at `/opt/hope/ros/install`, including the HOPE
messages and recording helpers. Building the images alone does not provide
that overlay. The checkout is mounted at `/work`, logs remain in `real_logs/`,
and current-machine calibration uses `artifacts/macos_setup/operator-calibration/`
instead of reusing the checkout's calibration. The home and SSH host keys are
retained in the `hope-macos-operator-home` and `hope-macos-operator-state` volumes.

This support does **not** establish Docker-to-LAN NatNet/DDS routing, clock
synchronization or Xbox USB access. Those require separate venue validation
before replacing the Linux mocap acquisition machine. An SSH preflight or a
successful configuration change is not evidence of live mocap/recording data.

### Upgrade the supervisor, Console and container together

Install the matching `hope_lifecycle_core.py` and `hope_lifecycle_supervisor.py`
on the HDU and restart its supervisor only while STOPPED. Install the bundled
Console 1.8.14, which includes the **Operator computer** selector. Confirm the
profile and current Laptop IP together, either there or with `select-mac` /
`select-linux`. Schema 1/2 documents migrate to Linux; schema 3 persists the
profile. Older five-field clients preserve the current profile, but cannot
display/select it. The Mac-specific service environment variables are trusted
HDU configuration, not editable arbitrary commands from Foxglove.

Add the two Mac-profile identity fields to older `operator.json` files. `bind`
removes the old IP-based Mac SSH match, preserving only the stable alias so a
Linux laptop reusing that IP is not misrouted. An existing container
with the old hardcoded account, wrong checkout, port or helper contract is
rejected without stopping/replacing it. While STOPPED, first use
`python3 scripts/macos/operator.py select-linux`, then
`python3 scripts/macos/operator.py down`. Stop any separately installed legacy
auto-start job before maintenance. Remove **only the stopped container named
in your configuration**, not its volumes, rebuild the image with the commands
above and rerun `up`, `bind`, `select-mac`.

No container is automatically removed by these scripts. Reverse-tunnel socket
names include a digest of HDU/user/port, so changing venues cannot silently reuse
an old robot's connection. Legacy `tunnel.sock` connections are not adopted or
closed automatically. `down` closes only the current configuration's tunnel;
an already missing tunnel does not prevent stopping the local container.

For routine return to Linux, use `select-linux` while STOPPED, then `down`.
`ensure` is the optional idempotent check for an existing user-managed launcher;
it honors `down`'s disabled marker. This checkout does not install LaunchAgents.

For local ROS/build tooling and its limits, see
[macOS development](macos_deployment.md). This Mac-only port derives from PR #53
(`d54f8be9`, `3e353d0e`) but excludes its scheduler, PD Stand, Planner-age and
serve-asset changes.

## Automatic data collection for MuJoCo tuning

Normal START arms session-owned capture on all three machines; recording defaults
to **OFF**. The existing Foxglove **Record ON / OFF** switch controls all of it.
The MDU
records the four body joint state/command groups and both IMUs through iceoryx;
Runner CSVs retain observations, raw ONNX actions, decoded nominal targets and
final control targets. HDU records ROS traffic and Planner audit rows. Laptop
records mocap and markers into the current Runner session even when the mocap
bridge was started in an earlier session. Configuration snapshots, hashes and
realtime/monotonic clock samples accompany each recording.

After START, press **Record ON** and wait for **ON**, then use READY and policy
controls. ON is acknowledged only after all three capture workers report ready.
Kernel mode retains its existing MDU-only lifecycle: the same switch controls
its body/Runner recordings without requiring absent Laptop/Planner processes.
Planner and mocap data are unavailable in that mode; use the normal OptiTrack
policy workflow for a complete MuJoCo comparison dataset.
OFF closes raw MCAP and CSV files without stopping or restarting the policy.
Every subsequent ON creates a fresh `record_<UTC>_<id>` directory with the same
identifier across machines. STARTING/STOPPING are transitions; ERROR means the
capture cannot be assumed complete. The switch changes capture only, never
Runner mode, gains, targets or HAL state. Routine operational logs remain active.
After the trial, press **Record OFF**, then **KILL + COLLECT** and wait for
collection to finish before powering off. KILL also requests recording OFF.
The operator does not need to start another recorder. Robot data persists under
`/agibot/data/bag/hope_pingpong_telemetry/sim2real_sessions/<session>/<role>`;
the collected copy is under `real_logs/<session>/` on the Laptop. Existing
attempts are retained. Raw recording stops at 1 GiB free space on HDU and 2 GiB
on MDU/Laptop, and reports an
error; it does not delete previous recordings or pause the controller.

Audit/export with `foxglove/helpers/hope_sim2real_export.py SESSION --output NEW_DIR`
in a Python environment containing NumPy, mcap and rosbags. Its NPZ files retain
individual source messages and both source/recorder timestamps. Check the quality
report and CSV health files before fitting latency or dynamics. Source headers
can use different clocks: do not subtract command and state timestamps directly.
Joint effort is vendor telemetry; motor current, temperature, calibrated torque
and contact force are not available in this schema. Data presence does not
establish physical model equivalence.

## Operator surface

The custom console calls these exact services through port `8766`:

| Service | Type | Meaning |
|---|---|---|
| `/hope/lifecycle/apply_config` | `rcl_interfaces/srv/SetParameters` | Fixed network, table-side and operator-profile fields; not a generic parameter service |
| `/hope/lifecycle/start` | `std_srvs/srv/Trigger` | Start STEP 0/1/2A/2B/4/5 asynchronously |
| `/hope/lifecycle/kill_all_and_collect` | `std_srvs/srv/Trigger` | Kill all fixed lifecycle-managed sessions, restore `agibot_pm`, collect logs |

`start_parameter_services=False`, the bridge parameter allowlist is empty, and
browser topic publishing remains disabled. `apply_config` rejects missing,
duplicate and non-string fields; address fields also reject non-IPv4,
loopback, multicast and non-RFC1918 values, except that `motive_ip` also accepts
IPv4 link-local (`169.254.0.0/16`) for a direct Laptop-to-Motive cable. The Laptop
must have an address and route on that capture interface; its `laptop_wifi_ip`
remains the separate RFC1918 address used to reach the HDU. `table_side` accepts
only the exact enums `P1` and `P2`; `operator_profile` accepts `mac` or `linux`.
The only accepted names are:

- `laptop_wifi_ip`
- `hdu_wifi_ip`
- `mdu_internal_ip`
- `motive_ip`
- `table_side`
- `operator_profile`

Schema 3 persists the operator profile. Schema 1/2 files migrate to Linux;
older five-field clients preserve the currently selected profile. Upgrade the
core, supervisor and Console together before using Mac/Linux selection.

The packaged Rockchip runtime still fixes the HDU-side internal endpoint at
`10.42.10.10`; changing that value is a package/runtime rebuild, not a normal
venue input, so the UI deliberately does not pretend it can rewrite it.

The inputs remain editable in the UI at all times. Confirmation is accepted
only in `STOPPED` or `CONFIG_ERROR`; changing text during a run does not mutate
the active session. Starting requires a confirmed positive configuration
revision.

The fleet Fast DDS profile is address-agnostic UDPv4 rather than pinning old
HDU addresses in a checked-in XML. Each managed bridge/relay/Planner command
generates an XML `initialPeersList` from the confirmed peers (the environment
variable alone is not sufficient with the deployed HDU/MDU Fast DDS builds).
Changing `hdu_wifi_ip` still means Foxglove Desktop must
reconnect to `ws://<new-hdu-ip>:8766`; the panel cannot move its own existing
WebSocket connection.

## One-time prerequisites

The copy/paste deployment and first attended test procedure is
[`foxglove_first_hardware_test.md`](foxglove_first_hardware_test.md).
It also installs the Laptop-local 3D asset user service and stages the marker
publisher files that the managed OptiTrack Distrobox session starts.
Fresh Laptops must first install Distrobox and create the ROS-equipped `hope`
environment as documented in [`DISTROBOX_SETUP.md`](../DISTROBOX_SETUP.md).

Set the site addresses and accounts on the Laptop first. Do not commit the
resolved values:

```bash
cd "$(git rev-parse --show-toplevel)"
export HOPE_ROOT="$PWD"
export LAPTOP_USER="${USER}"
export ROBOT_USER="${ROBOT_USER:-agi}"
export LAPTOP_IP=<laptop-wifi-ip>
export HDU_IP=<hdu-wifi-ip>
export MDU_IP=<mdu-internal-ip>
export MOTIVE_INTERFACE_IP=<laptop-motive-nic-ip>
```

Store the same local wired-NIC address as `HOPE_MOTIVE_INTERFACE_IP` in
`~/.config/hope-foxglove/lifecycle.env`. The lifecycle helper validates that
the route to the Motive address uses this source IP and passes it to the
NatNet multicast launch; an empty or mismatched value fails before startup.

The HDU robot account must have non-interactive SSH authentication to the
Laptop operator account and the MDU robot account. The Laptop must retain its
existing non-interactive access to the HDU and MDU for STEP 0 evidence and STEP
6 collection. Verify all four directions before enabling the supervisor:

```bash
# From HDU; substitute the values printed on the Laptop.
ssh -o BatchMode=yes "LAPTOP_USER"@"LAPTOP_WIFI_IP" true
ssh -o BatchMode=yes "ROBOT_USER"@"MDU_INTERNAL_IP" true

# From Laptop
ssh -o BatchMode=yes "${ROBOT_USER}@${HDU_IP}" true
ssh -o BatchMode=yes -J "${ROBOT_USER}@${HDU_IP}" \
  "${ROBOT_USER}@${MDU_IP}" true
```

Do not put passwords in the repository, UI, ROS messages, unit files, helper
arguments or process environment. Install SSH public keys once instead.

`tmux` must be installed on all three machines. The MDU robot account also
needs narrow passwordless sudo only for the commands already present in STEP 4
and STEP 6. Install with `visudo -f /etc/sudoers.d/hope-lifecycle`:

```sudoers
<robot-user> ALL=(root) NOPASSWD: /usr/bin/systemctl stop agibot_pm.service
<robot-user> ALL=(root) NOPASSWD: /usr/bin/systemctl start agibot_pm.service
```

Confirm the resolved command path with `command -v systemctl` on the MDU
before installing that file. The helper uses `sudo -n` and fails closed rather
than prompting.

## Install the fixed helper

On the Laptop:

```bash
cd "$HOPE_ROOT"
sudo install -D -o root -g root -m 0755 \
  foxglove/helpers/hope-lifecycle \
  /usr/local/libexec/hope-lifecycle
sudo install -D -o root -g root -m 0755 \
  foxglove/helpers/hope_sim2real_capture.py \
  /usr/local/libexec/hope_sim2real_capture.py
sudo install -D -o root -g root -m 0755 \
  foxglove/helpers/hope_sim2real_control.py \
  /usr/local/libexec/hope_sim2real_control.py
```

Stage the same file to the HDU and MDU, then install it root-owned at the same
absolute path:

```bash
cd "$HOPE_ROOT"
scp foxglove/helpers/hope-lifecycle foxglove/helpers/hope_sim2real_capture.py foxglove/helpers/hope_sim2real_control.py \
  "${ROBOT_USER}@${HDU_IP}:/tmp/"
ssh "${ROBOT_USER}@${HDU_IP}" \
  'sudo install -o root -g root -m 0755 /tmp/hope-lifecycle /tmp/hope_sim2real_capture.py /tmp/hope_sim2real_control.py /usr/local/libexec/'

scp -o "ProxyJump=${ROBOT_USER}@${HDU_IP}" \
  foxglove/helpers/hope-lifecycle foxglove/helpers/hope_sim2real_capture.py foxglove/helpers/hope_sim2real_control.py \
  "${ROBOT_USER}@${MDU_IP}:/tmp/"
ssh -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}" \
  'sudo install -o root -g root -m 0755 /tmp/hope-lifecycle /tmp/hope_sim2real_capture.py /tmp/hope_sim2real_control.py /usr/local/libexec/'
```

The MDU active package also needs the logging-enabled Runner,
`a3_body_drive_debug_record`, its ROS type-support and record/playback libraries,
and `config/a3_body_drive_debug_record.iceoryx.yaml`. Updating helpers alone does
not add native observation/action capture. Keep the matching package hashes.

If Laptop system installation is unavailable, install both helpers together in
`~/.local/libexec/` and set the HDU supervisor's trusted systemd environment
`HOPE_LAPTOP_LIFECYCLE_HELPER` to that user's absolute `hope-lifecycle` path.
This is deployment configuration, not an operator-supplied remote command.

The supervisor also installs `foxglove/a3/hope_distributed_recording.py` alongside
the other Python modules. The HDU service needs a pre-created writable
`/home/<robot-user>/.local/state/hope-sim2real` directory and a matching
`ReadWritePaths` entry because its normal `ProtectHome=read-only` applies to local
record-control calls. Runner and Planner must both include the gated CSV writer;
`HOPE_RECORD_CONTROL_FILE` is supplied by the capture wrapper. This trusted
environment flag only affects worker-thread logging. The shared header is
`a3_deploy/recording_file.hpp`; isolated Planner builds can set
`HOPE_RECORDING_INCLUDE_DIR` to the staged include directory.

## Install the HDU supervisor

Stage `foxglove/a3/` as described in `foxglove/README.md` and refresh
`~/foxglove_a3/fastdds_bridge_profile.xml` from this branch, then run on the
HDU:

```bash
sudo install -D -o root -g root -m 0755 \
  ~/foxglove_a3/hope_lifecycle_supervisor.py \
  /usr/local/bin/hope_lifecycle_supervisor.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_a3/hope_lifecycle_core.py \
  /usr/local/lib/hope-foxglove/hope_lifecycle_core.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_a3/hope-lifecycle-supervisor.service \
  /etc/systemd/system/hope-lifecycle-supervisor.service
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_a3/bridge_params_control.yaml \
  /etc/hope-foxglove/control_bridge_params.yaml
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_a3/fastdds_bridge_profile.xml \
  /etc/hope-foxglove/fastdds_bridge_profile.xml

sudo systemctl daemon-reload
sudo systemctl enable --now \
  hope-observer.service \
  hope-command-proxy.service \
  hope-foxglove-control-bridge.service \
  hope-lifecycle-supervisor.service
```

These four units form the always-available, fixed-action control plane needed
to start a stopped robot from Foxglove. They do not start HAL, Planner or
Runner, and they do not stop `agibot_pm`. In lifecycle mode, leave them running
after `KILL ALL & COLLECT` so the next session can be started without a terminal.

Read-only verification:

```bash
systemctl status hope-lifecycle-supervisor.service --no-pager
ros2 topic echo /hope/lifecycle/summary --once
ros2 service type /hope/lifecycle/apply_config
ros2 service type /hope/lifecycle/start
ros2 service type /hope/lifecycle/kill_all_and_collect
```

Do not use Start as an installation smoke test on hardware.

## Start behavior

HDU preflight checks the installed Wi-Fi power/connected-scan policy before
launch. The optional Wi-Fi-only profile also stops cellular and rejects respawn.
See [robot runtime policy](robot_runtime_policy.md) for setup, manual rescan,
rollback, and the periodic I/O audit. Install these dependencies with the helper;
do not use START as a setup smoke test.

After the operator confirms the four boxes and acknowledges the physical
support/E-stop dialog, the supervisor creates one session and runs:

```text
SESSION -> OPTITRACK -> BASE_RELAY -> PLANNER -> HAL -> RUNNER(PASSIVE)
```

Each long-lived process runs in a fixed tmux session owned by the normal host
account. No generic command string enters tmux. A complete three-host preflight
runs before creating the session. An unmanaged bridge, base relay, Planner,
HAL or Runner makes preflight return to `STOPPED` instead of killing it or
offering cleanup for a process it does not own. A leftover fixed tmux session
is distinguishable as managed state: the supervisor enters `FAILED` and offers
`KILL ALL & COLLECT` recovery. STEP 4 retains the documented stop of `agibot_pm`.
An `aimrt_main_hal` inside the `/system.slice/agibot_pm.service` cgroup is the
expected vendor HAL and is accepted by preflight. STEP 4 stops the unit first,
verifies that this HAL has exited, and only then launches the fixed `hope-hal`
session. A HAL outside that cgroup, or one remaining after the unit stops,
fails closed; stale unmanaged HAL cleanup is an exceptional recovery action
rather than part of a normal one-click start.

`start` returns quickly. Progress and failures are published under
`/hope/lifecycle/**`; the console must reach `RUNNING · RUNNER` and
`START_COMPLETE_RUNNER_PASSIVE` before the operator uses Stand/Calibration/
Refresh x_hit/Ready or the Server flow.

The helper intentionally preserves the current fixed3 Runner argv. The checked
in model_21800 policy metadata is `training_recipe=rally_v14`, while the native
`--serve` controller rejects anything other than its qualified `rally_v17`
artifact. Local `SERVER/RECEIVER` role state is available, but this lifecycle
does not add `--serve` or bypass that artifact check. `Ready to Serve` and
`Serve` remain disabled until a separately qualified compatible Runner package
reports `serve_capability=AVAILABLE`.

## Kill behavior

`KILL ALL & COLLECT` is intentionally separate from E-stop. It is accepted for
a managed `RUNNING` or `FAILED` session without requiring a fresh Runner mode
or first transitioning to PASSIVE/PD_STAND. Before confirming it:

1. physically support the robot because active support may disappear immediately;
2. confirm the physical E-stop remains reachable;
3. use Stand first only when the situation allows it; Stand is recommended but
   is not a kill admission gate.

"All" means every fixed session created by this lifecycle: Runner, HAL,
Planner, base relay, world/calibration processes and the Laptop OptiTrack/
marker/packetizer session. It does not mean arbitrary host processes: the
helper uses exact managed tmux session names and never exposes `pkill`,
`killall`, a shell command box or a browser-supplied process selector.

The fixed reverse sequence is:

```text
Runner -> HAL -> restore agibot_pm -> Planner -> base relay
       -> Laptop bridge -> rsync HDU/MDU evidence to Laptop
```

If any cleanup step fails, the supervisor continues the remaining cleanup and
finishes in `FAILED` with the failed step in `last_result`. The vendor E-stop
remains a separate assert-only path and is never used by lifecycle startup or
kill/collection.

## Verification boundary

Repository unit tests, shell parsing and extension packaging prove only the
wire/configuration restrictions and fixed command construction. A supervised
three-machine dry rehearsal and then an attended hardware trial are still
required. In particular, verify SSH keys, MDU sudoers, tmux availability,
exact deployed Runner package, ROS service discovery, process identity, log
flush, `agibot_pm` restoration and the physical PASSIVE/PD_STAND transitions.

## Field gripper and Prepare configuration

Select the gripper while STOPPED and confirm the configuration before START.
`legacy` preserves close/open positions 800/2000 and force 20 (speeds 40/60).
`new_20261015` is the existing field profile identifier, updated on 2026-10-18:
close 850, open 1500, force 1, speed 100. These are device command units, not
millimetres or measured finger velocity. The default remains `legacy`.

The Console, HDU supervisor, lifecycle helper, package launcher, Python bridge
and native C++ release publisher carry the same selection. The launcher compares
native and Python serialized release bytes before creating a bridge or starting
Runner; mixed binaries/catalogs fail closed. Build/package the Runner and bridge
together. The catalog is included at `tools/gripper_profiles.json`; it is not a
browser-supplied file. Direct package use accepts `--gripper-profile new_20261015`.

Lifecycle schema 6 combines gripper, Mac/Linux operator and VRPN/pants options.
Both historical main and field schemas migrate without guessing the capture
protocol from an IP. Old five/six/seven-field requests preserve newer options. Install the HDU core/supervisor/helper and Console together
while STOPPED. No robot service is restarted by this source change.

Prepare retains its five-second duration and READY target. The left arm clears
first; the right shoulder pitch/roll and elbow begin at 2.0 s, then shoulder yaw
and wrists at 2.6 s. CSV playback is unchanged. The operator reported no further
Prepare leg contact after this field change; the regression test checks the
command ordering, not physical collision safety.

The two packaged ping-pong runtime configurations use release lead 3 (frame
48 event -> frame 45 command at 100 Hz). The controller accepts 0–4 frames only
while idle; frame 44 was a trial, not the retained default. These parameters do
not identify the physical command-to-detachment delay. The v20 CSV asset is
proposed separately in #53; this PR does not select a new serve CSV. Retain the
previous asset for rollback when changing an installed selection.

## Site options: ball physics and body timing

The Berkeley and local PPQ field setups share code, not implicit site defaults.
The Console's **Ball physics model** selector writes an explicit Berkeley
(`ball_physics_real_20261010.yaml`) or PPQ002 profile on both the HDU and the
selected Laptop. The same 14 parameters feed Planner and flight packetizer.
An absent selection preserves the installed base parameters. This change does
not replace the Berkeley C++/simulator defaults, model14600 policy, calibration,
Motive network interface, or the chosen Mac/Linux operator identity.

Select only while STOPPED. A two-host update retains prior selections for
rollback, including uncertain writes. Restart the Laptop capture/packetizer
after selecting, then perform the normal managed START. The supervisor reads
actual ROS parameters before HAL/Runner start; a saved selection alone is not
proof of runtime activation. `NOT_RUNNING` means the optional Laptop packetizer
was absent, not verified. Kernel-only serving does not require a ball model.

Deploy the matching supervisor, `hope_field_operations.py`, `hope_ball_models.py`
and lifecycle helper together. HDU installs `hope_ball_models.py` alongside
`hope_field_assets.py` in `/usr/local/bin`; retain both reviewed YAMLs in the HDU
workspace's Planner config directory. Laptop uses the checkout's matching
`hope-laptop-ball-model`, Python modules, configs, and updated bringup launch
files. Linux uses the existing Distrobox; the Mac operator container uses
`HOPE_LAPTOP_RUNTIME=direct` and its forced-command dispatcher must also include
`ball-model-laptop`. The `hope_bringup` and NatNet launch paths apply the saved
override after the base packetizer parameters. VRPN/pants setup also applies
this override; see [mocap setup](foxglove_mocap_setup.md). Motive network
interface and calibration paths remain unchanged.

Body timing is independent: Berkeley/main keeps
`target_turn_reject_stale_pose: true`; the local field chooses `false`, keeping
age diagnostics without an age veto. See the Planner README and
`field_body_timing_audit.yaml`. Selecting PPQ002 does not silently change this
option. Preserve the selected site's value when updating installed files.
