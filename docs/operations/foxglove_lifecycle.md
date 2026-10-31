# Foxglove Runner three-machine lifecycle

> Current deployment: **model17400 + PPQ004**, receive-only, using the
> [matching bundle and Console](../../foxglove/extensions/hope-a3-console/README.md#model17400-receive-policy).
> The host/SSH setup below applies to this managed lifecycle. Model17400 does
> not enable serving or Kernel mode; retain the bundle's declared capabilities.

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

## Player assignment and mocap loss

Normal play requires a fresh unrotated player pose at START. Console 1.8.18
reads `/hope/mocap/players/UCB_P1` and `/hope/mocap/players/UCB_P2` directly:
P1 requires any tracked player at `x < 0 m`; P2 requires one at `x > 2.74 m`.
Both relays convert units to metres and publish these poses in
`hope_table_global` before the selected P2 rotation. Evidence expires after
250 ms; duplicate source timestamps do not renew it. The raw-player topics are
capped separately at 30 Hz; policy/base/ball streams are unchanged. Any player
body remains eligible by venue policy; this verifies end occupancy, not robot
identity or that the selected runtime body name is present. The HDU checks the same
observations before hardware startup. START also shows/requires the HDU verdict
from `/hope/field/status`, so a Laptop/HDU discovery mismatch is visible rather
than a deceptively enabled START. Side selection stays locked until STOPPED;
crossing a boundary during play does not change the assignment.

The native Runner monitors its base-pose mailbox with a 1 s monotonic timeout
in normal SERVE/RALLY, independent of sampling rate (150 Hz or higher). On loss,
RALLY enters the existing PD_STAND blend. SERVE first aborts through its existing
validated controller recovery. Late serve completion cannot restart receive
after a dropout. Fresh data does not resume play automatically: the operator
must request play again. PASSIVE, manual Stand, cleanup and offline Kernel/Pure
Serve retain their behavior. Ball occlusion alone is not base-stream loss.

After **20 ms** without a real pelvis update, the next model tick reuses the last
applied pelvis position and orientation as an explicitly **HELD** input. It
does not continue gyro propagation, create a sensor packet, change sequence or
timestamps, advance differentiated-velocity history, confirm HOME/posture, or
authorize a new shot. Live joint/IMU inputs remain live. The original real-sample
age continues increasing; reuse cannot postpone the 1 s rescue deadline.
This runtime held-pose rescue window is independent of exported observation-age
normalization; the policy metadata and real-sample timestamp remain unchanged. Explicit invalid packets
still trigger recovery immediately. This continuity measure is not proof of
physical stability and cannot repair loss of local feedback or motor transport.
HELD telemetry is not a separate 20 ms mode-admission gate: entering play and
normal serve completion use the real observation/rescue deadline. An expired
completion records the safety reason even if the monitor has not yet recorded
it; a 20–999 ms held sample alone creates no stop event. L3 recovery-envelope
expiry clears a pending flight before the new-flight held-pose check.

Holding attitude without gyro propagation and retaining the prior velocity
estimate changes actor inputs during jitter. The new-flight guard defers a
commit for retry, not a flight drop, but this does not establish correct strike
timing. Replay real 150/300 Hz sessions with 5/25/60/99/250/999 ms gaps and compare target
commit time, retained velocity, strike time/contact error and recovery commands
with the prior policy-input path. Then qualify on supported hardware with an
attending E-stop operator; software regressions are not policy/hardware approval.

Normal managed play also enables `--operator-watchdog`. The Runner issues a
challenge every 20 ms. The current Console echoes role 1, and the operator
computer's Motive/VRPN relay echoes role 2, bound to its managed launch session
(`debug_session_id`). Both must be live before normal play entry. Loss of either
echo requests controlled PD_STAND recovery: **1 s for the Console**, **1 s for
the operator-computer relay**. A normal
serve uses its existing abort/recovery path. Held pelvis data cannot refresh
either heartbeat. Stand/PASSIVE, Teleop and offline Kernel/Pure Serve are exempt;
standalone tools do not opt into the UI watchdog unless explicitly requested.

Managed START reconnects the Laptop receiver with the new session ID before
starting HAL/Runner. This replaces any independently started debug session so
role-2 echoes match the Runner; a failed reconnect stops START before HAL.
Offline Kernel startup does not reconnect the receiver. These external-link
windows do not change the internal HAL/joint-state safe-halt deadline.

Challenge `[1, boot, session_fingerprint, nonce]` travels on
`/hope/runner/operator_challenge_flat` → `operator_challenge_hdu_flat`.
Echo `[1, boot, session_fingerprint, nonce, role]` travels back on
`/hope/runner/operator_echo_hdu_flat` → `operator_echo_flat`. Runner validates its
own monotonic challenge-issue age, not a remote clock or new echo receipt time.
Duplicate, wrong-session/boot and late echoes cannot renew liveness; no cached
heartbeat is timer-replayed. This is a trusted-LAN liveness protocol, not an
authentication boundary against malicious publishers.

These timeout values are watchdog triggers, not the time to finish the standing
pose. Detection runs on the command callback and is subject to its scheduling;
the existing stand blend/validated serve recovery determines motion duration.
A slow/frozen/minimized Console or a switch to another layout/tab can miss its
1 s deadline just like a disconnected one. Echoes come only from `onRender`;
this is not a background app-liveness service. The one-second UI window tolerates
short rendering/network delays, not a Console absent for more than 1 s.
Qualify actual foreground/background rendering, Wi-Fi jitter and CPU load
on the deployed system before competition; do not claim hard real-time timing
or hardware stability from unit tests.

Runner publishes `/hope/runner/mocap_safety_flat` at 50 Hz, forwarded unchanged to
`/hope/runner/mocap_safety_hdu_flat`. Its schema is
`[1, boot_id, session_fingerprint, state_seq, required, fresh, stop_count, interrupted,
held, real_age_ms, operator_required, ui_fresh, computer_fresh, operator_stop_reason, operator_stop_count]`.
An unavailable age is -1. The Console also accepts the original eight-value prefix.
The Console checks its age and current Runner boot, and displays **Stopped: mocap
lost**, **Stopped: Foxglove UI lost**, **Stopped: operator computer lost**, or the
held-pose age independently of the last operator-action result. `interrupted` stays set
after data returns until explicit play entry succeeds. The original 21-value
schema-2 Runner state and action codes stay unchanged. Refused play transitions
return `MOCAP_UNAVAILABLE`, `OPERATOR_UI_UNAVAILABLE` or
`OPERATOR_COMPUTER_UNAVAILABLE`, never `APPLIED`, and never leave a pending serve prepare.

The supervisor service uses the same explicit-peer wrapper as the control bridge
when `ROS_STATIC_PEERS` is configured in `/etc/hope-foxglove/network.env`. Include
the operator computer and MDU peers, then restart the updated service while
stopped. With no peers configured it retains the previous discovery setup so
configuration/cleanup remain available; normal play still needs verified player
reception. Verify both raw poses and the HDU `player_positions` verdict before START.
After a FAILED-session cleanup succeeds, a player/preflight rejection returns
STOPPED and releases the hardware-operation lock. A failed cleanup still retains
FAILED, the old session identity and the lock for explicit retry.

Install `hope_player_positions.py` with the HDU modules, update the supervisor
service, control decoder, mocap and Runner transport relays, rebuild the MDU Runner
with its matching AimRT config and install the matching Console. Rebuild
`hope_bringup` on the operator computer to install `operator_heartbeat.py`, update
the VRPN launch file and lifecycle helper, and deploy the control-bridge config
that permits **only** `/hope/runner/operator_echo_hdu_flat` client publishing.
All command/request topics and generic parameter operations remain denied.
Normal-play preflight rejects an old Runner or AimRT config that lacks the
operator-watchdog protocol, rather than silently running without this protection.

Before deployment approval, verify both ends and failed-start cleanup, then:

1. Bench-check 150/300 Hz input and 20–999 ms gaps: HELD pose is constant, real
   timestamps/sequence age normally, no synthetic velocity samples or new shots;
   mode entry and serve completion remain allowed until the actual deadline.
   Validate the actor-input/strike-timing replay described above before hardware.
2. With the robot supported and an operator at E-stop, interrupt the mocap
   stream and the Laptop–HDU/HDU–MDU transport separately. Record watchdog trigger
   time, actual joint-command continuity and the full PD_STAND recovery.
3. Independently close/disconnect/freeze the Console and stop/disconnect the
   managed operator relay/computer. Verify the Console's 1 s and computer's
   1 s triggers despite the other channel remaining live. Exercise tab/layout
   switches and minimize/background behavior, including brief interruptions that
   should not stop play. Delayed/replayed challenges must not postpone expiry.
4. Restore each link: no automatic play resumption. Check explicit re-entry,
   manual Stand/PASSIVE and unchanged offline Kernel/Pure Serve/Teleop behavior.

## Pre-operation clock correction

Guarded BSPs whose vendor launcher invokes `system/timesync_guard.py` need the
[receipt-bound adapter](guarded_clock_calibration.md) for the Time Calibration
button. Without its matching installation, the coordinator rejects
`GUARDED_BSP_REQUIRES_PAIRED_REBOOT` before changing either board. With it, only a
current, stopped-system, qualified maintenance operation can restore vendor PTP;
ordinary startup still runs the original guard. Keep the guard and runtime PTP.

For automatic power-on readiness, also apply the
[boot-clock setup](new_robot_commissioning.md#hope-开机一次性校时-setup).
The generic clock installer leaves bootstrap disabled; HOPE enables the pinned
one-shot before vendor and HOPE consumers, then verifies an attended paired boot.
Continuous chrony remains slew-only. Boot success does not substitute for button
acceptance, MDU/Laptop/mocap clock checks or physical function qualification.

For a large HDU clock offset, use the attended **Time Calibration** action
**before START**, not a long wait for runtime slewing. At `maxslewrate 100`,
reducing 168 ms to the 10 ms gate takes at least about 26 minutes. Chrony
[recommends stepping before time-sensitive applications start](https://chrony-project.org/doc/4.5/chrony.conf.html#makestep).
The continuous runtime configuration remains slew-only; this does not enable
unconditional `makestep` on a chrony service restart.

1. Physically support the robot, keep the E-stop reachable, stop all Runner
   sessions and external motion publishers, and verify lifecycle **STOPPED**
   and idle. PASSIVE is still a running Runner, not a stopped system.
2. With a fresh NTP observation and confirmed network configuration, select
   **Time Calibration** and accept the support/stopped-system confirmation.
   An operator may explicitly resynchronize even when the NTP gate is healthy.
   A completed calibration permits another manual request; an unfinished step
   retains its recovery interlock and cannot be repeated.
   The coordinator rechecks fresh STOPPED/idle evidence after acquiring the
   shared hardware-operation lock. It never automatically steps from START,
   PD_STAND, SERVE or RALLY/MOTION.
3. The backend snapshots the verified, currently active control-plane units,
   stops downstream MDU consumers, then HDU control/vendor consumers and the
   independently supervised HDU PTP services. Immediately
   before the one-shot bootstrap, it rechecks the MDU's same-boot stopped
   receipt and both boards' service/process state. Unknown/error observations,
   remaining processes or restarting units prevent the step.
4. It runs the approved bootstrap once, verifies the 10 ms clock gate, restores
   HDU PTP before vendor services, then MDU PTP/vendor services and the
   previously active Foxglove units.
   Port 8766 temporarily disconnects. Only after **COMPLETE** and fresh clock
   qualification should the operator explicitly START. Failure after stopping
   consumers retains the maintenance interlock; there is no automatic step retry
   or motion restart. Clock qualification remains diagnostic during ordinary
   operation; the existing startup checks still apply.

The HDU control plane may use either the split services or a consolidated
`hope_operations.py` process. Both require the monitor, data bridge, control
bridge and lifecycle-supervisor units to be active. Split deployments also
require active `hope-observer.service` and `hope-command-proxy.service`.
Consolidated deployments instead require the actual Python
`/usr/local/bin/hope_operations.py` process owned by the lifecycle-supervisor
service; its process ownership is checked, not just its unit name or wrapper
command. The observer/proxy units may be absent or loaded but inactive
compatibility stubs (for example, `ExecStart=/usr/bin/true`). Those stubs are not
missing workloads and must not be started to satisfy a six-service checklist.

Only the snapshotted active units are stopped, reset and restored. Inactive
optional units stay inactive. Before stepping, the coordinator still rejects
any known control/time consumer that remains active, unknown service/process
state, or an unexpected observer/proxy/operations process. Optional absence or
loaded-inactive status is harmless only with a verified deployment layout;
it is not permission to ignore an unowned process.

Normal and fail-safe restoration wait up to 30 seconds for the actual owned
`hope_operations.py` Python child to return in a consolidated layout; an active
shell wrapper and listening port alone do not prove recovery. If a previously
active unit has disappeared or cannot be verified, restoration still attempts
the remaining captured units, but reports failure rather than `COMPLETE`.
It never substitutes or starts previously inactive compatibility stubs.

Older field installations with vendor-owned HDU PTP remain supported only when
both dedicated PTP units are explicitly absent, PM is active and both workers
are present. Their workers must actually exit after PM stops, and PM is restored
before checking that PTP returns. Mixed/unknown topology, orphan workers and
topology changes during maintenance deny stepping; no packages are silently
installed and no running service is treated as absent because a probe failed.

Install the matching HDU coordinator/core, MDU helper and exact clock-action
sudo permissions together; see the [hardware installation/runbook](foxglove_first_hardware_test.md).
The final stopped-state check is not protection against an administrator or
unmanaged external publisher starting motion concurrently: exclusive attended
maintenance remains required. Local regression tests are not hardware approval.

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

Update the matching lifecycle implementation for the HDU's existing
[deployment layout](#install-the-hdu-supervisor), and restart its owning service
only while STOPPED. The split layout uses `hope_lifecycle_core.py` and
`hope_lifecycle_supervisor.py`; do not replace a consolidated field bundle with
these standalone samples without a separate deployment migration. Install the bundled
Console 1.8.18, which includes the **Operator computer**, gripper and mocap selectors. Confirm the
profile and current Laptop IP together, either there or with `select-mac` /
`select-linux`. Combined schema 6 preserves field schema-5 gripper, pants and
VRPN settings while adding the operator profile; main's older profile-aware
schemas also migrate. Older five/six/seven-field clients preserve newer selections.
Do not replace a live schema-5 backend with PR #54's former schema-3 files.
The Mac-specific service environment variables are trusted
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

For a new robot, replaced board or fresh BSP, use the mandatory
[new-robot commissioning checklist](new_robot_commissioning.md) before treating
an application upload or PASSIVE start as deployment completion. Verify HDU
chrony/bootstrap, Laptop NTP, MDU PTP and the selected mocap computer separately.
Time Calibration maintains an installed baseline; it does not install one.

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

`tmux` must be installed on all three machines. The MDU robot account needs
passwordless sudo for the vendor service actions and the fixed scheduler helper.
Install the service rules with `visudo -f /etc/sudoers.d/hope-lifecycle`:

```sudoers
<robot-user> ALL=(root) NOPASSWD: /usr/bin/systemctl stop agibot_pm.service
<robot-user> ALL=(root) NOPASSWD: /usr/bin/systemctl start agibot_pm.service
```

Confirm the resolved command path with `command -v systemctl` on the MDU
before installing that file. The helper uses `sudo -n` and fails closed rather
than prompting.

The scheduler installer creates `/etc/sudoers.d/hope-elink-scheduler`, owned by
root with mode `0440`. For the default account, its rule is:

```sudoers
agi ALL=(root) NOPASSWD: NOSETENV: /usr/local/libexec/hope_elink_scheduler.py check-privileges, /usr/local/libexec/hope_elink_scheduler.py apply --config /tmp/hope_real/*/mdu/system/hal_elink_a3_gripper.yaml
```

The helper validates the complete argument list and session path, including the
single session-name component matched by the sudoers wildcard. The rule grants
no Python interpreter, arbitrary script, shell, `inspect`, or `restore` command.
Inspection and restoration remain administrator actions. Do not add a general
`sudo python3` permission.

The installer requires `sudo`, `visudo`, and `runuser`; validates the existing
and proposed sudoers configuration; installs the helper as root-owned `0755`;
then runs the actual privilege probe as the selected robot account and checks
its `apply` authorization. If installation or either probe fails, it restores
the previous lifecycle, scheduler, and dedicated sudoers file. A successful
install prints `MDU_ELINK_SCHEDULER_INSTALLED`. Copying Python files alone is
not a complete scheduler installation.

From the configured robot account, check permissions without starting HAL:

```bash
sudo -n /usr/local/libexec/hope_elink_scheduler.py check-privileges
sudo -n -l -- /usr/local/libexec/hope_elink_scheduler.py apply \
  --config /tmp/hope_real/permission_probe/mdu/system/hal_elink_a3_gripper.yaml
```

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

ssh -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}" \
  'mkdir -p /tmp/hope-elink-install/a3 /tmp/hope-elink-install/helpers'
scp -o "ProxyJump=${ROBOT_USER}@${HDU_IP}" \
  foxglove/helpers/hope-lifecycle foxglove/helpers/hope_sim2real_capture.py foxglove/helpers/hope_sim2real_control.py \
  foxglove/helpers/hope_elink_scheduler.py \
  "${ROBOT_USER}@${MDU_IP}:/tmp/hope-elink-install/helpers/"
scp -o "ProxyJump=${ROBOT_USER}@${HDU_IP}" \
  foxglove/a3/install_mdu_elink_scheduler.sh \
  foxglove/a3/hope_elink_health.py foxglove/a3/hope_elink_fault.py \
  "${ROBOT_USER}@${MDU_IP}:/tmp/hope-elink-install/a3/"
ssh -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}" \
  'cd /tmp/hope-elink-install && python3 helpers/hope_elink_scheduler.py check-hardware --lifecycle helpers/hope-lifecycle && sudo install -o root -g root -m 0755 helpers/hope-lifecycle helpers/hope_sim2real_capture.py helpers/hope_sim2real_control.py /usr/local/libexec/ && sudo install -D -o root -g root -m 0644 a3/hope_elink_health.py /usr/local/lib/hope-foxglove/hope_elink_health.py && sudo install -D -o root -g root -m 0644 a3/hope_elink_fault.py /usr/local/lib/hope-foxglove/hope_elink_fault.py && sudo bash a3/install_mdu_elink_scheduler.sh --robot-user "$(id -un)"'
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

## ELink startup checks and runtime diagnostics

HOPE uses **startup checks and runtime diagnostics**, not runtime ELink
motion-admission gates. The startup checks address the recorded dead-management-IPC
and post-readiness HAL reset/crash sequence without treating unknown monitoring
state as a proven ELink failure. It does not repair the vendor's native
reinitialization race or certify active-motion fault recovery.

### Operational contract

- Before managed HAL/Runner startup, passively verify the transfer process, HAL
  process and IPC listener. Do not connect to the vulnerable management socket.
  Unavailable/incomplete inspection rejects that startup attempt without a
  permanent lockout; a later explicit retry repeats the checks.
- Reject current-session logs containing the recorded failure chain after
  readiness. An earlier readiness marker cannot hide a later reset/crash.
- KILL always executes the existing cleanup/restoration sequence when permitted
  by the baseline lifecycle state and hardware-operation lock. It then verifies
  vendor process/listener restoration for up to eight seconds. A failed
  restoration is reported, not silently called successful, and remains retryable.
- Runtime process, telemetry, battery and clock observations are diagnostic.
  Missing/unaudited contracts, absent decoders, stale data, observer timeouts,
  publisher changes and failed clock qualification do not add START, Stand,
  Serve, Rally, gripper, role-change or KILL vetoes.
- Native simulation, standalone startup, automatic serve-to-receive transition
  and periodic swing rearming retain their existing behavior. Runtime ELink
  diagnostics do not participate in keyboard or remote action admission.
  Managed startup retains its existing PASSIVE requirement.
- Existing command-fault, joint-limit, serve-phase, role and vendor emergency
  protections are unchanged. Diagnostics never issue motor commands, limp,
  restart services, clear a vendor emergency, or automatically recover ELink.

The supervisor publishes `/hope/elink/health` with
`admission_enforced=false`. Its health/telemetry fields describe evidence, not
permission to move. A lost observer or reported fault does not change lifecycle
RUNNING to FAILED or create a durable admission latch. A new supervisor may
read legacy fault/session records for diagnostics, but does not
arm, write, clear or use them to deny operation. Preserve those records
for incident review; their existence alone no longer requires a lockout.

### Telemetry and battery diagnostics

Passive vendor subscriptions check publisher identity, generation, source and
receipt timestamps. BMS `header.seq=0` is accepted using strictly advancing
source timestamps; local receipt alone is not source progress. Two progressing
samples establish the diagnostic freshness result. Other channel assumptions,
sample-age thresholds and acquisition semantics still need matched-publisher
qualification; an unqualified result is **not an operational gate**.

Battery protection belongs to A3. Startup zero/absent packs, undervoltage,
reported battery communication/health statuses and invalid battery measurements
are bounded advisories in `telemetry.channels.bms.warnings`, not HOPE faults.
Missing/malformed message envelopes remain diagnostic errors. Existing vendor
emergency behavior is not overridden.

`foxglove/a3/elink_telemetry.example.json` remains deliberately unaudited.
Do not manufacture evidence by setting `audited=true`. For useful acquisition
qualification, capture an approved baseline HOPE session using the actual T3
factory-derived configuration with hand publishing enabled, not the unrelated
default Ultra publisher. Preserve the generated
`/tmp/hope_real/<session_id>/mdu/system/hal_elink_a3_gripper.yaml`,
`gripper_config_receipt.txt`, hashes, publisher identity and timestamped samples.
Resolve P1D0/Ultra versus `A3_T3D0` against the actual inventory; do not rename
hardware or bypass the pre-existing model/hash checks.

The original chrony probe time/boot/sequence is published on
`/hope/ntp/qualification` for diagnostics. Neither a missing update nor the
10 ms display threshold is a new runtime motion gate. For slow boot correction,
use the explicit [stopped Time Calibration flow](#pre-operation-clock-correction).
The boot-owned `elink.core` transfer is not stopped or required to disappear;
managed Runner/HAL/PTP consumers must still be stopped before stepping.

### Deployment and verification boundary

Update the matching supervisor/helper and these Python modules during an
approved stopped maintenance window, preserving the installed control-plane
layout and its module ownership: `hope_elink_health.py`,
`hope_elink_fault.py` (startup-log/legacy-record diagnostics),
`hope_elink_telemetry.py`, `hope_elink_telemetry_ros.py` and
`hope_elink_clock.py`. Both boards need the passive inspection and startup-log
helpers where called; vendor protobuf/ROS decoder packages are needed only for
the optional passive telemetry diagnostics. Runtime diagnostics require no
changes to the existing Runner/AimRT transport configuration.

Retained A3 evidence supports the dead IPC/HAL failure sequence, real BMS
zero-sequence/zero-pack layouts and slow clock correction. It does not establish
universal motion-veto deadlines or safety of restarting the native ELink stack.
Local isolated regressions verify code paths, not physical recovery, competition
latency or a successful full robot deployment. Validate those separately before
field release. These ELink checks do not patch vendor binaries, restart the
transport daemon, or reinitialize UART/IPC. The explicit lifecycle and clock
maintenance workflows retain their documented service stop/restore actions.

Read-only inspection of the current A3 confirmed a consolidated
`hope_operations.py` control plane and loaded-inactive observer/proxy
compatibility stubs. The candidate topology detector passed three read-only
runs, each capturing exactly the four common active units and verifying the
owned consolidated process in the supervisor's cgroup. The observed clock
offset was approximately 1.6 ms and
already qualified, so no live clock step or service stop/restore was performed.
Isolated topology/ordering regressions and that read-only inspection do not
constitute a live hard-step recovery test.

## Install the HDU supervisor

**Deployment-layout boundary:** the commands below are installation examples
for the **split** control plane, not an in-place migration recipe for every
field robot. First inspect `systemctl cat`/`systemctl status` for the supervisor,
observer and command-proxy units, including their actual processes. An existing
`hope-lifecycle-supervisor.service` running `/usr/local/bin/hope_operations.py`
owns the combined lifecycle, observer and proxy implementation. Preserve its
unit, field bundle, module versions and venue configuration; do not overwrite
them with the split sample units/modules or enable its inactive compatibility
stubs. Update that bundle through its matching deployment procedure. Changing
between layouts requires a separately reviewed stopped migration, not a clock
calibration prerequisite.

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
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_player_positions.py /usr/local/lib/hope-foxglove/hope_player_positions.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_elink_health.py /usr/local/lib/hope-foxglove/hope_elink_health.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_elink_fault.py /usr/local/lib/hope-foxglove/hope_elink_fault.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_elink_telemetry.py /usr/local/lib/hope-foxglove/hope_elink_telemetry.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_elink_telemetry_ros.py /usr/local/lib/hope-foxglove/hope_elink_telemetry_ros.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_elink_clock.py /usr/local/lib/hope-foxglove/hope_elink_clock.py
# Build/qualify the native monitor as described in foxglove/README.md first.
sudo install -D -o root -g root -m 0755 ~/hope_field_native_build/hope-monitor /usr/local/libexec/hope-monitor
sudo install -D -o root -g root -m 0755 ~/foxglove_a3/hope_time_calibration.py /usr/local/bin/hope_time_calibration.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_time_calibration_core.py /usr/local/lib/hope-foxglove/hope_time_calibration_core.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope-time-calibration.service /etc/systemd/system/hope-time-calibration.service
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope-chrony-burst.service /etc/systemd/system/hope-chrony-burst.service
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
  hope-lifecycle-supervisor.service \
  hope-time-calibration.service
```

In the split layout, these units form the always-available, fixed-action control plane needed
to start a stopped robot from Foxglove. They do not start HAL, Planner or
Runner, and they do not stop `agibot_pm`. In lifecycle mode, leave them running
after `KILL ALL & COLLECT` so the next session can be started without a terminal.
The time-calibration coordinator is idle until an explicit attended request;
enabling it does not step a clock. `hope-chrony-burst.service` is a fixed oneshot
dependency, not an enabled boot task. Clock calibration also requires the
approved bootstrap/PTP installation and the MDU helper's exact sudo rules in
the [hardware runbook](foxglove_first_hardware_test.md#72-进入-mdu-并配置最小-sudo-权限).

For an upgrade, `enable --now` does not reload an already-running Python
process. During the approved, supported/stopped maintenance window, restart the
updated monitor and the control-plane services that actually own the installed
layout after staging the entire matching
set; confirm that `/hope/ntp/qualification` is published by the updated monitor.
Do not restart these services during active robot motion. A legacy monitor's
display Bool lacks the original probe timestamp needed for these diagnostics;
neither it nor the new qualification snapshot adds a motion-admission gate.

Read-only verification:

```bash
systemctl status hope-lifecycle-supervisor.service hope-time-calibration.service --no-pager
ros2 topic echo /hope/lifecycle/summary --once
ros2 service type /hope/lifecycle/apply_config
ros2 service type /hope/lifecycle/start
ros2 service type /hope/lifecycle/kill_all_and_collect
ros2 service type /hope/lifecycle/time_calibration
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
`KILL ALL & COLLECT` recovery. Runtime ELink observations and legacy fault/session
records do not block cleanup or retry. STEP 4 retains the documented stop of `agibot_pm`.
An `aimrt_main_hal` inside the `/system.slice/agibot_pm.service` cgroup is the
expected vendor HAL and is accepted by preflight. STEP 4 stops the unit first,
verifies that this HAL has exited, and only then launches the fixed `hope-hal`
session. A HAL outside that cgroup, or one remaining after the unit stops,
fails closed; stale unmanaged HAL cleanup is an exceptional recovery action
rather than part of a normal one-click start.

`start` returns quickly. Progress and failures are published under
`/hope/lifecycle/**`; the console must reach `RUNNING · RUNNER` and
`START_COMPLETE_RUNNER_PASSIVE` before the operator uses Stand/Calibration/
Ready or the Server flow. The Console removes Refresh x_hit; shared HOME is
captured on policy entry.

Model17400's receive-only bundle does not enable `--serve` or consume a serve
CSV. Local `SERVER/RECEIVER` role state does not override bundle capabilities;
serve controls remain unavailable for this deployment. The selected policy,
deploy parameters and generated Planner profile must come from the same
verified bundle, with PPQ004 on Planner and packetizer.

## Kill behavior

`KILL ALL & COLLECT` is intentionally separate from E-stop. It is accepted for
a managed `RUNNING` or `FAILED` session without requiring a fresh Runner mode
or first transitioning to PASSIVE/PD_STAND. ELink diagnostic failures and old
fault/session records do not add a teardown veto.
Before confirming a kill:

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

## Independent PD Stand transition

Runner's `--pd-stand-blend-sec` defaults to **5.0 seconds**, independently of
the existing motion/policy entry blend. From PASSIVE, the transition starts
from measured joint position/velocity and zero gains, then blends to the
standing target and Kp/Kd using the existing quintic transition. Entry velocity
decays over 0.25 seconds. This changes the command transition, not the standing
target or policy, and is not proof of a safe physical stand on another robot.

Planner's `target_turn_max_pose_age_s: 0.2` receive-age and source-age checks
remain enabled by default. Sites can explicitly set
`target_turn_reject_stale_pose: false` to record and warn on pose age without
vetoing a shot; invalid-pose, strike-deadline and turn-angle checks remain.
See the [Planner site-option documentation](../../hope_ws/src/hope_planner_cpp/README.md).
Historical deployment evidence and its limitations are preserved in [the 2026-10-18 runtime field notes](runtime_field_notes_20261018.md).

## ELink transport scheduling

Choose the scheduler profile before installation: the default installer below
is pinned to T3. For the P1 BSP with the `61a3581e…` executable and `cff7ec66…`
ELink library, use the separate
[new robot P1 procedure](new_robot_commissioning.md#43-elink-scheduler-必须匹配-helper-与厂商-bsp).
The older `install_mdu_elink_scheduler_p1_field.sh` does not provide the current
hardware/privilege preflight and must not be mixed with the current helper.


The audited A3_T3D0 vendor ELink library creates three transport threads (main
loop, UDP send, UDP receive) outside the named AimRT executors. Changing only
`hand_control_worker` does not prioritize those threads. On this deployment,
`hope_elink_scheduler.py` places only the three transport threads on CPU 1 with
`SCHED_RR` priorities 24 (receive), 23 (main), and 20 (send). Receive/main must
preempt the sender: equal priorities retained millisecond waits during sender
CPU bursts. Runner, EtherCAT, the hand command cycle, and the serial bridge
retain their existing settings. CPU 7 was also trialed but retained waits.

Run `foxglove/a3/install_mdu_elink_scheduler.sh` as root on the MDU, keeping
its sibling `../helpers/hope_elink_scheduler.py` in the copied package. Pass
`--robot-user <account>` for the account used by the HDU's MDU SSH connection
(default `agi`, matching `HOPE_ROBOT_USER`). The installer backs up and patches
the installed lifecycle helper and installs the dedicated sudo rule described
below without restarting robot services. Managed START applies and verifies this policy after ELink
readiness and before EtherCAT/Runner startup; failure stops START admission.
The installer and lifecycle use the scheduler's read-only `check-hardware`
command to verify the same `A3_T3D0_ELINK_HAND_V2` profile. Installation rejects
an incompatible installed lifecycle before changing any files. PREFLIGHT and
START repeat the check before stopping `agibot_pm`, so a vendor update is caught
before the robot changes process ownership. They also execute the helper's
read-only `check-privileges` action through noninteractive sudo and query the
exact session's `apply` permission. Missing sudo setup fails early with
`ELINK_SCHEDULER_SUDO_NOT_READY` or `ELINK_SCHEDULER_APPLY_NOT_PERMITTED`.
The supported paths are:

| Component | Audited profile value |
| --- | --- |
| Robot model (`/agibot/data/info/model`) | `A3_T3D0` |
| ELink factory configuration | `/opt/agibot/config/hal_elink/hal_elink_a3_cfg.yaml` |
| EtherCAT configuration | `/opt/agibot/config/hal_ethercat/hal_ethercat_a3_t3d0_cfg.yaml` |
| `/opt/agibot/bin/libhal_elink_pkg.so` SHA-256 | `f83690f1c8a3d7e1f50a7c38bd5ed2ccc01543274b680d548af38f7f154225af` |
| `/opt/agibot/bin/aimrt_main` SHA-256 | `9ab4c3fa441b4c13fac543b8778cd650a5e1751cad26fda1f4a918912e113ad6` |

These hashes match the T3 deployment's existing HAL-clock audit manifest; this
fix does not authorize another vendor build. The earlier `A3_P1D0` scheduler
restriction was inconsistent with this lifecycle. P1/Ultra remains unsupported
by the combined startup path. The separate legacy
`install_mdu_elink_gripper_backend.sh` targets P1/Ultra and is not a prerequisite
for this T3 lifecycle, which renders a session-owned ELink configuration.

The scheduler also checks the exact executable and mapped ELink library hashes, managed
session path, process/thread start times, and the audited three-thread layout.
An unknown vendor build requires a new audit, not removal of these checks.

To check the installed pair without root or starting any process:

```bash
python3 /usr/local/libexec/hope_elink_scheduler.py check-hardware \
  --lifecycle /usr/local/libexec/hope-lifecycle
```

For an existing session, the root-only helper accepts `inspect`, `apply`,
`verify`, and `restore`, each with
`--config /tmp/hope_real/<session>/mdu/system/hal_elink_a3_gripper.yaml`.
Execute `/usr/local/libexec/hope_elink_scheduler.py` directly; its fixed
`/usr/bin/python3 -I` shebang runs isolated from user Python paths and startup
modules. Privileged actions check that the helper and all parent directories are
root-owned and have no group/other write permissions.
Apply needs no process restart and publishes no robot commands. The first
pre-change settings are retained in `/var/lib/hope/elink-scheduler/`; repeated
apply preserves that rollback receipt. Restore affects the same process/thread
instance only. To disable persistence as well, restore the lifecycle helper from
the installer backup under `/var/backups/hope-elink-scheduler-*` after reconciling
any subsequent changes.

During `apply` or `restore`, catchable termination (`SIGINT`, `SIGTERM`,
`SIGHUP`) is deferred to cancellation checkpoints. Cancellation before the final
verified commit rolls touched threads back to their pre-call settings, keeps
cleanup protected from repeated signals, and exits unsuccessfully. An ordinary
error or `KeyboardInterrupt` also attempts rollback; a failed rollback is
reported, never treated as success. A late cancellation after the verified
commit may report failure with all requested settings already applied.

`SIGKILL`, process crashes and power loss cannot be handled in-process. If the
same ELink process survives with a partial scheduling change, an administrator
can use the existing explicit `restore` action. It requires the matching
root-owned, non-writable-by-others original receipt, exact process/thread start
times, audited binaries and hardware. In addition to normal factory/target
settings, **only restore** admits the known intermediate state: `SCHED_OTHER`,
priority 0, CPU `[1]`. Unknown policies, priorities or CPU sets, missing/corrupt
receipts and replaced processes remain rejected. Do not delete or recreate a
receipt to bypass these checks. `apply`, `verify` and normal START remain strict;
they do not silently repair partial settings. A failed START does not itself
guarantee ELink teardown; use the attended managed recovery procedure if explicit
restoration is unavailable. No recovery action sends gripper or joint commands.

Scheduling readback and passive transport timings do not measure finger motion
or ball release. Physical serve repeatability must be tested separately.

## Field v20 serve asset

The operator reported successful ball contact with
`a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/a3p_op3_serve025_edge_center_v20.csv`
on 2026-10-18. Its adjacent manifest records the exact asset hash, field runtime
requirements, simulation results and remaining limits. This report establishes
successful contact, not a measured success rate or consistently legal serves.

The CSV preserves v19's left arm, body/leg channels, Prepare target, event
sequence and recovery from frame 160. Only right-arm position/derivative columns
in frames 63–159 change. Two measured-response models were used to move the
strike-window racket center toward a 40 mm higher target while retaining its
normal. Smooth admission at frames 64–71 avoids the backswing proximity of an
earlier discarded candidate. The 40 mm value is an optimization target, not an
independently measured robot displacement.

This is a selectable field asset, not a default-motion replacement. Upload it
through the field CSV interface only while the lifecycle is stopped and idle.
Keep the previous selected asset/hash for rollback. Use managed START and verify
the loaded CSV hash and `START_COMPLETE_RUNNER_PASSIVE` before operator motion.
The recorded deployment used 468 frames at 100 Hz, handoff frame 467, and
`serve_release_lead_frames: 3`: the CSV RELEASE event remains at frame 48 and the
command is sent at frame 45. Do not shift the CSV event as well.

The successful field deployment used a separate existing Runner build, the
`new_20261015` gripper profile (grab/release positions 850/1500, force 1, velocity
100), and its previously tested Prepare transition. Exact Runner/config hashes
are in the manifest. Those field runtime/profile changes are not introduced by
this asset addition; a stock main build does not reproduce the full tested
setup merely by selecting v20. Preserve the field runtime when using this
asset; treat a different runtime or gripper as a new validation configuration.
The asset is selected by upload, so the packaged v14 default remains unchanged.

Offline validation used five measured v19 joint-response cycles (first three
for selection, last two held out). In 315 position/delay cases per response
model at assumed 60–120 ms detachment delay, contacts increased from 295 to 315;
legal bounce sequences increased from 236 to 264 (lag/gain) or 255 (FIR).
Native-controller floating-base MuJoCo cases at 60/80/100/110/120 ms all produced
the legal bounce sequence without simulated falls. These are conditional
simulation results, not field success rates. Wider 0–180 ms sweeps introduced
misses at 10–50 ms. Actual physical detachment latency is not identified; the
rigid gripper CAD, ball release position and contact materials remain model
assumptions. Keep these limitations when comparing future field results.

## Verification boundary

Repository unit tests, shell parsing and extension packaging prove only the
wire/configuration restrictions and fixed command construction. A supervised
three-machine dry rehearsal and then an attended hardware trial are still
required. In particular, verify SSH keys, MDU sudoers, tmux availability,
exact deployed Runner package, ROS service discovery, process identity, log
flush, `agibot_pm` restoration and the physical PASSIVE/PD_STAND transitions.

## Field gripper and mocap configuration

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

Combined schema 6 loads both main's older operator/gripper schemas and field
schema 5, preserving gripper, pants, mocap source, VRPN endpoint/units, table side
and network settings. Older clients retain settings they cannot edit. Install
the matching HDU core/supervisor/helper and Console only during a separately
approved stopped deployment. The Mac local build does not install them remotely.
Gripper and mocap selection do not change the selected serving CSV.

Mocap capture is independently selected and started on the operator computer.
See [Motive / ChingMu setup](foxglove_mocap_setup.md),
[VRPN pants assets](vrpn_pants_profiles.md), and the
[final Mac build](macos_deployment.md). ChingMu feeds the Mac VRPN receiver;
the Mac publishes ROS2 to the HDU. No MDU mocap-server connection is required.

## Prepare motion and release timing

Prepare retains its five-second duration and READY target. The left arm clears
first; the right shoulder pitch/roll and elbow begin at 2.0 s, then shoulder yaw
and wrists at 2.6 s. CSV playback is unchanged. The operator reported no further
Prepare leg contact after this field change; the regression test checks the
command ordering, not physical collision safety.

The two packaged ping-pong runtime configurations use release lead 3 (frame
48 event -> frame 45 command at 100 Hz). The controller accepts 0–4 frames only
while idle; frame 44 was a trial, not the retained default. These parameters do
not identify the physical command-to-detachment delay. The v20 CSV asset was
added separately in #53; these runtime changes do not select a new serve CSV.
Retain the previous asset for rollback when changing an installed selection.
