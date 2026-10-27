# HOPE A3 Console

Download `hopeopen.hope-a3-console-1.8.18.foxe` from this directory and open it in
Foxglove Desktop's Extensions screen to install the console. No Node.js build is
needed for this prebuilt release. Import
[`model21800_console.json`](../../layouts/model21800_console.json) and connect to
`ws://<HDU-IP>:8766` after installing the matching HDU/Laptop services.

The current deployment is **model17400 + PPQ004**, with mocap-based receive play
and the pelvis-aligned 17-sticker shell. Model17400 is receive-only: serving and
Kernel mode are not qualified for this bundle. The console includes Xbox
A/B/X/Y actions, LT movement and LB+RB software E-stop. It calls the dedicated
services in `bridge_params_control.yaml`; Runner remains the body-command owner.

Install the matching control-plane helpers, configs and service units from this
same checkout. See [Runtime operation](../../../docs/operations/runtime_xbox.md)
for the complete setup. A Foxglove panel installation alone does not install
robot-side services or start motion.

To rebuild the installer from the public source:

```bash
npm ci
npm run package
```

`npm run package` performs the production build and packages the JavaScript,
fonts, manifest and notices into the `.foxe`. `dist/` and `node_modules/` are
build inputs/outputs; the ready-to-install `.foxe` is committed as an ordinary
Git blob. When changing console source, regenerate the installer from that source.
When changing the release version, update the exact filename allowed by this
directory's `.gitignore` and the download links.

This release uses combined lifecycle schema 6, migrating field schema 5 without
losing gripper/pants/VRPN settings and retaining Mac/Linux operator selection.
Choose the gripper and **MOCAP SYSTEM** explicitly while STOPPED; see
[mocap setup](../../../docs/operations/foxglove_mocap_setup.md).
The Mac container account uses group `hope`, not Ubuntu's system `operator` group.
Install matching services during an approved deployment; installing this panel
alone does not update a live schema-5 HDU.

Install the generated `.foxe` through Foxglove Desktop's Extensions screen,
then import `foxglove/layouts/model21800_console.json`.

Before start, choose **Linux** or **Mac** under **Operator computer**, confirm
that computer's current Laptop IP, choose `UCB_P1 · DIRECT` or
`UCB_P2 · ROTATE 180°`, and confirm configuration. The operator profile selects
the trusted HDU SSH account/helper configuration; the panel does not accept
usernames, helper paths or shell commands. Switching profiles requires a stopped
lifecycle and the matching profile-aware HDU supervisor. An older supervisor
does not supply the required profile topic, so upgrade the services before
using this release. P2 tracks Motive rigid body UCB_P2 and rotates every ball
and rigid-body pose about the table centre, so both end robots consume the same
policy convention. The selection is locked while a lifecycle is active.

Console 1.8.18 verifies the physical table end directly from raw player mocap
poses before the P2 rotation. P1 is selectable only with a fresh tracked player
at `x < 0 m`, and P2 only at `x > 2.74 m`. Either player body can establish
presence; a ball or table pose cannot. The buttons subscribe to
`/hope/mocap/players/UCB_P1` and `/hope/mocap/players/UCB_P2`
(`geometry_msgs/PoseStamped`, metres, `hope_table_global`). Both Motive and VRPN
relays publish these unrotated topics. Missing, invalid or replayed source stamps
cannot keep a player fresh beyond 250 ms. These START-only observation topics
are capped at 30 Hz without reducing the base-pose or ball streams. Any-player
eligibility checks end occupancy, not robot identity or the runtime's body name.

Start the mocap receiver while STOPPED, select the side and confirm configuration.
The HDU independently repeats the position check at START before hardware startup.
START also requires the HDU's published `player_positions` verdict; the panel
explains an HDU discovery mismatch while side selection still uses raw mocap.
Side changes remain locked through STARTING/RUNNING/FAILED/KILLING until STOPPED.
Player positions are not used to reassign sides during play.

Normal mocap-based SERVE/RALLY requires a fresh MDU base-pose mailbox. A 100 ms
signal gap or an explicit invalid pose stops RALLY into the existing PD_STAND
blend; SERVE requests its authored abort and validated recovery first. Fresh
mocap does not automatically restart play. Stand, emergency PASSIVE, cleanup
and offline Kernel/Pure Serve remain available. Ball occlusion alone does not
trigger this watchdog.

At 20 ms without a real pelvis update, the model uses a marked last-pose hold,
not a new measurement. Its timestamps/sequence and the 100 ms rescue timer are
never renewed. The footer shows the real-sample age and HELD state.
HELD does not shorten mode admission or serve completion to 20 ms: these use
the actual observation/rescue deadline. New-flight commits still wait for a
real pose; replay/field validation must qualify the effect on strike timing.

Normal managed mocap play also requires independent Console and operator-relay
challenge echoes. The panel replies only to the current boot/session challenges;
closing/freezing/disconnecting it triggers controlled recovery after **1 s**.
The managed operator-computer relay retains its separate **100 ms** timeout.
Delayed/replayed echoes cannot extend either deadline. No timer publishes
cached heartbeats. Recovery motion can take longer; Kernel/Pure Serve and Teleop
are exempt. Echoes depend on panel rendering: switching away from the Console,
minimizing/backgrounding or overloading it for more than 1 s may still stop play.
The longer UI deadline tolerates short delays, not an absent Console. Qualify
actual foreground/background Foxglove/Wi-Fi timing before operation.

The control bridge enables client publishing **only** on
`/hope/runner/operator_echo_hdu_flat`; no command publisher or generic parameter
access is enabled. The operator relay's heartbeat is bound to its launch session,
not inferred from general DDS process presence.

The footer displays fresh Runner mocap health and **Stopped: mocap lost**,
independently of the last action acknowledgment. The stop reason persists after
the stream returns until explicit play re-entry succeeds. The read-only
`/hope/runner/mocap_safety_hdu_flat` telemetry is boot-checked and expires after
one second. It does not alter the existing schema-2 Runner state/action wire.

Deploy the updated relays on the mocap computer, `hope_player_positions.py`
alongside the HDU supervisor modules, the updated supervisor service/control decoder,
Runner transport relay, and a rebuilt MDU Runner with its matching AimRT config
together with this panel. Also update the control-bridge config and managed
lifecycle helper, and rebuild the operator workspace to install the heartbeat
helper and matching VRPN launch file. Configure `/etc/hope-foxglove/network.env` peers as
described in the lifecycle guide. A panel-only upgrade cannot
provide the native dropout response. Bench verification precedes attended
hardware validation.

The Console uses the current pelvis-aligned Motive asset. The former Venue
Calibration card, Load/Export JSON and Cali V2/V3 buttons are removed. Opening
the panel starts no processes.

| Control | State | Effect |
| --- | --- | --- |
| Motive Live Asset | ASSET ACCEPTED / CHECK MOTIVE ASSET / WAITING FOR MOTIVE; separate live pelvis state | Read-only current body status |
| Marker audit | Visible / defined markers, e.g. 10/12 | Definition count follows the streamed ModelDef |
| Serve CSV | Select/upload while stopped | Existing Runner CSV validation and serve controls |
| OptiTrack / Normal play | Select while stopped; start OptiTrack separately | Uses the imported asset's nominal pelvis frame |
| Recording | OFF / STARTING / ON / STOPPING | Explicit MCAP start/stop |

For the current 12-sticker-per-player installations on the unchanged 17-station
shell, use the matching [P1/P2 Motive pair](../../../agibot/mocap_sticker_shell_v3/motive_asset/README.md),
named `UCB_P1` / `UCB_P2` with IDs 9 / 10. They match the ChingMu selections
and nominal CAD geometry. Confirm import/re-export, installed pattern and
pelvis pivot/axes; resolve existing name/ID collisions in a backed-up project.
Select the matching table side, set Motive Streaming Up Axis to Z, and enable
Rigid Bodies. Enable Labeled Markers for the physical visibility audit.
Then use the existing OptiTrack start control. Shared-only occluded views can
still confuse these patterns; validate both bodies together before play.

The default launch and Laptop lifecycle helper use `localization_source:=motive_live`.
This mode reads no local Motive file and imposes no fixed SHA-256, marker count
or marker layout. It accepts the selected `UCB_P1`/`UCB_P2` body with a valid
live ID and structurally valid ModelDef, assuming the imported origin/axes
already describe `pelvis_link`. The marker monitor's `expected_count: 0`
follows the live definition; it never caps a 17-marker body at 12.

`ASSET ACCEPTED` means the current body definition was accepted, not that its
geometry was compared with a reference. The status text identifies live versus
pinned mode. Stale data, malformed definitions, source timestamps, world receipt,
pose jumps and invalid quaternions retain their existing checks. The identity
extrinsic, table-side transform and single policy Z offset preserve the schema-2
base packet. Its provenance ID is not an asset-file checksum.

This mode assumes pelvis alignment; it does not measure the physical mounting
or verify the Motive table origin. Set the table/world origin in Motive as before.
`TIME CALIBRATION` remains a separate operation. The Console no longer exposes
`Refresh x_hit`; shared HOME is captured when entering policy mode. No
`calibration/ucb_robot_to_pelvis.json` is read, created or overwritten.

The side-specific 12-marker check is opt-in via `localization_source:=motive_asset`;
rebuild/install both reference files and the updated relay/contract together.
Legacy measured calibration is opt-in via
`localization_source:=calibration start_calibration:=true`; its `stickers_v3`
profile still requires all 17 sites and cannot calibrate these new subsets.

Original measurement exports are preserved in the
[shell's source-evidence folder](../../../agibot/mocap_sticker_shell_v3/documents/source_evidence/optitrack/README.md).
Obsolete all-17 imports are no longer in the player release folders; historical
regression fixtures are not installed. Native import and live tracking of the
new pair still require deployment validation.


Pure Serve uploads accept the existing full31 serve025 CSV schema: 468 frames,
100 Hz, 31 SDK joints and the existing event/limit/velocity contract. The MDU
runs the same C++ parser through `--validate-serve-timeline`, which exits before
backend or gripper initialization. Successful uploads are stored by SHA-256;
invalid uploads preserve the previous selection. The hash is checked before
Run and again on MDU launch. Only Pure Serve uses the uploaded selection;
normal play continues to use the package's built-in serve. The Runner's
`--serve-only` prevents policy/receiver/reference modes even for keyboard or
other clients. Its final CSV frame returns through the existing Stand entry
blend. Normal serve still hands off to native receive policy.

In this build_6 checkout, the HDU's `DistributedRecorder` coordinates the
session-owned Laptop, HDU and MDU capture workers. Recording defaults to OFF;
Record ON/OFF controls MCAP, Runner trace/obs, Planner and mocap/flight CSVs,
including the MDU body-drive/IMU recorder. Kernel mode only requires its MDU
worker. This supersedes the earlier MCAP-only recording description.
The UI displays actual STARTING/ON/STOPPING/OFF/ERROR and segment identity.
Closing the panel does not cancel recording; use OFF and wait for OFF.
OFF does not stop Runner/HAL, while lifecycle KILL requests recording closure
as part of stopping the session. Routine diagnostic logs remain active.
See [three-machine recording](../../../docs/operations/foxglove_lifecycle.md)
and the [field runbook](../../../docs/operations/deploy_and_gate3.md) for capture
contents, collection and the distinction between exporting and stopping.

The field API is `/hope/field/command`, a dedicated `SetParameters`-shaped
service accepting one string parameter named `request` with a fixed JSON
operation. Status is `/hope/field/status`. Content is sent as JSON on SSH stdin
to fixed laptop/MDU helper endpoints; clients cannot choose remote commands or
destination paths. Persistent selections are in
`/var/lib/hope-lifecycle/field/selection.json`; MDU CSVs are in
`/agibot/hope-field-assets/`. The writable MCAP directory is
`/agibot/data/bag/hope_pingpong_telemetry/`.

For the 17-sticker update, rebuild Laptop `hope_bringup`, update the staged
Laptop marker monitor and runtime helper, and install Console 1.8.14. Update
the HDU supervisor to stop polling legacy venue files. This change does not
require replacing the Runner or its policy. Broader deployment changes still
follow the wired runbook. The shared HDU Python
modules include `hope_field_assets`, `hope_field_operations`, `p1_calibration`
and `base_pose_contract`; the MDU needs `hope_field_assets`. Installing only the
extension does not upgrade the remote Runner.

The gripper indication is intentionally command-publication telemetry, not a
finger or ball sensor. The operator loads only after the raised hand has
stopped, clears the loading/swing area before play, and has a person ready
before an aborted retained-ball OPEN.

The CPU card reports aggregate HDU load and the process with the largest CPU
delta. E-stop is assert-only: the panel follows the persistent HDU latch and
does not expose a reset operation.

The system-lifecycle card is backed by the separately installed HDU supervisor
documented in `docs/operations/foxglove_lifecycle.md`. `START SYSTEM`
replaces runbook STEP 0/1/2A/2B/4/5 and leaves the Runner in PASSIVE; it does
not replace physical robot support or access to the hardware E-stop.

`TIME CALIBRATION` is a separate attended maintenance action for runbook 10.4.
It is enabled only with confirmed network configuration, a stopped lifecycle,
and a fresh failing NTP gate. The fixed root coordinator stops MDU consumers
before HDU consumers, performs at most one hard-step per maintenance cycle,
restores in dependency order, and publishes persistent status after the expected
temporary 8766 disconnect. Before preflight it may recover only an exact,
same-boot MDU `STOPPED` receipt created by the managed helper; arbitrary stopped
services remain fail-closed. Structured helper reasons are shown directly in
the panel. It shares an exclusive operation interlock with `START SYSTEM`; the
panel exposes no clock parameters or shell input. The NTP hard gate is selected
source + Leap Normal + at most 10 ms wall-clock offset. Chrony Skew is displayed
as frequency-uncertainty audit telemetry and is deliberately separate from the
ROS 2 message-latency card.

## Public release

The current installer is
[Console 1.8.18](hopeopen.hope-a3-console-1.8.18.foxe), built from this directory's
source. Install its matching HDU/MDU services and relays, then confirm the
operator profile and model17400/PPQ004 selections before START.

## Model17400 receive policy

The current receive deployment uses model17400. Selection is allowed only while
stopped. It verifies the installed HDU Planner
profile and MDU policy/deploy hashes, persists the choice, and restores both
endpoints after a failed or uncertain write. The displayed choice is the saved
bundle; START SYSTEM loads it. Install the matching bundle before selecting it.

Model17400 requires PPQ004 on both the Planner and packetizer, shared session
HOME, and mocap receive mode. Kernel and serve are not qualified for this model.
Selecting its receive-only bundle clears a previously selected serve CSV.

Install `a3/hope_policy_models.py` with `hope_field_assets.py` in
`/usr/local/lib/hope-foxglove` on HDU and MDU, together with the updated lifecycle
helper, HDU supervisor and field-operations module. Prepare the bundle using
`a3_deploy/a3_deploy_example/scripts/prepare_model17400_bundle.py` from HOPE main.
It pins the downloaded policy and deploy hashes, rejects incomplete or mixed
architecture distributions, and includes the HOME publication configuration.
Its x86 inspection option validates the matching source build when cross staging
ARM64; the receipt explicitly records that the ARM executable was not run.

Install the generated deployment at `/agibot/a3_deploy_model17400` and the
generated `model17400.json` profile at
`/var/lib/hope-lifecycle/policy-models/model17400.json`. Add `hdu_entry.json` and
`mdu_entry.json` under the `model17400` key in the respective version-1 catalogs;
preserve all existing model entries and `selected.json`. Both catalog entries
bind the complete installed files by SHA256. Package preparation never installs,
selects, restarts or moves a robot. Hardware installation and PASSIVE verification
remain a separate managed operation.

## PPQ004 ball model

For model17400, select local three-star **PPQ004** while STOPPED. The selector
writes matching 14-parameter overlays to HDU and Laptop,
rolls back partial writes, and distinguishes saved/configured/actually loaded
parameters. Restart the separately managed Laptop mocap/packetizer after a
selection, then START SYSTEM.
Install `foxglove/a3/hope_ball_models.py` alongside the existing HDU helpers in
`/usr/local/bin`, update `hope-lifecycle` on both hosts, update the Laptop helper
and ROS launch files, and install the matching PPQ004 YAML/receipt pair. Rebuild
and install the Console through the existing deployment workflow. Source-only
changes do not hot-reload an already running supervisor or packetizer.

See `calibration/ppq004_20261020/README.md` for complete trajectories, provenance
and offline reproduction. No speed-adaptive model is selected automatically.

### Hit speed scale

The Ready controls expose a nonnegative racket-velocity multiplier. Wait for
fresh Planner diagnostics after Apply; a service receipt alone is not displayed
as an applied value. The hardware profile persists the accepted scale across
restart. Kernel Mode and exact question fixtures do not expose this control.
Position, normal and strike deadline remain unchanged; landing predictions still
refer to the original, unscaled solution. This is not physical-hit qualification.
