# HOPE A3 Console

Download `hopeopen.hope-a3-console-1.8.13.foxe` from this directory and open it in
Foxglove Desktop's Extensions screen to install the console. No Node.js build is
needed for this prebuilt release. Import
[`model21800_console.json`](../../layouts/model21800_console.json) and connect to
`ws://<HDU-IP>:8766` after installing the matching HDU/Laptop services.

The public console supports Xbox A/B/X/Y mode actions, LT movement, LB+RB software
E-stop and software reset; Normal Play and Kernel Mode; and the public 24-sticker
calibration plus separate legacy V2/V3 profiles. It calls the dedicated services
in `bridge_params_control.yaml`; Runner remains the body-command owner.

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
directory's `.gitignore` and the download links, and remove the superseded installer.

Install the generated `.foxe` through Foxglove Desktop's Extensions screen,
then import `foxglove/layouts/model21800_console.json`.

Before start, the operator chooses `UCB_P1 · DIRECT` or `UCB_P2 · ROTATE 180°` and
confirms configuration. P2 tracks Motive rigid body UCB_P2 and rotates every ball
and rigid-body pose about the table centre, so both end robots consume the same
policy convention. The selection is locked while a lifecycle is active.

The no-OptiTrack receiver test sequence shown by the UI is Stand, then Ready.
`START SYSTEM` does not launch or wait for the Laptop OptiTrack stack, and
the optional Motive address may be left empty. `Ready` does not require a fresh
base packet. The native Runner still owns the
role/mode transition and all of its fault checks. With OptiTrack absent there
is no live ball/base stream, so this proves Runner entry and attended motion
behavior rather than incoming-ball performance.

The Console uses the current pelvis-aligned Motive asset. The former Venue
Calibration card, Load/Export JSON and Cali V2/V3 buttons are removed. Opening
the panel starts no processes.

| Control | State | Effect |
| --- | --- | --- |
| Motive Live Asset | ASSET ACCEPTED / CHECK MOTIVE ASSET / WAITING FOR MOTIVE; separate live pelvis state | Read-only current body status |
| Marker audit | Visible / defined markers, e.g. 15/17 | Definition count follows the streamed ModelDef |
| Serve CSV | Select/upload while stopped | Existing Runner CSV validation and serve controls |
| OptiTrack / Normal play | Select while stopped; start OptiTrack separately | Uses the imported asset's nominal pelvis frame |
| Recording | OFF / STARTING / ON / STOPPING | Explicit MCAP start/stop |

For the 17-sticker shell, import
[A3_v3_17_stickers.motive](../../../agibot/mocap_sticker_shell_v3/motive_asset/A3_v3_17_stickers.motive).
Its internal NodeName, AssetName, JointName and all constraint joint references
are `UCB_P1`. Keep its pivot/axes unchanged. Disable any old body with the same
canonical name, select P1 in the Console, set Motive Streaming Up Axis to Z,
and enable Rigid Bodies. Enable Labeled Markers for the physical visibility
audit. Then use the existing OptiTrack start control.

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
`TIME CALIBRATION` and `Refresh x_hit` remain separate operations. No
`calibration/ucb_robot_to_pelvis.json` is read, created or overwritten.

The pinned 17-marker check is opt-in via `localization_source:=motive_asset`;
legacy measured V2/V3 calibration is opt-in via
`localization_source:=calibration start_calibration:=true`.

The 17-sticker file was copied from the user-supplied USB file, byte-identical
before renaming to the asset at upstream commit
[`10b1bd710`](https://github.com/hitchopen/HOPE/commit/10b1bd710c074cf9851a3ca2f63494dc23a39e06).
Only the 21 exact body-name/reference attributes were renamed from `P1` to
`UCB_P1`; marker coordinates, pivot, axes, IDs and other settings are unchanged.
The USB original is backed up as `A3_v3_17_stickers.motive.original-P1.bak`.
The older 17-sticker asset and original exports under `motive_asset/source/`
remain available. These offline preparation checks do not establish successful
Motive import or live NatNet tracking.


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
Laptop marker monitor and runtime helper, and install Console 1.8.13. Update
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

The earlier reviewed main installer is
[hopeopen.hope-a3-console-1.8.13.foxe](hopeopen.hope-a3-console-1.8.13.foxe).
It includes main's 24-sticker calibration interface. For the new 17-sticker
asset workflow, use the locally built
[Console 1.8.13](hopeopen.hope-a3-console-1.8.13.foxe) described above.
See [current Runner/serve behavior](../../../docs/operations/runtime_xbox.md)
for Kernel completion, v12 timing and release limitations.
