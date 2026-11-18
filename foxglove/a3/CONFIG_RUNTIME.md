# Configuration runtime candidate (not installed)

The editable source is `/var/lib/hope-lifecycle/config.json` on the selected HDU.
Foxglove receives its complete values, revision and runtime evidence together on
`/hope/lifecycle/config/status`. Successful `apply_config` means saved, not applied.
The Console and server independently require a fresh matching report for START
and time calibration. STOP and E-stop admission are unchanged.

The candidate adds:

- A canonical environment wrapper for the seven HDU network consumers. It
  overrides inherited peers and records its source and network hash.
- A root audit/reconciler that reads actual process environments, child processes
  and generated DDS profiles. It only refreshes active stale services while the
  shared hardware lock is held, with a fresh matching STOPPED owner, idle clock
  calibration, and no local Planner or remote MDU Runner. Failure/unreachable
  state does not prove idle. Each service gets at most one refresh per revision.
- A read-only laptop probe that compares the generated receipt, processes,
  actual ordered ROS parameter files and endpoint settings. It identifies the
  resolved executable/installed-script paths. These are startup parameters;
  it does not claim live dynamic ROS parameter readback or tracking qualification.
- A shared compatibility source for old wifi/wired shell aliases and the Xbox
  target. Those aliases contain no independent robot or mocap endpoint values.
- Explicit VRPN server admission and no site-specific Motive launch fallback.

`network.env` retains installation metadata (accounts and helper paths). Drop-ins
are generated from the target's captured effective units by `hope_config_stage.py`;
their original commands and target-specific hardware arguments are preserved.
`laptop-runtime.env` is a generated receipt, not another editing surface.

## Activation boundary

No part of this candidate is installed, enabled, restarted or applied by building
or packaging it. Do not install the Console alone: old servers do not publish the
new evidence, so START and time calibration will remain unverified/disabled.

The current robot is A3_P1D0. The repository helper contains another hardware
variant. Apply only the reviewed configuration-related delta to the current
installed helpers; never replace them wholesale. Preserve the selected
model14200best policy, deployment YAML, serve CSV, binary and hardware transports.
The local `hope_ws/install` resolves into `Hope_v11_colcon`; source edits alone do
not update those installed launch/script/config files.

After the operator ends testing, recapture identities, configuration, effective
systemd units and installed-file hashes. Save rollback copies of all touched
files. Prepare drop-ins from that exact capture and reject source-hash drift.
Install the paired modules, laptop probe/evidence helper, narrow helper changes,
resolved installed launch changes, compatibility aliases, root audit unit and
Console together during an idle maintenance window. Replace the peer assignment
in network.env with canonical derivation; preserve all installation metadata.
The native-operations installer is not a shortcut for this P1D0 field migration.

Re-read the same configuration via apply_config to regenerate the laptop receipt.
Confirm atomic saved values and all audit rows match before considering activation
complete. Verify actual installed hashes/paths, fresh local Foxglove status and
real browser button reasons. Keep pre-existing asset/world confirmations intact;
this migration cannot qualify an unconfirmed mocap asset or initiate robot motion.

Rollback restores only the saved configuration-related modules/drop-ins/helper
hunks and prior Console, then reloads/restarts the affected communication services
while idle. No model or policy rollback is required because they are not changed.

## Audit scope

The active VRPN chain was inspected read-only with server 192.168.100.99, port
3883, mm/0.001, v4 and P1 in saved config and actual client/relay launch inputs.
This is a point-in-time startup-configuration check, not proof of fresh tracking.
Hidden launch defaults 192.168.1.100 (VRPN) and 192.168.100.111 (generic Motive)
are removed by this candidate. The upstream vendor examples (`cfg.yaml` with
130.149.82.37 and VRPN example localhost) are not loaded by the managed HOPE
launch; they remain examples. Historical logs/rollback snapshots retain their
original addresses as evidence. Offline replay inputs are not live configuration.

Known unrelated baseline failures: 4 legacy schema-upgrade cases in
`test_mocap_setup.py` reproduce unchanged on the field checkout; one i18n source
lint flags an existing literal in `LiveTelemetry.tsx`. Do not label the entire
repository test suite green. Tests here do not prove installed or physical behavior.

## Neighboring field repair evidence (2026-11-09 18:46 +08:00)

The chat titled `核实近期改动是否已部署` recorded that raw UCB_P1 was
already arriving near 299 Hz while the base adapter still had both setup
confirmations false and emitted valid=0. It also found the IMU process on .194.
After the operator's visual attestation and supported-stop confirmation, that
chat performed managed Stop, reloaded the receiver, refreshed IMU, and started
to PASSIVE. Its receipt shows valid=1/fresh=1 and host_robot_available=true,
with the original model14200best policy/CSV hashes retained.

This candidate now exposes raw-pose freshness separately from robot localization,
the base reason, base adapter confirmation parameters, and packetizer verification.
Do not revert the neighboring chat's confirmed setup during activation.
Schema 7 stores both operator confirmations in config.json. Changing the robot
address, source endpoint/protocol/units, pants or side clears them; save the new
setup first, then verify it and explicitly save its confirmations. These are
operator attestations, not VRPN geometric verification or hardware discovery.
Robot identity/assets changed without a config-field change still require the
operator to clear and recheck confirmations.

`hope_config_stage.migrate_attested_config` prepares a migration from a schema-6
snapshot, the exact attested receiver receipt and its recorded operator evidence.
It checks the receipt SHA256 and the complete robot/mocap/profile/side binding.
A subsequent managed session-id change is allowed only with every other receipt
byte unchanged. It preserves the neighboring recorded true/true confirmations,
advances the revision, and outputs hashes to recheck before activation. It never
writes installed config or starts services. The prepared field migration is in
configuration-attestation-migration.json. Recheck source hashes at cutover.

Remove retired lifecycle.env confirmation assignments using `attestation_free_env`
only after preserving them in the migrated canonical config. The helper receives
both flags as explicit canonical arguments; retired environment variables cannot
override them. The receiver receipt remains a generated output. Both the actual
receiver parameters and this receipt must match the canonical config before
APPLIED. Offline tests cover binding invalidation, legacy requests, explicit
reconfirmation, environment precedence and drift rejection.
