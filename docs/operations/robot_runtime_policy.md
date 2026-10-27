# HDU Wi-Fi and periodic background policy

This policy targets the A3 HDU's **vendor RTL8852BE driver**, `wifi_hdu`, and
`/proc/net/rtl8852be/wifi_hdu/scan_deny`. It is not a generic mac80211 recipe.
Install it while the managed lifecycle is `STOPPED`, `busy=false`, with local
Wi-Fi access. Installation does not run START, restart robot control, or reconnect
Wi-Fi. Keep the existing per-robot lifecycle settings.

## 1. Foxglove START enforcement

HDU preflight calls `hope_wifi_policy.py check` before background cleanup or
robot launch. The policy service must be active and enabled, power saving must
be off, and scan blocking must match association state. An open manual scan
window rejects START with `WIFI_POLICY_NOT_READY`.

The background guard stops only its explicit HDU camera allowlist; vendor
startup/audio, remote control and UI/status dependencies are preserved. This
does not by itself verify physical handheld or E-stop availability.
If `/etc/hope-foxglove/wifi-only` contains `wifi-only-v1`, it also stops the fixed
`zy_cellular.service`, and includes that service in its two-second quiet window.
Stop failures, unknown service state, and respawning reject START. Its normal
JSON receipt includes cellular before/after state. Without this marker, cellular
is outside the cleanup allowlist.

The optional polkit rule allows user `agi` only the `stop` operation on
`zy_cellular.service`; it works with the supervisor's `NoNewPrivileges=true`.
It grants no permission to start units, change routes, or control other services.

## Vendor handheld and status display

### Cleanup policy and protected services

HOPE deliberately stops the HDU camera application `hal_hdu_camera` and its
exact matched leftovers during the BACKGROUND step. These are application-side
shutdowns, not evidence of a vendor crash. The optional Wi-Fi-only profile
additionally stops/disables cellular; do not use it if controller or safety
connectivity depends on cellular.

Earlier HOPE cleanup also stopped `agent`, `hal_audio`, their watchdogs and
`agibot_ui.service`. The inspected robot additionally had a HOPE condition
blocking UI startup and a disabled remote boot entry; the origin of the remote
entry edit was not established. Live logs confirmed that
an unregistered `agent` causes `Startup` → `Ready` rollback. The updated
cleanup therefore preserves the following dependencies and does not require
them to be stopped for START:

| Dependency | Preservation requirement |
| --- | --- |
| HDU `agent`, `hal_audio` and their watchdogs | Preserve vendor app registrations, startup processing and audio prompts; do not kill them as unrelated load. |
| HDU `hal_remote` and `agibot_ui.service` | Preserve the configured remote bridge and status/network/IP display, including vendor boot settings. A running bridge alone does not prove handheld E-stop operation. |
| Vendor `gateway`, `sm`, MQTT, drive HAL, ELink and EtherCAT | Do not stop or reconfigure them as background/performance cleanup. They participate in the control and emergency chain. |
| Process manager, RouDi/IPC, controller networking/radio, routing and time services | Preserve the required dependencies. Do not stop a parent service to remove an unrelated child, or change a required link/clock during play. |

HOPE installation and normal cleanup must not rewrite vendor launch defaults
or the factory automatic-mode policy. On the tested A3, holding the system in
`Ready` did not pass the handheld startup wizard; restoring the factory
`Auto_transToManual: true` and rebooting did. Entering `Manual` activates native
`mc` and may enable joint control; it is not just a display or network repair.

### Upgrading an existing robot

Updating the cleanup helper does not undo an already disabled UI unit,
boot-blocking drop-in or edited vendor launch list. Repair those settings only
if inspection confirms they were changed:

1. Keep HOPE stopped and not busy. For vendor startup changes, mechanically
   support the robot with an on-site operator and an available wired E-stop.
   Back up the exact installed files before editing.
2. Install the updated `hope_catch_background.py` beside the existing
   `/usr/local/libexec/hope-lifecycle`. Preserve robot-specific lifecycle,
   supervisor, hardware and network settings; do not replace the whole vendor
   configuration or run a generic start-all operation.
3. Restore missing `agent`/`hal_audio` registrations from the matching vendor
   profile, the required audio/remote boot entries, and the vendor UI's enabled
   state. Remove only the identified HOPE UI-blocking condition, retaining its
   backup. Keep camera cleanup separate from these startup dependencies.
4. Preserve the matched vendor state-machine policy, including automatic
   `Manual` on the tested A3. Changes take effect only when loaded; inspect the
   running configuration as well as the saved file. Use an approved,
   operator-controlled reboot rather than killing safety services or forcing
   a state transition to pass a wizard.
5. Check the expected vendor state and connectability response, then verify
   the real handheld connection, status/IP display and startup prompts.
   Validate physical E-stop operation under the supported/stopped vendor
   procedure before motion. Do not fabricate connectability or substitute a
   played prompt for a completed initialization.

Keep robot-local rollback backups; they are not repository deployment assets.
The installers do not automatically apply the robot-specific vendor repairs
above.

### Restoring supervisor hardening

The former UI-stop command required `sudo` and led to the field-only override
`/etc/systemd/system/hope-lifecycle-supervisor.service.d/70-upstream-startup-sudo.conf`
setting `NoNewPrivileges=false`. The updated cleanup has no UI-stop command;
that workaround is no longer needed for background cleanup. The optional
cellular stop uses its narrow polkit rule, not sudo, and its installer tests
the stop under `setpriv --no-new-privs`.

After installing the updated helper, inspect the effective unit and drop-ins.
If this override exists only for the obsolete UI stop, back it up and remove
**only that file**, retaining unrelated robot-specific overrides. Confirm that
the remaining unit configuration restores `NoNewPrivileges=true`. Do not use
a broad `systemctl revert` or replace a consolidated supervisor with a sample
unit. Another locally added elevation requirement needs its own audit, not
blanket removal of hardening.

Run `systemctl daemon-reload`, then apply the change at an approved reboot or
supervisor restart while HOPE is stopped, not busy and has no hardware operation
in progress. A daemon reload alone does not harden an already running process.
After it is applied, inspect the effective setting and running PID:

```bash
systemctl show hope-lifecycle-supervisor.service -p NoNewPrivileges -p MainPID
```

Verify hardening is enabled and the running MainPID's `/proc/<PID>/status` has
`NoNewPrivs: 1`. Recheck controller/safety
availability before motion. This repository change does not automatically
remove an installed field override or restart the robot's supervisor.

### Validation and remaining limits

On 2026-10-27, the repaired robot loaded the restored automatic-Manual policy
and reached `Manual / IN_READY`; vendor remote/audio/agent/UI and MDU HAL/native
`mc` services were running while HOPE remained stopped. The operator confirmed
that the handheld fix worked after reboot. No physical E-stop assertion/reset
or HOPE motion test was performed by this verification.

This change is limited to background cleanup and the documented startup repair.
The existing managed MDU HAL handover still stops its vendor process manager
before session HAL/ELink takes ownership, and attended clock calibration
temporarily stops/restores vendor consumers. Neither mechanism is changed
here, and uninterrupted physical remote E-stop through them is **not**
established. Qualify that safety path separately; do not launch competing HAL
owners to avoid the handover.

### Required live validation before deployment

**High-priority qualification remains open:** reboot recovery with HOPE stopped
does not validate a full HOPE START → STOP cycle. Until the handheld path is
qualified under HOPE drive ownership, treat an on-site, independently verified
**wired E-stop as the only credited E-stop in HOPE sessions**. Do not credit
the handheld as an available primary or backup stop based on connection icons,
process presence, RPC advertisements or Foxglove readiness alone.

There are two distinct ownership hazards:

- During START, `start_hal()` stops the MDU vendor process manager before HOPE
  takes over the drive HAL/ELink. Handheld E-stop operation during HOPE play
  and the ownership transitions has not been tested.
- During STOP, `stop_mdu()` stops Runner and session HAL/ELink, then restarts
  the vendor process manager. With `agent` available and the restored factory
  `Auto_transToManual: true`, the vendor can automatically enter `Manual` and
  activate its own `mc` after STOP. Clock calibration's vendor-service restore
  has the same potential. A HOPE `STOPPED` indication does not prove that
  vendor joint control is disabled.

Required attended test, not an automated startup action:

1. Mechanically support the robot, keep controls neutral, clear its motion
   envelope and station an operator at a verified wired E-stop. Use a reviewed
   vendor-supported procedure for physical stop testing; do not start a motion
   test if the wired stop is unverified for the tested configuration.
2. Record the installed revision/configuration, loaded automatic-Manual policy,
   service/control owners, joint-enable/control state and handheld status before
   START. Run a complete supported START → STOP cycle, not only a preflight or
   reboot. Observe each drive-ownership transition.
3. During the agreed controlled HOPE play phase, test the handheld E-stop under
   the approved supported procedure. Record the actual drive/actuator response
   and stop timing, not only a successful command acknowledgement. If it fails
   or cannot be tested safely, leave it explicitly unqualified and continue to
   credit only the verified wired stop.
4. At STOP, verify Runner and session HAL/ELink have exited before vendor drive
   ownership resumes, with no competing owners. Observe whether vendor `mc`
   becomes active and whether `Manual`, joint enable, torque or motion appears
   without a new operator command. Record the resulting state, recheck the
   handheld connection and verify wired-stop availability after the restore.
   Unexpected actuator activation or competing owners is a failed test, not
   evidence of successful cleanup.
5. Separately run attended, pre-operation clock calibration and observe its
   complete vendor-service restore using the same support/wired-stop precautions.
   Check for automatic `Manual`/vendor `mc` activation and unintended actuator
   output, ensure HOPE does not restart play, and recheck controller and stop
   availability. A qualified clock alone does not qualify this restore.

Retain timestamped lifecycle/vendor/Runner evidence, actual ownership and
actuator/stop observations, pass/fail results for **both** restore paths, and
operator sign-off. Tests not performed must remain marked pending. This PR
documents these deployment requirements; it does not change the START/STOP or
clock-restore implementations, add a runtime gate, or claim the live tests pass.

Cloud LLM conversation is a separate, unfinished isolation task. The last
read-only agent query returned `normal` and `enable_voice=true`; this repair
does not claim cloud dialogue is disabled. Use vendor feature controls only
after verifying independence from startup prompts and handheld connectivity.
Microphone muting alone does not prove all cloud connections are stopped, and
stopping the whole `agent`/audio stack is not an acceptable shortcut.

## 2. One-time per-robot setup and recovery

Run the installer once on each robot during initial setup. The installed systemd
service and NetworkManager settings persist across reboot; normal boot and each
Foxglove START do not rerun the installer or rewrite saved connection profiles.
START only checks the Wi-Fi policy and performs the configured background cleanup.
Reinstall only when upgrading this policy or restoring a reimaged robot.

Copy the `foxglove/helpers` and `foxglove/a3` directories together, preserving
layout. On the supported HDU, from the checkout, choose **one** command:

```bash
# Wi-Fi policy only; preserve cellular use:
sudo bash foxglove/a3/install_hdu_wifi_policy.sh

# Explicit local-Wi-Fi-only operation; additionally stop/disable cellular:
sudo bash foxglove/a3/install_hdu_wifi_policy.sh --wifi-only
```

The installer backs up affected files, service states, live radio settings and
saved connection power settings under a root-only `/var/backups/hope-wifi-policy-*`
directory. It preserves the installed lifecycle file and inserts only its Wi-Fi
check. An unfamiliar preflight layout is rejected before installation.
It updates every saved Wi-Fi profile to `802-11-wireless.powersave=2`, supplies a
NetworkManager default for new `wifi_hdu` profiles, and enables the policy service.
The Wi-Fi-only option also deploys the matching background guard and narrow
polkit rule; its stop is tested as `agi` with no-new-privileges before cellular
is disabled. Re-running without the option does not undo an existing Wi-Fi-only
profile. Installation errors after mutation invoke the generated restore script.

The daemon checks association/power state once per second. Connected: suppress
all scans using the vendor driver flag. Disconnected: release the flag so initial
association/reconnection can scan normally. Stopping the daemon also releases it.
Power saving is corrected after network changes. It logs state changes, not every
poll. The vendor setting application can still request rescans; the driver rejects
them while connected. Automatic roaming scans are therefore suppressed too;
this profile is intended for a robot using one local AP.

For an explicitly requested scan (expect a temporary latency spike):

```bash
sudo python3 /usr/local/libexec/hope_wifi_policy.py rescan
```

This opens a root-controlled window of at most 30 seconds, waits for a completed
`iw` scan, prints cached results without requesting another scan, and closes the
window even on normal command failure. The daemon expires abandoned windows.
Use this command instead of raw `nmcli device wifi rescan` while connected.

Read back after installation and after the operator reboots:

```bash
python3 /usr/local/libexec/hope_wifi_policy.py check
systemctl is-enabled hope-wifi-policy.service
journalctl -b -u hope-wifi-policy.service --no-pager
# For Wi-Fi-only: expect disabled, inactive; these commands return nonzero normally.
systemctl is-enabled zy_cellular.service
systemctl is-active zy_cellular.service
ip route
ping -n -I wifi_hdu -c 120 -i 0.5 -W 2 <local-router-address>
```

Also verify fresh `/hope/lifecycle/summary` and monitor topics from the laptop.
Stopping the modem removes its address/default route; the monitor may restart
DDS once on the interface change. Verify recovery before playing. The installer
leaves Wi-Fi and internal Ethernet routes alone. A disabled cellular service
removes mobile Internet/failover, not the directly connected local Wi-Fi subnet.

Rollback uses the exact backup path printed by the installer:

```bash
sudo bash /var/backups/hope-wifi-policy-XXXXXXXX/restore.sh
```

Rollback restores the pre-install snapshot, including cellular enabled/running
state, and may redial cellular. Run while stopped; later edits to these files or
saved profile power settings are overwritten. It does not delete robot logs or
recordings. An upgrade rollback returns to the previous policy, not necessarily
to vendor defaults. Do not deploy the new lifecycle helper alone: install its
Wi-Fi policy dependency on the supported HDU at the same time.

## Periodic I/O and logging: retain until an effect is measured

Both boards run `agibot_log_sync.timer` roughly every minute. Its plain `sync`
requests writeback of pending filesystem data/metadata globally, rather than
only the robot log. Linux already performs background dirty-page writeback:
these boards report `dirty_writeback_centisecs=500` and
`dirty_expire_centisecs=3000`. Removing the extra timer would not make a half-day
of logs accumulate indefinitely in RAM, but would change explicit flush behavior.
Neither periodic sync nor those kernel intervals guarantees every application
record survives sudden power loss. See the [kernel writeback settings](https://kernel.org/doc/html/latest/admin-guide/sysctl/vm.html)
and [GNU sync documentation](https://www.gnu.org/software/coreutils/manual/html_node/sync-invocation.html).

The 2026-10-18 audit did not establish a playback/control latency benefit from
removing this sync, so this change preserves it and kernel writeback settings.
It also preserves recordbag and application logs:

- HDU `agibot_top.service`: 1-second atop sampling, approximately 1-minute files,
  retains 120 files; already uses nice 19 / idle I/O priority.
- MDU `agibot_irq.service`: also configures EtherCAT IRQ affinity/priority. Its
  logging loop records atop into tmpfs, moves a file to disk about every minute,
  and retains 600 files. **Do not stop this service just to stop its logger.**
- Recordbag growth needs separate disk-capacity monitoring; rolling atop files
  do not imply that recordings have bounded retention. Existing data is retained.
- Cellular probes ran about every 7 seconds. Their pings were bound to the modem;
  failed pings alone did not cause resets. Missing IP/route/process can trigger
  redial/reset and route changes. The Wi-Fi-only profile removes this activity.

## Field evidence and limits (2026-10-18)

The driver already reported IPS/LPS disabled; toggling advertised power saving
alone did not remove the spikes. With power saving still on, suppressing scans
changed a 60-packet robot-to-router sample from 70.462 ms average / 896.212 ms max
(scans restored) to 1.914 / 5.972 ms (scans blocked). This isolates connected
scanning as the demonstrated large latency source; power saving remains disabled
as the intended low-latency policy.

After an actual operator reboot, the journal showed scans allowed while offline
and blocked after association. Three simultaneous 120-packet probes had no loss:
robot-router average/max 1.929/18.831 ms; laptop-robot 2.303/12.642 ms;
laptop-router 0.536/6.027 ms. This proves boot association and policy persistence,
not a physical link-loss/reconnect endurance test.

A subsequent 65-second disk sample, including normal sync/rotation, measured:

| Board | Mean physical writes | Peak 1-second disk utilization | Largest per-sample write await |
| --- | ---: | ---: | ---: |
| HDU | 101 KiB/s | 3.0% | 22.4 ms |
| MDU | 479 KiB/s | 5.3% | 5.99 ms |

These are device statistics, not robot control deadline measurements. There was
no disk saturation in this stopped-state sample. MDU's largest second wrote
3.5 MiB, adjacent to its scheduled sync. Storage free at the audit was 23 GiB HDU
and 47 GiB MDU. A separate 100-second MDU recording sample grew about 27 MiB;
linear extrapolation is about 11.5 GiB over 12 hours at that sampled rate. Active
play can change recording rates; this is not a half-day endurance qualification.

After the Wi-Fi-only install (including reinstall/rollback verification during
the probe), another 120 packets per path had no loss: robot-router average/max
1.516/10.225 ms and laptop-robot 2.245/31.636 ms. This does not establish a
separate latency benefit from cellular removal.

Focused validation (44 tests passed):

```bash
python3 -m pytest -q foxglove/tests/test_wifi_policy.py \
  foxglove/tests/test_catch_background.py foxglove/tests/test_lifecycle_core.py
bash -n foxglove/helpers/hope-lifecycle
bash -n foxglove/a3/install_hdu_wifi_policy.sh
bash -n foxglove/a3/restore_hdu_wifi_policy.sh
```

Tests cover disconnect/reconnect transitions, manual expiry/failure, disabled
policy and power drift admission failures, cellular opt-in, stop denial,
respawn rejection, and preservation of recording. The live HDU additionally
validated install/reinstall, restoration to the immediately previous policy,
cellular stop under no-new-privileges, and unchanged local routes. No robot START
or motion is used as an installation test. Cellular disablement still needs its
own subsequent reboot readback; the earlier reboot tested Wi-Fi policy.
