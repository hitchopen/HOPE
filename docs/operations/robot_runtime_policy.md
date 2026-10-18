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

The existing background guard still stops its explicit voice/camera/UI allowlist.
If `/etc/hope-foxglove/wifi-only` contains `wifi-only-v1`, it also stops the fixed
`zy_cellular.service`, and includes that service in its two-second quiet window.
Stop failures, unknown service state, and respawning reject START. Its normal
JSON receipt includes cellular before/after state. Without this marker, cellular
is outside the cleanup allowlist.

The optional polkit rule allows user `agi` only the `stop` operation on
`zy_cellular.service`; it works with the supervisor's `NoNewPrivileges=true`.
It grants no permission to start units, change routes, or control other services.
Existing UI-stop permissions remain a separate prerequisite.

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
