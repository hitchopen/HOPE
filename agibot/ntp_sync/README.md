# Agibot A3 Chrony Installation and Operations Runbook

This directory is the canonical deployable companion to the English **A3
Platform Time Synchronization Improvement Plan v1.5**. Run the commands in this
document on
the A3 HDU terminal. Do not reconstruct configuration files from the PDF; the
reviewed files are already included in this directory.

The change replaces the stock `ntp2rtc.py` system-clock path with continuous
`chrony` discipline while preserving the audited internal distribution path:

```text
approved regional NTP sources -> chrony -> HDU CLOCK_REALTIME
  -> phc2sys -> eth_hdu PHC -> ptp4l -> A3 boards / MDU
  -> existing sensor synchronization and timestamp conversion
```

Motive and NatNet remain an external observation path. They do not set the A3
system clock, PHC, PTP domain, RTC, or chrony state.

## Safety rules

This is a controlled maintenance procedure, not a one-command installer.

- Support the robot mechanically, keep its feet off the floor, make the E-stop
  available, and prevent all motion commands.
- Stop recording and external command sources before changing time or services.
- Use a stable terminal session. A local console or wired maintenance link is
  preferred for the first activation.
- Do not permit motion until both UTC qualification and the existing internal
  board/sensor synchronization gates pass.
- Do not continue from a failed command. Capture the output and investigate.
- Phase 0 and Phase 1A do not authorize motion or production use.
- The example `10 ms` remaining-correction and `5 ppm` skew gates are
  provisional. Robot controls, board/firmware, sensor integration, platform,
  and event operations owners must approve final limits before Phase 1B.

## Package layout

```text
ntp_sync/
|-- README.md
|-- MANIFEST.sha256
|-- etc/
|   |-- agibot-time/regions/{china,us,europe}/chrony.sources
|   |-- chrony/{chrony.conf,chrony-bootstrap.conf}
|   |-- default/{agibot-clock-bootstrap,agibot-timesync}
|   `-- systemd/system/
|       |-- agibot-clock-bootstrap.service
|       |-- agibot_pm.service.d/20-clock-ordering.conf
|       `-- chrony.service.d/agibot-controls.conf
|-- opt/agibot/entry/system/timesync.sh
`-- usr/local/sbin/{agibot-clock-bootstrap,agibot-time-region}
```

## 0. Open the package on A3

Copy this entire directory to A3, then enter it. If the HOPE repository is
already on A3:

```bash
cd ~/HOPE/agibot/ntp_sync
export PKG_ROOT="$PWD"
test -f "$PKG_ROOT/MANIFEST.sha256"
```

If it is stored elsewhere, replace the `cd` path. Keep this terminal open for
the whole procedure; later commands use `PKG_ROOT`.

Verify package integrity before using `sudo`:

```bash
cd "$PKG_ROOT"
sha256sum -c MANIFEST.sha256
```

Every line must report `OK`. A mismatch is a hard stop.

## 1. Phase 0: read-only preflight

Confirm that this is a supported A3 platform and inspect the active time stack.
Do not assume that any dependency is installed or absent:

```bash
uname -m
cat /etc/os-release
id agi
ip link show eth_hdu
timedatectl status
systemctl status agibot_pm.service --no-pager || true
systemctl status agibot_timesync.service --no-pager || true
pgrep -a -f 'ntp2rtc|chronyd|ptp4l|phc2sys' || true
systemctl cat agibot_pm.service
grep -nE 'ntp2rtc|chrony|ptp4l|phc2sys|timedatectl' \
  /opt/agibot/entry/system/timesync.sh
dpkg-query -W -f='${Package}\t${Version}\t${db:Status-Abbrev}\n' \
  chrony ethtool linuxptp iproute2 procps systemd 2>/dev/null || true
command -v chronyd chronyc ethtool ptp4l phc2sys hwclock || true
```

Required platform behavior before migration:

- Debian 12 on `aarch64`.
- User and group `agi` exist.
- `eth_hdu` exists.
- `/opt/agibot/entry/system/timesync.sh` is the production owner path.
- The stock script launches `ntp2rtc.py`, `ptp4l`, and `phc2sys`.
- Active PTP arguments use `ptp4l -i eth_hdu -2 -E -m`.
- Active PHC direction is `CLOCK_REALTIME` to `eth_hdu`.
- `agibot_timesync.service` is disabled/inactive and must remain so.
- `chrony` may be installed or absent. If present, record its package version,
  configuration hash, enabled state, and active state.
- No `chronyd` process may be actively disciplining `CLOCK_REALTIME` at the same
  time as the stock `ntp2rtc.py` path.

Stop if the platform, path, interface, process ownership, PTP mode, PHC
direction, or clock-owner behavior differs. Installed-package presence alone is
not a reason to stop; an unexplained active clock owner is.

Check for files left by an earlier attempt:

```bash
for PATHNAME in \
  /usr/local/sbin/agibot-clock-bootstrap \
  /usr/local/sbin/agibot-time-region \
  /etc/systemd/system/agibot-clock-bootstrap.service \
  /etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf \
  /etc/systemd/system/chrony.service.d/agibot-controls.conf \
  /etc/agibot-time/current \
  /etc/chrony/sources.d/hope-approved.sources; do
  sudo test -e "$PATHNAME" -o -L "$PATHNAME" && \
    echo "PRE-EXISTING: $PATHNAME"
done
```

Any output requires review against the previous change record or rollback
backup before continuing.

### Mandatory board and sensor inventory

Before installation, record the native clock, timestamp point, synchronization
path, owner limit, and failure action for every HDU/MDU board, camera, IMU,
force/contact sensor, and joint/motor feedback stream. Capture at least 30
minutes of baseline data under representative CPU, network, camera, recorder,
and control load.

For each signal, retain median, p95, p99, maximum offset, jitter, dropouts, role
changes, resets, and timestamp discontinuities. This inventory cannot be
inferred by the generic installation package and is a blocking Phase 1B gate.

## 2. Back up the stock system

Run this while the robot is physically safe. Save the printed `BACKUP` path in
the change record.

```bash
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
BACKUP="/var/backups/agibot-time/$STAMP"
echo "BACKUP=$BACKUP"

sudo install -d -o root -g root -m 0700 "$BACKUP/rootfs"
sudo cp -a --parents /opt/agibot/entry/system/timesync.sh "$BACKUP/rootfs"
sudo sha256sum /opt/agibot/entry/system/timesync.sh | \
  sudo tee "$BACKUP/vendor-timesync.sha256"
sudo stat -Lc '%n %U:%G %a %s %y' \
  /opt/agibot/entry/system/timesync.sh | \
  sudo tee "$BACKUP/vendor-timesync.stat"

if sudo test -e /etc/chrony/chrony.conf; then
  sudo cp -a --parents /etc/chrony/chrony.conf "$BACKUP/rootfs"
  sudo sha256sum /etc/chrony/chrony.conf | \
    sudo tee "$BACKUP/chrony-conf.sha256"
  sudo touch "$BACKUP/chrony-conf-present"
else
  sudo touch "$BACKUP/chrony-conf-absent"
fi

dpkg-query -W -f='${Package}\t${Version}\t${db:Status-Abbrev}\n' \
  bash chrony coreutils ethtool findutils grep iproute2 linuxptp mawk \
  procps sed systemd util-linux-extra bsdutils 2>/dev/null | \
  sudo tee "$BACKUP/dependency-packages.txt" || true
apt-cache policy chrony ethtool linuxptp | \
  sudo tee "$BACKUP/dependency-policy.txt" || true
systemctl is-enabled chrony.service 2>&1 | \
  sudo tee "$BACKUP/chrony-enabled.txt" || true
systemctl is-active chrony.service 2>&1 | \
  sudo tee "$BACKUP/chrony-active.txt" || true
sudo systemctl cat agibot_pm.service | \
  sudo tee "$BACKUP/agibot_pm.service.txt"
sudo systemctl list-dependencies agibot_pm.service | \
  sudo tee "$BACKUP/agibot_pm.dependencies.txt"
sudo pgrep -a -f 'ntp2rtc|chronyd|ptp4l|phc2sys' | \
  sudo tee "$BACKUP/time-processes.txt" || true
sudo timedatectl status | sudo tee "$BACKUP/timedatectl.txt"

if sudo test -d /agibot/log/tsync; then
  sudo cp -a /agibot/log/tsync "$BACKUP/tsync-logs"
fi

sudo ln -sfn "$BACKUP" /var/backups/agibot-time/latest
sudo readlink -f /var/backups/agibot-time/latest
```

Confirm that the vendor file, its metadata, and the prior chrony configuration
state were captured before continuing:

```bash
sudo test -s "$BACKUP/vendor-timesync.sha256"
sudo test -s "$BACKUP/vendor-timesync.stat"
sudo test -e "$BACKUP/chrony-conf-present" -o \
  -e "$BACKUP/chrony-conf-absent"
sudo ls -la "$BACKUP"
```

## 3. Phase 1A: install while disabled

Phase 1A changes files but does not start the new clock owner or the robot.
After the `agibot_pm` drop-in is installed, an unexpected reboot will block
`agibot_pm` until chrony qualifies. Keep the robot offline and complete or roll
back the maintenance in the same window.

### 3.1 Stop robot and time-distribution processes

```bash
sudo systemctl stop agibot_pm agibot_roudi agibot_top agibot_ui || true
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true
sudo pkill -TERM -f '/ntp2rtc.py' || true
sleep 3
pgrep -a -f 'ntp2rtc|chronyd|ptp4l|phc2sys' || true
```

The final command must show no active `ntp2rtc.py`, `chronyd`, `ptp4l`, or
`phc2sys` process. If a process remains, investigate before continuing.

### 3.2 Verify and install only missing dependencies

Do not run `apt-get update` or reinstall chrony merely because it is disabled.
First mask an existing chrony unit, detect required commands, and map only
missing commands to Debian packages:

```bash
command -v sudo apt-get dpkg dpkg-query
sudo systemctl mask --now chrony.service 2>/dev/null || true

declare -A COMMAND_PACKAGE=(
  [bash]=bash
  [chronyd]=chrony
  [chronyc]=chrony
  [ethtool]=ethtool
  [ptp4l]=linuxptp
  [phc2sys]=linuxptp
  [ip]=iproute2
  [pgrep]=procps
  [pkill]=procps
  [systemctl]=systemd
  [systemd-analyze]=systemd
  [hwclock]=util-linux-extra
  [logger]=bsdutils
  [timeout]=coreutils
  [install]=coreutils
  [stat]=coreutils
  [readlink]=coreutils
  [sha256sum]=coreutils
  [find]=findutils
  [xargs]=findutils
  [grep]=grep
  [sed]=sed
  [awk]=mawk
)

: > /tmp/agibot-time-missing-packages
for COMMAND in "${!COMMAND_PACKAGE[@]}"; do
  if ! command -v "$COMMAND" >/dev/null 2>&1; then
    printf '%s\n' "${COMMAND_PACKAGE[$COMMAND]}" \
      >> /tmp/agibot-time-missing-packages
    printf 'MISSING command=%s package=%s\n' \
      "$COMMAND" "${COMMAND_PACKAGE[$COMMAND]}"
  fi
done
sort -u -o /tmp/agibot-time-missing-packages \
  /tmp/agibot-time-missing-packages
printf '%s\n' 'Packages requiring installation:'
cat /tmp/agibot-time-missing-packages
```

If the file is empty, skip all package installation commands. If it is not
empty, refresh package metadata and inspect the exact proposed transaction:

```bash
if [[ -s /tmp/agibot-time-missing-packages ]]; then
  mapfile -t MISSING_PACKAGES < /tmp/agibot-time-missing-packages
  sudo apt-get update
  apt-cache policy "${MISSING_PACKAGES[@]}"
  sudo apt-get --simulate install --no-install-recommends --no-upgrade \
    "${MISSING_PACKAGES[@]}"
fi
```

Record candidate versions and review the simulation. Stop if it upgrades,
removes, or replaces an existing platform package outside the approved change.
After approval, install only the recorded missing list:

```bash
if [[ -s /tmp/agibot-time-missing-packages ]]; then
  mapfile -t MISSING_PACKAGES < /tmp/agibot-time-missing-packages
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    --no-install-recommends --no-upgrade "${MISSING_PACKAGES[@]}"
fi
sudo systemctl mask --now chrony.service
```

Verify every required command and record package versions after installation:

```bash
FAILED=0
for COMMAND in "${!COMMAND_PACKAGE[@]}"; do
  command -v "$COMMAND" >/dev/null 2>&1 || {
    echo "STILL MISSING: $COMMAND"
    FAILED=1
  }
done
test "$FAILED" -eq 0

dpkg-query -W -f='${Package}\t${Version}\t${db:Status-Abbrev}\n' \
  bash chrony coreutils ethtool findutils grep iproute2 linuxptp mawk \
  procps sed systemd util-linux-extra bsdutils 2>/dev/null || true
for COMMAND in "${!COMMAND_PACKAGE[@]}"; do
  COMMAND_PATH=$(command -v "$COMMAND")
  printf '%-18s %s\n' "$COMMAND" "$COMMAND_PATH"
  dpkg -S "$COMMAND_PATH" 2>/dev/null || \
    echo "WARNING: dpkg does not own $COMMAND_PATH"
done
chronyd --version
ptp4l -v
phc2sys -v
ethtool --version
systemctl is-enabled chrony.service || true
systemctl is-active chrony.service || true
```

Expected service state is `masked` and inactive. Package versions are facts to
record, not assumptions. A version outside the approved A3 compatibility matrix
requires configuration parsing and the complete bench acceptance suite before
production use.

### 3.3 Install reviewed files

Run from the same shell where `PKG_ROOT` is set:

```bash
sudo install -D -o root -g root -m 0644 \
  "$PKG_ROOT/etc/chrony/chrony.conf" \
  /etc/chrony/chrony.conf
sudo install -D -o root -g root -m 0644 \
  "$PKG_ROOT/etc/chrony/chrony-bootstrap.conf" \
  /etc/chrony/chrony-bootstrap.conf
sudo install -D -o root -g root -m 0644 \
  "$PKG_ROOT/etc/default/agibot-clock-bootstrap" \
  /etc/default/agibot-clock-bootstrap
sudo install -D -o root -g root -m 0644 \
  "$PKG_ROOT/etc/default/agibot-timesync" \
  /etc/default/agibot-timesync

for REGION in china us europe; do
  sudo install -D -o root -g root -m 0644 \
    "$PKG_ROOT/etc/agibot-time/regions/$REGION/chrony.sources" \
    "/etc/agibot-time/regions/$REGION/chrony.sources"
done

sudo install -D -o root -g root -m 0755 \
  "$PKG_ROOT/usr/local/sbin/agibot-clock-bootstrap" \
  /usr/local/sbin/agibot-clock-bootstrap
sudo install -D -o root -g root -m 0755 \
  "$PKG_ROOT/usr/local/sbin/agibot-time-region" \
  /usr/local/sbin/agibot-time-region
sudo install -D -o root -g root -m 0644 \
  "$PKG_ROOT/etc/systemd/system/agibot-clock-bootstrap.service" \
  /etc/systemd/system/agibot-clock-bootstrap.service
sudo install -D -o root -g root -m 0644 \
  "$PKG_ROOT/etc/systemd/system/chrony.service.d/agibot-controls.conf" \
  /etc/systemd/system/chrony.service.d/agibot-controls.conf
sudo install -D -o root -g root -m 0644 \
  "$PKG_ROOT/etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf" \
  /etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf

VENDOR_UID=$(sudo stat -Lc %u /opt/agibot/entry/system/timesync.sh)
VENDOR_GID=$(sudo stat -Lc %g /opt/agibot/entry/system/timesync.sh)
VENDOR_MODE=$(sudo stat -Lc %a /opt/agibot/entry/system/timesync.sh)
sudo install -D -o "$VENDOR_UID" -g "$VENDOR_GID" -m "$VENDOR_MODE" \
  "$PKG_ROOT/opt/agibot/entry/system/timesync.sh" \
  /opt/agibot/entry/system/timesync.sh
```

The replacement preserves the current vendor file's numeric owner, group, and
mode. It does not assume those values are identical across the A3 fleet. All
new package-owned configuration, helper, and unit files are root-owned.

### 3.4 Select the regional NTP profile

China is the default:

```bash
sudo /usr/local/sbin/agibot-time-region china
sudo /usr/local/sbin/agibot-time-region --show
readlink -f /etc/chrony/sources.d/hope-approved.sources
```

Expected target:

```text
/etc/agibot-time/regions/china/chrony.sources
```

The three available profiles are:

| Region | Approved endpoints |
| --- | --- |
| `china` | `ntp1.aliyun.com`, `ntp2.aliyun.com`, `ntp3.aliyun.com` |
| `us` | `time-a-g.nist.gov`, `time-b-g.nist.gov`, `time-c-g.nist.gov` |
| `europe` | `ptbtime1.ptb.de`, `ptbtime2.ptb.de`, `ptbtime3.ptb.de` |

Public NTP is not cryptographically authenticated; approval does not remove
that network trust boundary. Profiles intentionally do not use chrony's
`trust` option and must not be mixed.

Leap handling is an explicit package deviation from the design-plan proposal:

- The China/Aliyun profile is treated as provider-managed leap-smear time.
- The US/NIST and Europe/PTB profiles are treated as standard UTC sources.
- `leapsectz right/UTC` is omitted from the shared continuous and bootstrap
  configs because one global directive cannot safely represent both policies,
  regional `.sources` files cannot carry it, and some A3 images do not install
  the `right/` timezone tree.
- Never mix leap-smearing and standard-UTC sources. Before a scheduled leap
  event, confirm the active provider policy and run a separate go/no-go review.

Exactly one `.sources` file may exist in `/etc/chrony/sources.d`. Check it:

```bash
sudo find /etc/chrony/sources.d -maxdepth 1 -name '*.sources' \
  -printf '%f -> %l\n'
test "$(sudo find /etc/chrony/sources.d -maxdepth 1 \
  -name '*.sources' | wc -l)" -eq 1
```

Any additional source is a hard stop. Do not leave DHCP-injected or venue NTP
sources active beside `hope-approved.sources`.

### 3.5 Validate the disabled installation

```bash
sudo bash -n /opt/agibot/entry/system/timesync.sh
sudo bash -n /usr/local/sbin/agibot-clock-bootstrap
sudo bash -n /usr/local/sbin/agibot-time-region
sudo chronyd -p -f /etc/chrony/chrony.conf >/dev/null
sudo chronyd -p -f /etc/chrony/chrony-bootstrap.conf >/dev/null

sudo systemctl daemon-reload
sudo systemd-analyze verify \
  agibot-clock-bootstrap.service chrony.service agibot_pm.service
sudo systemctl disable agibot-clock-bootstrap.service

systemctl cat agibot-clock-bootstrap.service
systemctl cat chrony.service
systemctl cat agibot_pm.service
systemctl is-enabled chrony.service || true
systemctl is-active chrony.service || true
```

Confirm all of the following before Phase 1B:

- `chrony.service` is masked and inactive.
- `agibot-clock-bootstrap.service` is disabled and inactive.
- `chrony.conf` contains `maxslewrate 100`, `corrtimeratio 3`, and no
  `makestep`, `/run/chrony-dhcp`, `allow`, or `local` directive.
- The bootstrap config has `makestep 0.001 1`; only the supervised bootstrap
  may step time.
- The chrony drop-in has `Restart=on-failure` and `RestartSec=5`.
- The `agibot_pm` drop-in orders after chrony and runs the synchronous
  `timesync.sh --preflight` gate.
- The replacement script contains no `ntp2rtc.py` launch and preserves
  `ptp4l -2 -E` plus `phc2sys -s CLOCK_REALTIME -c eth_hdu`.
- The readiness loop requires `eth_hdu` to be Link Up and verifies its PTP
  hardware clock plus hardware transmit, receive, and raw-clock capabilities
  with `ethtool -T` before starting LinuxPTP.

Useful assertions:

```bash
grep -E '^(maxslewrate|corrtimeratio|logchange|minsources)' \
  /etc/chrony/chrony.conf
! grep -Eq '^[[:space:]]*(makestep|allow|local|sourcedir /run/chrony-dhcp)' \
  /etc/chrony/chrony.conf
! grep -q 'ntp2rtc' /opt/agibot/entry/system/timesync.sh
grep -F 'ptp4l -i "$device_name" -2 -E -m' \
  /opt/agibot/entry/system/timesync.sh
grep -F 'phc2sys -s CLOCK_REALTIME -c "$device_name" -w -O 0 -S 10 -m' \
  /opt/agibot/entry/system/timesync.sh
ethtool -T eth_hdu
```

Record the Phase 1A results and obtain owner approval before activation.

## 4. Phase 1B: supervised first activation

Keep the robot supported and all robot applications stopped. This procedure may
step `CLOCK_REALTIME` once before the distributed time domain restarts.

### 4.1 Run the one-time bootstrap, then continuous chrony

```bash
sudo systemctl stop agibot_pm agibot_roudi agibot_top agibot_ui || true
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true
sleep 2

sudo systemctl unmask chrony.service
sudo systemctl enable agibot-clock-bootstrap.service chrony.service
sudo systemctl start agibot-clock-bootstrap.service
sudo test -e /run/agibot-time/bootstrap-qualified
sudo systemctl start chrony.service
```

`bootstrap-qualified` is created before the RTC write. If
`/run/agibot-time/rtc-updated` is absent, the runtime step succeeded but the RTC
write failed; repair the RTC separately before accepting cold-boot behavior.

Wait for continuous chrony to select a source. The values below are provisional
and must be replaced by approved venue limits:

```bash
chronyc waitsync 60 0.010 5 2
chronyc tracking
chronyc sources -v
chronyc sourcestats -v
sudo /opt/agibot/entry/system/timesync.sh --preflight
```

The preflight requires `Leap status: Normal` and a selected `^*` source. It is
necessary but not sufficient: operators must also review remaining correction,
skew, delay, dispersion, source identity, and the owner-approved thresholds.

### 4.2 Restart the internal A3 time path in a no-motion state

```bash
sudo systemctl start agibot_roudi agibot_top agibot_ui
sudo systemctl start agibot_pm
sleep 5

systemctl is-active agibot-clock-bootstrap chrony agibot_pm
pgrep -a -x chronyd
pgrep -a -x ptp4l
pgrep -a -x phc2sys
sudo journalctl -u agibot-clock-bootstrap -u chrony -u agibot_pm \
  --since '-10 minutes' --no-pager
```

On an online boot, the first `agibot_pm` preflight can occur after chrony starts
but before it selects a source. One initial failed attempt followed by a clean
service retry can be an acquisition race; repeated failures or a failure beyond
the approved source-acquisition window are faults.

Verify from the process output and logs that:

- There is one runtime owner of `CLOCK_REALTIME`: `chronyd`.
- `ptp4l` still uses `eth_hdu`, layer 2, E2E mode (`-2 -E`).
- `phc2sys` still copies `CLOCK_REALTIME` to the `eth_hdu` PHC.
- No `ntp2rtc.py`, `ntpd`, `ntpsec`, or second `agibot_timesync` owner is active.
- Expected boards retain their roles and lock.
- Every safety-critical sensor remains monotonic and within its signed limits.

Starting services here is only for supported, no-motion acceptance testing. It
does not release the robot for ping-pong.

### 4.3 Soak, fault tests, and cold boot

Capture at least 30 minutes under load and compare pre/post median, p95, p99,
maximum, jitter, dropout, role-change, and discontinuity results. Exercise:

- NTP unavailable at boot, DNS failure, and silent UDP/123 drop.
- NTP loss and return with both small and large offsets.
- `chronyd` failure and restart within 5 seconds.
- RTC write failure; runtime marker remains and RTC marker is absent.
- Motive/NatNet disconnect and restart; no A3 clock process changes.
- PTP client disconnect, sensor reset, and timestamp discontinuity detection.
- Supervised re-step with all robot applications stopped.

Only after bench acceptance, test a cold boot in the same physically safe state:

```bash
sudo reboot
```

After reconnecting:

```bash
systemctl is-active agibot-clock-bootstrap chrony agibot_pm
sudo test -e /run/agibot-time/bootstrap-qualified
chronyc tracking
chronyc sources -v
sudo /opt/agibot/entry/system/timesync.sh --preflight
pgrep -a -f 'ntp2rtc|chronyd|ptp4l|phc2sys'
```

Measure five online boots, five immediate DNS failures, and five silent firewall
drops. Set `BOOTSTRAP_TIMEOUT_S` in
`/etc/default/agibot-clock-bootstrap` from venue measurements; the supported
range is 5 to 150 seconds and the unit ceiling is 180 seconds.

## 5. Motion release gate

Motion remains prohibited until the responsible owners sign all of these:

- Approved regional source identity and leap policy.
- Chrony is selected, stable, within correction/skew/delay/dispersion limits,
  and running with a 100 ppm maximum slew rate.
- No runtime clock step or rate-cap violation occurred.
- PHC distribution is no worse than baseline.
- Exactly one expected PTP master exists and every required client is locked.
- Every board and safety-critical sensor passes its own offset, jitter, age,
  monotonicity, dropout, reset, and failure-action limits.
- Applications show no backward/duplicate timestamps, timeout storms, data
  corruption, or control instability.
- Motive/NatNet changes create no A3 clock, PHC, PTP, or sensor-sync event.

UTC qualification and internal synchronization are separate conclusions. One
cannot substitute for the other.

## 6. Holdover operation

Holdover is a deliberate UTC-unqualified operating mode for a venue where
approved NTP is unavailable but the A3 internal board/sensor time domain remains
healthy. It is not automatic fallback and it does not claim UTC accuracy.

Before enabling holdover, require two-person sign-off, record the start time and
maximum duration, confirm the approved drift budget, support the robot, stop
motion and recording, and verify the internal synchronization gates.

```bash
sudo systemctl stop agibot_pm agibot_roudi agibot_top agibot_ui || true
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true

sudo sed -i -E \
  's/^AGIBOT_ALLOW_UNQUALIFIED_TIME=.*/AGIBOT_ALLOW_UNQUALIFIED_TIME=1/' \
  /etc/default/agibot-timesync
sudo chown root:root /etc/default/agibot-timesync
sudo chmod 0644 /etc/default/agibot-timesync
grep '^AGIBOT_ALLOW_UNQUALIFIED_TIME=1$' /etc/default/agibot-timesync

sudo install -d -o root -g root -m 0755 /var/lib/agibot-time
printf 'UTC_UNQUALIFIED holdover enabled %s\n' "$(date -u +%FT%TZ)" | \
  sudo tee /var/lib/agibot-time/utc-unqualified-holdover >/dev/null
sudo chmod 0644 /var/lib/agibot-time/utc-unqualified-holdover

sudo systemctl start chrony.service
chronyc tracking
sudo /opt/agibot/entry/system/timesync.sh --preflight
sudo systemctl start agibot_roudi agibot_top agibot_ui
sudo systemctl start agibot_pm
```

The override still requires an active, responsive chrony daemon and an approved
regional profile; it relaxes only the selected-source and `Leap status: Normal`
requirement. The persistent policy value and holdover marker must be shown in the
operator UI and event log. Motion remains prohibited unless the signed holdover
procedure explicitly authorizes it within its drift and duration limits.

Monitor `chronyc tracking`, PHC/PTP offsets, every board/sensor gate, elapsed
holdover time, and application timestamp behavior throughout operation.

When approved NTP returns, stop applications first and fail closed before
requalification:

```bash
sudo systemctl stop agibot_pm agibot_roudi agibot_top agibot_ui || true
sudo sed -i -E \
  's/^AGIBOT_ALLOW_UNQUALIFIED_TIME=.*/AGIBOT_ALLOW_UNQUALIFIED_TIME=0/' \
  /etc/default/agibot-timesync
sudo chown root:root /etc/default/agibot-timesync
sudo chmod 0644 /etc/default/agibot-timesync
grep '^AGIBOT_ALLOW_UNQUALIFIED_TIME=0$' /etc/default/agibot-timesync

chronyc online
chronyc refresh
chronyc waitsync 60 0.010 5 2
chronyc tracking
chronyc sources -v
sudo /opt/agibot/entry/system/timesync.sh --preflight
sudo rm -f /var/lib/agibot-time/utc-unqualified-holdover

sudo systemctl start agibot_roudi agibot_top agibot_ui
sudo systemctl start agibot_pm
```

Do not remove the UTC-unqualified marker or restart robot applications if
requalification fails. Because the override persists across reboot, clearing it
and closing the holdover record are mandatory end-of-operation steps.

## 7. Change NTP region

Switch profiles only while robot applications and downstream distribution are
stopped. Never mix regions.

```bash
sudo systemctl stop agibot_pm agibot_roudi agibot_top agibot_ui || true
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true

sudo /usr/local/sbin/agibot-time-region china    # default
# sudo /usr/local/sbin/agibot-time-region us
# sudo /usr/local/sbin/agibot-time-region europe

sudo /usr/local/sbin/agibot-time-region --show
chronyc waitsync 60 0.010 5 2
chronyc tracking
chronyc sources -v
sudo /opt/agibot/entry/system/timesync.sh --preflight
```

Re-run the UTC and internal synchronization gates before restarting robot
applications. The selector refuses a change while `agibot_pm.service` is active.

## 8. Supervised large-offset re-step

Continuous chrony is deliberately slew-only and capped at 100 ppm. Approximate
best-case correction times are 100 seconds for 10 ms, 50 minutes for 0.3 s,
5 hours 33 minutes for 2 s, and 3.47 days for 30 s. Use this procedure when the
approved operational threshold requires a faster recovery.

Require two-person approval and keep the robot physically supported:

```bash
sudo systemctl stop agibot_pm agibot_roudi agibot_top agibot_ui || true
sudo systemctl stop chrony.service
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true
sleep 2
pgrep -a -f 'chronyd|ptp4l|phc2sys' || true

sudo systemctl restart agibot-clock-bootstrap.service
sudo test -e /run/agibot-time/bootstrap-qualified
sudo systemctl start chrony.service
chronyc waitsync 60 0.010 5 2
chronyc tracking
chronyc sources -v
sudo /opt/agibot/entry/system/timesync.sh --preflight
```

Then restart the A3 services in a supported no-motion state and requalify every
board and sensor before motion:

```bash
sudo systemctl start agibot_roudi agibot_top agibot_ui
sudo systemctl start agibot_pm
```

## 9. Monitoring commands

```bash
chronyc tracking
chronyc sources -v
chronyc sourcestats -v
systemctl show chrony.service \
  -p ActiveState -p SubState -p NRestarts -p ExecMainStatus
sudo journalctl -u chrony -u agibot-clock-bootstrap -u agibot_pm \
  --since '-30 minutes' --no-pager
pgrep -a -f 'ntp2rtc|chronyd|ptp4l|phc2sys'
grep '^AGIBOT_ALLOW_UNQUALIFIED_TIME=' /etc/default/agibot-timesync
ls -l /var/lib/agibot-time/utc-unqualified-holdover 2>/dev/null || true
sudo tail -n 100 /agibot/log/tsync/ntp_*.log
sudo tail -n 100 /agibot/log/tsync/ptp4l_*.log
sudo tail -n 100 /agibot/log/tsync/phc2sys_*.log
```

Alert on source changes, large correction or skew, excessive delay/dispersion,
chrony restarts, PHC offset regression, PTP role changes, missing clients,
sensor resets, backward timestamps, or a missing qualification marker.

## 10. Troubleshooting

### Bootstrap times out

```bash
getent ahostsv4 ntp1.aliyun.com
sudo journalctl -u agibot-clock-bootstrap --no-pager
sudo /usr/local/sbin/agibot-time-region --show
readlink -f /etc/chrony/sources.d/hope-approved.sources
```

Check DNS, routing, firewall access to UDP/123, and whether at least two approved
sources are reachable. Do not bypass `minsources 2` to make a venue pass.

### Chrony is active but has no selected source

```bash
chronyc activity
chronyc sources -v
chronyc sourcestats -v
chronyc tracking
```

`^?` sources are unreachable or not yet usable. `^+` sources are candidates.
Exactly one `^*` source should be selected after convergence.

### agibot_pm will not start

```bash
sudo systemctl status agibot_pm.service --no-pager
sudo journalctl -u agibot_pm.service --since '-10 minutes' --no-pager
sudo /opt/agibot/entry/system/timesync.sh --preflight
```

The synchronous preflight intentionally blocks startup when chrony, the active
profile, leap status, or selected source is invalid.

### RTC marker is missing

```bash
ls -la /run/agibot-time
sudo hwclock --show --utc
sudo journalctl -u agibot-clock-bootstrap --no-pager
```

Do not delete `bootstrap-qualified` merely because `rtc-updated` is absent. The
runtime step may be valid even when the RTC write fails.

### Unexpected source file

```bash
sudo find /etc/chrony/sources.d -maxdepth 1 -name '*.sources' -ls
```

Stop the rollout. Back up and remove the unapproved source through change
control, then rerun package validation and bootstrap qualification.

## 11. Concrete rollback

Rollback is required for any board/sensor regression, clock-rate violation,
timestamp discontinuity, duplicate owner, PTP role/direction change, instability,
or source-governance failure. Keep the robot supported and motion disabled.

Recover the latest backup path and inspect it first:

```bash
BACKUP=$(sudo readlink -f /var/backups/agibot-time/latest)
echo "BACKUP=$BACKUP"
sudo test -s "$BACKUP/vendor-timesync.sha256"
sudo test -s "$BACKUP/vendor-timesync.stat"
sudo ls -la "$BACKUP/rootfs/opt/agibot/entry/system/timesync.sh"
sudo test -e "$BACKUP/chrony-conf-present" -o \
  -e "$BACKUP/chrony-conf-absent"
```

Restore stock ownership and startup behavior:

```bash
sudo systemctl stop agibot_pm agibot_roudi agibot_top agibot_ui || true
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true
sudo systemctl disable --now chrony.service agibot-clock-bootstrap.service || true

sudo cp -a \
  "$BACKUP/rootfs/opt/agibot/entry/system/timesync.sh" \
  /opt/agibot/entry/system/timesync.sh

if sudo test -e "$BACKUP/chrony-conf-present"; then
  sudo cp -a "$BACKUP/rootfs/etc/chrony/chrony.conf" \
    /etc/chrony/chrony.conf
elif sudo test -e "$BACKUP/chrony-conf-absent"; then
  sudo rm -f /etc/chrony/chrony.conf
else
  echo 'STOP: prior chrony.conf state is unknown'
  false
fi

sudo rm -f /etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf
sudo rm -f /etc/systemd/system/chrony.service.d/agibot-controls.conf
sudo rm -f /etc/systemd/system/agibot-clock-bootstrap.service
sudo rm -f /etc/chrony/sources.d/hope-approved.sources
sudo rm -f /etc/chrony/chrony-bootstrap.conf
sudo rm -f /etc/default/agibot-clock-bootstrap
sudo rm -f /etc/default/agibot-timesync
sudo rm -f /usr/local/sbin/agibot-clock-bootstrap
sudo rm -f /usr/local/sbin/agibot-time-region
sudo rm -f /etc/agibot-time/current
sudo rm -f /etc/agibot-time/regions/china/chrony.sources
sudo rm -f /etc/agibot-time/regions/us/chrony.sources
sudo rm -f /etc/agibot-time/regions/europe/chrony.sources
sudo rm -f /var/lib/agibot-time/utc-unqualified-holdover
sudo rmdir /etc/agibot-time/regions/china 2>/dev/null || true
sudo rmdir /etc/agibot-time/regions/us 2>/dev/null || true
sudo rmdir /etc/agibot-time/regions/europe 2>/dev/null || true
sudo rmdir /etc/agibot-time/regions /etc/agibot-time 2>/dev/null || true
sudo rmdir /var/lib/agibot-time 2>/dev/null || true
sudo systemctl unmask chrony.service || true
sudo systemctl daemon-reload

BASE_CHRONY_ENABLED=$(head -n 1 "$BACKUP/chrony-enabled.txt")
case "$BASE_CHRONY_ENABLED" in
  enabled) sudo systemctl enable chrony.service ;;
  masked) sudo systemctl mask chrony.service ;;
  disabled|not-found|static|indirect|generated) ;;
  *) echo "Review unrecognized prior chrony enablement: $BASE_CHRONY_ENABLED" ;;
esac
sudo systemctl reset-failed

sudo sha256sum -c "$BACKUP/vendor-timesync.sha256"
sudo stat -Lc '%n %U:%G %a %s %y' \
  /opt/agibot/entry/system/timesync.sh
sudo cat "$BACKUP/vendor-timesync.stat"
sudo systemctl start agibot_roudi agibot_top agibot_ui
sudo systemctl start agibot_pm
pgrep -a -f 'ntp2rtc|chronyd|ptp4l|phc2sys' || true
```

The checksum must report `OK`, and the restored owner/mode must match the
captured metadata. Rollback restores the captured vendor launcher, prior
presence or absence of `chrony.conf`, and recognized prior chrony enablement; it
does not guess ownership. Mark external-time correlation as unqualified.
Dependencies newly installed during Phase 1A remain disabled and are not purged
during an incident; package removal is a separate reviewed maintenance action.

## 12. Design invariants

- `chrony` is the only runtime owner of HDU `CLOCK_REALTIME`.
- Runtime operation contains no `makestep`; steps occur only before protected
  services or during supervised maintenance.
- `CLOCK_REALTIME -> eth_hdu PHC -> internal boards` direction is preserved.
- Continuous added slew is capped at 100 ppm.
- Every required board and safety-critical sensor passes pre/post comparison.
- Motive/NatNet cannot control chrony, PHC, PTP, RTC, or `clock_settime`.
- No production deployment occurs without recorded evidence and owner sign-off.

## Official references

- [Alibaba Cloud public NTP service](https://help.aliyun.com/en/ecs/user-guide/alibaba-cloud-ntp-server/)
- [NIST Internet Time Service](https://tf.nist.gov/tf-cgi/servers.cgi/en-en/)
- [PTB NTP/NTS legal-time service](https://www.ptb.de/cms/fileadmin/internet/fachabteilungen/abteilung_8/8.5_metrologische_informationstechnik/8.51/PTB-8.51-MB09-ZS-EN-V09.pdf)
- [chrony 4.3 configuration reference](https://chrony-project.org/doc/4.3/chrony.conf.html)
- [chronyc command reference](https://chrony-project.org/doc/4.4/chronyc.html)
- [LinuxPTP documentation](https://www.linuxptp.org/documentation/)

The architectural rationale, risk analysis, acceptance matrix, and sign-off
form remain in `../../docs/HOPE_A3_Clock_Synchronization_Improvement_Plan.pdf`.
