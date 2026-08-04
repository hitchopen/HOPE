# Agibot A3 Chrony Quick Start

This is the self-contained installation path for replacing `ntp2rtc.py` with
continuous `chrony` discipline on an A3 HDU. Stage the complete `ntp_sync`
directory on the robot first, then run the A3 commands from
`~/HOPE/agibot/ntp_sync` in one terminal.

> **Offline behavior is preserved.** A3 applications and the existing internal
> board/sensor synchronization continue to start without Internet or NTP.
> Offline UTC is reported as unqualified; only coordination with a separate
> mocap system must wait for the additional qualification in the
> [full plan](../../docs/HOPE_A3_Clock_Synchronization_Improvement_Plan.pdf).

The change keeps the current direction:

```text
chrony -> HDU CLOCK_REALTIME -> phc2sys -> eth_hdu PHC
  -> ptp4l -> A3 boards and synchronized sensors
```

It does not synchronize Motive, change A3's internal PTP design, or add an
Internet dependency to robot startup.

## 0. Stage the complete package on A3

If the HOPE repository is not already on the robot, run the following from the
HOPE repository root on the operator workstation. Set the target explicitly.
An approved fleet deployment method may be used instead of `scp`, but do not
transfer only this README.

```bash
test -f agibot/ntp_sync/MANIFEST.sha256
A3_HOST=REPLACE_WITH_A3_ADDRESS
test "$A3_HOST" != REPLACE_WITH_A3_ADDRESS || {
  echo 'Set A3_HOST to the target robot hostname or address'; false;
}
ssh "agi@$A3_HOST" 'mkdir -p ~/HOPE/agibot'
scp -r agibot/ntp_sync "agi@$A3_HOST:~/HOPE/agibot/"
```

The remaining commands run on the A3 terminal. Staging files does not install
them or change any robot service.

## 1. Verify the package and inspect this A3

```bash
cd ~/HOPE/agibot/ntp_sync
export PKG_ROOT="$PWD"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
test -f "$PKG_ROOT/MANIFEST.sha256"
sha256sum -c MANIFEST.sha256

MISSING=()
for PACKAGE in chrony ethtool linuxptp; do
  dpkg-query -W -f='${db:Status-Status}\n' "$PACKAGE" 2>/dev/null | \
    grep -qx installed || MISSING+=("$PACKAGE")
done
printf 'Missing packages: %s\n' "${MISSING[*]:-none}"

export CHRONY_WAS_ACTIVE=0 CHRONY_WAS_ENABLED=0 CHRONY_WAS_MASKED=0
systemctl is-active --quiet chrony.service && CHRONY_WAS_ACTIVE=1
systemctl is-enabled --quiet chrony.service && CHRONY_WAS_ENABLED=1
test "$(systemctl is-enabled chrony.service 2>/dev/null)" = masked && CHRONY_WAS_MASKED=1

test -e /opt/agibot/entry/system/timesync.sh
ip link show eth_hdu
grep -q 'ntp2rtc.py' /opt/agibot/entry/system/timesync.sh
grep -q 'ptp4l' /opt/agibot/entry/system/timesync.sh
grep -q 'phc2sys' /opt/agibot/entry/system/timesync.sh
```

Every manifest entry must say `OK`. Stop if `eth_hdu` or any of the three stock
launcher tokens is absent. Package presence is detected per robot; nothing here
assumes chrony is installed or missing.

## 2. Put the robot in a supported maintenance state

Disable motion, recording, and external commands. Record the active services so
the procedure restores only the original running set:

```bash
export ACTIVE_STATE=/tmp/agibot-time-active-services
: > "$ACTIVE_STATE"
for SERVICE in agibot_roudi agibot_top agibot_ui agibot_pm; do
  systemctl is-active --quiet "$SERVICE" && printf '%s\n' "$SERVICE" >> "$ACTIVE_STATE"
done
cat "$ACTIVE_STATE"

sudo systemctl stop agibot_pm || true
sudo systemctl stop agibot_ui agibot_top agibot_roudi || true
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true
sudo pkill -TERM -f '/ntp2rtc.py' || true
sudo systemctl mask --now chrony.service 2>/dev/null || true
sleep 2

if pgrep -a -f '[n]tp2rtc.py|[c]hronyd|[p]tp4l|[p]hc2sys'; then
  echo 'STOP: a time process is still active'
  false
fi
```

## 3. Install dependencies and optionally back up

```bash
if ((${#MISSING[@]})); then
  if ! sudo apt-get update || \
     ! sudo apt-get install --no-install-recommends --no-upgrade "${MISSING[@]}"; then
    echo 'Dependency installation failed; restoring the previous running state.'
    ((CHRONY_WAS_MASKED)) || sudo systemctl unmask chrony.service
    ((CHRONY_WAS_ENABLED)) && sudo systemctl enable chrony.service
    ((CHRONY_WAS_ACTIVE)) && sudo systemctl start chrony.service
    while IFS= read -r SERVICE; do sudo systemctl start "$SERVICE"; done < "$ACTIVE_STATE"
    exit 1
  fi
fi
for TOOL in /usr/sbin/chronyd /usr/bin/chronyc /usr/sbin/ethtool \
  /usr/sbin/ptp4l /usr/sbin/phc2sys; do
  test -x "$TOOL" || { echo "Missing executable: $TOOL"; false; }
done
/usr/sbin/ethtool -T eth_hdu
```

`eth_hdu` must report a PTP hardware clock and hardware transmit, receive, and
raw-clock timestamping. If any package is missing and cannot be installed while
offline, stop maintenance and leave the stock files unchanged.

Optional quick backup:

```bash
BACKUP="/var/backups/agibot-time/$(date -u +%Y%m%dT%H%M%SZ)"
sudo install -d -o root -g root -m 0700 "$BACKUP/rootfs"
BACKUP_ITEMS=(opt/agibot/entry/system/timesync.sh)
if sudo test -e /etc/chrony/chrony.conf; then
  BACKUP_ITEMS+=(etc/chrony/chrony.conf)
fi
if ! (
  set -o pipefail
  sudo tar -C / -cpf - "${BACKUP_ITEMS[@]}" |
    sudo tar -C "$BACKUP/rootfs" -xpf -
); then
  echo 'Backup failed; do not continue.'
  false
fi
sudo test -f "$BACKUP/rootfs/opt/agibot/entry/system/timesync.sh"
sudo ln -sfn "$BACKUP" /var/backups/agibot-time/latest
```

Use the PDF procedure when a complete fleet backup and tested rollback are
required.

## 4. Install and validate the reviewed files

```bash
for REGION in china us europe; do
  sudo install -D -o root -g root -m 0644 \
    "$PKG_ROOT/etc/agibot-time/regions/$REGION/chrony.sources" \
    "/etc/agibot-time/regions/$REGION/chrony.sources"
done

for REL in etc/chrony/chrony.conf etc/chrony/chrony-bootstrap.conf \
  etc/default/agibot-timesync etc/default/agibot-clock-bootstrap \
  etc/systemd/system/agibot-clock-bootstrap.service \
  etc/systemd/system/chrony.service.d/agibot-controls.conf \
  etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf; do
  sudo install -D -o root -g root -m 0644 "$PKG_ROOT/$REL" "/$REL"
done

for REL in usr/local/sbin/agibot-time-region \
  usr/local/sbin/agibot-clock-bootstrap; do
  sudo install -D -o root -g root -m 0755 "$PKG_ROOT/$REL" "/$REL"
done

VENDOR_UID=$(sudo stat -Lc %u /opt/agibot/entry/system/timesync.sh)
VENDOR_GID=$(sudo stat -Lc %g /opt/agibot/entry/system/timesync.sh)
VENDOR_MODE=$(sudo stat -Lc %a /opt/agibot/entry/system/timesync.sh)
sudo install -D -o "$VENDOR_UID" -g "$VENDOR_GID" -m "$VENDOR_MODE" \
  "$PKG_ROOT/opt/agibot/entry/system/timesync.sh" \
  /opt/agibot/entry/system/timesync.sh

sudo bash -n /opt/agibot/entry/system/timesync.sh
sudo bash -n /usr/local/sbin/agibot-clock-bootstrap
sudo bash -n /usr/local/sbin/agibot-time-region
sudo /usr/sbin/chronyd -p -f /etc/chrony/chrony.conf >/dev/null
sudo /usr/sbin/chronyd -p -f /etc/chrony/chrony-bootstrap.conf >/dev/null
sudo systemctl daemon-reload

grep -Fx 'After=chrony.service' \
  /etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf
if systemctl cat agibot_pm.service | grep -q 'timesync.sh --preflight'; then
  echo 'STOP: normal A3 startup must not be gated on NTP'
  false
fi
```

The continuous configuration has no `makestep`, caps `maxslewrate` at 100 ppm,
and does not load DHCP-provided time sources. The `agibot_pm` drop-in contains
ordering only, with no `Wants`, `Requires`, or `ExecStartPre` gate.

## 5. Select the region and enable continuous chrony

China is the default. Select only one profile:

```bash
EXTRA_SOURCES=$(sudo find /etc/chrony/sources.d -maxdepth 1 \
  -name '*.sources' ! -name 'hope-approved.sources' -print 2>/dev/null)
test -z "$EXTRA_SOURCES" || { printf 'STOP: unexpected sources:\n%s\n' "$EXTRA_SOURCES"; false; }

sudo /usr/local/sbin/agibot-time-region china
# sudo /usr/local/sbin/agibot-time-region us
# sudo /usr/local/sbin/agibot-time-region europe

sudo systemctl disable --now agibot-clock-bootstrap.service
sudo systemctl unmask chrony.service
sudo systemctl enable --now chrony.service

if chronyc waitsync 15 0.010 5 2; then
  echo 'UTC is qualified for external coordination'
else
  echo 'UTC is unqualified; standalone A3 remains available'
fi
chronyc tracking
chronyc sources -v
sudo /opt/agibot/entry/system/timesync.sh --runtime-status
```

The bootstrap service stays disabled in normal operation. An unavailable NTP
source is not an installation failure and cannot block A3 startup.

## 6. Restore the original service set and verify

```bash
while IFS= read -r SERVICE; do sudo systemctl start "$SERVICE"; done < "$ACTIVE_STATE"
sleep 5

systemctl is-active chrony
while IFS= read -r SERVICE; do systemctl is-active "$SERVICE"; done < "$ACTIVE_STATE"
mapfile -t CHRONY_PIDS < <(pgrep -x chronyd)
test "${#CHRONY_PIDS[@]}" -ge 1
test "${#CHRONY_PIDS[@]}" -le 2
CHRONY_MAIN=$(systemctl show chrony.service -p MainPID --value)
CHRONY_CGROUP=$(systemctl show chrony.service -p ControlGroup --value)
printf '%s\n' "${CHRONY_PIDS[@]}" | grep -qx "$CHRONY_MAIN"
for PID in "${CHRONY_PIDS[@]}"; do
  grep -Fxq "0::$CHRONY_CGROUP" "/proc/$PID/cgroup"
done
! pgrep -a -f '[n]tp2rtc.py'
if grep -qx agibot_pm "$ACTIVE_STATE"; then
  pgrep -a -x ptp4l | grep -F -- '-i eth_hdu -2 -E'
  pgrep -a -x phc2sys | grep -F -- '-s CLOCK_REALTIME -c eth_hdu'
fi
sudo /opt/agibot/entry/system/timesync.sh --runtime-status
sudo journalctl -u chrony -u agibot_pm --since '-10 minutes' --no-pager
```

Expected: one main `chronyd` plus at most one Debian privilege-separated helper,
with every PID in `chrony.service`; no `ntp2rtc.py`; unchanged `ptp4l` and
`phc2sys` directions; and all previously active A3 services restored. With
Internet, chrony should eventually show `Leap status: Normal` and a selected
`^*` source. Without Internet, A3 remains operational and UTC is simply
unqualified.

## 7. Choose the operating option

All supported options keep the continuous configuration at `maxslewrate 100`.
Choose one according to whether external mocap must be ready immediately.

| Option | Use | A3 behavior without Internet |
|---|---|---|
| A. Normal continuous chrony | Default standalone operation | Starts normally; UTC remains unqualified |
| B. Supervised bootstrap | Recommended competition preparation | Run only when approved NTP is reachable |
| C. Controlled boot bootstrap | Correct before services on one planned boot | May delay that boot; A3 still starts after failure |
| D. Early power-on and slew | No clock step is permitted | Starts normally and converges when NTP is available |

### Option A: normal offline-capable operation

This is the installed default. Chrony disciplines the clock when NTP is
available, but neither chrony synchronization nor Internet reachability gates
the A3 application chain.

Activate or restore this option:

```bash
sudo systemctl disable --now agibot-clock-bootstrap.service
sudo systemctl unmask chrony.service
sudo systemctl enable --now chrony.service
sudo /opt/agibot/entry/system/timesync.sh --runtime-status
```

Use this profile for ordinary A3 operation. Do not start external mocap
coordination while the status says UTC is unqualified.

### Option B: supervised bootstrap before competition (recommended)

The bootstrap steps the full offset while A3 applications and downstream time
distribution are stopped. Use it when approved NTP is reachable and immediate
mocap qualification is preferable to waiting for a long slew.

After a step, clock offset can be small while chrony's skew estimate still needs
several NTP poll intervals to settle. The audited Wi-Fi path used approximately
64-second polls and needed about 12 minutes to fall below the provisional 5 ppm
gate, so the strict examples allow up to 20 minutes rather than failing after
two minutes.

Support the robot, disable motion, recording, and external commands, then run:

```bash
BOOTSTRAP_STATE=/tmp/agibot-bootstrap-active-services
: > "$BOOTSTRAP_STATE"
for SERVICE in agibot_roudi agibot_top agibot_ui agibot_pm; do
  systemctl is-active --quiet "$SERVICE" && \
    printf '%s\n' "$SERVICE" >> "$BOOTSTRAP_STATE"
done

sudo systemctl stop agibot_pm || true
sudo systemctl stop agibot_ui agibot_top agibot_roudi || true
sudo pkill -TERM -x phc2sys || true
sudo pkill -TERM -x ptp4l || true
sudo systemctl disable agibot-clock-bootstrap.service
sudo systemctl stop chrony.service
sleep 2

MOCAP_TIME_READY=0
if sudo systemctl restart agibot-clock-bootstrap.service && \
   sudo test -e /run/agibot-time/bootstrap-qualified; then
  sudo systemctl start chrony.service
  if chronyc waitsync 600 0.010 5 2 && \
     sudo /opt/agibot/entry/system/timesync.sh --preflight; then
    MOCAP_TIME_READY=1
  fi
else
  echo 'Bootstrap failed; restoring standalone A3 with UTC unqualified.'
  sudo systemctl start chrony.service
fi

while IFS= read -r SERVICE; do sudo systemctl start "$SERVICE"; done \
  < "$BOOTSTRAP_STATE"
printf 'MOCAP_TIME_READY=%s\n' "$MOCAP_TIME_READY"
```

`MOCAP_TIME_READY=1` qualifies only the A3 clock layer. Complete the PDF's
board/sensor and Motive/NatNet latency gates before play. A value of `0` does
not prevent standalone A3 operation; it prohibits external mocap coordination.

### Option C: bootstrap during one controlled boot

This option orders bootstrap before `chrony`, `agibot_roudi`, `agibot_top`,
`agibot_ui`, and `agibot_pm`. Use it only for a planned online boot. Enabling it
persistently would add an NTP wait to every offline boot.

Activate it immediately before the planned reboot:

```bash
sudo systemctl enable agibot-clock-bootstrap.service
sudo reboot
```

After reconnecting, disable it first so the next boot returns to Option A, then
inspect the result:

```bash
sudo systemctl disable agibot-clock-bootstrap.service
if sudo test -e /run/agibot-time/bootstrap-qualified && \
   chronyc waitsync 600 0.010 5 2 && \
   sudo /opt/agibot/entry/system/timesync.sh --preflight; then
  echo 'A3 clock layer is ready for external mocap qualification.'
else
  echo 'Bootstrap did not qualify UTC; standalone A3 remains available.'
fi
```

If NTP is unavailable, the network-online wait plus bootstrap timeout can delay
startup, but bootstrap does not become an `agibot_pm` success dependency. Do
not leave this service enabled after the controlled boot.

### Option D: power on early and converge by slew

Use this when clock stepping is not approved. A 0.3-second correction at
100 ppm takes approximately 50 minutes, so allow about one hour before external
mocap use. A3 applications may run during convergence.

Activate the normal profile, then wait in an operator terminal:

```bash
sudo systemctl disable --now agibot-clock-bootstrap.service
sudo systemctl unmask chrony.service
sudo systemctl enable --now chrony.service
chronyc waitsync 0 0.010 5 2
sudo /opt/agibot/entry/system/timesync.sh --preflight
```

The zero retry limit makes `waitsync` wait indefinitely for less than 10 ms
remaining correction and less than 5 ppm skew. Interrupting that terminal wait
does not stop A3, but external mocap remains unqualified.

### Unsupported production option: faster continuous slew

Do not permanently raise `maxslewrate` to 1000 ppm. It can distribute a 0.1%
clock-rate change through `phc2sys`, the PHC, PTP clients, and adjusted Linux
timers while the robot is running. Use Option B to correct a large startup
offset before timestamp consumers start. Any higher-slew experiment belongs in
the PDF's supported, motion-disabled bench validation process.

## 8. Confirm one cold boot

```bash
sudo reboot
# Reconnect after boot:
systemctl is-active chrony
systemctl is-enabled agibot-clock-bootstrap.service | grep -qx disabled
if systemctl is-enabled --quiet agibot_pm.service; then
  systemctl is-active agibot_pm.service
fi
sudo /opt/agibot/entry/system/timesync.sh --runtime-status
chronyc tracking
chronyc sources -v
```

An unavailable NTP source must not make an enabled A3 application service fail.
For routine status, use `--runtime-status`; do not place `--preflight` in the A3
startup chain.

Before using a separate mocap system, follow the PDF's strict UTC preflight,
NatNet timestamp-domain, latency, board/sensor, supervised re-step, and rollback
procedures. Those task-level gates do not change standalone A3 operation.
