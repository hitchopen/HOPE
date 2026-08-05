# Agibot A3 Clock Synchronization Quick Start

This package replaces `ntp2rtc.py` with continuous `chrony` discipline on the
HDU and hardens the existing HDU-to-MDU PTP path. It does not synchronize
Motive, change the robot's timestamp architecture, or require Internet access
for normal A3 operation.

> **Important correction:** the previous package still launched `ptp4l` and
> `phc2sys` as detached children of `agibot_pm` and did not supervise MDU
> `phc2sys`. A temporary PHC disappearance could therefore leave MDU
> `CLOCK_REALTIME` drifting even after `ptp4l` relocked. This revision assigns
> every HDU and MDU clock worker to a restartable systemd service.

The preserved clock direction is:

```text
chrony -> HDU CLOCK_REALTIME
  -> agibot-hdu-phc2sys -> eth_hdu PHC
  -> agibot-hdu-ptp4l -> eth_mdu PHC
  -> agibot-mdu-phc2sys -> MDU CLOCK_REALTIME
```

## 0. Choose the clock-correction mode

Both modes install and use the same supervised HDU and MDU PTP services. The
only difference is how chrony initially corrects HDU `CLOCK_REALTIME`.

### Option A: hard clock reset before play (recommended)

This is a one-time clock **step**, not a robot reboot. It removes a large UTC
offset in seconds and is the preferred competition-preparation path.

> **WARNING:** a clock step can invalidate timers and timestamp assumptions.
> Physically support and secure the robot, disable motion and external
> commands, and stop HDU and MDU robot services before running it. Never step
> the clock while the robot is standing or executing a policy.

After the step, continuous chrony returns to the 100 ppm runtime limit. Chrony's
skew estimate can still need several poll intervals to settle before external
mocap is qualified.

### Option B: slow runtime synchronization

Use this when a clock step is not permitted. Chrony continuously slews at no
more than 100 ppm while A3 remains operational. A 0.3-second correction takes
about 50 minutes, so power on roughly one hour before external mocap use.

Without Internet, A3 applications still start and the internal HDU/MDU time
domain continues in holdover. UTC is unqualified until an NTP source returns.

## 1. Stage and verify the complete package

From the operator workstation, stage the complete directory:

```bash
cd /path/to/HOPE
A3_HOST=REPLACE_WITH_A3_ADDRESS
test "$A3_HOST" != REPLACE_WITH_A3_ADDRESS
ssh "agi@$A3_HOST" 'mkdir -p ~/HOPE/agibot'
scp -r agibot/ntp_sync "agi@$A3_HOST:~/HOPE/agibot/"
```

Run the remaining commands on the HDU terminal. The audited internal MDU
address is the default; override it for robots with a different inventory.

```bash
cd ~/HOPE/agibot/ntp_sync
export PKG_ROOT="$PWD"
export MDU_HOST="${MDU_HOST:-10.42.10.12}"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

sha256sum -c MANIFEST.sha256
hostname | grep -qx hdu
ssh -o BatchMode=yes "agi@$MDU_HOST" 'hostname' | grep -qx mdu
ip link show eth_hdu
ssh -o BatchMode=yes "agi@$MDU_HOST" 'ip link show eth_mdu'
```

Every manifest entry must say `OK`. Stop if either board identity or internal
interface differs.

## 2. Verify or install required packages

Do not assume package presence. Chrony is required only on HDU; both boards need
`linuxptp` and `ethtool`.

```bash
HDU_MISSING=()
for PACKAGE in chrony ethtool linuxptp; do
  dpkg-query -W -f='${db:Status-Status}\n' "$PACKAGE" 2>/dev/null | \
    grep -qx installed || HDU_MISSING+=("$PACKAGE")
done
if ((${#HDU_MISSING[@]})); then
  sudo apt-get update
  sudo apt-get install --no-install-recommends --no-upgrade "${HDU_MISSING[@]}"
fi

ssh -t "agi@$MDU_HOST" '
  MISSING=()
  for PACKAGE in ethtool linuxptp; do
    dpkg-query -W -f="\${db:Status-Status}\n" "$PACKAGE" 2>/dev/null |
      grep -qx installed || MISSING+=("$PACKAGE")
  done
  if ((${#MISSING[@]})); then
    sudo apt-get update
    sudo apt-get install --no-install-recommends --no-upgrade "${MISSING[@]}"
  fi
'

for TOOL in chronyd chronyc ethtool ptp4l phc2sys; do command -v "$TOOL"; done
sudo ethtool -T eth_hdu
ssh "agi@$MDU_HOST" 'command -v ethtool ptp4l phc2sys; sudo ethtool -T eth_mdu'
```

Both interfaces must report a PTP hardware clock and hardware transmit,
receive, and raw-clock timestamping. If missing packages cannot be installed
offline, stop without changing the stock files.

## 3. Secure the robot and stop timestamp consumers

Complete this section for initial installation and before Option A.

```bash
export HDU_ACTIVE_STATE=/tmp/agibot-time-hdu-active-services
export MDU_ACTIVE_STATE=/tmp/agibot-time-mdu-active-services
: > "$HDU_ACTIVE_STATE"
for SERVICE in agibot_roudi agibot_top agibot_ui agibot_pm; do
  systemctl is-active --quiet "$SERVICE" && printf '%s\n' "$SERVICE" >> "$HDU_ACTIVE_STATE"
done

ssh "agi@$MDU_HOST" 'rm -f /tmp/agibot-time-mdu-active-services; touch /tmp/agibot-time-mdu-active-services; for SERVICE in agibot_roudi agibot_top agibot_ui agibot_pm; do systemctl is-active --quiet "$SERVICE" && printf "%s\n" "$SERVICE" >> /tmp/agibot-time-mdu-active-services; done'

sudo systemctl stop agibot_pm agibot_ui agibot_top agibot_roudi 2>/dev/null || true
ssh "agi@$MDU_HOST" 'sudo systemctl stop agibot_pm agibot_ui agibot_top agibot_roudi 2>/dev/null || true'

sudo systemctl stop agibot-hdu-phc2sys.service agibot-hdu-ptp4l.service 2>/dev/null || true
sudo pkill -TERM -x phc2sys 2>/dev/null || true
sudo pkill -TERM -x ptp4l 2>/dev/null || true
sudo pkill -TERM -f '/ntp2rtc.py' 2>/dev/null || true
ssh "agi@$MDU_HOST" 'sudo systemctl stop agibot-mdu-phc2sys.service agibot-mdu-ptp4l.service 2>/dev/null || true; sudo pkill -TERM -x phc2sys 2>/dev/null || true; sudo pkill -TERM -x ptp4l 2>/dev/null || true'
sudo systemctl stop chrony.service 2>/dev/null || true
```

Optional backup:

```bash
BACKUP="/var/backups/agibot-time/$(date -u +%Y%m%dT%H%M%SZ)"
sudo install -d -o root -g root -m 0700 "$BACKUP/rootfs"
HDU_BACKUP=(opt/agibot/entry/system/timesync.sh)
sudo test -e /etc/chrony/chrony.conf && HDU_BACKUP+=(etc/chrony/chrony.conf)
(set -o pipefail; sudo tar -C / -cpf - "${HDU_BACKUP[@]}" | sudo tar -C "$BACKUP/rootfs" -xpf -)
sudo ln -sfn "$BACKUP" /var/backups/agibot-time/latest

ssh "agi@$MDU_HOST" 'BACKUP="/var/backups/agibot-time/$(date -u +%Y%m%dT%H%M%SZ)"; sudo install -d -o root -g root -m 0700 "$BACKUP/rootfs/opt/agibot/entry/system"; sudo cp -a /opt/agibot/entry/system/timesync.sh "$BACKUP/rootfs/opt/agibot/entry/system/"; sudo ln -sfn "$BACKUP" /var/backups/agibot-time/latest'
```

## 4. Install the common hardening

Install HDU configuration and supervised clock workers:

```bash
for REGION in china us europe; do
  sudo install -D -o root -g root -m 0644 \
    "$PKG_ROOT/etc/agibot-time/regions/$REGION/chrony.sources" \
    "/etc/agibot-time/regions/$REGION/chrony.sources"
done

for REL in \
  etc/chrony/chrony.conf \
  etc/chrony/chrony-bootstrap.conf \
  etc/default/agibot-timesync \
  etc/default/agibot-clock-bootstrap \
  etc/systemd/system/agibot-clock-bootstrap.service \
  etc/systemd/system/agibot-hdu-ptp4l.service \
  etc/systemd/system/agibot-hdu-phc2sys.service \
  etc/systemd/system/chrony.service.d/agibot-controls.conf \
  etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf; do
  sudo install -D -o root -g root -m 0644 "$PKG_ROOT/$REL" "/$REL"
done

for REL in usr/local/sbin/agibot-time-region usr/local/sbin/agibot-clock-bootstrap; do
  sudo install -D -o root -g root -m 0755 "$PKG_ROOT/$REL" "/$REL"
done

VENDOR_UID=$(sudo stat -Lc %u /opt/agibot/entry/system/timesync.sh)
VENDOR_GID=$(sudo stat -Lc %g /opt/agibot/entry/system/timesync.sh)
VENDOR_MODE=$(sudo stat -Lc %a /opt/agibot/entry/system/timesync.sh)
sudo install -o "$VENDOR_UID" -g "$VENDOR_GID" -m "$VENDOR_MODE" \
  "$PKG_ROOT/opt/agibot/entry/system/timesync.sh" \
  /opt/agibot/entry/system/timesync.sh

sudo systemctl disable --now agibot_timesync.service 2>/dev/null || true
sudo systemctl daemon-reload
sudo systemctl enable agibot-hdu-ptp4l.service agibot-hdu-phc2sys.service
```

Install the MDU workers and replace its detached launcher:

```bash
ssh "agi@$MDU_HOST" 'mkdir -p /tmp/hope-mdu-time'
scp "$PKG_ROOT/mdu/etc/systemd/system/agibot-mdu-ptp4l.service" \
    "$PKG_ROOT/mdu/etc/systemd/system/agibot-mdu-phc2sys.service" \
    "$PKG_ROOT/mdu/opt/agibot/entry/system/timesync.sh" \
    "agi@$MDU_HOST:/tmp/hope-mdu-time/"

ssh "agi@$MDU_HOST" '
  set -e
  VENDOR_UID=$(sudo stat -Lc %u /opt/agibot/entry/system/timesync.sh)
  VENDOR_GID=$(sudo stat -Lc %g /opt/agibot/entry/system/timesync.sh)
  VENDOR_MODE=$(sudo stat -Lc %a /opt/agibot/entry/system/timesync.sh)
  sudo install -o root -g root -m 0644 /tmp/hope-mdu-time/agibot-mdu-ptp4l.service /etc/systemd/system/agibot-mdu-ptp4l.service
  sudo install -o root -g root -m 0644 /tmp/hope-mdu-time/agibot-mdu-phc2sys.service /etc/systemd/system/agibot-mdu-phc2sys.service
  sudo install -o "$VENDOR_UID" -g "$VENDOR_GID" -m "$VENDOR_MODE" /tmp/hope-mdu-time/timesync.sh /opt/agibot/entry/system/timesync.sh
  sudo systemctl disable --now agibot_timesync.service 2>/dev/null || true
  sudo systemctl daemon-reload
  sudo systemctl enable agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service
'

sudo bash -n /opt/agibot/entry/system/timesync.sh
sudo bash -n /usr/local/sbin/agibot-clock-bootstrap
sudo chronyd -p -f /etc/chrony/chrony.conf >/dev/null
sudo chronyd -p -f /etc/chrony/chrony-bootstrap.conf >/dev/null
ssh "agi@$MDU_HOST" 'sudo bash -n /opt/agibot/entry/system/timesync.sh; systemd-analyze verify /etc/systemd/system/agibot-mdu-ptp4l.service /etc/systemd/system/agibot-mdu-phc2sys.service'
```

The four workers use `Restart=always` with no start-rate limit. If a PHC
temporarily disappears, an exited worker is retried until the device returns.
The application services have ordering only and are never success-gated on NTP
or PTP lock, preserving offline A3 startup behavior.

## 5. Option A activation: hard clock reset

Confirm again that the robot is supported, motion is disabled, and both boards'
application services are stopped. Approved NTP must be reachable.

```bash
sudo /usr/local/sbin/agibot-time-region china
# Use "us" or "europe" instead when appropriate.

sudo systemctl disable agibot-clock-bootstrap.service
sudo systemctl stop chrony.service
BOOTSTRAP_OK=0
if sudo systemctl restart agibot-clock-bootstrap.service && \
   sudo test -e /run/agibot-time/bootstrap-qualified; then
  BOOTSTRAP_OK=1
else
  echo 'Hard clock reset failed; continuing only in UTC-unqualified mode.'
fi

sudo systemctl start chrony.service
sudo systemctl start agibot-hdu-ptp4l.service agibot-hdu-phc2sys.service
ssh "agi@$MDU_HOST" 'sudo systemctl start agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service'

MOCAP_TIME_READY=0
if test "$BOOTSTRAP_OK" -eq 1 && \
   chronyc waitsync 600 0.010 5 2 && \
   sudo /opt/agibot/entry/system/timesync.sh --preflight && \
   ssh "agi@$MDU_HOST" 'systemctl is-active --quiet agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service'; then
  MOCAP_TIME_READY=1
fi
printf 'MOCAP_TIME_READY=%s\n' "$MOCAP_TIME_READY"
```

If the bootstrap command fails, do not repeat it with robot services running.
Start chrony and all four clock workers as shown above; standalone A3 remains
available with UTC unqualified.

After an authorized operator confirms the robot is clear to resume:

```bash
while IFS= read -r SERVICE; do sudo systemctl start "$SERVICE"; done < "$HDU_ACTIVE_STATE"
ssh "agi@$MDU_HOST" 'while IFS= read -r SERVICE; do sudo systemctl start "$SERVICE"; done < /tmp/agibot-time-mdu-active-services'
```

## 6. Option B activation: slow runtime synchronization

This path never steps the running clock. It starts the same supervised internal
distribution chain, then lets chrony converge at `maxslewrate 100`.

```bash
sudo /usr/local/sbin/agibot-time-region china
sudo systemctl disable --now agibot-clock-bootstrap.service
sudo systemctl unmask chrony.service
sudo systemctl enable --now chrony.service
sudo systemctl start agibot-hdu-ptp4l.service agibot-hdu-phc2sys.service
ssh "agi@$MDU_HOST" 'sudo systemctl start agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service'

while IFS= read -r SERVICE; do sudo systemctl start "$SERVICE"; done < "$HDU_ACTIVE_STATE"
ssh "agi@$MDU_HOST" 'while IFS= read -r SERVICE; do sudo systemctl start "$SERVICE"; done < /tmp/agibot-time-mdu-active-services'

sudo /opt/agibot/entry/system/timesync.sh --runtime-status
chronyc tracking
chronyc sources -v
```

A3 can operate while chrony converges or while NTP is unavailable. Before
external mocap use, wait separately:

```bash
chronyc waitsync 0 0.010 5 2
sudo /opt/agibot/entry/system/timesync.sh --preflight
```

Do not raise continuous `maxslewrate` to 1000 ppm. A one-time secured clock
step is preferable to distributing a 0.1% runtime rate error through every
board and timestamp consumer.

## 7. Verify persistence and incident hardening

```bash
systemctl is-enabled agibot-hdu-ptp4l.service agibot-hdu-phc2sys.service
systemctl is-active agibot-hdu-ptp4l.service agibot-hdu-phc2sys.service
systemctl show agibot-hdu-ptp4l.service agibot-hdu-phc2sys.service -p MainPID -p NRestarts
pgrep -a -x ptp4l
pgrep -a -x phc2sys

ssh "agi@$MDU_HOST" '
  systemctl is-enabled agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service
  systemctl is-active agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service
  systemctl show agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service -p MainPID -p NRestarts
  pgrep -a -x ptp4l
  pgrep -a -x phc2sys
  journalctl -b --no-pager -u agibot-mdu-ptp4l.service -u agibot-mdu-phc2sys.service -n 40
'
```

There must be exactly one `ptp4l` and one `phc2sys` process per board, each in
its named systemd cgroup. A later PHC error may increment `NRestarts`, but the
service must return to `active (running)`. Confirm one cold boot before play;
an unavailable NTP source must not prevent enabled A3 application services from
starting.

`MOCAP_TIME_READY=1` qualifies only the A3 clock layer. The OptiTrack adapter
now maps Motive QPC capture ticks into its own Chrony-disciplined ROS system
time with NatNet echo synchronization, but that adapter must independently
pass `natnet_preflight.py` and its clock-uncertainty gate. Defer remaining
board/sensor acceptance, rollback, and the runner's mandatory official-stand
behavior for invalid target/base data to the
[full plan](../../docs/HOPE_A3_Clock_Synchronization_Improvement_Plan.pdf).
