# HDU clock baseline installation and current Laptop NTP access

Use this procedure with the [new-robot commissioning checklist](new_robot_commissioning.md).
It installs a pinned vendor-owned HDU PTP baseline. It does not prove MDU PTP,
Laptop synchronization, motion-capture time, or physical robot readiness.

## 1. Identify the machine and restoration topology

Record the current HDU/MDU identities, boot IDs, interfaces, routes and trusted SSH
host keys. Read the actual `/opt/agibot/entry/run_agibot.sh` on the HDU. If it invokes
`system/timesync_guard.py`, use the
[guarded BSP paired-reboot procedure](new_robot_commissioning.md#新-bsp-的-timesync_guardpy必须先分支).
Ordinary `--install` rejects that topology before mutation. Hot Time Calibration
also rejects it unless the matching [receipt-bound adapter](guarded_clock_calibration.md)
is installed. Do not remove the vendor guard or fabricate MDU uptime.

Installation requires an attended maintenance window: the robot is physically
supported, external motion is disabled, Runner is stopped, and the physical E-stop
is reachable. Record and back up the effective service configuration first. For
paired-reboot mode the MDU vendor services, HAL, Runner and PTP must already be
stopped; the installer verifies this through trusted HDU-to-MDU SSH.

## 2. Verify and stage the fixed package

On **Laptop HOST**, export `REPO_ROOT` for this HOPE checkout and `HDU_IP` for the
verified current HDU. Do not reuse a previous robot's address or checkout path.

```bash
(
set -euo pipefail
: "${REPO_ROOT:?export the current HOPE checkout}"
: "${HDU_IP:?export the verified current HDU address}"
cd "$REPO_ROOT"
CLOCK_PACKAGE=$PWD/agibot/ntp_sync
CLOCK_INSTALLER=$PWD/foxglove/a3/install_hdu_clock_baseline.sh

test "$(sha256sum "$CLOCK_PACKAGE/MANIFEST.sha256" | awk '{print $1}')" = \
  92fa52faef8a89608be2c90bad3a4250803c624c61d87d96f127b3ed3fdc2168
test "$(sha256sum "$CLOCK_INSTALLER" | awk '{print $1}')" = \
  419a103d3557ffd0172bf826d7a707a5d826daa6605fc973d9707f6fb842c534
"$CLOCK_INSTALLER" --verify-package-only "$CLOCK_PACKAGE"
ssh "agi@$HDU_IP" 'install -d -m 0755 "$HOME/hope_clock_baseline"'
rsync -azP "$CLOCK_PACKAGE/" "agi@$HDU_IP:hope_clock_baseline/"
scp "$CLOCK_INSTALLER" "agi@$HDU_IP:/tmp/hope-install-hdu-clock-baseline"
ssh "agi@$HDU_IP" \
  'chmod 0755 /tmp/hope-install-hdu-clock-baseline &&
   /tmp/hope-install-hdu-clock-baseline --verify-package-only "$HOME/hope_clock_baseline"'
echo HDU_CLOCK_PACKAGE_STAGED_OK
)
```

Require `HDU_CLOCK_PACKAGE_VERIFY_OK` locally and remotely, then
`HDU_CLOCK_PACKAGE_STAGED_OK`. Package verification does not install anything.
The package's `timesync.sh` SHA-256 is
`80c35a34fef2f7b7abd845e09c7c3770dd9a863ed68e9caa71f6f4161afccc5b`.
It keeps vendor-owned Layer-2/E2E/domain-0 PTP with `--tx_timestamp_timeout 100`.
Do not simultaneously install separate HDU ptp4l/phc2sys units.

The installer currently selects the China profile. For another region, prepare
and review that region's baseline before installation. The China source delay
allowance does not relax the final UTC correction limit of 10 ms.

## 3. Install only in the confirmed maintenance window

For an ordinary supported BSP, run from **Laptop HOST**:

```bash
ssh -tt "agi@$HDU_IP" \
  'sudo /tmp/hope-install-hdu-clock-baseline --install "$HOME/hope_clock_baseline"'
```

For a guarded BSP, use `--install-for-paired-reboot` instead, after completing the
paired-board stop checks in the commissioning checklist. The installer asks for
`INSTALL_HDU_CLOCK_BASELINE` and saves original files and active-service state to
`/var/backups/hope-hdu-clock-baseline-<UTC>`.

- `HDU_CLOCK_BASELINE_INSTALLED_OK`: ordinary installation and restoration passed;
  still perform live clock acceptance below.
- `HDU_CLOCK_BASELINE_STAGED_PENDING_PAIRED_REBOOT`: guarded-BSP assets are staged,
  with time consumers left stopped. Complete the attended bootstrap and paired
  reboot, then verify both boot IDs and all time consumers.
- `HDU_CLOCK_BASELINE_ROLLED_BACK`: installation failed and original files/services
  were restored. Diagnose before retrying.
- `HDU_CLOCK_BASELINE_ROLLBACK_INCOMPLETE`: keep the robot supported and resolve the
  incomplete restoration. Preserve the full terminal output.

New dependency packages are retained after rollback. Never run the remote `/tmp`
installer as a local Laptop command. The continuous clock remains slew-only;
installation does not authorize an unattended hard step or motion restart.

### 3.1 Complete the HOPE boot and button policy

The generic baseline deliberately leaves `agibot-clock-bootstrap.service`
disabled. For HOPE power-on readiness, stage both
`foxglove/a3/configure_hdu_boot_clock.sh` and its adjacent
`30-hope-clock-boot-ordering.conf`, then run on the HDU:

```bash
sudo bash ~/HOPE/foxglove/a3/configure_hdu_boot_clock.sh --enable-next-boot
```

This enables the fixed one-shot ahead of vendor and HOPE consumers for the next
boot; it performs no current clock operation. Follow the
[boot acceptance procedure](new_robot_commissioning.md#hope-开机一次性校时-setup).
An enabled bootstrap is expected after this additional HOPE setup, while the
generic baseline alone remains disabled. Reapply this policy after reinstalling
the generic baseline. For guarded BSPs, also install and qualify the
[Time Calibration button adapter](guarded_clock_calibration.md). Treat these as
separate setup/acceptance steps; successful boot NTP does not prove the button.

## 4. Authorize the current Laptop as an NTP client

On the **HDU**, set `LAPTOP_WIFI_IP` to the current Laptop address on the verified
route. Back up an existing drop-in before replacing this one file. An old Laptop
ACL must not be mistaken for permission for the new computer.

```bash
(
set -euo pipefail
: "${LAPTOP_WIFI_IP:?set the verified current Laptop IPv4 address}"
python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "$LAPTOP_WIFI_IP"
FILE=/etc/systemd/system/chrony.service.d/hope-laptop-access.conf
sudo install -d -m 0755 "$(dirname "$FILE")"
if sudo test -e "$FILE"; then sudo cp -a "$FILE" "$FILE.before-$(date -u +%Y%m%dT%H%M%SZ)"; fi
printf '[Service]\nExecStartPost=/usr/bin/chronyc allow %s/32\n' "$LAPTOP_WIFI_IP" | sudo tee "$FILE"
sudo systemctl daemon-reload
sudo chronyc allow "$LAPTOP_WIFI_IP/32"
chronyc -n sources -v
chronyc tracking
chronyc waitsync 1 0.010 0 1
echo HDU_CLOCK_BASELINE_OK
)
```

This installs a persistent ACL and applies it to the running daemon without a
clock step. Check the effective unit/drop-ins and selected external source. The
HDU must not synchronize from this Laptop while the Laptop synchronizes from HDU.

## 5. Point Laptop chrony at the current HDU

On **Laptop HOST**, inspect all current chrony sources and back them up. Retire the
previous robot's source after confirming it is obsolete. The command below only
replaces the HOPE source file; it does not remove other configured time sources.

```bash
(
set -euo pipefail
: "${HDU_IP:?set the verified current HDU IPv4 address}"
python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "$HDU_IP"
FILE=/etc/chrony/sources.d/hope-hdu.sources
sudo install -d -m 0755 "$(dirname "$FILE")"
if sudo test -e "$FILE"; then sudo cp -a "$FILE" "$FILE.before-$(date -u +%Y%m%dT%H%M%SZ)"; fi
printf 'server %s iburst prefer minpoll 2 maxpoll 4\n' "$HDU_IP" | sudo tee "$FILE"
sudo systemctl restart chrony.service
sudo chronyc burst 8/8 "$HDU_IP"
chronyc waitsync 120 0.010 0 1
chronyc -n sources -v | awk -v host="$HDU_IP" '$1 == "^*" && $2 == host {ok=1} END {exit !ok}'
chronyc tracking
echo LAPTOP_CURRENT_HDU_NTP_OK
)
```

No `makestep` is issued. If the current HDU is not selected or the correction is
still above 10 ms, stop acceptance and diagnose source, route and firewall state.
Finally verify MDU PTP, fresh cross-board observations and the used mocap computer's
time separately. Repeat acceptance after the attended reboot; pre-reboot success
is not proof of persistence.
