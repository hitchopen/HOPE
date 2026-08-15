# Foxglove Runner three-machine lifecycle

This layer replaces the repeated terminal work in hardware-runbook STEP
0/1/2A/2B/4/5 with a fixed HDU-resident supervisor. It does not expose a
remote shell. Foxglove can only:

1. confirm four validated RFC1918 IPv4 addresses while the stack is stopped;
2. start the fixed Laptop -> HDU -> MDU sequence;
3. kill every lifecycle-managed session in reverse order, restore `agibot_pm`,
   and collect the current session logs.

The native MDU Runner remains the authority for PASSIVE, PD_STAND, MOTION,
role, serve state, action admission and action acknowledgements. The lifecycle
supervisor owns process state only.

## Operator surface

The custom console calls these exact services through port `8766`:

| Service | Type | Meaning |
|---|---|---|
| `/hope/lifecycle/apply_config` | `rcl_interfaces/srv/SetParameters` | Exact four-string configuration request; not a generic parameter service |
| `/hope/lifecycle/start` | `std_srvs/srv/Trigger` | Start STEP 0/1/2A/2B/4/5 asynchronously |
| `/hope/lifecycle/kill_all_and_collect` | `std_srvs/srv/Trigger` | Kill all fixed lifecycle-managed sessions, restore `agibot_pm`, collect logs |

`start_parameter_services=False`, the bridge parameter allowlist is empty, and
browser topic publishing remains disabled. `apply_config` rejects missing,
duplicate, non-string, non-IPv4, loopback, multicast, link-local and non-RFC1918
values. The only accepted names are:

- `laptop_wifi_ip`
- `hdu_wifi_ip`
- `mdu_internal_ip`
- `motive_ip`

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
```

The HDU robot account must have non-interactive SSH authentication to the
Laptop operator account and the MDU robot account. The Laptop must retain its
existing non-interactive access to the HDU and MDU for STEP 0 evidence and STEP
6 collection. Verify all four directions before enabling the supervisor:

```bash
# From HDU; substitute the values printed on the Laptop.
ssh -o BatchMode=yes <laptop-user>@<laptop-wifi-ip> true
ssh -o BatchMode=yes <robot-user>@<mdu-internal-ip> true

# From Laptop
ssh -o BatchMode=yes "${ROBOT_USER}@${HDU_IP}" true
ssh -o BatchMode=yes -J "${ROBOT_USER}@${HDU_IP}" \
  "${ROBOT_USER}@${MDU_IP}" true
```

Do not put passwords in the repository, UI, ROS messages, unit files, helper
arguments or process environment. Install SSH public keys once instead.

`tmux` must be installed on all three machines. The MDU robot account also
needs narrow passwordless sudo only for the commands already present in STEP 4
and STEP 6. Install with `visudo -f /etc/sudoers.d/hope-lifecycle`:

```sudoers
<robot-user> ALL=(root) NOPASSWD: /usr/bin/systemctl stop agibot_pm.service
<robot-user> ALL=(root) NOPASSWD: /usr/bin/systemctl start agibot_pm.service
```

Confirm the resolved command path with `command -v systemctl` on the MDU
before installing that file. The helper uses `sudo -n` and fails closed rather
than prompting.

## Install the fixed helper

On the Laptop:

```bash
cd "$HOPE_ROOT"
sudo install -D -o root -g root -m 0755 \
  foxglove/helpers/hope-lifecycle \
  /usr/local/libexec/hope-lifecycle
```

Stage the same file to the HDU and MDU, then install it root-owned at the same
absolute path:

```bash
cd "$HOPE_ROOT"
scp foxglove/helpers/hope-lifecycle \
  "${ROBOT_USER}@${HDU_IP}:/tmp/hope-lifecycle"
ssh "${ROBOT_USER}@${HDU_IP}" \
  'sudo install -D -o root -g root -m 0755 /tmp/hope-lifecycle /usr/local/libexec/hope-lifecycle'

scp -o "ProxyJump=${ROBOT_USER}@${HDU_IP}" \
  foxglove/helpers/hope-lifecycle \
  "${ROBOT_USER}@${MDU_IP}:/tmp/hope-lifecycle"
ssh -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}" \
  'sudo install -D -o root -g root -m 0755 /tmp/hope-lifecycle /usr/local/libexec/hope-lifecycle'
```

## Install the HDU supervisor

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
  hope-lifecycle-supervisor.service
```

These four units form the always-available, fixed-action control plane needed
to start a stopped robot from Foxglove. They do not start HAL, Planner or
Runner, and they do not stop `agibot_pm`. In lifecycle mode, leave them running
after `KILL ALL & COLLECT` so the next session can be started without a terminal.

Read-only verification:

```bash
systemctl status hope-lifecycle-supervisor.service --no-pager
ros2 topic echo /hope/lifecycle/summary --once
ros2 service type /hope/lifecycle/apply_config
ros2 service type /hope/lifecycle/start
ros2 service type /hope/lifecycle/kill_all_and_collect
```

Do not use Start as an installation smoke test on hardware.

## Start behavior

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
`KILL ALL & COLLECT` recovery. STEP 4 retains the documented stop of `agibot_pm`.
An `aimrt_main_hal` inside the `/system.slice/agibot_pm.service` cgroup is the
expected vendor HAL and is accepted by preflight. STEP 4 stops the unit first,
verifies that this HAL has exited, and only then launches the fixed `hope-hal`
session. A HAL outside that cgroup, or one remaining after the unit stops,
fails closed; stale unmanaged HAL cleanup is an exceptional recovery action
rather than part of a normal one-click start.

`start` returns quickly. Progress and failures are published under
`/hope/lifecycle/**`; the console must reach `RUNNING · RUNNER` and
`START_COMPLETE_RUNNER_PASSIVE` before the operator uses Stand/Calibration/
Refresh x_hit/Ready or the Server flow.

The helper intentionally preserves the current fixed3 Runner argv. The checked
in model_21800 policy metadata is `training_recipe=rally_v14`, while the native
`--serve` controller rejects anything other than its qualified `rally_v17`
artifact. Local `SERVER/RECEIVER` role state is available, but this lifecycle
does not add `--serve` or bypass that artifact check. `Ready to Serve` and
`Serve` remain disabled until a separately qualified compatible Runner package
reports `serve_capability=AVAILABLE`.

## Kill behavior

`KILL ALL & COLLECT` is intentionally separate from E-stop. It is accepted for
a managed `RUNNING` or `FAILED` session without requiring a fresh Runner mode
or first transitioning to PASSIVE/PD_STAND. Before confirming it:

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

## Verification boundary

Repository unit tests, shell parsing and extension packaging prove only the
wire/configuration restrictions and fixed command construction. A supervised
three-machine dry rehearsal and then an attended hardware trial are still
required. In particular, verify SSH keys, MDU sudoers, tmux availability,
exact deployed Runner package, ROS service discovery, process identity, log
flush, `agibot_pm` restoration and the physical PASSIVE/PD_STAND transitions.
