# macOS development and operator setup

The Mac build includes both NatNet2ROS2 and VRPN2ROS2, selectable mocap capture,
the field gripper catalogue, and checked Mac/Linux operator profiles. It does
not change motion CSVs, Prepare/PD Stand timing, Planner admission or ELink scheduling.

## Accounts and permissions

Run these tools as your normal Mac account. No personal username is embedded
in the scripts. Your checkout may live anywhere you can read and write,
including a path containing spaces. `~` in `operator.json` expands to your home.
Configuration and SSH credentials stay in `~/.config/hope-macos/` and are not
committed. Do not run the Mac operator scripts with `sudo`.

There are three independent identities:

| Identity | Where it is used | Configuration |
|---|---|---|
| Mac login | Local files, Docker client, Foxglove | Current signed-in user |
| Container operator | Restricted incoming SSH from the HDU | `laptop_user`; generic default `operator` |
| HDU service account | Mac's outgoing SSH to the robot | `ssh_user`; normally `agi` |

The container account must match the HDU's `HOPE_MAC_USER`, and its advertised
helper must match `HOPE_MAC_LIFECYCLE_HELPER`. Their defaults are `operator` and
`/home/operator/.local/libexec/hope-lifecycle`. Neither needs to match the Mac
login. Linux uses its separate `HOPE_LAPTOP_USER` / `HOPE_LAPTOP_LIFECYCLE_HELPER`
settings. Root is used only for isolated container initialization, not on macOS.
The container login uses the dedicated `hope` primary group; Ubuntu's unrelated
pre-existing `operator` system group is left unchanged.

Docker Desktop and Foxglove must already be installed and accessible to the
current user. An organization's software installation policy may require an
administrator; ordinary operator commands do not. The optional launcher uses
Launch Services, so Foxglove need not be installed in `/Applications`.

## Local build environment

From this checkout, with Docker Desktop running:

```bash
# Final local build: development image, both capture backends, functional
# regressions (including schema/gripper compatibility), and operator image.
python3 scripts/macos/hope.py build-operator --arch arm64 --jobs 2 --clean-cache

# Individual steps remain available:
python3 scripts/macos/hope.py image --arch arm64
python3 scripts/macos/hope.py build-ros --arch arm64 --jobs 2
python3 scripts/macos/hope.py test-ros --arch arm64
python3 scripts/macos/hope.py shell --arch arm64
```

These commands run Linux tools in Docker, not on the robot. Sources remain in
the checkout; ROS output goes to the `hope-macos-arm64` volume at `/opt/hope/ros`.
The build and test entry points both discover sibling `NatNet2ROS2/src` and
`VRPN2ROS2/src` sources. The image already includes `ros-jazzy-vrpn`; that library
alone is not the `vrpn_mocap` ROS node, which must be compiled from VRPN2ROS2.
`build-operator` never calls robot SSH, changes an operator selection, installs
HDU/MDU services, or starts acquisition/motion. `--no-tests` is build-only, not
verification. Functional tests fail the command on any regression. Add
`--with-lint` to include upstream ament lint/style checks; the inherited VRPN
style backlog is separate and is not reported as passing by a functional run.
Use `build-ros --clean-cache` after changing checkout/source layouts. Linux
runtime tests that need `/proc`, scheduling APIs or GNU utilities belong inside
the container, not directly on macOS.

For the x86 Runner and simulator on Apple Silicon:

```bash
python3 scripts/macos/hope.py image --arch amd64
python3 scripts/macos/hope.py build-ros --arch amd64 --jobs 2
python3 scripts/macos/hope.py build-runner --arch amd64 --build-only --jobs 2
python3 scripts/macos/hope.py build-sim --arch amd64 --jobs 2
```

Source discovery supports `a3_deploy/` and the existing `agi/` development layout,
without moving either. Native Runner/Gate3 tools require `--arch amd64`; target
cross-compilation uses `build-rockchip` or `build-thor` and requires the matching
vendor sysroot. Thor additionally requires access to the vendor builder image.
No sysroot, private dependency or robot credential is supplied by this port.

`build-runner` / `build-rockchip` / `build-thor` accept either `--build-only` or
`--policy-dir PATH` for packaging, with optional `--teleop-policy-dir PATH`.
They use the existing package builder and its safety/asset checks. Mac emulated
native builds opt out of oneTBB IPO; other build entry points keep their defaults.
The missing legacy RallyV10 scripts are required only for legacy V10 packages,
not policy-directory packages. No new policy or motion is selected by these changes.

Full packaging recreates its target `dist` directory; keep any package you need
in an independent versioned directory first. `--build-only` is not a deployable
package. Successful compilation is not a robot ABI, Gate3 or hardware validation.

For a local command or viewer:

```bash
python3 scripts/macos/hope.py exec --arch arm64 -- ros2 pkg list
python3 scripts/macos/hope.py shell --arch arm64 --publish-port 8765
```

Published TCP ports bind only to Mac localhost. `--virtual-display` runs a
shell/command through Xvfb; it does not provide native Mac graphics or real-time
performance guarantees.

## Operator installation and upgrade

Follow the complete [Mac operator procedure](foxglove_lifecycle.md#macos-operator)
for the container image, private configuration, trusted host keys, matching HDU
supervisor/core and Console 1.8.15. The ready-to-install Console must come from
this same release; older bundled panels do not have the profile selector.

Once prerequisites are complete and lifecycle is STOPPED:

```bash
python3 scripts/macos/operator.py up
python3 scripts/macos/operator.py bind
python3 scripts/macos/operator.py select-mac
python3 scripts/macos/operator.py status
# Return session ownership to Linux before stopping the Mac helper:
python3 scripts/macos/operator.py select-linux
python3 scripts/macos/operator.py down
```

These commands do not press START or send motion commands. The Mac account
requires access to the HDU service account, but no Mac administrator privilege.
The one-time DDS peer configuration on the **HDU** may need its administrator;
this is separate from Mac account permissions. The port installs no sudo policy.

`up`, `bind` and `select-mac` reject stale container identity, checkout, config
mount or loopback-port settings. Existing containers/volumes are not deleted
automatically. Tunnel sockets are scoped to the HDU/account/port; an old venue's
tunnel is never silently reused. `select-linux` remains usable without a healthy
Mac container. The launcher uses only the configured checkout and user's saved
Foxglove layout; it does not install a LaunchAgent.
This release uses container contract 3 so a pre-`hope`-group image is not silently
reused. Recreate only the named container during the stopped upgrade, keeping
its home/state/build volumes; the tools never delete it automatically.

This provides an operator/backend build path, not proof of Docker-to-LAN DDS,
NatNet/VRPN reception, clock synchronization or Xbox device access. Validate those
separately before replacing a Linux mocap acquisition machine. No robot changes
are made merely by cloning, building, or installing the Foxglove extension.

## Schema-5 venue compatibility and ChingMu capture

The combined backend loads field schema 5 without losing `new_20261015`, V2/V4
pants, mocap source, VRPN IP/port/units, table side, addresses or revision. Main's
older Mac/Linux schema lineage is also accepted. A confirmed update writes schema
6, adding the persistent `operator_profile`; never relabel an old schema-3 file
as schema 5 or copy the old backend over an installed schema-5 deployment.
The `select-mac`/`select-linux` scripts submit all twelve fields together and
refuse an older HDU backend before changing settings. Building this checkout
does not upgrade the HDU. Keep an existing live schema-5 backend as-is until a
separately approved, stopped deployment of the matching backend and Console.

Mocap acquisition remains independently started. The 14-field Laptop helper
accepts the gripper, source, VRPN endpoint/units and pants selection; `vrpn`
launches `avatar_pro_hope_bridge.launch.py`, not NatNet or its marker monitor.
See [mocap setup](foxglove_mocap_setup.md) for the confirmed-config command and
the guarded pelvis/world confirmations. The native and Python gripper release
publishers use the same packaged catalogue and are wire-compared before Runner
starts; the default remains `legacy`, and building does not select or actuate a gripper.

The required network path is **ChingMu -> Mac receiver/ROS2 -> HDU**. The MDU
does not need a mocap-server route. A dedicated Mac capture interface needs an
unused address on the ChingMu LAN; the separate HDU link remains intact.
A Wi-Fi address alias only works when that interface actually reaches the same
LAN. Do not route acquisition through the HDU/MDU as a substitute for the Mac's
capture connection. Server `192.168.100.99` is an endpoint, not a multicast group.
Confirm the server's actual VRPN/export mode and any multicast group/data port.

The helper container's default Docker bridge/loopback SSH port is a control-plane
setup, not proof of inbound multicast or bidirectional DDS. Before live use,
verify capture packets inside the receiving container, increasing source stamps,
the intended 200 Hz raw output cap, and fresh ROS2 topics on the HDU in domain
232. [Docker Desktop host networking](https://docs.docker.com/engine/network/drivers/host/#limitations)
is not the Linux host network namespace;
do not assume it supplies physical-NIC multicast membership. A validated host
receiver/UDP relay or bridged Linux capture environment may be needed for a true
multicast export. These network checks remain required after a successful build.

For PR #54, the operator confirmed that live **ChingMu -> Mac -> HDU** network
testing is complete. See the [release verification record](pr54_deployment_verification.md)
for the distinction between that field acceptance, automated local checks, and
the separately controlled HDU backend upgrade.
