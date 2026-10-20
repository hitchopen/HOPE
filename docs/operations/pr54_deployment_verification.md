# PR #54 deployment verification

Release review record, 2026-10-19. This covers the Mac operator build and its
matching source release, not authorization to start the robot.

## Acceptance and scope

- **Live ChingMu -> Mac -> HDU networking: complete, confirmed by the operator.**
  This is operator-provided field acceptance, not an automated test performed by
  the coding agent. The MDU does not need access to the ChingMu server.
- Both NatNet2ROS2 and VRPN2ROS2 are compiled into the Mac ROS workspace. The
  confirmed mocap selection starts only the chosen capture stack.
- The container account remains `operator`, with dedicated primary group `hope`.
  Contract 3 rejects silent reuse of an older container; volumes are retained.
- The combined backend reads field schema 5, preserves venue/gripper/VRPN/pants
  settings and revisions, and writes schema 6 with Mac/Linux operator selection.
  Older configuration clients preserve fields they do not submit.
- The live HDU backend and its schema-5 configuration were **not replaced**.
  Profile-selection scripts reject that old backend before changing settings.
- New gripper presets use one packaged catalogue for native and Python
  publishers, with a fail-closed release-wire comparison before Runner startup.
  `legacy` remains the default; a build does not actuate or select a gripper.
- No changes to incoming/return physics, motion assets, Prepare/PD Stand timing,
  serving release lead, or ELink scheduling are introduced by this PR. Existing
  changes already on `main` must be preserved when reconciling branches.

## Local validation

The following checks passed on the ARM64 Mac / Docker Linux environment:

| Check | Result |
| --- | --- |
| ROS build | Six packages: `hope_msgs`, `hope_planner_cpp`, `hope_bringup`, `motion_capture_tracking_interfaces`, `motion_capture_tracking`, `vrpn_mocap` |
| C++ functional tests | All three selected package test runs passed |
| Planner Python tests | 106 passed |
| Focused Mac / VRPN / gripper Python tests | 160 passed |
| Isolated ROS config-snapshot transport | 1 passed, localhost-only domain 229 |
| Console tests | 15 Node tests passed; production package and archive integrity passed |
| Final operator/build-contract tests | 45 passed (overlaps the focused suite; not an additional total) |
| Combined-tree control-plane regression | 400 passed, 14 skipped; includes all Foxglove tests plus gripper and main's v20 asset checks, isolated from the robot domain |
| Real VRPN loopback | Native synthetic server -> actual `vrpn_mocap` -> HOPE relay: 20 raw and 20 normalized samples, millimetre conversion and increasing source stamps checked |
| Schema-5 migration | Read-only check against the field document; simulated Mac selection retained venue settings, with no HDU save/apply |
| Operator smoke check | `operator:hope`, contract 3, six package prefixes, ROS imports, writable log/calibration folders, 14-field helper preflight and authenticated HDU SSH |
| Static checks | Shell syntax, native Mac CLI help and `git diff --check` passed |

The loopback uses standard VRPN with synthetic data; it is separate from the
operator's live-network acceptance above. Neither establishes hardware safety.
The broader run's skipped tests require their separate ROS/native integration
environment; the dedicated domain-229 snapshot transport check did execute.

The inherited VRPN ament lint/style suite is **not passing**: the full run
reported 107 style/check failures and 11 skipped checks. Functional mode excludes
the `linter` label explicitly; it does not claim those checks passed. Use
`--with-lint` to require the full suite. This limitation remains visible to the
reviewer and is not a waived functional failure.

## Reproduction and upgrade boundary

```bash
python3 scripts/macos/hope.py build-operator --arch arm64 --jobs 2 --clean-cache
# Optional full style gate (currently exposes the inherited backlog):
python3 scripts/macos/hope.py test-ros --arch arm64 --with-lint
```

The build never deploys HDU/MDU services, selects an operator profile, or starts
capture, HAL, Runner, motion, gripper commands or robot clock calibration.
The helper container was stopped after verification; persistent volumes remain.

The matching Console installer is
`foxglove/extensions/hope-a3-console/hopeopen.hope-a3-console-1.8.15.foxe`, SHA-256
`16ac5b117b0da13d730787a66a6ea88c6e31f6170bf829c0b88b1a5f1ba4456d`.
It was built and verified, but was not installed into the running Foxglove app.

Deploying the combined backend/Console remains a separately approved, stopped
upgrade. Native Runner/package ABI checks and attended robot motion validation
are not covered by this local Mac acceptance. See
[Mac deployment](macos_deployment.md) and [mocap setup](foxglove_mocap_setup.md).
