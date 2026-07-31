# A3 high-level arm runtime

This runtime replays one manifest-bound serve trajectory through the
installed A3 high-level motion-control interface. It publishes only the 14 arm
joints on `/motion/control/arm_joint_command`; the vendor stack keeps control
of the waist, legs and neck.

The route is intentionally narrow. MuJoCo planning, DLS IK and CSV export run
offboard in the parent application. The MDU package contains no ONNX/RKNN
model, learned policy, planner, gripper command, standalone EtherCAT HAL, or
31-axis body-drive runner. Its default source asset is the A3-validated PR #18
reference:

```text
assets/validated/serve_policy.csv
SHA-256: 2a7de3f1c97a300069899c139c9eb96e94fd61d3419701d5e44ef37b2bf6641d
```

## Build on the MDU

Build natively after sourcing the installed vendor ROS 2 environment:

```bash
cd /path/to/HOPE/apps/a3_mujoco_serve
source /agibot/software/v0/entry/env/env.sh
bash runtime/scripts/build_a3_app.sh --jobs 2
```

The isolated result is `dist/a3_serve_vendor_arm/`. Copy that whole directory
to a new directory on the MDU; do not overwrite the normal vendor deployment.
The package manifest binds the runner, wrapper, trajectory, runtime contract,
qualification result and approval registry by SHA-256. The default build must
print `qualification: approved`; its motion hash remains the PR #18 hash shown
above.

## Operator flow

The default is a read-only preflight. It validates package hashes, the
AArch64 ELF, linked libraries, trajectory limits, vendor action, arm topics
and `motion_player` state without stopping the vendor publisher:

```bash
cd /path/to/a3_serve_vendor_arm
./run_a3_app.sh
```

Real modes require an interactive terminal and the literal
`--confirm-real-commands` argument:

```bash
./run_a3_app.sh --hold-only --confirm-real-commands
./run_a3_app.sh --prepare-only --confirm-real-commands
./run_a3_app.sh --serve-only --confirm-real-commands
```

`--hold-only` verifies a short measured-state hold. `--prepare-only` moves both
arms to the committed ready pose and holds until Ctrl-C. In `--serve-only`,
press Space at physical ball release; the runner holds READY for 1.000 seconds
and then plays the original 100 Hz stroke and return. Nominal strike is about
1.060 seconds after Space. The left wrist-roll remains at its measured entry
value because the source value exceeds the high-level A3 limit.

Immediately before custom publication, the wrapper prearms the runner, asks
the vendor process manager to stop only `motion_player`, verifies publisher
handoff, and releases the runner with `SIGUSR1`. It stops the custom publisher
before automatically restarting `motion_player` on exit or failure.

The wrapper rehashes the complete package and requires an `approved`
qualification immediately before every real mode. The former
`A3_VENDOR_ARM_QUICK_DEPLOY=1` shortcut is deprecated and no longer skips this
audit. A motion that passes offline gates but is absent from
`approved_motions.json` remains a `candidate`: read-only preflight works, while
all command-publishing modes fail closed.
The registry itself has a hash trust anchor in both the builder and runtime;
adding a future approved hash therefore requires an explicit reviewed code
change rather than an adjacent manifest edit.

## Safety and evidence boundary

The exact PR #18 reference runtime and CSV were fully tested on Agibot A3 and
are executable and safe. The parent application's
`assets/validated/pr18_a3_serve_demo.mp4` is retained as a demo reference. That
hardware result applies to the exact reference application and retained
operator procedure. Simulation and offline checks do not establish safety for
a different, newly generated trajectory. Use a trained operator, physical
support/safety rope, working e-stop, cleared workspace and the team's
robot-specific approval process whenever qualifying another artifact.
