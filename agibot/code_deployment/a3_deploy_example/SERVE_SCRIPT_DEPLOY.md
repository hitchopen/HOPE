# Fixed A3 vendor-arm serve

This optional package replays one committed serve trajectory through the
installed A3 high-level motion-control interface. It publishes only the 14 arm
joints on `/motion/control/arm_joint_command`; the vendor stack keeps control
of the waist, legs and neck.

The route is intentionally narrow. It contains no ONNX/RKNN model, learned
policy, planner, gripper command, standalone EtherCAT HAL, or 31-axis
body-drive runner. The fixed source asset is:

```text
assets/a3_runtime/serve/motions/serve_policy.csv
SHA-256: 2a7de3f1c97a300069899c139c9eb96e94fd61d3419701d5e44ef37b2bf6641d
```

## Build on the MDU

Build natively after sourcing the installed vendor ROS 2 environment:

```bash
cd agibot/code_deployment/a3_deploy_example
source /agibot/software/v0/entry/env/env.sh
bash scripts/build_serve_vendor_arm_pkg.sh --jobs 2
```

The isolated result is `dist/a3_serve_vendor_arm/`. Copy that whole directory
to a new directory on the MDU; do not overwrite the normal vendor deployment.
The package manifest binds the runner, wrapper, trajectory and runtime
contract by SHA-256.

## Operator flow

The default is a read-only preflight. It validates package hashes, the
AArch64 ELF, linked libraries, trajectory limits, vendor action, arm topics
and `motion_player` state without stopping the vendor publisher:

```bash
cd /path/to/a3_serve_vendor_arm
./run_serve_vendor_arm.sh
```

Real modes require an interactive terminal and the literal
`--confirm-real-commands` argument:

```bash
./run_serve_vendor_arm.sh --hold-only --confirm-real-commands
./run_serve_vendor_arm.sh --prepare-only --confirm-real-commands
./run_serve_vendor_arm.sh --serve-only --confirm-real-commands
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

## Safety and evidence boundary

This is experimental real-robot code, not a certified deployment path.
Simulation and offline checks do not establish physical safety. Use a trained
operator, physical support/safety rope, working e-stop, cleared workspace and
the team's robot-specific approval process. The retained field evidence is an
operator report of ready motion, stroke and ball contact from an earlier
deployment; an exact receipt for this packaged runner was not captured, and
the current original-timing package has not been independently certified.
