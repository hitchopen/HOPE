# a3_deploy / a3_deploy_example

> Current deployment: **model17400 + PPQ004**, receive-only. See [bundle staging](../../docs/operations/deploy_and_gate3.md#model17400-pr87-runtime-alignment).
> For the shared real-ball coefficients, see [ball physics](../../docs/operations/ball_physics_calibration.md).
> Runner mode transitions and terminal serve recovery: [control contract](../../docs/operations/foxglove_runner_control_contract.md).
> Staging a package does not activate the robot.

**`a3_deploy/` is a revised fork of the official AgiBot A3 deploy code.** This
directory documents the public HOPE deploy contract and ships **two runners with
two roles**:

- a **clean-room Python reference runner** (`reference/`) — the MuJoCo
  evaluation/simulation harness that implements the contract end to end, and
- the **native C++ runner** `a3_pingpong` (`src/`) — the hardware deploy path
  that was exercised on a real Agibot A3.

## What is and is not here

**Shipped (open):**

- The Python **reference runner** (`reference/`, package
  `a3_deploy_onnx_ref_pingpong`): builds the 110-D observation, runs the exported
  ONNX actor, consumes the planner's flat-wire commands, runs the swing lifecycle
  at 50 Hz, and drives the shipped MuJoCo model in-process.
- The **C++ runner sources** (`src/a3/a3_deploy_onnx_ref/`): AimRT-based, 110-D
  observation builder, fail-closed ONNX metadata loader, passive-neck handling,
  `--planner` mode consuming `std_msgs` flats, iceoryx body-drive, watchdog and
  safe-halt. Build support: `CMakeLists.txt` + `cmake/` modules,
  `setup_a3_env.sh` (fetches public ONNX Runtime and Unitree SDK2 releases), and `docker/`
  cross-build images (rockchip / thor).
- The shared **ActionAdapter** config and the **110-D runtime config** for the
  Python harness (`config/`), the tracked runtime assets (`assets/a3_runtime/` —
  reference serve clip + manifest), and the `joint_msgs` message sources the
  build depends on (`thirdparty/joint_msgs/`).
- Launch scripts (`scripts/`) for the sim harness and a documented real-hardware
  template.

Native regression tests in `src/a3/a3_deploy_onnx_ref/unit_tests/` are maintained
source, not robot runtime dependencies. Build the `run_tests` CMake target and
execute that binary directly; this target is not registered with CTest. Keep the
source checkout and checked-in CSV fixtures available when running it. Tests
cover normal mode transitions, terminal serve recovery and emergency precedence;
passing unit tests does not replace the hardware deployment checks above.

**Not shipped (you supply):**

- The AgiBot vendor runtime payload (SDK bundles, meshes, model binaries —
  multi-GB, vendor-gated). The vendor's own deploy example remains under
  `../../agibot/code_deployment/` with its build flow.

## The public contract (what any runner must satisfy)

| Piece | Spec |
| --- | --- |
| Observation | **110-D `hitter_pure`**, single layout, no normalization (raw). See `reference/.../observation.py` and `../../docs/POLICY_INTERFACE.md`. |
| Action | **31-D** `raw_action` → `q_des = default_q + raw_action * action_scale`, clamped to the official A3 joint limits. The two head columns (idx 3, 4) are passive: the runtime holds the neck at nominal. |
| ONNX | Use the exact model17400 policy/deploy pair, checked by native `--inspect-policy` during staging. The loader validates exported metadata and maps training/SDK joint order. |
| Joint order | 31-DOF Agibot A3, fixed. See `reference/.../joint_order.py`. |
| Command wire | `/racket/command_flat` (`std_msgs/Float64MultiArray`, schema 1 or 2: valid, swing_sign, position, velocity, timing, flight/revision identity) plus `/a3/base_pose_flat` for mocap base pose. Rich `hope_msgs/RacketCommand` remains for tooling. See `../../docs/PLANNER_INTERFACE.md`. |
| Lifecycle | `ready -> swing -> follow-through -> recovery -> ready`, one swing per flight, no state reset between balls. |
| Rate | 50 Hz. |

The 110-D observation, in order:
`base_ang_vel(3)`, `joint_pos(31, q-default_q)`, `joint_vel(31)`,
`actions(31, previous applied action)`, `projected_gravity(3)`,
`base_forward_xy(2)`, `base_target_delta_xy(2)`, `racket_target_rel_base(3)`,
`racket_target_vel_w(3)`, `time_to_strike(1)`.

There is no swing-side observation — the side travels on the wire as
`swing_sign` and is consumed by the runner's engage logic, never by the actor.

## Current deployment bundle

Build a complete target-matched Runner distribution, then stage the model17400
export without editing its metadata or deploy parameters:

```bash
python3 scripts/prepare_model17400_bundle.py \
  --package /path/to/model17400 \
  --runner-dist /path/to/target-runner-dist \
  --output /path/to/new-staging-directory
```

For cross staging, add `--inspection-runner-dist /path/to/native-runner-dist`
from the same source. The helper checks policy/deploy identities, inspects the
native contract, rejects mixed ELF architectures and emits the deployment,
Planner profile and HDU/MDU catalog entries. It does not install or start hardware.
See [the installation procedure](../../foxglove/extensions/hope-a3-console/README.md#model17400-receive-policy).

Deploy model17400 with PPQ004 on both Planner and packetizer and a live mocap
base stream. Its immutable session HOME and exported support parameters must
match the generated Planner profile. This bundle is receive-only; serving and
Kernel mode are not qualified. Source-tree simulation and regression fixtures
are not copied into the runtime distribution.

## PD Stand transition

The C++ runner enters PD_STAND over **5 seconds** by default. It smoothly
interpolates from the measured joint pose to the stand pose while increasing
Kp/Kd to the original standing gains. Entry joint velocity decays over 0.25 s
instead of being extrapolated across the full transition.

Set `--pd-stand-blend-sec S` to change this duration (finite and positive).
This setting is independent of `--motion-blend-sec` and
`--policy-entry-blend-sec`; the policy entry duration remains unchanged.

Normal `--planner` play stops after 100 ms without a fresh base pose, measured
on the MDU monotonic clock. RALLY uses the existing PD_STAND blend; SERVE requests
the existing abort/validated stand recovery. Fresh data does not restart play.
Manual Stand, emergency PASSIVE and offline `--kernel-mode` / `--serve-only`
operation remain available.

After 20 ms, normal-play model inputs explicitly hold the last applied pelvis
pose, without further gyro propagation or fake sensor/velocity updates. Original
timestamps and sequence remain unchanged; the hold ends at the rescue deadline.
HELD is separate from loss: mode entry and normal serve completion remain
available while the real sample is within its observation/rescue deadline.
New-flight commits still wait for a real, non-held pose. Qualify the hold's
effect on strike timing with replay and attended hardware tests.
The managed launch also enables `--operator-watchdog`: the Console heartbeat
expires after 1 s; the operator-computer relay expires after 100 ms. Either
requests controlled recovery. These are trigger times, not physical stand
transition durations. Offline
Kernel/Pure Serve and Teleop are not covered by this operator-link watchdog.
See [deployment checks](../../docs/operations/foxglove_lifecycle.md#player-assignment-and-mocap-loss).

## Building the C++ runner

Use the Ubuntu 24.04/ROS 2 Jazzy `hope` Distrobox created by
[`docs/DISTROBOX_SETUP.md`](../../docs/DISTROBOX_SETUP.md); its canonical
package list includes the CMake, compiler, ZeroMQ, Eigen, YAML, GTest and zlib
dependencies used here. Then:

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd "$HOME/workspace/HOPE/a3_deploy/a3_deploy_example"
source setup_a3_env.sh        # ROS 2 env + public ONNX Runtime and Unitree SDK2
cmake -S . -B build
cmake --build build --target a3_deploy_onnx_ref_pingpong -j4
```

`cmake/` provides the AimRT fetch/patch modules; `docker/` contains the
rockchip/thor cross-build images for the robot's motion unit. The runner binary
subscribes the flat topics in `--planner` mode and drives the robot (or the
AimRT MuJoCo sim) over the vendor body-drive interface — operational guidance in
`../../docs/RUN_ON_AGIBOT.md`, backend architecture in
[README_robot_io_backend.md](README_robot_io_backend.md).

## Configuration

- The staged deployment's `policy/params/deploy.yaml` and `policy/exported/policy.onnx`
  are the verified model17400 pair; never substitute parameters from another export.
- The generated `model17400.json` configures the HDU Planner from the same native
  inspection. Install the generated catalog entries on HDU and MDU together.
- Include the matching `config/a3_aimrt_config.pingpong_ros2body.yaml` for Runner
  state, HOME and mocap-safety telemetry over ROS 2.

## Running on real hardware

Complete simulation, bench, tether, e-stop, and low-gain verification before any
free-standing run. The vendor backend's gains, limits, and e-stop stay
authoritative; the public code never sets, probes, or bypasses them.
The hardware path is the C++ runner plus the licensed AgiBot vendor payload,
started through the [managed lifecycle](../../docs/operations/foxglove_lifecycle.md).

## License

Apache-2.0 (see the repository `LICENSE`). Copyright holder for the reference
runner: Intelligent Racing Inc. (dba Hitch Interactive). The MuJoCo sim under
`../A3_MuJoCo_Sim` carries its own Mulan PSL v2 license; vendor-derived sources
keep their own headers.
