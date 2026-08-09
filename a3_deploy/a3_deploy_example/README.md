# a3_deploy / a3_deploy_example

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
  `setup_a3_env.sh` (fetches a public onnxruntime release), and `docker/`
  cross-build images (rockchip / thor).
- The shared **ActionAdapter** config and the **110-D runtime config** for the
  Python harness (`config/`), the tracked runtime assets (`assets/a3_runtime/` —
  reference serve clip + manifest), and the `joint_msgs` message sources the
  build depends on (`thirdparty/joint_msgs/`).
- The published **model_21800** bundle (`models/model_21800/policy/`): the exact
  deploy ONNX plus its Unitree-style `deploy.yaml`.
- Launch scripts (`scripts/`) for the sim harness and a documented real-hardware
  template.

**Not shipped (you supply):**

- The AgiBot vendor runtime payload (SDK bundles, meshes, model binaries —
  multi-GB, vendor-gated). The vendor's own deploy example remains under
  `../../agibot/code_deployment/` with its build flow.

## The public contract (what any runner must satisfy)

| Piece | Spec |
| --- | --- |
| Observation | **110-D `hitter_pure`**, single layout, no normalization (raw). See `reference/.../observation.py` and `../../docs/POLICY_INTERFACE.md`. |
| Action | **31-D** `raw_action` → `q_des = default_q + raw_action * action_scale`, clamped to the official A3 joint limits. The two head columns (idx 3, 4) are passive: the runtime holds the neck at nominal. |
| ONNX | Compact exports use `observation[1,110] -> raw_action[1,31]`; model_21800 uses `obs[1,110]` plus a reference `time_step[1,1]` and exposes `actions[1,31]` plus debug outputs. The wrapper consumes only the actor output and maps training/SDK joint order. |
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

## Quickstart (MuJoCo sim, Python harness)

```bash
pip install -r reference/requirements.txt          # numpy pyyaml onnxruntime mujoco
scripts/run_pingpong_sim.sh --view --realtime      # windowed, wall-clock 50 Hz
scripts/run_pingpong_sim.sh --duration 20          # headless, 20 s
```

The reference runner loads the same `a3_pingpong` MJCF that the AimRT MuJoCo sim
wraps (`../A3_MuJoCo_Sim`) and steps MuJoCo in-process, so you can watch the
policy drive the robot without the AimRT/iceoryx stack. With the ROS 2 planner
running (`hope_ws`), `python -m a3_deploy_onnx_ref_pingpong --planner` consumes
the live `/racket/command_flat` stream. See `reference/README.md` for the module
layout and integration seams.

## Building the C++ runner

```bash
source setup_a3_env.sh        # ROS 2 env + fetches onnxruntime into thirdparty/
cmake -S . -B build && cmake --build build -j
```

`cmake/` provides the AimRT fetch/patch modules; `docker/` contains the
rockchip/thor cross-build images for the robot's motion unit. The runner binary
subscribes the flat topics in `--planner` mode and drives the robot (or the
AimRT MuJoCo sim) over the vendor body-drive interface — operational guidance in
`../../docs/RUN_ON_AGIBOT.md`, backend architecture in
[README_robot_io_backend.md](README_robot_io_backend.md).

## Configuration

- `models/model_21800/policy/params/deploy.yaml` — the published model's action
  offsets/scales/clips, joint ordering, timing and PD arrays. The neutral
  `config/action_adapter.yaml` remains available for custom examples.
- `config/hope_pingpong_runtime.yaml` — the 110-D runtime config for the Python
  harness. It selects model_21800 by default and uses the exported PD arrays only
  to realize its targets in simulation; it never configures a robot backend.

## Running on real hardware

Complete simulation, bench, tether, e-stop, and low-gain verification before any
free-standing run. The vendor backend's gains, limits, and e-stop stay
authoritative; the public code never sets, probes, or bypasses them.
`scripts/run_pingpong_real.sh` documents the Python-harness handoff template;
the hardware path is the C++ runner plus your licensed AgiBot vendor payload —
see `../../docs/RUN_ON_AGIBOT.md`.

## License

Apache-2.0 (see the repository `LICENSE`). Copyright holder for the reference
runner: Intelligent Racing Inc. (dba Hitch Interactive). The MuJoCo sim under
`../A3_MuJoCo_Sim` carries its own Mulan PSL v2 license; vendor-derived sources
keep their own headers.
