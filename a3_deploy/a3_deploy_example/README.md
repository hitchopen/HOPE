# a3_deploy / a3_deploy_example

**`a3_deploy/` is a revised fork of the official AgiBot A3 deploy code.** This
directory documents the public HOPE PingPong deploy contract and ships a
**clean-room reference runner** that implements it. It is not the vendor deploy
runner.

## What is and is not here

**Shipped (open):**

- A clean-room Python **reference runner** (`reference/`) authored from scratch
  against the public contract. It builds the 111-D observation, runs the exported
  ONNX actor, consumes `RacketCommand`, runs the swing lifecycle at 50 Hz, and
  drives the shipped MuJoCo sim.
- The shared **ActionAdapter** config and the clean **111-D runtime config**
  (`config/`).
- Launch scripts (`scripts/`) for the sim and a documented real-hardware template.

**Not shipped (you supply):**

- The proprietary AgiBot A3 C++ deploy runner and the vendor real-time backend.
  These are **not redistributed** here. Real-robot execution uses **your own
  licensed AgiBot vendor deploy package**.
- The exported policy binary (`hope_pingpong.onnx`) — see `models/README.md`.
  (The A3 URDF/meshes ship with the starter under `agibot/URDF/`; `../URDF/README.md`
  describes the optional user-supplied override location.)

## The public contract (what any runner must satisfy)

| Piece | Spec |
| --- | --- |
| Observation | **111-D**, single layout, no normalization (raw). See `reference/.../observation.py`. |
| Action | **31-D** `raw_action`; the passive head columns (idx 3, 4) are zeroed to form the **applied action**, which is fed back as next-tick `last_action` (matching training). |
| ONNX | `observation[1, 111] -> raw_action[1, 31]`, single output. |
| ActionAdapter | `q_des = default_q + raw_action * action_scale`, then a joint clamp. One shared config: `config/action_adapter.yaml`. |
| Joint order | 31-DOF Agibot A3, fixed. See `reference/.../joint_order.py`. |
| Command | `RacketCommand` (task_id / task_revision / swing_side / position / velocity / time_to_strike). |
| Lifecycle | `ready -> swing -> follow-through -> recovery -> ready`, one swing per `task_id`, no state reset between balls. |
| Rate | 50 Hz. |

The 111-D observation, in order:
`base_ang_vel(3)`, `joint_pos(31, q-default_q)`, `joint_vel(31)`,
`last_action(31)`, `projected_gravity(3)`, `base_forward_xy(2)`,
`fixed_station_error_xy(2)`, `racket_target_rel_base(3)`,
`racket_target_vel_w(3)`, `time_to_strike(1)`, `swing_side(1)`.

`swing_side` (+1 forehand / -1 backhand) is chosen once per `task_id` and locked.
See `../../docs/POLICY_INTERFACE.md` and `../../docs/PLANNER_INTERFACE.md` for the
observation and command details.

## Quickstart (MuJoCo sim)

```bash
pip install -r reference/requirements.txt          # numpy pyyaml onnxruntime mujoco
# put your exported policy at models/hope_pingpong.onnx (or pass --onnx)
scripts/run_pingpong_sim.sh --view --realtime      # windowed, wall-clock 50 Hz
scripts/run_pingpong_sim.sh --duration 20          # headless, 20 s
```

The reference runner loads the same `a3_pingpong` MJCF that the AimRT MuJoCo sim
wraps (`../A3_MuJoCo_Sim`) and steps MuJoCo in-process, so you can watch the policy
drive the robot without the AimRT/iceoryx stack. See `reference/README.md` for the
module layout, the design, and the integration seams.

## Configuration

- `config/action_adapter.yaml` — the **shared** ActionAdapter (also read by the
  training package). Neutral example `default_q`, uniform `action_scale`, and
  example joint clamp limits. **Tune for your robot.**
- `config/hope_pingpong_runtime.yaml` — the clean 111-D runtime config: control
  rate, `observation_normalization: none`, ONNX path, ActionAdapter path, MuJoCo
  model path, and **example** simulation PD gains (used only to drive the sim's
  torque actuators — not vendor deploy gains).

## Running on real hardware

Real-robot execution is **not** performed by anything in this repository. Bring up
your own licensed AgiBot vendor deploy package and its safety systems (motor
protection, hard limits, e-stop), then wire the public contract above into it:
build the 111-D observation, run `hope_pingpong.onnx`, zero the passive head columns
to form the applied action (also the next `last_action`), map it through the shared
ActionAdapter to 31 joint targets, and command the vendor backend at 50 Hz.
The vendor backend's gains, limits, and e-stop stay authoritative; the public code
never sets, probes, or bypasses them.

`scripts/run_pingpong_real.sh` is a **documented, non-executing template** for that
handoff. See also `../../docs/RUN_ON_AGIBOT.md`.

## License

Apache-2.0 (see the repository `LICENSE`). Copyright holder for the reference
runner: Intelligent Racing Inc. (dba Hitch Interactive). The MuJoCo sim under
`../A3_MuJoCo_Sim` carries its own Mulan PSL v2 license.
