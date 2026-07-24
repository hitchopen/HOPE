# g1_deploy / g1_deploy_example

The **Unitree G1** twin of `a3_deploy/a3_deploy_example/`. It documents the public HOPE deploy
contract for the G1 (29 DOF, no head) and ships a **clean-room reference runner** that implements
it. It is not a vendor deploy runner. See [../../docs/RUN_ON_G1.md](../../docs/RUN_ON_G1.md) for the
full train → export → deploy loop.

The G1 is structurally the A3 minus its two passive neck joints, so this package is the A3 reference
runner with the robot-specific pieces swapped: a 29-DOF `joint_order.py`, a 105-D observation
(`18 + 3*29`), name-keyed PD groups (the G1 canonical order interleaves the waist chain), and
`passive_neck: false`. Everything else (lifecycle, ONNX wrapper, ROS command source, quaternion
math, sim bridge) is shared logic.

## What is and is not here

**Shipped (open):**

- A clean-room Python **reference runner** (`reference/g1_deploy_onnx_ref_pingpong/`): builds the
  105-D observation, runs the exported ONNX actor, consumes `RacketCommand`, runs the swing
  lifecycle at 50 Hz.
- The shared **ActionAdapter** config (also read by training) and the clean **105-D runtime config**
  (`config/`).
- Launch scripts (`scripts/`) for the sim and a documented real-hardware template.

**Not shipped (you supply):**

- The exported policy binary (`hope_pingpong_g1.onnx`) — see `models/README.md`.
- A **G1 MuJoCo MJCF** with a racket collision geom + racket site (plus a `pelvis` free-joint and a
  gyro sensor for the sim bridge). This is the one deferred follow-up — the in-process MuJoCo sim
  path is not runnable until you provide `--model-xml` (adapt one from
  `TTRL-ICRA2026/Beyondmimic_Deploy_G1/mjmodel.xml`).
- Any vendor real-time backend + safety systems (motor protection, hard limits, e-stop).

## The public contract (what any runner must satisfy)

| Piece | Spec |
| --- | --- |
| Observation | **105-D**, single layout, no normalization (raw). See `reference/.../observation.py`. |
| Action | **29-D** `raw_action`; **no passive columns** (the G1 has no neck), so the applied action equals the raw action. |
| ONNX | `observation[1, 105] -> raw_action[1, 29]`, single output. |
| ActionAdapter | `q_des = default_q + raw_action * action_scale`, then a joint clamp. One shared config: `config/action_adapter.yaml`. |
| Joint order | 29-DOF Unitree G1, fixed. See `reference/.../joint_order.py` and `docs/interfaces/joint_order.md`. |
| Command | `RacketCommand` (task_id / task_revision / swing_side / position / velocity / time_to_strike). |
| Lifecycle | `ready -> swing -> follow-through -> recovery -> ready`, one swing per `task_id`, no state reset between balls. |
| Rate | 50 Hz. |

The 105-D observation, in order:
`base_ang_vel(3)`, `joint_pos(29, q-default_q)`, `joint_vel(29)`, `last_action(29)`,
`projected_gravity(3)`, `base_forward_xy(2)`, `fixed_station_error_xy(2)`,
`racket_target_rel_base(3)`, `racket_target_vel_w(3)`, `time_to_strike(1)`, `swing_side(1)`.

## Quickstart (MuJoCo sim)

```bash
pip install -r reference/requirements.txt          # numpy pyyaml onnxruntime mujoco
# put your exported policy at models/hope_pingpong_g1.onnx (or pass --onnx)
# a G1 MJCF is not shipped yet -> pass your own with --model-xml
scripts/run_pingpong_sim.sh --view --realtime --model-xml /path/to/g1_pingpong.xml
```

## Configuration

- `config/action_adapter.yaml` — the **shared** ActionAdapter (also read by training via
  `load_g1_action_adapter_config`). G1 ready-stance `default_q`, uniform `action_scale`, and the
  URDF joint clamp. **Tune for your robot.** Parity with training is asserted by
  `hope_training/whole_body_tracking/tests/test_g1_contract.py`.
- `config/hope_pingpong_runtime.yaml` — the clean 105-D runtime config: control rate,
  `observation_normalization: none`, ONNX path, ActionAdapter path, MuJoCo model path (TODO), and
  **example** simulation PD gains (name-keyed groups; used only to drive the sim's torque actuators).

## License

Apache-2.0 (see the repository `LICENSE`). Copyright holder for the reference runner: Intelligent
Racing Inc. (dba Hitch Interactive).
