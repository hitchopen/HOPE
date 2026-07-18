# Extending HOPE PingPong

Everything shipped here — motions, reward, action adapter, side selector, physics, policy — is a
clean, documented example meant to be replaced. This page lists the extension points and the exact
files behind each.

## Bring your own motions

Replace the two reference clips with your own forehand/backhand motions. The `.npz` + `.yaml` schema
and the CLI to point training at them are in [REPLACE_MOTIONS.md](REPLACE_MOTIONS.md). Nothing scores
or gates the clips — the loader just reads the format.

## Bring your own reward

The example reward is eleven simple terms with illustrative weights:

- Term functions: `hope_training/.../tasks/tracking/mdp/hope_rewards.py`
- Weights/wiring: the `RewardsCfg` in `hope_training/.../tasks/tracking/config/agibot_a3/hope_env_cfg.py`

Edit the weights, add or remove terms, or write new term functions. The `success_rate` metric
([TRAIN_POLICY.md](TRAIN_POLICY.md#evaluation)) is independent of the reward, so you can reshape the
reward freely without changing how success is measured.

## Bring your own action adapter

The policy emits a 31-D raw action; the **ActionAdapter** turns it into 31 joint-position targets.
Training and the deploy runner read the **same** config so they stay in sync:

- Config (shared): `a3_deploy/a3_deploy_example/config/action_adapter.yaml`
  (`default_q`, `action_scale`, clamp limits — the shipped values are neutral examples, **tune them
  for your robot**).
- Training side: `hope_training/.../tasks/tracking/mdp/hope_actions.py`
  (`ClampedJointPositionAction`).
- Deploy side: the `action_adapter` module in
  `a3_deploy/a3_deploy_example/reference/a3_deploy_onnx_ref_pingpong/`.

Keep the two sides reading the same file so a change applies everywhere. See
[POLICY_INTERFACE.md](POLICY_INTERFACE.md#actionadapter).

## Bring your own side selector

Forehand/backhand is chosen by the planner from where the ball crosses the fixed strike plane:

- Parameter: `swing_side_split_y` (and `swing_side_hysteresis_y`) in
  `hope_ws/src/hope_planner/config/hope_planner.yaml`.
- Logic: `hope_ws/src/hope_planner/hope_planner/side_selection.py` (pure function; the node
  delegates to it). Convention: `crossing_y < split -> FOREHAND`, at/above -> BACKHAND.

Replace the simple lateral split with your own rule. The chosen side is published as the formal
`swing_side` field of `RacketCommand` ([PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)) and fed into the
policy observation — keep both in agreement.

## Bring your own ball physics

The no-spin ball model is a single config fit from real data:

- Config: `configs/ball_physics.yaml` (read by training, planner, and eval).
- Fitting code + data format: `hope_training/ball_physics_fit/` (with a small sample capture).

Re-fit the constants from your own captured trajectories and copy the results into
`configs/ball_physics.yaml`; all three consumers pick them up. The model stays no-spin —
`[x, y, z, vx, vy, vz]` only.

## Bring your own policy

Any actor that honors the contract — `observation[111] -> raw_action[31]`, 50 Hz, no observation
normalization ([POLICY_INTERFACE.md](POLICY_INTERFACE.md)) — is a drop-in. Retrain with your changes
and re-export, or supply an ONNX exported elsewhere; the runner and evaluators only depend on the
contract, not on how the policy was produced.

## Bring your own robot

The stack targets the Agibot A3 (31 DOF). The joint order
(`hope_training/config/joint_order_agibot_a3.yaml`), the robot articulation config
(`hope_training/.../robots/agibot_a3.py`), and the URDF/asset prep
([RUN_ON_AGIBOT.md](RUN_ON_AGIBOT.md)) are the places a different robot would diverge; the observation
and action dimensions are sized to 31 DOF throughout, so a different robot means revisiting the
contract dimensions as well.
