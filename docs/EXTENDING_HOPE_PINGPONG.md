# Extending HOPE

Everything shipped here — motions, task recipe, reward, observation contract, side selection,
physics, policy — is documented and meant to be replaced or extended. This page lists the extension
points and the exact files behind each.

## Bring your own motions

Replace the two placeholder clips with your own forehand/backhand motions. The `.npz` + `.yaml`
schema and the CLI to point training at them are in
[REPLACE_MOTIONS.md](REPLACE_MOTIONS.md).

## Compose your own task

Tasks are Hydra YAMLs under `hope_training/whole_body_tracking/cfg/task/`, layered with a
`defaults:` chain — the shipped `HOPEPingPong.yaml` composes the shared `cfg/base/*` defaults
and states the full recipe. To experiment, add a new YAML that inherits `HOPEPingPong` (start
it with `defaults: [HOPEPingPong, _self_]`) and override fields (reward weights, DR ranges,
question-bank mix, `gym_task` id). **The task YAML overrides the Python env-cfg** — when a
value exists in both, edit the YAML. The base dataclasses live in
`hope_training/.../tasks/tracking/config/agibot_a3/hope_env_cfg.py`.

## Bring your own observation contract

Actor observation layouts are a registry, not scattered constants:
`hope_training/.../tasks/tracking/actor_observation_contract.py` defines the named contracts
(`full`=180, `deploy_parity`=175, `hitter_footwork`=177, `hitter_pure`=110, `hitter_pure_v15`=118)
term by term, with each term's deploy-side data source. A task selects one via its
`actor_obs_contract` key, the exporter stamps the contract name into the ONNX metadata, and the
deploy loader fails closed on a mismatch — so a new contract means: add it to the registry, build
its terms in `tasks/tracking/mdp/hope_observations.py`, reference it from your task YAML, and teach
the runner to assemble it. See [POLICY_INTERFACE.md](POLICY_INTERFACE.md).

## Bring your own reward

- Term functions: `hope_training/.../tasks/tracking/mdp/hope_rewards.py`
- Weights/wiring: the task YAML (see above) on top of the `RewardsCfg` in
  `hope_training/.../tasks/tracking/config/agibot_a3/hope_env_cfg.py`

Edit weights, add or remove terms, or write new term functions. The evaluation story
([TRAIN_POLICY.md](TRAIN_POLICY.md#evaluation) — Isaac evaluation, MuJoCo sim-to-sim,
closed-loop rehearsal) is independent of the reward, so you can reshape the reward freely without
changing how success is measured.

## Bring your own action realization

The policy emits a 31-D raw action, realized as `q_des = default_q + raw_action * action_scale`
plus a deterministic joint clamp:

- Training side: `hope_training/.../tasks/tracking/mdp/hope_actions.py`
  (`ClampedJointPositionAction` and the V11+ safe-clamp variants).
- Deploy side: the constants (default pose, scale, clamps, PD gains) are **exported with the
  policy** — `policy.deploy.json` / `params/deploy.yaml` plus fail-closed ONNX metadata — and the
  C++ runner validates them at load. Change the training values and re-export; do not hand-edit
  the deploy side.

See [POLICY_INTERFACE.md](POLICY_INTERFACE.md#action-31-dims).

## Bring your own side selector

Forehand/backhand is chosen **inside the planner** from where the predicted ball crosses the hit
plane, with hysteresis so alternating rallies don't flap:

- Parameters: `swing_side_split_y` and `swing_side_hysteresis_y` (declared in
  `hope_ws/src/hope_planner/hope_planner/node.py`; tuned per preset in
  `hope_ws/src/hope_planner/config/hope_planner*.yaml` and mirrored in the C++ planner's
  `hope_ws/src/hope_planner_cpp/config/model21800_hardware.yaml`).
- Logic: `_select_swing_sign` in the Python planner node; the C++ planner implements the same rule.

Replace the lateral split with your own rule. The chosen side travels on the wire as `swing_sign`
in `/racket/command_flat` ([PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)) and drives the runner's
engage machine — the policy observation itself has **no side term**, so no training change is
needed to alter side selection.

## Bring your own ball physics

The no-spin ball model is a small set of configs fitted from real data:

- `configs/ball_physics.yaml` — the generic model (read by training, planner, and eval).
- `configs/ball_physics_venue.yaml` — the real venue fit (e.g. `drag_k = 0.1261`).
- `configs/incoming_ball_venue.yaml` — the measured serve envelope for that venue.
- Fitting code: `hope_training/ball_physics_fit/` (stage 1/2 fits plus `falsify/` checks). It fits
  your own captured trajectory CSVs — capture with `hope_bag_to_csv`, then re-fit and copy the
  constants into the configs; all consumers pick them up.

The published command stays no-spin — `[x, y, z, vx, vy, vz]` only (the C++ planner's optional
spin *shadow* estimator is diagnostics-only).

## Bring your own policy

Any actor that honors the contract — `observation[110] -> raw_action[31]`, 50 Hz, no observation
normalization ([POLICY_INTERFACE.md](POLICY_INTERFACE.md)) — is a drop-in, **provided the ONNX
carries the fail-closed deploy metadata** the loader checks. Retrain with your changes and
re-export with the shipped exporters; the runner and evaluators depend only on the contract, not on
how the policy was produced.

## Bring your own robot

The stack targets the Agibot A3 (31 DOF). The joint order
(`hope_training/config/joint_order_agibot_a3.yaml`), the robot articulation config
(`hope_training/.../robots/agibot_a3.py`), and the URDF/asset prep
([RUN_ON_AGIBOT.md](RUN_ON_AGIBOT.md)) are the places a different robot would diverge; the
observation contracts and action dimensions are sized to 31 DOF throughout, so a different robot
means revisiting the contract dimensions (and the deploy metadata) as well.
