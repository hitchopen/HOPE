# Training the policy

The training package (`hope_training/whole_body_tracking/`) is an Isaac Lab extension that trains one
feed-forward actor — shared by forehand and backhand — with PPO and a privileged critic. This page
covers the design; exact commands and the package layout are in the
[package README](../hope_training/whole_body_tracking/README.md).

## The task

There is a single Gym task, `HOPE-PingPong-AgibotA3-v0` (`task=HOPEPingPong`), defined by
`HOPEPingPongEnvCfg`:

- **Two motion clips** — clip 0 forehand, clip 1 backhand — imitated by the upper body. A new clip
  (side) is chosen per swing, so all four adjacent transitions (FH→FH, FH→BH, BH→FH, BH→BH) appear
  across a batch. The lower body is free to balance and recentre.
- **A racket-target goal** (`RacketTargetCommand`): a sampled racket target position, target
  velocity, and time-to-strike, plus the `swing_side`, all fed to the actor.
- **A fixed station**: a startup-constant base target. The observation carries
  `fixed_station_error_xy` (station minus current base XY) so the policy learns to recentre in place
  as it drifts over a rally. The station never moves — there is no station or footstep planning.
- **Continuous rallies**: `wrap_teleport = false`. Robot state, joint state, and `last_action` carry
  across swings; the environment only resets on the fixed episode timeout or a physical fall (an
  ordinary lifecycle event, not a gate).
- **50 Hz** control (decimation 4 over a 200 Hz physics step).

The actor sees the 111-D observation ([POLICY_INTERFACE.md](POLICY_INTERFACE.md)); the critic
additionally sees privileged, simulation-only signals (the reference joint stream, motion-anchor
errors, and the true racket state) for its value estimate. Those never enter the deployed policy.

## Reward terms

The example reward is a simple sum of eleven terms with **illustrative weights** — tune them:

1. upright / balance
2. forehand-or-backhand sample imitation (upper body, gated to the swing)
3. racket position (at the strike)
4. racket velocity (at the strike)
5. simplified blade direction (at the strike)
6. ball contact
7. net crossing
8. opponent-half first bounce
9. in-place follow-through / recovery
10. action smoothness
11. joint-limit regularization

No private weights, windows, deadbands, curricula, or ablations are shipped. See
`tasks/tracking/mdp/hope_rewards.py` and [EXTENDING_HOPE_PINGPONG.md](EXTENDING_HOPE_PINGPONG.md).

## Evaluation

`success_rate` is the only metric, and it is defined as an **actual physical return**: the racket
contacts the ball, the ball crosses the net, and its first bounce lands on the opponent half.

- `scripts/mujoco_eval_onnx.py` is the **authoritative** evaluator: it runs the exported ONNX against
  a real MuJoCo ball that physically bounces off the racket, table, and net, and reports
  `success_rate` from that simulated trajectory. Use this number. By default it evaluates a
  **continuous rally** (robot/policy state persist across serves, serve pattern FH, FH, BH, BH, …
  so all four adjacent side transitions are exercised); pass `--eval-mode independent` for the
  isolated per-serve variant (resets the robot each serve; does not validate transitions).
- `scripts/evaluate.py` is a **fast in-Isaac estimate**: during training the reward terms above shape
  contact/net/bounce with a no-spin *analytic* outgoing-ball model (the racket's strike velocity is
  rolled out ballistically), and `evaluate.py` reports that same analytic estimate. It is a cheap
  proxy for iterating in the training environment — trust `mujoco_eval_onnx.py` for the physical
  number.

If you want the training rewards themselves to use fully simulated contact, the repository ships a
complete PhysX ball/table/net scene in `tasks/table_tennis/` (Gym id `HOPE-TableTennis-AgibotA3-v0`)
that you can compose into the training environment; this is a supported extension point rather than
the default, so that the default trains without the extra simulation cost.

Both evaluators print only `{"success_rate": <float>}`. There is no threshold, best-checkpoint
selection, early stop, or exit-code effect.

## Checkpoints and logging

Checkpoints are written locally: a `periodic` checkpoint every `save_interval` iterations and a
`final` at the end. There are no `best`/`candidate`/`accepted`/`promoted` checkpoints, no Weights &
Biases / TensorBoard, no reward-term or per-side logging — only the single `success_rate` when you
run an evaluator.

## Configuration

- `cfg/task/HOPEPingPong.yaml` — the task: motion clip paths, `wrap_teleport: false`, episode length,
  and an `overrides:` block for dotted-path env tweaks.
- `cfg/algo/ppo.yaml` — PPO (`empirical_normalization: false`, i.e. raw observations), iteration and
  save intervals.
- `cfg/base/*.yaml` — shared env / sim / randomization defaults.

Detailed reward and racket-target values live in `HOPEPingPongEnvCfg` (kept in code, not the YAML).

## The user runs training

Training needs your GPU and your real motion clips; run it yourself with the commands in the
[package README](../hope_training/whole_body_tracking/README.md#train). Launch from the repository
root so the relative motion paths resolve.
