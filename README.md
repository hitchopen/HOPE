# HOPE PingPong

HOPE PingPong is an open-source stack for training and deploying a **whole-body table-tennis
policy** on the [Agibot A3](https://www.zhiyuan-robot.com/) humanoid (31 actuated DOF). A single
feed-forward policy — shared by forehand and backhand — stands at a fixed station and returns a
continuous stream of incoming balls, choosing forehand or backhand per ball, using its legs only
to balance and recentre in place.

It is a **reference implementation**: the motions, rewards, action adapter, side selector, and
physics constants shipped here are clean, documented examples meant to be replaced with your own.

## What you can do

```
install dependencies
  -> train one unified forehand/backhand policy on the sample motions
  -> export an actor-only ONNX policy
  -> start the planner
  -> return a continuous stream of incoming balls in simulation
  -> forehand / backhand chosen automatically per ball
  -> keep a fixed station and rally continuously
  -> swap in your own motions, reward, side selector, or policy and iterate
```

The whole product is intentionally focused. It **includes**: one unified forehand+backhand policy;
one forehand and one backhand sample motion; automatic side selection; continuous multi-rally play
with no robot-state reset between balls; a fixed station and base heading; leg control for in-place
balance and recovery; a no-spin ball model; Isaac Lab + PPO training; planner, MuJoCo, and Agibot
run entry points; and a single public metric, `success_rate`.

It **excludes** (by design): station movement, footstep planning, locomotion, ball spin, motion
retiming/TOPP, opponent adaptation, shot strategy, and all internal shadow/gate/debug/replay,
failure-check, and checkpoint-promotion machinery.

## The one metric

`success_rate` is the only reported number. A ball counts once it enters the robot's strike task; a
return succeeds when the racket **actually contacts** the ball **and** the ball **crosses the net**
**and** its **first bounce lands on the opponent half**.

```
success_rate = successful_return_tasks / incoming_balls_that_entered_a_strike_task
```

Forehand, backhand, and all rally rounds merge into this one number. It is descriptive only — no
threshold, no best-checkpoint selection, no effect on exit codes or deployment.

## Repository layout

| Path | What |
|------|------|
| [`configs/ball_physics.yaml`](configs/ball_physics.yaml) | The single no-spin ball-physics config (real-data fit), read by training, planner, and eval. |
| [`hope_training/`](hope_training/) | Isaac Lab + PPO training package, the two sample motions, and the ball-physics fitting code. |
| [`hope_ws/`](hope_ws/) | ROS 2 workspace: `hope_msgs` (RacketCommand), `hope_planner`, `hope_bringup`, and the vendored `vrpn_mocap` driver. |
| [`agibot/`](agibot/) | Agibot A3 materials: deploy example, URDF/meshes, AimRT MuJoCo sim, and PKU open-source hardware. |
| [`a3_deploy/`](a3_deploy/) | The deploy side: the Mulan-licensed MuJoCo simulation and a clean-room reference runner. See note below. |
| [`mocap/`](mocap/) | The motion-capture frame and topic contract, plus the preserved arena design document (EN/ZH). |
| [`docs/`](docs/) | Interfaces and how-to guides (index below), plus [`docs/reference/`](docs/reference/) — preserved design documents and the competition rules. |

### Two deploy paths

The repository ships two ways to run the policy on the Agibot A3:

- [`agibot/`](agibot/) — the full Agibot A3 deploy example (deploy sources, URDF/meshes, the AimRT
  MuJoCo simulation, and the PKU open-source hardware). Component licensing is summarized in
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
- [`a3_deploy/`](a3_deploy/) — a lightweight reference runner (by Intelligent Racing
  Inc. dba Hitch Interactive) that implements the public 111-D observation / 31-D action /
  `RacketCommand` contract and drives the Mulan-licensed MuJoCo simulation. It is a minimal,
  dependency-light illustration of the deployment contract described in
  [docs/POLICY_INTERFACE.md](docs/POLICY_INTERFACE.md).

## Documentation

- [docs/QUICKSTART.md](docs/QUICKSTART.md) — end-to-end: install → train → export → evaluate → run in sim.
- [docs/TRAIN_POLICY.md](docs/TRAIN_POLICY.md) — training the unified forehand/backhand policy.
- [docs/REPLACE_MOTIONS.md](docs/REPLACE_MOTIONS.md) — the motion-clip format and how to use your own.
- [docs/POLICY_INTERFACE.md](docs/POLICY_INTERFACE.md) — the 111-D observation and 31-D action contract.
- [docs/PLANNER_INTERFACE.md](docs/PLANNER_INTERFACE.md) — the planner pipeline and `RacketCommand`.
- [docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md) — running in MuJoCo and integrating your vendor package.
- [docs/EXTENDING_HOPE_PINGPONG.md](docs/EXTENDING_HOPE_PINGPONG.md) — bring your own reward, adapter, side selector, motions, physics.
- [ROADMAP.md](ROADMAP.md) — what is shipped, what is out of scope by design, and what is next.

### Background and rules

[docs/reference/](docs/reference/) preserves the original HOPE system design
documents — the planner derivation, the WBC training plan, the deployment and
safety architecture, the mocap arena build — plus the **HOPE AI Challenge 2026
competition rules** (EN/ZH). That material predates this stack and is kept for
design context; where it disagrees with the code, the documents above win.
[docs/reference/REMOVED_FROM_STARTER.md](docs/reference/REMOVED_FROM_STARTER.md)
records what the focused rewrite dropped and what replaced it.

## Motions are reference examples only

The two clips under `hope_training/motions/preprocessed/` are short, physically-neutral placeholders
that make the pipeline run out of the box. **They are not performance-tuned.** Replace them with your
own retargeted forehand/backhand motions before training a policy you intend to deploy.

## Credits

The whole-body training package is built on [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking)
(MIT), an Isaac Lab motion-tracking framework, adapted here for table tennis; SMPL-X → robot
retargeting and video → SMPL-X extraction referenced in the docs use [GMR](https://github.com/YanjieZe/GMR)
(MIT) and GVHMR. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## License

Apache-2.0. Copyright 2025–2026 Intelligent Racing Inc. (dba Hitch Interactive). See
[LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for third-party components
(the vendored VRPN driver, the AimRT MuJoCo simulation, and the table USD asset).
