# Roadmap

Scope and direction for HOPE. For the ledger of what was dropped, restored,
and deliberately withheld, see
[`docs/REMOVED_FROM_STARTER.md`](docs/REMOVED_FROM_STARTER.md).

## Shipped

- The **hardware-exercised 110-D rally line**: one unified forehand/backhand
  `hitter_pure` policy for the Agibot A3 (31 actuated DOF), automatic per-ball
  side selection in the planner, continuous multi-rally play, and the
  `HitterPingPong` task — the recipe validated on real A3 hardware
  (`docs/TRAIN_POLICY.md`).
- A station-anchored base (x-locked hit plane, in-place recentring/footwork
  toward a commanded station); no free locomotion or gait planning.
- A no-spin ball model **fitted from real venue data**
  ([`configs/ball_physics.yaml`](configs/ball_physics.yaml),
  [`configs/ball_physics_venue.yaml`](configs/ball_physics_venue.yaml),
  [`configs/incoming_ball_venue.yaml`](configs/incoming_ball_venue.yaml)),
  shared by training, planner, and eval, plus the fitting pipeline
  (`hope_training/ball_physics_fit/`).
- Isaac Lab + PPO training with named actor-observation contracts, actor-only
  ONNX export with fail-closed deploy metadata, and the layered evaluation
  story: Isaac evaluation, MuJoCo sim-to-sim, and closed-loop rehearsal
  ([`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md#evaluation)).
- **Dual planners** publishing one wire contract: Python
  [`hope_planner`](hope_ws/src/hope_planner) (reference + presets +
  `planner_imitate`) and C++
  [`hope_planner_cpp`](hope_ws/src/hope_planner_cpp) (hardware line).
- The **native C++ deploy runner** `a3_pingpong` under
  [`a3_deploy/`](a3_deploy) — CMake build with rockchip/thor cross-build
  images, `--planner` flat-topic mode, iceoryx body-drive — alongside the
  Python reference harness for MuJoCo rehearsal, plus Agibot's own example
  under [`agibot/code_deployment/`](agibot/code_deployment).
- Independent raw-mocap workspaces ([`NatNet2ROS2/`](NatNet2ROS2) and
  [`VRPN2ROS2/`](VRPN2ROS2)), and the ROS 2 planner/relay workspace
  ([`hope_ws/`](hope_ws)) with the `table_p1_to_p2_v1` world-frame contract and
  fail-closed calibration receipts.

## Out of scope, by design

Free locomotion and gait planning (the base is station-anchored), motion
retiming/TOPP, opponent adaptation, and shot strategy. Spin-aware **planning**
is out of scope: the C++ planner carries only a diagnostics-only spin *shadow*
estimator, and the published command stays no-spin. The shipped placeholder
motions, reward recipes, and physics constants are documented and meant to be
replaced or re-fitted — see
[`docs/EXTENDING_HOPE_PINGPONG.md`](docs/EXTENDING_HOPE_PINGPONG.md).

## Not shipped / next

- **Real motion clips.** The proven line trained on the *v12fix*-generation
  forehand/backhand clips, which are not shipped; the committed clips are
  schema-valid placeholders. Bring your own via
  [`docs/REPLACE_MOTIONS.md`](docs/REPLACE_MOTIONS.md) (converters and
  validators are included).
- **Trained checkpoints / ONNX weights.** The exporters, parity checks, and
  fail-closed loaders ship; the weights do not. Reproducing a deploy-grade
  policy from a clean clone requires your GPU time and your clips.
- **Spin-aware planning** beyond the shadow estimator — promoting spin from a
  diagnostics channel into the published racket command.
- **Non-A3 robots.** The G1/SMPL scaffolding exists again in the tracking
  stack, but their assets are not distributed and the contracts are sized to
  the A3's 31 DOF; a second robot means revisiting the observation/action
  contracts and deploy metadata.
- **CI** for the non-Isaac checks, and optional GPU smoke jobs.
