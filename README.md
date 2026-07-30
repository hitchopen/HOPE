# HOPE: Hitch Open Ping-Pong Embodied AI Challenge

HOPE is an open platform for humanoid robot table tennis, developed by [Hitch Interactive](https://hitchinteractive.com) (Intelligent Racing Inc.) in collaboration with the [ROAR Platform](https://roar.berkeley.edu) at UC Berkeley. The challenge invites teams to deploy whole-body humanoid controllers that can rally a ping-pong ball against human opponents or other robots, using off-the-shelf humanoid hardware and an open-source perception and planning stack.

This repository contains the full HOPE stack for the Agibot A3: Isaac Lab training for a **unified forehand/backhand whole-body policy**, a **no-spin planner** (ROS 2), a **MuJoCo evaluator** with real ball physics, and an **A3 deploy reference runner** — plus the preserved HOPE **reference design documents**, the **challenge rulebooks**, and the Agibot-provided A3 starter materials under `agibot/`.

## How To Read This Repository

Start with this README, then follow [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md).
The rest of the repository is organized into four layers:

| Layer | What to read or run | Purpose |
|-------|---------------------|---------|
| Required path | `QUICKSTART_A3_ISAAC.md`, `hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py`, `agibot/URDF/A3T2.5-URDF-std-pingpang/`, `hope_training/whole_body_tracking/` | Prepare the A3 Isaac asset, train the unified forehand/backhand policy (`task=HOPEPingPong`), export `hope_pingpong.onnx`, and evaluate `success_rate`. |
| Stable public contracts | `A3_ASSETS.md`, `docs/interfaces/`, `docs/POLICY_INTERFACE.md`, `docs/PLANNER_INTERFACE.md` | Frame conventions, joint order, the 111-D observation / 31-D action policy IO, ROS topics, `RacketCommand`, and asset expectations that stay stable when you integrate your own code. |
| Deploy and simulation references | `a3_deploy/`, `agibot/` | The public deploy contract and clean-room reference runner (`a3_deploy/a3_deploy_example/`), the MuJoCo/AimRT simulation fork, and the Agibot-provided A3 materials (URDF variants, vendor deploy example). |
| Background material | `hope_ws/`, `mocap/`, root `HOPE_*_Reference_Setup.md`, `REFERENCE_DOCS.md`, `ROADMAP.md` | The ROS 2 mocap/planner workspace for arena integration, the mocap frame/topic docs, the preserved design documents, and current scope/direction. |

A fresh clone contains only tracked files. Generated Isaac assets, training logs,
checkpoints, exported `.onnx` files, and ROS build artifacts are git-ignored and
regenerated locally; Agibot-provided materials under `agibot/` are tracked.

## Quickstart

```bash
# 1. Prepare the A3 Isaac asset (bundled racket-equipped URDF, from the repo root)
python3 hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py --force

# 2. Train the unified forehand/backhand policy (inside your Isaac Lab Python environment)
cd hope_training/whole_body_tracking
source setup_train_env.sh        # defines the `isaac_py` launcher for the Isaac Sim Python
python scripts/train.py task=HOPEPingPong algo=ppo headless=true

# 3. Export the deployable actor -> <run>/exported/hope_pingpong.onnx
python scripts/export_onnx.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt

# 4. Evaluate (real MuJoCo ball, continuous rally) -> {"success_rate": <float>}
python scripts/mujoco_eval_onnx.py --onnx logs/rsl_rl/hope_pingpong/<run>/exported/hope_pingpong.onnx

# 5. Watch the deploy reference runner drive the robot in MuJoCo
cp logs/rsl_rl/hope_pingpong/<run>/exported/hope_pingpong.onnx ../../a3_deploy/a3_deploy_example/models/
cd ../../a3_deploy/a3_deploy_example
bash scripts/run_pingpong_sim.sh
```

To close the loop with the planner (ROS 2):

```bash
cd hope_ws && colcon build && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true   # mocap-free smoke test

# in another terminal (same ROS env sourced):
cd a3_deploy/a3_deploy_example/reference
python -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

### Required mocap bringup: align P1 to A3 `pelvis_link`

Before using a newly installed A3 pelvis marker shell, perform the one-time P1
calibration and save its constant 6-DOF `P1 → pelvis_link` transform to a
persistent JSON file. The mocap system supplies the dynamic `world → P1`
pose from the P1 rigid-body markers. Calibration additionally requires an
**independent full-6DOF** `geometry_msgs/PoseStamped` measurement of
`world → pelvis_link`, for example on `/a3/calibration/pelvis_pose`. The
calibrator synchronizes those two pose topics and estimates their constant
relative transform; it does not consume a `Table` topic or TF.

The checked-in A3 hardware bridge publishes `/body_drive/pelvis_imu/data`, but
an IMU does not provide absolute pelvis translation. It therefore is **not** a
producer for `/a3/calibration/pelvis_pose`. Before real-robot calibration, the
A3 integration must bridge an independent external 6-DOF tracker or state
estimator to that topic with `header.frame_id: world` and timestamps in the
same clock domain as `/P1/pose`. The MuJoCo-only
`/sim/a3/pelvis_pose` publisher may be used for simulation validation after
both inputs have been expressed in its `odom` frame; it is not a hardware
producer. Do not run `p1_pelvis_tf_publisher` while collecting calibration,
and never construct the reference pose from `P1`, because either would make
the measurement circular.

Collect while moving the pelvis smoothly through at least 0.10 m translation
and 10 degrees rotation for at least one second. Residual RMS alone only tests
repeatability; the calibrator separately gates measured motion excitation,
timestamp uniqueness, and pair skew so a stationary capture cannot pass.
OptiTrack uses latency-compensated local-ROS acquisition timestamps; both
sources must use the same acquisition-time epoch.

```bash
cd hope_ws && colcon build --packages-select hope_bringup && source install/setup.bash
ros2 run hope_bringup p1_pelvis_calibrator \
  --p1-topic /P1/pose \
  --pelvis-topic /a3/calibration/pelvis_pose \
  --reference-frame world \
  --pelvis-frame pelvis_link \
  --p1-frame P1 \
  --output calibration/p1_to_pelvis.json
```

At normal runtime, load that JSON to publish the constant `P1 → pelvis_link` TF:

```bash
ros2 run hope_bringup p1_pelvis_tf_publisher \
  --calibration-file calibration/p1_to_pelvis.json
```

The resulting chain is `world → P1 → pelvis_link`; the first transform remains
the live mocap measurement, while only the second is static. An optional
alternative is to absorb the saved correction into Motive's P1 pivot
definition. If that is done, disable the static publisher and verify by
rerunning calibration that the correction is approximately identity. Never
apply both corrections. The `Table` asset may be enabled separately for arena
setup when needed; this tool does not use it, and competition must not stream it.
See [mocap/README.md](mocap/README.md#calibrating-a-humanoid-p1-body-to-pelvis_link)
and [docs/OPTITRACK.md](docs/OPTITRACK.md#calibrating-p1-to-an-a3-pelvis_link).

The bundled motion clips under `hope_training/motions/preprocessed/` are
**reference-only placeholders** — replace them with real forehand/backhand clips
([docs/REPLACE_MOTIONS.md](docs/REPLACE_MOTIONS.md)) before training a policy you
intend to deploy. See [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) for the full
install → train → export → evaluate → run loop, and
[docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md) for the deploy path.

## Preserved Reference Documents

| Document | Description | Version |
|----------|-------------|---------|
| [Motion Capture System Reference Setup](mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md) | OptiTrack/NatNet and Chingmu/VRPN ROS 2 arena configuration, coordinate frames, competition rigid bodies (`Ball`, `P1`, `P2`; `Table` for calibration only), robot root-frame registration, and streaming pipelines | v0.7 |
| [7DOF Racket Model-based Planner Reference Setup](HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md) | Ball state estimation, trajectory prediction, and racket target planning (Stages 1–3 of the HITTER framework), reimplemented in the HOPE canonical frame | v0.1 |
| [WBC Simulation Training Reference Setup](HOPE_WBC_Simulation_Training_Reference_Setup.md) | SMPL-X motion acquisition, GMR retargeting, BeyondMimic RL training pipeline for whole-body control (Stage 4), with dual-backend support for Isaac Lab and mjlab | v0.5 |
| [Hardware Deployment Reference Setup](HOPE_Hardware_Deployment_Reference_Setup.md) | Platform-specific real-robot deployment paths (including `legged_control2` and AimRT): ONNX inference, ROS 2 node graph, PD gain tuning, safety procedures, and competition workflow | v0.2 |

The three documents at the repository root each contain a **Section 0 prologue**
listing implementation differences from the original HITTER work (see
References); the mocap pair documents its differences inline. Where a preserved
document disagrees with the shipped code, the code and the `docs/` contracts win —
[REFERENCE_DOCS.md](REFERENCE_DOCS.md) is the index.

The competition rulebooks ship at the repository root:
[HOPE_AI_Challenge_2026_Rules_EN.docx](HOPE_AI_Challenge_2026_Rules_EN.docx) (English) and
[HOPE_AI_Challenge_2026_Rules_ZH.docx](HOPE_AI_Challenge_2026_Rules_ZH.docx) (中文).

## Folder Map

| Path | Purpose |
|------|---------|
| [README.md](README.md) | Repository orientation and the shortest train → export → evaluate → run commands. New teams should start here. |
| [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) | Step-by-step fresh-clone A3/Isaac setup. |
| `docs/` | Task guides ([TRAIN_POLICY](docs/TRAIN_POLICY.md), [REPLACE_MOTIONS](docs/REPLACE_MOTIONS.md), [RUN_ON_AGIBOT](docs/RUN_ON_AGIBOT.md), [OPTITRACK](docs/OPTITRACK.md), [EXTENDING_HOPE_PINGPONG](docs/EXTENDING_HOPE_PINGPONG.md), [REMOVED_FROM_STARTER](docs/REMOVED_FROM_STARTER.md)) and the public contracts ([POLICY_INTERFACE](docs/POLICY_INTERFACE.md), [PLANNER_INTERFACE](docs/PLANNER_INTERFACE.md), `docs/interfaces/`). |
| [A3_ASSETS.md](A3_ASSETS.md) | Asset map for the racket-equipped A3 URDF, the generated Isaac copy, joint order, and the Agibot-provided A3 reference materials. |
| [REFERENCE_DOCS.md](REFERENCE_DOCS.md) | Index of preserved architecture, rules, mocap, training, and deployment reference documents. |
| [ROADMAP.md](ROADMAP.md) | What is shipped, what is out of scope by design, and what comes next. |
| `HOPE_*_Reference_Setup.md` | Preserved system design documents (planner / WBC training / hardware deployment). |
| `HOPE_AI_Challenge_2026_Rules_EN.docx`, `..._ZH.docx` | Challenge rulebooks (English / 中文). |
| `configs/` | The shared no-spin ball model ([ball_physics.yaml](configs/ball_physics.yaml)) used by training, planner, and eval. |
| `hope_training/` | The Isaac Lab training extension (`whole_body_tracking/` with task cfg, train/export/eval scripts), placeholder motion clips (`motions/preprocessed/`), the canonical A3 joint order (`config/joint_order_agibot_a3.yaml`), and the ball-physics fitting tools (`ball_physics_fit/`). |
| `hope_ws/` | ROS 2 workspace: `hope_planner` (no-spin planner), `hope_bringup` (launch files, `pose_to_posearray` / `optitrack_mct_relay` adapters, fake-ball publishers), `hope_msgs` (`RacketCommand.msg`), and the vendored mocap drivers (`vrpn_mocap` for VRPN rigs, `motion_capture_tracking` for OptiTrack/NatNet). |
| `a3_deploy/` | Public deploy contract and clean-room reference runner (`a3_deploy_example/`), the MuJoCo/AimRT simulation fork (`A3_MuJoCo_Sim/`), and the optional user-supplied URDF override location (`URDF/`). |
| `agibot/` | Agibot-provided A3 bundle: the racket-equipped source URDF (`URDF/A3T2.5-URDF-std-pingpang/`), the vendor deploy example (`code_deployment/`), the MuJoCo/AimRT simulation reference (`A3_MuJoCo_Sim/`), and mounting hardware models (`pku/`). |
| `mocap/` | Motion-capture frame/topic contract ([mocap/README.md](mocap/README.md)) and the preserved mocap reference documents (EN/ZH). |

## System Architecture

```
       ┌──────────────────────────────┐   ┌──────────────────────────────┐
       │ OptiTrack Motive             │   │ Chingmu CMTracker / MCServer │
       │ NatNet UDP                   │   │ VRPN server                  │
       │ Ball / P1 / P2               │   │ Ball / P1 / P2               │
       └──────────────┬───────────────┘   └──────────────┬───────────────┘
                      │ NatNet                            │ VRPN
                      ▼                                   ▼
       ┌──────────────────────────────┐   ┌──────────────────────────────┐
       │ motion_capture_tracking      │   │ vendored vrpn_mocap          │
       │ /optitrack/poses             │   │ /vrpn_mocap/<name>/          │
       │   (NamedPoseArray)           │   │   pose_id_<N> (PoseStamped)  │
       └──────────────┬───────────────┘   └──────────────┬───────────────┘
                      │ optitrack_mct_relay              │ pose_to_posearray
                      └──────────────────┬────────────────┘
                                         ▼
                    ┌─────────────────────────────┐
                    │ /poses: full rigid-body pose │
                    │ xyz + quaternion (xyzw)      │
                    └──────────┬──────────────────┘
                               │
                               ▼
                    ┌─────────────────────────────┐
                    │  hope_planner (Stages 1–3)   │
                    │                              │
                    │  Ball state estimation       │
                    │  → no-spin trajectory        │
                    │    prediction                │
                    │  → racket target planning    │
                    │    (side, position, velocity,│
                    │     time_to_strike)          │
                    └──────────┬──────────────────┘
                               │ /racket/command
                               │ (hope_msgs/RacketCommand)
                               ▼
                    ┌─────────────────────────────┐
                    │  Policy runner (Stage 4)     │
                    │                              │
                    │  hope_pingpong.onnx @ 50 Hz  │
                    │  111-D obs → 31-D action     │
                    │                              │
                    │  Receives:                   │
                    │   • RacketCommand            │
                    │   • robot root pose          │
                    │   • Joint encoders           │
                    │                              │
                    │  Outputs:                    │
                    │   • Robot joint              │
                    │     position cmds            │
                    └──────────┬──────────────────┘
                               │
                               ▼
                    ┌─────────────────────────────┐
                    │  Agibot A3 humanoid          │
                    │                              │
                    │  PD controller               │
                    │  → joint torques             │
                    └─────────────────────────────┘
```

During competition the motion-capture stream carries the named rigid bodies `Ball`, `P1`, and
`P2` (`Ball` is always first in `/poses`; the default VRPN bringup aggregates only the ball); a
`Table` asset is used for calibration only and appears only in training-data recordings. The ball pose includes position `(x, y, z)` and quaternion orientation
`(qx, qy, qz, qw)`; pitch/yaw/roll are derived display values. The shipped planner currently
uses only the ball position, while the full pose is preserved for validation and future
spin-aware estimation.

> The diagram shows both supported source paths: **Chingmu/VRPN** uses
> `vrpn_mocap` + `pose_to_posearray` (the default `mocap_backend:=vrpn`),
> while **OptiTrack/Motive uses NatNet** through the vendored
> `motion_capture_tracking` driver plus `optitrack_mct_relay`
> (`mocap_backend:=optitrack`). Both feed the identical `/poses` contract, so
> everything below that hop is unchanged. See
> [docs/OPTITRACK.md](docs/OPTITRACK.md).

The same policy runner drives either the shipped MuJoCo simulation
(`a3_deploy/a3_deploy_example/scripts/run_pingpong_sim.sh`) or, via your own
licensed Agibot vendor deploy package, the real robot
([docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md)).

## Key Design Decisions

**Racket tracking is prohibited.** During competition the motion capture system streams the named rigid bodies `Ball`, `P1`, and `P2` — the ball and the two humanoid marker-cluster frames. A calibrated static transform maps each P1/P2 frame to that robot's declared URDF root frame (`pelvis` on Unitree G1; `pelvis_link` on Agibot A3). The table is a calibrated static world origin (a `Table` asset is used during setup only and appears only in training-data recordings). The ball stream contains position `(x, y, z)` and orientation, represented in ROS 2 as a quaternion `(qx, qy, qz, qw)`. Pitch, yaw, and roll are derived views, not fields carried by `geometry_msgs/Pose`. No reflective markers may be placed on the racket, the robot's hand, or the wrist link. Each robot must infer its paddle's 6-DOF pose through forward kinematics from its declared root frame plus joint encoders. This is a deliberate competition constraint that tests autonomous paddle control through the robot's internal body model.

**Implementation scope.** The preserved reference documents include robot-specific integration examples; the code currently shipped in this repository implements the Agibot A3 (31 actuated DOF) path end to end: Isaac Lab training of one unified forehand/backhand policy (`HOPE-PingPong-AgibotA3-v0`), MuJoCo evaluation with real ball physics, and a clean-room deploy reference runner, alongside Agibot's own deploy example and MuJoCo/AimRT simulation reference.

**Open-source training stack.** The WBC training pipeline is built entirely on open-source code: [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) (MIT license), from which the `hope_training/whole_body_tracking/` extension derives, [GMR](https://github.com/YanjieZe/GMR) (MIT license) for SMPL-X to robot retargeting, and [GVHMR](https://github.com/zju3dv/GVHMR) for monocular video-to-SMPL-X extraction. The HITTER paper's trained weights are not released; all training starts from scratch, and the bundled motion clips are placeholders to be replaced with your own retargeted swings ([docs/REPLACE_MOTIONS.md](docs/REPLACE_MOTIONS.md)).

## Supported Robots

The table below reports current repository implementation coverage.

| Robot | DOF | Status |
|-------|-----|--------|
| Agibot Expedition A3 | 31 actuated | Shipped end-to-end path: Isaac Lab training, ONNX export, MuJoCo evaluation, deploy reference runner, plus the Agibot vendor deploy example. |
| Unitree G1 | 29 | Discussed in the preserved reference design documents (`HOPE_*_Reference_Setup.md`) only; no shipped code path. |

## Coordinate Frame Convention

All contracts share a common world frame (ROS 2 REP 103):

- **Origin**: Near-side left corner of the table surface
- **X**: Toward opponent (along the 2.74 m table length)
- **Y**: Left (along the 1.525 m table width)
- **Z**: Up
- **Table surface height**: 0.76 m above floor

The motion-capture system must stream Z-up poses to match this convention (in
OptiTrack Motive: **Up Axis → Z**). See [docs/interfaces/frames.md](docs/interfaces/frames.md)
and [mocap/README.md](mocap/README.md).

## Prerequisites

Each piece has its own dependencies — install only what the step you are on needs:

- **Training / export**: NVIDIA Isaac Sim + Isaac Lab (with `rsl_rl`), Python 3.10, PyTorch, CUDA GPU
- **MuJoCo evaluation / reference runner**: `mujoco`, `onnxruntime`, `numpy` (no GPU needed)
- **Planner workspace**: ROS 2 Jazzy (`rclpy`), `numpy`, `pyyaml`
- **Real arena**: OptiTrack Motive streams **NatNet** through the vendored `motion_capture_tracking` driver and `optitrack_mct_relay` (`mocap_backend:=optitrack`; set Motive **Up Axis = Z** and see [docs/OPTITRACK.md](docs/OPTITRACK.md)). Chingmu CMTracker/MCServer streams **VRPN** through `vrpn_mocap` and `pose_to_posearray` (`mocap_backend:=vrpn`). In either case configure the named 6-DOF competition rigid bodies `Ball`, `P1`, and `P2`; `Table` is calibration-only and is not streamed during competition.

## Related Repositories

| Repository | Purpose |
|-----------|---------|
| [HybridRobotics/whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking) | BeyondMimic training code — upstream of `hope_training/whole_body_tracking/` |
| [YanjieZe/GMR](https://github.com/YanjieZe/GMR) | General Motion Retargeting (SMPL-X → robot) |
| [zju3dv/GVHMR](https://github.com/zju3dv/GVHMR) | Video-to-SMPL-X pose estimation |
| [AimRT/aimrt](https://github.com/AimRT/aimrt) | Agibot's lightweight robotics middleware, used by the A3 MuJoCo sim and vendor deploy example |
| [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) | MuJoCo — physics for the evaluator and the reference runner |

The preserved reference design documents cite additional G1-era repositories
(`legged_control2`, `motion_tracking_controller`, mjlab); see
[REFERENCE_DOCS.md](REFERENCE_DOCS.md).

## References

- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*. [Project page](https://humanoid-table-tennis.github.io/)
- SMASH: Mastering Scalable Whole-Body Skills for Humanoid Ping-Pong with Egocentric Vision (University of Hong Kong). *arXiv:2604.01158*. [Paper](https://arxiv.org/abs/2604.01158)
- Hu, M., Chen, W., Li, W., Mandali, F., He, Z., Zhang, R., Krisna, P., Christian, K., Benaharon, L., Ma, D., et al. (2025). PACE: Physics Augmentation for Coordinated End-to-end Reinforcement Learning toward Versatile Humanoid Table Tennis (Purdue TRACE Lab, ICRA 2026). *arXiv:2509.21690*. [Code](https://github.com/purdue-tracelab/PACE-ICRA2026)
- Liao, Q., et al. (2025). BeyondMimic: From Motion Tracking to Versatile Humanoid Control via Guided Diffusion. *arXiv:2508.08241v4*. [Project page](https://beyondmimic.github.io/)
- Araújo, J. P., Ze, Y., Xu, P., Wu, J., & Liu, C. K. (2025). Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking. *arXiv:2510.02252*.
- Ze, Y., et al. (2025). LATENT: Learning Athletic Humanoid Tennis Skills from Imperfect Human Motion Data. *arXiv:2603.12686*.
- mjlab: A Lightweight Framework for GPU-Accelerated Robot Learning. *arXiv:2601.22074*.
- Peng, X. B., et al. (2024). SMPLOlympics: Sports Environments for Physically Simulated Humanoids. *arXiv:2407.00187*.

## Technical Sponsors

The HOPE open platform is developed with the support of our technical sponsors, whose humanoid and motion-capture hardware make the reference design possible:

- **AgiBot (Zhiyuan Robotics)** — humanoid robot platforms (Expedition A3 and related hardware). [agibot.com](https://www.agibot.com)
- **ChingMu (青瞳视觉)** — CHINGMU optical motion-capture systems (CMTracker / CMAvatar). [chingmu.com](https://www.chingmu.com)
- **OptiTrack — Leyard (NaturalPoint, Inc., a Leyard company)** — OptiTrack optical motion-capture cameras and Motive software. [optitrack.com](https://www.optitrack.com)

## License

This project is licensed under the [Apache License, Version 2.0](LICENSE). See [LICENSE](LICENSE) for the full terms.

Some starter materials are derived from or interoperate with third-party software and robot assets. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for provenance and per-directory license notes.

## Contact

**Allen Yang**, Co-founder and CTO, Hitch Interactive (Intelligent Racing Inc.); Chair, AI Racing ROAR Platform, UC Berkeley; Founding Executive Director, VIVE AR Center, UC Berkeley

**Development team:** Franco Huang (lead), Jeremy Wei, Yikang Yu, and Jiayi Zhu.
