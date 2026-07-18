# HOPE: Hitch Open Ping-Pong Embodied AI Challenge

HOPE is an open platform for humanoid robot table tennis, developed by [Hitch Interactive](https://hitchinteractive.com) (Intelligent Racing Inc.) in collaboration with the [ROAR Platform](https://roar.berkeley.edu) at UC Berkeley. The challenge invites teams to deploy whole-body humanoid controllers that can rally a ping-pong ball against human opponents or other robots, using off-the-shelf humanoid hardware and an open-source perception and planning stack.

This repository contains the **reference design documents** for the HOPE system architecture, a public **Agibot A3 + Isaac Lab starter**, and Agibot-provided A3 reference materials under `agibot/`. The starter is intended to get new teams from clone to asset load, table-tennis scene smoke test, and local-motion PPO smoke training.

## How To Read This Repository

Start with this README, then follow [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md).
That is the only required path for the v1 public starter. The rest of the
repository is organized into four layers:

| Layer | What to read or run | Purpose |
|-------|---------------------|---------|
| Required starter path | `QUICKSTART_A3_ISAAC.md`, `scripts/prepare_a3_isaac_asset.py`, `agibot/URDF/A3T2.5-URDF-std-pingpang/`, `hope_training/whole_body_tracking/` | Prepare the A3 Isaac asset, import the task package, launch the table-tennis scene, and run local-motion PPO smoke training. |
| Stable public contracts | `A3_ASSETS.md`, `docs/interfaces/` | Explain frame conventions, joint order, observations/actions, ROS topics, and asset expectations that other teams should keep stable when integrating their own code. |
| Agibot A3 reference bundle | `agibot/` | Agibot-provided A3 URDF variants, MuJoCo/AimRT simulation reference, and deployment example. Only the racket-equipped URDF is required for the Isaac quickstart. |
| Optional or background material | `hope_ws/`, `mocap/`, `HOPE_*_Reference_Setup.md`, `ROADMAP.md` | Preserve broader HOPE architecture, ROS/mocap/planner context, and future work. These are not required before the Isaac smoke run. |

A fresh clone contains only tracked files. Developer-local folders such as
`external_repos/`, `vendor_assets/`, generated Isaac assets, logs, checkpoints,
and WandB caches are git-ignored. Agibot-provided materials under `agibot/` are
tracked. If you switch branches in an existing checkout and still see an ignored
local folder, that usually means it is an untracked folder left on disk; it is
not part of the tracked public starter contents.

## Public Starter Quickstart

Start here:

```bash
python3 scripts/prepare_a3_isaac_asset.py --force
cd hope_training/whole_body_tracking
source setup_train_env.sh
hope_isaac_py -c "import whole_body_tracking.tasks; print('HOPE tasks import ok')"
hope_isaac_py scripts/play_table_tennis.py --headless --steps 300
hope_isaac_py scripts/create_smoke_motion.py --headless --frames 120 \
  --output sample_motions/agibot_a3_smoke_stand.npz
hope_isaac_py scripts/train.py task=TrackingFlat algo=ppo \
  headless=true logger=tensorboard \
  motion_file=sample_motions/agibot_a3_smoke_stand.npz \
  num_envs=32 max_iterations=3
```

See [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) for the full path. WandB is optional; the public smoke path uses a local `motion_file` and TensorBoard.

## Preserved Reference Documents

| Document | Description | Version |
|----------|-------------|---------|
| [Motion Capture System Reference Setup](mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md) | OptiTrack/ROS 2 arena configuration, coordinate frames, tracked object taxonomy, humanoid base_link marker setup, ball tracking, and streaming pipeline | v0.3 |
| [7DOF Racket Model-based Planner Reference Setup](HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md) | Ball state estimation, trajectory prediction, and racket target planning (Stages 1–3 of the HITTER framework), reimplemented in the HOPE canonical frame | v0.1 |
| [WBC Simulation Training Reference Setup](HOPE_WBC_Simulation_Training_Reference_Setup.md) | SMPL-X motion acquisition, GMR retargeting, BeyondMimic RL training pipeline for whole-body control (Stage 4), with dual-backend support for Isaac Lab and mjlab | v0.5 |
| [Hardware Deployment Reference Setup](HOPE_Hardware_Deployment_Reference_Setup.md) | Real-robot deployment via `legged_control2` (G1) or AimRT (A3): ONNX inference, ROS 2 node graph, PD gain tuning, safety procedures, and competition workflow | v0.1 |

Each document contains a **Section 0 prologue** listing all implementation differences from the original HITTER work (see References).

## Folder Map

| Path | Purpose |
|------|---------|
| [README.md](README.md) | Repository orientation and shortest public smoke-run commands. New teams should start here. |
| [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) | Step-by-step fresh-clone A3/Isaac setup. This is the v1 success path. |
| [A3_ASSETS.md](A3_ASSETS.md) | Asset map for the racket-equipped A3 URDF, generated Isaac copy, joint order, and Agibot-provided A3 reference materials. |
| [REFERENCE_DOCS.md](REFERENCE_DOCS.md) | Index of preserved architecture, rules, mocap, training, and deployment reference documents. |
| [ROADMAP.md](ROADMAP.md) | Current starter scope, optional integrations, and future work. |
| `scripts/` | Small setup and smoke-run helpers. The main public entry is `prepare_a3_isaac_asset.py`. |
| `agibot/` | Public Agibot A3 reference bundle. `agibot/URDF/A3T2.5-URDF-std-pingpang/` is the racket-equipped variant required for Isaac; `agibot/A3_MuJoCo_Sim/` contains the Agibot MuJoCo/AimRT simulation reference; `agibot/code_deployment/` contains the Agibot A3 deploy example. |
| `agibot/code_deployment/` | Optional Agibot A3 deployment example for connecting exported policies to A3 body-drive state/command topics. Not required for Isaac smoke training. |
| `hope_training/config/` | Shared robot configuration, including the public A3 joint order. |
| `hope_training/whole_body_tracking/` | Isaac Lab whole-body tracking starter, table-tennis scene, PPO entry points, and smoke-motion generator. |
| `docs/interfaces/` | Compact contracts for frames, joint order, policy IO, and ROS topics. These are the public interface docs, not an internal gate system. |
| `hope_ws/` | Optional ROS 2 workspace skeleton for future mocap/planner integration. Not required for the Isaac quickstart. |
| `mocap/` | Preserved motion-capture reference docs and assets. Useful when building a real arena pipeline. |
| `external_repos/`, `vendor_assets/`, generated assets/logs/checkpoints | Local-only ignored folders. They may exist in a developer checkout but are not tracked starter contents. |

## System Architecture

```
                    ┌─────────────────────────────┐
                    │     OptiTrack Cameras        │
                    │     (9×, 360 Hz)             │
                    └──────────┬──────────────────┘
                               │ NatNet
                               ▼
                    ┌─────────────────────────────┐
                    │  motion_capture_tracking     │
                    │  (ROS 2 Jazzy)               │
                    │                              │
                    │  Publishes:                  │
                    │   • Ball position (3D)       │
                    │   • P1/P2 base_link pose     │
                    │   • Table origin frame       │
                    └───┬──────────────┬──────────┘
                        │              │
                        ▼              ▼
              ┌──────────────┐  ┌──────────────────┐
              │  HOPE Planner │  │  Whole-Body       │
              │  (Stages 1-3) │  │  Controller       │
              │               │  │  (Stage 4)        │
              │  Ball state   │  │                    │
              │  estimation   │  │  BeyondMimic RL    │
              │  → trajectory │  │  policy (50 Hz)    │
              │  prediction   │──│                    │
              │  → racket     │  │  Receives:         │
              │  target       │  │   • Racket target  │
              │  planning     │  │   • base_link pose │
              └──────────────┘  │   • Joint encoders  │
                                │                    │
                                │  Outputs:          │
                                │   • Robot joint    │
                                │     position cmds  │
                                └────────┬───────────┘
                                         │
                                         ▼
                                ┌──────────────────┐
                                │  Humanoid Robot   │
                                │  (Unitree G1 /    │
                                │   Agibot A3)      │
                                │                   │
                                │  PD controller    │
                                │  → joint torques  │
                                └──────────────────┘
```

## Key Design Decisions

**Racket tracking is prohibited.** The motion capture system tracks exactly three categories of objects: the ping-pong table origin frame (PPT), each humanoid's `base_link` (P1, P2), and the ball. No reflective markers may be placed on the racket, the robot's hand, or the wrist link. Each robot must infer its paddle's 6-DOF pose through forward kinematics from its own `base_link` + joint encoders. This is a deliberate competition constraint that tests autonomous paddle control through the robot's internal body model.

**Multi-platform support.** The reference design documents discuss Unitree G1 and Agibot Expedition A3 paths. This public starter currently focuses on Agibot A3 in Isaac Lab for setup and smoke training. It also includes Agibot's A3 deployment and MuJoCo/AimRT reference materials for teams that want to study the body-drive interface or optional simulation path after exporting policies.

**Open-source training stack.** The WBC training pipeline is built entirely on open-source code: [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) (MIT license) for motion tracking RL, [GMR](https://github.com/YanjieZe/GMR) (MIT license) for SMPL-X to robot retargeting, and [GVHMR](https://github.com/zju3dv/GVHMR) for monocular video-to-SMPL-X extraction. The HITTER paper's trained weights are not released; all training starts from scratch.

## Supported Robots

| Robot | DOF | Simulation Backend | Model Format | Status |
|-------|-----|-------------------|--------------|--------|
| Unitree G1 | 29 | Isaac Lab + PhysX | USD | Reference platform |
| Unitree G1 EDU | 29 + hands | Isaac Lab + PhysX | USD | Supported (hand DOFs unused) |
| Agibot Expedition A3 | 31 active joints in starter | Isaac Lab + PhysX smoke path; Agibot MuJoCo/AimRT reference included | URDF / MuJoCo reference assets | Public starter smoke path; optional deploy example included |

## Coordinate Frame Convention

All three documents share a common world frame (ROS 2 REP 103):

- **Origin**: Near-side left corner of the table surface
- **X**: Toward opponent (along the 2.74 m table length)
- **Y**: Left (along the 1.525 m table width)
- **Z**: Up
- **Table surface height**: 0.76 m above floor

The OptiTrack system must be configured with **Up Axis → Z** in Motive to match this convention.

## Prerequisites

The reference documents assume familiarity with:

- ROS 2 Jazzy
- Python 3.10+
- NVIDIA Isaac Lab 2.1.0 for the public starter
- OptiTrack Motive (or compatible motion capture system)
- PyTorch
- TensorBoard for the default public smoke run
- Weights & Biases (WandB), optional for registry-based motion loading or cloud logging

## Related Repositories

| Repository | Purpose |
|-----------|---------|
| [HybridRobotics/whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking) | BeyondMimic training code (Isaac Lab) |
| [mujocolab/mjlab](https://github.com/mujocolab/mjlab) | BeyondMimic training code (MuJoCo Warp) |
| [HybridRobotics/motion_tracking_controller](https://github.com/HybridRobotics/motion_tracking_controller) | ROS 2 deployment (ONNX inference) |
| [qiayuanl/legged_control2](https://qiayuanl.github.io/legged_control2_doc/) | Low-level controller framework for legged robots |
| [qiayuanl/unitree_bringup](https://github.com/qiayuanl/unitree_bringup) | Unitree robot bringup utilities |
| [unitreerobotics/unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) | Unitree official mjlab integration |
| [YanjieZe/GMR](https://github.com/YanjieZe/GMR) | General Motion Retargeting (SMPL-X → robot) |
| [zju3dv/GVHMR](https://github.com/zju3dv/GVHMR) | Video-to-SMPL-X pose estimation |
| [IMRCLab/motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking) | ROS 2 motion capture bridge |
| [google-deepmind/mujoco_warp](https://github.com/google-deepmind/mujoco_warp) | GPU-accelerated MuJoCo |
| [AimRT/aimrt](https://github.com/AimRT/aimrt) | Agibot's lightweight robotics middleware |

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
