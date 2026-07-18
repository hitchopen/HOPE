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

The bundled motion clips under `hope_training/motions/preprocessed/` are
**reference-only placeholders** — replace them with real forehand/backhand clips
([docs/REPLACE_MOTIONS.md](docs/REPLACE_MOTIONS.md)) before training a policy you
intend to deploy. See [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) for the full
install → train → export → evaluate → run loop, and
[docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md) for the deploy path.

## Preserved Reference Documents

| Document | Description | Version |
|----------|-------------|---------|
| [Motion Capture System Reference Setup](mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md) | OptiTrack/ROS 2 arena configuration, coordinate frames, tracked object taxonomy, humanoid base_link marker setup, ball tracking, and streaming pipeline | v0.4 |
| [7DOF Racket Model-based Planner Reference Setup](HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md) | Ball state estimation, trajectory prediction, and racket target planning (Stages 1–3 of the HITTER framework), reimplemented in the HOPE canonical frame | v0.1 |
| [WBC Simulation Training Reference Setup](HOPE_WBC_Simulation_Training_Reference_Setup.md) | SMPL-X motion acquisition, GMR retargeting, BeyondMimic RL training pipeline for whole-body control (Stage 4), with dual-backend support for Isaac Lab and mjlab | v0.5 |
| [Hardware Deployment Reference Setup](HOPE_Hardware_Deployment_Reference_Setup.md) | Real-robot deployment via `legged_control2` (G1) or AimRT (A3): ONNX inference, ROS 2 node graph, PD gain tuning, safety procedures, and competition workflow | v0.2 |

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
| `docs/` | Task guides ([TRAIN_POLICY](docs/TRAIN_POLICY.md), [REPLACE_MOTIONS](docs/REPLACE_MOTIONS.md), [RUN_ON_AGIBOT](docs/RUN_ON_AGIBOT.md), [EXTENDING_HOPE_PINGPONG](docs/EXTENDING_HOPE_PINGPONG.md), [REMOVED_FROM_STARTER](docs/REMOVED_FROM_STARTER.md)) and the public contracts ([POLICY_INTERFACE](docs/POLICY_INTERFACE.md), [PLANNER_INTERFACE](docs/PLANNER_INTERFACE.md), `docs/interfaces/`). |
| [A3_ASSETS.md](A3_ASSETS.md) | Asset map for the racket-equipped A3 URDF, the generated Isaac copy, joint order, and the Agibot-provided A3 reference materials. |
| [REFERENCE_DOCS.md](REFERENCE_DOCS.md) | Index of preserved architecture, rules, mocap, training, and deployment reference documents. |
| [ROADMAP.md](ROADMAP.md) | What is shipped, what is out of scope by design, and what comes next. |
| `HOPE_*_Reference_Setup.md` | Preserved system design documents (planner / WBC training / hardware deployment). |
| `HOPE_AI_Challenge_2026_Rules_EN.docx`, `..._ZH.docx` | Challenge rulebooks (English / 中文). |
| `configs/` | The shared no-spin ball model ([ball_physics.yaml](configs/ball_physics.yaml)) used by training, planner, and eval. |
| `hope_training/` | The Isaac Lab training extension (`whole_body_tracking/` with task cfg, train/export/eval scripts), placeholder motion clips (`motions/preprocessed/`), the canonical A3 joint order (`config/joint_order_agibot_a3.yaml`), and the ball-physics fitting tools (`ball_physics_fit/`). |
| `hope_ws/` | ROS 2 workspace: `hope_planner` (no-spin planner), `hope_bringup` (launch files, `pose_to_posearray` adapter, fake-ball publisher), `hope_msgs` (`RacketCommand.msg`), and the vendored `vrpn_mocap` driver. |
| `a3_deploy/` | Public deploy contract and clean-room reference runner (`a3_deploy_example/`), the MuJoCo/AimRT simulation fork (`A3_MuJoCo_Sim/`), and the optional user-supplied URDF override location (`URDF/`). |
| `agibot/` | Agibot-provided A3 bundle: the racket-equipped source URDF (`URDF/A3T2.5-URDF-std-pingpang/`), the vendor deploy example (`code_deployment/`), the MuJoCo/AimRT simulation reference (`A3_MuJoCo_Sim/`), and mounting hardware models (`pku/`). |
| `mocap/` | Motion-capture frame/topic contract ([mocap/README.md](mocap/README.md)) and the preserved mocap reference documents (EN/ZH). |

## System Architecture

```
                    ┌─────────────────────────────┐
                    │  Motion Capture Cameras      │
                    │  (OptiTrack / ChingMu)       │
                    └──────────┬──────────────────┘
                               │ VRPN
                               ▼
                    ┌─────────────────────────────┐
                    │  vrpn_mocap (ROS 2)          │
                    │  per-tracker PoseStamped     │
                    │  /vrpn_mocap/<t>/pose_id_<N> │
                    └──────────┬──────────────────┘
                               │
                               ▼
                    ┌─────────────────────────────┐
                    │  pose_to_posearray           │
                    │  → /poses (PoseArray)        │
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
                    │   • base_link pose (mocap)   │
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

The same policy runner drives either the shipped MuJoCo simulation
(`a3_deploy/a3_deploy_example/scripts/run_pingpong_sim.sh`) or, via your own
licensed Agibot vendor deploy package, the real robot
([docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md)).

## Key Design Decisions

**Racket tracking is prohibited.** The motion capture system tracks exactly three categories of objects: the ping-pong table origin frame (PPT), each humanoid's `base_link` (P1, P2), and the ball. No reflective markers may be placed on the racket, the robot's hand, or the wrist link. Each robot must infer its paddle's 6-DOF pose through forward kinematics from its own `base_link` + joint encoders. This is a deliberate competition constraint that tests autonomous paddle control through the robot's internal body model.

**One robot, end to end.** The preserved reference design documents discuss both Unitree G1 and Agibot Expedition A3 paths. The shipped stack targets the A3 (31 actuated DOF) end to end: Isaac Lab training of one unified forehand/backhand policy (`HOPE-PingPong-AgibotA3-v0`), MuJoCo evaluation with real ball physics, and a clean-room deploy reference runner, alongside Agibot's own deploy example and MuJoCo/AimRT simulation reference.

**Open-source training stack.** The WBC training pipeline is built entirely on open-source code: [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) (MIT license), from which the `hope_training/whole_body_tracking/` extension derives, [GMR](https://github.com/YanjieZe/GMR) (MIT license) for SMPL-X to robot retargeting, and [GVHMR](https://github.com/zju3dv/GVHMR) for monocular video-to-SMPL-X extraction. The HITTER paper's trained weights are not released; all training starts from scratch, and the bundled motion clips are placeholders to be replaced with your own retargeted swings ([docs/REPLACE_MOTIONS.md](docs/REPLACE_MOTIONS.md)).

## Supported Robots

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
- **Real arena**: OptiTrack Motive or a compatible VRPN motion-capture system

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
