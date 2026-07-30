# Reference Material

Preserved HOPE background: the original system design documents and the
competition rules. **This is background, not the current contract.**

These documents predate the focused HOPE stack and describe the
broader HOPE program — the full G1/A3 architecture, the mocap arena build, the
planner derivation, and the deployment/safety rationale. They are kept for
design context and provenance.

If you want to *run* something, do not start here. Start at
[`README.md`](README.md), then [`QUICKSTART_A3_ISAAC.md`](QUICKSTART_A3_ISAAC.md).

## Where the current contracts live

Where a preserved document disagrees with the shipped code, the code and these
docs win:

| Topic | Current, authoritative |
|-------|------------------------|
| Observation / action contract | [`docs/POLICY_INTERFACE.md`](docs/POLICY_INTERFACE.md) |
| Planner pipeline and `RacketCommand` | [`docs/PLANNER_INTERFACE.md`](docs/PLANNER_INTERFACE.md) |
| Training the policy | [`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md) |
| Running on the A3 (sim and real) | [`docs/RUN_ON_AGIBOT.md`](docs/RUN_ON_AGIBOT.md) |
| Mocap frames and topics | [`mocap/README.md`](mocap/README.md) |
| Asset map / joint order | [`A3_ASSETS.md`](A3_ASSETS.md) |

## What the rewrite dropped

[REMOVED_FROM_STARTER.md](docs/REMOVED_FROM_STARTER.md) records every file the
HOPE rewrite removed — what it was, what replaced it, and how to
retrieve it. It also lists the one **known gap** (a retargeted-CSV → `.npz`
motion converter) and the exact steps to revive it.

## System design documents

| Document | Scope | Version |
|----------|-------|---------|
| [HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md](HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md) | Ball state estimation, trajectory prediction, and racket target planning (Stages 1–3 of the HITTER framework), in the HOPE canonical frame | v0.1 |
| [HOPE_WBC_Simulation_Training_Reference_Setup.md](HOPE_WBC_Simulation_Training_Reference_Setup.md) | SMPL-X motion acquisition, GMR retargeting, and the BeyondMimic RL pipeline for whole-body control (Stage 4), with the original dual-backend Isaac Lab / mjlab plan | v0.5 |
| [HOPE_Hardware_Deployment_Reference_Setup.md](HOPE_Hardware_Deployment_Reference_Setup.md) | Real-robot deployment architecture for G1 (`legged_control2`) and A3 (AimRT): ONNX inference, ROS 2 node graph, PD tuning, and safety procedures | v0.2 |
| [mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md](mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md) · [ZH](mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup_ZH.md) | OptiTrack/ROS 2 arena configuration, coordinate frames, tracked-object taxonomy, robot root-frame registration, and ball tracking | v0.4 |

The three system design documents preserved at the repository root
(RACKET_PLANNER / WBC_TRAINING / HARDWARE_DEPLOYMENT) each carry a **Section 0
prologue** listing their implementation differences from the original HITTER
work (see [References](#references)); the mocap pair documents its differences
inline instead.

The mocap documents stay under [`mocap/`](mocap/) next to the frame/topic
contract they elaborate on, and next to the `two_ball_types.jpeg` figure they
embed.

## Competition rules

| Document | Language |
|----------|----------|
| [HOPE_AI_Challenge_2026_Rules_EN.docx](HOPE_AI_Challenge_2026_Rules_EN.docx) | English |
| [HOPE_AI_Challenge_2026_Rules_ZH.docx](HOPE_AI_Challenge_2026_Rules_ZH.docx) | 中文 |

### A rule that shapes the whole design

**Racket tracking is prohibited.** Motion capture tracks exactly three
categories: the table origin frame (PPT), each humanoid's marker-cluster rigid
body (P1, P2), and the ball. A calibrated static transform maps P1/P2 to the
robot's declared URDF root frame (`pelvis` on Unitree G1; `pelvis_link` on
Agibot A3). No markers are placed
on the racket, hand, or wrist link. Each robot must infer its paddle's 6-DOF
pose by forward kinematics from that root frame plus joint encoders. This
constraint is why the policy contract exposes joint state rather than a
measured racket pose.

## References

- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*. [Project page](https://humanoid-table-tennis.github.io/)
- Liao, Q., et al. (2025). BeyondMimic: From Motion Tracking to Versatile Humanoid Control via Guided Diffusion. *arXiv:2508.08241v4*. [Project page](https://beyondmimic.github.io/)
- Araújo, J. P., Ze, Y., Xu, P., Wu, J., & Liu, C. K. (2025). Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking. *arXiv:2510.02252*.

## Maintenance

If a preserved document falls behind the code, **add a note or an index entry
rather than deleting it.** Keep the original information; record what changed.
