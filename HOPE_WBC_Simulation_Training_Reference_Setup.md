# HOPE Whole-Body Controller Simulation Training Reference Setup

v0.5 — 2026-03-25

## Overview

This document describes the simulation training pipeline for the HOPE ping-pong whole-body controller (WBC), which is Stage 4 in the HITTER hierarchical framework. The WBC is a reinforcement learning policy that receives racket target commands from the HOPE model-based planner (Stages 1–3) and produces coordinated whole-body joint commands to position the paddle at the desired interception point with the correct velocity and orientation.

The training pipeline is based on **BeyondMimic** (Liao et al., arXiv:2508.08241), an open-source motion tracking framework that trains humanoid whole-body controllers from human motion references. The HITTER paper explicitly states it was "built upon" BeyondMimic — Qiayuan Liao is a co-author on both papers, and the same lab (UC Berkeley HybridRobotics) maintains both codebases.

BeyondMimic supports two simulation backends:

- **Isaac Lab + PhysX** (primary) — The original implementation, using NVIDIA Isaac Sim with USD robot assets. This is the reference path for the Unitree G1.
- **mjlab + MuJoCo Warp** (alternative) — A lightweight reproduction using the same Isaac Lab-style manager API but powered by MuJoCo Warp for GPU-accelerated physics. This is the recommended path for the Agibot Expedition A3 and other robots with MJCF models but no USD assets.

Both backends share the same MDP formulation, reward functions, and training hyperparameters. Policies trained in either framework export to ONNX and deploy through the same `motion_tracking_controller` inference pipeline.

The pipeline proceeds through four phases, all covered in this document:

1. **Human motion acquisition** — Obtain table tennis forehand/backhand swing clips as SMPL/SMPL-X skeleton data
2. **Motion retargeting** — Convert SMPL motions to the target robot's joint space using GMR
3. **Motion preprocessing** — Convert retargeted motions to the BeyondMimic training format
4. **RL policy training** — Train the motion tracking policy using the BeyondMimic MDP (Section 4 for Isaac Lab/G1, Section 4A for mjlab/A3)

The trained policy is exported as an ONNX model for deployment. Deployment to the real robot via ROS 2 is covered in the companion *HOPE Hardware Deployment Reference Setup*.

---

## 0  Prologue — Differences from the HITTER Paper

This document describes the HOPE whole-body controller (WBC) simulation training pipeline, which is Stage 4 in the HITTER hierarchical framework. Unlike the companion planner document (Stages 1–3), which is a clean-room reimplementation from paper equations, the WBC training pipeline is based entirely on open-source code: BeyondMimic (`whole_body_tracking`, MIT license) and GMR (`github.com/YanjieZe/GMR`, MIT license). The HITTER paper's WBC training procedure closely follows BeyondMimic's published framework; the differences documented below are between this HOPE reference setup and the HITTER paper's described approach.

### 0.1  Implementation Differences from HITTER

| # | Aspect | HITTER paper | This HOPE reference setup | Rationale |
|---|--------|-------------|---------------------------|-----------|
| 1 | **Simulation backend** | Isaac Lab + PhysX only | Isaac Lab + PhysX (G1) **and** mjlab + MuJoCo Warp (A3) | HITTER targets only the Unitree G1. HOPE supports multiple humanoid platforms including Agibot A3, which lacks USD assets for Isaac Lab. Section 4A covers the mjlab path. |
| 2 | **Target robots** | Unitree G1 (29 DOF) only | G1 (29 DOF) and Agibot Expedition A3 (DOF TBD) | HOPE is a multi-platform competition. |
| 3 | **Reference motion source** | Two clips (forehand + backhand) from "video clips" via GVHMR + GMR | Same pipeline, but expanded guidance on video recording requirements, broadcast footage extraction (P2ANet, Olympics), and building a multi-stroke motion library | HITTER provides minimal detail on video acquisition. Section 1.1 Option A provides detailed specifications. |
| 4 | **SMPL-X hand pose** | Not discussed (implicitly discarded during retargeting) | Explicitly documented as discarded; detailed analysis of the hand → wrist → fixed racket mount kinematic chain | Section 2.8 explains why SMPL hand pose is irrelevant and how the 3D-printed wrist mount determines paddle orientation via the 7-DOF arm's FK. |
| 5 | **Racket mount model** | "3D-printed connector" on right wrist (Fig. 1a); not described in simulation detail | Explicit URDF/MJCF fixed-joint model with `T_mount` transform; calibration procedure documented | HITTER does not publish the mount's kinematic parameters or its simulation model. Section 2.8 provides the complete fixed-joint formulation. |
| 6 | **Robot asset format** | USD (downloaded from GCS bucket) | USD (GCS bucket, HuggingFace, or URDF import) with detailed explanation of USD format and three source variants | HITTER assumes familiarity with USD. Section 4.3 explains what USD is, documents three non-interchangeable G1 USD sources, and notes the Isaac Sim ≥ 5.0 URDF import path. |
| 7 | **MDP reward structure** | Three components: imitation + goal tracking + regularization (Section IV) | Same three components, documented with explicit observation vector dimensions and activation timing | HITTER describes the reward structure qualitatively. Section 4.6 provides the quantitative formulation. |
| 8 | **PPO hyperparameters** | MLP [512, 256, 128] published; other PPO params not published | MLP [512, 256, 128] confirmed; remaining params documented with source annotations (**[H]** = HITTER, **[B]** = BeyondMimic code, **[R]** = RSL-RL default) | HITTER publishes the network architecture and confirms PPO + asymmetric actor-critic, but does not publish LR, batch size, γ, λ, or other PPO coefficients. Section 4.8 annotates each parameter's provenance. |
| 9 | **Motion preprocessing** | Not described (internal pipeline) | Full `csv_to_npz.py` workflow with WandB Registry setup | BeyondMimic's preprocessing pipeline is open-source; HITTER does not describe this step. |
| 10 | **Deployment** | Described as "deployed zero-shot to the real robot" | Deferred to a separate deployment document; ONNX export and sim-to-sim verification covered here | HOPE separates training from deployment for clarity. |
| 11 | **Forehand/backhand selection** | Based on ball Y position relative to robot center | Same logic, documented as a WBC observation (swing type = ±1) | HITTER describes this in a single sentence; Section 4.6 specifies the observation encoding. |
| 12 | **Racket sensing architecture** | Racket pose is never measured by mocap; inferred via FK from joint encoders | Same: racket tracking by mocap is explicitly prohibited; FK chain `base_link → ... → wrist → T_mount → racket` computes paddle state | HOPE formalizes this as a competition rule. The mocap system tracks only the table origin (PPT), humanoid base_links (P1/P2), and ball. See companion Mocap doc Section 3.1 and this doc Section 2.8. |

### 0.2  What Is Directly Reused from HITTER / BeyondMimic

The following elements are used as-is from the published open-source code, with no modifications:

- **BeyondMimic MDP formulation** — DeepMimic reward functions, adaptive reference state initialization, asymmetric actor-critic architecture (from `whole_body_tracking`)
- **GMR retargeting pipeline** — SMPL-X to robot joint space conversion (from `github.com/YanjieZe/GMR`)
- **Motion preprocessing** — `csv_to_npz.py` forward kinematics computation (from `whole_body_tracking`)
- **PPO training loop** — RSL-RL PPO implementation with the published hyperparameters (from `whole_body_tracking`)
- **ONNX export** — Policy export and sim-to-sim verification (from `motion_tracking_controller`)

### 0.3  What Is Not Implemented in This Document

- **Real robot deployment** — The `motion_tracking_controller` ROS 2 inference pipeline, `legged_control2` low-level controller, and `unitree_bringup` launch configuration are covered in the companion *HOPE Hardware Deployment Reference Setup*.
- **Diffusion-based multi-skill composition** — BeyondMimic's diffusion distillation for composing multiple motion skills into a single policy is not used. HITTER trains separate forehand and backhand policies and switches between them.
- **Ball spin response** — Neither HITTER nor this reference setup adjusts the WBC's racket orientation to counteract ball spin.
- **HITTER's trained policy weights** — The HITTER paper has not released trained model weights. All training in this document starts from scratch using the BeyondMimic framework.

### 0.4  How HITTER Uses BeyondMimic

The HITTER paper (Su et al., arXiv:2508.21043v2) describes its WBC as a PPO-trained policy operating at 50 Hz that generates desired joint positions for the Unitree G1's 29 DOF. The paper incorporates two human reference motions (forehand and backhand table tennis swings) derived from video clips via the following pipeline:

```
Human video → GVHMR (pose estimation) → SMPL skeleton → GMR (retargeting) → G1 joint trajectories
```

The WBC is then trained in Isaac Lab using the BeyondMimic motion tracking formulation: the policy learns to track these reference swing motions while simultaneously responding to racket target commands from the planner. The key insight is that the reference motions provide the *style* of human-like swings, while the planner commands provide the *target* — where and when the paddle must arrive.

### 0.5  What BeyondMimic Provides

BeyondMimic is an open-source framework with three repositories:

| Repository | Purpose | URL |
|-----------|---------|-----|
| `whole_body_tracking` | Isaac Lab training code (MDP, rewards, PPO config) | `github.com/HybridRobotics/whole_body_tracking` |
| `mjlab` (tracking task) | MuJoCo Warp training code (same API, alternative backend) | `github.com/mujocolab/mjlab` |
| `motion_tracking_controller` | ROS 2 Jazzy deployment code (ONNX inference, `legged_control2`) | `github.com/HybridRobotics/motion_tracking_controller` |

This document covers the training side. Section 4 describes training via Isaac Lab (`whole_body_tracking`), and Section 4A describes the alternative mjlab path for robots without USD assets (e.g., Agibot A3). The framework provides a unified MDP and single set of hyperparameters that can train motion tracking policies for any motion in the LAFAN1 dataset without parameter tuning — the same setup is used for walking, sprinting, cartwheels, and table tennis swings.

### 0.6  HOPE-Specific Adaptations

The standard BeyondMimic training produces a motion *tracker* — a policy that follows a reference trajectory. For HOPE ping-pong, the WBC must additionally:

1. **Accept racket target commands** (`p_intercept`, `v_racket`, `n_racket`, `t_strike`) from the planner as observations
2. **Track the racket target** at the commanded time while following the reference swing style
3. **Reposition the base** by stepping to reach balls at different lateral positions
4. **Recover balance** after each swing for consecutive rallies

These adaptations correspond to the reward modifications described in HITTER Section IV (separate base position commands, racket goal tracking activated near strike time, and imitation reward for swing style). The base BeyondMimic MDP is extended with these additional observation and reward terms.

---

## 1  Phase 1 — Human Motion Acquisition

The WBC requires reference table tennis swing motions in SMPL or SMPL-X skeleton format. The HITTER paper uses two clips: one forehand swing and one backhand swing, each approximately 1.7 seconds long with the strike occurring at t ≈ 0.86 s.

### 1.1  Motion Sources

There are three ways to obtain SMPL swing motions:

**Option A — Extract from video via GVHMR (recommended for custom swings).**

GVHMR (Gravity-View Human Motion Recovery) is a monocular video-to-SMPL-X pipeline developed at Zhejiang University (SIGGRAPH Asia 2024). It recovers world-grounded, gravity-aligned 3D human motion from a single RGB video — no depth camera, no markers, no multi-view rig. The pipeline runs in well under a second for a typical swing clip on an RTX 4090.

**Recording setup for custom swings:**

Position a single camera to capture the player performing forehand and backhand table tennis swings. The recording requirements are driven by two bottlenecks in the GVHMR preprocessing pipeline: (1) a person detector + tracker must reliably produce bounding boxes around the player in every frame, and (2) a 2D pose estimator (ViTPose) must detect all major body keypoints within those crops.

Camera placement and framing:

- **Side view at 90° to the table's long axis** — the camera looks along the table's Y axis (in the HOPE coordinate convention), so the player's swing arc is fully visible in the image plane. This is the view that maximizes the visible range of arm, shoulder, and torso rotation. The HITTER paper uses this viewpoint for its reference swing clips.
- **Distance**: 3–5 meters from the player. The player should occupy roughly 40–70% of the frame height. Too far reduces keypoint resolution; too close risks the racket arm leaving the frame during the backswing.
- **Height**: Camera at approximately chest height (1.0–1.4 m). Floor-level or overhead angles introduce severe foreshortening of the legs or arms respectively, degrading 3D lift accuracy.
- **Background**: A clean, uncluttered background behind the player improves person detection reliability. Avoid other people standing directly behind the player in frame.
- **Lighting**: Even, diffuse lighting. Avoid strong backlighting (silhouettes) or harsh shadows that obscure arm/hand boundaries. Indoor gym lighting or overcast outdoor conditions work well.

Resolution and frame rate:

| Parameter | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| Resolution | 720p (1280×720) | 1080p (1920×1080) or higher | The person detector crops a bounding box around the player; the ViTPose keypoint estimator then operates on this crop. At 720p with the player at ~50% frame height, the crop is ~360 px tall — marginal for wrist and finger keypoints. At 1080p the crop is ~540 px, which gives robust keypoints including wrists. 4K provides no meaningful further improvement for GVHMR. |
| Frame rate | 25 fps | 30 fps | GVHMR processes every frame. Higher fps (60, 120) provides denser temporal sampling of the fast swing phase (~0.3 s from backswing peak to contact) but increases preprocessing time linearly. 30 fps yields ~9 frames across the swing phase, which is sufficient for SMPL-X pose interpolation. The LAFAN1 and AMASS datasets that GVHMR was trained on use 30 fps. If recording at 60 fps, the video can be downsampled to 30 fps before GVHMR processing with no quality loss. |
| Codec | H.264 / H.265 | H.264 at high bitrate (≥ 20 Mbps) | Avoid heavy compression artifacts that blur limb edges. Smartphone "cinematic" mode or action camera "standard" mode at 1080p30 is fine. |
| Duration | 1.5 s per swing | 3–5 s (swing + setup + recovery) | Each clip should contain one complete swing cycle: ready stance → backswing → forward swing → contact → follow-through → recovery. The HITTER paper uses ~1.7 s clips with contact at t ≈ 0.86 s. Include 0.5 s of standing before and after the swing for clean motion boundaries. |

What NOT to record:

- Do not use slow-motion (240 fps / 120 fps) without awareness that GVHMR will treat each frame as a regular timestep, producing an artificially slow motion. If using slow-motion capture, resample to 30 fps before GVHMR.
- Do not record from directly behind or in front of the player — the arm swing occludes the torso, and depth ambiguity is maximized in the axis-angle estimates.
- Do not include multiple players in the frame if possible. GVHMR's preprocessing tracks the largest detected person; if both players are visible, manually crop to isolate the target player or specify the tracking ID.

**GVHMR preprocessing pipeline (what happens under the hood):**

Given a monocular video, GVHMR runs four preprocessing stages before the main network:

1. **Person detection + tracking** — Detects the player's bounding box in every frame using an off-the-shelf detector (e.g., YOLO, RT-DETR) and links detections across frames with a tracker (ByteTrack or similar). The bounding box crop is the input to all subsequent stages.
2. **2D keypoint estimation** — ViTPose (a vision-transformer-based pose estimator) predicts 2D body keypoints (shoulders, elbows, wrists, hips, knees, ankles, etc.) from each bounding box crop. This is the most resolution-sensitive stage — if the crop is too small, wrist/hand keypoints become unreliable.
3. **Image feature extraction** — A CNN backbone (e.g., HMR2.0's ViT encoder) extracts per-frame image features from the crop.
4. **Camera rotation estimation** — Visual odometry (DPVO or similar) or device gyroscope data estimates the relative camera rotation between frames. For a static tripod-mounted camera, this is trivial (identity rotation).

The main GVHMR network then fuses these features into per-frame tokens, processed by a relative transformer, to output per-frame SMPL-X parameters (body pose θ, global orientation, root translation, shape β) in a gravity-aligned world coordinate system.

```bash
# Install GVHMR (https://github.com/zju3dv/GVHMR)
cd path/to/GVHMR
python tools/demo/demo.py --video=forehand_swing.mp4 -s
# Output: GVHMR/outputs/demo/forehand_swing/hmr4d_results.pt
```

The output `.pt` file contains per-frame SMPL-X parameters: body pose (in axis-angle), global orientation, translation, and shape parameters β.

**Using Olympic and professional broadcast footage as a video library:**

Researchers can extract SMPL-X pose sequences from recorded professional table tennis matches — including Olympic games, World Championships, and ITTF tour events — using the same GVHMR pipeline. This is a viable path to building a diverse motion library covering strokes that are difficult to record in a lab (e.g., high-speed loops, chop defense, backhand flick at competition speed). The SMPLOlympics project (Peng et al., 2024) demonstrates this principle across multiple sports, extracting SMPL motion from broadcast footage and using it as motion priors for physically simulated humanoids including table tennis.

Several existing datasets provide organized broadcast footage that can serve as the starting point:

- **P2ANet** (Ping Pong Action Network) — 2,721 video clips from professional matches (2017–2021), including the Tokyo 2020 Olympic table tennis competition, World Cups, and World Championships. The dataset was collected from the ITTF Museum and China Table Tennis Museum with authorization. All clips are annotated with stroke type labels (14 classes) and temporal boundaries. This is the most directly usable source for building a table tennis SMPL-X motion library.
- **TTStroke-21** — Untrimmed videos of table tennis games with stroke annotations, used in the MediaEval benchmark for action detection.
- **TT3D** (Table Tennis 3D Reconstruction) — A recent project (2025) that reconstructs 3D ball trajectories and player poses from broadcast table tennis footage, using RTMPose for 2D keypoints and MotionBERT for 3D lifting. Their automated camera calibration pipeline can help transform GVHMR's camera-space output to world coordinates for broadcast footage.

Practical considerations for broadcast footage:

- **Resolution is usually sufficient.** Professional broadcast footage is typically 1080p or 4K at 25 fps (PAL regions) or 30 fps (NTSC). The side-angle camera common in table tennis broadcasts (positioned perpendicular to the table, elevated ~3 m) provides a good viewpoint for GVHMR — similar to the recommended recording setup above. However, broadcast footage frequently cuts between camera angles; each cut must be treated as a separate clip.
- **Frame rate may be marginal.** Broadcast at 25 fps gives only ~7–8 frames across a 0.3 s swing phase. This is at the lower bound for GVHMR's temporal modeling. If the source footage is interlaced (common for older broadcasts), deinterlacing to 50 fps before processing can help, though with some quality loss.
- **Player isolation is required.** In broadcast footage, both players are typically visible. GVHMR processes one person at a time. The player of interest must be either (a) cropped manually before processing, or (b) identified via the tracker by selecting the correct detection ID. The near-side player (closer to the camera) generally produces better 3D estimates due to larger bounding box size.
- **Camera motion must be accounted for.** Broadcast cameras often pan and zoom to follow the ball. GVHMR handles moderate camera motion via its visual odometry preprocessing stage, but rapid zooms can degrade quality. Clips from the fixed "wide shot" camera (showing the full table from the side) are preferable to close-up tracking shots.
- **Copyright and licensing.** Olympic and ITTF match footage is copyrighted. Using extracted SMPL-X pose sequences (not the video frames themselves) for research purposes — specifically as motion references for RL training — falls in a gray area. The pose parameters are a derived mathematical representation, not a reproduction of the footage. The P2ANet dataset was collected with ITTF authorization. Researchers should verify licensing terms for their specific use case and institution.
- **Quality varies by era and event.** Tokyo 2020 and later events are available in 4K HDR; older World Championships may be 720p or standard definition. Prioritize recent footage (2018+) for best GVHMR results.

A practical workflow for building a swing library from broadcast footage:

```
1. Download authorized match clips (P2ANet, or directly from ITTF/YouTube)
2. Segment into per-stroke clips using P2ANet annotations or manual trimming
3. Crop to isolate the target player (near-side preferred)
4. Run GVHMR on each clip → per-stroke .pt files with SMPL-X parameters
5. Visually verify each extraction in the GVHMR viewer
6. Retarget to G1/A3 via GMR (Phase 2)
7. Register in WandB as named motions (e.g., "forehand_loop_fast", "backhand_chop_defense")
```

This workflow can produce dozens of stroke variations from a single match, enabling multi-skill policy training beyond the two-stroke (forehand + backhand) setup in the HITTER paper.

**Option B — Use existing motion capture datasets.**
The LAFAN1 dataset (Ubisoft) contains diverse locomotion but no table tennis swings. For ping-pong specific motions, candidate sources include:

- Custom recordings via a marker-based mocap system (OptiTrack/Vicon) exported as SMPL-X via MoSh++ or similar fitting
- The AMASS dataset (aggregated MoCap data in SMPL-H/SMPL-X format) — search for racket sport clips
- Video-extracted clips from professional table tennis footage via GVHMR

**Option C — Synthesize swing motions procedurally.**
Define a parametric swing trajectory in G1 joint space directly, bypassing SMPL entirely. This loses the human-like naturalness but can serve as a bootstrap for initial training. The HITTER paper does not use this approach.

### 1.2  SMPL / SMPL-X Body Model

The SMPL-X body model represents a human as:

- **Shape parameters β** (10 dims) — body proportions (height, limb lengths, bulk)
- **Body pose θ** (21 joints × 3 = 63 dims) — axis-angle rotations for each joint
- **Global orientation** (3 dims) — root rotation in axis-angle
- **Translation** (3 dims) — root position in world frame
- **Hand pose** (optional, 2 × 15 joints × 3 dims for SMPL-X)
- **Expression** (optional, 10 dims for SMPL-X)

For the retargeting pipeline, only the body pose, global orientation, and translation are used. Hand pose and expression are discarded. See Section 2.8 for a detailed explanation of why hand pose is irrelevant for the HOPE racket mount configuration and how the wrist orientation drives the paddle face angle instead.

### 1.3  Required Output Format

Each reference motion clip should be a time series at 30 fps (or higher) containing per-frame:

- Root position `[x, y, z]`
- Root orientation (quaternion or axis-angle)
- Body joint rotations (axis-angle per joint)

This data is stored in the GVHMR `.pt` format or as SMPL-X `.npz`/`.pkl` files, which GMR can consume directly.

---

## 2  Phase 2 — Motion Retargeting (SMPL → Target Robot)

The SMPL skeleton has different proportions and joint structure from any humanoid robot. **GMR** (General Motion Retargeting) is an optimization-based retargeting tool that maps SMPL motions to any target robot URDF/MJCF while preserving the motion's style and physical plausibility.

GMR supports multiple robots out of the box. For HOPE, the relevant targets are:

| Robot ID | Robot | DOF breakdown | Status |
|----------|-------|---------------|--------|
| `unitree_g1` | Unitree G1 | Leg (2×6) + Waist (3) + Arm (2×7) = 29 | ✅ Fully supported |
| `unitree_g1_with_hands` | Unitree G1 EDU | 29 + Hand (2×7) = 43 | ✅ Supported |
| `agibot_a3` | Agibot Expedition A3 | TBD (multi-DOF waist, 7-DOF arms expected) | Requires custom robot config |

### 2.1  GMR Overview

GMR performs retargeting in three stages:

1. **Key body matching** — Maps SMPL joints to G1 links (pelvis↔pelvis, shoulders↔shoulders, wrists↔wrists, etc.)
2. **Scaling** — Adjusts the SMPL motion to match the G1's body proportions while preserving the motion structure
3. **Optimization** — Solves for G1 joint angles that minimize the pose error between the retargeted G1 and the scaled SMPL, subject to joint limits and collision avoidance

The output is a G1 joint trajectory in generalized coordinates (29 DOF), suitable for BeyondMimic training.

### 2.2  Installation

```bash
# GMR (https://github.com/YanjieZe/GMR)
git clone https://github.com/YanjieZe/GMR.git
cd GMR
pip install -e .

# Download SMPL-X body models to assets/body_models/smplx/
# Required files: SMPLX_NEUTRAL.pkl, SMPLX_FEMALE.pkl, SMPLX_MALE.pkl
# Download from https://smpl-x.is.tue.mpg.de/
```

### 2.3  Retargeting from GVHMR Output

If the reference motion was extracted from video via GVHMR:

```bash
python scripts/gvhmr_to_robot.py \
    --gvhmr_pred_file path/to/hmr4d_results.pt \
    --robot unitree_g1 \
    --record_video
```

This produces a `.pkl` file containing the retargeted G1 joint trajectory. The `--record_video` flag opens a MuJoCo viewer to visualize the retargeted motion for quality verification.

### 2.4  Retargeting from SMPL-X / AMASS Data

For SMPL-X data from AMASS or other sources:

```bash
# Single motion
python scripts/smplx_to_robot.py \
    --src_file path/to/smplx_motion.npz \
    --robot unitree_g1 \
    --save_path path/to/output.pkl

# Batch retargeting (entire dataset directory)
python scripts/smplx_to_robot_dataset.py \
    --src_folder path/to/smplx_dir/ \
    --tgt_folder path/to/robot_dir/ \
    --robot unitree_g1
```

### 2.5  Retargeting from BVH (LAFAN1)

The LAFAN1 dataset uses BVH format, which GMR supports directly:

```bash
python scripts/bvh_to_robot.py \
    --bvh_file path/to/motion.bvh \
    --robot unitree_g1 \
    --save_path path/to/output.pkl \
    --format lafan1
```

### 2.6  G1 Joint Configuration

GMR targets the following G1 joint configuration (29 DOF):

```
Legs:      2 × 6 DOF = 12 DOF  (hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll)
Waist:     3 DOF               (yaw, roll, pitch)
Arms:      2 × 7 DOF = 14 DOF  (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
                                 wrist_yaw, wrist_roll, wrist_pitch)
Total:     29 DOF
```

For the G1 EDU variant with dexterous hands, GMR can also retarget hand joints (2 × 7 DOF = 43 DOF total), but the hand DOFs are not used in the HOPE configuration — the right hand is replaced by a 3D-printed racket mount (see Section 2.8).

### 2.7  Quality Verification

After retargeting, verify the motion quality:

1. **Visual inspection** — Play the retargeted motion in MuJoCo and compare side-by-side with the original SMPL video
2. **Joint limit violations** — Check that all joint angles stay within the G1's mechanical limits
3. **Ground penetration** — The retargeted feet should not penetrate the ground plane; GMR includes a ground penetration penalty but artifacts can occur
4. **Self-collision** — Arms should not intersect the torso during swings; this is the most common artifact for table tennis motions due to the cross-body nature of backhand swings
5. **Temporal continuity** — Joint trajectories should be smooth; sudden jumps indicate optimization failures

If artifacts are present, GMR's optimization parameters can be tuned per-motion. However, the default parameters work well for most motions evaluated on the LAFAN1 dataset.

### 2.8  Racket Mount Kinematics: From Human Hand to Fixed Wrist Attachment

> **HOPE Racket Tracking Prohibition.** The motion capture system must NOT track the ping-pong racket. No reflective markers may be placed on the racket, the robot's hand/gripper, or the wrist link. The motion capture system provides exactly three categories of tracking data: (1) the ping-pong table origin frame (PPT), (2) each humanoid's `base_link` (P1, P2), and (3) the ping-pong ball (single unlabeled marker). The racket's 6-DOF pose is NEVER measured externally — each robot must infer its paddle state through forward kinematics from `base_link` + joint encoders through the arm kinematic chain. This is a deliberate HOPE competition design constraint that tests autonomous paddle control. See the companion *HOPE Motion Capture System Reference Setup* (Section 3.1 — Racket Exclusion Policy) for enforcement details and rationale.

In the HITTER system, the G1's right hand is physically removed and replaced with a 3D-printed connector that rigidly mounts the racket to the right wrist link. The LATENT tennis system uses an identical approach. This hardware modification has important implications for every stage of the pipeline — motion acquisition, retargeting, simulation, and deployment — and is the mechanism by which the racket tracking prohibition is satisfied: the robot controls its paddle entirely through its own joint angles, with no external sensing of the end-effector.

**The physical setup:**

```
Human player:                          G1 robot:
  forearm → wrist → hand → fingers     forearm → wrist_link → [3D-printed mount] → racket
            ↕ grip adjusts paddle              ↕ FIXED transform (no DOF)
            ↕ angle continuously               ↕ racket face is rigid
```

In a human player, the paddle orientation is controlled by the combination of wrist rotation AND finger/grip adjustments. In the G1 with the 3D-printed mount, the paddle orientation is determined ENTIRELY by the wrist link pose — there are zero additional degrees of freedom between the wrist and the racket face. The mount defines a fixed homogeneous transform `T_mount` from the wrist link frame to the racket center frame:

```
T_racket = T_base × T_hip × ... × T_shoulder × T_elbow × T_wrist × T_mount
           ├──────── FK through kinematic chain ────────┤   ├── fixed ──┤
```

The 7-DOF right arm provides:

```
shoulder_pitch, shoulder_roll, shoulder_yaw  →  3 DOF (shoulder)
elbow                                        →  1 DOF
wrist_yaw, wrist_roll, wrist_pitch           →  3 DOF (wrist)
                                         Total: 7 DOF
```

A racket target has 6 DOF (3 position + 3 orientation). The 7-DOF arm is therefore kinematically redundant by 1 DOF — this extra DOF manifests as "elbow posture" (the elbow can be in different positions while the wrist achieves the same end-effector pose). The BeyondMimic reference motion resolves this redundancy by providing a human-like elbow trajectory.

**Impact on Phase 1 (SMPL-X extraction):**

When GVHMR processes a video of a human player swinging a paddle, it estimates the full SMPL-X body including hand pose (15 joints × 3 dims per hand = 45 dims). For the HOPE pipeline:

- The **hand pose parameters are discarded**. They encode the grip shape around the paddle handle, which does not exist on the G1.
- The **wrist orientation is the critical output**. In the human, the wrist rotation determines the forearm's orientation relative to the upper arm. This, combined with the fixed mount transform, determines where the racket face ends up on the robot.
- The **SMPL-X wrist joint implicitly encodes the paddle angle** because a human player's wrist angle is the primary control variable for paddle face orientation during play. The grip is relatively static during a swing — the paddle angle changes through wrist pronation/supination and flexion/extension, which map directly to the G1's `wrist_yaw`, `wrist_roll`, and `wrist_pitch` joints.

In practice, no special handling is needed during GVHMR extraction — just ignore the hand parameters in the output `.pt` file. The wrist orientation is estimated correctly regardless of whether the hand pose is accurate.

**Impact on Phase 2 (GMR retargeting):**

GMR retargeting to `unitree_g1` (29 DOF) already drops hand joints by design — the target robot config only includes the 29 body DOFs (12 leg + 3 waist + 14 arm). The retargeting optimization maps SMPL-X joint rotations to G1 joint angles using key body matching:

```
SMPL-X joint          →  G1 joint(s)         →  What it controls
─────────────────────────────────────────────────────────────────
right_shoulder         →  right_shoulder_*     →  Upper arm direction
right_elbow            →  right_elbow          →  Forearm angle
right_wrist            →  right_wrist_*        →  Forearm roll + paddle face angle
right_hand (15 joints) →  (discarded)          →  N/A — no hand on robot
```

The critical mapping is the **SMPL right_wrist → G1 wrist_yaw/roll/pitch**. GMR's optimization minimizes the rotation error between the SMPL wrist frame and the G1 wrist link frame. Because the SMPL wrist rotation encodes the forearm orientation that a human player uses to control the paddle, the retargeted G1 wrist angles will produce a similar paddle face orientation through the fixed mount.

**However**, there is a calibration subtlety: the SMPL wrist frame and the G1 wrist link frame have different default orientations. GMR handles this through its key body matching step, which aligns the reference frames. But the **mount transform `T_mount` must be consistent between the simulation model and the physical robot**. If the 3D-printed mount is designed so that the racket face is perpendicular to the wrist link's Z-axis when all wrist joints are at zero, then the mount transform is a simple rotation. If the mount has an offset angle (common in ergonomic designs), this offset must be encoded in `T_mount`.

**Impact on Phase 4 (RL training):**

In the Isaac Lab / mjlab simulation, the racket is modeled as a rigid body attached to the right wrist link via a fixed joint:

```xml
<!-- In the G1 URDF/MJCF: add racket as a fixed child of right_wrist_link -->
<link name="racket_link">
  <visual>
    <geometry><mesh filename="racket.stl"/></geometry>
  </visual>
  <collision>
    <geometry><cylinder radius="0.075" length="0.005"/></geometry>
  </collision>
  <inertial>
    <mass value="0.18"/>  <!-- 180g paddle -->
    <origin xyz="0 0 0.08"/>  <!-- CoM offset from wrist -->
  </inertial>
</link>

<joint name="racket_mount" type="fixed">
  <parent link="right_wrist_link"/>
  <child link="racket_link"/>
  <!-- T_mount: transform from wrist link to racket center -->
  <origin xyz="0.0 0.0 0.15" rpy="0.0 0.0 0.0"/>
</joint>
```

The WBC's racket target tracking reward (Section 4.6) computes the actual racket state via forward kinematics through this fixed joint:

```python
# In the reward function:
# racket_pos and racket_quat are computed by FK through the kinematic chain
# including the fixed T_mount joint
racket_pos = robot.data.body_pos_w[:, racket_link_idx]    # shape: (num_envs, 3)
racket_quat = robot.data.body_quat_w[:, racket_link_idx]  # shape: (num_envs, 4)
racket_normal = quat_to_matrix(racket_quat)[:, :, 2]      # Z-axis of racket frame = face normal

# Compare against planner's desired racket state
pos_error = torch.norm(racket_pos - desired_racket_pos, dim=-1)
vel_error = torch.norm(racket_vel - desired_racket_vel, dim=-1)
normal_error = torch.acos(torch.clamp(
    torch.sum(racket_normal * desired_racket_normal, dim=-1), -1, 1))
```

The policy never directly controls the racket orientation — it controls the 7 arm joint angles, and the racket orientation emerges from FK through the fixed mount. The BeyondMimic reference motion provides the "style" of the swing (human-like arm trajectory), and the racket target from the planner provides the "goal" (where the racket face must be at strike time). The policy learns to blend these two objectives.

**Summary of the hand → mount kinematic chain:**

| Pipeline stage | Human hand | Robot equivalent | What happens |
|---------------|------------|-----------------|--------------|
| Video recording | Player grips paddle, wrist controls face angle | N/A | Record normally |
| GVHMR extraction | Estimates hand pose (θ_hand) + wrist orientation | N/A | Hand pose discarded; wrist orientation is the useful output |
| GMR retargeting | Maps SMPL wrist → G1 wrist joints | 3 wrist DOF (yaw, roll, pitch) | Hand joints dropped; wrist rotation preserved |
| Simulation model | N/A | Fixed joint: wrist_link → racket_link | `T_mount` defines racket pose relative to wrist |
| RL training reward | N/A | FK: base → ... → wrist → mount → racket | Racket state computed via FK; compared to planner target |
| Deployment | N/A | 3D-printed bracket on physical robot | Mount transform must match simulation `T_mount` exactly |

**Calibrating `T_mount`:**

The mount transform must be measured on the physical 3D-printed bracket and replicated exactly in the simulation URDF/MJCF. To calibrate:

1. Assemble the bracket and racket on the physical G1
2. Command all right arm joints to their zero position
3. Measure the racket face center position and normal direction relative to the wrist link frame (using a ruler, or more precisely, using the OptiTrack system with temporary markers on the racket face during a one-time offline calibration — these markers must be removed before competition play per the racket exclusion policy)
4. Express this as a homogeneous transform `T_mount = [R | t]`
5. Enter this transform as the `<origin>` of the fixed joint in the simulation model

If `T_mount` is wrong in simulation, the trained policy will systematically miss the ball on the real robot — the planner computes where the racket face should be, but the policy positions the wrist such that the actual racket face is offset from the intended position.

---

## 3  Phase 3 — Motion Preprocessing for BeyondMimic

The retargeted G1 joint trajectory (a `.pkl` or `.csv` file in Unitree convention) must be converted to the BeyondMimic training format, which includes maximum-coordinate information (body poses, velocities, and accelerations) computed via forward kinematics.

### 3.1  Conversion to BeyondMimic Format

BeyondMimic uses Unitree's CSV convention for retargeted motions. The `csv_to_npz.py` script in the `whole_body_tracking` repository performs the conversion and uploads to the WandB registry:

```bash
cd whole_body_tracking

# Convert retargeted motion to BeyondMimic format
python scripts/csv_to_npz.py \
    --input_file forehand_swing.csv \
    --input_fps 30 \
    --output_name hope_forehand \
    --headless
```

This script:

1. Loads the retargeted joint trajectory
2. Builds the G1 URDF model in Isaac Sim
3. Runs forward kinematics to compute all body link poses, velocities, and accelerations
4. Packages the result as an `.npz` file
5. Uploads to the WandB Registry under the configured collection

### 3.2  WandB Registry Setup

BeyondMimic uses WandB (Weights & Biases) to manage motion data and training runs:

1. Create a WandB account and organization
2. In the WandB dashboard, navigate to **Registry → Core** and create a new collection named `Motions` with artifact type "All Types"
3. Set the `WANDB_ENTITY` environment variable to your organization name:
   ```bash
   export WANDB_ENTITY=your-org-name
   ```

### 3.3  Verifying the Preprocessed Motion

Replay the processed motion in Isaac Sim to confirm it loaded correctly:

```bash
python scripts/replay_npz.py \
    --registry_name=your-org-name-org/wandb-registry-motions/hope_forehand
```

This renders the G1 model playing back the reference motion. Verify that the swing timing, racket trajectory, and body posture match the original human reference.

---

## 4  Phase 4 — RL Policy Training in Isaac Lab

### 4.1  Prerequisites

| Requirement | Version |
|-------------|---------|
| Isaac Sim | 4.5.0 |
| Isaac Lab | 2.1.0 |
| Python | 3.10 |
| GPU | NVIDIA RTX 4090 or better (24+ GB VRAM recommended) |
| Platform | Linux (Ubuntu 22.04 or 24.04) |

### 4.2  Installation

```bash
# 1. Install Isaac Lab v2.1.0
# Follow: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
# Conda installation is recommended

# 2. Clone the BeyondMimic training repository
git clone https://github.com/HybridRobotics/whole_body_tracking.git
cd whole_body_tracking

# 3. Download G1 robot description (USD assets for Isaac Sim)
curl -L -o unitree_description.tar.gz \
    https://storage.googleapis.com/qiayuanl_robot_descriptions/unitree_description.tar.gz && \
tar -xzf unitree_description.tar.gz \
    -C source/whole_body_tracking/whole_body_tracking/assets/ && \
rm unitree_description.tar.gz

# 4. Install the training package
python -m pip install -e source/whole_body_tracking
```

### 4.3  G1 Robot Assets: USD Format Explained

Isaac Lab's physics engine (PhysX) requires robot models in **USD (Universal Scene Description)** format. USD is an open-source 3D scene description framework originally developed by Pixar and now maintained by the Alliance for OpenUSD (with NVIDIA as a founding member). NVIDIA adopted USD as the native format for its Omniverse platform, which Isaac Sim is built on.

A USD file is a hierarchical scene graph — a tree of "prims" (primitives) — that bundles everything the simulator needs to instantiate a robot in a single structured file:

- **Visual mesh geometry** — High-fidelity 3D meshes for rendering (used in the Isaac Sim viewport but not in physics)
- **Collision geometry** — Simplified convex hulls or primitives (boxes, capsules, spheres) used by PhysX for contact detection
- **Kinematic tree** — Parent-child link hierarchy with joint types (revolute, prismatic, fixed), joint axes, and joint limits
- **Inertial properties** — Per-link mass, center of mass, and rotational inertia tensor
- **Actuator parameters** — Joint stiffness, damping, armature, effort limits, and velocity limits
- **Material properties** — Surface friction, restitution, and visual materials/textures
- **Instanceability metadata** — Flags that allow Isaac Sim to efficiently clone thousands of robots in parallel RL environments by sharing mesh data on the GPU

This is in contrast to URDF (the ROS-native format), which separates the robot description (`.urdf` XML) from its meshes (`.stl`/`.dae` files in a `meshes/` folder) and cannot express actuator dynamics, simulation-specific parameters, or instancing metadata. USD encapsulates all of this in a single self-contained asset.

**Where to obtain G1 USD assets:**

There are three sources of G1 USD files, and they are **not interchangeable** — they differ in DOF count, actuator tuning, and collision geometry:

1. **BeyondMimic GCS bucket** (used by `whole_body_tracking`) — Downloaded in step 3 of the installation above. This is a custom-tuned G1 asset with 29 DOF, armature values, and PD gains specifically calibrated for BeyondMimic motion tracking. This is the recommended asset for HOPE training.

   ```bash
   curl -L -o unitree_description.tar.gz \
       https://storage.googleapis.com/qiayuanl_robot_descriptions/unitree_description.tar.gz
   ```

2. **Unitree official USD repository** — Hosted on HuggingFace at `huggingface.co/datasets/unitreerobotics/unitree_model`. Contains USD assets for G1, B2, H1, Go2, and other Unitree robots. Used by Unitree's own `unitree_rl_lab` repository. Note: the official G1 USD has different actuator parameters from the BeyondMimic version, and directly substituting it may cause training convergence issues (as documented in Isaac Lab issue #4037).

   ```bash
   git clone https://huggingface.co/datasets/unitreerobotics/unitree_model
   # Configure UNITREE_MODEL_DIR in your robot config to point here
   ```

3. **Isaac Lab built-in assets** — Isaac Lab ships with its own G1 USD (accessible via Omniverse Nucleus). This is a third variant with yet another DOF configuration (23 DOF in some versions). Not recommended for HOPE as it does not match the 29-DOF configuration used by BeyondMimic and HITTER.

**URDF alternative (Isaac Sim ≥ 5.0 only):**

Starting with Isaac Sim 5.0, Isaac Lab supports direct URDF import without requiring a pre-converted USD file. Unitree's standard URDF files are available in the `unitree_ros` repository:

```bash
git clone https://github.com/unitreerobotics/unitree_ros.git
# URDF at: unitree_ros/robots/g1_description/urdf/g1.urdf
```

This eliminates the USD conversion step but requires Isaac Sim ≥ 5.0, which is newer than the version BeyondMimic's `whole_body_tracking` was tested against (Isaac Sim 4.5.0). If using this path, verify that the URDF's joint limits, actuator gains, and collision geometry match the BeyondMimic training assumptions.

**For the Agibot A3:** USD assets are not publicly available. This is the primary reason Section 4A recommends the mjlab + MuJoCo Warp path for A3 training, which consumes MJCF files directly.

### 4.4  BeyondMimic MDP Formulation

The BeyondMimic training environment is defined as a Markov Decision Process (MDP) with the following structure. This is the base formulation; the HOPE-specific extensions for racket target tracking are described in Section 4.6.

**State space** — The observation vector includes:

- Proprioceptive state: joint positions, joint velocities, base angular velocity (from IMU), projected gravity vector
- Reference motion state: target joint positions and velocities from the reference motion at the current and future timesteps
- Phase variable: normalized time within the current swing cycle

**Action space** — Target joint positions for all 29 DOF, output at 50 Hz. These are converted to joint torques via a PD controller with configurable stiffness and damping.

**Reward function** — The BeyondMimic reward follows the DeepMimic formulation:

```
r = w_pose · r_pose + w_vel · r_vel + w_ee · r_ee + w_smooth · r_smooth
```

Where:
- `r_pose` — Joint angle tracking error (exponential kernel)
- `r_vel` — Joint velocity tracking error
- `r_ee` — End-effector position tracking (hands, feet)
- `r_smooth` — Action smoothness penalty (penalizes jerk)

**Episode structure** — Each episode begins with the robot initialized near a point in the reference motion (with randomization). The episode runs for the duration of the reference clip or until early termination (e.g., the robot falls).

**Adaptive sampling** — BeyondMimic uses adaptive reference state initialization (RSI): episodes are more likely to start from frames where the policy previously failed, concentrating training on the hardest parts of the motion.

### 4.5  Code Structure

```
whole_body_tracking/
├── scripts/
│   ├── csv_to_npz.py              # Motion preprocessing
│   ├── replay_npz.py              # Motion playback in Isaac Sim
│   └── rsl_rl/
│       ├── train.py               # Training entry point
│       └── play.py                # Evaluation entry point
└── source/whole_body_tracking/whole_body_tracking/
    ├── tasks/tracking/
    │   ├── tracking_env_cfg.py    # MDP hyperparameters
    │   ├── mdp/
    │   │   ├── commands.py        # Reference motion loading, error computation,
    │   │   │                      # adaptive sampling, initial state randomization
    │   │   ├── rewards.py         # DeepMimic reward functions + smoothing
    │   │   ├── events.py          # Domain randomization
    │   │   ├── observations.py    # Observation terms
    │   │   └── terminations.py    # Early termination conditions
    │   └── config/g1/agents/
    │       └── rsl_rl_ppo_cfg.py  # PPO hyperparameters
    ├── robots/                    # Robot-specific parameters
    │   │                          # (armature, PD gains, action scale)
    └── assets/                    # G1 URDF/USD model files
```

### 4.6  HOPE-Specific Training Extensions

For HOPE ping-pong, the base BeyondMimic MDP must be extended with the racket target tracking objective described in HITTER Section IV. The key modifications are:

**Additional observations (appended to the base observation vector):**

- Desired racket position relative to base frame: `p̂_intercept − p_base` (3 dims)
- Desired racket velocity in world frame: `v̂_racket` (3 dims)
- Desired racket face normal: `n̂_racket` (3 dims)
- Time remaining until strike: `t_left = t_strike − t_current` (1 dim)
- Desired base XY position in world frame: `p̂_base,xy` (2 dims)
- Swing type: forehand (+1) or backhand (−1) (1 dim)

**Sensing architecture note:** The observations above contain only the *desired* racket state (from the planner) — never the *measured* racket state. Per the HOPE racket tracking prohibition (Section 2.8, companion Mocap doc Section 3.1), no motion capture data is available for the racket's actual pose. The WBC is a proprioceptive controller: it receives `base_link` pose from mocap and joint encoder readings from the robot, then relies on its trained internal model to drive the 7-DOF arm such that the racket (attached via fixed mount) arrives at the commanded state. The `r_racket` reward term (below) is computed in simulation via FK — on the real robot, there is no closed-loop feedback on racket position error.

**Additional reward terms (added to the BeyondMimic base reward):**

```
r_total = w_i · r_imitation + w_g · r_goal + w_r · r_regularization
```

Where:

- `r_imitation` — Upper-body reference motion tracking (BeyondMimic base reward, applied to arm and waist joints). Activated throughout the episode.
- `r_goal` — Racket target tracking. Contains two sub-terms:
  - **Base position tracking** (`r_base`): Activated *before* the strike time. Encourages the robot to step to the desired base position.
  - **Racket state tracking** (`r_racket`): Activated *near* the strike time (within a window of ~0.2 s). Tracks racket position, velocity, and orientation against the planner's commands.
- `r_regularization` — Joint torque smoothness, contact force penalties, energy minimization. Active throughout.

**Training procedure:**

Each training episode simulates a single rally:

1. The robot is initialized in a standing pose at a random lateral offset from the table center
2. A random racket target is sampled (position on the virtual hitting plane, incoming ball velocity, outgoing direction)
3. The swing type (forehand/backhand) is selected based on the target's Y position relative to the robot
4. The corresponding reference motion (forehand or backhand clip) is loaded
5. The policy receives the racket target as observations and must track both the reference swing style and the racket target
6. After the strike, the episode continues briefly for balance recovery, then resets

**Domain randomization** (from BeyondMimic `events.py`, extended for HOPE):

- Joint PD gain randomization (±20%)
- Link mass randomization (±15%)
- Friction coefficient randomization
- External force perturbations (push disturbances)
- Observation noise (simulating IMU and encoder noise)
- Motor strength randomization
- Racket target position randomization (within the reachable workspace)

### 4.7  Training Command

**Standard BeyondMimic motion tracking (baseline, no racket target):**

```bash
python scripts/rsl_rl/train.py \
    --task=Tracking-Flat-G1-v0 \
    --registry_name your-org-org/wandb-registry-motions/hope_forehand \
    --headless \
    --logger wandb \
    --log_project_name hope_wbc \
    --run_name forehand_tracking
```

**HOPE ping-pong WBC (with racket target tracking):**

This requires the HOPE-specific environment configuration. The environment extends `Tracking-Flat-G1-v0` with the additional observations and rewards from Section 4.6:

```bash
python scripts/rsl_rl/train.py \
    --task=HOPE-PingPong-G1-v0 \
    --registry_name your-org-org/wandb-registry-motions/hope_forehand \
    --headless \
    --logger wandb \
    --log_project_name hope_wbc \
    --run_name hope_forehand_racket_tracking
```

**Typical training time:** 2–4 hours on a single RTX 4090 for motion tracking convergence. The HOPE extensions (racket target tracking + base repositioning) add approximately 50% more training time due to the larger observation space and additional reward terms.

### 4.8  Training Hyperparameters

The PPO hyperparameters are defined in `rsl_rl_ppo_cfg.py`. BeyondMimic's key design choice is that **the same hyperparameters work for all motions** without tuning.

The table below documents the training configuration. Each parameter is annotated with its source: **[H]** = confirmed by the HITTER paper (arXiv:2508.21043), **[B]** = from BeyondMimic open-source code (`whole_body_tracking`), **[R]** = RSL-RL default value. Parameters marked **[R]** are standard defaults that should be verified against the `rsl_rl_ppo_cfg.py` file in the `whole_body_tracking` repository at training time, as the BeyondMimic authors may have modified them.

| Parameter | Value | Source | Notes |
|-----------|-------|--------|-------|
| Algorithm | PPO | **[H]** | Proximal Policy Optimization. HITTER: "trained end-to-end using PPO" |
| Actor network | MLP [512, 256, 128] | **[H]** | HITTER: "three hidden layers of sizes 512, 256, and 128" |
| Critic network | MLP [512, 256, 128] | **[H]** | HITTER: "Both the actor and critic are implemented as MLPs [512, 256, 128]" |
| Activation function | ELU | **[R]** | RSL-RL default. Not explicitly stated in HITTER. |
| Asymmetric critic | Yes | **[H]** | HITTER: critic receives privileged info (body poses T_B, time left t_left) |
| PD gains | Heuristic | **[H]** | HITTER: "set heuristically following [BeyondMimic]" |
| Control frequency | 50 Hz | **[H]** | HITTER: "outputs desired joint positions for all 29 joints at 50 Hz" |
| Num environments | 4096 | **[B]** | BeyondMimic training command uses 4096 parallel envs |
| Learning rate | 1e-3 (initial) | **[R]** | RSL-RL default. HITTER does not publish LR. Verify in `rsl_rl_ppo_cfg.py`. |
| LR schedule | Adaptive or cosine | **[R]** | RSL-RL supports adaptive KL-based and fixed schedules. Verify in config. |
| Discount γ | 0.99 | **[R]** | Standard PPO default. Not stated in HITTER. |
| GAE λ | 0.95 | **[R]** | Standard PPO default. Not stated in HITTER. |
| Clip range | 0.2 | **[R]** | Standard PPO default. Not stated in HITTER. |
| Entropy coefficient | 0.01 | **[R]** | RSL-RL default. Not stated in HITTER. |
| Num steps per env | 24 | **[R]** | RSL-RL default for Isaac Lab humanoid tasks. Verify in config. |
| Num epochs per update | 5 | **[R]** | RSL-RL default. Not stated in HITTER. |
| Simulation dt | 0.005 s (200 Hz) | **[R]** | Standard Isaac Lab humanoid default. 4 physics substeps per policy step. |
| Episode length | ~170 steps (~3.4 s) | Inferred | HITTER uses ~1.7 s swing clips. Episode may span ~2× clip length for base repositioning + swing + recovery. |

**What HITTER does NOT publish:** The HITTER paper explicitly states it was "built upon" BeyondMimic [2] but does not publish the following: learning rate, learning rate schedule, batch size, mini-batch size, number of PPO epochs per update, discount factor, GAE lambda, clip range, entropy coefficient, simulation timestep, or episode length. The values marked **[R]** above are RSL-RL defaults that are reasonable starting points but should be cross-checked against the actual `rsl_rl_ppo_cfg.py` file in the `whole_body_tracking` repository before training.

**How to verify:** After cloning `whole_body_tracking`, inspect the config directly:

```bash
cat source/whole_body_tracking/whole_body_tracking/tasks/tracking/config/g1/agents/rsl_rl_ppo_cfg.py
```

This file contains the definitive hyperparameters that BeyondMimic uses (and by extension, that HITTER was built upon). Any values that differ from the **[R]** defaults listed above should be adopted from this file.

**Asymmetric actor-critic:** During training, the critic receives privileged information unavailable to the actor at deployment:

- Robot body poses (full state, not just proprioception)
- Ground truth contact forces
- Time left in episode
- True base velocity (actor only gets IMU-derived estimate)

This improves value estimation and handles sparse rewards more effectively.

### 4.9  Policy Evaluation

After training, evaluate the policy in simulation:

```bash
python scripts/rsl_rl/play.py \
    --task=Tracking-Flat-G1-v0 \
    --num_envs=2 \
    --wandb_path=your-org/hope_wbc/run_id
```

The WandB run path can be found in the run overview page. It follows the format `{organization}/{project_name}/{8-char-id}`.

**Evaluation metrics for HOPE:**

| Metric | Target | Description |
|--------|--------|-------------|
| Tracking success rate | > 90% | Fraction of episodes completed without falling |
| Racket position error at strike | < 7.5 cm | Must be within racket radius |
| Racket velocity error at strike | < 0.5 m/s | Along the desired velocity direction |
| Racket normal error at strike | < 15° | Angle between desired and actual face normal |
| Base repositioning time | < 0.8 s | Time to reach target base position (0.75 m displacement) |
| Motion naturalness | Qualitative | Visual comparison to human reference clip |

---

## 4A  Alternative: Training via mjlab + MuJoCo Warp (Agibot A3 and Other MJCF Robots)

The Isaac Lab path described in Section 4 requires USD robot assets and the NVIDIA Omniverse runtime. For robots that only have MJCF/URDF models — notably the Agibot Expedition A3 — an alternative training path is available via **mjlab**, a lightweight framework that reproduces the Isaac Lab manager-based API on top of MuJoCo Warp for GPU-accelerated physics.

mjlab is not a third-party reimplementation. It is maintained by the same research circle as BeyondMimic (acknowledged in the mjlab repository), ships the BeyondMimic motion tracking task as a built-in reference implementation, and produces policies that deploy through the same `motion_tracking_controller` ONNX inference pipeline. Unitree's official RL repository (`unitree_rl_mjlab`) is built on mjlab and already supports Go2, A2, G1, H1_2, and R1.

### 4A.1  Why mjlab for the A3

| Consideration | Isaac Lab + PhysX | mjlab + MuJoCo Warp |
|---------------|-------------------|----------------------|
| Robot model format | USD (Omniverse-specific) | MJCF (MuJoCo native) |
| Installation | Isaac Sim + Omniverse runtime (~50 GB, minutes to start) | `pip install mjlab mujoco-warp` (~500 MB, seconds to start) |
| GPU acceleration | PhysX CUDA | MuJoCo Warp CUDA |
| BeyondMimic MDP | ✅ `whole_body_tracking` | ✅ Built-in tracking task |
| G1 support | ✅ | ✅ |
| A3 support | Requires MJCF→USD conversion (non-trivial) | MJCF native (Agibot provides MJCF) |
| Manager-based API | Isaac Lab original | Same pattern, different backend |
| Physics debugging | PhysX state is opaque | MuJoCo state is fully inspectable via `mjData` |
| Multi-GPU training | ✅ | ✅ via `--gpu-ids` |
| macOS evaluation | ❌ (Linux only) | ✅ (eval only, no GPU training) |

The key advantage: Agibot provides MJCF models for MuJoCo through their developer program and open-source repositories (the Agibot X1 training code includes MJCF under `resources/robots/`). The A3 follows the same conventions. Loading an MJCF file into mjlab is trivial — no Omniverse USD conversion is needed.

### 4A.2  Installation

```bash
# Install mjlab (includes MuJoCo Warp)
pip install mjlab mujoco-warp

# Or from source for development
git clone https://github.com/mujocolab/mjlab.git
cd mjlab
pip install -e ".[dev]"
```

Verify the installation:

```bash
# Run the built-in demo (should open a viewer with humanoids)
uvx --from mjlab demo
```

### 4A.3  Adding the A3 Robot to mjlab

**Step 1 — Obtain the A3 MJCF model.**

If Agibot provides an MJCF file directly, use it. If only a URDF is available, convert it:

```bash
# MuJoCo ships a URDF→MJCF converter
python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('a3.urdf')
mujoco.mj_saveLastXML('a3.xml', model)
"
```

The resulting `a3.xml` must be validated. Key checks:

- All actuated joints have `<actuator>` entries with correct force limits and PD gains
- The `<compiler>` section specifies the correct mesh paths
- The `<option>` section sets gravity to `[0, 0, -9.81]` (Z-up)
- The pelvis/torso body is the floating base (no joint connecting it to worldbody, or a `freejoint`)

**Step 2 — Create an A3 robot configuration in mjlab.**

mjlab uses instance-based configuration (not class inheritance like Isaac Lab). Create a new config file following the existing G1 pattern:

```python
# src/mjlab/tasks/tracking/config/a3_tracking.py

from mjlab.envs import SceneCfg, EntityCfg
from mjlab.tasks.tracking.tracking_env_cfg import TrackingEnvCfg
from mjlab.managers import ActionsCfg, JointPositionActionCfg

# A3 robot configuration
a3_tracking_cfg = TrackingEnvCfg(
    scene=SceneCfg(
        robot=EntityCfg(
            mjcf_path="path/to/agibot_a3.xml",
            base_type="floating",
        ),
        num_envs=4096,
        dt=0.005,          # 200 Hz physics
    ),
    actions=ActionsCfg(
        joint_pos=JointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*"],    # all actuated joints
            scale=0.25,            # action scaling — tune per A3 actuators
        ),
    ),
    # The rest of the MDP (observations, rewards, terminations, events)
    # inherits from TrackingEnvCfg and works without modification.
    # The BeyondMimic reward functions are robot-agnostic — they operate
    # on joint errors and end-effector positions computed from the MJCF model.
)
```

**Step 3 — Register the task.**

```python
from mjlab.envs import register_mjlab_task

register_mjlab_task(
    task_id="Mjlab-Tracking-Flat-Agibot-A3",
    env_cfg=a3_tracking_cfg,
    rl_cfg=rsl_rl_ppo_cfg,    # same PPO hyperparameters as G1
)
```

### 4A.4  Retargeting Motions to the A3

GMR must be configured with a new robot definition for the A3. This requires:

1. **URDF/MJCF file** — The A3's kinematic model
2. **Joint mapping** — Which SMPL joints correspond to which A3 joints (GMR calls this "key body matching")
3. **Body proportions** — Limb lengths and mass distribution for the scaling step

Create the GMR robot config:

```python
# gmr/robots/agibot_a3.py

ROBOT_CONFIG = {
    "name": "agibot_a3",
    "urdf_path": "path/to/agibot_a3.urdf",
    "key_bodies": {
        "pelvis":          "pelvis",        # A3 pelvis link name
        "left_shoulder":   "left_shoulder",
        "right_shoulder":  "right_shoulder",
        "left_elbow":      "left_elbow",
        "right_elbow":     "right_elbow",
        "left_wrist":      "left_wrist",
        "right_wrist":     "right_wrist",
        "left_ankle":      "left_ankle",
        "right_ankle":     "right_ankle",
    },
    # Joint names and ordering must match the MJCF actuator order
    "joint_names": [
        # Legs (left then right)
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
        "left_knee", "left_ankle_pitch", "left_ankle_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
        "right_knee", "right_ankle_pitch", "right_ankle_roll",
        # Waist (A3 has multi-DOF flexible waist)
        "waist_yaw", "waist_roll", "waist_pitch",
        # Arms (left then right, 7 DOF each)
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
        "left_elbow", "left_wrist_yaw", "left_wrist_roll", "left_wrist_pitch",
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
        "right_elbow", "right_wrist_yaw", "right_wrist_roll", "right_wrist_pitch",
    ],
}
```

**Note**: The joint names above are placeholders. Teams must replace them with the actual joint names from the A3's URDF/MJCF, which Agibot provides through their developer program.

Then retarget:

```bash
python scripts/gvhmr_to_robot.py \
    --gvhmr_pred_file path/to/forehand_swing.pt \
    --robot agibot_a3 \
    --record_video
```

The A3's multi-DOF flexible waist is an advantage for retargeting — it provides more expressive torso rotation than the G1's single waist yaw, so retargeted table tennis swings may look more natural on the A3.

### 4A.5  Motion Preprocessing for mjlab

mjlab uses the same CSV → NPZ preprocessing pipeline as the Isaac Lab path, with the same WandB registry for motion management:

```bash
# From the mjlab repository (or unitree_rl_mjlab)
uv run scripts/tracking/csv_to_npz.py \
    --input-file a3_forehand_swing.csv \
    --output-name hope_a3_forehand \
    --input-fps 30 \
    --output-fps 50 \
    --render
```

The `--render` flag generates a preview video of the processed motion for verification.

### 4A.6  Training in mjlab

```bash
# Train A3 motion tracking policy
MUJOCO_GL=egl uv run train \
    Mjlab-Tracking-Flat-Agibot-A3 \
    --registry-name your-org/motions/hope_a3_forehand \
    --env.scene.num-envs 4096

# Multi-GPU training (if available)
MUJOCO_GL=egl uv run train \
    Mjlab-Tracking-Flat-Agibot-A3 \
    --gpu-ids "[0, 1]" \
    --env.scene.num-envs 4096

# Evaluate during training (in a separate terminal)
uv run play \
    Mjlab-Tracking-Flat-Agibot-A3-Play \
    --wandb-run-path your-org/mjlab/run-id
```

The training hyperparameters (PPO, network architecture, learning rate schedule) are identical to the Isaac Lab path described in Section 4.8. The BeyondMimic MDP formulation is physics-engine-agnostic — the same reward functions and observation terms work whether PhysX or MuJoCo computes the dynamics.

### 4A.7  Comparison: G1 on Isaac Lab vs. A3 on mjlab

The HOPE-specific MDP extensions (Section 4.6) — racket target observations, goal tracking reward, base repositioning, swing type selection — apply identically in mjlab. The manager-based API structure is the same:

```
Isaac Lab (G1):                      mjlab (A3):
  tracking_env_cfg.py                  tracking_env_cfg.py     (same API)
  mdp/commands.py                      mdp/commands.py         (same API)
  mdp/rewards.py                       mdp/rewards.py          (same API)
  mdp/observations.py                  mdp/observations.py     (same API)
  mdp/events.py                        mdp/events.py           (same API)
  mdp/terminations.py                  mdp/terminations.py     (same API)
  config/g1/agents/rsl_rl_ppo_cfg.py   config/a3/rsl_rl_ppo_cfg.py (same hyperparams)
```

The only differences are:

- The robot MJCF/USD file and actuator parameters
- The joint name mapping in GMR for retargeting
- The PD gains and action scale (robot-specific, must be tuned per actuator)

Policies from both backends export to the same ONNX format and deploy through the same `motion_tracking_controller` ROS 2 inference pipeline.

### 4A.8  A3 Middleware Bridging

As noted in the HOPE Motion Capture Reference Setup (Section 4.3), the A3 runs on Agibot's AimRT middleware natively, not ROS 2. For deployment, the ONNX policy can be:

- **Option A**: Loaded in a ROS 2 node (using the standard `motion_tracking_controller`), with AimRT's ROS 2 protocol bridge forwarding joint commands to the A3.
- **Option B**: Loaded directly in an AimRT node using the ONNX C++ runtime, bypassing ROS 2 entirely for lowest latency.

Both options use the same trained ONNX file — the deployment path does not affect the training pipeline.

---

## 5  Policy Export for Deployment

After training, the policy must be exported for real-time inference on the robot's onboard computer.

### 5.1  ONNX Export

BeyondMimic exports trained policies as ONNX models with embedded metadata (joint order, PD gains, action scaling). The `motion_tracking_controller` repository contains the export script:

```bash
# From the motion_tracking_controller repository
python scripts/export_onnx.py \
    --wandb_path=your-org/hope_wbc/run_id \
    --output_path=hope_forehand_policy.onnx
```

The exported ONNX file contains:

- The actor network (observation → action mapping)
- Metadata: joint names, joint order, PD stiffness/damping, action scale, observation normalization parameters
- The reference motion (embedded for motion tracking inference)

### 5.2  Sim-to-Sim Verification

Before deploying to hardware, verify the exported policy in a MuJoCo simulation:

```bash
# From the motion_tracking_controller repository
ros2 launch motion_tracking_controller mujoco.launch.py \
    policy_path:=$(pwd)/hope_forehand_policy.onnx
```

This runs the ONNX policy in a MuJoCo environment with the same G1 model, confirming that the export preserved the policy's behavior.

---

## 6  Summary of Complete Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│  Phase 1: Human Motion Acquisition                           │
│                                                              │
│  Video of table tennis swing                                 │
│       │                                                      │
│       ▼                                                      │
│  GVHMR → SMPL-X parameters (.pt)                             │
│  (body pose θ, orientation, translation per frame)           │
└──────────────────┬───────────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  Phase 2: Motion Retargeting                                 │
│                                                              │
│  GMR: SMPL-X → Target robot joint space                      │
│       ├── unitree_g1 (29 DOF)                                │
│       └── agibot_a3  (TBD DOF, multi-DOF waist)             │
│  Output: .pkl / .csv with robot joint trajectories           │
└──────────────────┬───────────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  Phase 3: Motion Preprocessing                               │
│                                                              │
│  csv_to_npz.py → Forward kinematics                          │
│  Computes body poses, velocities, accelerations              │
│  Uploads to WandB Registry as .npz                           │
└──────────────────┬───────────────────────────────────────────┘
                   │
          ┌────────┴────────┐
          │                 │
          ▼                 ▼
┌─────────────────┐  ┌─────────────────┐
│  Phase 4:       │  │  Phase 4A:      │
│  Isaac Lab      │  │  mjlab          │
│  + PhysX (G1)   │  │  + MuJoCo (A3)  │
│                 │  │                 │
│  Same MDP,      │  │  Same MDP,      │
│  same rewards,  │  │  same rewards,  │
│  USD assets     │  │  MJCF assets    │
└────────┬────────┘  └────────┬────────┘
         │                    │
         └────────┬───────────┘
                  │
                  ▼
┌──────────────────────────────────────────────────────────────┐
│  Export                                                      │
│                                                              │
│  ONNX model (actor network + metadata + reference motion)    │
│  Sim-to-sim verification in MuJoCo                           │
│  Ready for deployment via motion_tracking_controller         │
└──────────────────────────────────────────────────────────────┘
```

---

## 7  Known Limitations

1. **Reference motion dependency** — The trained policy can only produce swings whose style matches the reference motions it was trained on. Adding new stroke types (e.g., chop, lob, sidespin) requires new reference clips and retraining.

2. **SMPL-to-robot retargeting artifacts** — Target robots' proportions differ from a human's. The G1's shorter legs and lower center of mass cause specific artifacts; the A3's multi-DOF waist provides better torso expressiveness but introduces its own optimization challenges. GMR mitigates but does not eliminate these artifacts for either platform.

3. **Sim-to-real gap** — Despite domain randomization, the trained policy may exhibit degraded performance on hardware due to unmodeled dynamics (backlash, cable routing, thermal effects). The `motion_tracking_controller` deployment framework includes additional gain tuning for real-world deployment.

4. **Single-policy limitation** — The current formulation trains separate policies for forehand and backhand. The HITTER paper selects between them based on ball Y position. A more advanced approach would train a single multi-skill policy or use BeyondMimic's diffusion distillation to compose both skills.

5. **No spin response** — The WBC does not adjust the racket orientation to counteract ball spin. This matches the HITTER paper's no-spin assumption in the planner.

6. **Fixed strike timing** — The reference motion has a fixed strike time (t ≈ 0.86 s). The WBC must learn to temporally stretch or compress the swing to match the planner's `t_strike`, which may differ from the reference. BeyondMimic's adaptive sampling helps but does not fully address temporal generalization.

---

## References

- Liao, Q., Truong, T. E., Huang, X., Gao, Y., Tevet, G., Sreenath, K., & Liu, C. K. (2025). BeyondMimic: From Motion Tracking to Versatile Humanoid Control via Guided Diffusion. *arXiv:2508.08241v4*.
- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*.
- Araújo, J. P., Ze, Y., Xu, P., Wu, J., & Liu, C. K. (2025). Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking. *arXiv:2510.02252*.
- Ze, Y., et al. (2025). LATENT: Learning Athletic Humanoid Tennis Skills from Imperfect Human Motion Data. *arXiv:2603.12686*.
- mjlab: A Lightweight Framework for GPU-Accelerated Robot Learning. *arXiv:2601.22074*.
- Peng, X. B., et al. (2024). SMPLOlympics: Sports Environments for Physically Simulated Humanoids. *arXiv:2407.00187*.
- Ma, S., et al. (2024). P2ANet: A Large-Scale Benchmark for Dense Action Detection from Table Tennis Match Broadcasting Videos. *arXiv:2207.12730v2*.
- TT3D: Table Tennis 3D Reconstruction: https://cogsys-tuebingen.github.io/tt3d/
- BeyondMimic project page: https://beyondmimic.github.io/
- BeyondMimic training code (Isaac Lab): https://github.com/HybridRobotics/whole_body_tracking
- BeyondMimic training code (mjlab): https://github.com/mujocolab/mjlab
- Unitree RL mjlab (official Unitree mjlab integration): https://github.com/unitreerobotics/unitree_rl_mjlab
- BeyondMimic deployment code: https://github.com/HybridRobotics/motion_tracking_controller
- MuJoCo Warp: https://github.com/google-deepmind/mujoco_warp
- GMR retargeting: https://github.com/YanjieZe/GMR
- GVHMR pose estimation: https://github.com/zju3dv/GVHMR
- LAFAN1 retargeted dataset: https://huggingface.co/datasets/lvhaidong/LAFAN1_Retargeting_Dataset
- Unitree G1 USD assets (official): https://huggingface.co/datasets/unitreerobotics/unitree_model
- Unitree G1 URDF (ROS): https://github.com/unitreerobotics/unitree_ros
- Unitree RL Lab (Isaac Lab, supports USD and URDF): https://github.com/unitreerobotics/unitree_rl_lab
- Companion documents:
  - *HOPE Motion Capture System Reference Setup for Ping-Pong Arena*
  - *HOPE 7DOF Racket Model-based Planner Reference Setup*
  - *HOPE Hardware Deployment Reference Setup*
