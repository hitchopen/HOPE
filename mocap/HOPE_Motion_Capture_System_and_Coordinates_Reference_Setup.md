# Motion Capture System Reference Setup for HOPE Ping-Pong Arena

**v0.4** — 2026-06-17

> **Preserved reference document.** This predates the current HOPE
> stack and is kept for arena-build background and provenance. The authoritative
> frame and topic contract is [`mocap/README.md`](README.md); the live driver
> shipped in this repo is the vendored
> [`hope_ws/src/vrpn_mocap/`](../hope_ws/src/vrpn_mocap). Index:
> [`REFERENCE_DOCS.md`](../REFERENCE_DOCS.md).

---

## 1  Compatible Motion Capture Systems

The original reference setup used an OptiTrack system. This reference design document creates a reference system compatible with several mainstream motion capture brands — principally **OptiTrack**, **Vicon**, and **青瞳视觉 (CHINGMU)** — and is expected to extend to the other marker-based brands supported by the `motion_capture_tracking` library, including Qualisys, NOKOV, VRPN, FZMotion, and Motion Analysis. These systems differ in their cameras and vendor software — OptiTrack pairs Motive with the NatNet protocol, Vicon uses Vicon Tracker, and Chingmu uses CMTracker/CMAvatar streaming over VRPN, TrackD, DTrack, OpenVR, and its native LiveStream, each shipping C/C++, Python, and ROS SDKs — but this design unifies them under a single ROS 2 REP 103 coordinate frame and `/poses` + `/tf` topic interface. How each system's data is converted into ROS 2 messages is covered in Section 6 (the Chingmu path is in Section 6.5).

Specifically, that original setup installed:

- OptiTrack **Motive v3.4** (camera management and tracking software)
- **NatNet SDK v4.4** (streaming protocol for delivering tracking data over the network)
- **9 OptiTrack cameras** operating at **360 Hz** with millimeter-level accuracy

For the HOPE reference design, the minimum recommended specification is:

- At least **6 cameras** (8–12 preferred) arranged to cover the full table volume plus a 1.5 m margin on each player's side
- Camera frame rate **≥ 120 Hz** (240–360 Hz recommended for competitive ball tracking at speeds exceeding 5 m/s)
- Sub-millimeter reconstruction accuracy within the tracking volume

---

## 2  Setup of the Environment Markers and Coordinate Frames

To avoid calibration error and potential platform movements, the most straightforward approach is to anchor the motion capture system origin directly on the Ping-Pong Table (PPT). However, a common source of confusion is that the default coordinate frame in OptiTrack (Y-up) differs from both ROS 2 (Z-up, REP 103) and Vicon (Z-up). **In this reference design, we adopt the ROS 2 REP 103 convention as the canonical world frame.**

### 2.1  Canonical World Frame (ROS 2 REP 103)

The world frame origin is placed at the **near-side left corner of the table surface**, from Player One's (P1's) perspective:

| Axis | Direction | Range on table surface |
|------|-----------|------------------------|
| **X** | Forward — toward Player Two (P2) along the table length | 0 → +2.74 m |
| **Y** | Left — along the table width, from P1's perspective | 0 → −1.525 m |
| **Z** | Up — vertical | 0 = table surface |

This convention is **identical** to the frame used in the companion document *HOPE 7DOF Racket Model-based Planner Reference Setup*, ensuring that all ball trajectory predictions, racket target computations, and ROS 2 topic messages share a single consistent coordinate system.

Key landmarks in this frame:

| Landmark | X (m) | Y (m) | Z (m) |
|----------|-------|-------|-------|
| Origin (P1 near-side left corner) | 0.0 | 0.0 | 0.0 |
| Net center line | 1.37 | −0.7625 | 0.0 |
| P1 half center | 0.685 | −0.7625 | 0.0 |
| P2 half center | 2.055 | −0.7625 | 0.0 |
| Floor directly below origin | 0.0 | 0.0 | −0.76 |
| Virtual hitting plane (planner) | x = x_hit ≈ 0.0 | — | — |

The table surface occupies the region: `x ∈ [0, 2.74]`, `y ∈ [−1.525, 0]`, `z = 0`.

### 2.2  Correcting OptiTrack's Default Coordinate Frame

OptiTrack Motive defaults to a **Y-up** coordinate system, which is incompatible with ROS 2's Z-up convention. To correct this:

1. In Motive, navigate to **Edit → Settings → Streaming** (or open the Data Streaming pane).
2. Under **Advanced Network Options**, change **Up Axis** from "Y Axis" to **"Z Axis"**.
3. Orient the calibration ground plane so that the calibration square's long edge aligns with the desired X-axis direction (toward P2). This sets the world frame orientation during the calibration wand procedure.

Vicon Tracker defaults to Z-up and generally requires no axis correction. However, verify during ground-plane calibration that the X-axis points along the table length toward P2.

For **青瞳 (Chingmu) CMTracker**, the world frame is fixed by the L-frame / calibration-square placement during the ground-plane calibration step, and the up axis is configurable in the streaming/export settings. Set the up axis to **Z** so that streamed data matches the ROS 2 REP 103 convention, and place the calibration square so its long edge points along the table length toward P2. If a particular CMTracker installation can only stream in a Y-up or otherwise non-REP-103 frame, do **not** attempt to re-calibrate around it — instead apply the fixed axis conversion in the ROS 2 bridge node described in Section 6.5.3.

### 2.3  Table Rigid Body Definition (PPT)

Reflective markers or retroreflective patches (at least 10 mm × 10 mm) are attached to the **outer frame** of the PPT. Collectively, these markers form one rigid body defined in Motive (or Vicon Tracker) as the asset **"PPT"**.

Placement requirements:

- Attach **at least 4 markers** in an asymmetric configuration on the table frame's outer edges.
- Place markers where they are visible from the majority of camera positions and will not be occluded by players, the net, or the ball during play.
- **Do not place markers on the playing surface** — they would interfere with ball bounce dynamics and could be confused with the ball marker.

The PPT rigid body's pivot point must be set to the **near-side left corner of the table surface** (the origin), with the body's local frame aligned with the world frame axes defined above. After calibration, the PPT rigid body should report identity pose (position ≈ [0, 0, 0], orientation ≈ [0, 0, 0, 1]) when the table is stationary and properly aligned.

The PPT rigid body serves two purposes:

1. **Origin anchor** — It defines the world frame origin for all other tracked objects.
2. **Table movement detection** — If the table is bumped or shifted during play, the PPT pose will deviate from identity, allowing the planner to compensate or flag a re-calibration need.

---

## 3  Tracked Object Taxonomy

The motion capture system tracks exactly **three categories** of objects. The racket/paddle is explicitly **not** one of them.

### 3.1  Racket Exclusion Policy — Paddle Is NOT Tracked by Motion Capture

**The motion capture system must not track the ping-pong racket (paddle).** No reflective markers or tracking assets should be placed on or attached to the racket. This is a deliberate architectural decision aligned with the HOPE competition design:

**Rationale:**

1. **Forward kinematics inference.** The humanoid must infer its paddle's 6-DOF pose (position and orientation) from its own proprioceptive state — joint encoder readings plus the tracked `base_link` position — using forward kinematics through its arm kinematic chain. This tests the robot's internal body model accuracy, which is a core competency for any real-world manipulation task.

2. **No external sensing of end-effector.** In this architecture, the whole-body controller (WBC) receives a desired racket state `(p_intercept, v_racket, n_racket, t_strike)` from the planner and uses its RL policy to drive the 7-DOF arm to achieve that state. The controller never receives measured racket pose from the motion capture system. The racket's actual position is an emergent property of the robot's joint configuration, not an externally measured quantity.

3. **Competition fairness.** Tracking the racket externally would provide closed-loop feedback that bypasses the robot's control challenge. The HOPE competition requires each team's humanoid to demonstrate autonomous paddle control through its own kinematic model.

4. **Practical reliability.** Markers on a rapidly swinging paddle (arm speeds exceeding 3 m/s) suffer from severe occlusion, motion blur, and centripetal marker detachment. Excluding the paddle from tracking eliminates a fragile sensing link.

**Enforcement:** During competition setup, referees verify that no retroreflective material is present on the racket, the robot's hand, or the wrist link beyond the last tracked rigid-body marker on the robot's torso/pelvis.

**Cross-references:** The companion *HOPE 7DOF Racket Model-based Planner Reference Setup* (Section 0.1) documents that the planner outputs a desired racket state without any racket pose feedback. The companion *HOPE WBC Simulation Training Reference Setup* (Section 2.8 — Racket Mount Kinematics) documents the complete FK chain from `base_link` through the 7-DOF arm to the 3D-printed fixed racket mount, including the `T_mount` calibration procedure that ensures the simulation model matches the physical bracket.

### 3.2  Tracked Objects Summary

| Object ID | Asset type | What is tracked | Markers | Tracking mode |
|-----------|-----------|-----------------|---------|---------------|
| **PPT** | Rigid body (vendor-tracked) | Ping-pong table frame | ≥ 4 asymmetric on table outer frame | Vendor 6-DOF |
| **P1, P2, ...** | Rigid body (vendor-tracked) | Humanoid `base_link` | ≥ 4 asymmetric on torso/pelvis plate | Vendor 6-DOF |
| **Ball** | Single unlabeled marker | Ping-pong ball center | 1 retroreflective marker or tape on ball | Frame-to-frame point tracking |

No other objects should carry retroreflective markers within the tracking volume during play. Stray markers cause false associations and corrupt ball tracking.

---

## 4  Setup of the Humanoid base_link Markers

In this reference design, the humanoid infers its paddle's 6-DOF pose using **forward kinematics from `base_link`** through the arm's kinematic chain. Therefore, the only spatial anchor the motion capture system provides for each robot is its `base_link` location.

### 4.1  base_link Convention — General Principles

There is no universal standard for where a humanoid robot's `base_link` is defined. The convention varies by manufacturer, URDF authoring choices, and the robot's intended control architecture. However, three common patterns have emerged across the industry:

**Pattern A — Pelvis root (most common for bipedal locomotion).** The `base_link` is the pelvis link, located at the center of the hip plate where the leg kinematic chains branch downward and the torso chain branches upward. This is the standard for RL-trained locomotion controllers because the pelvis is the most stable reference during walking — it is the floating-base frame in whole-body dynamics. Used by Unitree G1, Unitree H1, Boston Dynamics Atlas, Agility Digit, and most humanoids trained in Isaac Lab or MuJoCo.

**Pattern B — Torso/chest root.** Some platforms place `base_link` at the upper torso or chest, above the waist joint(s). This is less common for bipedal locomotion (the pelvis is more dynamically stable) but can appear in manipulation-focused configurations where the arms are the primary concern and the legs are treated as a mobile base subsystem.

**Pattern C — Waist joint root.** A compromise where `base_link` sits at the waist joint itself — the interface between legs and torso. In many simple designs this is co-located with the pelvis origin (Pattern A). In robots with multi-DOF waist articulation, the waist joint is above the pelvis, and choosing it as `base_link` places the root between the two subsystems.

**For the HOPE competition, the critical requirement is:**

> The `base_link` must be the root of the forward kinematics chain that reaches the paddle-holding hand. The planner outputs a desired racket state in the world frame; the robot's WBC must compute the arm joint trajectory from `base_link` to the paddle that achieves it.

This means the complete FK chain is: `world → base_link (from mocap) → waist joints → shoulder → elbow → wrist → paddle tip (from joint encoders)`. Every joint between `base_link` and the paddle must be instrumented with encoders whose readings are available to the robot's control software.

### 4.2  Unitree G1

The Unitree G1 is the primary reference platform for HOPE.

| Property | Value |
|----------|-------|
| `base_link` location | **Pelvis** — center of lower torso at the waist, approximately at the intersection of the two hip yaw joint axes |
| Pattern | A (pelvis root) |
| Standing pelvis height | ~0.78 m above floor (z ≈ +0.02 m in HOPE frame) |
| Robot overall height | 1.27–1.32 m |
| Weight | ~35 kg with battery |
| Total DOF | 23 (base) to 43 (EDU with dexterous hands) |
| Arm DOF | 7 per arm |
| Waist DOF | 1 (yaw) |
| URDF source | `github.com/unitreerobotics/unitree_ros` → `robots/g1_description` |
| Middleware | ROS 2 natively supported |

The kinematic tree branches from the pelvis:

```
pelvis (base_link)
├── left_hip_yaw_joint  → left leg (6 DOF)
├── right_hip_yaw_joint → right leg (6 DOF)
└── waist_yaw_joint     → torso → shoulder → elbow → wrist (7 DOF per arm)
```

**Marker placement:** Attach a 4-marker asymmetric cluster on a rigid plate secured to the pelvis shell. Set the rigid body pivot point in Motive to the pelvis origin (center of the hip plate). If markers are on the outer shell surface, calibrate a static TF offset of a few centimeters in Z.

### 4.3  Agibot Expedition A3

The Expedition A3 is Agibot's next-generation athletic humanoid, demonstrated performing aerial kung fu maneuvers. As of March 2026 it is pre-production with mass production planned for later in 2026.

| Property | Value |
|----------|-------|
| `base_link` location | **To be confirmed** — likely pelvis (Pattern A), but the flexible waist may warrant Pattern C |
| Standing height | Full-size (~1.75 m, estimated from video) |
| Weight | Not publicly disclosed |
| Total DOF | Not publicly disclosed; described as "highly anthropomorphic full-body degrees of freedom" |
| Arm DOF | Not publicly disclosed (7 DOF per arm expected, based on Agibot platform lineage) |
| Waist DOF | **Multi-DOF flexible waist** — a key distinguishing feature engineered to mirror the human range of motion, enabling rotation and swaying for complex whole-body movements |
| URDF source | Not publicly available as of March 2026 |
| Middleware | **AimRT** (Agibot's native C++20 runtime); supports ROS 2 protocol bridging |

**Key considerations:**

1. **Flexible waist implications.** The A3's multi-DOF flexible waist is specifically engineered for the kind of torso rotation and weight transfer that table tennis demands. However, if the waist has 2–3 DOF (pitch, roll, yaw), the choice of where `base_link` sits relative to the waist joints significantly affects the FK chain length. For ping-pong, the waist DOFs contribute directly to racket positioning (waist rotation extends the arm's effective reach and angle), so `base_link` should ideally be **below** the waist (Pattern A) to include waist DOFs in the paddle FK chain.

2. **Pre-production status.** Teams planning to use the A3 for HOPE should coordinate directly with Agibot to obtain the URDF and confirm the `base_link` convention, `base_link` height, and the complete joint chain from `base_link` to the paddle-holding hand. The open-source Agibot X1 training repository (`github.com/AgibotTech/agibot_x1_train`) contains URDF files under `resources/robots/` and may serve as a reference for Agibot's kinematic tree conventions.

3. **Middleware bridging.** The A3 runs on AimRT natively, not ROS 2. AimRT supports ROS 2 as one of several communication protocols (alongside HTTP, gRPC, MQTT, and Zenoh). For the HOPE architecture, two integration approaches are available:
   - **Approach 1 (recommended):** Run the HOPE planner as a ROS 2 node; bridge the `RacketCommand` topic into AimRT where the A3's native WBC consumes it. The `base_link` pose from motion capture still flows through ROS 2 → AimRT.
   - **Approach 2:** Run the planner within AimRT directly, subscribing to the motion capture data via AimRT's ROS 2 protocol support.

### 4.4  Competition Registration Requirements

Each team must declare the following during HOPE competition registration. This information is needed to verify that the motion capture system, planner, and WBC are correctly integrated for their specific humanoid platform.

| Item | Description | Example (Unitree G1) |
|------|-------------|---------------------|
| **Robot model** | Manufacturer and model designation | Unitree G1 EDU |
| **`base_link` URDF link name** | The exact link name in the URDF that corresponds to `base_link` | `pelvis` |
| **`base_link` physical location** | Description of where the link origin sits on the physical robot | Center of hip plate, at intersection of hip yaw axes |
| **`base_link` pattern** | Which convention (A/B/C from Section 4.1) | Pattern A (pelvis root) |
| **Standing `base_link` height** | Height of `base_link` origin above the floor when standing in nominal pose | 0.78 m (z ≈ +0.02 m in HOPE frame) |
| **Mocap-to-URDF static offset** | Translation [dx, dy, dz] from the mocap marker cluster centroid to the URDF `base_link` origin | [0.0, 0.0, −0.03] m (markers on outer shell, 3 cm above pelvis origin) |
| **Arm DOF count** | Number of actuated joints from `base_link` to paddle grip, including waist | 1 waist + 7 arm = 8 DOF |
| **Middleware** | ROS 2 native, AimRT with ROS 2 bridge, or other | ROS 2 native |
| **URDF availability** | Public URL or "provided to organizers under NDA" | `github.com/unitreerobotics/unitree_ros` |

The static offset (mocap-to-URDF) is published as a `static_transform_publisher` in the team's launch file:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '--x', '0.0', '--y', '0.0', '--z', '-0.03',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'P1_mocap',
        '--child-frame-id', 'P1_base_link'
    ],
)
```

### 4.5  What the Robot Knows vs. What Motion Capture Provides

| Information | Source | Used by |
|-------------|--------|---------|
| Ball position [x, y, z] at 360 Hz | Motion capture → ROS 2 topic | Planner (Stages 1–3) |
| Humanoid `base_link` 6-DOF pose | Motion capture → ROS 2 topic | WBC (Stage 4) for base position commands |
| Table frame (PPT) pose | Motion capture → ROS 2 topic | Planner (origin reference / drift detection) |
| Paddle 6-DOF pose | **Forward kinematics** from joint encoders + `base_link` | WBC internal state; **not** from motion capture |
| Paddle desired state | Planner output (Stage 3) | WBC (Stage 4) as tracking target |

---

## 5  Ball Tracking Configuration

The ping-pong ball is tracked as a **single unlabeled marker** — a single retroreflective point visible to the motion capture system but not associated with any rigid body definition.

### 5.1  Ball Preparation

- Attach a **6–9 mm retroreflective marker** to the ball, or wrap the ball in retroreflective tape.
- The marker must be small enough to avoid significantly altering the ball's mass (2.7 g) or aerodynamic properties.
- Use a single marker per ball. Multiple markers on one ball would create ambiguity in the point cloud.

### 5.2  Tracking Mode

In `motion_capture_tracking`, the ball is tracked using the `librigidbodytracker` frame-to-frame point tracking mode. This works by nearest-neighbor association of an unlabeled point cloud marker between successive frames. Unlike rigid body tracking (which requires ≥ 3 markers for 6-DOF), this mode tracks a single point and provides only 3-DOF position `[x, y, z]`.

Configuration in `motion_capture_tracking` `cfg.yaml`:

```yaml
type: "optitrack"           # or "optitrack_closed_source", "vicon", etc.
hostname: "MOTIVE_PC_IP"

robot_types:
  ball:
    motion_capture:
      tracking: "librigidbodytracker"   # frame-to-frame single marker
      initial_position: [1.37, -0.7625, 0.2]  # approximate initial position
      dynamics:
        max_velocity: 10.0     # m/s, upper bound for association
```

### 5.3  Ball Spin (Future Extension)

In the current reference design, ball spin is ignored. The ball is tracked as a single point with position `[x, y, z]` only. The HOPE planner's aerodynamic model uses translational drag but does not model Magnus force (spin-induced lift).

Future extensions may investigate whether finer marker patterns on the ball (e.g., multiple small patches in a known geometric pattern) could enable spin estimation. This would require upgrading to rigid-body tracking of the ball (≥ 3 markers) and depacketizing the orientation quaternion as angular velocity. Such an extension is outside the scope of the current reference design.

### 5.4  Ball Coating Types and Expected Tracking Performance

Two retroreflective ball preparations are common in practice. They behave very differently under the single-marker tracking strategy of this reference design.

![Two reflective ping-pong ball types: (left) fully coated retroreflective sphere; (right) white ball with multiple retroreflective dot patches.](two_ball_types.jpeg)

**Left — fully coated retroreflective ball.** The entire surface is retroreflective (the "wrap the ball in retroreflective tape" option of Section 5.1). Each camera sees a single bright blob whose centroid projects to the sphere's geometric center, so the ball reconstructs as **one 3-DOF point at the true ball center**. This is exactly what the single-marker frame-to-frame tracker (Section 5.2) expects.

**Right — white ball with multiple retroreflective patches.** Several discrete patches are applied to the surface. Each camera-visible patch reconstructs as its **own** unlabeled point, so a single ball injects multiple simultaneous markers into the point cloud — the ambiguity Section 5.1 explicitly warns against.

Expected performance under the current (single-marker) strategy:

| Criterion | Fully coated ball (left) | Multi-patch ball (right) |
|-----------|--------------------------|--------------------------|
| Points produced per ball | 1 (single blob) | Several (one per visible patch) |
| Reported position | True ball **center** | A **surface patch**, ~radius (≈20 mm) off-center |
| Effect of spin | **Spin-invariant** — centroid stays at center | Tracked point jumps between patches as patches appear/occlude; apparent position jumps up to a ball diameter |
| Frame-to-frame association | Robust (one unambiguous point) | Fragile; breaks down above ~5 m/s as nearest-neighbor matches the wrong patch |
| Compatibility with Section 5.2 tracker | ✅ Direct | ❌ Violates single-marker assumption |
| Suitable tracking mode | Single unlabeled marker (current design) | Rigid-body / constellation tracking, ≥3 markers (Section 5.3, out of scope) |

**Conclusion.** The fully coated ball is the recommended preparation for HOPE: it yields a single, spin-invariant, center-accurate point that the reference design tracks reliably. The multi-patch ball is **not** reliably tracked by the current single-marker pipeline — it would require the rigid-body spin-estimation extension of Section 5.3, which is fragile for a small, fast, spinning sphere (≥3 co-visible patches of known geometry cannot be guaranteed each frame) and is outside the current scope.

---

## 6  Streaming Data to ROS 2

Neither OptiTrack nor Vicon directly publishes tracking data in ROS 2 message format. A **translator driver** on the ROS 2 Linux host converts the vendor protocol into ROS 2 topics.

### 6.1  Network Architecture

```
┌─────────────────────────┐       NatNet (UDP)       ┌────────────────────────────────────┐
│   Windows PC             │  ─────────────────────▶  │   Linux PC (ROS 2 Jazzy)            │
│                          │    multicast/unicast      │                                     │
│   OptiTrack Motive 3.4   │    same LAN subnet       │   motion_capture_tracking            │
│   (cameras, solving,     │                          │   (NatNet depacketizer →             │
│    rigid bodies)         │                          │    /poses, /tf, point cloud)         │
│                          │                          │                                     │
│                          │                          │   HOPE Planner (Stages 1–3)          │
│                          │                          │   WBC (Stage 4)                      │
└─────────────────────────┘                           └────────────────────────────────────┘
            same LAN switch
```

Both machines must be on the same subnet. Motive broadcasts NatNet frames at the camera rate (e.g., 360 Hz), and `motion_capture_tracking` on the Linux side depacketizes them and publishes standard ROS 2 messages. No special bridging or VPN is needed — plain UDP networking.

The same architecture applies to Vicon: replace Motive with Vicon Tracker, and set `type: "vicon"` in the `motion_capture_tracking` configuration. The ROS 2 topic interface remains identical.

For robots using AimRT (e.g., Agibot Expedition A3), the `RacketCommand` and `base_link` pose topics can be bridged from ROS 2 into AimRT using AimRT's built-in ROS 2 protocol support (see Section 4.3).

The 青瞳 (Chingmu) system follows the same physical topology — a Windows PC running CMTracker solves the cameras and streams over the LAN to the Linux ROS 2 host — but uses a different on-the-wire protocol (VRPN / LiveStream rather than NatNet). The conversion of that stream into the ROS 2 topics below is detailed in Section 6.5.

### 6.2  Recommended ROS 2 Driver

The `mocap4ros2_optitrack` driver (MOCAP4ROS2 Project) is a common OptiTrack-only option. However, that package has limitations for this reference design:

- It only supports OptiTrack (no Vicon compatibility).
- It does **not** perform coordinate frame conversion (passes through NatNet data as-is).
- It does not support single-marker tracking for the ball.
- The closed-source NatNet SDK it depends on is x86-64 Linux only.

**For the HOPE reference design, we recommend `motion_capture_tracking`** (IMRCLab):

- Repository: https://github.com/IMRCLab/motion_capture_tracking
- Supports **OptiTrack, Vicon, Qualisys, VRPN, NOKOV, FZMotion, and Motion Analysis** through a unified interface.
- Provides `librigidbodytracker` for **single unlabeled marker tracking** (ping-pong ball).
- Released for both **ROS 2 Humble** (Ubuntu 22.04) and **ROS 2 Jazzy** (Ubuntu 24.04) via apt:
  ```bash
  sudo apt install ros-jazzy-motion-capture-tracking
  ```
- Publishes via **tf2** and a `/poses` topic with configurable QoS.

### 6.3  Motive Streaming Settings Checklist

Before streaming from Motive to the ROS 2 host, verify these settings in the Data Streaming pane:

| Setting | Required value | Notes |
|---------|---------------|-------|
| Enable NatNet | ✅ Enabled | Must be on to stream |
| Transmission Type | Multicast (or Unicast) | Unicast preferred for bandwidth control |
| Up Axis | **Z Axis** | Critical — aligns with ROS 2 Z-up convention |
| Labeled Markers | OFF | Not needed; reduces packet size |
| Unlabeled Markers | **ON** | Required for ball tracking |
| Marker Sets | OFF | Not needed |
| Rigid Bodies | **ON** | Required for PPT and humanoid base_link |
| Skeletons | OFF | Not needed |
| Command Port | 1510 (default) | |
| Data Port | 1511 (default) | |

### 6.4  Expected ROS 2 Topics

After configuration, the following ROS 2 topics are available:

| Topic | Message type | Content | Rate |
|-------|-------------|---------|------|
| `/poses` | `geometry_msgs/PoseArray` | All tracked rigid bodies and custom-tracked markers | 360 Hz |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree: world → PPT, world → P1, world → P2, world → Ball | 360 Hz |

The planner subscribes to the ball position from `/poses` or `/tf` and produces `RacketCommand` messages as described in the planner document. The WBC subscribes to both the `RacketCommand` and the humanoid's `base_link` transform from `/tf`.

### 6.5  Converting 青瞳 (Chingmu) Motion Capture Data to ROS 2

The Chingmu server streams over **VRPN**, carrying all three HOPE tracked categories on one connection — but it addresses them in two different ways, and that difference dictates which ROS 2 client to use:

| Tracked object | What you read | How VRPN addresses it |
|----------------|---------------|-----------------------|
| **PPT** (table) | 6-DOF pose (position + orientation) | its **own VRPN sender name** (e.g. `PPT`) |
| **P1, P2** (`base_link`) | 6-DOF pose (position + orientation) | each its **own VRPN sender name** |
| **Ball** | 3-DOF position `[x, y, z]`; orientation is identity and ignored | a **marker = sensor ID** under a shared sender (no dedicated sender name) |

Chingmu confirms that each rigid body (PPT, P1, P2) can be assigned a distinct VRPN **sender name**, but the ball is a **marker**, not a rigid body, so it is delivered as a **sensor ID** with no sender name of its own. No separate point-cloud SDK is required — everything is on the one VRPN connection — but the ROS 2 client must be able to resolve **sensor IDs**, not just sender names.

> **Driver choice.** Use **`vrpn_mocap`** with `multi_sensor: true`: it publishes one topic per sensor ID and so exposes both the named rigid bodies and the ball marker. Do **not** use `motion_capture_tracking`'s VRPN backend (libmotioncapture) here — it keys trackers by sender name only and discards the sensor ID, so it cannot separate the ball marker. Chingmu's own `ChingMuVrpnRos` parses the stream correctly but targets ROS 1; it is referenced only for validation, not used in HOPE's ROS 2 stack.

### 6.5.1  ROS 2 VRPN Client — `vrpn_mocap`

On the ROS 2 Jazzy host, run `vrpn_mocap`, a native ROS 2 VRPN client, pointed at the Chingmu server (the CMTracker / `MCServer` PC), with `multi_sensor: true` so that each marker sensor ID gets its own topic:

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=CHINGMU_SERVER_IP port:=3883
```

with a parameter file:

```yaml
/vrpn_mocap_client:
  ros__parameters:
    server: "CHINGMU_SERVER_IP"   # CMTracker / MCServer PC
    port: 3883                    # Chingmu VRPN port
    frame_id: "world"
    multi_sensor: true            # one topic per sensor ID — required for the ball marker
    update_freq: 100.0
    refresh_freq: 1.0
```

`vrpn_mocap` discovers every VRPN sender and publishes a `geometry_msgs/PoseStamped` per sender, and — with `multi_sensor: true` — per sensor index, under the `/vrpn_mocap` namespace:

```
/vrpn_mocap/<sender>/pose_id_<sensor_id>      geometry_msgs/PoseStamped
```

Assign and record fixed VRPN **sender names** for **PPT**, **P1**, **P2** in CMTracker so the planner and WBC subscribe to the right topics. Each rigid body is a single-sensor sender, so it appears as `pose_id_0` carrying full `pose.position` + `pose.orientation` (a 6-DOF measurement):

```
/vrpn_mocap/PPT/pose_id_0
/vrpn_mocap/P1/pose_id_0
/vrpn_mocap/P2/pose_id_0
```

Add a small ROS 2 relay node to merge these into the single `/poses` `PoseArray` and to (re)broadcast `world → PPT`, `world → P1`, `world → P2` on `/tf`, matching Section 6.4 exactly.

### 6.5.2  The Ping-Pong Ball as a 3-DOF Marker

The ball is a **marker**, not a rigid body, so Chingmu delivers it as a **sensor ID under a shared marker sender** — it has no dedicated sender name. With `multi_sensor: true` (Section 6.5.1), `vrpn_mocap` exposes it as a per-sensor topic:

```
/vrpn_mocap/<marker_sender>/pose_id_<ball_sensor_id>   geometry_msgs/PoseStamped
# pose.position is the valid ball position [x, y, z]; pose.orientation is identity and ignored
```

Confirm the marker sender name and the ball's sensor index with Chingmu, and verify with `ros2 topic list`. The ball must be the **single** unlabeled marker required by Section 5 — stray markers would share the marker sender and make the sensor index ambiguous.

For the planner, subscribe to that topic and use only the position fields. The shipped stack does exactly this with the `hope_bringup` `pose_to_posearray` node, which aggregates the tracker topic(s) into the shared `/poses` `PoseArray` the planner (and the calibration recorder `hope_bag_to_csv`, via `ros2 bag record /poses`) consume. Alternatively a trivial relay can republish it as `geometry_msgs/PointStamped` on `/ball/point`:

```python
# ball_pose_to_point.py  (ROS 2, sketch)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

class BallRelay(Node):
    def __init__(self):
        super().__init__('ball_pose_to_point')
        self.pub = self.create_publisher(PointStamped, '/ball/point', 10)
        self.create_subscription(
            PoseStamped, '/vrpn_mocap/<marker_sender>/pose_id_<ball_sensor_id>', self.cb, 10)

    def cb(self, msg: PoseStamped):
        out = PointStamped()
        out.header = msg.header          # frame_id = 'world'
        out.point = msg.pose.position    # orientation discarded
        self.pub.publish(out)
```

If CMTracker cannot stream directly in the HOPE world frame, apply the coordinate conversion of Section 6.5.3 inside this relay (and on the rigid-body poses) before republishing.

### 6.5.3  Coordinate Conversion to the HOPE World Frame

All Chingmu output must arrive in the canonical REP 103 Z-up frame of Section 2.1. The preferred approach is to configure CMTracker to stream **Z-up** with the calibration square aligned toward P2 (Section 2.2), in which case no software rotation is needed. If an installation can only stream in a right-handed **Y-up** frame, apply this fixed rotation in the bridge node (or as a `static_transform_publisher`) before publishing:

```
# Right-handed Y-up  →  REP 103 Z-up
x_ros =  x_mocap
y_ros = -z_mocap
z_ros =  y_mocap
```

Verify the handedness of the source frame empirically before trusting any conversion: place a marker at a known table landmark (e.g., the net-center line, `x = 1.37, y = −0.7625`) and confirm the published coordinates match Table in Section 2.1. A mirrored axis indicates a left-handed source frame, which requires negating the appropriate component rather than the rotation above.

### 6.5.4  CMTracker Streaming Settings Checklist

| Setting | Required value | Notes |
|---------|---------------|-------|
| VRPN streaming | ✅ Enabled | Single stream carries rigid bodies **and** markers |
| VRPN port | 3883 (default) | Match the `port` param of the ROS 2 client |
| ROS 2 client | `vrpn_mocap`, `multi_sensor: true` | libmotioncapture VRPN backend is unsuitable — it drops sensor IDs |
| Rigid bodies (PPT, P1, P2) | Each a distinct VRPN **sender name** | Each → `/vrpn_mocap/<name>/pose_id_0` (6-DOF) |
| Ball | A **marker = sensor ID** under a shared sender | → `/vrpn_mocap/<marker_sender>/pose_id_<id>` (position valid, orientation identity) |
| Single marker only | No stray markers in volume | Extra markers make the ball's sensor index ambiguous |
| Up axis | **Z** | Aligns with ROS 2 Z-up; else convert per 6.5.3 |
| Server / Linux host subnet | Same LAN subnet | Plain UDP, as in Section 6.1 |

Once the ROS 2 client is running, `ros2 topic list` shows a `/vrpn_mocap/<sender>/pose_id_<id>` per object (relay into `/poses` + `/tf` and `/ball/point` as desired, per Section 6.4), and the planner and WBC require no vendor-specific changes.

---

## 7  Integration with the HOPE Planner

The companion planner document (*HOPE 7DOF Racket Model-based Planner Reference Setup*) consumes ball position data published by `motion_capture_tracking` and produces racket target commands. The data flow through the complete system is:

```
Motion Capture System (360 Hz)                         Humanoid (proprioceptive)
  │                                                      │
  ├── Ball [x,y,z] (single marker) ──▶ HOPE Planner     │
  │                                     Stages 1–3       │
  ├── PPT 6-DOF ──▶ origin validation     │              │
  │                                        ▼              │
  └── P1 base_link 6-DOF ──────────▶ WBC (Stage 4) ◀── RacketCommand
                                           │              (p_intercept,
                                           │               v_racket,
                                           ▼               n_racket,
                                     Joint commands        t_strike)
                                     (varies by platform)
                                           │
                                           ▼
                                     Paddle pose
                                     (inferred via FK from
                                      base_link + joint encoders,
                                      NOT measured by mocap)
```

The planner operates entirely in the HOPE canonical world frame defined in Section 2.1. The `motion_capture_tracking` driver delivers positions already in this frame (assuming Motive's Up Axis is set to Z and the calibration ground plane was aligned with the table).

---

## 8  Summary

The HOPE motion capture reference system tracks exactly three categories of objects:

1. **PPT** — the ping-pong table, providing the world frame origin and drift detection.
2. **P1, P2** — humanoid `base_link` poses, providing the spatial anchor for each robot. The `base_link` definition varies by manufacturer (Section 4); each team declares theirs at registration.
3. **Ball** — the ping-pong ball as a single unlabeled marker, providing position input to the planner.

**The paddle/racket is never tracked by the motion capture system.** Each humanoid must infer its own paddle pose through forward kinematics from joint encoders and the tracked `base_link`. This is the fundamental sensing architecture: external perception (ball trajectory) feeds the model-based planner, while internal proprioception (joint states + `base_link`) drives the whole-body controller that positions the paddle. See the companion *HOPE WBC Simulation Training Reference Setup* (Section 2.8) for the complete forward kinematics chain from `base_link` through the 7-DOF arm to the 3D-printed racket mount.

---

## References

- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*.
- HITTER project page: https://humanoid-table-tennis.github.io/
- motion_capture_tracking: https://github.com/IMRCLab/motion_capture_tracking
- 青瞳视觉 (CHINGMU) motion capture: https://www.chingmu.com/ (EN: https://en.chingmu.com/) — VRPN/LiveStream streaming, C/C++/C#/Python/ROS SDKs
- ChingMuVrpnRos (official Chingmu ROS VRPN driver; publishes 6-DOF rigid bodies and 3-DOF markers): https://github.com/ChingMuVisionTech/ChingMuVrpnRos
- vrpn_mocap (ROS 2 VRPN client): https://index.ros.org/p/vrpn_mocap/
- Agibot X1 training code (reference for Agibot kinematic conventions): https://github.com/AgibotTech/agibot_x1_train
- Companion document: *HOPE 7DOF Racket Model-based Planner Reference Setup, v0.1*
- Companion document: *HOPE WBC Simulation Training Reference Setup, v0.5*
- Companion document: *HOPE Hardware Deployment Reference Setup, v0.1*
