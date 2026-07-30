# Motion Capture System Reference Setup for HOPE Ping-Pong Arena

**v0.7** — 2026-07-21

> **Arena reference document.** This records the reference arena configuration
> and the shipped transport paths. The authoritative frame and topic contract is
> [`mocap/README.md`](README.md). The live paths
> shipped in this repo are the vendored OptiTrack/NatNet driver
> (`motion_capture_tracking` + `optitrack_mct_relay`) and the vendored VRPN
> client for Chingmu. Index:
> [`REFERENCE_DOCS.md`](../REFERENCE_DOCS.md).

---

## 1  Compatible Motion Capture Systems

This reference design document creates a reference system compatible with several mainstream motion capture brands — principally **OptiTrack**, **Vicon**, and **青瞳视觉 (CHINGMU)** — and is expected to extend to the other marker-based brands supported by the `motion_capture_tracking` library, including Qualisys, NOKOV, VRPN, FZMotion, and Motion Analysis. These systems differ in their cameras and vendor software — OptiTrack pairs Motive with the NatNet protocol, Vicon uses Vicon Tracker, and Chingmu uses CMTracker/CMAvatar streaming over VRPN, TrackD, DTrack, OpenVR, and its native LiveStream, each shipping C/C++, Python, and ROS SDKs — but this design unifies them under a single ROS 2 REP 103 coordinate frame and `/poses` + `/tf` topic interface. During competition the arena streams the named **6-DOF rigid bodies** `Ball`, `P1`, and `P2`; the planner requires `Ball` at index 0 of `/poses` and the default VRPN bringup aggregates only that ball pose. A fourth asset, `Table`, is defined for arena setup/calibration only and appears only in training-data recordings — it is **not** tracked or reported during competition. The shipped paths are intentionally transport-specific: **OptiTrack/Motive uses NatNet** through `motion_capture_tracking` and `optitrack_mct_relay`, while **Chingmu uses VRPN** through `vrpn_mocap` and `pose_to_posearray`. Both produce the same HOPE planner contract; Section 6 specifies the two paths.

For the HOPE reference design, the minimum specification is:

- At least **8 cameras**, arranged to cover the full table volume plus a 1.5 m margin on each player's side
- Camera frame rate **≥ 300 Hz** (competitive ball tracking at speeds exceeding 5 m/s)
- Sub-millimeter reconstruction accuracy within the tracking volume
- The ball **must** be provided with stable rigid-body modeling and tracking — a vendor-qualified `Ball` rigid-body asset (Section 5) with verified high-speed tracking, occlusion recovery, and ID stability. Single-point / unlabeled-marker ball tracking does not meet this reference design.

---

## 2  Setup of the Environment Markers and Coordinate Frames

To avoid calibration error and potential platform movements, the most straightforward approach is to anchor the motion capture system origin directly on the `Table` rigid body (older notes may call it PPT). However, a common source of confusion is that the default coordinate frame in OptiTrack (Y-up) differs from both ROS 2 (Z-up, REP 103) and Vicon (Z-up). **In this reference design, we adopt the ROS 2 REP 103 convention as the canonical world frame.**

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
2. Under **Advanced Network Options**, change **Up Axis** from "Y Axis" to **"Z Axis"**. This setting is required for the shipped **NatNet** OptiTrack path (Section 6.2), so its output is configured directly for ROS 2 REP 103. Still validate it at surveyed table landmarks before play (Section 6.5): a rig with a reversed, shifted, or otherwise nonconforming source frame needs a full-pose transform upstream of the HOPE relay, never a component-wise pitch/yaw/roll edit.
3. Orient the calibration ground plane so that the calibration square's long edge aligns with the desired X-axis direction (toward P2). This sets the world frame orientation during the calibration wand procedure.

Vicon Tracker defaults to Z-up and generally requires no axis correction. However, verify during ground-plane calibration that the X-axis points along the table length toward P2.

For **青瞳 (Chingmu) CMTracker**, the world frame is fixed by the L-frame / calibration-square placement during the ground-plane calibration step, and the up axis is configurable in the streaming/export settings. Set the up axis to **Z** so that streamed data matches the ROS 2 REP 103 convention, and place the calibration square so its long edge points along the table length toward P2. If a particular CMTracker installation can only stream in a Y-up or otherwise non-REP-103 frame, do **not** attempt to re-calibrate around it — instead apply the fixed axis conversion in the ROS 2 bridge node described in Section 6.4.

### 2.3  Table Rigid Body Definition (`Table`; legacy `PPT` name)

Reflective markers or retroreflective patches (at least 10 mm × 10 mm) are attached to the **outer frame** of the table. Collectively, these markers form one rigid body defined in Motive or CMTracker as the asset **`Table`**. Older arena notes may call this same asset `PPT` (Ping-Pong Table); `Table` is the canonical asset name in setup sessions and training-data recordings. **The `Table` asset is a setup/calibration tool only — it is not streamed or reported during competition.**

Placement requirements:

- Attach **at least 4 markers** in an asymmetric configuration on the table frame's outer edges.
- Place markers where they are visible from the majority of camera positions and will not be occluded by players, the net, or the ball during play.
- **Do not place markers on the playing surface** — they would interfere with ball bounce dynamics and may degrade rigid-body identification.

The `Table` rigid body's pivot point must be set to the **near-side left corner of the table surface** (the origin), with the body's local frame aligned with the world frame axes defined above. After calibration, the `Table` rigid body should report identity pose (position ≈ [0, 0, 0], orientation ≈ [0, 0, 0, 1]) when the table is stationary and properly aligned.

The `Table` rigid body serves two purposes:

1. **Origin anchor** — It defines the world frame origin for all other tracked objects.
2. **Movement verification between sessions** — during setup or verification sessions, a `Table` pose deviating from identity indicates the table was bumped or shifted and the arena needs re-calibration. During competition the table is treated as a static, surveyed world origin: no live `Table` stream exists, so any suspected shift is handled by re-running the verification, not by a runtime topic.

---

## 3  Tracked Object Taxonomy

During competition the motion capture system streams the named rigid bodies **`Ball`, `P1`, and `P2`**. The `Table` asset exists only for setup/calibration and in training-data recordings (Section 2.3). The racket/paddle is explicitly tracked by **nothing**, ever.

### 3.1  Racket Exclusion Policy — Paddle Is NOT Tracked by Motion Capture

**The motion capture system must not track the ping-pong racket (paddle).** No reflective markers or tracking assets should be placed on or attached to the racket. This is a deliberate architectural decision aligned with the HOPE competition design:

**Rationale:**

1. **Forward kinematics inference.** The humanoid must infer its paddle's 6-DOF pose (position and orientation) from its own proprioceptive state — joint encoder readings plus its declared URDF root pose (`pelvis` on Unitree G1; `pelvis_link` on Agibot A3) — using forward kinematics through its arm kinematic chain. Motion capture tracks the P1/P2 marker cluster; a calibrated static transform maps that marker frame to the robot root. This tests the robot's internal body model accuracy, which is a core competency for any real-world manipulation task.

2. **No external sensing of end-effector.** In this architecture, the whole-body controller (WBC) receives a desired racket state `(p_intercept, v_racket, n_racket, t_strike)` from the planner and uses its RL policy to drive the 7-DOF arm to achieve that state. The controller never receives measured racket pose from the motion capture system. The racket's actual position is an emergent property of the robot's joint configuration, not an externally measured quantity.

3. **Competition fairness.** Tracking the racket externally would provide closed-loop feedback that bypasses the robot's control challenge. The HOPE competition requires each team's humanoid to demonstrate autonomous paddle control through its own kinematic model.

4. **Practical reliability.** Markers on a rapidly swinging paddle (arm speeds exceeding 3 m/s) suffer from severe occlusion, motion blur, and centripetal marker detachment. Excluding the paddle from tracking eliminates a fragile sensing link.

**Enforcement:** During competition setup, referees verify that no retroreflective material is present on the racket, the robot's hand, or the wrist link beyond the last tracked rigid-body marker on the robot's torso/pelvis.

**Cross-references:** The companion *HOPE 7DOF Racket Model-based Planner Reference Setup* (Section 0.1) documents that the planner outputs a desired racket state without any racket pose feedback. The companion *HOPE WBC Simulation Training Reference Setup* (Section 2.8 — Racket Mount Kinematics) documents the complete FK chain from the declared robot root frame through the 7-DOF arm to the 3D-printed fixed racket mount, including the `T_mount` calibration procedure that ensures the simulation model matches the physical bracket.

### 3.2  Tracked Objects Summary

| Object ID | Asset type | What is tracked | Markers | Tracking mode |
|-----------|-----------|-----------------|---------|---------------|
| **Table** | Rigid body (setup/calibration only — **not streamed in competition**; poses appear only in training data) | Ping-pong table frame and world origin | ≥ 4 asymmetric on table outer frame | Vendor 6-DOF |
| **P1** | Rigid body (vendor-tracked) | Player 1 marker-cluster frame; statically calibrated to its declared robot root | ≥ 4 asymmetric on torso/pelvis plate | Vendor 6-DOF |
| **P2** | Rigid body (vendor-tracked) | Player 2 marker-cluster frame; statically calibrated to its declared robot root | ≥ 4 asymmetric on torso/pelvis plate | Vendor 6-DOF |
| **Ball** | Rigid body (vendor-tracked) | Ping-pong ball center pose | Vendor-qualified rigid-body pattern/constellation | Vendor 6-DOF |

No other objects should carry unregistered retroreflective patterns within the tracking volume during play. Give every rigid body a unique asymmetric signature and stable asset name so the vendor solver cannot swap asset identities.

---

## 4  Setup of the Humanoid Root-Frame Markers

In this reference design, the humanoid infers its paddle's 6-DOF pose using **forward kinematics from its declared URDF root frame** through the arm's kinematic chain. Motion capture supplies the P1/P2 marker-cluster pose, and a calibrated static transform maps it to that root (`pelvis` on Unitree G1; `pelvis_link` on Agibot A3).

### 4.1  Robot Root-Frame Convention — General Principles

There is no universal name or location for a humanoid robot's URDF root frame. A platform may call it `base_link`, `pelvis`, `pelvis_link`, or something else. The convention varies by manufacturer, URDF authoring choices, and the robot's intended control architecture. However, three common patterns have emerged across the industry:

**Pattern A — Pelvis root (most common for bipedal locomotion).** The declared root is the pelvis link, located at the center of the hip plate where the leg kinematic chains branch downward and the torso chain branches upward. This is a common choice for RL-trained locomotion controllers because the pelvis is the most stable reference during walking — it is the floating-base frame in whole-body dynamics. Unitree G1 uses the exact URDF link name `pelvis`; Agibot A3 uses `pelvis_link`.

**Pattern B — Torso/chest root.** Some platforms place their declared root at the upper torso or chest, above the waist joint(s). This is less common for bipedal locomotion (the pelvis is more dynamically stable) but can appear in manipulation-focused configurations where the arms are the primary concern and the legs are treated as a mobile base subsystem.

**Pattern C — Waist joint root.** A compromise where the declared root sits at the waist joint itself — the interface between legs and torso. In many simple designs this is co-located with the pelvis origin (Pattern A). In robots with multi-DOF waist articulation, the waist joint is above the pelvis, and choosing it as the root places it between the two subsystems.

**For the HOPE competition, the critical requirement is:**

> The declared robot root frame must anchor the forward kinematics chain that reaches the paddle-holding hand. The planner outputs a desired racket state in the world frame; the robot's WBC must compute the arm joint trajectory from that root to the paddle.

The complete spatial chain is: `world → P1/P2 (live mocap) → declared robot root (calibrated static TF) → waist joints → shoulder → elbow → wrist → paddle tip (joint encoders)`. Every joint between the declared root and the paddle must be instrumented with encoders whose readings are available to the robot's control software.

### 4.2  Unitree G1

The Unitree G1 is one robot-specific integration example; the same registration and frame requirements apply to every participating humanoid.

| Property | Value |
|----------|-------|
| Declared URDF root link | **`pelvis`** |
| Declared URDF root location | **Pelvis** — center of lower torso at the waist, approximately at the intersection of the two hip yaw joint axes |
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
pelvis (declared URDF root)
├── left_hip_yaw_joint  → left leg (6 DOF)
├── right_hip_yaw_joint → right leg (6 DOF)
└── waist_yaw_joint     → torso → shoulder → elbow → wrist (7 DOF per arm)
```

**Marker placement:** Attach a 4-marker asymmetric cluster on a rigid plate secured to the pelvis shell. Set the rigid body pivot point in Motive to the pelvis origin (center of the hip plate). If markers are on the outer shell surface, calibrate a static TF offset of a few centimeters in Z.

### 4.3  Agibot Expedition A3

The Expedition A3 is an athletic humanoid by Agibot (Zhiyuan Robotics).

| Property | Value |
|----------|-------|
| Declared URDF root link | **`pelvis_link`** — confirmed from the A3 URDF |
| Standing height | Full-size (~1.75 m, estimated from video) |
| Weight | Not publicly disclosed |
| Total DOF | Not publicly disclosed; described as "highly anthropomorphic full-body degrees of freedom" |
| Arm DOF | Not publicly disclosed (7 DOF per arm expected, based on Agibot platform lineage) |
| Waist DOF | **Multi-DOF flexible waist** — a key distinguishing feature engineered to mirror the human range of motion, enabling rotation and swaying for complex whole-body movements |
| URDF source | Repository paths `agibot/URDF/a3_t2d5/urdf/model.urdf` and `agibot/URDF/A3T2.5-URDF-std-pingpang/urdf/URDF-JOINT-LINK.urdf` |
| Middleware | **AimRT** (Agibot's native C++20 runtime); supports ROS 2 protocol bridging |

**Key considerations:**

1. **Flexible waist implications.** A3's root `pelvis_link` sits below the articulated waist, so the waist joints remain part of the forward-kinematics chain to the paddle. This is important for table-tennis torso rotation, weight transfer, and reach.

2. **Kinematic-chain validation.** Teams using A3 should validate the supplied URDF revision, standing `pelvis_link` height, and complete joint chain from `pelvis_link` to the paddle-holding hand against the deployed robot.

3. **Middleware bridging.** The A3 runs on AimRT natively, not ROS 2. AimRT supports ROS 2 as one of several communication protocols (alongside HTTP, gRPC, MQTT, and Zenoh). For the HOPE architecture, two integration approaches are available:
   - **Approach 1 (recommended):** Run the HOPE planner as a ROS 2 node; bridge the `RacketCommand` topic into AimRT where the A3's native WBC consumes it. The `pelvis_link` pose, obtained from `world → P1 → pelvis_link`, flows through ROS 2 → AimRT.
   - **Approach 2:** Run the planner within AimRT directly, subscribing to the motion capture data via AimRT's ROS 2 protocol support.

4. **A3 P1 registration and current marker status.** The v2 CAD table and the physical `a3_hip_marker_shell_p1_mocap_balls_0702.x_t` shell define all ten marker centres `f1`–`f5`, `b1`–`b5`; a physical mocap experiment confirmed that all ten points are visible. The complete set has a nominal centroid of `[-0.0024, 0, -0.1490] m` in `pelvis_link`. Create the P1 Motive asset from all ten markers. After aligning P1 axes to `pelvis_link` (`+X` forward, `+Y` left, `+Z` up), the CAD cross-check for its centroid-to-pelvis pivot translation is `[+0.0024, 0, +0.1490] m` (`[+2.4, 0, +149.0] mm`). Centroid data alone does **not** establish orientation. During one-time bringup, use `p1_pelvis_calibrator` to compare mocap `/P1/pose` (`world → P1`) with an independently produced, acquisition-time-synchronized, full-6DOF `world → pelvis_link` `PoseStamped`; the tool does not use `Table`, because a common Table transform cancels exactly. The current A3 hardware bridge provides pelvis IMU data but no absolute pelvis translation, so a real-hardware integration must supply the independent `/a3/calibration/pelvis_pose` source from external metrology or a state estimator before this procedure can run. Collect while translating and rotating the pelvis: the tool separately gates motion excitation and timing coverage because a low residual RMS measures consistency, not observability. Save the installed plate's full 6-DOF `P1 → pelvis_link` correction. At normal bringup, `p1_pelvis_tf_publisher` loads this JSON as a static TF. Its live result supersedes the CAD nominal value. Absorbing the correction into Motive's P1 pivot is an optional alternative; never use it together with the ROS 2 static TF. The tool and validation procedure are documented in [`docs/OPTITRACK.md`](../docs/OPTITRACK.md#calibrating-p1-to-an-a3-pelvis_link).

### 4.4  Competition Registration Requirements

Each team must declare the following during HOPE competition registration. This information is needed to verify that the motion capture system, planner, and WBC are correctly integrated for their specific humanoid platform.

| Item | Description | Example (Agibot A3) |
|------|-------------|---------------------|
| **Robot model** | Manufacturer and model designation | Agibot Expedition A3 |
| **Declared URDF root link** | Exact root link name used by the controller | `pelvis_link` |
| **Root-frame physical location** | Description of where the root origin sits on the physical robot | Center of hip plate, at intersection of hip yaw axes |
| **Root-frame pattern** | Which convention (A/B/C from Section 4.1) | Pattern A (pelvis root) |
| **Standing root height** | Height of the root origin above the floor in nominal stance | Team-measured value |
| **Mocap-to-root static transform** | Full 6-DOF transform from the P1/P2 marker-cluster frame to the declared URDF root | Calibrated value; `P1 → pelvis_link` on A3 |
| **Arm DOF count** | Number of actuated joints from the declared root to paddle grip, including waist | Platform-specific |
| **Middleware** | ROS 2 native, AimRT with ROS 2 bridge, or other | AimRT with ROS 2 bridge |
| **URDF availability** | Public URL or "provided to organizers under NDA" | Repository path under `agibot/URDF/` |

The calibrated mocap-to-URDF offset is saved in JSON and published at bringup.
For A3 the resulting chain is `world → P1 → pelvis_link`:

```bash
ros2 run hope_bringup p1_pelvis_tf_publisher \
  --calibration-file calibration/p1_to_pelvis.json
```

### 4.5  What the Robot Knows vs. What Motion Capture Provides

| Information | Source | Used by |
|-------------|--------|---------|
| Ball 6-DOF pose: position `[x, y, z]` + quaternion `[qx, qy, qz, qw]` at the capture rate | Motion capture → ROS 2 topic | Planner uses position (Stages 1–3); orientation is preserved for validation and future spin-aware estimation |
| Humanoid root-frame 6-DOF pose (`pelvis_link` on A3) | Live P1/P2 pose composed with the calibrated static transform | WBC (Stage 4) for root position commands |
| `Table` rigid-body pose | Setup/calibration sessions and training-data recordings only — **no competition stream** | Arena calibration (world-origin verification) |
| Paddle 6-DOF pose | **Forward kinematics** from joint encoders + declared robot root | WBC internal state; **not** from motion capture |
| Paddle desired state | Planner output (Stage 3) | WBC (Stage 4) as tracking target |

---

## 5  Ball Rigid-Body Tracking Configuration

Both OptiTrack Motive and Chingmu CMTracker now solve the ping-pong ball as a named **rigid-body asset**. The measured state is a full pose: translation `(x, y, z)` and orientation. The asset pivot must coincide with the ball's geometric center; if that is not possible in the vendor tool, record and apply a fixed asset-to-center transform.

### 5.1  Ball Preparation and Asset Definition

- Use the vendor-qualified ball preparation and marker pattern/constellation for rigid-body tracking. **Verified preparation: retroreflective marker dots added to a standard ping-pong ball achieve stable 6-DOF rigid-body tracking on both OptiTrack and Chingmu.** Do not infer a working marker count or layout from the retired single-point setup.
- Minimize changes to the ball's mass, center of mass, diameter, surface friction, and aerodynamics, and validate the prepared ball against competition rules.
- Make the pattern asymmetric and distinguishable from `Table` and robot patterns throughout the camera volume.
- Define a stable rigid-body asset name and ID (recommended logical name: `Ball`) in Motive or CMTracker. Topic and sender names are case-sensitive.
- Set the rigid-body pivot to the geometric center and document its local axes. Validate high-speed tracking, occlusion recovery, and ID stability before recording data.

> **Legacy ball preparations are incompatible.** Earlier revisions of this document (≤ v0.4,
> single-marker tracking) recommended a *fully coated* retroreflective ball. That
> recommendation is now **inverted**: a uniformly coated sphere presents no distinguishable
> marker constellation and cannot be identified as a rigid body. Rigid-body tracking requires
> a patterned preparation. Do not reuse balls prepared for the retired single-point setup.

### 5.2  Pose Representation

Operators may inspect the state as `(x, y, z, pitch, yaw, roll)`, but ROS 2 should never carry Euler angles as the canonical orientation representation. VRPN tracker reports carry a quaternion; ROS 2 stores it in `geometry_msgs/Pose.orientation` as `(x, y, z, w)`:

```text
geometry_msgs/Pose
  position:    x, y, z
  orientation: x=qx, y=qy, z=qz, w=qw
```

Normalize each quaternion and reject NaN, zero-norm, or stale poses. If Euler angles are needed for display or analysis, state the frame, handedness, and rotation order. Motive displays X as Pitch, Y as Yaw, and Z as Roll in its documented right-handed local-axis convention; do not assume that another vendor's Euler display uses the same convention.

### 5.3  Orientation, Angular Velocity, and Spin

The current HOPE planner remains a **no-spin** planner: it consumes `(x, y, z)` and models translational drag, but it does not use ball orientation or Magnus force. The ROS 2 bridges nevertheless preserve the measured quaternion so recordings remain 6-DOF and future estimators can use it.

A rigid-body quaternion is not itself angular velocity. A spin-aware extension must unwrap quaternion sign, differentiate rotations using source timestamps, filter noise, and verify that the tracked rigid-body pattern is mechanically locked to the ball. If the marker carrier slips relative to the shell, the reported attitude is not physical ball spin.

### 5.4  Acceptance Checks

Before an arena session:

1. Place the Ball asset at surveyed table landmarks and confirm the reported pivot is the ball center.
2. Rotate the prepared ball through known attitudes and confirm the quaternion is normalized and the displayed pitch/yaw/roll change on the intended axes.
3. Launch, bounce, and strike the ball across the full volume; measure dropouts, latency, reacquisition behavior, and any asset-ID swaps.
4. Confirm the ROS 2 `frame_id`, source timestamps, units, and complete position-plus-orientation fields with `ros2 topic echo`.
5. Record the vendor asset name/ID, marker preparation, pivot transform, local-axis definition, and software versions with the session metadata.

---

## 6  Streaming Rigid-Body Poses to ROS 2

The vendor application performs camera reconstruction and rigid-body solving. A ROS 2 bridge receives the vendor's native stream and maps every solved pose to standard ROS messages. Position is in metres; orientation remains a quaternion from the vendor stream through ROS 2.

### 6.1  Confirmed Transport Paths

The planner contract is uniform, but the wire protocol is deliberately not. **OptiTrack uses
NatNet; Chingmu uses VRPN.** Both paths preserve the solved position and quaternion, then
publish `geometry_msgs/PoseArray` on `/poses` with `Ball` at index 0.

```text
OptiTrack cameras → Motive → NatNet UDP → motion_capture_tracking_node
                                        → /optitrack/poses (NamedPoseArray)
                                        → optitrack_mct_relay → /poses

Chingmu cameras → CMTracker/MCServer → VRPN → vrpn_mocap
                                             → /vrpn_mocap/<sender>/pose_id_<N> (PoseStamped)
                                             → pose_to_posearray → /poses
```

| System | Vendor payload | ROS 2 bridge | ROS 2 result |
|--------|----------------|--------------|--------------|
| **OptiTrack / Motive** | NatNet rigid-body frames for `Ball`, `P1`, and `P2`: asset name, position vector, and quaternion | vendored `motion_capture_tracking` in namespace `/optitrack`, then `optitrack_mct_relay` | `/optitrack/poses` is `NamedPoseArray` (`header` plus `{string name, geometry_msgs/Pose pose}` entries); relay produces HOPE `/poses`, `/ball/point`, optional `/P1/pose` and `/P2/pose`, and TF |
| **Chingmu** | VRPN tracker report for a named CMTracker rigid body: sender name, sensor index, position vector, and quaternion | vendored `vrpn_mocap`, then `pose_to_posearray` | `/vrpn_mocap/<sender>/pose_id_<sensor_id>` as `geometry_msgs/PoseStamped`; adapter copies the complete pose to `/poses` |

`(pitch, yaw, roll)` is an operator-facing representation. Both paths carry orientation as a
quaternion and preserve it end-to-end. `PoseArray` has no per-pose name field, so `Ball` must
remain first and names are retained in the OptiTrack raw stream or session metadata. The raw
OptiTrack topic is intentionally namespaced: publishing `NamedPoseArray` on the bare `/poses`
name would collide with the planner's `geometry_msgs/PoseArray` DDS type.

### 6.2  OptiTrack / NatNet Path

In Motive, define the competition assets `Ball`, `P1`, and `P2` as rigid bodies, with the
`Ball` pivot at the ball center. Keep `P1`/`P2` as marker-cluster rigid-body frames and
calibrate a static transform from each one to its robot's declared URDF root. The
`Table` asset is used only in a separate setup/calibration or training-recording session
(Section 2.3); disable or omit it before competition streaming.

The expected Motive settings are:

| Setting | Required value | Notes |
|---------|----------------|-------|
| NatNet | ✅ Enabled | Required by the shipped OptiTrack backend; Motive's VRPN Streaming Engine is not used by this path |
| Up Axis | **Z** | Matches the HOPE ROS 2 REP 103 world frame; validate at landmarks before play |
| Delivery | Unicast preferred | The client connects to the Motive PC; NatNet negotiates stream details with the server |
| Command port | UDP 1510 (normally) | The vendored driver uses NatNet's command channel and obtains data-port details from Motive; permit the negotiated UDP data traffic through the firewall |
| Rigid Bodies | `Ball`, `P1`, `P2` in competition; `Table` only during calibration | Names are case-sensitive and are passed through verbatim to the relay |
| Ball | 6-DOF rigid-body asset named `Ball` | Set its pivot to the geometric ball center; a tracking loss removes the `Ball` entry and pauses `/poses`, rather than republishing a stale ball |

Start the complete planner path with the Motive PC address:

```bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=optitrack \
  mocap_server:=MOTIVE_PC_IP \
  mocap_network_latency_ms:=MEASURED_ONE_WAY_MS
```

The chain is `Motive → motion_capture_tracking_node → /optitrack/poses →
optitrack_mct_relay → /poses`. The driver publishes one `NamedPoseArray` per camera frame;
the relay maps names according to `config/optitrack_relay.yaml`, scales positions only if
configured otherwise, and preserves the quaternion. The default driver configuration uses
`topics.header_time: ros_latency_compensated`: it maps exposure time into the local ROS epoch
by subtracting NatNet Camera/Motive latencies and the measured one-way network/host latency
from receipt time. Bare `ros` is arrival time; bare `camera` is the Motive host's unrelated
clock epoch. The legacy Motive VRPN port 3883 is not part of this connection. See
[`docs/OPTITRACK.md`](../docs/OPTITRACK.md) for build, launch, and diagnostic details.

### 6.3  Chingmu / VRPN Path

In CMTracker/MCServer, define the ball as a rigid body and assign a stable VRPN sender name such as `Ball`. The Ball is no longer handled as an unlabeled marker under a shared sender. Set the streaming up axis to **Z** (Section 2.2) so no software frame conversion is needed. Run the vendored native ROS 2 VRPN client against the Chingmu server:

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=CHINGMU_SERVER_IP port:=3883
```

```yaml
/vrpn_mocap_client:
  ros__parameters:
    server: "CHINGMU_SERVER_IP"
    port: 3883
    frame_id: "world"
    multi_sensor: true
    use_vrpn_timestamps: false  # set true only when server and ROS clocks are synchronized
    update_freq: 100.0
    refresh_freq: 1.0
```

The client auto-discovers VRPN tracker senders. Its pose callback maps `vrpn_TRACKERCB.pos[0:3]` directly to `PoseStamped.pose.position` and `quat[0:4]` directly to `PoseStamped.pose.orientation.{x,y,z,w}`. With `multi_sensor: true`, typical single-sensor rigid bodies appear as:

```text
/vrpn_mocap/P1/pose_id_0     geometry_msgs/PoseStamped
/vrpn_mocap/P2/pose_id_0     geometry_msgs/PoseStamped
/vrpn_mocap/Ball/pose_id_0   geometry_msgs/PoseStamped
```

Actual names and capitalization come from CMTracker and are case-sensitive. If CMTracker assigns a sensor index other than zero, use that published index rather than rewriting it. `multi_sensor: true` is a safe default and prevents collisions if a sender exposes more than one sensor.

Configure `hope_bringup/pose_to_posearray` with the Ball topic first to preserve the planner's default `ball_pose_index: 0`. The adapter publishes `/poses` but does not create `/tf`; add a `tf2_ros` broadcaster if the deployment also requires named transforms.

### 6.4  Coordinate and Orientation Conversion

Both vendor outputs must arrive in the canonical REP 103 Z-up frame of Section 2.1. Configure
Motive's NatNet stream and CMTracker's VRPN stream as Z-up (Section 2.2) and validate them at
landmarks before play. The shipped adapters do not apply a vendor-specific axis rotation. If a
venue cannot produce the canonical frame, add one explicit upstream transform and apply it to
the **entire pose**, not just its three position values:

```text
p_HOPE = R_HOPE_FROM_MOCAP · p_mocap + t_HOPE_FROM_MOCAP
R_HOPE_BODY = R_HOPE_FROM_MOCAP · R_mocap_body
```

For a nonconforming right-handed Y-up source, translation alone maps as `x_HOPE=x_mocap`, `y_HOPE=-z_mocap`, `z_HOPE=y_mocap`. Apply the same fixed rotation to the orientation using `tf2` or quaternion/matrix composition. Component-wise edits to pitch/yaw/roll are not a valid general pose transform.

Verify the source handedness and axis directions at surveyed table landmarks before trusting any conversion. A mirrored source frame needs an installation-specific correction; do not guess it from the vendor name.

### 6.5  ROS 2 Validation Checklist

```bash
# OptiTrack / NatNet
ros2 topic echo --once /optitrack/poses

# Chingmu / VRPN
ros2 topic echo --once /vrpn_mocap/Ball/pose_id_0

# Either backend: the planner input
ros2 topic echo --once /poses
```

Confirm all of the following:

- `Ball` (and `P1`/`P2` where streamed) are distinct, stable rigid bodies; no asset ID swaps occur after occlusion. Confirm **no `Table` topic is being streamed** during competition.
- `position` is in metres, `orientation` is finite and unit length, and the Ball pivot is at its geometric center.
- The message `frame_id` and axes match the HOPE world frame. **Landmark validation is mandatory before play** for every vendor and installation: place the `Ball` asset at surveyed landmarks (e.g. the net-center line `x = 1.37, y = −0.7625, z = 0.02`) and confirm the streamed coordinates match Section 2.1. This catches an incorrect Up Axis, shifted origin, mirrored frame, or accidental double transform.
- Occlusion produces a **dropout**, not a frozen or all-zero pose. On the NatNet path, a missing `Ball` entry causes the relay to pause `/poses`; consumers must not substitute a stale pose.
- The shipped NatNet config uses `ros_latency_compensated` plus measured `mocap_network_latency_ms`; do not use bare receipt-time `ros` or mix the Motive `camera` epoch directly with ROS. A VRPN vendor timestamp requires a verified NTP/PTP or equivalent clock mapping.
- `/poses` index order matches the planner configuration. The current no-spin planner reads the Ball position while the full quaternion remains in the message and bag recording.

---

## 7  Integration with the HOPE Planner

The companion planner document (*HOPE 7DOF Racket Model-based Planner Reference Setup*) consumes ball position data from the `/poses` stream described in Section 6 and produces racket target commands. The data flow through the complete system is:

```
Motion Capture System (360 Hz)                         Humanoid (proprioceptive)
  │                                                      │
  ├── Ball 6-DOF rigid-body pose ──▶ HOPE Planner      │
  │      (planner currently uses xyz)  Stages 1–3       │
  │                                        ▼              │
  └── P1 marker 6-DOF ──▶ robot root TF ──▶ WBC (Stage 4) ◀── RacketCommand
                                           │              (p_intercept,
                                           │               v_racket,
                                           ▼               n_racket,
                                     Joint commands        t_strike)
                                     (varies by platform)
                                           │
                                           ▼
                                     Paddle pose
                                     (inferred via FK from
                                      robot root + joint encoders,
                                      NOT measured by mocap)
```

The planner operates entirely in the HOPE canonical world frame defined in Section 2.1. The OptiTrack/NatNet and Chingmu/VRPN paths deliver complete rigid-body poses in this frame after the configuration or transform in Section 6.4. The current planner uses the Ball translation; the orientation quaternion remains available to other consumers and recordings.

---

## 8  Summary

The HOPE motion capture reference system defines four named rigid-body assets; only `Ball`, `P1`, and `P2` are streamed during competition:

1. **`Ball`** — the ping-pong ball as a vendor-tracked 6-DOF rigid body. ROS 2 receives position plus quaternion orientation; the current no-spin planner uses position only.
2. **`Table`** — a setup/calibration-only asset anchoring the world frame origin (legacy arena notes may call this `PPT`); it appears in training-data recordings but is **not** streamed during competition.
3. **`P1`** — the Player 1 humanoid marker-cluster rigid body.
4. **`P2`** — the Player 2 humanoid marker-cluster rigid body.

Each team declares its robot-specific URDF root frame and provides the calibrated static transform from P1/P2 to that root (Section 4). For A3 this mapping is `P1 → pelvis_link`.

**The paddle/racket is never tracked by the motion capture system.** Each humanoid must infer its own paddle pose through forward kinematics from joint encoders and its declared robot root pose. This is the fundamental sensing architecture: external perception (ball trajectory) feeds the model-based planner, while internal proprioception (joint states + robot root pose) drives the whole-body controller that positions the paddle. See the companion *HOPE WBC Simulation Training Reference Setup* (Section 2.8) for the complete forward kinematics chain from the robot root through the 7-DOF arm to the 3D-printed racket mount.

---

## References

- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*.
- HITTER project page: https://humanoid-table-tennis.github.io/
- motion_capture_tracking (vendored in-tree as the supported OptiTrack/NatNet backend — `hope_ws/src/motion_capture_tracking/`, exact pins and local patches in its PIN.md; publishes `NamedPoseArray`, bridged to the `/poses` contract by `optitrack_mct_relay`; upstream: https://github.com/IMRCLab/motion_capture_tracking)
- OptiTrack Motive VRPN Streaming Engine (rigid bodies only; default port 3883): https://docs.optitrack.com/motive-ui-panes/settings/settings-streaming
- VRPN protocol: https://github.com/vrpn/vrpn
- 青瞳视觉 (CHINGMU) motion capture: https://www.chingmu.com/ (EN: https://en.chingmu.com/) — VRPN/LiveStream streaming, C/C++/C#/Python/ROS SDKs
- ChingMuVrpnRos (official Chingmu ROS/VRPN reference): https://github.com/ChingMuVisionTech/ChingMuVrpnRos
- vrpn_mocap (ROS 2 VRPN client): https://index.ros.org/p/vrpn_mocap/
- Agibot X1 training code (reference for Agibot kinematic conventions): https://github.com/AgibotTech/agibot_x1_train
- Companion document: *HOPE 7DOF Racket Model-based Planner Reference Setup, v0.1*
- Companion document: *HOPE WBC Simulation Training Reference Setup, v0.5*
- Companion document: *HOPE Hardware Deployment Reference Setup, v0.1*
