# HOPE Hardware Deployment Reference Setup

v0.2 — 2026-04-02

## Overview

This document describes how to deploy trained HOPE ping-pong WBC policies onto physical humanoid hardware — the Unitree G1 or Agibot Expedition A3. It is the final step in the HOPE pipeline, consuming the ONNX policy exported from simulation training (see companion *HOPE WBC Simulation Training Reference Setup*, Section 5) and connecting it to the live motion capture system (see companion *HOPE Motion Capture System Reference Setup*) and the real-time planner (see companion *HOPE 7DOF Racket Model-based Planner Reference Setup*).

The deployment architecture is a ROS 2 Jazzy graph running on the robot's onboard computer or a tethered workstation. The policy runs ONNX CPU inference at 50 Hz, reads joint encoder feedback and `base_link` pose from the motion capture system, receives racket target commands from the planner, and outputs 29-DOF (G1) or equivalent-DOF (A3) joint position commands that are converted to torques by a PD controller on the robot's actuator bus.

> **HOPE Racket Tracking Prohibition.** As documented in all companion references, the motion capture system tracks only the table origin (PPT), humanoid base_links (P1/P2), and the ball. The racket is NEVER tracked. The deployed policy infers its paddle state via forward kinematics from `base_link` + joint encoders through the fixed wrist mount (see WBC Training doc Section 2.8).

---

## 0  Prologue — Differences from the HITTER Paper

| # | Aspect | HITTER paper | This HOPE reference setup | Rationale |
|---|--------|-------------|---------------------------|-----------|
| 1 | **Inference framework** | Not named; paper states "built upon" BeyondMimic and "deployed zero-shot" | BeyondMimic `motion_tracking_controller` (C++ ONNX, `legged_control2`) for G1; AimRT native ONNX or ROS 2 bridged for A3 | HITTER does not name the deployment software. We infer `motion_tracking_controller` from shared authorship (Qiayuan Liao) and BeyondMimic attribution. |
| 2 | **Target platforms** | Unitree G1 only | G1 (via `legged_control2` + `unitree_bringup`) and Agibot A3 (via AimRT with ROS 2 protocol bridge) | HOPE multi-platform support. |
| 3 | **Safety system** | Not described | E-stop via Unitree joystick (G1) or AimRT safety node (A3); mandatory pre-flight checklist documented | Physical safety is critical for fast-moving arm swings near humans. |
| 4 | **PD gain tuning** | "Joint PD gains are set heuristically following [BeyondMimic]"; "deployed zero-shot" (no real-world RL training) | Same heuristic gains from BeyondMimic, embedded in ONNX metadata; sim-to-real gain adjustment procedure documented for cases where hardware response differs | "Zero-shot" means no additional RL training on real hardware; it does not preclude hardware-level gain tuning, which HITTER does not discuss. |
| 5 | **Network architecture** | Not described | Full ROS 2 node graph with topic names, QoS profiles, and latency budget (HOPE analysis) | HITTER omits ROS 2 details. Latency numbers are HOPE's engineering analysis, not HITTER's published data. |
| 6 | **Forehand/backhand switching** | "We trigger a forehand strike when the ball's y position is negative and a backhand strike when positive"; single sentence, no deployment mechanism | Two-ONNX-model runtime switching documented; controller must load both forehand and backhand ONNX sessions and select per ball Y-position | BeyondMimic embeds one reference motion per ONNX file. HITTER does not describe how the runtime switch between two models is implemented. |

---

## 1  System Architecture

### 1.1  ROS 2 Node Graph

The deployment runs as a ROS 2 Jazzy graph. All nodes run on a single machine (the robot's onboard computer or a tethered laptop connected via Ethernet) except for the OptiTrack Motive server (Windows PC).

```
┌──────────────────────────────────────────────────────────────────────────┐
│  Windows PC (OptiTrack Motive)                                           │
│  ┌─────────────────────────┐                                             │
│  │  Motive 3.x             │                                             │
│  │  NatNet server (unicast) │─── Ethernet (1 Gbps) ──┐                   │
│  └─────────────────────────┘                          │                   │
└───────────────────────────────────────────────────────┼──────────────────┘
                                                        │
┌───────────────────────────────────────────────────────┼──────────────────┐
│  Linux PC (Ubuntu 24.04, ROS 2 Jazzy)                 │                   │
│                                                        ▼                  │
│  ┌─────────────────────────────────────────────────────────────┐         │
│  │  motion_capture_tracking node                                │         │
│  │  Publishes:                                                  │         │
│  │    /ball/point          (PointStamped, 360 Hz)               │         │
│  │    /P1/pose             (PoseStamped, 360 Hz)                │         │
│  │    /P2/pose             (PoseStamped, 360 Hz)                │         │
│  │    /table/pose          (PoseStamped, 10 Hz)                 │         │
│  └──────────┬──────────────────────────┬───────────────────────┘         │
│             │                          │                                  │
│             ▼                          ▼                                  │
│  ┌──────────────────────┐   ┌──────────────────────────────┐             │
│  │  HOPE Planner Node   │   │  WBC Controller Node         │             │
│  │  (Python, 50 Hz)     │   │  (C++, 50 Hz)                │             │
│  │                      │   │                               │             │
│  │  Subscribes:         │   │  Subscribes:                  │             │
│  │   /ball/point        │   │   /racket/command             │             │
│  │   /P1/pose           │   │   /P1/pose (base_link)       │             │
│  │                      │   │   Joint encoder feedback      │             │
│  │  Publishes:          │   │                               │             │
│  │   /racket/command    │──▶│  ONNX inference (50 Hz)       │             │
│  │   (RacketCommand)    │   │  → 29-DOF joint pos targets   │             │
│  └──────────────────────┘   │                               │             │
│                             │  Publishes:                   │             │
│                             │   Joint position commands     │             │
│                             │   → PD controller → torques   │             │
│                             └───────────┬───────────────────┘             │
│                                         │                                 │
│                                         │ Ethernet (192.168.123.x)        │
│                                         ▼                                 │
│                             ┌───────────────────────────┐                 │
│                             │  Unitree G1 / Agibot A3   │                 │
│                             │  Actuator bus (CAN/EtherCAT)│                │
│                             └───────────────────────────┘                 │
└───────────────────────────────────────────────────────────────────────────┘
```

### 1.2  Latency Budget

The total perception-to-actuation latency must be under 20 ms for competitive play. The HITTER paper does not publish end-to-end latency numbers but states the system achieves "sub-second reaction times" (measured from the opponent's hit to the robot's return, which includes ball flight time). The latency budget below is HOPE's engineering analysis of the pipeline components:

| Stage | Budget | Actual (typical) |
|-------|--------|------------------|
| OptiTrack capture + NatNet | 2.8 ms | ~2.8 ms at 360 Hz |
| `motion_capture_tracking` ROS 2 node | < 1 ms | ~0.5 ms |
| Planner (ball estimation + trajectory prediction + racket target) | < 5 ms | ~2–4 ms (Python, single ball) |
| WBC ONNX inference | < 5 ms | ~1–2 ms (CPU, MLP [512,256,128]) |
| Actuator bus command transmission | < 2 ms | ~1 ms (Unitree DDS / AimRT EtherCAT) |
| PD controller + motor response | < 5 ms | ~3–5 ms |
| **Total** | **< 20 ms** | **~10–15 ms** |

---

## 2  Path A — Unitree G1 Deployment

The G1 deployment uses the BeyondMimic `motion_tracking_controller` package, which provides a production-ready C++ ONNX inference node built on the `legged_control2` framework. The HITTER paper does not explicitly name its deployment framework, but states it was "built upon" BeyondMimic, and Qiayuan Liao (the `legged_control2` and `motion_tracking_controller` author) is a co-author on the HITTER paper. The `motion_tracking_controller` repository is the official BeyondMimic deployment code and is the canonical path for deploying BeyondMimic-trained policies on Unitree hardware.

### 2.1  Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 24.04 LTS (Noble) |
| ROS 2 | Jazzy |
| `legged_control2` | Latest (Jazzy apt packages) |
| `unitree_bringup` | Latest (from source) |
| `motion_tracking_controller` | Latest (from source) |
| Trained ONNX policy | From WBC training (Section 5 of training doc) |
| Unitree G1 hardware | With 3D-printed racket mount on right wrist |
| Ethernet cable | Connecting PC to G1 (static IP 192.168.123.11) |
| Unitree RC joystick | For E-stop and mode switching |

### 2.2  Installation

```bash
# 1. Install ROS 2 Jazzy
# Follow: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# 2. Install legged_control2 (Debian + source)
# Follow: https://qiayuanl.github.io/legged_control2_doc/installation.html

# Add Unitree-specific apt source
echo "deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/ ./" \
    | sudo tee /etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list
echo "yaml https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" \
    | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_unitree_buildfarm.list
sudo apt-get update

# Install Unitree packages
sudo apt-get install ros-jazzy-unitree-description
sudo apt-get install ros-jazzy-unitree-systems

# 3. Create workspace and clone repos
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/qiayuanl/unitree_bringup.git
git clone https://github.com/HybridRobotics/motion_tracking_controller.git

# 4. Install ROS dependencies
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    --packages-up-to unitree_bringup
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    --packages-up-to motion_tracking_controller
source install/setup.bash
```

### 2.3  Sim-to-Sim Verification (MuJoCo)

Before deploying to hardware, verify the ONNX policy in a MuJoCo simulation:

```bash
# Load from WandB
ros2 launch motion_tracking_controller mujoco.launch.py \
    wandb_path:=your-org/hope_wbc/run_id

# OR load from local ONNX file (absolute path)
ros2 launch motion_tracking_controller mujoco.launch.py \
    policy_path:=/home/user/hope_forehand_policy.onnx
```

The MuJoCo simulation should show the G1 model performing the trained swing motion. Verify that the swing timing, racket trajectory, and balance recovery match the Isaac Lab training environment.

### 2.4  Hardware Connection

1. Connect the PC to the G1 via Ethernet cable
2. Set the Ethernet adapter to static IP: `192.168.123.11`
3. Identify the network interface name:
   ```bash
   ifconfig
   # Look for the interface connected to the robot, e.g., eth0, enp3s0
   ```
4. Verify connectivity:
   ```bash
   ping 192.168.123.1    # G1 default IP
   ```

### 2.5  Real Robot Launch

> ⚠️ **SAFETY WARNING**: Running RL policies on real robots is dangerous. The G1's arms can swing at speeds exceeding 3 m/s during table tennis strokes. Ensure all personnel are clear of the robot's workspace. Always have the Unitree RC joystick in hand with the E-stop ready.

**Pre-flight checklist:**

1. ☐ Robot is powered on and standing in a stable position
2. ☐ 3D-printed racket mount is securely attached to the right wrist link
3. ☐ Racket is mounted and the `T_mount` transform matches the simulation model (see WBC Training doc Section 2.8)
4. ☐ OptiTrack system is running and tracking the table (PPT), robot base_link (P1), and ball
5. ☐ `motion_capture_tracking` node is publishing `/P1/pose` and `/ball/point`
6. ☐ HOPE planner node is running and publishing `/racket/command`
7. ☐ E-stop (Unitree RC joystick button B) has been tested
8. ☐ All personnel are at least 2 m from the robot's arm reach envelope
9. ☐ Table is positioned correctly relative to the robot (verify in OptiTrack)

**Launch the controller:**

```bash
# Load from WandB
ros2 launch motion_tracking_controller real.launch.py \
    network_interface:=<network_interface> \
    wandb_path:=your-org/hope_wbc/run_id

# OR load from local ONNX file
ros2 launch motion_tracking_controller real.launch.py \
    network_interface:=<network_interface> \
    policy_path:=hope_forehand_policy.onnx
```

**Mode switching via Unitree RC joystick:**

| Button combination | Action |
|--------------------|--------|
| `L1 + A` | Enter standby controller (joint position control, safe standing pose) |
| `R1 + A` | Activate motion tracking controller (the trained WBC policy begins running) |
| `B` | **E-STOP** — immediate damping on all joints (use in any emergency) |

**Recommended startup sequence:**

1. Launch the controller node — the robot enters standby mode automatically
2. Verify that `/P1/pose` and `/racket/command` topics are active:
   ```bash
   ros2 topic hz /P1/pose           # Should be ~360 Hz
   ros2 topic hz /racket/command     # Should be ~50 Hz when ball is detected
   ```
3. Press `L1 + A` to confirm standby mode (robot holds standing pose)
4. Have a partner gently toss a ball onto the table
5. Press `R1 + A` to activate the WBC policy — the robot will begin responding to planner commands
6. If behavior is unexpected at any time, press `B` immediately

### 2.6  Code Structure

The `motion_tracking_controller` repository contains three key components:

```
motion_tracking_controller/
├── include/motion_tracking_controller/
│   ├── MotionTrackingController.hpp  # Manages observations (like RL env)
│   │                                  # and passes them to the policy
│   ├── MotionOnnxPolicy.hpp          # Wraps ONNX model, runs inference,
│   │                                  # extracts reference motion from ONNX
│   └── MotionCommand.hpp             # Observation terms aligned with
│                                      # training code
├── src/                               # C++ implementations
├── config/g1/                         # G1-specific configuration
│   ├── standby_controller.yaml       # PD gains for standby mode
│   └── state_estimation.yaml         # State estimator parameters
├── launch/
│   ├── mujoco.launch.py              # Sim-to-sim verification
│   └── real.launch.py                # Real robot deployment
└── package.xml
```

The `MotionTrackingController` class constructs the observation vector at each control step (50 Hz), matching the observation structure from the Isaac Lab / mjlab training environment and HITTER Table I. It reads:

- Joint positions and velocities (from robot encoders via `unitree_systems`)
- Base angular velocity and projected gravity (from onboard IMU)
- Previous action (the joint position targets from the last control step — standard RL observation for temporal smoothness)
- `base_link` pose (from motion capture via `/P1/pose` subscription)
- Racket target command: desired base XY position, desired racket position relative to base, desired racket velocity in world frame, time to strike (from planner via `/racket/command` subscription)
- Reference motion: target joint positions and velocities at the current and future timesteps, reference base angular velocity (extracted from the ONNX model's embedded 50 Hz reference trajectory via the `forward()` function)

The observation vector is passed to `MotionOnnxPolicy`, which runs the ONNX actor network and returns target joint positions. These are converted to joint torques by the PD controller in `legged_control2` and sent to the robot's actuator bus via Unitree's DDS interface.

### 2.7  PD Gain Tuning

The simulation trains with specific PD gains (stiffness `Kp` and damping `Kd`) per joint. The HITTER paper states that joint PD gains are "set heuristically following [BeyondMimic]" — meaning they are hand-tuned values (not learned), following the conventions in the BeyondMimic codebase. These gains are embedded in the ONNX model metadata by BeyondMimic's exporter and applied by `motion_tracking_controller` automatically at deployment. However, the real actuators may have different dynamic responses than the simulated ones, requiring gain adjustment.

**Signs that gains need tuning:**

- Joint oscillation or buzzing at high frequency → `Kd` too low or `Kp` too high
- Sluggish response, the arm does not reach the ball in time → `Kp` too low
- Jerky motion at transitions between standing and swinging → gain schedule needs smoothing

**Tuning procedure:**

1. Start with the simulation-trained gains from the ONNX metadata
2. Run the policy in MuJoCo sim-to-sim first — if behavior matches training, the gains are correct for simulation
3. On real hardware, begin with gains at 70% of simulation values and increase gradually
4. Monitor joint torque commands vs. actual joint positions via `ros2 topic echo`
5. Adjust `Kp` and `Kd` in the config YAML (`config/g1/standby_controller.yaml`), rebuild, and retry

---

## 3  Path B — Agibot A3 Deployment

The A3 runs on Agibot's AimRT middleware natively. AimRT is a lightweight C++20 runtime framework that supports multiple communication protocols including ROS 2, HTTP, gRPC, and Zenoh. For HOPE deployment, there are two options:

### 3.1  Option 1 — ROS 2 Bridge (Recommended for HOPE)

Use AimRT's ROS 2 protocol plugin to bridge between the HOPE ROS 2 graph and the A3's AimRT control bus. An AimRT node with the ROS 2 Humble/Jazzy plugin functions as a native ROS 2 node while internally communicating with the A3's actuator bus via AimRT's EtherCAT interface.

```
ROS 2 graph (HOPE):                   AimRT (A3 internal):
  /racket/command ──┐
  /P1/pose         ─┤                  ┌──────────────────────┐
                    ├──▶ AimRT node ──▶│  A3 actuator bus     │
  Joint feedback  ◀─┤    (ROS 2        │  (EtherCAT/FDCAN)    │
                    │     plugin)       └──────────────────────┘
                    └──
```

This approach reuses the same ONNX inference logic from `motion_tracking_controller`, adapted for the A3's joint naming and ordering. The main modifications are:

- **Joint mapping**: Replace G1 joint names with A3 joint names in the controller configuration
- **PD gains**: Set A3-specific stiffness and damping per joint (from Agibot's actuator datasheets)
- **Base_link transform**: Apply the A3's `base_link` offset (different from G1's pelvis location)
- **`T_mount` transform**: The A3's racket mount geometry will differ from the G1's

The Agibot X1/X2 AimDK already provides ROS 2 standard interfaces for joint control, following the `/aimdk_msgs/` message conventions. The A3 (Expedition series) is expected to follow the same pattern.

### 3.2  Option 2 — Native AimRT (Lowest Latency)

For minimum latency, load the ONNX policy directly in an AimRT module using the ONNX C++ Runtime, bypassing ROS 2 entirely. This eliminates the ROS 2 serialization/deserialization overhead (~0.1–0.2 ms per message) and runs the entire control loop within AimRT's single-process scheduler.

```cpp
// Pseudocode for AimRT-native ONNX inference module
class HopeWbcModule : public aimrt::ModuleBase {
    Ort::Session onnx_session_;
    
    void Initialize() override {
        // Load ONNX model
        onnx_session_ = Ort::Session(env, "hope_a3_policy.onnx", session_options);
    }
    
    void OnJointState(const JointStateMsg& msg) {
        // Build observation vector (same structure as training)
        auto obs = BuildObservation(msg, base_link_pose_, racket_command_);
        
        // Run ONNX inference
        auto action = onnx_session_.Run(obs);
        
        // Send joint position commands to actuator bus
        SendJointCommand(action);
    }
};
```

This path requires deeper integration with the A3's AimRT SDK and is recommended only for teams with Agibot development experience. The ROS 2 bridge path (Option 1) is simpler and sufficient for HOPE competition latency requirements.

### 3.3  A3 Installation

```bash
# 1. Install ROS 2 (Humble or Jazzy, depending on AimRT plugin version)
# Follow: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# 2. Clone AimRT (if using ROS 2 bridge)
git clone https://github.com/AimRT/aimrt.git
cd aimrt && mkdir build && cd build
cmake .. -DAIMRT_BUILD_WITH_ROS2=ON
make -j$(nproc) && sudo make install

# 3. Install Agibot-specific packages
# Follow Agibot's developer documentation for the A3/Expedition series
# Reference: https://github.com/AgibotTech/agibot_x1_infer for X1 patterns

# 4. Build HOPE controller for A3
# Adapt motion_tracking_controller for A3 joint configuration
cd ~/colcon_ws/src
git clone <your-hope-a3-controller-repo>
cd ~/colcon_ws
colcon build --symlink-install --packages-up-to hope_a3_controller
source install/setup.bash
```

---

## 4  Integrating the Full HOPE Pipeline

### 4.1  Launch Order

The full HOPE system requires multiple nodes launched in a specific order:

```bash
# Terminal 1: Motion capture bridge
ros2 launch motion_capture_tracking optitrack.launch.py \
    server_ip:=192.168.1.100

# Terminal 2: HOPE planner
ros2 run hope_planner hope_planner_node \
    --ros-args -p table_origin_frame:=PPT

# Terminal 3: WBC controller (G1 example)
ros2 launch motion_tracking_controller real.launch.py \
    network_interface:=enp3s0 \
    policy_path:=/home/user/hope_forehand_policy.onnx

# Terminal 4 (optional): Monitoring
ros2 topic hz /ball/point /P1/pose /racket/command
```

### 4.2  Topic Map

| Topic | Type | Publisher | Subscriber(s) | Rate |
|-------|------|-----------|---------------|------|
| `/ball/point` | `geometry_msgs/PointStamped` | `motion_capture_tracking` | HOPE Planner | 360 Hz |
| `/P1/pose` | `geometry_msgs/PoseStamped` | `motion_capture_tracking` | HOPE Planner, WBC Controller | 360 Hz |
| `/P2/pose` | `geometry_msgs/PoseStamped` | `motion_capture_tracking` | (opponent's controller) | 360 Hz |
| `/table/pose` | `geometry_msgs/PoseStamped` | `motion_capture_tracking` | (drift monitor) | 10 Hz |
| `/racket/command` | `hope_msgs/RacketCommand` | HOPE Planner | WBC Controller | 50 Hz |
| `/joint_states` | `sensor_msgs/JointState` | Robot driver | WBC Controller | 500 Hz |

### 4.3  QoS Profiles

For real-time control, all high-frequency topics must use Best Effort reliability with Keep Last 1 history depth:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

mocap_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

The planner and WBC controller should use this QoS for all subscriptions. Using Reliable QoS will introduce retransmission latency that violates the 20 ms budget.

---

## 5  Sim-to-Real Gap Mitigation

### 5.1  Common Failure Modes

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot falls immediately when policy activates | Gravity/mass mismatch between sim and real | Verify URDF inertial params; start with lower Kp |
| Swing never reaches the ball | PD gains too low on real hardware | Increase Kp by 10% increments |
| Arm oscillates during swing | Kd too low; actuator backlash not modeled | Increase Kd; add low-pass filter on action output |
| Robot drifts sideways during play | IMU bias not calibrated; base_link mocap offset wrong | Re-calibrate IMU; verify mocap marker offset |
| Ball contact but no return | `T_mount` mismatch between sim and real | Re-measure the physical racket mount transform |
| Forehand/backhand selection wrong | Base_link Y offset from table center incorrect | Verify robot placement relative to table origin |

### 5.2  Observation Noise Matching

The training environment adds observation noise to simulate real sensor imperfections (see WBC Training doc Section 4.6, domain randomization). The real-world noise characteristics should be within the training distribution:

- **Joint encoder noise**: Unitree G1 encoders are ±0.01 rad — well within typical training noise of ±0.05 rad
- **IMU noise**: Onboard IMU angular velocity noise ~0.01 rad/s — within training noise of ±0.2 rad/s
- **Mocap base_link noise**: OptiTrack at 360 Hz provides sub-mm accuracy — much better than the ±5 mm training noise
- **Latency**: The ~10–15 ms total latency is within the 1-step (20 ms) observation delay modeled in training

If any real sensor is noisier than the training distribution, the policy may degrade. The fix is to retrain with increased noise parameters in the domain randomization config.

---

## 6  Competition Workflow

A typical HOPE competition match follows this sequence:

1. **Setup** (5 min): Position robot at the table, power on, verify mocap tracking, attach racket
2. **Calibration** (2 min): Verify `base_link` and `T_mount` in OptiTrack; confirm planner receives `/ball/point`
3. **Pre-flight** (1 min): Run through the pre-flight checklist (Section 2.5)
4. **Warm-up** (30 s): Activate policy in standby; toss a test ball to verify planner + WBC connectivity
5. **Match**: Press `R1 + A` (G1) to activate the WBC; opponent serves
6. **E-stop**: If the robot behaves unexpectedly, press `B` immediately; the robot enters joint damping mode
7. **Teardown**: Press `L1 + A` to return to standby; power down

---

## 7  Known Limitations

1. **Zero-shot transfer is not guaranteed in the strict hardware sense.** The HITTER paper states the policy is "deployed zero-shot to the real robot," meaning no additional real-world reinforcement learning or fine-tuning is performed — the simulation-trained policy runs directly on hardware. However, "zero-shot" in this context does not mean zero hardware configuration. PD gain adjustment, `T_mount` verification, and mocap calibration are infrastructure setup steps that HITTER presumably performed but does not describe. Teams should expect that the first deployment on new hardware requires these adjustments even if the policy itself is unchanged.

2. **No online adaptation.** The deployed ONNX policy is a frozen neural network. It does not learn or adapt during play. Performance degradation due to actuator wear, temperature changes, or loose racket mounts must be addressed offline.

3. **Forehand/backhand switching requires two ONNX models.** BeyondMimic's ONNX exporter embeds a single reference motion per file. Since HITTER uses two reference motions (forehand and backhand), deployment requires two ONNX models. The HITTER paper triggers forehand when the ball's Y position is negative (robot's right side) and backhand when positive (left side). The runtime switching logic — loading both ONNX sessions and selecting which to query each control step based on the planner's swing type signal — must be implemented in the controller node. This is not provided by `motion_tracking_controller` out of the box, which loads a single ONNX file. A multi-skill policy trained with BeyondMimic's diffusion distillation (future work) would eliminate this dual-model requirement.

4. **A3 deployment is experimental.** The Agibot A3 deployment path is based on AimRT patterns from the X1/X2 platforms. The A3 (Expedition series) is pre-production and its exact joint-level control API may differ.

5. **No ball spin response.** Consistent with the training setup, the deployed policy does not adjust racket orientation to counteract incoming spin.

---

## References

- BeyondMimic deployment code: https://github.com/HybridRobotics/motion_tracking_controller
- `legged_control2` documentation: https://qiayuanl.github.io/legged_control2_doc/
- `unitree_bringup`: https://github.com/qiayuanl/unitree_bringup
- `legged_template_controller` (minimal example): https://github.com/qiayuanl/legged_template_controller
- AimRT framework: https://github.com/AimRT/aimrt
- Agibot X1 inference (reference for A3 patterns): https://github.com/AgibotTech/agibot_x1_infer
- Agibot X2 AimDK ROS 2 interfaces: https://x2-aimdk.agibot.com/en/latest/Interface/index.html
- Unitree SDK2: https://github.com/unitreerobotics/unitree_sdk2
- Companion documents:
  - *HOPE Motion Capture System Reference Setup for Ping-Pong Arena*
  - *HOPE 7DOF Racket Model-based Planner Reference Setup*
  - *HOPE WBC Simulation Training Reference Setup*
