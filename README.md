# HOPE: Hitch Open Ping-Pong Embodied AI Challenge

HOPE is an open platform for humanoid robot table tennis, developed by [Hitch Interactive](https://hitchinteractive.com) (Intelligent Racing Inc.) in collaboration with the [ROAR Platform](https://roar.berkeley.edu) at UC Berkeley. The challenge invites teams to deploy whole-body humanoid controllers that can rally a ping-pong ball against human opponents or other robots, using off-the-shelf humanoid hardware and an open-source perception and planning stack.

This repository contains the full HOPE stack for the Agibot A3: Isaac Lab training for a **unified forehand/backhand whole-body policy** (the deploy-grade 110-D `hitter_pure` rally line, proven on real A3 hardware), a **no-spin planner** in both Python and low-latency C++ (ROS 2), a **MuJoCo/AimRT simulation with a real ball plant** for closed-loop evaluation, and the **native C++ A3 deploy runner** (`a3_pingpong`) — plus the preserved HOPE **reference design documents**, the **challenge rulebooks**, and the Agibot-provided A3 starter materials under `agibot/`.

## How To Read This Repository

Start with this README, run the published model via
[docs/MODEL_21800.md](docs/MODEL_21800.md), then follow
[QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) to train or export your own.
The rest of the repository is organized into four layers:

| Layer | What to read or run | Purpose |
|-------|---------------------|---------|
| Required path | `QUICKSTART_A3_ISAAC.md`, `hope_training/whole_body_tracking/` (incl. `scripts/prepare_a3_isaac_asset.py`), `agibot/URDF/A3T2.5-URDF-std-pingpang/` | Prepare the A3 Isaac asset, train the deploy-grade rally policy (`task=HOPEPingPong`), export the ONNX policy, and evaluate it in Isaac and MuJoCo. |
| Stable public contracts | `A3_ASSETS.md`, `docs/interfaces/`, `docs/POLICY_INTERFACE.md`, `docs/PLANNER_INTERFACE.md` | Frame conventions, joint order, the 110-D `hitter_pure` observation / 31-D action policy IO, ROS topics incl. the schema-tagged flat wire, `RacketCommand`, and asset expectations that stay stable when you integrate your own code. |
| Deploy and simulation references | `apps/a3_mujoco_serve/`, `a3_deploy/`, `agibot/`, `docs/RUN_ON_AGIBOT.md` | The self-contained deterministic MuJoCo → DLS IK → CSV → high-level A3 serve app; the native C++ deploy runner with its gate/rehearsal scripts (`a3_deploy/a3_deploy_example/`) and the MuJoCo/AimRT simulation fork with the real ball plant (`a3_deploy/A3_MuJoCo_Sim/`); and Agibot-provided A3 materials. |
| Background material | `NatNet2ROS2/`, `VRPN2ROS2/`, `hope_ws/`, `mocap/`, root `HOPE_*_Reference_Setup.md`, `REFERENCE_DOCS.md`, `ROADMAP.md` | The independent raw-mocap adapters and HOPE ROS 2 planner workspace (Python + C++ planners) for arena integration, the mocap frame/topic docs, the preserved design documents, and current scope/direction. |

A fresh clone includes the public `model_21800` checkpoint, its exact deploy
ONNX/parameters, and a MuJoCo video. Other generated Isaac assets, training logs,
checkpoints, exported `.onnx` files, and ROS build artifacts remain git-ignored;
Agibot-provided materials under `agibot/` are tracked.

## Run `model_21800`

```bash
python3 -m venv .venv-mujoco
source .venv-mujoco/bin/activate
python -m pip install -r a3_deploy/a3_deploy_example/reference/requirements.txt
a3_deploy/a3_deploy_example/scripts/run_pingpong_sim.sh --view --realtime
```

The default runtime selects the deployed 110-D `hitter_pure` actor. See the
[complete model guide](docs/MODEL_21800.md) and
[Gate 3 MuJoCo video](docs/assets/model_21800_gate3_mujoco.mp4).

## Train and export your own

```bash
# 1. Prepare the A3 Isaac asset (bundled racket-equipped URDF)
cd hope_training/whole_body_tracking
python3 scripts/prepare_a3_isaac_asset.py --force

# 2. Train the deploy-grade rally policy (inside your Isaac Lab Python environment).
#    The bundled clips are schema placeholders — swap in real forehand/backhand clips
#    (docs/REPLACE_MOTIONS.md) before training a policy you intend to deploy.
source setup_train_env.sh        # defines the `hope_isaac_py` Isaac Sim launcher
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true \
    motion_file=../motions/preprocessed/hope_forehand.npz \
    motion_file_2=../motions/preprocessed/hope_backhand.npz

# 3. Evaluate the checkpoint in Isaac
hope_isaac_py scripts/evaluate.py \
    --checkpoint logs/rsl_rl/agibot_a3_hitter_pingpong/<run>/model_<iter>.pt

# 4. Export the deployable actor -> <run>/exported/
hope_isaac_py scripts/export_onnx.py \
    --checkpoint logs/rsl_rl/agibot_a3_hitter_pingpong/<run>/model_<iter>.pt

# 5. MuJoCo sim-to-sim check of the exported ONNX (real ball physics)
python3 scripts/mujoco_eval_onnx.py \
    --onnx logs/rsl_rl/agibot_a3_hitter_pingpong/<run>/exported/policy.onnx
```

To close the loop with the planner (ROS 2):

```bash
cd hope_ws && colcon build && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true   # mocap-free smoke test

# in another terminal (same ROS env sourced):
cd a3_deploy/a3_deploy_example
PYTHONPATH=reference python3 -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

The staged evaluation ladder (Isaac eval → MuJoCo sim-to-sim → closed-loop
MuJoCo with the real planner → hardware) is documented in
[docs/TRAIN_POLICY.md](docs/TRAIN_POLICY.md#evaluation) and
[docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md).

## ROS 2 motion-capture adapters and shared NTP time

HOPE provides two independently built ROS 2 motion-capture adapters:
[NatNet2ROS2](NatNet2ROS2/README.md#ros-2-ntp-timestamp-estimation) for
OptiTrack Motive/NatNet and
[VRPN2ROS2](VRPN2ROS2/README.md#ros-2-ntp-timestamp-estimation-and-validation)
for Chingmu CMTracker/MCServer. Both paths express every accepted ROS 2 header
timestamp in the adapter host computer's NTP-disciplined
`RCL_SYSTEM_TIME`/Unix epoch. Here “world clock” means absolute UTC/Unix wall
clock, not the ROS `world` coordinate frame.

Both adapters validate every received source report before applying a
configurable ROS 2 output-rate cap, reducing DDS traffic without changing the
selected report's source timestamp. Set `output_rate_hz:=0.0` to publish
every accepted report.

| Adapter | Default ROS 2 output cap | Downsampled output |
|---|---:|---|
| **NatNet2ROS2** | **200 Hz** | One filtered `/optitrack/poses` array containing only available `Ball`, `P1`, and `P2` entries; an empty array is the live-source/no-competition-body heartbeat; no raw marker cloud or duplicate TF output |
| **VRPN2ROS2** | **200 Hz** | Each pose, velocity, and acceleration topic independently, per tracker sensor |

| Adapter | Source timestamp | Conversion into the adapter host world clock | Trust requirement |
|---|---|---|---|
| **NatNet2ROS2** | Motive `CameraMidExposureTimestamp` in the Motive QPC domain | NatNet echo exchanges estimate the QPC-to-adapter-steady-clock mapping; the measured capture age is subtracted from the adapter's Chrony-disciplined `RCL_SYSTEM_TIME`. Motive's Windows wall clock is not used for this conversion. The filtered ROS pose array is capped at 200 Hz by default (`output_rate_hz` is configurable). | Mapping age and uncertainty must pass the configured gates. |
| **VRPN2ROS2** | CMTracker/MCServer report `timeval` | The source seconds/microseconds are decoded as Unix time and validated against the adapter's Chrony-disciplined `RCL_SYSTEM_TIME`; absolute-age and sliding minimum-age gates detect invalid or changed timing regimes. Raw ROS outputs are capped at 200 Hz per topic/sensor by default. | The VRPN server host must already share the same NTP epoch; VRPN itself does not synchronize clocks. |

The adapter computer and the humanoid robot computer **must be synchronized to
the same approved NTP server or equivalent common time source before live
mocap control**. Matching ROS message types or plausible-looking timestamps is
not evidence of synchronization. For the VRPN path, the CMTracker/MCServer host
must also be NTP-disciplined because its wall-clock `timeval` is carried on the
wire. NatNet's QPC mapping avoids depending on the Motive Windows wall clock,
but the adapter host and robot must still share UTC so the resulting ROS stamp
can be compared with robot state and future execution time.

The Agibot A3 implementation is under
[`agibot/ntp_sync/`](agibot/ntp_sync/README.md). Chrony disciplines the A3 HDU
system clock, and the supervised HDU-to-MDU PTP chain distributes that time to
the internal controller. See also the
[full clock synchronization plan](docs/HOPE_A3_Clock_Synchronization_Improvement_Plan.pdf).
Internet/NTP loss does not block ordinary offline A3 applications, but the
robot is not qualified for external-mocap strike timing until both the adapter
and A3 clock-health gates pass. Clock drift or mixed domains can corrupt state
estimation, latency measurement, recording, calibration, and strike timing
even while ROS 2 topics appear healthy.

## Optional marker-CAD alignment: P1 to A3 `pelvis_link`

The imported Foxglove work includes a ten-waist-marker CAD calibration tool.
It is not invoked by the integrated V17 console: the native flow uses Stand,
`/hope/v17/refresh_x_hit`, and Ready, while the ten-marker topic remains
operator telemetry. The colleague branch's `/hope/control/enter_prepare`
orchestration belongs to the legacy TTY adapter and is not bridge-exposed in
this integration.

When an approved setup procedure explicitly requires a new P1-to-pelvis
calibration, run the tool on the external computer only after the Runner has
entered and settled in PD_STAND. A successful fit atomically replaces the
computer's `calibration/p1_to_pelvis.json`; the resulting matrix is then fixed
for that policy run.
The tool registers Motive's P1-local marker centres
to the A3 hip-shell CAD centres (`f1`–`f5`, `b1`–`b5`) and requires live
same-frame samples for every selected marker to pass the physical-layout and
residual gates. The named non-collinear 3-D layout makes the fixed six-DOF
transform observable while the robot is stationary in PD_STAND.

```bash
cd <HOPE_REPO>
source hope_ws/install/setup.bash
ros2 run hope_bringup p1_marker_cad_calibrator \
  --topic /optitrack/rigid_body_markers \
  --asset-name P1 \
  --marker-names f1,f2,f3,f4,f5,b1,b2,b3,b4,b5 \
  --stationary-prepare \
  --attest-installed-layout \
  --allow-nominal-only-markers \
  --output calibration/p1_to_pelvis.json
```

The relative path is resolved from the external computer's HOPE repository
root (for example, `/home/user/HOPE/calibration/p1_to_pelvis.json`). After the
atomic replacement, the computer-side `hope_base_pose_flat_relay` only reads
that file for the rest of the run, composes the live `world → P1` pose with the
fixed `P1 → pelvis_link` transform, and publishes `/a3/base_pose_flat`. It also
publishes the unshifted reconstructed pose on `/a3/mocap/pelvis_pose` for
diagnostics. No recalculation occurs while the robot is playing. The robot
receives the final `/a3/base_pose_flat` stream only; it never stores, reads, or
receives the calibration JSON.

`/a3/calibration/pelvis_pose` is different: it is the independent
`world → pelvis_link` input of the older two-PoseStamped
`p1_pelvis_calibrator`. No checked-in hardware node publishes that topic. It is
not the result of the ten-marker calculation, and feeding a P1-derived result
to it would make that older calibration circular. Keep the pose-pair tool only
for an explicitly independent external 6-DOF reference or simulation check.

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
| `configs/` | The shared no-spin ball model: the generic [ball_physics.yaml](configs/ball_physics.yaml) plus the real venue fits [ball_physics_venue.yaml](configs/ball_physics_venue.yaml) and [incoming_ball_venue.yaml](configs/incoming_ball_venue.yaml) (measured drag/restitution and the serve envelope used by the proven line). |
| `hope_training/` | The Isaac Lab training extension (`whole_body_tracking/` with the `HitterPingPong` task and the train/eval/export scripts, including `scripts/prepare_a3_isaac_asset.py`), placeholder motion clips (`motions/preprocessed/`), the canonical A3 joint order (`config/joint_order_agibot_a3.yaml`), and the ball-physics fitting tools (`ball_physics_fit/`). |
| `NatNet2ROS2/` | Independent ROS 2 workspace for the OptiTrack/Motive NatNet adapter, named-pose interfaces, acquisition-time mapping, and driver tests. Build and launch it separately from `hope_ws`. |
| `VRPN2ROS2/` | Independent ROS 2 workspace for the ChingMu/VRPN client, strict server-time/NTP validation, and raw per-tracker `PoseStamped` topics. |
| `hope_ws/` | ROS 2 workspace: `hope_planner` (Python no-spin planner + presets + fake-planner mode), `hope_planner_cpp` (low-latency C++ planner used on hardware), `hope_bringup` (relays, world-frame publisher, calibration tools, time-sync configs, fake publishers), `hope_msgs` (`RacketCommand.msg`), and `calibration_receipts/` (venue calibration evidence). Raw acquisition lives in the two sibling adapter workspaces. Bring-up guides: [BRINGUP_TUTORIAL](hope_ws/BRINGUP_TUTORIAL.md), [SMOKE_TEST](hope_ws/SMOKE_TEST.md), [SHADOW_MODE](hope_ws/SHADOW_MODE.md). |
| `a3_deploy/` | The native C++ deploy runner, gate/rehearsal script suite, parity harness, and deploy runbooks (`a3_deploy_example/`); the MuJoCo/AimRT simulation fork with the real ball plant (`A3_MuJoCo_Sim/`); and the optional user-supplied URDF override location (`URDF/`). |
| [`apps/a3_mujoco_serve/`](apps/a3_mujoco_serve/README.md) | Self-contained deterministic serve contribution: official A3 MuJoCo model/racket contact, legal-serve physics search, DLS IK, all-joint CSV export, replay validation, and the PR #18 high-level A3 runtime. |
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
       │ NatNet2ROS2 (standalone)     │   │ VRPN2ROS2 (standalone)       │
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
                    │  hope_planner /              │
                    │  hope_planner_cpp (Stages 1–3)│
                    │                              │
                    │  Ball state estimation       │
                    │  → no-spin trajectory        │
                    │    prediction                │
                    │  → racket target planning    │
                    │    (side, position, velocity,│
                    │     face normal, timing)     │
                    └──────────┬──────────────────┘
                               │ /racket/command (RacketCommand, tooling)
                               │ /racket/command_flat + /a3/base_pose_flat
                               │ (schema-tagged Float64MultiArray — hardware wire)
                               ▼
                    ┌─────────────────────────────┐
                    │  a3_pingpong C++ runner      │
                    │  (Stage 4, --planner mode)   │
                    │                              │
                    │  policy.onnx @ 50 Hz         │
                    │  110-D obs → 31-D action     │
                    │                              │
                    │  Receives:                   │
                    │   • racket command flats     │
                    │   • mocap base pose flats    │
                    │   • Joint encoders (iceoryx) │
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

> The diagram shows both supported source paths: **Chingmu/VRPN** uses the
> independent `VRPN2ROS2` client + `pose_to_posearray` (the default
> `mocap_backend:=vrpn`),
> while **OptiTrack/Motive uses NatNet** through the independently built
> `NatNet2ROS2` driver plus the `hope_ws` `optitrack_mct_relay`
> (`mocap_backend:=optitrack`). Both feed the identical `/poses` contract, so
> everything below that hop is unchanged. See
> [docs/OPTITRACK.md](docs/OPTITRACK.md).

The Python reference harness drives the same policy in plain MuJoCo
(`a3_deploy/a3_deploy_example/scripts/run_pingpong_sim.sh`); the native C++
runner (sources under `a3_deploy/a3_deploy_example/src/`) drives either the
shipped MuJoCo/AimRT simulation or, cross-built for the robot's motion unit,
the real A3 ([docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md)).

## Key Design Decisions

**Racket tracking is prohibited.** During competition the motion capture system streams the named rigid bodies `Ball`, `P1`, and `P2` — the ball and the two humanoid marker-cluster frames. A calibrated static transform maps each P1/P2 frame to that robot's declared URDF root frame (`pelvis` on Unitree G1; `pelvis_link` on Agibot A3). The table is a calibrated static world origin (a `Table` asset is used during setup only and appears only in training-data recordings). The ball stream contains position `(x, y, z)` and orientation, represented in ROS 2 as a quaternion `(qx, qy, qz, qw)`. Pitch, yaw, and roll are derived views, not fields carried by `geometry_msgs/Pose`. No reflective markers may be placed on the racket, the robot's hand, or the wrist link. Each robot must infer its paddle's 6-DOF pose through forward kinematics from its declared root frame plus joint encoders. This is a deliberate competition constraint that tests autonomous paddle control through the robot's internal body model.

**Implementation scope.** The preserved reference documents include robot-specific integration examples; the code currently shipped in this repository implements the Agibot A3 (31 actuated DOF) path end to end: Isaac Lab training of one unified forehand/backhand policy on the 110-D `hitter_pure` contract (the `HitterPingPong` task, gym id `HOPE-HitterPingPong-AgibotA3-v0` — the recipe validated on real hardware), Isaac evaluation, MuJoCo sim-to-sim and closed-loop evaluation with a real ball plant, and the native C++ deploy runner — alongside Agibot's own deploy example and MuJoCo/AimRT simulation reference.

**Open-source training stack.** The WBC training pipeline is built entirely on open-source code: [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) (MIT license), from which the `hope_training/whole_body_tracking/` extension derives, [GMR](https://github.com/YanjieZe/GMR) (MIT license) for SMPL-X to robot retargeting, and [GVHMR](https://github.com/zju3dv/GVHMR) for monocular video-to-SMPL-X extraction. The HITTER paper's trained weights are not released; all training starts from scratch, and the bundled motion clips are placeholders to be replaced with your own retargeted swings ([docs/REPLACE_MOTIONS.md](docs/REPLACE_MOTIONS.md)).

## Supported Robots

The table below reports current repository implementation coverage.

| Robot | DOF | Status |
|-------|-----|--------|
| Agibot Expedition A3 | 31 actuated | Shipped end-to-end path: Isaac Lab training (deploy-grade `hitter_pure` rally line), ONNX export with fail-closed metadata, Isaac/MuJoCo evaluation gates, native C++ deploy runner — exercised on real A3 hardware — plus the Agibot vendor deploy example. |
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
- **Real arena**: OptiTrack Motive streams **NatNet** through the independently built/launched `NatNet2ROS2` workspace and the HOPE-side `optitrack_mct_relay` (`mocap_backend:=optitrack`; set Motive **Up Axis = Z** and see [docs/OPTITRACK.md](docs/OPTITRACK.md)). Chingmu CMTracker/MCServer streams **VRPN** through the independent `VRPN2ROS2` workspace and HOPE-side `pose_to_posearray` (`mocap_backend:=vrpn`). In either case configure the named 6-DOF competition rigid bodies `Ball`, `P1`, and `P2`; `Table` is calibration-only and is not streamed during competition.

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
