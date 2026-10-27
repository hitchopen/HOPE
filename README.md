# HOPE: Hitch Open Ping-Pong Embodied AI Challenge

Mac users: see [macOS development and operator setup](docs/operations/macos_deployment.md).
The operator backend supports any normal Mac login, with explicit Mac/Linux
profiles and separate configurable container/robot accounts; no personal Mac
username or local administrator account is hardcoded. Console 1.8.18 includes
operator, gripper and NatNet/VRPN selectors. The final Mac build includes both
mocap workspaces, uses the `hope` container group, and preserves schema-5 venue
settings through the combined schema-6 backend. It does not deploy to the robot
or change robot motions or scheduling.

Normal play now verifies the P1/P2 table end from raw mocap at START and returns
to PD_STAND on base-stream or operator-link loss. A gap over 20 ms uses a marked
last-pose hold without shortening mode admission or resetting the 100 ms rescue
deadline. Separate Console (1 s) and operator-computer (100 ms) heartbeats
protect normal mocap play only.
Offline Kernel/Pure Serve and Teleop retain their existing behavior;
see [deployment safety](docs/operations/foxglove_lifecycle.md#player-assignment-and-mocap-loss).

A3 operations: [ELink startup checks and runtime diagnostics](docs/operations/foxglove_lifecycle.md#elink-startup-checks-and-runtime-diagnostics)
detect startup process/IPC and log failures without adding telemetry, battery or
clock-based motion gates. Normal Stand, Serve and Rally retain the existing
Runner/vendor protections. Clock correction uses the explicit, stopped
[Time Calibration workflow](docs/operations/foxglove_lifecycle.md#pre-operation-clock-correction).
Follow the matched-component installation and hardware validation requirements
in those guides before deployment.

Current deployment: [build and deployment](docs/operations/deploy_and_gate3.md#model17400-pr87-runtime-alignment),
[shared ball physics](docs/operations/ball_physics_calibration.md),
[Foxglove recording and live Motive localization](foxglove/extensions/hope-a3-console/README.md),
and the [17-marker shell](agibot/mocap_sticker_shell_v3/README.md).
The training recipe remains `task=HOPEPingPong algo=ppo`; deployment uses the
verified model17400 bundle described below.

HOPE is an open platform for humanoid robot table tennis, developed by [Hitch Interactive](https://hitchinteractive.com) (Intelligent Racing Inc.) in collaboration with the [ROAR Platform](https://roar.berkeley.edu) at UC Berkeley. The challenge invites teams to deploy whole-body humanoid controllers that can rally a ping-pong ball against human opponents or other robots, using off-the-shelf humanoid hardware and an open-source perception and planning stack.

This repository contains the full HOPE stack for the Agibot A3: Isaac Lab training for a **unified forehand/backhand whole-body policy** (the deploy-grade 110-D `hitter_pure` rally line, proven on real A3 hardware), a **no-spin planner** in both Python and low-latency C++ (ROS 2), a **MuJoCo/AimRT simulation with a real ball plant** for closed-loop evaluation, and the **native C++ A3 deploy runner** (`a3_pingpong`) — plus the preserved HOPE **reference design documents**, the **challenge rulebooks**, and the A3 source assets, reference software and mounting hardware indexed in [agibot/README.md](agibot/README.md).

The current printable mounting hardware is the
[v3 pelvis marker shell](agibot/mocap_sticker_shell_v3/README.md) and the
[right wrist paddle flange](agibot/right_wrist_paddle_flange/README.md).
Their component guides link the release STLs, printing instructions and marker
transforms to `pelvis_link`.

## How To Read This Repository

Start with the verified new-machine setup in
[`docs/DISTROBOX_SETUP.md`](docs/DISTROBOX_SETUP.md), then use the
[managed A3 deployment workflow](docs/operations/foxglove_lifecycle.md) and
[current deployment](#a3-deployment-model17400--ppq004) below.
Follow [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) to train or export your own.
The rest of the repository is organized into four layers:

| Layer | What to read or run | Purpose |
|-------|---------------------|---------|
| Required path | `docs/DISTROBOX_SETUP.md`, `QUICKSTART_A3_ISAAC.md`, `hope_training/whole_body_tracking/` (incl. `scripts/prepare_a3_isaac_asset.py`), `agibot/URDF/A3T2.5-URDF-std-pingpang/` | Reproduce the verified workstation, prepare the A3 Isaac asset, train the deploy-grade rally policy (`task=HOPEPingPong`), export the ONNX policy, and evaluate it in Isaac and MuJoCo. |
| Stable public contracts | `A3_ASSETS.md`, `docs/interfaces/`, `docs/POLICY_INTERFACE.md`, `docs/PLANNER_INTERFACE.md` | Frame conventions, joint order, the 110-D `hitter_pure` observation / 31-D action policy IO, ROS topics incl. the schema-tagged flat wire, `RacketCommand`, and asset expectations that stay stable when you integrate your own code. |
| Deploy and simulation references | `apps/a3_mujoco_serve/`, `a3_deploy/`, `agibot/`, `docs/RUN_ON_AGIBOT.md` | The self-contained deterministic MuJoCo → DLS IK → CSV → high-level A3 serve app; the native C++ deploy runner with its gate/rehearsal scripts (`a3_deploy/a3_deploy_example/`) and the MuJoCo/AimRT simulation fork with the real ball plant (`a3_deploy/A3_MuJoCo_Sim/`); and Agibot-provided A3 materials. |
| Background material | `NatNet2ROS2/`, `VRPN2ROS2/`, `hope_ws/`, `mocap/`, root `HOPE_*_Reference_Setup.md`, `REFERENCE_DOCS.md`, `ROADMAP.md` | The independent raw-mocap adapters and HOPE ROS 2 planner workspace (Python + C++ planners) for arena integration, the mocap frame/topic docs, the preserved design documents, and current scope/direction. |

A fresh clone includes the source, reference motions and tracked robot assets.
The current model17400 deployment requires its verified trained-policy bundle;
do not infer that it is installed from the presence of repository source alone.
Generated Isaac assets, training logs, checkpoints and ROS build artifacts are
not deployment inputs unless explicitly included in the staged bundle.

## Reproduce the working environment on a new machine

[`docs/DISTROBOX_SETUP.md`](docs/DISTROBOX_SETUP.md) is the single source of
truth for workstation installation. It records the current working machine and
provides complete copy-paste flows for two isolated environments:

- `grasping`: a digest-pinned NVIDIA Distrobox with Isaac Sim 5.1.0, Python
  3.11, Isaac Lab 0.54.2, PyTorch 2.7.0+cu128 and `rsl_rl` 3.1.2;
- `hope`: Ubuntu 24.04 with ROS 2 Jazzy and the native C++/MuJoCo build
  dependencies.

Do not replace individual packages with their latest releases during initial
setup. First reproduce the pinned environment and pass its CUDA/import/asset
smoke; treat a later Isaac or PyTorch upgrade as a separate migration.

## A3 deployment: model17400 + PPQ004

Current robot deployment uses **model17400 with PPQ004** through the
Foxglove-managed Laptop/Mac → HDU → MDU lifecycle. It is a **receive-only**
policy: normal mocap receive play is supported; serving and Kernel mode are not
qualified for this bundle.

Stage the exact policy/deploy export with
[`prepare_model17400_bundle.py`](a3_deploy/a3_deploy_example/scripts/prepare_model17400_bundle.py)
and a complete target-matched Runner distribution. Install its generated MDU
deployment and HDU Planner profile/catalog entries. While STOPPED, select
**17400 · Receive only** and **PPQ004**, configure the gripper, mocap source and
table side, then confirm configuration and START. HDU/MDU policy and deploy
hashes must agree; PPQ004 must match on the Planner and operator packetizer.
These installed selections are verified again before hardware startup.

START leaves the Runner in PASSIVE. Use the attended Stand/Ready workflow to
enter receive play. P1/P2 occupancy is verified from unrotated mocap at START;
the side is then locked until STOP. During play, loss of the base-pose stream
requests controlled PD_STAND recovery and does not automatically resume play.
Foxglove and the managed operator relay must each return genuine challenge
echoes; loss triggers recovery after 1 s for the Console or 100 ms for the
operator-computer relay. Switching away from the Console for over 1 s can
therefore stop play; qualify deployed UI timing before operation.
See the [lifecycle installation and operation guide](docs/operations/foxglove_lifecycle.md),
[Console model controls](foxglove/extensions/hope-a3-console/README.md#model17400-receive-policy)
and [model17400 runtime alignment](docs/operations/deploy_and_gate3.md#model17400-pr87-runtime-alignment).
Use the verified installed bundle and current runbook, not a model number inferred
from a layout, service, session-prefix or file name. Local regression checks do
not establish a fresh live-hardware qualification.

## Train and export your own

```bash
# 1. Prepare the A3 Isaac asset (bundled racket-equipped URDF)
distrobox enter grasping
if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh
hope_isaac_py scripts/prepare_a3_isaac_asset.py --force

# 2. Train the deploy-grade rally policy (inside the pinned Isaac environment).
#    The complete validated forehand/backhand reference pair is included.
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

The complete validated reference pair is included under the stable public names
`hope_training/motions/preprocessed/hope_forehand.npz` and `hope_backhand.npz`.
Use [docs/REPLACE_MOTIONS.md](docs/REPLACE_MOTIONS.md) only to substitute your own
retargeted motions. See [QUICKSTART_A3_ISAAC.md](QUICKSTART_A3_ISAAC.md) for the full
install → train → export → evaluate → run loop, and
[docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md) for the deploy path.

## ROS 2 motion-capture adapters and shared NTP time

HOPE provides two independently built ROS 2 motion-capture adapters:
[NatNet2ROS2](NatNet2ROS2/README.md#ros-2-ntp-timestamp-estimation) for
OptiTrack Motive/NatNet and
[VRPN2ROS2](VRPN2ROS2/README.md#ros-2-ntp-timestamp-estimation-and-validation)
for Chingmu CMTracker/MCServer. Both paths express every accepted ROS 2 header
timestamp in the adapter host computer's NTP-disciplined
`RCL_SYSTEM_TIME`/Unix epoch. Here “world clock” means absolute UTC/Unix wall
clock, not the ROS `world` coordinate frame.

The OptiTrack competition baseline is **MotiveBody 3.5.0.1 Beta 1 with native
NatNet 4.5 multicast**: command port `1510`, data port `1511`, and multicast
group `239.255.42.99`. Motive's arena-network Local Interface is
`192.168.50.1`; pass that server address as NatNet2ROS2 `hostname`, while
`interface_ip` must be the adapter computer's own wired-NIC IPv4 address.
Because multicast always carries Motive's native bitstream, the adapter's
bounded NatNet 4.1+ decoder skips the additive 4.5 IMU/GPIO sections safely
instead of requesting an older stream version. The venue preflight reports
MotiveBody/NatNet version differences as warnings rather than rejecting a
working patch or beta revision; multicast settings, competition assets, clock
mapping, live decoding, and native frame rate remain blocking checks. See the
[OptiTrack bring-up guide](docs/OPTITRACK.md) and
[NatNet2ROS2 details](NatNet2ROS2/README.md).

Both adapters validate every received source report before applying a
configurable ROS 2 output-rate cap, reducing DDS traffic without changing the
selected report's source timestamp. Set `output_rate_hz:=0.0` to publish
every accepted report.

| Adapter | Default ROS 2 output cap | Downsampled output |
|---|---:|---|
| **NatNet2ROS2** | **200 Hz** | Receives the 300 Hz NatNet 4.5 competition source and publishes one filtered `/optitrack/poses` array containing only available `Ball`, `P1`, and `P2` entries; an empty array is the live-source/no-competition-body heartbeat; no raw marker cloud or duplicate TF output |
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

For the A3 clock chain and installation runbook, see
[agibot/README.md](agibot/README.md#clock-synchronization).
Internet/NTP loss does not block ordinary offline A3 applications, but the
robot is not qualified for external-mocap strike timing until both the adapter
and A3 clock-health gates pass. Clock drift or mixed domains can corrupt state
estimation, latency measurement, recording, calibration, and strike timing
even while ROS 2 topics appear healthy.

## Optional marker-CAD alignment: P1 to A3 `pelvis_link`

The current sticker profile uses the **17-station v3 sticker shell: seven unchanged
front markers and two rear rows of five (original IDs)**, with the supplied
[A3_v3_17_stickers.motive](agibot/mocap_sticker_shell_v3/motive_asset/A3_v3_17_stickers.motive)
asset, internally named **`UCB_P1`**. Its origin targets the nominal CAD `pelvis_link`
origin and ROS axes **X forward, Y left, Z up**. With Motive Streaming **Up Axis
= Z Up** and the existing MODELDEF axis conversion, the intended fixed
`UCB_P1 → pelvis_link` correction is zero translation and identity rotation.
This is an offline-checked candidate, not a verified live calibration: the five
restored rear points are CAD-nominal, not observed in the source export. Validate
Motive import/re-export, then generate a fresh installed-layout receipt.
For the current runtime, keep `UCB_P1` for table side P1 or rename it to `UCB_P2`
for table side P2, use the `stickers_v3` profile, and save the fresh
receipt to `calibration/ucb_robot_to_pelvis.json`. All earlier 12-point,
old 24-station and ten-marker receipts cannot be reused for this sticker profile.

Hardware, frame-table links and the setup procedure are maintained in
[agibot/README.md](agibot/README.md#v3-sticker-p1-to-pelvis-calibration).
Receipt storage and migration rules are in [calibration/README.md](calibration/README.md).
The shared mocap contract remains in [mocap/README.md](mocap/README.md).

### ChingMu / VRPN asset for the same shell

The [shell-v3 VRPN asset package](agibot/mocap_sticker_shell_v3/vrpn_asset/README.md)
provides one CMTracker import file,
[A3_v3_17_stickers.json](agibot/mocap_sticker_shell_v3/vrpn_asset/A3_v3_17_stickers.json),
using the same filename stem and internal **`UCB_P1`** body name as the Motive
asset. It is byte-identical to the existing **V4 P1** runtime profile; existing
runtime profiles remain unchanged for compatibility. The **v3 CAD folder and
V4 runtime selector refer to the same 17-sticker shell**. Configure the site
body name (`UCB_P1` or `UCB_P2`) and unique IDs in CMTracker as needed, without
changing the shared pelvis-local geometry. VRPN then transports poses, not
the asset definition.
The uncentered marker coordinates target nominal `pelvis_link`, with ROS
**X forward, Y left, Z up**; do not subtract the centroid or apply Motive's
Y-up conversion. Server world/table alignment is a separate commissioning
step. CMTracker import/re-export and live installation checks are still
required: pose-only VRPN has no ModelDef to verify geometry, and this asset
is not a physical-calibration receipt. The optional measured NatNet procedure
above is unchanged. See the [VRPN operator guide](docs/operations/vrpn_pants_profiles.md).

### ChingMu 18-marker ping-pong ball

The [18-sticker ball jig and CMTracker asset](mocap/README.md#chingmu-18-sticker-ping-pong-ball-template-and-asset)
use eighteen 6 mm stickers on a 40 mm ball. The asymmetric layout has optimized
edge clearance and audited partial-match ambiguity and viewing-direction
coverage. Use the matching jig and `Ball` asset together, and position all
eighteen stickers with the supplied jig.
The package includes a two-piece STL, native import JSON, preview and the
[printing/import guide](mocap/README.md#chingmu-18-sticker-ping-pong-ball-template-and-asset).

## Database

Open-source datasets for HOPE and humanoid robot table-tennis research:

1. [HOPE-2026-Beijing (ModelScope)](https://www.modelscope.cn/datasets/hitchopen/HOPE-2026-Beijing/files) — Beijing robot table-tennis motion-capture recordings in CSV and FBX formats.
2. [PPQ004 three-star ball dataset](calibration/ppq004_20261020/README.md) — three raw C3D recordings, complete trajectory CSVs and extracted NPZ data, with checksums and archived fit results.

For the recorded ball/table setup, the [calculated flight and bounce coefficients](hope_ws/src/hope_planner_cpp/config/ball_physics_ppq004_20261020.yaml)
are provided alongside the [Python fitting pipeline](calibration/ppq004_20261020/tools/refit_ppq_multibounce.py),
which uses SciPy and a C++ RK4 kernel. See the [reproduction instructions](calibration/ppq004_20261020/REPRODUCE.md)
for dependencies and commands.

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
Both specify a professional three-star 40+ plastic table-tennis ball for official
matches and the exhibition match (§2(8)).

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
| `hope_training/` | The Isaac Lab training extension (`whole_body_tracking/` with the `HitterPingPong` task and the train/eval/export scripts, including `scripts/prepare_a3_isaac_asset.py`), the complete validated forehand/backhand reference motions (`motions/preprocessed/`), the canonical A3 joint order (`config/joint_order_agibot_a3.yaml`), and the ball-physics fitting tools (`ball_physics_fit/`). |
| `NatNet2ROS2/` | Independent ROS 2 workspace for the OptiTrack/MotiveBody NatNet 4.5 multicast adapter, named-pose interfaces, acquisition-time mapping, and driver tests. Build and launch it separately from `hope_ws`. |
| `VRPN2ROS2/` | Independent ROS 2 workspace for the ChingMu/VRPN client, strict server-time/NTP validation, and raw per-tracker `PoseStamped` topics. |
| `hope_ws/` | ROS 2 workspace: `hope_planner_cpp` (the supported C++ packetizer/Planner runtime), `hope_bringup` (relays, world-frame publisher, calibration tools, time-sync configs, fake publishers), `hope_msgs` (wire messages), and `calibration_receipts/` (venue calibration evidence). The retired Python Planner source is excluded from colcon and retained only for offline comparison. Raw acquisition lives in the two sibling adapter workspaces. Bring-up guides: [BRINGUP_TUTORIAL](hope_ws/BRINGUP_TUTORIAL.md), [SMOKE_TEST](hope_ws/SMOKE_TEST.md), [SHADOW_MODE](hope_ws/SHADOW_MODE.md). |
| `a3_deploy/` | The native C++ deploy runner, gate/rehearsal script suite, parity harness, and deploy runbooks (`a3_deploy_example/`); the MuJoCo/AimRT simulation fork with the real ball plant (`A3_MuJoCo_Sim/`); and the optional user-supplied URDF override location (`URDF/`). |
| [foxglove/](foxglove/README.md) | Operator console, managed lifecycle, startup checks, runtime diagnostics and stopped clock calibration. Deployment assets live under `a3/`, `helpers/` and `laptop/`; `tests/` is regression coverage, not robot installation content. |
| [`apps/a3_mujoco_serve/`](apps/a3_mujoco_serve/README.md) | Self-contained deterministic serve contribution: official A3 MuJoCo model/racket contact, legal-serve physics search, DLS IK, all-joint CSV export, replay validation, and the PR #18 high-level A3 runtime. |
| [agibot/](agibot/README.md) | A3 source robot models, vendor deploy/simulation references, clock synchronization and mounting hardware: the [v3 marker shell](agibot/mocap_sticker_shell_v3/README.md), [right wrist paddle flange](agibot/right_wrist_paddle_flange/README.md) and serving gripper STL. See the folder README for the complete map, printing units and hardware-specific guidance. |
| `mocap/` | Motion-capture frame/topic contract ([mocap/README.md](mocap/README.md)) and the preserved mocap reference documents (EN/ZH). |

## System Architecture

```
       ┌──────────────────────────────┐   ┌──────────────────────────────┐
       │ MotiveBody 3.5.0.1 Beta 1    │   │ Chingmu CMTracker / MCServer │
       │ NatNet 4.5 multicast, 300 Hz │   │ VRPN server                  │
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
                    │  hope_ball_flight_packetizer │
                    │  + hope_planner_cpp (1–3)   │
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

**Training stack.** The WBC training code builds on open-source [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) (MIT license), from which the `hope_training/whole_body_tracking/` extension derives, plus [GMR](https://github.com/YanjieZe/GMR) (MIT license) for SMPL-X to robot retargeting and [GVHMR](https://github.com/zju3dv/GVHMR) for monocular video-to-SMPL-X extraction. This repository also publishes the complete validated forehand/backhand reference motions; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the per-asset documentation for applicable terms.

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

- **Training / export**: the pinned NVIDIA `grasping` Distrobox documented in
  [`docs/DISTROBOX_SETUP.md`](docs/DISTROBOX_SETUP.md); the checked working
  stack is Isaac Sim 5.1.0, Isaac Lab 0.54.2, PyTorch 2.7.0+cu128 and
  `rsl_rl` 3.1.2
- **MuJoCo evaluation / reference runner**: `mujoco`, `onnxruntime`, `numpy` (no GPU needed)
- **Planner workspace**: the Ubuntu 24.04/ROS 2 Jazzy `hope` Distrobox from the
  same setup guide (`rclpy`, `numpy`, `pyyaml`, C++ build dependencies)
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

## Runner and Xbox update

The current model17400 deployment is described in
[A3 deployment: model17400 + PPQ004](#a3-deployment-model17400--ppq004).
The runtime includes Xbox LT locomotion, A/B/X/Y mode requests and LB+RB software
stop. Bundle capabilities remain authoritative: model17400 is receive-only.
See the
[operator guide](docs/operations/runtime_xbox.md) and
[validation limits](docs/operations/runtime_public_validation.md). This checkout
includes the ready-to-install Foxglove `.foxe`, HumanLike locomotion bundle and
serve CSVs.
