# Real-robot SHADOW mode tutorial (no hardware control)

> **Historical example.** This walkthrough was written for the earlier 180-D
> (`full` contract) policy generation (`model_15200`). The shadow-mode technique —
> run perception→policy against live robot state, publish nothing — applies unchanged
> to the current 110-D `hitter_pure` line; swap in your exported `policy.onnx` and the
> 110-D observation contract ([docs/POLICY_INTERFACE.md](../docs/POLICY_INTERFACE.md)).

Shadow mode = run the full perception→policy path **against the real A3's live state**, but
**publish nothing** and **move nothing**. It subscribes the robot's joint state + IMU and the
fake planner's `/racket/command`, builds the exact 180-D observation, runs `model_15200`
deterministically, and logs obs/action/`target_q` to CSV for inspection.

**Hardware safety (triple-locked):**
- `mode:=shadow` → the node **never creates a joint-command publisher**.
- Publishing requires `mode==hardware AND hardware_enable AND !estop` — shadow fails the first
  condition, so it can never publish.
- We never set `mode:=hardware` here.

## Where things run — dev machine, HDU, MDU (when you need SSH)

There are **three machines** (see the deploy guide (`docs/RUN_ON_AGIBOT.md`), the
authoritative robot-side guide):

| machine | what runs there | how you reach it |
|---|---|---|
| **dev machine** (this PC, in `distrobox enter hope`) | `planner_imitate`, `wbc_runner`, `ros2` CLI | local terminal |
| **HDU** — Wi-Fi jump host | nothing you start; just the SSH hop | `ssh agi@<HDU_WIFI_IP>` |
| **MDU** — robot compute unit | the robot bridge (`hal_ethercat`) that produces joint/IMU state | `ssh -J agi@<HDU_WIFI_IP> agi@<MDU_IP>` (default `<MDU_IP>=10.42.10.12`) |

**You need SSH only to reach the robot (MDU) to start the state bridge.** Everything in *this*
tutorial that you type yourself (`planner_imitate`, `wbc_runner`, `ros2 topic …`) runs on the **dev
machine**, locally — no SSH. SSH (jump through HDU to MDU) is for **Step 1 only**: starting
`hal_ethercat` on the robot so it publishes joint/IMU state. Fill in `<HDU_WIFI_IP>` (on-site Wi-Fi)
and `<MDU_IP>` at the arena.

> ⚠️ **Big prerequisite — is the robot state on DDS or iceoryx?** `wbc_runner` is a ROS 2 (DDS)
> node, so it can only subscribe state that is on the **DDS graph**. On the **default Rockchip/MDU**,
> the six state channels (`/body_drive/*` + the two IMUs) go over **iceoryx (shared memory, MDU-local)
> — NOT DDS**. A rclpy node (even running on the MDU) **cannot see them**, and they never cross the
> network to the dev machine. So **shadow mode against the default Rockchip robot does not work
> out-of-the-box.** Two ways to get the state onto DDS first:
> 1. **Thor/ADU** compute unit — it uses **ros2 transport**, so the state IS on DDS (then this
>    tutorial works; run `wbc_runner` where DDS discovery reaches the ADU).
> 2. **iceoryx→ROS 2 bridge** on the MDU (AimRT's `ros2_plugin` can republish the iceoryx topics as
>    DDS, like the a3_pingpong sim does for `/sim/a3/...`). Until such a bridge republishes joint
>    state + IMU as DDS topics, `ros2 topic list` won't show them and shadow has nothing to subscribe.
>
> **Practical reading:** if `ros2 topic list` (Step 2) does **not** show a joint-state/IMU topic, you
> are on the iceoryx path and shadow-against-real is blocked until a bridge exists — that bridge is a
> separate piece of work (related to, but not the same as, the MuJoCo shadow bridge).

### Discovered on THIS robot (on-site MDU scan) — current status

A live scan on the MDU (`ros2 node/topic list --no-daemon`) found:

- **Domain:** `ROS_DOMAIN_ID=232` (domain 0 is empty). **You must `export ROS_DOMAIN_ID=232`** or you
  see nothing. Discovery is open across the subnet (`ROS_LOCALHOST_ONLY=0`,
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, FastDDS profile in `/opt/agibot/entry/cfg/...xml`).
- **IMUs ARE on DDS** ✅ (AimRT's ros2 plugin already bridges them):
  - `/ros2/body_drive/pelvis_imu/data`  ← the pelvis/base IMU (use this for the obs)
  - `/ros2/body_drive/torso_imu/data`
  (the `/ros2/...` prefix = real ROS 2 messages; the `/aima/... pb_...` topics are AimRT protobuf
  channels, NOT usable as standard msgs.)
- **Joint state is NOT on DDS** ❌ — there is no `/ros2/body_drive/*_joint_state`. Joint feedback is
  still iceoryx-only. **This is the one remaining blocker for full shadow.**

**So on THIS robot:** the IMU half of shadow works over DDS today; the joint half does not. The fix is
config, not code — **add the joint-state channels to the same AimRT `ros2_plugin` that already bridges
the IMUs** (so `/ros2/body_drive/arm_joint_state` etc. appear), then point `joint_state_topic:=` at it.
Until then, run shadow with IMU live + joints at default (partial: gravity/gyro real, joint terms not).

## What shadow mode reads (and the launch args)

| arg | what | default |
|---|---|---|
| `mode:=shadow` | predict on real state, never publish | (you set it) |
| `state_source:=ros` | subscribe real topics (vs `synthetic`) | `synthetic` |
| `onnx_path:=…` | the 2-clip `model_15200` policy.onnx (~1.24 MB) | (required) |
| `joint_state_topic:=…` | robot joint feedback | `/a3/joint_states` |
| `joint_state_type:=…` | `sensor_msgs` (std) or `joint_msgs` (a3 sim) | `sensor_msgs` |
| `imu_topic:=…` | `sensor_msgs/Imu` | `/a3/imu` |
| `base_pose_topic:=…` | optional `geometry_msgs/PoseStamped` pelvis world pose | `""` (→ nominal) |
| `csv_path:=…` | log file | `""` |

The defaults are **placeholders** — your real bridge's topic names will differ. Steps 2–3 below
show how to discover them. `joint_msgs` is only needed if `joint_state_type:=joint_msgs`; the
default `sensor_msgs` path needs nothing extra.

---

## Tutorial (8 steps)

**Step 1 runs ON THE ROBOT (MDU, via SSH).** Steps 2–8 run wherever DDS can discover the robot's
domain-232 topics — either on the **dev machine** (if it is on the **same subnet** as the MDU; the HDU
Wi-Fi hop is for SSH, not necessarily for DDS multicast — confirm with `ros2 topic list` in Step 2) or
**on the MDU itself** (topics are local there). Prologue for every terminal:
```bash
distrobox enter hope -- bash               # dev machine, local (NOT ssh)
export HOPE_REPO=/path/to/your/clone        # root of this repository checkout
source /opt/ros/jazzy/setup.bash
source $HOPE_REPO/hope_ws/install/local_setup.bash
export ROS_DOMAIN_ID=232                    # THIS robot's domain (REQUIRED; domain 0 is empty)
export ROS_LOCALHOST_ONLY=0                 # allow dev-machine <-> MDU cross-host discovery
export ONNX=$HOPE_REPO/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope/2026-06-28_23-01-24_phase050_perclippos_scratch/exported/policy.onnx
```
> If `ros2 topic list` on the dev machine shows nothing from the robot, your dev machine isn't on the
> MDU's DDS subnet — run `wbc_runner` **on the MDU** instead (needs `onnxruntime` + this package there),
> or set up a DDS route. RMW must match the robot (FastDDS / `rmw_fastrtps_cpp`).

### 1. Start the A3 state bridge — ON THE ROBOT, via SSH (HDU → MDU)
This is the **only** SSH step. Follow the deploy guide (`docs/RUN_ON_AGIBOT.md`) (steps
3–6) for the full, authoritative procedure. The minimum to get state flowing:
```bash
# from the dev machine: jump through HDU to the robot's MDU
ssh -J agi@<HDU_WIFI_IP> agi@<MDU_IP>          # default <MDU_IP>=10.42.10.12

# on the MDU: stop the system service and start EtherCAT (this publishes joint/IMU state)
sudo systemctl stop agibot_pm
source /agibot/software/v0/entry/env/env.sh
cd /agibot/software/v0
bash scripts/hal_ethercat/start_hal_ethercat.sh   # KEEP this terminal open (it streams joint/IMU)
```
- *Safety:* starting `hal_ethercat` does **not** move the robot — it only reads state and waits for
  commands. We never send commands in shadow mode. Still: robot on a stand/hoist, e-stop in hand.
- *Reality check (Rockchip/MDU):* these channels are **iceoryx**, so `ros2 topic hz /body_drive/...`
  on the MDU shows nothing — that's normal (see the prerequisite box above). To use shadow mode you
  need them on **DDS** (Thor/ADU, or an iceoryx→ROS 2 bridge). If you can't see a ROS 2 joint-state
  topic in Step 2, stop here — shadow-against-real is blocked until that bridge exists.

### 2. List the topics and find joint-state / IMU / base-pose
```bash
export ROS_DOMAIN_ID=232         # THIS robot's domain (domain 0 is empty) — set in EVERY terminal
export ROS_LOCALHOST_ONLY=0      # allow cross-host discovery (dev machine <-> MDU subnet)
ros2 daemon stop                 # avoid a stale-daemon xmlrpc error
ros2 topic list --no-daemon | grep -iE "body_drive|joint|imu|pelvis|torso"
```
On THIS robot you should see (confirmed on-site):
- **IMU** ✅ `/ros2/body_drive/pelvis_imu/data` (pelvis/base — use this) and `/ros2/body_drive/torso_imu/data`
- **joint state** ❌ — NOT present on DDS yet (iceoryx-only). Until it's bridged, full shadow is blocked
  on joints (see the "Discovered on THIS robot" box). Generic names to look for if your config differs:
  `/ros2/body_drive/arm_joint_state`, `/joint_states`.
- **base/pelvis pose** (optional) — not seen on this scan.

Verify the IMU is a real ROS message (not a protobuf channel):
```bash
ros2 topic info -v /ros2/body_drive/pelvis_imu/data    # expect Type: sensor_msgs/msg/Imu
```

> **DECISION POINT.** If `ros2 topic list` shows **no** joint-state/IMU topic, the robot state is on
> **iceoryx, not DDS** (the default Rockchip/MDU case) → `wbc_runner` cannot subscribe it, and
> shadow-against-real is **blocked** until an iceoryx→ROS 2 bridge republishes those channels (or you
> are on a Thor/ADU ros2-transport unit). This is expected per the bring-up checklist — it is not a
> bug in `wbc_runner`. If you DO see the topics (Thor/ADU or bridged), continue.

Check each one's **message type** (this decides `joint_state_type`):
```bash
ros2 topic info -v /a3/joint_states     # <- use YOUR joint-state topic name
ros2 topic info -v /a3/imu
```
- *Joint type `sensor_msgs/msg/JointState`* → `joint_state_type:=sensor_msgs` (default).
- *Joint type `joint_msgs/msg/JointState`* → `joint_state_type:=joint_msgs` (and that package must be
  built/sourced).
- *IMU should be `sensor_msgs/msg/Imu`.*

### 3. Echo joint state + IMU once (read the names + frame)
```bash
# IMU is on DDS on this robot (add --qos-reliability best_effort; sensor data is usually best-effort):
ros2 topic echo --once --qos-reliability best_effort /ros2/body_drive/pelvis_imu/data
# joint state: ONLY if a /ros2/.../*_joint_state exists (else it's still iceoryx -> not here yet):
# ros2 topic echo --once --qos-reliability best_effort /ros2/body_drive/arm_joint_state
```
- **Why it matters:** the IMU's `angular_velocity` must be the **pelvis body-frame gyro** and
  `orientation` the pelvis world orientation (the runner's `[obs verify]` confirms projected_gravity
  ≈ [0,0,-1] upright). For joints, the message `name:` list must match the ONNX `joint_names` — the
  runner reports the match count (step 5). On THIS robot the joint-state topic isn't on DDS yet, so
  you'll run step 5 with IMU live + joints at default until the joint channels are bridged.
- *Error: `does not appear to be published`* → wrong topic / wrong `ROS_DOMAIN_ID` / add
  `--qos-reliability best_effort` / your host isn't on the MDU's DDS subnet.

### 4. Start the fake planner (Terminal A) — level 0 or 1
```bash
ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=1
```
- `level:=0` = stand only (safest); `level:=1` = slow forehand. Publishes `/racket/command`.
- Reminder: `dry_run:=false` only **publishes the target topic**; it is not a hardware command.

### 5. Start wbc_runner in SHADOW mode (Terminal B)
Use the topics you confirmed in step 2 (this robot's real IMU shown; joint topic is a placeholder
until the joint channels are bridged to DDS):
```bash
ros2 launch hope_wbc_runner wbc_runner.launch.py \
  mode:=shadow state_source:=ros onnx_path:="$ONNX" \
  imu_topic:=/ros2/body_drive/pelvis_imu/data \
  joint_state_topic:=/ros2/body_drive/arm_joint_state \
  joint_state_type:=sensor_msgs \
  base_pose_topic:= \
  csv_path:=/tmp/wbc_runner_shadow.csv
```
- *Success log lines to look for:*
  - `wbc_runner started | mode=shadow | state_source=ros ...`
  - `WILL NOT PUBLISH (log only)`
  - `state_source=ros: joints '/a3/joint_states' (sensor_msgs/JointState), imu '/a3/imu'.`
  - **`[joint order] matched 31/31 policy joints by name`** ← the joint-order check. If it says
    e.g. `matched 27/31` it lists the missing joints — fix the bridge naming (or names won't map).
  - **`[obs verify] first real-state observation built`** with:
    - `projected_gravity (body) = [~0, ~0, ~-1]` when the robot stands upright (else the IMU
      orientation frame is wrong),
    - `base_ang_vel (body gyro) = [~0, ~0, ~0]` when standing still,
    - `base_pos_w = ... (NOMINAL fallback)` since you left `base_pose_topic` empty.
- *Error `No module named 'joint_msgs'`* → you set `joint_state_type:=joint_msgs` but that package
  isn't built; either build it or use `sensor_msgs`.

### 6. Verify the CSV (now includes the obs-verify columns)
The CSV logs the verification values **every row** so you can inspect/plot them over the run:
`proj_grav_x/y/z`, `base_ang_vel_x/y/z`, `joint_order_matched_count`, `missing_joint_count`,
`base_pose_source` (`real` | `nominal` | `synthetic`).
```bash
# column index map (so you can pick fields by number):
head -1 /tmp/wbc_runner_shadow.csv | tr ',' '\n' | nl | sed -n '1,21p'
# pull just the verify columns from the last few rows:
python3 - << 'EOF'
import csv
rows=list(csv.DictReader(open('/tmp/wbc_runner_shadow.csv')))
cols=['t','swing_type','published','proj_grav_x','proj_grav_y','proj_grav_z',
      'base_ang_vel_x','base_ang_vel_y','base_ang_vel_z',
      'joint_order_matched_count','missing_joint_count','base_pose_source']
print(' | '.join(cols))
for r in rows[-5:]:
    print(' | '.join(str(r[c]) for c in cols))
EOF
echo "--- active swing rows (forehand) ---"
awk -F, 'NR>1 && $4=="forehand" && $9=="1"' /tmp/wbc_runner_shadow.csv | wc -l
```
- *Success:*
  - `published` column is **always 0** (shadow never publishes).
  - `proj_grav_z ≈ -1.0`, `proj_grav_x/y ≈ 0` while standing upright (IMU frame OK).
  - `base_ang_vel_* ≈ 0` while standing still (and changes if you gently rotate the base).
  - `joint_order_matched_count = 31`, `missing_joint_count = 0` (joint names map correctly).
  - `base_pose_source = nominal` (you left `base_pose_topic` empty) or `real` (if you set it).
  - active forehand rows have non-zero `obs_norm`/`action_norm` (model ran on **real** state).
- *Note:* before the first real IMU/joint message arrives, these columns read `nan` / `-1` /
  `synthetic` — that's expected for the first moment; they fill in once the bridge data flows.

### 7. Verify there is NO joint-command publisher
```bash
ros2 node info /wbc_runner
```
- *Success:* under **Publishers:** you see only `/wbc_runner/diagnostics` (+ `/rosout`,
  `/parameter_events`). There is **no** `/a3/joint_command` (or any joint command) publisher.
- Cross-check the whole graph: `ros2 topic list | grep -iE 'joint_command|joint_cmd|body_drive'` →
  nothing owned by wbc_runner.

### 8. Verify estop gating
```bash
ros2 topic pub -1 /hope/estop std_msgs/Bool "{data: true}"
```
- *Success:* wbc_runner logs `ESTOP engaged -> publishing disabled, commanding STAND.`; the CSV
  switches to `stand` rows with `valid=0` (and `published` stays 0). Release with `{data: false}`.
- This proves the estop path. Combined with step 7, the publish gate
  (`mode==hardware AND hardware_enable AND !estop`) is verified safe: shadow can never publish, and
  estop is a hard block even in hardware mode (which we are NOT using).

Stop with `Ctrl-C` in Terminals A and B.

---

## Verification checklist (what "good" looks like)

Each item is in both the console log AND the CSV (column in `code`), so you can verify live and
re-check the collected data later:

- [ ] **Joint order** — `[joint order] matched 31/31`; CSV `joint_order_matched_count=31`, `missing_joint_count=0`.
- [ ] **IMU frame / projected gravity** — `projected_gravity ≈ [0,0,-1]` upright; CSV `proj_grav_x/y/z`.
- [ ] **Base angular velocity** — `≈ [0,0,0]` standing still; CSV `base_ang_vel_x/y/z` (moves when you rotate the base).
- [ ] **Base pose source** — CSV `base_pose_source` = `nominal` (no pose topic) or `real` (pose topic wired).
- [ ] **No output** — CSV `published` always `0`; `ros2 node info /wbc_runner` shows only `/wbc_runner/diagnostics`.
- [ ] **Estop** — `/hope/estop true` → STAND + `published=0`.
- [ ] **hardware_enable** — irrelevant in shadow (never publishes); only matters in hardware mode.

## Known limitations

**1. Transport (the big one): real Rockchip/MDU state is iceoryx, not DDS.** `wbc_runner` is a ROS 2
(DDS) node and can only shadow state that is on the DDS graph. The default robot publishes joint/IMU
state over iceoryx → not visible to `ros2`/rclpy → shadow-against-real needs a Thor/ADU (ros2
transport) unit or an iceoryx→ROS 2 bridge first. See the prerequisite box and Step 2's decision
point. (This bridge is separate from the deferred MuJoCo shadow bridge.)

**2. Base pose / localisation.** Without `base_pose_topic`, the absolute pelvis WORLD pose is a
nominal upright guess, so the
**anchor-position** obs term is approximate (joint, IMU, gravity, gyro, racket-target terms are
exact). If your bridge publishes a pelvis pose, pass it:
`base_pose_topic:=/a3/pelvis_pose` — the runner then uses it and `[obs verify]` shows
`base_pos_w = ... (real base_pose_topic)`.

## Troubleshooting

- **No joint-state/IMU topic in `ros2 topic list`** → robot state is on iceoryx (default Rockchip/MDU),
  not DDS. Shadow-against-real is blocked until an iceoryx→ROS 2 bridge republishes it (or use Thor/ADU).
  Normal per the deploy guide (`docs/RUN_ON_AGIBOT.md`), not a `wbc_runner` bug.
- `ssh: Could not resolve hostname` / can't reach MDU → check `<HDU_WIFI_IP>` and `<MDU_IP>`; you must
  jump through HDU: `ssh -J agi@<HDU_WIFI_IP> agi@<MDU_IP>` (default MDU `10.42.10.12`).
- `xmlrpc ... unknown tag 'rclpy.endpoint_info.TopicEndpointInfo'` → stale daemon: `ros2 daemon stop`.
- `echo` shows nothing (topic exists) → add `--qos-reliability best_effort`.
- `[joint order] matched <31` → the bridge's joint names don't match the ONNX `joint_names`; remap
  or rename. (Until fixed, unmatched joints stay at the default angle and the obs is wrong.)
- `projected_gravity` not ≈ `[0,0,-1]` upright → the IMU orientation is in a different frame than
  training expects; check the IMU driver's frame convention before trusting shadow numbers.
