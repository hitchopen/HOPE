# A3 Pingpong MuJoCo Sim

This repository provides an AimRT + MuJoCo simulation example for the A3 T2.5 pingpong robot. The current main example is `a3_pingpong`: a 31-DOF serial A3 model with the right-hand pingpong racket, body_drive-compatible joint control, iceoryx real-time channels, and ROS2 debug/reset interfaces.

The implementation is intentionally split into a real-time control path and a tooling/debug path:

```text
External controller
        |
        | iceoryx: /body_drive/*
        v
AimRT MujocoSimModule
        |
        | MuJoCo model, actuators, sensors, reset hooks
        v
A3 pingpong MJCF
        |
        | ROS2: /sim/a3/*, /tf
        v
debug tools, reset tools, future ball/table logic
```

## Current Example

Main files:

- `src/models/bin/cfg/a3_pingpong_iceoryx_cfg.yaml`
- `src/models/bin/cfg/model/a3_pingpong/a3_pingpong.xml`
- `src/models/bin/start_a3_pingpong_iceoryx.sh`
- `src/protocols/joint_msgs`
- `src/protocols/mujoco_sim_msgs`
- `src/module/mujoco_sim_module`

The A3 model uses:

- 31 motor actuators: waist 3, neck 2, arms 14, legs 12.
- Visual meshes copied into `src/models/bin/cfg/model/a3_pingpong/meshes`.
- Optimized convex collision meshes copied into `meshes/collision_optimized`.
- Ankle roll hull collision for foot contact.
- A `stand` keyframe for reset.
- Pelvis/torso IMU sensors, pelvis pose/twist sensors, and right racket pose sensors.

The original exported URDF source package is not required at runtime. Runtime assets live under `src/models/bin/cfg/model/a3_pingpong`.

## Build

Build requirements:

- Linux with a C++20 compiler, for example GCC 11+ or Clang 14+.
- CMake 3.24+.
- Python 3.
- Git and network access for CMake `FetchContent`, unless local sources are provided.
- OpenGL/X11 development libraries for MuJoCo/GLFW viewer support.
- ROS2 development environment when `AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=ON`.

Ubuntu package sketch:

```bash
sudo apt update
sudo apt install -y \
  build-essential git python3 \
  cmake pkg-config \
  libgl1-mesa-dev libx11-dev libxinerama-dev libxcursor-dev \
  libxi-dev libxrandr-dev libxxf86vm-dev
```

ROS2 packages used by the current A3 example:

```bash
sudo apt install -y \
  ros-${ROS_DISTRO}-ament-cmake \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-tf2-msgs \
  ros-${ROS_DISTRO}-rosidl-default-generators
```

Source ROS2 before building:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

If your distribution package provides CMake older than 3.24, install a newer CMake before running `build.sh`.

AimRT and MuJoCo are fetched by CMake:

- AimRT: `cmake/GetAimRT.cmake`, default tag `v1.6.0`.
- MuJoCo: `cmake/GetMujoco.cmake`, default version `3.1.6`.

For offline or pinned local builds, pass local source directories:

```bash
./build.sh \
  -Daimrt_LOCAL_SOURCE=/path/to/AimRT \
  -Dmujoco_LOCAL_SOURCE=/path/to/mujoco
```

```bash
./build.sh
```

Useful verification commands:

```bash
cmake --build build -j$(nproc)
cmake --build build --target aimrt_mujoco_sim_models_build_all
```

After build, runtime files are installed under:

```text
build/install/bin
```

## Run

From the install `bin` directory:

```bash
cd build/install/bin
./start_a3_pingpong_iceoryx.sh
```

The script starts `iox-roudi` if available, sources the generated ROS2 message setups, updates `LD_LIBRARY_PATH`, and starts:

```bash
./aimrt_main --cfg_file_path=./cfg/a3_pingpong_iceoryx_cfg.yaml
```

## Architecture

`MujocoSimModule` owns the simulation loop. Each simulation tick:

1. Applies pending subscriber commands to MuJoCo controls or state.
2. Steps the MuJoCo model.
3. Publishes configured sensor/state outputs according to each publisher frequency.

The A3 example config wires the module as follows:

- `body_drive_joint_actuator` subscribers consume joint commands.
- `body_drive_joint_sensor` publishers emit joint states.
- `imu_sensor_ros2` publishers emit pelvis/torso IMU data.
- `pose_sensor_ros2`, `twist_sensor_ros2`, and `odometry_ros2` publish ROS2 debug data and `/tf`.
- `sim_reset_ros2` subscribes reset/control requests from ROS2.

The body_drive state and IMU publishers are configured at 500 Hz and share the same simulation publish context so their timestamps and sequence progression are synchronized.

## External Interfaces

### body_drive Control Path

The body_drive path is the main controller interface and uses the AimRT iceoryx backend only.

Real-robot parity: the `/body_drive/*` interface is intentionally aligned with the interface used on the real robot. This includes topic names, message field definitions, joint group ordering, publish frequencies for state/IMU feedback, and AimRT iceoryx backend routing. A controller that already implements the real-robot body_drive contract should not need simulator-specific message or transport changes.

| Direction | Topic | Type | Frequency |
| --- | --- | --- | --- |
| subscribe | `/body_drive/waist_joint_command` | `joint_msgs/msg/JointCommand` | controller-driven |
| subscribe | `/body_drive/neck_joint_command` | `joint_msgs/msg/JointCommand` | controller-driven |
| subscribe | `/body_drive/arm_joint_command` | `joint_msgs/msg/JointCommand` | controller-driven |
| subscribe | `/body_drive/leg_joint_command` | `joint_msgs/msg/JointCommand` | controller-driven |
| publish | `/body_drive/waist_joint_state` | `joint_msgs/msg/JointState` | 500 Hz |
| publish | `/body_drive/neck_joint_state` | `joint_msgs/msg/JointState` | 500 Hz |
| publish | `/body_drive/arm_joint_state` | `joint_msgs/msg/JointState` | 500 Hz |
| publish | `/body_drive/leg_joint_state` | `joint_msgs/msg/JointState` | 500 Hz |
| publish | `/body_drive/pelvis_imu/data` | `sensor_msgs/msg/Imu` | 500 Hz |
| publish | `/body_drive/torso_imu/data` | `sensor_msgs/msg/Imu` | 500 Hz |

Joint group order in `a3_pingpong_iceoryx_cfg.yaml` is the contract. The current layout is:

- `waist`: `waist_yaw_joint`, `waist_roll_joint`, `waist_pitch_joint`
- `neck`: `head_yaw_joint`, `head_pitch_joint`
- `arm`: left arm 7 joints, then right arm 7 joints
- `leg`: left leg 6 joints, then right leg 6 joints

`joint_msgs/msg/JointCommand` contains a list of `Command`:

```text
string name
uint32 sequence
float64 position
float64 velocity
float64 effort
float64 stiffness
float64 damping
```

The actuator computes torque as:

```text
ctrl = effort + stiffness * (position - q) + damping * (velocity - dq)
```

The result is written to the MuJoCo actuator control for the matching joint.

### ROS2 Debug and Reset Path

ROS2 is for observation, reset, and future external tools. It is not the main body_drive control path.

| Direction | Topic | Type | Frequency |
| --- | --- | --- | --- |
| publish | `/sim/a3/pelvis_pose` | `geometry_msgs/msg/PoseStamped` | 100 Hz |
| publish | `/sim/a3/pelvis_twist` | `geometry_msgs/msg/TwistStamped` | 100 Hz |
| publish | `/sim/a3/right_racket_pose` | `geometry_msgs/msg/PoseStamped` | 100 Hz |
| publish | `/tf` | `tf2_msgs/msg/TFMessage` | 100 Hz |
| subscribe | `/sim/a3/reset` | `mujoco_sim_msgs/msg/SimReset` | event-driven |

`/sim/a3/reset` supports:

```text
uint8 MODE_ABSOLUTE=0
uint8 MODE_KEYFRAME=1
uint8 mode
int32 keyframe_id

bool set_base
geometry_msgs/Pose pelvis_pose

bool set_base_twist
geometry_msgs/Twist pelvis_twist

bool set_joints
sensor_msgs/JointState joint_state

bool zero_all_velocities
bool clear_ctrl
```

Use `MODE_KEYFRAME` with `keyframe_id=0` to reset to the MJCF `stand` keyframe. Use `MODE_ABSOLUTE` with the `set_*` flags to override base pose, base twist, or selected joints.

## Model Notes

The A3 MJCF intentionally keeps the model simple and explicit:

- The ordinary T2.5 serial kinematic tree is used. There is no A3 loop solver in this example.
- Joint damping, friction loss, armature, actuator force limits, and keyframe values are aligned with the reference A3 assets where applicable.
- Collision uses optimized convex body collision meshes plus ankle hulls. Visual meshes do not participate in collision.
- Adjacent body collision excludes are configured to avoid false self-collision at shoulders, wrists, pelvis, hips, knees, and ankles.
- The right pingpong racket is kept as visual geometry plus a dedicated low-poly collision mesh for the racket face. The face collision mesh is generated from the red/black visual face outline and keeps flat front/back contact surfaces with a small physical thickness.
- The right racket handle and grasping hand use small primitive collision geoms instead of a large hand convex hull, so the hand collision does not block or overlap the racket face.
- The `right_racket` site is an invisible pose sensor anchor for `/sim/a3/right_racket_pose`; it is not a physical pingpong ball.

## Extending the Example

### Add Ball, Table, or Game Logic

For scene objects, add independent MuJoCo bodies/geoms rather than embedding them into the robot body tree.

Recommended shape:

- `table`: static body or mocap body with its own visual/collision geoms.
- `ball`: free body with sphere geom, mass/inertia, contact parameters, and a stable site such as `pingpong_ball_site`.
- `ball` sensors: add `framepos`, `framequat`, `framelinvel`, and `frameangvel` sensors bound to the ball site.
- ROS2 observation topics: publish ball/table state through ROS2 so downstream tools can consume data as if it came from perception or motion capture.
- Reset topics: extend `/sim/a3/reset` or add a scene reset message for ball/table placement and velocity reset.
- Keep `/body_drive/*` unchanged so external controllers remain compatible.

A typical ball observation interface should look like a real motion-capture source:

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| publish | `/sim/a3/ball_pose` | `geometry_msgs/msg/PoseStamped` | simulator-native ball pose |
| publish | `/sim/a3/ball_twist` | `geometry_msgs/msg/TwistStamped` | simulator-native ball velocity |
| publish | `/mocap/pingpong_ball/pose` | `geometry_msgs/msg/PoseStamped` | mocap-compatible alias for policy/perception consumers |
| publish | `/sim/a3/table_pose` | `geometry_msgs/msg/PoseStamped` | optional if the table is movable |

If the real system only provides ball position, use one of these approaches:

- Publish `PoseStamped` with identity orientation so the simulated and real interfaces stay type-compatible.
- Add a small `PointStamped` publisher if consumers should explicitly treat the source as position-only.
- Add a custom message later only when confidence, marker id, occlusion state, or multi-ball tracking is required.

Implementation steps for ball pose publishing:

1. Add a free ball body, sphere geom, and `pingpong_ball_site` in `a3_pingpong.xml`.
2. Add MuJoCo sensors:
   - `framepos name="ball_framepos" objtype="site" objname="pingpong_ball_site"`
   - `framequat name="ball_framequat" objtype="site" objname="pingpong_ball_site"`
   - `framelinvel name="ball_framelinvel" objtype="site" objname="pingpong_ball_site"`
   - `frameangvel name="ball_frameangvel" objtype="site" objname="pingpong_ball_site"`
3. Add ROS2 channel QoS entries in `a3_pingpong_iceoryx_cfg.yaml` for the new topics.
4. Reuse `pose_sensor_ros2` and `twist_sensor_ros2` publisher entries if `PoseStamped`/`TwistStamped` are enough.
5. Add a scene reset subscriber only if ball/table reset semantics become different from robot reset semantics.

Table objects usually do not need high-rate publishing if they are static. Publish table pose once at startup or at low rate if a downstream consumer expects it.

## Repository Layout

```text
cmake/                              AimRT dependency setup
src/module/mujoco_sim_module/       MuJoCo simulation module
src/protocols/joint_msgs/           body_drive joint command/state messages
src/protocols/mujoco_sim_msgs/      simulation reset/control messages
src/models/bin/cfg/                 runtime AimRT configs
src/models/bin/cfg/model/           MJCF models and meshes
src/models/bin/start_*.sh           installed launch scripts
```

## Development Checklist

For model or interface changes, run:

```bash
cmake --build build -j$(nproc)
cmake --build build --target aimrt_mujoco_sim_models_build_all
git diff --check
```

For model validity, load the installed MJCF:

```bash
python3 - <<'PY'
from pathlib import Path
import mujoco

xml = Path("build/install/bin/cfg/model/a3_pingpong/a3_pingpong.xml")
model = mujoco.MjModel.from_xml_path(str(xml))
print("nq", model.nq, "nv", model.nv, "nu", model.nu)
PY
```
