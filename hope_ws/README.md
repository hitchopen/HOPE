# HOPE ROS 2 workspace

`hope_ws` contains the HOPE planners, the HOPE message contract, and
source-specific relays that convert external mocap data into the canonical
`/poses` interface. It builds independently from both raw motion-capture
drivers.

```bash
cd hope_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## What's inside

| Member | Role |
|--------|------|
| `src/hope_planner` | Python planner (reference implementation) with venue presets (`config/hope_planner.yaml`, `.hitter_pure`, `.rally_v17_r10`, `.sim`) and the `planner_imitate` fake-planner for mocap-less bring-up. |
| `src/hope_planner_cpp` | C++ planner — the low-latency hardware line (`config/model21800_hardware.yaml`, `/planner/diagnostics`). Publishes the same wire contract as the Python planner. |
| `src/hope_msgs` | `RacketCommand` (position, velocity, normal, strike timing, outgoing ball velocity, validity/feasibility flags) — the field-by-field contract is [`docs/PLANNER_INTERFACE.md`](../docs/PLANNER_INTERFACE.md). |
| `src/hope_bringup` | Launch files, mocap relays (`optitrack_mct_relay`, `pose_to_posearray`), the world-frame contract `config/hope_world_frame.yaml` (`table_p1_to_p2_v1`), preflight/probe scripts, and the P1 calibration tools (`p1_pelvis_calibrator`, `p1_marker_cad_calibrator`, `p1_pelvis_tf_publisher`). |
| `calibration_receipts/` | Fail-closed calibration receipts (world origin, P1 marker-CAD registrations) that `hope_world_frame.yaml` pins by SHA — see [`docs/interfaces/frames.md`](../docs/interfaces/frames.md). |

Bring-up documentation lives next to this file:
[`BRINGUP_TUTORIAL.md`](BRINGUP_TUTORIAL.md) (dry-run walkthrough),
[`SMOKE_TEST.md`](SMOKE_TEST.md) (no-hardware build/launch check), and
[`SHADOW_MODE.md`](SHADOW_MODE.md) (run against the real robot's state,
publish nothing).

## OptiTrack input

The raw OptiTrack adapter lives in the sibling `NatNet2ROS2` workspace and is
built/launched separately. `hope_ws` never connects to Motive directly; its
`optitrack_mct_relay` subscribes to
`/optitrack/poses` (`motion_capture_tracking_interfaces/NamedPoseArray`) and
publishes the HOPE-standard `/poses` and related topics. The raw adapter sends
an empty-array heartbeat when NatNet remains live but none of the exact-name
`Ball`/`P1`/`P2` assets is present; the relay uses those callbacks for a
throttled tracking-loss warning but emits no placeholder pose or TF.

The host running that relay needs local message type support. Source the
adapter workspace before the HOPE overlay:

```bash
source NatNet2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack
```

See [`../NatNet2ROS2/README.md`](../NatNet2ROS2/README.md) and
[`../docs/OPTITRACK.md`](../docs/OPTITRACK.md) for the complete two-workspace
bringup.

## VRPN input

The ChingMu/VRPN client lives in the sibling `VRPN2ROS2` workspace. Start it
independently, then launch only the HOPE aggregator and planner:

```bash
# Terminal 1
source VRPN2ROS2/install/setup.bash
ros2 launch vrpn_mocap client.launch.yaml \
  server:=<CHINGMU_SERVER_IP> port:=3883

# Terminal 2
source VRPN2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=vrpn \
  ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0
```

`pose_to_posearray` preserves the incoming `PoseStamped` header. See
[`../VRPN2ROS2/README.md`](../VRPN2ROS2/README.md) for the strict source-time
and NTP epoch requirements. VRPN2ROS2 validates every source report and caps
each raw ROS output topic/sensor at 200 Hz by default; this bringup selects a
21-sample planner fit window for an approximately 100 ms estimator horizon.
