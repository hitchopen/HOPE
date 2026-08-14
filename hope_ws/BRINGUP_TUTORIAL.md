# HOPE C++ Planner bring-up

The supported Planner is `hope_planner_cpp`. The old Python `hope_planner`
package is an offline reference only and is excluded from colcon.

## Choose the procedure

| Goal | Procedure |
| --- | --- |
| Verify ROS transport and Planner without a Runner | [`SMOKE_TEST.md`](SMOKE_TEST.md) |
| Run model_21800 with a MuJoCo window | [`docs/MODEL_21800.md`](../docs/MODEL_21800.md), steps 0–7 |
| Start the three-machine Foxglove hardware stack | [`docs/operations/foxglove_first_hardware_test.md`](../docs/operations/foxglove_first_hardware_test.md) |

## Runtime topology

```text
VRPN2ROS2 or NatNet2ROS2
  -> hope_bringup relay -> /poses
  -> hope_ball_flight_packetizer -> /ball/flight_packet
  -> hope_planner_cpp_node -> /racket/command_flat
  -> model_21800 native C++ Runner
```

`hope_bringup.launch.py` owns exactly one packetizer and one C++ Planner. Its
three local forms are:

```bash
# Synthetic ball; no mocap hardware.
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true

# Independently launched VRPN2ROS2 input.
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=vrpn \
  ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0

# Independently launched NatNet2ROS2 input.
ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack
```

Build and source `hope_msgs`, `hope_bringup` and `hope_planner_cpp` first, as
shown in [`SMOKE_TEST.md`](SMOKE_TEST.md). The packetizer window is time-based:
`flight_window_s:=0.18` is the default. Do not use the retired
`planner_fit_window` sample-count argument.

This tutorial never starts HAL or a robot Runner. The real-hardware lifecycle,
calibration, e-stop and log collection remain in the Foxglove hardware guide.
