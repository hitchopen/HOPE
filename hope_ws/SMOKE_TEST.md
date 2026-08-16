# C++ Planner dry-run smoke test

This smoke starts only synthetic ball transport, the C++ flight packetizer and
the C++ Planner. It does not start the native Runner, HAL or robot control.

Run on an Ubuntu 24.04 host with ROS 2 Jazzy, or inside the `hope` distrobox
created by [`docs/DISTROBOX_SETUP.md`](../docs/DISTROBOX_SETUP.md):

```bash
cd "$HOME/workspace/HOPE_OPEN"
source /opt/ros/jazzy/setup.bash

cd hope_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install \
  --packages-select hope_msgs hope_bringup hope_planner_cpp \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash

test -x install/hope_planner_cpp/lib/hope_planner_cpp/hope_ball_flight_packetizer
test -x install/hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true
```

In a second terminal, source the same ROS and workspace overlays:

```bash
cd "$HOME/workspace/HOPE_OPEN/hope_ws"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

ros2 node list | grep -E '/hope_ball_flight_packetizer|/hope_planner'
ros2 topic info /ball/flight_packet
ros2 topic info /racket/command_flat
ros2 topic echo /planner/diagnostics --once
```

Stop the first terminal with `Ctrl-C`. The supported runtime must contain
`hope_ball_flight_packetizer` and `hope_planner_cpp_node`; it must not contain
`hope_planner_node`. The retired Python package is excluded from colcon by
`src/hope_planner/COLCON_IGNORE`.

For the full model_21800 MuJoCo closed loop, continue with
[`docs/MODEL_21800.md`](../docs/MODEL_21800.md). For real hardware, use
[`docs/operations/foxglove_first_hardware_test.md`](../docs/operations/foxglove_first_hardware_test.md).
