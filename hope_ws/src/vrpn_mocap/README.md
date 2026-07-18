# vrpn_mocap (HOPE)

VRPN client for ROS 2 that publishes ChingMu/VRPN motion-capture poses. This
package originates from the ChingMu VRPN ROS 2 plugin and is vendored into
`hope_ws` for the HOPE motion-capture pipeline.

Tested target: Linux + ROS 2 Jazzy.

## Install

Install the distro package, then let `rosdep` resolve the remaining
dependencies during the workspace build:

```
sudo apt install ros-jazzy-vrpn-mocap
```

## Build (inside hope_ws)

From the `hope_ws` workspace root:

```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```
ros2 launch vrpn_mocap client.launch.yaml server:=<MOCAP_SERVER_IP> port:=3883
```

`client.launch.yaml` arguments:

- `server` -- VRPN server IP/hostname (default: `localhost`)
- `port` -- VRPN server port (default: `3883`)

`config/client.yaml` parameters:

- `update_freq` (double) -- frequency of the motion-capture data publisher (default: `100.`)
- `refresh_freq` (double) -- frequency of dynamically adding newly tracked objects (default: `1.`)
- `sensor_data_qos` (bool) -- use best-effort QoS for the VRPN data stream; set to `false` for the reliable system-default QoS (default: `true`)
- `multi_sensor` (bool) -- set to `true` if more than one sensor (frame) reports on the same object (default: `false`)

## Inspect the data stream

```
ros2 topic list
ros2 topic echo /vrpn_mocap/<tracker>/pose_id_<N> --once
```

Raw topics live under the `/vrpn_mocap` namespace: `pose` per tracker with
`multi_sensor: false`, or `pose_id_<N>` per sensor with `multi_sensor: true`
(the bundled `client.launch.yaml` forces the latter, so a tracker named `ball`
publishes `/vrpn_mocap/ball/pose_id_0`). The HOPE PingPong planner consumes a
`geometry_msgs/PoseArray` with the ball at `ball_pose_index`; map or relay your
tracker's ball pose onto that topic.

## Runtime

The `hope_bringup` package provides launch files that start this client
together with the HOPE PingPong planner.

## License

See [LICENSE](LICENSE).
