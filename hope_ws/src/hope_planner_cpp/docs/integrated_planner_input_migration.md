# Streaming input migration and rollback

Run in `distrobox enter hope`, from this **integration worktree**, not the outer
broad-V2 checkout. Clean-rebuild all C++ dependents: V2 provenance extends
`BallSample`, and mixed old binary objects are unsupported.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/tpark/workspace/HOPE/integrated-racket-planner-v2/hope_ws
colcon build --base-paths src ../NatNet2ROS2/src/motion_capture_tracking_interfaces \
  --packages-select hope_msgs motion_capture_tracking_interfaces hope_bringup hope_planner_cpp \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch hope_planner_cpp integrated_planner.launch.py
ros2 run hope_planner_cpp integrated_preflight.py
```

The default profile waits for qualified ball/base sources and publishes only
shadow topics. It does not start or stop a camera, localization or robot runner.
Copy `config/integrated_planner.shadow.yaml` to an experiment-specific file and
pass `params_file:=/absolute/experiment.yaml`. Supply the measured `frame_id`,
`registration_id`, `world_frame_id` (the existing receipt's U52 correlation ID),
`ball_id`, clock mapping and policy-height convention. `UNQUALIFIED_SHADOW` and
world ID 1 are explicit unqualified defaults, not venue receipts.

Retain the actual `optitrack_mct_relay` and `hope_base_pose_flat_relay`. Configure
the former's `ball_calibration_id` to the same registered identity, and the
latter's existing calibration/world receipts and `source_stamp_mode:=input_header`.
Keep existing `table_side` geometry consistent between both relays. Naming
frames or supplying a calibration string does not transform or survey them.
The test script launches both installed adapters through these actual inputs;
it does not fabricate a base flat array or bypass the mocap relay.

| Legacy graph | Integrated shadow graph |
| --- | --- |
| MCT -> relay -> `/poses` -> packetizer -> `/ball/flight_packet` -> legacy planner | MCT -> same relay -> typed V2 observations -> single integrated planner |
| Pose -> base relay -> `/a3/base_pose_flat` -> legacy planner/runner | Same registered base producer -> configured integrated subscription |
| Legacy planner -> existing command topic | Integrated planner -> exact shadow schema-2 topic -> test decoder |

The integrated node never subscribes to `/poses` or `/ball/flight_packet`, never
launches the standalone forecaster, and requires no referee/context publisher.
The default profile rejects nonzero backhand plane offsets, unknown backends,
unsupported runner profiles and live output at startup. VRPN integrated mode
is unsupported until a typed producer passes the same ladder.

For an isolated comparison, stop only the legacy **planner and packetizer**
in that experiment's process group. Preserve shared sensor and base services.
No command in this guide actuates a robot. On systems where shared source
configuration must not change, use a separate isolated ROS domain/replay.

Rollback: terminate the integrated planner, retain the existing sensor/base
services, restore the original relay parameters if changed for the experiment,
then start the original packetizer/planner launch and original YAMLs unchanged.
Use `prediction_backend:=legacy` (also the default); do not layer the integrated
YAML over a legacy profile. Restart the **receiver together with the selected
command producer** before switching a schema-2 publisher session. Do not infer
nonce support or hot-restart safety from a topic reconnect.

Original legacy input/command paths are exercised by `test_flight_packet_pipeline`,
`test_planner_node_one_shot`, `test_planner_node_turn_guard` and the preserved
numerical goldens. This proves software rollback of those paths; a venue-wide
service/launch transition and operator deployment decision remain separate.

Reproduce all feasible checks using
`scripts/flight_prediction_validation/verify_integrated.sh /new/absolute/evidence`.
The entry point retains inherited bringup lint failures in its exit status.
The continuation entry point also reproduces the pinned `e9bf015` failures,
runs RA/SP/BP cases and the ten-round installed reacquisition matrix, and checks
both policy artifacts using CPU ONNX Runtime. Install NumPy, SciPy and
onnxruntime in the validation Python environment; optionally set
`HOPE_POLICY_PYTHONPATH` to an external tool-only dependency directory.
Use a new evidence directory for every run. Full logs, individual exit codes,
JUnit/CTest output, source pins and raw chain traces are retained even when a
gate fails. Current outcomes are in `integrated_planner_reacquisition_spin_qualification.md`.

The integrated shadow profile now explicitly enables `automatic_rearm_enabled`.
Set it false to retain explicit-boundary recovery; standalone V2 continues to
default false. Rebuild all C++ dependents after this change. No sensor producer
or runner wire migration is needed. PR64 is closed and unmerged; its historical
receipts are not qualification of this continuation, and publication is a
separate decision.
`test/integrated_producer_chain.py` exercises 150/300 Hz and both plane policies,
including pre-bounce UNKNOWN-history commands, real C++ decoding, invalid frame
recovery, missing-base revocation and retained legacy relay outputs. It starts
after the camera driver and uses synthetic registration, not a live camera.
