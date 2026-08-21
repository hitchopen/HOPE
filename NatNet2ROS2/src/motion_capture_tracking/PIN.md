# Vendored: IMRCLab motion_capture_tracking (OptiTrack/NatNet driver)

Vendored 2026-07-18 for the HOPE OptiTrack mocap backend and moved into the
standalone `NatNet2ROS2` workspace on 2026-08-05. Plain-copy pin, the same
convention as the sibling `VRPN2ROS2/src/vrpn_mocap` package (this repo does
not use git submodules).

## Provenance

| Component | Upstream | Commit |
|---|---|---|
| motion_capture_tracking (+ `motion_capture_tracking_interfaces`) | https://github.com/IMRCLab/motion_capture_tracking | `037adc497c67b635d1ee50992a9f74a9f753aff4` (v1.0.9) |
| deps/libmotioncapture (submodule, materialized) | https://github.com/IMRCLab/libmotioncapture | `24321e4c1c923a1bc5d6cecdaa7834f11b79d8f2` |
| deps/librigidbodytracker (submodule, materialized for provenance; no longer built by the HOPE competition adapter) | https://github.com/IMRCLab/librigidbodytracker | `020e541b6825595c841f1103bf86e42914d08c9f` |

License: MIT (upstream `LICENSE` kept in this directory).

## Local modifications (complete list)

1. **Removed unused vendor SDK trees** (~25 MB; each referenced only inside its
   disabled `if(LIBMOTIONCAPTURE_ENABLE_*)` guard):
   `deps/libmotioncapture/deps/{vrpn, pybind11, vicon-datastream-sdk,
   qualisys_cpp_sdk, NatNetSDKCrossplatform}`
2. **`CMakeLists.txt` backend trim**: the upstream `CACHE INTERNAL` pins force
   QUALISYS/VICON/VRPN/FZMOTION ON and cannot be overridden from the command
   line; set all to OFF, keeping only `LIBMOTIONCAPTURE_ENABLE_OPTITRACK`
   (open-source NatNet direct depacketizer, all platforms incl. aarch64).
   The built-in `mock` backend is unconditional and remains available.
3. **`CMakeLists.txt` libNatNet install guard**: the x86-64 `install(FILES
   ...libNatNet.so)` now also requires `ENABLE_OPTITRACK_CLOSED_SOURCE` (the
   .so belongs to the removed closed-source SDK tree).
4. **`deps/libmotioncapture/src/optitrack.cpp` unicast fix (2026-07-18, field
   finding vs Motive 3.1.0 / NatNet 4.1)**: Motive's unicast mode streams
   FRAMEOFDATA back to the SOURCE endpoint of the NAT_CONNECT packet — NOT to
   the advertised data port. Upstream registers from a transient command
   socket and then listens on a separate data socket bound to the data port,
   which therefore never receives anything (and Motive expires the silent
   client after a few seconds; upstream sends no keep-alive). Patch: after
   binding, the DATA socket sends its own NAT_CONNECT (so unicast frames
   target it) and `waitForNextFrame` sends a ~1 Hz `NAT_KEEPALIVE` (=10) from
   the same socket. Multicast mode is unaffected (extra NAT_SERVERINFO replies
   are dropped by the existing MessageID==7 filter). Verified live: 0 Hz
   before, camera-rate stream after. Worth upstreaming.

5. **`src/motion_capture_tracking_node.cpp` empty-tracker guard (2026-07-18)**:
   skip `tracker.update()` when no custom `rigid_bodies` are configured (the
   HOPE rigid-body-ball venue tracks the ball as a Motive asset, so the
   point-cloud tracker is unused). librigidbodytracker's `initializePose`
   returns false on an EMPTY cloud, so with unlabeled-marker streaming off the
   unguarded call warns "initialization failed" on every frame (camera-rate
   log flood). Superseded for the HOPE competition executable by patch #13,
   which removes that tracker path from the build and node entirely.

6. **`deps/libmotioncapture/src/optitrack.cpp` reply/stream interleave fix +
   model-definition self-heal (2026-07-19, field finding)**: once patch #4
   registers a socket, Motive floods it with FRAMEOFDATA immediately, so the
   constructor's blind single-packet receives for NAT_SERVERINFO /
   NAT_MODELDEF raced against the frame stream — losing the race either
   killed the node (frame larger than the response struct →
   `boost::asio` message_size throw; the "process has died" startups) or
   silently skipped the model-definition parse → EVERY streamed rigid body
   named `''` (the relay then matches nothing). Fix: loop both constructor
   receives with a message-id filter on a full-size buffer. Additionally,
   rigid-body names are cached at connect, so assets created/renamed in
   Motive later would stream with empty names until a restart — now
   `waitForNextFrame` detects unnamed streamed bodies, re-requests the model
   definition (1 Hz, from the data socket) and parses replies in the
   latest-only drain hook: renames self-heal in ~1-2 s without a bridge
   restart.

7. **`src/motion_capture_tracking_node.cpp` warn throttle (2026-07-21, review
   finding)**: the per-frame `RCLCPP_WARN` for an untracked tracker body
   (line ~321) became `RCLCPP_WARN_THROTTLE(…, 2000 ms)` — an object out of
   the volume is a normal state and the unthrottled warn fired at camera rate.
   Superseded for the HOPE competition executable by patch #13.

8. **`config/cfg.yaml` placeholder hostname + legacy launch removal
   (2026-07-21, hardened 2026-08-05)**: the upstream example's lab IP
   (`130.149.82.37`) is replaced with `MOCAP_PC_IP`. The upstream ROS 2
   `launch.py` and ROS 1 `node.launch` were removed: both bypassed the required
   HOPE clock configuration/remaps and made an unsafe entry point look
   supported. `natnet2ros2.launch.py`, which always loads
   `config/hope_optitrack.yaml`, is the only installed launch entry point. It
   exposes `header_time:=camera_utc|ros` and the output-rate cap; `ros` is an
   explicit diagnostic-only fallback for a Motive version without NatNet echo
   support.

9. **`src/motion_capture_tracking_node.cpp` acquisition-time mapping
   (2026-07-30)**: add `topics.header_time=ros_latency_compensated`, which
   stamps each frame in the local ROS epoch as receive time minus NatNet's
   per-frame Camera + Motive processing latencies and a deployment-measured
   `topics.network_latency_ms`. Bare `ros` is arrival time and biases moving
   cross-sensor calibration; bare `camera` is the Motive host's unrelated
   high-resolution-clock epoch. The mapping avoids both failure modes without
   pretending the remaining one-way network latency can be inferred from a
   single UDP packet.

10. **`deps/libmotioncapture/src/optitrack.cpp` model-definition type mask +
   bounded handshake (2026-07-30, field finding vs Motive 3.1.0.4 / NatNet
   4.1)**: Motive silently DROPS a payload-less `NAT_REQUEST_MODELDEF` — it
   replies only when a 4-byte descriptor-type bitmask follows the 4-byte
   header. Masks with undefined bits set (0x7f, 0xff, ~0) are dropped too, so
   the request carries exactly the two descriptor types `parseModelDef()`
   consumes (`MODELDEF_TYPES = 0x3`: bit0 MarkerSet | bit1 RigidBody). Before
   this patch the constructor's model-definition wait was an unbounded
   blocking receive, so the unanswered request deadlocked the node before
   `create_publisher()` — `/optitrack/poses` existed with 0 publishers and no
   error was logged anywhere. Both connect-time waits (NAT_SERVERINFO,
   NAT_MODELDEF) now use a 5 s receive timeout with 3 send/await attempts and
   throw `std::runtime_error` on exhaustion, so an unresponsive Motive fails
   loudly instead of hanging forever. The patch #6 1 Hz self-heal re-request
   carries the same mask. Pre-launch triage lives in
  `hope_ws/src/hope_bringup/scripts/natnet_preflight.py`, which distinguishes this
   condition from an unreachable Motive or a wrong-interface multicast join
   without needing ROS.

11. **NatNet QPC → adapter absolute-time mapping (2026-08-05)**: add
    `topics.header_time=camera_utc`. The open-source backend performs NatNet
    `NAT_ECHOREQUEST` / `NAT_ECHORESPONSE` clock synchronization (20 startup
    samples, minimum-RTT selection, non-blocking 500 ms refresh) and exposes
    the age plus mapping uncertainty of `CameraMidExposureTimestamp`. The ROS
    node subtracts that age from `RCL_SYSTEM_TIME`, so Linux publishes Unix
    epoch timestamps from its Chrony-disciplined `CLOCK_REALTIME`; it never
    treats Motive QPC ticks as an epoch. Frames exceeding the configured clock
    uncertainty or capture-age limits are dropped. The old
    `ros_latency_compensated` fixed-network-latency mode remains an explicit
    fallback only. Pure clock-mapping tests live in
    `deps/libmotioncapture/tests/test_natnet_clock_sync.cpp`. The legacy V2
    vendor-microsecond field also now divides QPC ticks before multiplication;
    the previous `ticks * 1e6 / frequency` overflowed `uint64_t` after roughly
    21 days at a 10 MHz QPC. Zero or non-monotonic A/B/D timing fields are
    rejected before unsigned latency subtraction, preventing underflow from
    becoming an enormous false latency.

    Runtime RTT-floor recovery is deliberate: the normal update filter rejects
    echoes more than 0.25 ms above the measured minimum, but ten consecutive
    valid high-RTT replies rebase the floor to the best sample in that new
    regime. At the 500 ms echo period this recovers a permanent route/link
    change in about five seconds without restarting. The selected RTT/2 and
    full offset correction still count toward uncertainty, so a poor or
    asymmetric path cannot bypass the ROS-side 2 ms publication gate. NatNet
    gives only Motive's receive tick, not its transmit tick; systematic path
    asymmetry therefore has a bounded bias of up to minimum RTT/2. Deployment
    documentation requires a wired adapter path and distinguishes this 2 ms
    mapping term from the two hosts' clock errors.

12. **Coherent ROS 2 output downsampling (2026-08-05)**: add
    `topics.output_rate_hz`, initially defaulted to 180 Hz and exposed by the supported launch
    file. Every NatNet frame still services the backend, clock mapping, and
    timestamp gates; every valid frame also reaches logging and the optional
    point-cloud tracker. A monotonic deadline selects at most one valid frame
    per output period, then publishes that same frame on `poses`, `pointCloud`,
    and TF without changing its acquisition timestamp. Missed periods never
    produce catch-up bursts; `0.0` disables the cap. Pure limiter tests live in
    `test/test_output_rate_limiter.cpp`. The multi-output portion of this
    historical patch is superseded by patch #13; only filtered `poses` remains.

    Maintenance note: this implementation is the behavioral reference for the
    intentionally duplicated VRPN2ROS2 limiter. Port limiter fixes and matching
    tests to both independently built workspaces in the same change.

13. **Strict competition ROS egress + 200 Hz default (2026-08-10)**: the
    standalone node now publishes exactly one ROS topic, relative `poses`
    (`/optitrack/poses` under the supported launch namespace). Each array is
    filtered and ordered by the exact case-sensitive allowlist `Ball`, `P1`,
    `P2`; missing bodies are silently omitted. A selected, timestamp-valid
    source frame with none publishes an empty array heartbeat so body-tracking
    loss is distinguishable from adapter or transport loss without exposing
    any additional body. All other Motive rigid bodies (including `Table`), marker
    coordinates, point-cloud output, raw TF, skeletons, and arbitrary assets
    are blocked from ROS 2. The legacy unlabeled-marker tracker, PCL, fmt,
    point-cloud logger, and TF publisher are no longer built or executed. The
    downstream HOPE relay remains the sole per-body topic and TF authority.
    The output-rate default is now 200 Hz in the node, deployment YAML, and
    supported launch file. `test/test_competition_rigid_body_filter.cpp`
    locks the allowlist's membership, case sensitivity, and canonical order.

14. **Motive 3.2 / NatNet 4.2 multicast hardening (2026-08-21)**: NatNet 4.2
    adds a quaternion rotation offset to each rigid-body model description.
    The old parser read its first four bytes as the marker count, shifting the
    rest of every definition. MODELDEF decoding is now length-checked, obeys
    NatNet 4.1 `description_size`, consumes the 4.2 quaternion, and is covered
    by 4.1/4.2 plus truncation fixtures. Multicast requires an explicit local
    `interface_ip`; the driver logs the group/interface/buffer, rejects packets
    from other hosts and invalid declared lengths, omits unicast-only recurring
    keepalives, and exits cleanly with a non-zero status on a one-second frame
    timeout. The supported standalone launch respawns the node after two
    seconds, while managed venue supervisors retain their own lifecycle
    recovery. The matching preflight uses the same interface,
    performs a real 4.2 MODELDEF decode, and reports the Motive/NatNet versions,
    multicast group/port, and all three competition assets.

    Maintenance note: the C++ MODELDEF decoder in
    `deps/libmotioncapture/include/libmotioncapture/natnet_modeldef.h` and the
    Python venue decoder in
    `hope_ws/src/hope_bringup/scripts/natnet_preflight.py` intentionally
    implement the same wire schema. Port every schema, version-gate, bounds,
    or dataset-skip fix to both implementations in the same change, with
    matching fixtures in `deps/libmotioncapture/tests/test_natnet_modeldef.cpp`
    and `hope_ws/src/hope_bringup/test/test_natnet_preflight_clock_sync.py`.

15. **MotiveBody 3.5 / NatNet 4.5 additive frame sections (2026-08-21)**:
    multicast cannot negotiate an older bitstream, so MotiveBody 3.5.0.1 Beta
    1 streams native NatNet 4.5. The old FRAMEOFDATA decoder walked every
    field with unchecked `strlen`/`memcpy` and assumed the 4.2 section order;
    NatNet 4.5 IMU/GPIO data could therefore shift the timing suffix. NatNet
    4.1+ frame decoding now uses each section's authoritative byte count,
    bounds-checks rigid bodies and labeled markers, skips appended 4.5 stream
    types until the fixed timestamp suffix, and consumes the two precision/PTP
    timestamp words that the old decoder left behind. `test_natnet_frame.cpp`
    covers 4.2, 4.5 extension sections, timestamps, truncation, and EOD. The
    C++ and Python MODELDEF fixtures both cover unknown 4.5 descriptions. The
    venue preflight identifies Motive/MotiveBody 3.5.0.1 and NatNet 4.5.x as
    the validated profile but only warns on version differences. Multicast
    group `239.255.42.99`, command/data ports 1510/1511, actual decode success,
    and the 300 Hz source-rate check remain operational gates.

16. **NatNet rigid-body marker member IDs (2026-08-21)**: NatNet labeled
    markers encode a model ID in the high word and a 1-based member ID in the
    low word. MODELDEF carries the rigid body's marker array but no separate
    member-ID field, so the decoder assigns array position `marker_index + 1`.
    This keeps MODELDEF marker 1 aligned with FRAMEOFDATA member 1 and restores
    the node's deterministic ID association for P1 calibration; the geometric
    association remains only as a fallback. The MODELDEF fixture locks member
    IDs 1 and 2, matching the FRAMEOFDATA fixture's first member ID of 1.

## Re-pin procedure

```bash
git clone --recurse-submodules https://github.com/IMRCLab/motion_capture_tracking.git
# checkout desired commit, then:
rsync -a --exclude='.git' --exclude='.gitmodules' --exclude='.github' \
  motion_capture_tracking/{motion_capture_tracking,motion_capture_tracking_interfaces} NatNet2ROS2/src/
find NatNet2ROS2/src/motion_capture_tracking* -name '.git' -o -name '.gitmodules' | xargs -r rm -rf
rm -rf NatNet2ROS2/src/motion_capture_tracking/deps/libmotioncapture/deps/{vrpn,pybind11,vicon-datastream-sdk,qualisys_cpp_sdk,NatNetSDKCrossplatform}
# re-apply the complete local modification list above, including the standalone
# launch/config files, then update this file.
```

## Interface facts verified at this pin (used by hope_bringup)

- Executable: `motion_capture_tracking_node` (node name the same). Topics:
  relative `poses` only (→ `/optitrack/poses` inside the supported namespace).
  No point-cloud, marker-coordinate, or raw-TF publisher exists.
- `poses` msg: `motion_capture_tracking_interfaces/msg/NamedPoseArray`
  (`header` + `NamedPose[]{string name, geometry_msgs/Pose pose}`) when
  `topics.poses.version: 1`; a V2 with vendor timestamp/latencies exists. Both
  versions carry only available `Ball`, `P1`, `P2` entries, in that order.
- Params actually read by the node: `type`, `hostname`, `interface_ip`,
  `topics.frame_id`, `topics.header_time`
  (`ros`=arrival time, `camera`=vendor clock, `camera_utc`=NatNet-echo mapped
  CameraMidExposureTimestamp in the adapter host's ROS system-time/Unix epoch,
  `ros_latency_compensated`=legacy local ROS receive time minus NatNet
  Camera/Motive latency and `topics.network_latency_ms`),
  `topics.output_rate_hz` (maximum filtered ROS output rate; default 200 Hz,
  `0.0`=every valid source frame),
  `topics.max_clock_sync_uncertainty_ms`, `topics.max_capture_age_ms`,
  `topics.poses.version`,
  `topics.poses.qos.mode` (`none`=reliable depth-1 | `sensor`=SensorDataQoS
  keep-last-1 + deadline), and `topics.poses.qos.deadline` (Hz). For mock-mode
  bench tests only, `rigid_bodies.<name>.initial_position` overrides are read
  to populate the mock backend; the same Ball/P1/P2 egress filter still applies.
- The only pose source is the vendor-native rigid-body map. The node never
  requests or publishes the unlabeled-marker point cloud and never reconstructs
  a body from individual marker coordinates.
- OptiTrack backend: queries the Motive command port (cfg key `port_command`,
  default 1510) over UDP, auto-discovers the data port and multicast-vs-unicast
  from the server response. Build deps: Boost headers (asio), Threads, Eigen3,
  ROS 2 `rclcpp`, and the named-pose interface package.
