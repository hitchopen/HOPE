# Vendored: ChingMuVrpnROS2

The `vrpn_mocap` ROS 2 package originated from
<https://github.com/ChingMuVisionTech/ChingMuVrpnROS2> and was first vendored
into HOPE in commit `cff284e73c12b8b0ffd56ac59bd6ad13744e8e27`
(2026-07-18). The original import did not record an exact upstream commit.
The upstream `main` HEAD observed during the 2026-08-05 workspace migration was
`05c90f08e3d54ac517437f66af15e4fe82ebf861`; this is provenance information,
not a claim that the original copy is byte-identical to that revision.

Local HOPE changes include:

1. Sensor-data QoS and multi-sensor topic support used by the HOPE pipeline.
2. Optional preservation of the VRPN server's `msg_time` in ROS headers.
3. Strict validation of server timestamps against the adapter host's
   NTP-disciplined `RCL_SYSTEM_TIME`, with stale/future/invalid frames dropped.
4. A sliding minimum-age change detector plus an optional commissioned baseline
   for startup-static server offset detection. This is validation, not clock
   synchronization: it observes offset plus one-way delay.
5. Pure C++ timestamp/monitor tests and a live ROS timestamp probe.
6. Receipt-time fallback fixed to `RCL_SYSTEM_TIME`, invalid angular derivative
   intervals dropped, and publishers created only after a valid first frame.
7. Extraction into the independently built `VRPN2ROS2` workspace.
8. Per-topic/per-sensor ROS output limiting, default 200 Hz, applied only after
   every VRPN report passes timestamp and minimum-age validation. Selected
   messages retain their original server timestamp; `0.0` disables limiting.
9. VRPN sensor indices are accepted only in the range 0–255 before either the
   limiter or publisher vectors are grown, preventing malformed reports from
   causing unbounded allocations. HOPE rigid-body deployments normally use 0.

Maintenance note: this workspace builds independently, so its
`output_rate_limiter.h` is intentionally duplicated. The NatNet2ROS2 copy is
the behavioral reference; port limiter fixes and matching tests to both
workspaces in the same change.

VRPN protocol fact: the server supplies the message `timeval`; the VRPN core
serializes and forwards it without server/client clock synchronization. Vendor
evidence or a hardware comparison is still required to prove that CMTracker
sets that field to camera exposure time rather than server report time.

Protocol references:

- VRPN tracker server/client callbacks:
  <https://github.com/vrpn/vrpn/blob/master/vrpn_Tracker.C>
- VRPN connection marshal/decode path:
  <https://github.com/vrpn/vrpn/blob/master/vrpn_Connection.C>
- VRPN platform time implementation:
  <https://github.com/vrpn/vrpn/blob/master/vrpn_Shared.C>
