# HOPE onboard node architecture

This consolidation is based on `main` at `1534c2c2`. It retains failed-START
recovery and late DDS discovery from PR #71, the HOME/hit-speed/connection
controls integrated by PR #76, and the measured policy-handoff tolerances from
PR #74. It changes process ownership and transport, not Runner motion policy.
No robot deployment or hardware command was executed for this change.

## Normal node inventory

A full OPTITRACK session has **10 HOPE onboard ROS nodes: 9 on the HDU and 1 on
the MDU**, down from 12 before the three operations nodes were combined. This
count excludes Laptop nodes, vendor HAL/AimRT components, ROS
CLI daemons and helper processes. A process can own worker threads without
creating more ROS nodes or DDS participants.
Opt-in rosbag recording adds recorder nodes beyond this steady deployment count.

| Host | ROS node / component | Lifetime and responsibility |
| --- | --- | --- |
| HDU | Fleet Foxglove bridge, port 8765 | Persistent monitoring and the existing assert-only E-stop surface |
| HDU | Attended Foxglove bridge, port 8766 | Persistent fixed operator service surface |
| HDU | `hope_monitor` | Persistent health, scene and independent safety endpoints |
| HDU | `hope_imu_telemetry` | Persistent native IMU reception and clock telemetry |
| HDU | `hope_operations` | One persistent Python node containing lifecycle, command proxy and observer modules |
| HDU | `hope_time_calibration` | Separate privileged maintenance process, with the existing hardware-operation interlock |
| HDU | C++ Planner | Managed session process; omitted in KERNEL mode |
| HDU | `hope_base_pose_transport_relay` | Independent native base relay, started before Planner/HAL; omitted in KERNEL mode |
| HDU | `hope_transport_gateway` | Managed native node for five Runner wire routes, started after Runner |
| MDU | Native Runner | Managed session process; sole authority for mode, motion, role and action acknowledgements |

The optional `hope_mocap_config_transport` adds **one HDU node**, making 11 in
a full session. It remains separate so its network participant can restart
without interrupting operations or the hardware lock. KERNEL omits Planner and
the base relay; all five Runner gateway routes remain available.

## Operations ownership and scheduling

`hope_operations.py` owns one ROS node. The existing observer, command-proxy
and lifecycle implementations are modules on that node. Runner state and base
pose are decoded once into shared snapshots, then delivered directly to their
consumers. Field status also uses direct local fanout. Public ROS topics and
services remain available for external consumers.

The shared Runner and base readers use reliable, volatile QoS at depth 10.
Each has its own mutually exclusive callback group, separate from telemetry
timers, so the two sources can progress independently without concurrent
callbacks on the same wire. Malformed input preserves the command modules'
last-good value and original receipt time: it cannot refresh freshness. A
decoded invalid base packet explicitly invalidates the source. Separate decode
diagnostics keep malformed packets visible to the observer.

Filesystem cache failures, overdue refreshes and initial pending reads publish
`UNKNOWN` diagnostics. The observer retains known session identity while
suppressing stale Planner/x_hit health claims; a cached result is not evidence
of fresh source data.

The observer's `runtime_snapshot_max_age_s` parameter explicitly defaults to
**2.0 seconds** and must be positive and finite. This is an intentional advisory
budget for filesystem-backed session, Planner-process and x_hit UI snapshots;
it tolerates short file-read stalls without tightening the existing default.
If a refresh stalls after a successful read, the previous health result can
remain displayed for up to that budget, then becomes `UNKNOWN` on the next
publication tick. With the default `publish_period_s=0.2`, that adds at most
one nominal 0.2-second tick: **2.2 seconds before executor/DDS delivery delay**,
not a hard real-time bound. An initial pending read never establishes health;
it is labelled overdue after the same budget. A read error marks the snapshot
unknown at the next tick without waiting out the remaining hold time.
Session/attempt identity remains available as last-known context until a
successful refresh replaces it.

This cache is not used for command admission, command acknowledgements,
Runner/base freshness or lifecycle Runner readiness. Those paths continue to
use their own direct reads, decoded source receipts and existing deadlines.
The read-only observer layout restores colored session/Planner summaries and
places `/hope/session/runtime_known` prominently beside observer liveness.
When that indicator says `UNKNOWN`, disregard the session/Planner Bool
summaries and inspect the compact `/hope/session/text` and
`/hope/planner/process_status` panels for the exact diagnostic. A current
snapshot does not by itself mean that a session or Planner exists.

Blocking SSH, filesystem/asset and command work uses bounded Python workers;
status callbacks consume cached snapshots. Workers do not create ROS nodes.
This keeps long operations out of source-data callbacks and rejects excess
work instead of building an unbounded queue. The native gateway and IMU tool
retain the high-rate wire paths. This design does **not** establish a CPU or
latency improvement: compare profiles under representative load before making
performance claims.

The canonical service remains `hope-lifecycle-supervisor.service`, preserving
its state directory, hardware lock and venue-specific systemd drop-ins. It now
runs `hope_operations.py`. The old `hope-observer.service` and
`hope-command-proxy.service` files are compatibility oneshot units that run
`/usr/bin/true` and require the canonical service. They create no ROS process
and are not normally enabled. Their old Python entry points remain compatibility
wrappers; do not launch them alongside `hope_operations`.
Starting a compatibility shim starts the canonical service, but stopping a shim
does **not** stop operations. Use `hope-lifecycle-supervisor.service` for stop or
restart; the observer cannot be independently disabled in the combined node.

A detected network change must not restart operations during an active
lifecycle session or explicit action. Recreation is deferred until the
lifecycle is STOPPED and its workers and hardware-lock owners are idle;
`/hope/operations/status` reports `DDS_RECONNECT_DEFERRED` while waiting.
The optional mocap configuration
transport can recreate its own participant independently; it forwards each
original snapshot once and does not invent cached heartbeats.

## Runtime scheduling and fault reporting

| Component | Scheduling and freshness behavior |
| --- | --- |
| Operations executor | Five threads; the observer snapshot timer has its own mutually exclusive group. Command state and ball receipts are copied under a short lock before formatting and publishing. |
| Monitor probes | CPU reads and delta calculation run together in a worker; interface reads also use the probe pool. Five workers serve at most one pending probe per source. The collecting timer propagates deliberate reconnect exceptions. |
| Observer cache | Successful filesystem snapshots wait half of their maximum-age budget before another refresh. Initial reads and failed reads retry promptly. Network and field-status caches retain their existing polling behavior. |
| Gateway graph and readiness | Graph queries refresh at nominal 1 Hz; the 500 ms readiness timer and all five forwarding callbacks stay serialized. An overdue graph snapshot is refreshed before use. Initial `HEALTHY` requires two eligible reports and an additional forwarded Runner sample between them. An unhealthy report resets the gate. |
| Shared field status | An immutable parsed snapshot accompanies the unchanged external JSON wire. The command proxy skips local JSON decoding but polls every 0.5 seconds to reevaluate live base receipt freshness, even when metadata is unchanged. |
| Wire reads | Each command guard/wait iteration uses one immutable `WireSnapshot`. The observer retains `/poses` depth 4 and its existing receipt-rate semantics. Base, Runner and IMU forwarding remains native. |
| Telemetry faults | Observer snapshot/ball/command callbacks, lifecycle publishing and the composed field-status poll report faults without terminating lifecycle ownership. Control and reconnect callbacks remain outside this guard. |
| Reconnect status | `/hope/operations/status` is offered `DDS_RECONNECTING` before raising `DdsReconnectRequired`. Failure exit and graceful drain remain intact even if that status publish fails. Gateway rematching requires attended verification. |

Lifecycle/config/mocap topics retain periodic publication, including changing
ages, countdowns, state/busy values and freshness-bearing heartbeats. The
[Jazzy QoS durability table](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)
shows that a transient-local publisher matches a volatile subscriber for new messages;
historical samples require the subscriber to request transient-local durability
too. Publisher durability alone does not replace periodic publication or
field-status freshness checks.

The observer refresh floor is 1.0 second with the default 2.0-second age budget.
The age budget and next-publication behavior described above remain the limit
for displaying the last completed filesystem snapshot; this is not a guarantee
of instant detection of a new filesystem error. A completed read failure makes
the cache unknown at the next publication and permits an immediate retry.

For telemetry faults, the attended bridge exposes
`/hope/operations/telemetry_fault` (`String`) and
`/hope/operations/telemetry_fault_count` (`UInt64`). The string includes `FAULT`
or `OK`, active callback names and the last error; the counter is cumulative
within the current process and resets on restart. These diagnostics publish on
failure/recovery and with a one-second heartbeat. Inspect the string to
distinguish an active failure from a nonzero historical count. Continued
lifecycle availability does not establish that an affected display is current.
The read-only observer layout includes this status above its plots, with the
counter embedded in the text so it remains visible after a transient recovery.
Snapshot failures also attempt to clear observer/health indicators; a failed
publisher cannot itself guarantee delivery of that invalidation.

The callback guards deliberately do not wrap `_check_network()` or broadly
catch the normal executor loop. A real reconnect exception must still escape
through Jazzy, drain accepted responses, and exit non-zero. The reconnect gate
below remains required. Monitor probe, observer concurrency/fault and native
gateway regressions must also run with ROS enabled in the isolated test domain;
a skipped local test is not deployment evidence. These changes make no measured
CPU or latency claim, and do not replace the attended field/rematching checks.

Every checked-in Foxglove `.yaml` and `.yml` asset is parsed by
`test_yaml_assets.py`; PyYAML is required rather than an optional skipped check.
This extends the existing control-bridge YAML parse test to all YAML assets.
CPU probe collection clears a completed future before retrieving its result,
so a caller that recovers from a probe exception can submit a fresh probe on
its next poll. The deliberate interface-reconnect exception path is unchanged.

## Reconnect verification required before deployment

The HDU unit explicitly sources `/opt/ros/jazzy/setup.bash`. Operations relies
on that `rclpy` runtime propagating a timer callback exception through
`MultiThreadedExecutor.spin_once()`: `_check_network()` seals admission and
raises `DdsReconnectRequired` once operations is quiescent. `run_node()` lets
that exception escape after its `finally` block drains accepted work and
responses, joins the executor, and destroys the node. The resulting non-zero
process exit qualifies for the unit's `Restart=on-failure` policy.
The [Jazzy executor source](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/executors.py)
retrieves completed callback results and re-raises their exceptions.

The async service adapter is valid on Jazzy: `await ready` suspends the service
coroutine until `finish()` stores its response; it does not return an unfinished
response. See [Jazzy `Future.__await__`](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/task.py).
The drain tests separately hold a resumed RPC after worker completion to verify
that cleanup does not destroy DDS before the response is sent.
The timeout subprocess regression holds a service coroutine indefinitely after
its worker completes, while keeping cleanup blocked longer than the response
budget. It verifies that the response deadline starts only after cleanup,
reports the pending callback source, and allows executor/node teardown.

Treat the reconnect subprocess test as a deployment gate, using the same
Jazzy package versions as the intended HDU installation. Run from the matching
checkout in an isolated test environment; the tests replace hardware backends:

```bash
source /opt/ros/jazzy/setup.bash
dpkg-query -W ros-jazzy-rclpy
python3 -c 'import inspect; from rclpy.executors import MultiThreadedExecutor; print(inspect.getfile(MultiThreadedExecutor))'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 \
  python3 -m pytest -q -rs foxglove/tests/test_operations_reconnect_ros.py \
  foxglove/tests/test_operations_drain_ros.py foxglove/tests/test_operations_drain_timeout_ros.py \
  foxglove/tests/test_operations_service.py
```

Record the actual `rclpy` package version/source path and the unskipped result.
A skipped ROS class or a suite-wide pass count does not satisfy this gate.
The required evidence is the real periodic network callback reaching the
executor, `DdsReconnectRequired` escaping the production entry point, cleanup
finishing, and the child process exiting non-zero rather than remaining alive
with sealed admission. The direct `_check_network()` call in
`test_operations_ros.py` verifies the quiescence/seal guard and exception type
only. Worker-core and peer-wrapper tests cannot establish executor propagation.
The subprocess test explicitly launches a second process and verifies that it
starts with admission open; this is a replacement-instance check, not a systemd
restart observation.
Inspect the effective unit and retained drop-ins for `Restart=on-failure`;
the subprocess test and that setting establish the restart prerequisites, not
an observed systemd restart on the HDU. Record a stopped, attended restart
observation separately before claiming field recovery is verified.

Local verification on 2026-10-22 passed in the network-isolated
`hope-dev:jazzy-arm64` container with Python 3.12.3 and
`ros-jazzy-rclpy` package `7.1.12-1noble.20260911.111116`. Its executor source was
`/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/executors.py`. The production
timer exception propagated through the executor and entry point; cleanup
completed, the reconnect process exited with code 1, and the separately
launched replacement started open and exited normally on the fixture's SIGTERM.
This closes the local executor verification gap for that package. Matching the
intended HDU runtime and observing its effective systemd restart remain required
before deployment; no HDU or hardware action was performed for this check.

`WorkerAdmission` deliberately has no transition back to `OPEN`: a replacement
process creates fresh admission. An executor that swallowed this exception
would leave the old process rejecting managed operations requests, including
`/hope/runner/emergency_passive`. Maintained Humble also re-raises callback
exceptions; this is not a general Humble rollback hazard. The historical
swallowed-exception report was from older Foxy-era code, with the fix later
backported, so verify the installed package rather than infer behavior solely
from a distro name. See the [Humble executor](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py),
[original report](https://github.com/ros2/rclpy/issues/983), and
[fix/backports](https://github.com/ros2/rclpy/pull/1073).

This failure would degrade the managed Runner leg of Foxglove E-stop, not remove
the independent `/hope/safety/trigger_estop` service in `hope_monitor`. The
monitor still attempts the vendor emergency RPC and reports `PARTIAL E-STOP`
if the vendor accepts while the managed Runner request fails. That path still
depends on the vendor service being available; separate monitor ownership does
not itself establish vendor readiness. No E-stop request is needed to run the
reconnect tests.

## Native transport routes and startup order

The independent base relay and five-route Runner gateway preserve every
`Float64MultiArray` data/layout field, including
source timestamps, session identity and sequence. They do not generate commands,
replace source time or replay a cached payload from a timer.

| Owner | Input | Output | Reliable, volatile depth |
| --- | --- | --- | --- |
| Early base relay | `/a3/base_pose_laptop_flat` | `/a3/base_pose_flat` | 10 |
| Late Runner gateway | `/hope/runner/state_flat` | `/hope/runner/state_hdu_flat` | 10 |
| Late Runner gateway | `/hope/runner/control_request_hdu_flat` | `/hope/runner/control_request_flat` | 10 |
| Late Runner gateway | `/hope/runner/teleop_input_hdu_flat` | `/hope/runner/teleop_input_flat` | 1 |
| Late Runner gateway | `/hope/runner/teleop_state_flat` | `/hope/runner/teleop_state_hdu_flat` | 1 |
| Late Runner gateway | `/hope/runner/session_home_flat` | `/hope/runner/session_home_hdu_flat` | 1 |

The checked-in `with_fastdds_unicast.sh` remains the transport wrapper. It
retains domain 232, the confirmed Laptop/MDU peers, explicit discovery peers,
and route-derived Wi-Fi/internal interfaces. Do not replace it with a generic
profile or assume that `ROS_STATIC_PEERS` alone reproduces this discovery path.

Managed startup is:

```text
PREFLIGHT -> SESSION -> BASE_RELAY -> PLANNER -> HAL -> RUNNER(PASSIVE)
          -> TRANSPORT_GATEWAY -> RUNNER_VERIFY
```

KERNEL skips BASE_RELAY and Planner. Laptop mocap is independently managed.
The native base relay starts before Planner/HAL and remains independent of
Runner: live Laptop base data can reach the HDU when Runner is absent, Runner
startup fails or the Runner gateway crashes. The relay does not wait for base
traffic to start. HOME is inherently produced by Runner and remains on the late
gateway; it cannot be available before Runner produces it. The field Runner
participant must exist **before** the gateway's explicit-peer reader starts;
starting that reader first reproduces the known field rematching failure.
The helper therefore starts `hope-transport-gateway` only after Runner reports
its local startup readiness. Gateway readiness requires fresh Runner traffic
and bidirectional endpoints, with one operations subscriber sufficient. Missing
base/mocap traffic does not block this Runner transport gate.

The helper uses tmux session `hope-transport-gateway`, binary
`/usr/local/libexec/hope-transport-gateway`, and session log
`hdu/transport_gateway.log`. Only a `TRANSPORT GATEWAY HEALTHY` record written
during the current launch attempt qualifies readiness. The independent base
relay uses tmux session `hope-base` and native binary
`/usr/local/libexec/hope-base-pose-transport-relay`; it does not run Python 3.
Shutdown cleans both managed transports and legacy `hope-runner-transport`
sessions left by an older install.

## Stopped migration and installation

Use one matching checkout for the Python modules, native base relay, gateway
and lifecycle helper. Stage files first; only install when lifecycle is **STOPPED**, no
managed Runner/Planner/HAL session remains, and the operator has completed the
normal supported shutdown. Installing code is not a reason to issue a motion
or emergency command.

Before replacing units, save `systemctl cat` output and the existing
`/etc/hope-foxglove` configuration and `*.service.d` drop-ins. Stop and disable
the old observer/proxy services **before overwriting their unit files**;
otherwise their old Python processes can survive while the new shared node
starts. Stop the canonical service for the module update too.

On the HDU, with matching files staged in `$HOME/foxglove_a3` and the helper at
`/tmp/hope-lifecycle`:

```bash
# Preconditions above must already be satisfied. These commands change the HDU.
export HOPE_STAGE="$HOME/foxglove_a3"
for unit in hope-observer hope-command-proxy; do
  if systemctl cat "$unit.service" >/dev/null 2>&1; then
    sudo systemctl disable --now "$unit.service"
  fi
done
if systemctl cat hope-lifecycle-supervisor.service >/dev/null 2>&1; then
  sudo systemctl stop hope-lifecycle-supervisor.service
fi

source /opt/ros/jazzy/setup.bash
cmake -S "$HOPE_STAGE/native" -B "$HOME/hope_field_native_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/hope_field_native_build" -j2
sudo install -D -o root -g root -m 0755 "$HOME/hope_field_native_build/hope-transport-gateway" \
  /usr/local/libexec/hope-transport-gateway
sudo install -D -o root -g root -m 0755 "$HOME/hope_field_native_build/hope-base-pose-transport-relay" \
  /usr/local/libexec/hope-base-pose-transport-relay
sudo install -D -o root -g root -m 0755 "$HOME/hope_field_native_build/hope-imu-telemetry" \
  /usr/local/libexec/hope-imu-telemetry
sudo install -D -o root -g root -m 0755 /tmp/hope-lifecycle /usr/local/libexec/hope-lifecycle

# Stage the complete module set: copying only the entry point is insufficient.
sudo install -d -o root -g root -m 0755 /usr/local/lib/hope-foxglove
sudo install -o root -g root -m 0644 "$HOPE_STAGE"/hope_*.py /usr/local/lib/hope-foxglove/
for entry in hope_operations hope_lifecycle_supervisor hope_observer hope_command_proxy; do
  sudo install -D -o root -g root -m 0755 "$HOPE_STAGE/$entry.py" "/usr/local/bin/$entry.py"
done
for unit in hope-lifecycle-supervisor hope-observer hope-command-proxy; do
  sudo install -D -o root -g root -m 0644 "$HOPE_STAGE/$unit.service" "/etc/systemd/system/$unit.service"
done
sudo systemctl daemon-reload
```

The complete module set includes `hope_operations_core.py`,
`hope_operations_runtime.py`, `hope_telemetry.py`, the existing
observer/command/lifecycle cores, field operations/assets, ball-model and
distributed-recording dependencies. Keep the existing lifecycle-helper
companions (`hope_sim2real_capture.py`, `hope_sim2real_control.py`,
`hope_catch_background.py`, `hope_wifi_policy.py` and their dependencies) staged
from the same checkout. See the [full installation procedure](foxglove_first_hardware_test.md#64-安装-fleet-monitorrunner-control-plane-和-lifecycle-supervisor)
for the other entry points and services.

Reconcile user/path overrides and prior observer/proxy parameter overrides
into the canonical unit's drop-ins. An old drop-in that replaces `ExecStart`
can still launch the obsolete executable: update the canonical override to
`hope_operations.py`, and archive obsolete observer/proxy `ExecStart` overrides
outside their active `.service.d` directories after migrating their settings.
The compatibility units must resolve to `/usr/bin/true` when inspected with
`systemctl cat`; retain the original files as rollback evidence.
The canonical unit allows up to 45 minutes for graceful shutdown: it closes
new admission, reports `DRAINING`, and keeps lifecycle state, step, field status
and `/hope/operations/status` publishing while accepted jobs and their service
responses drain. Requests received during shutdown are rejected as `DRAINING`,
distinct from a busy worker. The hardware lock remains owned until accepted
cleanup finishes. Once cleanup completes, suspended service responses have a
separate five-second drain budget. Expiry logs pending callback names and counts
and abandons those responses before teardown; a client must not infer action
failure or success from a missing response. This deadline does not interrupt
worker cleanup or a callback blocked synchronously on an executor thread.
The outer 45-minute bound covers collection/recovery; a stopped,
idle instance exits promptly. Do not shorten this timeout through a retained
old drop-in or force-kill operations while it owns a hardware transaction.
Preserve venue addresses, DDS profiles, calibration assets, credentials and
recording paths; do not overwrite them with example files. Then enable only
the canonical operations unit, alongside the independently installed bridges,
monitor, IMU and time-calibration services:

```bash
sudo systemctl enable --now hope-lifecycle-supervisor.service
systemctl status hope-lifecycle-supervisor.service --no-pager
systemctl is-enabled hope-observer.service hope-command-proxy.service  # expected: disabled
```

If `ROS_STATIC_PEERS` is empty, the canonical unit starts with the existing
address-agnostic DDS profile and writes a warning to stderr. This migration
fallback keeps configuration and diagnostics available; the lifecycle helper
still requires configured peers before START. Configure the venue's
`/etc/hope-foxglove/network.env`, then restart the canonical service while the
lifecycle is STOPPED to apply its environment. With peers configured, the unit
uses the explicit-peer wrapper above.

The read-only inspection should show one `hope_operations` node and no separate
observer/proxy/lifecycle ROS nodes. During an approved normal session, verify
the independent base relay before Planner/HAL, one gateway after Runner, and
original source freshness on the base route and five Runner routes.
Do not use START, Stand or E-stop as installation smoke tests. Clock-maintenance
restore lists use the canonical operations service, not the two compatibility
units. Retain a coherent previous module/unit/native/helper bundle for rollback.

Validation for this change is isolated ROS transport/concurrency testing and
source checks. Venue interface selection, field Runner rematching and CPU
profiling still require a separate stopped deployment and attended field test.
