# Native operations service

`hope-operations` is the single ROS node `hope_operations`. It owns DDS,
Runner/base decoding and receipt timestamps, player-side evidence, ball markers,
and every observer scalar topic. `hope_operations_worker.py` is its non-ROS
child and retains the existing `CommandProxyModule` and `LifecycleModule`
hardware orchestration, configuration, recording, SSH and ordinary-action
admission rules. Fixed Runner wire dispatch, request IDs and ACK tracking are
native; the emergency-passive service has its own native executor.
The worker has no ROS context, executor or DDS participant.

This is a local performance candidate. The hardware workflows and motion
controller are not rewritten. No robot installation is implied by building it.

## Data and behavior contracts

- One reliable, volatile depth-10 reader each for Runner and pelvis state;
  ball/racket retain best-effort depth 4, raw player poses retain sensor QoS.
- Every sensor packet is processed natively. Player evidence retains the
  existing **any player** side rule, 250 ms receipt deadline, frame/quaternion
  checks, and duplicate/replayed source-stamp rejection.
- Runner/base snapshots retain original monotonic receipt times, exact integer
  identities and last-good-on-malformed behavior. A decoded invalid pelvis
  packet invalidates the source immediately. Observer diagnostics separately
  show malformed packets as invalid, matching the Python reference.
- Python reads snapshots on demand. A waiting command registers the last
  observed generation; a newer generation completes the wait immediately.
  This avoids the read-to-wait race without polling or forwarding every packet.
- Native observer topics, status-change/heartbeat behavior, 5 Hz numeric output,
  30 Hz ball-marker limit and rolling real-receipt ball rate match the reference.
  The file cache still runs the original Python parsers and age policy, sending
  a single compact runtime snapshot rather than all observer ROS messages.
  Its original age travels with it; IPC delay never renews filesystem evidence.
- Two private inherited Unix sockets: control RPC/status/snapshots and optional
  diagnostic samples. Diagnostic backpressure cannot occupy the control socket.
  The high-rate Runner/base/ball/racket/player path crosses neither socket.
  Vendor diagnostic CDR uses the original generated Python message types and
  publisher GIDs; no undocumented vendor layout is inferred.
  Pending samples are depth-one **per subscription** in C++ before serialization
  and again in the worker: a slow diagnostic
  consumer drops superseded samples, never RPC replies or the worker connection.
  Paired original monotonic/wall receipt clocks reach the generated CDR callback
  through MessageInfo, alongside publisher identity. This matches the original
  depth-one diagnostics, not a lossless event recorder.
- ROS request/input callbacks contain serialization and decode failures.
  Oversized field uploads retain the 9 MiB rejection; other oversized IPC
  requests fail individually and release their pending slots. Invalid UTF-8 is
  replaced at JSON serialization; corrupt subscription packets are dropped with
  throttled diagnostics rather than terminating the owner.
- Ordinary actions retain Python's existing bounded worker and action lock, so
  calibration and fixed Runner actions cannot overlap accidentally. Native
  Runner dispatch/ACK handling uses one publisher and ID allocator. Emergency
  passive bypasses Python and both sockets on its own executor, with one urgent
  action in flight. Native ACKs are captured at ingress, not lost when a later
  ACK replaces the latest state. A pre-emergency request still waiting in Python
  is rejected as superseded, never published after the emergency. These are
  explicit software requests, not a qualification of physical E-stop safety.
- Timeouts report
  an unknown outcome; they never replay an action. No new motion-admission gate
  or freshness deadline is added. Existing module action/client deadlines remain
  authoritative; the overall action-response override is disabled by default,
  so long asset/cleanup actions retain their original completion behavior.
  Individual control-frame writes have a five-second total budget, including
  serialization, writer-lock wait and partial writes. Snapshot/query timeouts
  include sending. A zero-byte send failure rejects only that request; a
  partially written frame breaks framing, so that channel cannot be reused.
  Native input callbacks enqueue bounded replies rather than blocking on IPC.
  Failure of the separate optional diagnostic channel does not restart the owner.
- During worker initialization, advertised services retain requests in their
  normal bounded DDS queues; their callback group joins the executor only after
  the worker is ready. No action runs against partially constructed modules.
  Failed or interrupted startup releases queued requests as failures, not
  actions. This preserves the reference's initialize-before-service-execution
  ordering without a second application queue or automatic retries.
- The standalone proxy network watcher is disabled. The combined owner only
  restarts after lifecycle, commands and admitted responses are idle. Active
  sessions retain their session identity and hardware lock and report
  `DDS_RECONNECT_DEFERRED`.
  Player IPC queries do not hold the lifecycle lock. Reconnect probes also run
  outside the service-admission lock; a generation check cancels sealing if
  any RPC was admitted during the probe, even if it has already returned.
- SIGTERM seals admission and drains accepted work while DDS and snapshot
  queries remain available. Only the worker's drain receipt permits native
  teardown after accepted native commands finish. The existing 45-minute systemd outer cleanup budget is preserved.
  A worker crash returns a failure and exits non-zero; no actions are replayed.

### Restart identity and operational diagnostics

The shared lifecycle module records `/var/lib/hope-lifecycle/operations_session.json`
before hardware mutation and updates it with the verified Runner boot identity.
This is a same-host-boot receipt bound to the session and unchanged configuration,
not permission to resume motion. After an owner restart, a matching receipt is
shown as `FAILED / MANAGED_RECOVERY_REQUIRED`, with the hardware interlock retained
when available. START is not replayed and cannot implicitly kill that surviving
session. Explicit **Kill All and Collect** verifies a fresh Runner packet against
the receipt's session fingerprint/boot and acquires the interlock before cleanup.
Split-layout supervisors subscribe directly to `/hope/runner/state_hdu_flat`
for the same identity check; they do not depend on a composed shared-state object
or trust the observer's mode/session-match scalar topics as recovery evidence.
Normal START/STOP, PD_STAND, SERVE and RALLY semantics are unchanged.

No fresh Runner packet does not prove that the robot has stopped. If the Runner
has crashed or cleanup/partial startup was interrupted, use the explicit offline
procedure below; do not delete `operations_session.json` or reboot to bypass it.
Conflicting configuration, corrupted receipt or a different host boot still
requires manual investigation; the procedure does not guess a session identity.
Receipts are retired after confirmed cleanup, before admitting another START.
Receipt write failures are logged but do not veto normal operations; recovery
cannot be guaranteed when evidence could not be saved. Existing sessions from
before this change have no receipt and are not guessed/adopted automatically.

#### Attended recovery when Runner identity is unavailable

This is a **cleanup operation, not a clear-latch shortcut**. Keep A3 mechanically
supported and motion stopped, with an on-site operator at a verified wired
E-stop. The existing STOP helper restarts the vendor MDU process manager and can
automatically enter Manual / enable joint control. Explicitly authorize that
restoration; do not rely on the handheld E-stop during HOPE ownership. No HOPE
START, stand, motion, clock step, or E-stop reset is sent by this procedure.

1. Record the receipt's `session`, mode and configuration and inspect current
   HDU/MDU processes. Keep the matched configuration, SSH keys and helper paths;
   do not edit the receipt/configuration to make validation pass.
2. Stop the canonical operations owner and any **present** split observer/proxy
   units. Wait for their accepted work to drain. Leave the native monitor,
   handheld remote and all E-stop services untouched. Do not run START or clock
   calibration concurrently. Review `systemctl status`/`systemctl cat` first on
   split installs; do not treat an unknown/missing systemctl response as stopped.

   ```bash
   sudo systemctl stop hope-lifecycle-supervisor.service
   # Only if these legacy split units are installed:
   # sudo systemctl stop hope-observer.service hope-command-proxy.service
   ```

3. As the configured robot account (normally `agi`, **not root**), source the
   installed ROS environment and run the following complete block, replacing
   the session value with the exact value read in step 1:

   ```bash
   (
   set -e
   source /opt/ros/jazzy/setup.bash
   export PYTHONPATH="/usr/local/lib/hope-foxglove${PYTHONPATH:+:$PYTHONPATH}"
   python3 /usr/local/lib/hope-foxglove/hope_lifecycle_supervisor.py \
     --recover-session '<EXACT_SAVED_SESSION_ID>' \
     --confirm-supported-stopped-wired-estop \
     --confirm-vendor-restore
   )
   ```

   Apply the same trusted `HOPE_ROBOT_USER`, `HOPE_LAPTOP_USER`, `HOPE_MAC_USER`
   and custom helper-path environment overrides as the deployed supervisor, if
   present. The command requires stopped owner units, their empty cgroups, the
   existing exclusive hardware interlock, and a same-boot matching receipt/config.
   It reuses the normal `kill_all_and_collect` backend, including its remote
   STOP and log-collection checks. It **does not require a Runner/DDS reply**.

4. On success the receipt is moved to
   `/var/lib/hope-lifecycle/operations_session.recovered-<id>.json`, preserving
   evidence. On any failure it remains in place; fix the reported problem and
   rerun the same command. A lost SSH connection is not confirmation of cleanup.
   After verifying the restored vendor state and collecting the printed result,
   restart only the intended operations layout and confirm lifecycle `STOPPED`
   before an explicitly attended START. Do not start legacy split units alongside
   a consolidated/native owner. No HOPE control-plane service is restarted by
   the CLI; vendor PM restoration is part of the STOP sequence above.

Native emergency requests are included in idle-reconnect checks. An emergency
accepted after the idle snapshot is still drained through its ACK or configured
timeout, including worker-failure shutdown. Its DDS response is sent before the
native shutdown seal completes; this does not guarantee client receipt across a
broken network and never implies that the physical stop was verified.

`/hope/operations/metrics` is a read-only JSON diagnostic (also allowed through
the control bridge): worker/channel health, native/worker sample and coalescing
counts, pending samples, worker dispatch delay, Runner pending/dispatched counts,
ACK timeouts and last/maximum ACK latency. These metrics add no motion gate.
Measure them with CPU during uploads, diagnostic bursts and paused-worker tests.
Latency is host-monotonic software latency, not physical stopping time.

Time calibration accepts stopped masked **optional legacy** observer/proxy units
under the verified consolidated owner, restores only previously active services,
and never unmasks them. Required units and remaining processes retain their
existing checks. Bridge/recorder ordering names the canonical supervisor.

`operations_observer_core.hpp` is a maintained schema/calculation twin of
`hope_observer_core.py`, `hope_runner_control_core.py`, `hope_player_positions.py`
and `ObserverModule._publish_snapshot`. Port changes to both implementations
and extend `test_native_observer_parity.py` in the same change. Python remains
the compatibility/reference implementation, not a second deployed ROS owner.
`-ffp-contract=off` preserves Python's separate floating-point operations on ARM.
Oversized synthetic timestamps use exact 128-bit integer subtraction before
conversion to double, avoiding platform-dependent `long double` precision on
Apple ARM64 while keeping exact timestamp and formatted-text parity assertions.
Do not enable fast-math for this target.

## Build and isolated verification

The maintained regression suites, inert fixtures and repeatable CPU benchmark
live under `foxglove/tests/`; keep them in the repository, not on the robot.
Production builds default to `BUILD_TESTING=OFF`, and the operations installer
explicitly uses that setting. Its deployment manifest copies only runtime
files, never test drivers, mock SDKs, benchmark output or temporary build files.
The Python reference remains for parity testing and the documented first-install
and rollback paths; it is not launched alongside the native owner.

Use the deployment's ROS Jazzy and vendor SDK prefix. The SDK mock under tests
is only for isolated verification and must never be installed on a robot.
Build dependencies include `nlohmann-json3-dev` and `libfmt-dev` as well as the
existing native ROS dependencies.

On Ubuntu, install the two additional build libraries with
`sudo apt-get install nlohmann-json3-dev libfmt-dev` before configuring this
shared native CMake project (including monitor-only builds from this checkout).

```bash
source /opt/ros/jazzy/setup.bash
# Source the installed A3 vendor ROS prefix here before production builds.
cmake -S foxglove/a3/native -B /tmp/hope-native-build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DHOPE_MONITOR_REQUIRE_VENDOR_RPC=ON
cmake --build /tmp/hope-native-build -j4
ctest --test-dir /tmp/hope-native-build --output-on-failure
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 \
PYTHONPATH="$PWD/foxglove/a3${PYTHONPATH:+:$PYTHONPATH}" HOPE_NATIVE_BIN_DIR=/tmp/hope-native-build \
python3 -m pytest -q -p no:cacheprovider foxglove/tests
```

The parity replay compares all observer scalar outputs, all Runner enums,
malformed/non-finite packets, last-good retention, invalidations, timestamps,
clock jumps, 150/300 Hz histories and player replay/boundary cases against the
unchanged Python calculations. ROS integration substitutes inert hardware
backends and verifies fixed service replies, emergency concurrency, calibration
client results, START/STOP, held hardware locks, telemetry-fault containment,
worker loss, SIGTERM draining and idle-only reconnect. These tests do not
qualify physical E-stop behavior or robot motion.

Tests and the CPU benchmark share `operations_processes.child_pids`: kernels
without `/proc/<pid>/task/<pid>/children` use direct PPIDs from `/proc/*/stat`.
Worker-count and fault-injection assertions still apply. Reconnect tests allow
three seconds for the cached OS probe, one-second network watcher and status
delivery, with a six-second **fixture-only** ACK deadline. They continuously
assert that the owner and unacknowledged request remain live during that wait,
then verify ACK/supersession, shutdown ordering and no replay. Production
Runner timeouts are unchanged.

To measure CPU in the same isolated environment after the tests finish:

```bash
PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 \
PYTHONPATH="$PWD/foxglove/a3${PYTHONPATH:+:$PYTHONPATH}" HOPE_NATIVE_BIN_DIR=/tmp/hope-native-build \
python3 foxglove/tests/benchmark_operations_cpu.py
```

This runs identical native publisher loads sequentially against the consolidated
Python reference and native service. It counts the native parent **and worker**,
uses 100% for one CPU core, and excludes the common publisher/test harness.
Ball/base input is 150 or 300 Hz, each player's START evidence is 30 Hz,
Runner state is 50 Hz and racket commands are 30 Hz. Each measurement follows
a 3-second warmup and samples CPU for 20 seconds.
The mocked, localhost workload is not an estimate of whole-A3 savings. Match
the live workload, subscriptions, diagnostics and recording before field claims.

Local ARM64 Ubuntu 24.04/Jazzy replay on 2026-10-29 (100% = one CPU core):

| Ball/base input | Consolidated Python | Native + Python worker | Observed CPU reduction |
| --- | ---: | ---: | ---: |
| 150 Hz | 100.8% | 11.2% | 88.9% |
| 300 Hz | 104.0% | 14.4% | 86.1% |

These are short matched-input samples, not A3 measurements or an endurance
claim. Python saturated roughly one core and delivered fewer marker updates
under this load; native delivered approximately 27–28 markers/s with the same
30 Hz cap. Do not extrapolate the reduction to total robot CPU or assume the
same saving for a lightly loaded field session. ROS used rclcpp 28.1.22 and
rclpy 7.1.12 in the isolated `hope-dev:jazzy-arm64` image.

Verification record (2026-10-31; integrated with main `04cd1e32`, #108):

- Native ARM64/Jazzy build succeeded with the test-only SDK fixture; **2 CTests
  passed**. The fixture must never be installed on A3.
- The complete `foxglove/tests` run passed **1,194 tests, with 28 skipped**
  (198.73 s), with no deselections or failures. This includes the previously
  intermittent monitor fresh-sample case and all corrected asset assertions.
- On A3 HDU kernel 6.1.118 / Jazzy rclcpp 28.1.13, the corrected maintained
  suite passed **29 tests** (103.27 s): 24 ROS integration cases against the
  installed native binary plus five process-discovery regressions. All hardware
  backends were inert and DDS was localhost-only domain 219. Coverage includes
  mock START/STOP, 300 Hz inputs, missing-Runner and split recovery, paused/lost
  workers, pending emergency replies, shutdown draining and idle-only reconnect.
- macOS ROS-free recovery/network/transport/deployment/command-proxy/calibration
  and process-discovery checks: **126 passed, 1 skipped, 155 subtests passed**.
  Updated asset checks also pass against main's bilingual Console, verifying
  translation-key wiring and both catalogs. The deployment-model assertion now
  checks the actual deployment section, preserving unrelated training history.

Earlier differential verification covered **868 replay scenarios** on Linux
ARM64 and a macOS ARM64 driver using real fmt; exact integer and formatted-text
assertions remain. The historical CPU comparison above is not an A3 saving
measurement. Two stopped-system HDU observations measured native parent plus
worker at 30.47% and 34.06% of one CPU core, without a matched Python baseline.
The live owner remained READY/STOPPED, with one operations node and fresh P2
eligibility. Production processes/restart counts and the historical Runner
request count were unchanged; temporary test files were removed afterward.
Installed orchestration/model helpers differ from current main, so a future
activation must use a reviewed matched bundle rather than individual old files.
No production service was restarted or physical motion/E-stop qualification
performed. Merge/test success is not a full live-hardware qualification.

## Installation and qualification

Stage this checkout's entire `foxglove/a3/` folder, including `native/`. Preserve
the existing configuration, state directory, service name and venue drop-ins.
Install the matching `hope_bringup` workspace first (calibration helpers/data
and the explicit Fast DDS peer wrapper). The [prerequisite installation commands](../../../docs/operations/foxglove_lifecycle.md#bridge-monitor-and-clock-coordinator-prerequisites)
cover monitor/bridge builds, control-bridge parameters, clock-coordinator
script/units and the post-activation enable/start sequence. On the HDU, after sourcing its vendor
ROS prefix:

```bash
bash ~/foxglove_a3/install_native_operations.sh --robot-user agi --robot-group agi
```

The installer builds only the operations executable and **stages** a complete,
checksummed candidate under `/usr/local/lib/hope-operations/staged/candidate-*`.
It changes no active unit, binary, shared Python module, systemd loaded state,
or boot enablement. A crash/reconnect, calibration restore, or reboot therefore
continues using the previous deployment. There is no staging `--enable` option.
Keep the printed candidate path for the separate activation step. The old
observer/proxy units remain compatibility shims; do not add `Conflicts=` against
shims that require the canonical owner. Inspect the effective unit/drop-ins
before an attended switch; old `ExecStart` overrides can still select Python.
Older split-layout installations may instead have real Python processes behind
`hope-observer.service` and `hope-command-proxy.service`. Activation requires
each legacy unit to be absent, explicitly **disabled**, **persistently masked**, or an effective
`Type=oneshot` `/usr/bin/true` compatibility stub without executable hooks.
Merely stopping an enabled legacy unit is insufficient: it could duplicate
services/topics after reboot. The check uses systemd's effective commands,
including drop-ins, and does not trust a stub with pending unit-file changes.
In the attended maintenance window, inspect and disable real legacy owners or
migrate them to the repository compatibility stubs; reload systemd after unit
edits, then retry. Staging/activation does not disable or rewrite those units.
Disabled real services must remain disabled and must not be started manually
or by other dependencies alongside the native owner; disabled is not masked.
Stopped persistent masks are accepted and left intact. A runtime-only mask is
refused with a reboot-risk explanation because it disappears on reboot; a
masked canonical supervisor requires explicit operator review/unmasking before
activation, rather than silently overwriting its mask. A mask never excuses
an active process or unverified pending unit changes.

During a supported/stopped maintenance window with an operator at the verified
wired E-stop, stop the canonical owner and wait for accepted cleanup to finish.
Do not force-kill it. Then activate the staged bundle explicitly:

```bash
sudo systemctl stop hope-lifecycle-supervisor.service
sudo python3 ~/foxglove_a3/hope_operations_deploy.py activate \
  /usr/local/lib/hope-operations/staged/candidate-REPLACE_WITH_PRINTED_ID \
  --confirm-supported-stopped-wired-estop
```

Activation refuses running/transitioning owners and a held hardware-operation
lock, backs up the previous unit, executable and every replaced module under
`/var/backups/hope-operations/activation-*`, then installs the matched candidate
and reloads systemd. An installation exception restores that backup. Existing
drop-ins, configuration and enablement are left unchanged. Activation does not
start a service; the confirmation flag is the operator's physical safety
attestation, not an automatic inference that the robot is supported.
This is a migration tool for an initialized HOPE installation. If the existing
`/var/lib/hope-lifecycle/hardware-operation.lock` is missing (for example, the
baseline supervisor has never run), activation/rollback explains the missing
initialization and refuses to create or bypass the interlock. A genuinely new
HDU uses the documented [first-install reference-owner initialization](../../../docs/operations/foxglove_lifecycle.md#first-install-only-initialize-the-reference-owner)
in the same attended supported/stopped maintenance procedure,
then stop the owner and retry. The split-layout check applies to native
activation, not rollback; rollback still requires every owner to be stopped.

After inspecting the effective unit, start only
`hope-lifecycle-supervisor.service` in that same attended window. Confirm one
`hope_operations` ROS node, a native parent and one non-ROS worker, correct
endpoint QoS, fresh Console status, P1/P2 eligibility, START/STOP and clock
restore ownership. Test network changes while stopped and while a session is
owned, retained hardware locking through delayed STOP, command acknowledgements
and urgent-command latency under recording and 150/300 Hz input. Handheld
E-stop continuity still requires its separate hardware qualification.

For rollback, stop the canonical owner in the same supported/stopped window,
then run `sudo python3 ~/foxglove_a3/hope_operations_deploy.py rollback
/var/backups/hope-operations/activation-REPLACE_WITH_PRINTED_ID
--confirm-supported-stopped-wired-estop`. This restores the previous matched
unit and module bundle, removing only candidate files that did not previously
exist; it does not start any service. Inspect the restored unit before starting
it. Never start the reference Python node alongside the native service.
