# Laptop ball states and A3-bound ball gate

The Foxglove computer's C++ `hope_ball_flight_packetizer` classifies ball motion
and owns both A3-bound ball channels. The same executable runs on Ubuntu under
ROS 2 and on macOS in the existing Ubuntu/Jazzy ARM64 operator container.
No estimator, service, protocol, bridge allowlist or binary on HDU/MDU is changed
by this feature. [PR #85](https://github.com/hitchopen/HOPE/pull/85) is a design reference; its integrated Planner, CKF,
spin-readiness changes and Runner changes are not imported.

## Coordinate and transport contract

```text
OptiTrack / ChingMu acquisition
  -> existing unit, world-frame and P1/P2 coordinate conversion
  -> /poses/planner_full (local original exposures, ball first)
  -> C++ five-state classifier + packet gate
       -> /ball/flight_packet -> unchanged HDU Planner
       -> /poses             -> unchanged HDU observer (ball only, <= 150 Hz)
       -> localhost:8768/ball-state (HTTP status + local button intents) <-> local Console
```

The classifier never subscribes to raw NatNet/VRPN coordinates. In its selected
player's table frame, our end is X=0, the opponent's end is X=2.74 m, net X=1.37 m,
Y is [-1.525,0] m and the table surface is Z=0. P2 conversion has already applied
`x'=2.74-x`, `y'=-1.525-y`, `z'=z`, including the existing quaternion rotation.
**Positive X is outgoing and negative X incoming at either physical table end.**
The classifier does not rotate a second time or use pelvis coordinates.

Both production launch files move the acquisition relay's diagnostic PoseArray
to `/poses/operator_raw` and PointStamped to `/ball/operator_raw_point`. This
prevents the old `/poses` observer path bypassing the gate. Raw acquisition,
full-rate normalized observations, and diagnostics remain available locally.
Robot poses, player-assignment evidence, heartbeats, E-stop and service traffic
are not gated. The browser is a display/control client, not the data-plane gate;
classification and suppression continue when the panel is not rendering.

**Pelvis localization bypasses the ball gate entirely.** The OptiTrack relay
publishes the selected body's pose in its acquisition callback before ball
output. ChingMu publishes its selected-body observation directly to the existing
base-pose relay. That relay immediately publishes the calibrated pelvis/base
wire; neither path waits for ball classification, launch fitting, a bounce, or
the ball output rate limiter. Existing pose validity/calibration checks remain.
No extra timer, queue or delay is added to `/a3/base_pose_laptop_flat` (or the
configured base topic), and the HDU base transport is unchanged.

## Five states

| State | Evidence / route | Send ball data to A3? |
|---|---|---|
| `INVALID` | Startup, explicit Stand/STOP reset, observed held/near-stationary ball or an observed flight violation | No |
| `RALLY` | Qualified ego +X shot | No |
| `SERVE` | Ego +X serve or local Serve button intent; retained across observation loss or while awaiting its first ball | No |
| `OPPONENT_RALLY` | Qualified -X shot, or confirmed in-table incoming return from ego RALLY/SERVE | Fresh qualified incoming arc; missing pre-net history is not a veto |
| `OPPONENT_SERVE` | Local Ready button intent, or -X shot with one observed opponent-side contact before the net | Fresh qualified incoming arc. Ready alone opens neither channel |

The ball classifier and Runner state machines are independent. **No Runner
mode, serve phase, boot, heartbeat, action acknowledgement or lifecycle feedback
is subscribed to by the classifier.** A3's automatic PD_STAND, SERVE follow-through,
faults, restart or MOTION transition cannot reset a local ball track or change
its eligibility. Eligible incoming ball data can therefore reach A3 while its
Runner is standing or completing a serve; the unchanged A3 controls decide
whether to use those data. The classifier never sends a state enum or mode command.

An explicit **local Stand/Stop button press** sets `INVALID`, clears history,
fits and pending retries, and holds the ball gate closed until another local
Ready or Serve press. Passive and E-stop button presses also set this local
stop intent, independently of their existing robot requests. Pelvis/base traffic
continues throughout. A3 feedback cannot release or assert this local stop.

Unknown match context classifies the **observed flight pattern**, not official
match scoring. A first striker-side bounce can reclassify rally to serve without
changing shot generation or dropping samples. Contacts are deduplicated by source
timestamp. This feature does not generate hit targets, arrival times or motion
decisions.

### Observation gaps and new shots

**Logical ball state and observation availability are independent.** Loss,
late visibility, missing pre-net history, bad coordinates and source timestamp
faults do not prove an invalid ball. All four established flight labels survive
these conditions. `observation_missing=true` separately pauses forwarding.
The local Console retains the state title and displays the missing observation.

A gap over 20 ms clears fitting/contact windows; 100 ms without an accepted
exposure pauses output. Cached fits and pending retries are cancelled on the
first pause. Another gap during reacquisition discards that partial window too.
Neither timeout sets `INVALID`. Fresh direction and ballistic evidence (60 ms)
reacquires automatically, without another Ready/Serve press. Contacts across a
gap are not inferred, and old samples cannot enter the new fit. Invalid
orientation alone does not reject valid XYZ. Duplicate or sub-1-ms source
exposures are dropped without renewing freshness or poisoning the logical shot.
The 35 m/s discontinuity threshold is unchanged: an outlier pauses observation,
it does not declare the ball illegal.

The observed state and substate rules come from the
[HOPE Score Board classifier](https://github.com/hitchopen/hope-scoreboard/blob/aab5349971921f2d92bdc98ae0bdbd0010112d81/tracking/laptop_play_state.cpp)
at source commit `aab5349971921f2d92bdc98ae0bdbd0010112d81`.
When the first usable observations are all on the receiving half, a confirmed
direction and a 35 ms ballistic arc can qualify a rally. Missing pre-net
history stays unknown. A return starts a new generation at the observed X
turnaround. The old fit, spin history and pending retries are cleared.

A held serve ends in `INVALID · Ball stopped`. A fresh ballistic launch can
establish the next flight without Ready or an ego serve. Between points, a
separate track after a gap or discontinuity, or sustained non-flight followed
by a new launch, can re-arm reception. A same-point rebound or late swing
cannot reopen the rejected trajectory on its own. Local Stop requires an
explicit Ready or Serve.

### Measured near-stationary rally

`RALLY` and `OPPONENT_RALLY` become `INVALID` when measured **3D speed remains
strictly below 0.10 m/s (10 cm/s) for 100 ms**, after a complete 40 ms measurement
window. `SERVE` and `OPPONENT_SERVE` do not use this new rule; their existing
contact/held-ball rules still apply. The 0.25 m/s X direction deadband is
separate and is not a minimum total flight speed.

There is no positive universal minimum instantaneous speed: a nearly vertical
ball can momentarily approach zero at its apex. Using the gravity approximation
`vz = vz0 - g*t`, a vertical ball spends about `2*0.10/9.81 = 20.4 ms` within
this speed band. The 100 ms confirmation therefore avoids rejecting that brief
apex. This is a conservative operational detector of near-stationary motion,
not a claimed physical limit or a minimum legal shot speed. The derivation uses
the [NASA ballistic-flight equations](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ballistic-flight-equations/);
actual venue noise and trajectories still need qualification.

The implementation uses fresh, timestamped XYZ observations after P1/P2
conversion, without quaternion/spin or forecast assumptions. In each full 40 ms
window (including the preceding exposure when sampling is irregular), it takes
the **largest** `norm(p[j]-p[i]) / (t[j]-t[i])` among pairs at
least 20 ms apart. This measures all three axes and avoids relying on one net
displacement across a reversal. Noise can defer low-speed rejection rather than
make an uncertain fit decisive. At 150/300 Hz, a clean speed drop is detected
about 140--153 ms later, including window warm-up and sample quantization.

A speed at/above the threshold, an unusable exposure, an exposure gap over
20 ms, a confirmed contact, or a new shot resets confirmation. Duplicates,
reordered packets and timer ticks cannot advance it. Sparse data cannot prove
sustained low speed; the independent 100 ms observation pause still applies.
Rejection reports `rally_speed_below_threshold` in the local Console, increments
the shot generation and clears the shot history, candidate samples, fit/spin and
queued retries. Continuous slow data cannot rearm the shot; explicit local button intent
or a fresh observed launch establishes a new trajectory. Pelvis messages remain
outside this gate, and no classifier state is sent to A3.

### Incoming serve, return and miss

A confirmed reversal near the net without reaching the receiving half marks a
net rebound. A table-height miss, a return before the required receiving bounce,
or two distinct contacts on one half end the shot. A second contact must show
separation above the first contact and sustained descending and rising motion.
The classifier uses the Score Board's ChingMu noise thresholds to filter
duplicate contacts. A tracking gap clears the fit while preserving confirmed
receiving contact evidence when the same shot is reacquired.

After a confirmed receiving bounce, a low ball has a 350 ms window for an
observed return. Both A3 ball channels stay closed during this window. A return
must reverse on the receiver's side and rise above table height. If no return
qualifies before the window expires, the shot becomes `INVALID`.

Qualified incoming fits use the shared drag/Magnus/contact equations to report
**forecast advisories only in v1**. No predicted violation can change the state,
forwarding, generation, retained samples, observed contact counts or queued
retransmissions. This also applies to repeated agreeing forecasts and forecasts
with a spin estimate: this implementation does not establish a bound on every
relevant source of uncertainty. Observed contacts/violations and explicit local Stand/Stop remain the validity
authority; observation loss only pauses usable data.

Missing spin uses a position-only hypothesis without Magnus force; that is not
proof of measured zero spin. A heavy topspin ball may land legally even when
this hypothesis predicts it will fly long. Warnings such as missing a bounce
before the hit plane, a predicted net collision, floor landing, side exit or extra
bounce are recorded as `forecast_advisory` in the local display. They never
permanently discard the incoming ball. The original observed-event `reason`
remains separate. The advisory is cleared by a new shot/reset, an observed
contact, or the next forecast if no current violation can be evaluated/found.

Plane-dependent forecasts require a fresh `/hope/planner/hit_plane`; no nominal
pelvis position is invented. Outgoing forecasts remain inactive. Future rejection
based on predictions would require a separately reviewed uncertainty contract;
consecutive agreement alone is not treated as proof. This does not change A3's
Planner, the flight-packet estimator/scheduler, or their target decisions.

An invalid or new shot clears the fit, incoming packet window and queued retries.
A qualified post-net arc can open pose forwarding without Ready. Flight
packets also need the existing scheduler and estimator qualification.
Out-of-order/duplicate timestamps cannot refresh the input watchdog. Source
exposures older than 100 ms or over 50 ms in the future close the gate; reception
of queued old packets cannot make a stale ball fresh. A vertical
apex is not a held ball. Missing orientation alone never rejects valid XYZ.

### Source-clock recovery

Duplicates and isolated reordered exposures are ignored without refreshing the
ball watchdog or entering a fit. A sustained backward clock step can establish
a new timestamp epoch without a lifecycle restart, but only after the old input
has expired and lower timestamps advance for at least 60 ms in both source time
and monotonic receipt time. Non-advancing samples, gaps over the input watchdog,
or a source/receipt step discrepancy over 20 ms restart that confirmation.
A burst of old packets is not clock-recovery evidence.

Recovery reports `source_clock_rebased`, starts a new generation and discards
old history, candidates, fit/spin and retries. Forecast and pose-rate timestamps
are also reset so they cannot retain the old high-water mark. The new stream
must qualify a new observed flight; clock recovery does not synthesize Ready or
Serve intent or release a local Stop. A stale/future hit-plane reference is
cleared independently.

Every exposure still passes the ROS boundary's absolute age checks: at most
100 ms old and 50 ms ahead of the operator computer's wall clock. If the mocap
clock remains outside that range after a step, synchronise the clocks first;
auto-recovery does not reinterpret stale timestamps as current observations.

## Local buttons and display

A confirmed Console button press has two independent paths: its existing robot
service request, and a local classifier intent. The classifier does not wait for
A3 acceptance and never rolls back its intent on a rejected/timed-out robot request.
It has no interaction with controls issued from other robot clients.

| Console button | Local classifier effect |
|---|---|
| Receiver Ready | `OPPONENT_SERVE`, armed before any ball observation |
| Prepare Serve / Ready to Serve | `SERVE`, armed before launch; repeating while still armed preserves the track |
| Stand / Stop (Kill All and Collect) / Passive / E-stop | `INVALID`, clear histories/retries and hold closed until explicit local Ready/Serve |

While Ready-armed, both channels wait for a qualified incoming arc. Ready sets
the waiting label without adding velocity, spin or contact evidence. The
classifier starts ordinary observed-flight validation when the launch qualifies.
A local Serve similarly waits for its first qualified launch.

**The state, its initialization and display stay on the operator computer.**
No ball-state DDS topic is created and no state enum is appended to A3 packets.
The Console reads `http://127.0.0.1:8768/ball-state` directly. Schema-2 JSON includes
producer identity, applied intent revision, local-stop flag, state, substate, reason,
forecast advisory, generation, forwarding, `observation_missing`, history/return evidence, arming flags
and contact counts. Decimal strings preserve 64-bit identifiers. Session ID is
packet audit metadata, not a match against robot lifecycle state.

Local initialization uses body-free POST `/ball-state/intent/READY`, `/SERVE`
or `/STOP`, with the current producer identity and intent revision in headers.
The server rejects stale producers, duplicate/reordered revisions and overflow;
a bounded queue is applied on the packetizer timer. Console button intents are
serialized locally and confirmed by the applied revision. Conflicts are shown
for explicit retry, never automatically retried against a newer Stop. Endpoint
failure is visible separately and cannot block the existing robot service request.
This endpoint only initializes the laptop classifier; it cannot command hardware.

The server checks Origin before reading state or queuing an intent. It permits
exactly `https://app.foxglove.dev` for the web Console, rejects other web origins
with HTTP 403, and returns an exact CORS origin plus `Vary: Origin`; no wildcard
is used. Missing Origin is permitted only for read-only status inspection (e.g.
local `curl`), never for button requests. Duplicate Origin headers are rejected.

**Foxglove Desktop pairing:** the installed Desktop renderer uses `file://`,
whose Origin is `null`. This is not a unique identity: hostile sandboxed pages
can also have a null origin. Desktop GET/POST therefore additionally require a
private 256-bit key in `X-Hope-Local-Key`. An OPTIONS response never authorizes
a subsequent request without the key. The key is never returned by HTTP, placed
in a URL, logged, stored in a layout, or published on ROS. Unknown web origins
remain forbidden even with a key. This protects against arbitrary browser pages,
not processes running as the operator account or other trusted Foxglove extensions.

At packetizer startup, the key is created once as an owned mode-0600 regular
file at `$ROS_HOME/hope-ball-controls.key` (default
`~/.ros/hope-ball-controls.key`). It persists across runtime restarts. Symlinks,
unsafe permissions and malformed keys disable Desktop access rather than falling
back to an unauthenticated endpoint. The `play_state_control_key_path` parameter
can change the file path, not carry the key itself. Import that private file using
**Pair local ball controls** in the Console footer. The Console stores it only
in its local application storage. A live local ball-state display confirms the
pairing. Import once per operator runtime/Console profile; rotating the key or
clearing app storage requires reimporting. Do not share the file.

For native Ubuntu, select the runtime user's `~/.ros/hope-ball-controls.key`
in the file picker. For the Mac Docker runtime, after its first launch, export
the key from the configured container to a private local file (this reads only
the operator container and does not contact A3):

```bash
python3 - <<'PYKEY'
import json, os, subprocess
from pathlib import Path
config_dir = Path.home() / ".config/hope-macos"
config = json.loads((config_dir / "operator.json").read_text())
key = config_dir / "hope-ball-controls.key"
# Reserve the local copy privately before docker cp writes its contents.
fd = os.open(key, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW, 0o600)
os.fchmod(fd, 0o600)
os.close(fd)
subprocess.run(["docker", "cp",
    f"{config['container']}:/home/{config['laptop_user']}/.ros/hope-ball-controls.key",
    str(key)], check=True)
key.chmod(0o600)
print("Import this private file into the Console:", key)
PYKEY
```

Bounded HTTP I/O runs outside ROS callbacks. A stalled UI cannot delay ball or
pelvis traffic. Status JSON is rebuilt only when fields change, with a 10 Hz
cached heartbeat and a one-second producer expiry. The Console polls at 10 Hz
and displays current local state without matching Runner mode, boot or session.
Ubuntu binds loopback. The Mac container publishes only host loopback
`127.0.0.1:8768:8768`. Use Foxglove Desktop on the operator computer; remote
viewers cannot access this feed. HTTP/private-network restrictions can disable
both local initialization and display, with a visible error; they do not silently
fall back to A3 or reset the classifier.

## Build and install on the operator computer

Do this while the operator runtime is stopped; do not run any HDU/MDU installer.

**Ubuntu ROS 2 Jazzy client** (same commands work with the repository's installed
ROS distro after sourcing its environment):

```bash
cd ~/HOPE/hope_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to hope_planner_cpp hope_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select hope_planner_cpp --return-code-on-test-failure
colcon test-result --verbose
cd ../foxglove/extensions/hope-a3-console
npm ci
npm run typecheck
npm test
npm run build
npm run local-install
```

**macOS client** (the supported runtime is Linux in Docker, not native macOS
ROS):

```bash
cd /path/to/HOPE
python3 scripts/macos/hope.py build-operator --arch arm64 --jobs 2
cd foxglove/extensions/hope-a3-console
npm ci
npm run typecheck
npm test
npm run build
npm run local-install
```

Rebuilding the configured operator workspace updates the executable for the
next laptop runtime launch. Follow the [Mac deployment guide](macos_deployment.md)
for the configured storage and container lifecycle. Keep the existing
Ubuntu/Mac operator profile, SSH route and DDS peers. Do not run `operator.py bind`, A3 installers, or modify
HDU bridge parameters for this update. Mac container contract 4 adds the local
status port. Existing contract-3 containers are not silently reused: while the
operator runtime is stopped, run `operator.py down`, remove only the configured
operator container (preserve its volumes), rebuild, then use the documented
`operator.py up` procedure. This recreates local transport, not robot software.
The installer refuses a missing or non-loopback status-port mapping. Reload the
Console extension in Foxglove. A direct container/manual packetizer launch must
set `play_state_status_bind:=0.0.0.0` and publish that port on host loopback;
native Ubuntu uses the default `127.0.0.1` bind. Verify locally with
`curl --fail http://127.0.0.1:8768/ball-state`.
Both `avatar_pro_hope_bridge.launch.py` and `optitrack_hope_bridge.launch.py`
enable the gate when their packetizer starts. `start_flight_packetizer:=false`
now leaves diagnostic acquisition available but sends neither ball channel.

Runtime thresholds are named `play_state_*` parameters in
`model21800_flight_packetizer.yaml` (the historical filename is still used by
the current profile): 100 ms input gap, 0.25 m/s X deadband, 20 ms direction
confirmation, 60 ms launch evidence, 180 ms held-ball window and 45 mm maximum
observed contact-center height. `play_state_rally_min_speed_mps: 0.10` and
`play_state_rally_slow_confirm_s: 0.100` configure the separate measured rally
rule. Configuration permits a lower positive speed threshold (at most 0.10 m/s)
or a longer confirmation (100--1000 ms), so tuning cannot silently make this
rule more aggressive than the reviewed defaults. Launch admission uses a
low-residual downward curvature fit; these thresholds require captured venue trajectories to qualify
very strong lift/spin, edge contacts and occlusion reacquisition.

## Verification and limits

Maintained regression suites cover:

| Area | Regression coverage |
|---|---|
| Classifier | Five states at 150/300 Hz and mirrored P1/P2; contacts, handovers, observed violations, short post-net acquisition, unknown spin, low-speed exception, source-clock recovery |
| Local intent | Ready waits for launch qualification; unusable/duplicate rejection; Stand/Stop holds closed despite qualifying incoming data; Ready/Serve explicitly release it |
| Independence | Real ROS replays at 150/300 Hz, fixed/adaptive schedulers, robot modes 0/1/3/5/6, boot changes, repeated terminal serve states and lifecycle STOPPED/RUNNING; no subscriptions to these topics and no new robot-control/state publisher |
| Held ball | Ego serve → one-second hold → fresh ballistic launch with or without Ready; both incoming channels recover without old-shot samples; hand translation does not qualify |
| Observation loss | Labels survive missing exposures and clock faults; forwarding pauses, fits/retries clear, and fresh ballistic observations recover automatically; local Stop remains latched |
| Recorded input | Captured launch with timestamp bursts remains eligible; captured second receiver bounce still rejects with an explicit reason |
| Local HTTP / Console | Producer/revision matching, stale/conflicting/duplicate requests, bounded queue, origin rejection, Desktop key checks/persistence, stalled reader, freshness, local button mapping and confirmation independent of robot acknowledgement |
| Ball transport | Invalid/ego suppression, incoming packets, immutable retransmissions, cancellation across invalidation and source epoch changes |
| Acquisition relays | Existing P1/P2/orientation conversion and immediate pelvis callback bypass |

Build and run the focused `test_laptop_play_state`, `test_laptop_state_gate`,
`test_flight_packet_pipeline` and `test_local_play_state_status` targets, plus
Console tests/typecheck and operator/relay contract tests before packaging.
Keep exact revisions, result counts and install receipts in PR validation,
not machine-specific addresses or temporary paths in this guide.
Synthetic replay does not qualify live competition behavior.

Before venue deployment:

1. Replay recorded OptiTrack and ChingMu tracks on the intended Ubuntu or Mac
   runtime. Compare P1/P2 and 150/300 Hz behavior, unknown spin, contact detection,
   apexes, low-speed rejection and reacquisition.
2. Exercise local Ready/Serve/Stand/Stop and local endpoint failure. Verify robot
   feedback, command rejection and automatic mode changes never alter the local
   classifier; a local Stop persists until explicit Ready/Serve. Confirm held ego
   serves admit a fresh ballistic launch with or without Ready while hand-carried
   motion remains blocked.
3. Confirm observation and both eligible incoming channels continue during A3
   serve follow-through, while local ego flights stay suppressed. Verify pelvis,
   heartbeats and E-stop traffic retain their independent paths.
4. Exercise ball loss and source-clock discontinuity in isolated replay: clear
   old fits/retries, require a new flight, preserve local Stop and reject genuinely
   stale timestamps. Do not step an in-play robot's clock for this test.
5. Check the intended production operator-to-HDU route. Local synthetic tests do
   not qualify Wi-Fi/DDS, cameras, robot timing or A3's use of already sent data.

**A laptop gate cannot recall a target already delivered to the unchanged A3
Planner/Runner.** It suppresses future messages and queued retransmissions.
Target revocation/acknowledgement would require a separate A3-side change and is
outside this laptop-only implementation. Keep existing independent hardware
safety controls and motion admission unchanged.

For rollback, stop the operator runtime and rebuild the last qualified revision
of both packetizer and bringup together, then install its matching Console
package. Use the operator deployment procedure and recorded installation receipt
to locate local backups; do not restore or change robot-side services for this
laptop-only update.
