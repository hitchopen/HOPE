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
| `INVALID` | Startup, explicit Stand/STOP reset, confirmed ball loss or sustained low rally speed; unqualified or illegal incoming flight | No |
| `RALLY` | Qualified ego +X shot; retained while visible unless measured near-stationary | No |
| `SERVE` | Ego +X serve or local Serve button intent; retained while visible or awaiting its first ball | No |
| `OPPONENT_RALLY` | Qualified -X shot, or confirmed in-table incoming return from ego RALLY/SERVE | With pre-net history or an observed return handover |
| `OPPONENT_SERVE` | Local Ready button intent, or -X shot with one observed opponent-side contact before the net | Fresh usable poses immediately when Ready-armed; qualified flight packets with known pre-net history |

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

### Conservative ego flight and incoming handover

**Ego `RALLY` and `SERVE` are retained until
confirmed loss of the tracked ball, with one measured low-speed exception for
`RALLY` described below**. A predicted miss, net collision, additional bounce or
visible excursion outside/below the table does not invalidate these outgoing
states. A visible held ego serve remains `SERVE`. Both outgoing states suppress
the A3-bound ball channels. This replaces the earlier rule that invalidated an
ego return on its second opponent-side bounce.

Disappearance means more than 100 ms without a fresh, finite, advancing position
sample. A missing pose, NaN/Inf position, duplicate, stale source timestamp, or
rejected discontinuity does not refresh that timer; one such exposure alone does
not invalidate ego play. A source gap over 100 ms also ends the old track. Once
lost, history/fit/retries are cleared. Invalid orientation alone is not ball loss.
A locally armed ego serve awaiting its first tracked ball has no disappearance to
verify and remains armed; explicit local Stand/Stop still resets it immediately.

Without an intervening confirmed hold, `RALLY` or `SERVE` changes to
`OPPONENT_RALLY` when observed -X velocity exceeds
0.25 m/s and remains direction-confirmed for 20 ms, with the current ball centre
inside X=[0,2.74], Y=[-1.525,0], Z>=0.02 m. No outgoing bounce proof is required.
A not-yet-launched UI serve must also show free-flight curvature; translating a
held ball is not a return. The check runs after P1/P2
conversion, so both physical placements have identical semantics. The plane is
not rotated again.

The handover starts a new generation and retains only samples at/after the
observed X turnaround. Fits, spin estimation and queued retries from the outgoing
shot are cleared. An observed return can admit fresh ball poses even if it is
first confirmed on our half; `return_confirmed=true` records this evidence while
`history_known=false` preserves genuinely missing pre-net history. It does not
invent a net crossing or a bounce. Flight-packet production still requires the
existing trajectory estimator's qualification; early pose forwarding is separate.

**A confirmed hold ends automatic handover from an ego serve.** A visible,
launched `SERVE` that is caught/held keeps its `SERVE` label, clears the old
shot history/fit/retries, and reports `serve_held_waiting_for_local_intent`.
A subsequent opponent launch cannot automatically become `OPPONENT_SERVE` or
be treated as a return of that old serve. Both ball channels remain closed.
The operator may press **Ready directly** to initialize `OPPONENT_SERVE`; an
intervening Stand is not required for this explicit intent. Stand/Stop instead
sets `INVALID`, and Serve initializes another ego serve. These buttons start
clean histories. A real input outage still sets `INVALID`, but does not release
the held-serve block without local intent. Source-clock recovery also preserves it.

Without a confirmed hold, an observed in-table opponent return still changes
`SERVE` to `OPPONENT_RALLY`, independently of Runner mode. The conservative
handover does not require complete outgoing bounce proof. A held ball before
the first ego launch keeps the armed serve intent; carrying it does not qualify
as a launch. The rally-only low-speed rule is unchanged.

### Measured near-stationary rally

`RALLY` and `OPPONENT_RALLY` become `INVALID` when measured **3D speed remains
strictly below 0.10 m/s (10 cm/s) for 100 ms**, after a complete 40 ms measurement
window. `SERVE` and `OPPONENT_SERVE` do not use this new rule; their existing
loss/contact/held-ball rules are unchanged. The 0.25 m/s X direction deadband is
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
sustained low speed; the independent 100 ms real-input-loss check still applies.
Rejection reports `rally_speed_below_threshold` in the local Console, increments
the shot generation and clears the shot history, candidate samples, fit/spin and
queued retries. Continuous slow data cannot rearm the shot; explicit local button intent
or a fresh observed launch establishes a new trajectory. Pelvis messages remain
outside this gate, and no classifier state is sent to A3.

### Incoming serve, return and miss

Incoming validation remains active. `OPPONENT_SERVE -> RALLY` requires known
serve history, one observed opponent-side bounce, a legal net crossing, one
observed own-side bounce, and a confirmed +X racket-height reversal on our half.
Contacts are recorded before this reversal. A table-induced reversal is not an
A3 racket hit. The new outgoing RALLY then follows the conservative ego policy.

An incoming serve becomes `INVALID` on a wrong contact order, an extra bounce,
net collision, lateral exit, missed table/return or lost input. Incoming rallies
allow at most one receiving-side bounce; a second remains invalid even after
crossing the robot hit plane. An observed return whose pre-net segment is unknown
counts real receiving-side contacts without pretending the net crossing was seen.
A confirmed incoming rules violation cannot be resurrected by a late swing.
New local Ready/Serve intent or a qualified new observed launch can establish a new point. A previously
held ego serve specifically requires explicit local intent as described above.

Qualified incoming fits use the shared drag/Magnus/contact equations to report
**forecast advisories only in v1**. No predicted violation can change the state,
forwarding, generation, retained samples, observed contact counts or queued
retransmissions. This also applies to repeated agreeing forecasts and forecasts
with a spin estimate: this implementation does not establish a bound on every
relevant source of uncertainty. Observed contacts/violations, real-input loss and
explicit local Stand/Stop and local input resets remain the validity authority.

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

An invalid/new shot clears the fit, incoming packet window and queued retries.
The rejected-shot direction is retained separately; another negative-X sample
does not re-arm that shot. A new qualified return away from the net/table, a
held-to-launch boundary, or an explicit local Ready/Serve intent can establish a clean shot.
Without Ready initialization, first observation after the net remains provisional
and is not forwarded. Ready initialization permits early pose forwarding but
does not invent missing contacts or authorize an incomplete flight packet.
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

While Ready-armed, the first fresh, finite, above-floor ball pose can reach A3
without waiting for the 60 ms launch fit or a bounce. Duplicate/stale/invalid
exposures, held balls and known outgoing motion are not forwarded. Flight packets
still require the normal fit and history; Ready invents no velocity, spin, bounce
or target. Once the incoming launch qualifies, normal serve-route validation
and the 100 ms real-ball-loss watchdog apply. Waiting for the first serve does
not time out due to absent robot feedback. A local Serve similarly waits for
its first ball and then tracks continuously, regardless of Runner mode.

**The state, its initialization and display stay on the operator computer.**
No ball-state DDS topic is created and no state enum is appended to A3 packets.
The Console reads `http://127.0.0.1:8768/ball-state` directly. Schema-2 JSON includes
producer identity, applied intent revision, local-stop flag, state, reason,
forecast advisory, generation, forwarding, history/return evidence, arming flags
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
| Classifier | Five states at 150/300 Hz and mirrored P1/P2; contacts, handovers, invalid incoming flights, conservative ego retention, unknown spin, low-speed exception, source-clock recovery |
| Local intent | Ready before the first ball; unusable/duplicate rejection; Stand/Stop holds closed despite qualifying incoming data; Ready/Serve explicitly release it |
| Independence | Real ROS replays at 150/300 Hz, fixed/adaptive schedulers, robot modes 0/1/3/5/6, boot changes, repeated terminal serve states and lifecycle STOPPED/RUNNING; no subscriptions to these topics and no new robot-control/state publisher |
| Held ball | Ego serve → one-second hold blocks automatic next opponent serve; explicit local Ready directly permits the new serve and both channels; no old-shot samples; real loss does not authorize restart |
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
   serves require explicit Ready before a new opponent serve can pass.
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
