# Flight state and prediction v2

This branch implements the refined 19 October handoff over fetched public HOPE
nightly_built, with PR49's six additive scaffold commits restored. V2 is selected
explicitly with `flight_prediction_v2_shadow.launch.py`. It cannot publish outside
`/hope/shadow/flight_v2/`, including through ROS remapping. It starts INVALID and
never launches a robot runner. Existing deployed YAML files are unchanged.

## Ownership and classification

`ShotContext` owns source-stamped observations, a separate bounded candidate queue,
physical-shot generation, semantic context revision, contact/event ledger and
trusted spin. `FlightPipeline` coordinates that context with the existing six-state
CKF. The existing correlated measurement-bias nuisance moments remain unchanged;
there is no new nine-state ball estimator or stochastic ensemble.

The five v2 public states are INVALID, OPPONENT_RALLY, RALLY, OPPONENT_SERVE and
SERVE. Legacy PlannerBallState and the old shadow topics are compatibility
interfaces, not v2 state names. A qualified racket/free-flight/restart boundary
clears the old posterior, observations, contacts and spin. Same-direction contacts
also start a new generation. A direction reversal alone does not establish racket
contact. INVALID latches out the rejected physical flight: acquisition cannot rearm
it without a new qualified boundary or explicit correction/restart.

A provisional +X/-X rally changes to the matching serve on its first confirmed
striker-side contact before the net. The observation queue, original timestamps,
event identity, spin lineage and shot generation survive. Semantic revision changes
immediately fence old solves. NORMAL_RALLY context instead forbids that contact;
SERVE context constrains the ordered two-side route. Incomplete acquisition does
not prove zero earlier bounces. Match type alone also cannot certify missing
history. A receiver-side bounce after incomplete acquisition remains provisional,
rather than proving that the net/striker bounce never happened. Trusted same-shot
historical events may precede an unqualified acquisition timestamp; they cannot
precede a qualified physical boundary. Packets preserve that distinction. Positive
forbidden ordering/count evidence still rejects immediately. Verified history
reclassification preserves the current samples and generation. Physical forecasts
remain available while route targets wait for supported history. An armed service toss cannot autoqualify.

Automatic acquisition requires at least six source observations over the configured
25 ms span, horizontal launch speed at least 1 m/s and acceleration consistent with
ballistic motion within the configured 8 m/s² residual. These are provisional
engineering thresholds. Qualified boundaries and explicit history-continuity evidence
are distinct; automatic acquisition always declares unknown prehistory. Held-ball rejection uses
elapsed time and spatial dispersion; moving non-ballistic rejection excludes
observed/reversing contact windows. A vertical-velocity root records an apex without
calling it held motion.

Observed contacts use separate occurrence and confirmation times. Forecast contacts
never increment observed counts or mutate the authoritative spin checkpoint. Time
and geometry association retain event IDs on confirmation; ambiguous associations
remain explicit. Superseded hypotheses are not observations. Confirmed route events
survive estimator-window eviction. Retired hypotheses are bounded separately.

## Model, spin and uncertainty

`common_ball_physics` supplies RK4 free flight, the thin-shell capped-slip impulse,
the position-only effective contact model and event ordering for fitting,
probabilistic propagation, nominal forecasts, receiving fallback, adaptive timing,
outgoing inverse candidates and native replay. The default coefficients come from
the unchanged 2026-10-10 calibration receipt. They are acceleration-law coefficients,
not force coefficients to divide by mass again.

MEASURED, HELD, CONTACT_PROPAGATED and POSITION_ONLY are distinct modes. Missing,
all-zero raw sentinel, nonfinite and rejected angular measurements preserve a
compatible complete trusted vector and its original measurement time. Missing
quaternions break differencing only. Reacquisition requires fresh increments;
q/-q equivalence is preserved, and increments whose possible rotation exceeds
half a revolution at the configured maximum rate are rejected. A rejected current
increment cannot refresh measurement age. A zero resulting from a trusted contact
is usable; absence of a prior never synthesizes angular state or Magnus. Event IDs
are identities, not chronological counters: historical evidence cannot suppress a
later impulse merely because it received a larger ID. Impulses before the current
measurement are already absorbed and never reapplied. A late applicable impulse
that would require undoing a later impulse is explicitly ambiguous, preserves the
cache and withholds route use until qualified evidence resolves it. Acquisition
replays each sample's available spin causally; new-shot boundaries clear model
spin as well as the trusted cache.

Batch fitting records multiple observed free-flight/contact segments, resolves
spin prospectively at source samples and retains a checkpoint outside window
trimming. It does not smear an impulse into a continuous acceleration fit. The CKF
carries a separate auxiliary model lineage; speculative propagation never confirms
a contact in the shot ledger. Packet snapshots preserve both trusted and posterior
auxiliary spin, the six-state posterior and existing bias moments. A truncated raw
suffix can be transferred without refitting it with future spin or losing history.

Free-flight Jacobians differentiate the exact RK4 stages. Contact derivatives use
the implemented event/reset/remaining-flight map; event-time covariance includes
plane/contact roots and an acceleration-based apex root. Covariance outputs are
explicitly **conditional p/v linearizations**. Auxiliary spin/model, plane and clock
uncertainty are not calibrated. Branch-sensitive contacts, event-order interval
overlap and uncertain net clearance withhold route targets. Small conditional
covariance alone never proves a hard forbidden event. The deterministic support API
requires an explicit conservative bound covering *all* relevant omissions and
separated event-order intervals; the default runtime has no such calibrated bound.
Confirmed forbidden events hard-invalidate immediately. Revocations report the
underlying lifecycle reason so a physical rejection can be distinguished from a
late publication or semantic revision.

The tabletop footprint is not enlarged by the radius. Near-edge contacts outside
the actual top return ambiguity. An initial below-top state is not clamped into a
rebound. The finite net is radius-expanded, including its ends; around-post flight
is preserved. Collisions win a quantized 1 ns numerical tie with targets. Contacts
resolve first, then remaining flight; all work/horizons are bounded. Outgoing RALLY
uses the B1–B2 apex exception, while serves have no apex exception. Legality stops
at the selected target, so a later irrelevant bounce cannot revoke it.

Post-racket angular transfer is unmodelled. Stage 3 therefore explicitly uses
POSITION_ONLY outgoing candidates. External landing inputs remain tabletop surface
points; the inverse solver adds the 20 mm radius once and verifies the first
resolved table contact and flight duration. `outgoing_route_valid` is required for
command validity; net metadata alone cannot admit an outgoing route.

## Transport and timing

New schema-2 observation, event, waypoint, prediction, context, packet and decision
messages coexist with v1 messages. They require a coordinated rebuild of their new
producers/consumers; existing wire types are unchanged. The relay emits authoritative
validity before legacy placeholder substitution. NamedPose provides no reliable
orientation-quality bit, so `optitrack_mct_relay.ball_orientation_authoritative`
defaults to false. Enable it only for a qualified orientation source; a finite
quaternion, including numeric identity, cannot qualify itself. CSV replay likewise
requires an explicit orientation-authority column. Legacy PoseArray/v1 packets
without matched authoritative validity remain position-only. A real identity
quaternion is usable only with authoritative validity. The parked Python planner
is COLCON_IGNORE; its spin helper has matching hold/reset semantics and a directly
executed regression suite.

All source/acquisition stamps are signed int64 nanoseconds, with explicit validity,
clock/mapping identity, sequence, separate receipt times and frame/ball/calibration
provenance. Zero simulation epoch is valid. Integration uses differences from an
exact integer origin. Duplicates and reordered samples are rejected. Clock
mapping loss revokes outputs; incompatible clock/frame/ball identity retires the
shot. Arrival is checked integer reference + duration, independent of publication
or receipt. Forecast events/waypoints are never admitted as measured samples.

Packet content hashes use canonical generated flow YAML with classic locale and
17-digit doubles, prefixed with the schema/hash-format identity. CDR padding is
excluded. Tests alter validity, checkpoint, sample order and hashes independently.

A running forecast owns an immutable pipeline/plane/config snapshot. The callback
continues admitting observations while one worker solves; one replaceable latest
pending snapshot bounds work. Generation/context, source age and monotone solve
sequence fence publications. Ordinary samples do not invalidate running solves.
The callback owns stable event-ID association when a result is published.

`/hope/shadow/flight_v2/prediction` carries continuous compact revisions and
revocations. `/trajectory` retains full means/covariances and optional spin at
50 Hz; `/packet` carries transferable snapshots at the same cadence. Both refer to
their original source time. `/context` accepts only the configured session and
authority. `/decision` is a planner-side shadow decision. No v2 publisher emits a
production flat command. Perception never publishes strike-start time.

## Planes and planner boundary

ON_REFRESH preserves production-profile freeze semantics: a deliberately frozen
plane does not expire merely because its source pose ages. FOLLOW_POSE snapshots
the newest qualified registered pose for each solve, derives x = base_x + 0.58 m,
and rejects stale/out-of-range poses without clamping or using an old plane. V2's
shadow profile explicitly selects FOLLOW_POSE. Geometry revisions are separate
from world/calibration identity and do not create ball/contact evidence.

The registered scalar plane assumes table +X forward. Arbitrary-heading plane
registration is not inferred from an X-only interface. Targets remain in the table
frame. Y bounds and minimum lead are planner decisions and never suppress useful
physical forecasts. Existing production incoming snapshot isolation, precommit
reversal cancellation, turn/reach guards, packet retransmission and one-shot
receiver protections remain on their existing paths and are regression tested.
The legacy selector additionally fences delayed pre-reset observations/forecasts
by the authorized reset source floor, preventing retired work from halting a new
session before its first flight. This inherited failure was reproduced against
the untouched scaffold and has a deterministic regression.

V2's shadow planner does not establish live robot feasibility or hardware
cancellation. The current receiver has identity/revision lifecycle protections but
no new cancellation protocol is invented here. An already selected shadow command
followed by revocation reports cancellation as unqualified; silence is not
cancellation. Physical frame, paddle reference, policy support and live receiver
qualification remain adoption gates, not claims established by synthetic tests.

## Provisional parameters to revisit

| Parameter | Current value/profile | Required future evidence |
|---|---|---|
| Contact Y bounds | [-1.20, -0.30] m, table frame | Robot workspace, turn/reach and team policy |
| Plane offset/range | +0.58 m; [-0.30,+0.30] m | Venue/base/paddle registration |
| Pose freshness/window | 0.20 s / 0.50 s; ten samples | Base stream and clock characterization |
| Policy height / turn / reach | +0.76 m once; 40 degrees; existing reach contract disabled | Policy origin, support envelope and robot qualification |
| Orientation authority | Relay default false | Qualified source quality semantics; cannot infer from quaternion values |
| Planner lead | 50 ms | Receiver/policy transport and actuation timing |
| Source publication age | 30 ms | Measured deployment load and clock budget |
| Table/net geometry | 2.74 × 1.525 m; net x=1.37, h=.1525 | Registered venue survey |
| Radius/gravity/inertia | .02 m, 9.81 m/s², 2/3 | Ball/profile identification |
| Drag/Magnus/contact coefficients | Existing 2026-10-10 receipt | Independent held-out flights; position-only scored separately |
| Constant free-flight spin | Prior; no fitted decay | Independent orientation/spin recordings |
| Qualification/held/contact thresholds | `ShotSettings` defaults | Timestamped varied-speed/spin/contact recordings |
| Event support | 3 sigma, 1 ms separation; conditional by default | Bounds covering spin/model/plane/clock omissions |
| Integration/horizon/full cadence | 1 ms / 1.5 s / 20 ms | Numerical and performance envelope changes |
| Context/registration IDs | fixture / UNQUALIFIED_SHADOW | Actual session authority and registration receipts |

Changing these values requires rerunning the affected numerical, transport,
availability and physical gates. Numerical agreement measures software solver
agreement, not real-ball accuracy.

The paced synthetic fixture derives analytic position from each actual acquisition
stamp. Nominal frame indices are used only for pacing/dropout scheduling: pairing
those nominal positions with delayed wall-clock stamps creates physically
inconsistent motion under scheduler pauses. The deterministic 150/300 Hz pause
regression checks source-time-consistent motion without relaxing availability.
