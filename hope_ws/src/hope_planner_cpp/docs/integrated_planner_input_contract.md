# Integrated planner contracts

Integration base: `4dc0959348f2aa777330d177d16eb70f778a79e6`. The default legacy
constructor and numerical modules remain unchanged except explicit backend
dispatch/destruction hooks. This is an application over the forward-only V2
libraries. `hope_flight_prediction_v2_node` still has no command dependencies.

## Inputs and ownership

| Input | Producer / boundary | Topic, type, QoS | Required semantics |
| --- | --- | --- | --- |
| Ball | `motion_capture_tracking` -> installed `optitrack_mct_relay` | `/hope/ball/observation_v2`, `hope_msgs/BallObservationV2` v2; best effort, volatile, depth 1 | Original acquisition Unix ns; registered frame and calibration; configured ball ID and mapping ID; finite XYZ; source age <=30 ms; clock uncertainty <=1 ms. |
| Base | Installed `hope_base_pose_flat_relay`, from `/UCB_P1/pose` | `/a3/base_pose_flat`, `std_msgs/Float64MultiArray` schema 2, 16 doubles; producer reliable depth 10, consumer best effort depth 10 | Flags include 47, finite unit-compatible quaternion, positive exact calibration/world IDs, monotonic source and sequence, age <=200 ms. Tested `input_header` timestamp mode. |
| Game context | Optional existing referee/operator process, when available | `/hope/integrated_planner/game_context`, `FlightContextV2`; reliable depth 4 | Only current-generation `CONTEXT` from configured authority, monotonic time no later than latest measurement. Diagnostic only. No START, CONTACT, RESET, or history upgrade is accepted here. |

The raw MCT topic is `/optitrack/poses` (`NamedPoseArrayV2`, best effort depth 1).
`NatNetClockUncertainty` must be present and qualified for usable ball mapping.
`ball_calibration_id` is an added relay parameter, empty by default. Set it to
the registered survey identity expected by the planner. This parameter does
not perform a transform or certify the survey. The relay's existing P1/P2
geometric transform remains authoritative. No receipt time replaces the ball
acquisition time. Missing/non-authoritative ball quaternion preserves XYZ with
POSITION_ONLY spin. The integration test begins after the camera driver and
labels all registration and clock metadata synthetic.

Host placement is configuration-specific. These tests run all software on one
Laptop in an isolated ROS domain; they do not qualify HDU/MDU network timing.
VRPN has no qualified typed observation adapter in this deliverable and is
explicitly unsupported for the integrated backend. Existing VRPN legacy mode
remains available. Base `local_receipt` mode is not qualified by the new tests;
the integrated input profile requires `input_header` and synchronized clocks.

## Coordinates and action representation

SI units; +X points toward the opponent, Z=0 is the table surface, and table Y
is [-1.525, 0]. The default receiving plane is `base_x + .58 m`, with FOLLOW_POSE
and ON_REFRESH supported. Nonzero `x_hit_bh_delta` fails startup. A calibrated
ON_REFRESH plane is frozen; independent live base freshness still gates every
command. Refresh is available through `/hope/integrated_planner/refresh_plane`.

Incoming and commanded positions are **ball-center intercepts**, matching the
actor target convention; no ball-radius face-center shift is applied to that
actor input. A geometric paddle face lies at `p_ball - R*n` under this model.
The landing objective is a surface point and becomes a descending ball-center
contact at Z=R once, inside `ValidatedReturnPlanner`.

Policy height is added once in `Schema2Packer`. If base flag 16 says that its
policy offset is already applied, the integrated runtime subtracts that offset
for physical reach calculations. The unchanged runner observation builder
then subtracts the policy-frame base from the policy-frame target. Native
tests exercise the actual 110-D builder at indices 103:106, 106:109 and 109.

The supported action subspace is `normal = normalized(racket_velocity)`, with
positive X velocity. This matches the training helper
`HopeCommand._normal_from_target_velocity` and the velocity-normal training
mode; schema 2 carries no extra normal. An actor with a different normal
convention needs a separate compatibility profile. Real achieved orientation
and tracking remain a measured simulation/physical requirement.

## Eligibility, physics and validity

`select_physical_return` uses the CKF nominal event rollout and confirmed local
events. It counts observed receiver bounces once and current predicted bounces
once, excluding predicted ledger duplicates. A single receiver-side bounce
must precede a future incoming plane event; a second receiver bounce, earlier
terminal event, missing covariance, ambiguous order, expired observation or
unqualified registration rejects the candidate. Three-sigma conditional event
intervals must be separated by 1 ms; contact time sigma <=50 ms and maximum
contact position variance <=(.15 m)^2. These are model support criteria, not
calibrated bounds on physical accuracy. A slow trajectory can remain uncertain
before its bounce and become usable after post-bounce measurements.

History UNKNOWN/INCOMPLETE, missing opponent-hit evidence, optional game context,
and game-illegal earlier opponent contacts are not gates. Missing history is
never upgraded to COMPLETE. An opt-in `ShotSettings.game_legality_advisory`
keeps physical tracking alive through game-rule violations; the standalone
forecaster's default remains false and its public `target_valid` is unchanged.

The outgoing solve uses bounded Newton endpoint shooting through
`integrate_ball`, followed by an explicit final residual check. A separately
named constant-restitution normal-only paddle law reconstructs the outgoing
velocity from the **actual proposed velocity and normal**. It verifies approach
and closure, then calls `advance_ball` until the first descending landing.
There is no second integrator or copied desired-state validation. Outgoing
spin is explicitly POSITION_ONLY; incoming angular evidence is not transferred
through the paddle. Net collision, wrong-side/early landing, edge ambiguity,
floor/horizon failure and around-post paths reject. Around-post is deliberately
unsupported in this initial profile. Durations tried are .50/.40/.60/.70 s;
iteration limit 12 per duration; endpoint <=1e-5 m, landing <=1 mm, timing
<=.5 ms, impact closure <=1e-8 m/s, racket speed <=8 m/s, ball speed <=20 m/s.

Final command publication also checks current physical generation/hard epoch,
source expiry, base/plane registration and movement, reach, 40-degree turn,
positive usable lead >=80 ms and exact wire identity. The packet cannot restore
validity after a failed physical check. This is a modeled valid **shadow
preview**, not authorization or proof of policy tracking. Live output is
explicitly unavailable pending the remaining qualification gates.

## Identity, time, freeze and revocation

All internal timestamps are signed int64 ns. This wire profile requires a
qualified Unix source epoch; an offline simulation epoch cannot be serialized
as hardware wall time. The contact deadline is absolute; countdown is computed
once from publication time. Outgoing duration is separate. Re-publication does
not refresh observation acquisition age.

Wire flight IDs follow physical generations, never the legacy 250 ms gap.
Same-shot source revisions retain a flight ID. Stale asynchronous work is
fenced by generation/hard epoch and source TTL; normal newer observations only
replace pending work. One worker plus one latest pending copy bounds solve
queues. Integer identity fields are bounded to 2^52 and overflow fails closed.
Planner and receiver must restart together: schema 2 has no producer nonce and
an existing receiver rejects rolled-back counters. The actual decoder tests
exercise this behavior and the coordinated fresh-receiver alternative.

The producer freezes revisions at <=120 ms lead. This is a producer policy,
not an estimate of the runner's actual commitment. The actual runner owns
retained/frozen targets and consumed-flight rejection. `valid=false` revokes
an unretained mailbox candidate; the unchanged runner may retain an already
verified flight and continue its swing. Tests explicitly show
`planner_schema22_cold_receipt_decision` accepts a retained flight even after
an invalid packet. No automatic physical cancellation is claimed.

Shadow channels under `/hope/shadow/integrated_planner/` are exact and cannot
be remapped: `racket_target` (19 schema-2 doubles, reliable depth 1), `command`
(rich physical diagnostic, best effort depth 1), `status` (20 ms input readiness,
reliable depth 1) and `trace` (per-command provenance/residuals, best effort
depth 10). The decoder probe is a test-only, non-actuating consumer. No live
distributed command-authority mechanism is claimed or enabled.
