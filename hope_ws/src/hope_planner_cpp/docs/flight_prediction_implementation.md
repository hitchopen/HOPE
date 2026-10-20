# Flight state and prediction v2: forecast-only contract

The V2 product publishes the estimated and future **ball** state, physical events
and registered receiving-plane intersection. Racket inversion, action feasibility,
command generation/packing, robot timing and policy behavior are outside this
pipeline. See `flight_prediction_scope_v2.md` for exact source dispositions.

## State ownership and classification

`ShotContext` owns bounded raw observations/candidates, generation, semantic
revision, confirmed/predicted event history and trusted spin. `FlightPipeline`
coordinates the existing six-state p/v CKF and its correlated measurement-bias
nuisance moments. There is no nine-state spin filter or stochastic ensemble.

The five public states are INVALID, OPPONENT_RALLY, RALLY, OPPONENT_SERVE and SERVE.
Qualified ±X acquisition starts a provisional rally family. Service context may
constrain `forecast_route` but never confirms an unobserved serve. A confirmed
first striker-side bounce before the net confirms the corresponding serve while
preserving the queue, timestamps, spin, generation and event ID. Complete history
plus a confirmed net crossing without a striker bounce confirms rally. History
loss recomputes classification; an observed B1 can independently support serve,
whereas context alone cannot preserve a stale CONFIRMED status.

Match type and history continuity are separate evidence. Acquisition after an
unseen net/bounce keeps physical forecasts and withholds unsupported route targets.
Trusted historical events may precede an unqualified acquisition timestamp, but
cannot precede a qualified physical boundary. Hard new shot, explicit invalidation
or incompatible provenance clears the posterior, raw active queue, spin and ledger.
Velocity reversal alone is not a new racket shot; a qualified same-direction shot
is. Rejected flights cannot re-arm merely from more samples of that flight.

Predicted contacts do not increment observed counts. Source reversal confirmation
retains distinct occurrence/confirmation times and propagates spin once. A predicted
contact can be confirmed after its nominal time, even if a transient horizon
superseded it. Recovering the same gated future hypothesis reuses its ID. Multiple
plausible associations remain ambiguous. Smooth-window acceleration cannot prove
non-ballistic motion across a plausible unresolved table contact; held-ball spatial
rejection remains independent. Net crossings carry bracketed p/v, not a zero
placeholder velocity. Apexes use the vertical-velocity root.

## Physics, spin and uncertainty

`common_ball_physics` is shared by fitting, CKF mean/sigma propagation, nominal
forecast, receiving fallback and V2 replay. It uses RK4, actual top footprint,
radius-aware descending contact, finite expanded net, event ordering and once-only
impulses followed by remaining-substep propagation. An initially below-table ball
is not teleported into a bounce; edge/tangent/branch ambiguity is explicit. Numerical
ties favor collision over a target. Multiple contacts and all work are bounded.

MEASURED, HELD, CONTACT_PROPAGATED and POSITION_ONLY are distinct. Missing, rejected,
nonfinite and raw all-zero sentinel vectors preserve the compatible whole vector
and original measurement source stamp. Producer provenance survives observation,
packet and forecast transport. Missing quaternions break differencing only;
authoritative identity may be valid, synthetic identity is not. q/−q and long-gap
ambiguity are checked. Positive-time propagation changes fresh MEASURED to HELD;
future points do not pretend to contain new measurements. Confirmed table events
propagate trusted angular state once; speculative forecasts modify only their
copies. No qualified prior means no synthesized angular state or Magnus through
any number of contacts. New measurements affect only subsequent intervals.

The covariance is a conditional p/v local linearization. Free-flight Jacobians
match RK4; event/contact maps and acceleration-based apex time derivatives provide
conditional event covariance. Auxiliary spin/model, plane and clock uncertainty
remain uncalibrated. PSD/symmetry and branch quality are checked. A narrow
conditional covariance never proves a hard forbidden event. The conservative
support API requires explicit bounds covering all omitted sources and separated
event-order intervals. Confirmed forbidden events invalidate immediately.

## Physical route events

Incoming rally requires our-side B1 then receiving plane before B2. Incoming serve
requires striker B1, net, receiver B2 then plane before B3. Outgoing rally uses
opponent B1 then end crossing; if B2 comes first, it selects the exact B1–B2 apex.
Outgoing serve requires B1/net/B2/end before B3 and has no apex exception. Legality
stops at the selected target; later irrelevant predicted events do not revoke it.
A historical apex is labelled historical. Warming up, exhausted horizon, uncertain
history/ordering or an unavailable plane can suppress a selected ball event while
a useful physical trajectory remains available. No robot lead/reach/Y gate exists.

## Wire revision 3 and channels

`BallObservationV2` and `FlightContextV2` remain wire revision 2. Forecast/packet
revision 3 requires rebuilding producers and consumers; prior packets are rejected.
All acquisition/reference/arrival stamps are signed int64 ns with validity and
clock/mapping identity. Integration uses differences from a pinned integer origin.
Zero and negative simulation epochs are valid. Duplicate/reordered measurements
are rejected. Clock mapping loss revokes outputs; incompatible identity retires a
shot. Exact arrival is reference + integrated duration, never publication/receipt.

Outputs are restricted before publisher creation and after remapping to:

- `/hope/shadow/flight_v2/prediction`: compact state/event/selected-ball revisions.
- `/hope/shadow/flight_v2/trajectory`: the same source snapshot plus full waypoints.
- `/hope/shadow/flight_v2/packet`: transferable posterior, raw suffix, history and
  trusted/model spin checkpoints.

There is no decision/control publisher. Physical coordinates never receive a policy
height offset. `current_state` is the source-time CKF posterior with 6×6 covariance;
waypoints have p/v/optional omega, original angular source/provenance, covariance,
quality, reference, delta and absolute arrival. `spin_available` means usable for
modelling; `spin_measured_valid` is true only at the actual measured instant.
Selected physical events include conditional 7×7 p/v/time covariance and time std.
The event ledger carries status, bounce index, pre/post velocity and spin, source
occurrence/confirmation and signed reference delta, including historical events.

`shot_id` is `shot_generation` within `session_id`, never a forecast revision.
Public forecast revisions are allocated at publication/revocation. Raw worker
sequence, generation/context and age fence results. One solve runs with one latest
replaceable pending snapshot. Compact publication precedes full serialization;
packet hashing runs outside the admission mutex and is fenced again afterward.
Canonical YAML hashing uses classic locale and 17-digit doubles, excluding CDR
padding. Duplicate/stale/equal-counter packet conflicts cannot silently rewind a
receiver. Producer changes require explicit session reset.

Full and packet cadence are separately configurable (`full_output_rate_hz`,
`packet_output_rate_hz`, defaults 50 Hz). Zero publishes every completed latest
result. Phase-based decimation preserves original source stamps. Actual publication
freshness is checked per channel. Full trajectories continue beyond the hitting
plane within the bounded horizon. `benchmark_cadence.py` records compact/full
coverage, receipt/publication latency, sampled CDR payload, node CPU/RSS, superseded
work and actual rates. Higher settings are measured choices, not blanket guarantees.

## Registered plane

`ON_REFRESH` intentionally freezes the qualified plane until permitted refresh;
its original pose does not expire during the rally. `FOLLOW_POSE` snapshots the
latest fresh registered base pose for each solve and holds it fixed within that
solve. Base +0.10 X moves the next plane +0.10. Missing/stale/out-of-range pose
withholds a target without clamping or using an old frozen plane. Normal geometry
updates do not change ball ownership; a world/calibration mismatch does.

The implemented registration is a table +X plane through base_x + 0.58 m, with
plane normal (1,0,0). This geometric normal is unrelated to a racket orientation.
No arbitrary-heading or live table transform is inferred from an unqualified source.
The default registration remains `UNQUALIFIED_SHADOW_REPLACE_BEFORE_ADOPTION`.

## Provisional values and adoption evidence

| Parameter | Current value | Evidence required before changing/adopting |
|---|---|---|
| Plane offset/range | +0.58 m / [−0.30,+0.30] m | Surveyed base/table registration |
| Pose source age/window/samples | 0.20 s / 0.50 s / ten | Source-clock and tracker characterization |
| Acquisition/ballistic thresholds | 25 ms, 1 m/s, 8 m/s² residual | Varied real flights and held/toss/contact cases |
| Observation window/capacity | 0.30 s / 192 samples | Dynamics and host workload |
| Contact association | 30 ms / 0.10 m | Independent contact annotation uncertainty |
| Clock/source budget | 30 ms | Measured deployment load and clock mapping |
| Integrator/output/horizon | 1 ms / 5 ms / 1.5 s | Numerical and performance envelope |
| Full/packet cadence | 50 Hz / 50 Hz | Committed cadence benchmark on target hardware |
| Table/net/radius/gravity | 2.74×1.525 m; net 1.37/.1525 m; .02 m; 9.81 m/s² | Venue and ball identification |
| Drag/Magnus/contact | 2026-10-10 receipt | Held-out independent ball recordings |
| Position-only coefficients | Provisionally borrowed | Separate physical error/uncertainty assessment |
| Orientation authority | Relay default false | Qualified source quality semantics |
| Constant free-flight omega | Prior, no fitted decay | Independent orientation recordings |
| Conservative forbidden bound | Unqualified by default | Spin/model/plane/clock error bounds |

No real-ball accuracy, full uncertainty calibration, robot feasibility or deployment
readiness is claimed. See the executed acceptance receipt and physical NOT_RUN file.
