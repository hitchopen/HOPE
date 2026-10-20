# Scope V2 to causal ball-flight forecasts and complete its physical output contract

V2 previously coupled ball prediction to a shadow racket decision and could label a
serve confirmed from match context before observing its first bounce. This change
ends V2 at ball-state, trajectory, physical-event and receiving-plane forecasts.
SERVE context can constrain the anticipated route; only observed evidence confirms
the public serve classification. Loss of history recomputes classification without
inventing a new physical shot.

The V2 node no longer constructs decisions, calls the inverse racket planner,
packs command previews, or applies robot Y/reach/turn/lead/policy-height admission.
`FlightDecisionV2` was unused outside that adapter and is removed. The inverse
`racket_target_planner.cpp` and `adaptive_flight_commit.cpp` are restored exactly
to upstream `8d599e7`. Original command scalar goldens remain. Legacy PR49 command
compatibility probes remain separately testable and are outside the V2 link graph.

A separate `hope_ball_physics_core` provides shared forward free-flight/contact
physics to estimation, forecasting, fallback and evaluation. All 18 fixed-input
racket fixtures match upstream exactly. Receiving forward-model changes remain:
two mutually valid differential fixtures differ by at most 49.67 ms arrival,
10.49 mm position and 0.675 m/s velocity; a third changes validity. These are
numerical behavior differences, not evidence of improved physical accuracy.

Forecast/packet wire revision 3 adds an explicit current posterior, shot identity,
spin validity/provenance, pre/post event states, source-referenced event lineage,
conditional covariance and registered plane geometry. Observation/context remain
revision 2. Consumers must rebuild the forecast interfaces together; old packets
are rejected. Only `/hope/shadow/flight_v2/` forecast channels are published, with
post-remapping guards. No runner, motion or production command publisher is enabled.

Contact predictions retain identity through retirement and source confirmation;
ambiguous simultaneous hypotheses remain explicit. Propagated spin is HELD rather
than MEASURED while preserving its original acquisition time. Unavailable planes
cannot create a default-X crossing event. Pose freshness is checked when preparing
the forecast, with geometry frozen per solve. Incompatible clock domains invalidate
ownership; missing compatible mappings revoke output without fabricating clock time.

Full-trajectory and packet cadence are independently configurable; default 50 Hz,
zero for every completed latest result. Compact updates retain exact source stamps.
The bounded worker and publication fences reject stale generations and revisions.
All 50/100/150/latest-result combinations at nominal 150/300 Hz are measured,
including failed optional profiles. Consult the acceptance receipt for actual
rates, coverage, freshness, CPU/RSS and CDR payload; higher settings are not a
universal throughput guarantee.

Validation uses a clean HOPE Jazzy distrobox build, 33/33 planner and 2/2 message
CTest groups, plus 24/26 bringup groups. The complete repository gate remains
failed for inherited flake8 (3,086) and pep257 (37) violations, reproduced before
changes. Native ASan/UBSan/leak checks pass 13/13 groups; supported Python spin
checks pass 6/6. Independent DOP853/analytic fixtures, causal replay, source/binary
scope audits, packet transfer and installed raw ROS scenarios cover both directions,
150/300 Hz, automatic two-bounce serves, spin dropout/reacquisition, plane policies,
clock/frame faults and new-shot revocation. Exact final code/config pins and
executed metrics are in `flight_prediction_acceptance.json` and its Markdown index.

Matched one-shot/progressive/V2 replay and scoring generate three figure families
on identical source cutoffs, retaining rejection and missing-truth denominators.
One-shot emits only its actual trigger epochs; whole-shot comparisons use common
valid pairs. Synthetic tooling generated 270 matched rows and six watermarked
figures. **Independent physical accuracy is NOT_RUN:** no qualified recordings,
annotations or registration/clock receipts are available. No physical superiority,
complete uncertainty calibration or deployment readiness is claimed.

Scope and future work: Stage 3 inverse landing/normal/velocity choices, command
admission and robot scheduling are deferred. Real table/plane registration,
position-only calibration and independent held-out accuracy remain necessary.
This branch is prepared for user review; no PR, merge or robot deployment is part
of this implementation task. See the scoped acceptance matrix for all 40 original
cases, X01–X10, explicit deferred subassertions and F01–F11 dispositions.
