# Ball-flight scope and differential receipt

This review branch continues `e73b3123bb881a10b562f1ba74a5173d667f31d8`.
The freshly checked upstream `nightly_built` remains
`8d599e7d21e196275cf426488555e2648e8fd789`. The 20 October scoped handoff narrows
the earlier integration plan. This deliverable ends at physical ball forecasts.
It does not choose or generate a racket strike or robot action.

## Ownership and file disposition

| File/module | Disposition | Observable consequence |
|---|---|---|
| `flight_prediction_v2_node.cpp` | KEEP forecast; REVERT command adapter | `planner_locked`, inverse solve, fixed landing objective, decision publisher, command preview, swing sign, lead/Y/turn/reach admission and policy height removed |
| `FlightDecisionV2.msg` / registration | REVERT unused V2-only type | Repository consumer audit found only the removed V2 node/smoke; no production command type removed |
| `FlightPredictionV2`, event/waypoint/packet schemas | KEEP / finish physical contract | Forecast and packet wire revision 3; exact times, posterior, spin provenance, conditional event covariance and registered plane geometry |
| `common_ball_physics` | KEEP forward physics | RK4, radius-aware events, finite net, dissipative contact; reusable without inverse planning |
| `batch_physics_estimator`, `spin_estimator`, `trajectory_predictor`, `future_contact_predictor` | PHYSICS-ONLY | Piecewise causal fitting, trusted spin and forward receiving fallback; no new landing optimization or control policy |
| `racket_target_planner.cpp` | REVERT to exact upstream bytes | Removes V2-added radius shift of inverse objective, first-contact inverse verification, alternate-candidate changes, net admission changes, and shared-integrator adoption inside the inverse solver |
| `adaptive_flight_commit.cpp` | REVERT to exact upstream bytes | Removes V2-added event-based bounce readiness/scheduling; upstream algorithm restored |
| `types.hpp::RacketCommand` | REVERT V2-only additions | No outgoing route validity, spin or selected-duration command extension |
| `planner_node.cpp`, relay/packet observation adapters | KEEP authoritative validity fixes | Unqualified legacy orientation cannot manufacture measured spin; existing command admission and publications remain on their original paths |
| PR49 selector/runner probes | KEEP historical compatibility | Opt-in launch and regression coverage; not prerequisites to the V2 executable or forecast-only acceptance target |
| Baseline replay adapter | EVALUATION ONLY | Existing fixed one-shot trigger and progressive estimator export ball predictions; never generate a command |

The following remain deferred proposals, **not passed scoped requirements**:
Stage 3 surface-to-center landing objective, inverse first-contact verification,
alternate outgoing candidate selection, robot Y/turn/reach/lead feasibility,
racket normal/velocity/FH-BH selection, motion retiming and hardware cancellation.

## Build boundary

`hope_ball_physics_core` contains forward physics, spin, fitting and receiving
prediction. `hope_forecast_core` and `hope_flight_transport_v2` link that library;
the V2 node does not link `hope_planner_core`. Production planner/legacy selector
code may depend on the forward library. The read-only baseline exporter separately
uses existing legacy acquisition/one-shot classes for accurate trigger timing.
`flight_v2_binary_scope` checks the V2 executable for inverse/packing symbols;
source and graph tests independently prohibit command construction/publications.

`integrate_ball` advances supplied p/v/optional spin. `advance_ball` additionally
resolves ordered contacts/events. Callers constructing hypothetical outgoing
states own their initial condition and angular availability. No API chooses an
outgoing landing objective or transfers incoming spin across an unmodelled racket
impact. Production inverse-planner adoption of the new integrator is deferred.

## Regression interpretation

`production_parity.py` compiles the pinned upstream and current implementations
against the same public headers and fixture inputs. All 18 fixed-ball-input
racket results match exactly, including velocity and normal. The original Stage 3
scalar goldens are also retained as a separate fixed-input test. Source hashes
for the inverse solver and adaptive scheduling enforce the upstream boundary.

This does **not** imply end-to-end production parity: the receiving forward model
was already changed in V2. On the small differential probe, two mutually valid
receiving cases differ in arrival by up to about 49.7 ms and position by about
10.5 mm; a third changes validity. Complete deltas, validity flags and velocity
differences are recorded in the acceptance JSON. These measure a model change,
not real-ball improvement. Existing one-shot, segmentation, precommit reversal,
FH/BH, packet, turn/reach and receiver regressions remain required.

## Interface compatibility

Observation/context wire revision 2 is preserved. Forecast and packet revision 3
require a coordinated regeneration/rebuild of the same V2 message names; schema-2
packets are rejected. This branch is pre-adoption, remains shadow-only, and does
not silently reinterpret legacy production messages. `shot_id` equals the
immutable `shot_generation` **within a session**; forecast revisions are distinct.
A restarted producer needs a fresh qualified session/reset before packet admission.
