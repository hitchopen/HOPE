# Proposed PR64 disposition — publication decision pending

Old upstream PR64 still points to `9118a491df658fa7e68f042581af48890ec8b0b8`.
Corrections on this follow-up branch do not resolve that reviewed head.

## Option A: focused forecast backport

Port aerodynamic acquisition, source-reversal identity, causal trusted spin,
explicit zero availability, finite-net narrow phase and event covariance into
the scoped branch. Port focused tests and the source-cutoff fixtures. Adapt
`spin_estimator` versus `flight_spin_estimator` deliberately; do not blindly
cherry-pick. Restore original legacy mode dispatch on the scoped branch, then
run its own compatibility, build, native, numerical and ROS gates. Integrated
racket commands and application diagnostics stay out of a forecast-only PR.

## Option B: cleaner superseding PR

Use the minimal lineage with these corrected forecast modules; explicitly
choose forecast-only scope or the single-node integrated shadow planner. Keep
schema-2, disabled live output and the complete limitation ledger. Reconcile
new upstream runner/deadline/calibration changes separately before choosing a
current-main PR base. Never present the old scoped PR as automatically fixed.

## Proposed reviewer response

R01: acquisition compares against configured aerodynamic acceleration.
R02: one observed reversal has one physical event identity; repeated windows
cannot apply a second impulse. Long unsupported gaps start a fresh qualified
acquisition instead of retaining another flight's ledger.
R03/R05: contact uses the causal pre-bracket spin checkpoint, refits the bounded
post-contact angular suffix and replays affected CKF intervals.
R04: a swept box is broad phase only; continuous distance to the finite
rectangle determines collision, including edges and corners.
R06: event covariance processes every earlier reset and bounce noise in order.
R07: no-op/rejected contexts remain no-ops; semantic worker results are fenced.
R08: original legacy sources and all five modes are retained, with friction
sensitivity and strict upstream numerical parity checks.
R09: authoritative/quaternion-qualified zero is available; placeholders remain
unqualified. Exact before/after evidence is in the corrections receipt.

These are proposed responses, not posted comments or resolved GitHub threads.
Do not open, retarget, merge, close or update upstream PR64 until the user
chooses the publication approach.

## Proposed commit and module map

All hashes below are correction-branch commits following `3346e030`. They are
review units, not a promise that applying them unmodified builds on PR64.

| Finding / concern | Corrections to port for A | Inclusion for B |
| --- | --- | --- |
| R01 | `b05bfba`, `5ff2b23`: `flight_state_v2.cpp` aerodynamic/recent-suffix qualifier and direct tests. | Both scopes. |
| R02 | `2a1349e`, `fe29f97`, `912d456`: reversal identity, restore from raw window, one authoritative corroboration; `flight_state_v2.hpp/.cpp`, transport tests. | Both scopes. |
| R03/R05/R09 | `c9efb43`, `10e4173`, `e001561`: `trusted_spin.hpp`, `flight_spin_estimator.cpp`, checkpoint/replay and causal contact state. Adapt scoped estimator API deliberately. | Both scopes. |
| R04 | `fd86925`: `common_ball_physics.hpp/.cpp` finite-net sweep; `5b96474` collision/time/priority controls. | Both scopes, including outgoing shared-kernel tests where selected. |
| R06 | `ef60bb6`, applicable `e001561`: `nominal_event_forecast.cpp` event-segment covariance and noise accounting; `be642ba` protects the resulting physical-admission decision. | Both scopes. |
| R07 | Retain original no-op/rejected-context guard, port standalone `65adda5` and `6961cf1` accepted/duplicate-contact ROS test; integrated `10b0811` revision fence only if actor scope chosen. | Forecast-only guard in either scope; integrated fence in shadow scope. |
| R08 | Explicit separate restoration patch for scoped legacy `TrajectoryPredictor`/`RacketTargetPlanner` mode dispatch and numeric-zero path; these are already restored on 3346 and have no correction-branch restoration commit. Port RV13 plus 701 parity, not new goldens. | Preserve minimal-lineage restored legacy sources. |
| Model identity | `fd86925`/`ef60bb6` and later `flight_state_v2.hpp`/`return_region.cpp` fingerprint changes; rebuild dependents. | Both scopes. |
| Tests / receipts | `a44e3ed`, `85d22fd`, `5b96474`, `526c70e`, `2df03aa`, `2dc5004`, `a5e8d48`, `00e8f29`, `136e186`, `dc914c8`, and final receipts. Split helper, transport, standalone, actor tests along selected boundary. | Forecast-only subset or complete integrated matrix. |

**A proposed diff:** start a new backport branch at actual PR64 head `9118a491`,
port the listed forecast modules and legacy restoration, adapt its
`spin_estimator` class to the minimal lineage's `FlightSpinEstimator`/trusted
spin contract, exclude `integrated_planner_runtime`, return action lifecycle,
runner commands and actor-only tests. Verify against its actual `nightly_built`
base and rerun its own installed/core/legacy tests. This backport is proposed,
not implemented or tested here; branch-lineage differences make blind
cherry-picking misleading.

**B proposed forecast-only diff:** start at minimal `4dc0959`, apply the forecast
subset above plus typed transport/replay and standalone tests. Leave outgoing
inverse solver, command lifecycle and integrated node dispatch out. Preserve
relevant core/scope receipts and rerun this reduced graph. This extraction is
proposed, not yet tested as a standalone branch.

**B proposed integrated-shadow diff:** publish this complete corrections branch,
which already includes outgoing `75d1579`, runtime `5ed18fa`, historical receipt
`3346e030` and the corrections map above. Title/scope must explicitly include
the single-node shadow racket planner. If maintainers choose a new upstream
base, handle `3b3c402` calibration, schemas 4/5 and runner deadline migration in
a separate reviewed compatibility change; current tests pin schema 2.

For either choice, the draft review response above must say **implemented on
this corrections branch; not yet on PR64** until the actual reviewed head is
updated or an approved superseding PR exists. Neither the fork push nor this
receipt resolves GitHub review threads. No upstream comment has been posted.
