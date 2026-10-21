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
