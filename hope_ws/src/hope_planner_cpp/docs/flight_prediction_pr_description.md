PR49 lacked the required flight transitions and physical-shot lifecycle. This branch
restores its reusable shadow scaffold on current `nightly_built` and adds a selectable
v2 pipeline with coordinated shot ownership, five-state classification, explicit
spin availability, common event physics and source-time target arrival semantics.

The new observation/packet/forecast/context interfaces are versioned alongside the
legacy interfaces. The relay preserves authoritative validity, the estimator remains
six-state, and fitting, nominal/fallback prediction, adaptive contact timing and
Stage 3 use the shared thin-shell/position-only kernel. Stage 3 converts a tabletop
landing point to ball-center height exactly once and verifies its first contact.
Existing planner/runner protections remain behind adapters; no learned policy or
hardware cancellation protocol is replaced.

The v2 launch is shadow-only and rejects output remapping outside its shadow
namespace. The new compliance profile selects FOLLOW_POSE; existing production
ON_REFRESH configuration remains unchanged. Contact Y bounds and several evidence,
clock/plane and model settings are explicitly provisional.

Validation and remaining qualification are recorded in
[flight_prediction_acceptance.md](flight_prediction_acceptance.md), with machine-readable
case mappings in `flight_prediction_acceptance.json`. Architecture and provisional
parameters are in [flight_prediction_implementation.md](flight_prediction_implementation.md);
source pins and port provenance are in `flight_prediction_port_manifest.json`.

No physical-accuracy, live frame/paddle registration, policy support or deployment
adoption claim is made from synthetic or transport tests. Independent recordings and
supervised adoption remain separate gates. This description is prepared locally;
no branch push, PR publication, merge or robot launch is part of this implementation.
