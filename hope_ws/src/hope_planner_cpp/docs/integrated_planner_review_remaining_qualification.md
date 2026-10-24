# Remaining integrated qualification

This is the historical PR64 correction roadmap. The later continuation results,
including recovered consecutive flights and the still-open timing gate, are in
[integrated_planner_reacquisition_spin_qualification.md](integrated_planner_reacquisition_spin_qualification.md).
Its G0–G7 numbering is separate from the historical V3 gates below.

Review corrections and broader V3 qualification are separate outcomes.

| Gate | Remaining work |
| --- | --- |
| G1 input | Broader negative QoS/restart/registration matrix and measured cross-host source-clock guarantees. |
| G2 handoff | Identical source-cutoff native/installed comparison is added here; broader source variants remain to be qualified. |
| G3 numerical | PASS for declared synthetic envelope: 144 independent outgoing candidates, corrected finite-net and event-covariance cases. Synthetic model agreement is not real accuracy. |
| G4 lifecycle | Semantic races, long-gap reacquisition and worker expiry are covered here; representative real receiver restart/postcommit operation remains. |
| G5 actor | Real schema-2 decoder and 110-D builder are tested; achieved paddle pose/normal and actual tracking remain NOT_RUN. |
| G6 graph/performance | Localhost synthetic high-rate matrix is tested here; target host/network load, combined diagnostic saturation and VRPN adapter remain unqualified. |
| G7 closed loop | NOT_RUN: normal runner three-arm original/original, V2/legacy, V2/validated comparison with contact, landing and stability scoring frozen before execution. No force-every-flight bypass. |
| G8 field | NOT_RUN for corrected planner: independent held-out prediction and registered pelvis/table/paddle/clock receipts plus operator-reviewed activation. |

Upstream main now includes `calibration/ppq004_20261020` real C3D/replay data and
archived holdouts. Those are available inputs for a separate real-flight study,
not missing files. Its deployed-profile tangential gain exceeds this branch's
thin-shell `<=.4` validation range; no coefficients are silently substituted.
Upstream also adds optional runner schemas 4/5 and new deadline behavior.
This work remains on the plan's exact integration base and schema-2 contract.

`software_complete`, `simulation_qualified`, `physically_qualified` and
`deployment_authorized` remain false. Original acceptance receipts remain
historical at `3346e030` (runtime `5ed18fa`). No robot was actuated.
