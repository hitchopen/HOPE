# PR85 planner software qualification

The integration checkpoint is `e7947b9486f1b49b70a560318eaf12bc7de9979a`, merging the selected upstream `91aecabfd44cb258ac998158e724be9017d883a7` into the existing PR85 branch. The final source is not yet scored. Qualification remains open until the complete clean verification and five fresh-process repetitions of each declared positive profile finish.

The immutable predeclared matrix and unchanged 95% coverage / 30 ms freshness / 30 ms decoder P95 gates are in `integrated_planner_software_contract.json`. The denominator remains independent source opportunities after 100 ms and with more than 130 ms lead, retaining existing fault exclusions.

Preliminary diagnosis found a source state/acquisition timestamp mismatch up to 2.1 ms and a ROS shutdown graph-query exception that could abort before flushing failure evidence. The source now samples one acquisition epoch; the runtime guards diagnostic graph calls only when ROS has shut down. Active-context errors still propagate. Bounded, default-off local stage accounting does not change topics, schemas, QoS, physics, safety tolerances or worker bounds. Publication age now includes the publication call.

Historical coverage failures and initial diagnostic failures remain evidence, and their causes are not uniquely established by later passing pilots. The cloud proxy currently blocks GitHub API operations; Git push is available. Simulation, physical qualification and deployment authorization remain false.

## Rejected first final candidate

The complete ROS suite on `63790a3` ran all 105 planner, 2 message and 27 bringup groups. Planner results were 101/105: one round's two rejection samples were absent from both the typed observer and planner input history, one cutoff fixture compared offered rather than actually consumed inputs, and two otherwise passing profiles hit an invalid-context exception while the harness intentionally stopped the base relay with SIGTERM. Bringup retained exactly the inherited lint categories with no added normalized findings. Native 20/20, ASan/UBSan 20/20, both 701-case legacy comparisons, DOP853 144/144 and 789 premerge V2 cutoffs passed. The parent verifier was stopped at the completed ROS boundary; no 160-trial campaign was started on this failed candidate. All results are retained.

The follow-up uses a bounded depth-eight input history at both raw relay and integrated V2 reader. The raw relay's legacy default remains one; the qualified profile explicitly opts into eight. A controlled actual-DDS callback pause proves that depth one erases intervening under-table observations and depth eight retains them, independently at 150 and 300 Hz. The exact first loss point of the original uninstrumented failure remains unresolved. Freshness, physical rejection, worker bounds and independent coverage denominators are unchanged.

The reacquisition cutoff fixture now records the exact ordered inputs passed to the real pipeline, including rejected inputs, and replays that consumed prefix. Every offered and missing row remains available separately. A new observed dropped-input example shows nonzero disagreement when replaying all offered rows and zero disagreement for the actual consumed prefix. This is an equivalence test; raw producer profiles still score every independently eligible offered epoch. The base-loss fixture now uses ROS's interrupt shutdown. Partial-startup failures also enter cleanup and retain available receipts, with a deliberate missing-decoder regression.

Five targeted profiles passed at 100% coverage with complete actual-stage accounting after these changes. This is preliminary diagnostic evidence; the corrected candidate still requires the entire clean ladder and all 160 declared trials.
