# PR85 planner software qualification

The integration checkpoint is `e7947b9486f1b49b70a560318eaf12bc7de9979a`, merging the selected upstream `91aecabfd44cb258ac998158e724be9017d883a7` into the existing PR85 branch. The final source is not yet scored. Qualification remains open until the complete clean verification and five fresh-process repetitions of each declared positive profile finish.

The immutable predeclared matrix and unchanged 95% coverage / 30 ms freshness / 30 ms decoder P95 gates are in `integrated_planner_software_contract.json`. The denominator remains independent source opportunities after 100 ms and with more than 130 ms lead, retaining existing fault exclusions.

Preliminary diagnosis found a source state/acquisition timestamp mismatch up to 2.1 ms and a ROS shutdown graph-query exception that could abort before flushing failure evidence. The source now samples one acquisition epoch; the runtime guards diagnostic graph calls only when ROS has shut down. Active-context errors still propagate. Bounded, default-off local stage accounting does not change topics, schemas, QoS, physics, safety tolerances or worker bounds. Publication age now includes the publication call.

Historical coverage failures and initial diagnostic failures remain evidence, and their causes are not uniquely established by later passing pilots. The cloud proxy currently blocks GitHub API operations; Git push is available. Simulation, physical qualification and deployment authorization remain false.
