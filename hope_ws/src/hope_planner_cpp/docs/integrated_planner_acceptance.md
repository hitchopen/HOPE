# Integrated planner implementation and qualification

The evidence below is historical at `3346e030` / runtime `5ed18fa`. Review
corrections were first verified at `dc914c8` (cumulative 67/67 planner groups).
The local `hope` distrobox follow-up at `be642ba` passes 67/67 planner groups in
one invocation and the expanded 26-case focused suite. Inherited bringup lint
remains visible; both sets of results are tracked separately in
[integrated_planner_pr64_review_corrections.md](integrated_planner_pr64_review_corrections.md).

This branch implements an opt-in **shadow** single-node V2 return path over
minimal `4dc0959348f2aa777330d177d16eb70f778a79e6`. It does not claim full V3
software completion, simulation qualification, physical qualification or live
authorization. The machine-readable companion records each V3 requirement
separately; passing component tests must not be read as passing every clause.

## Code map

| Location under `hope_ws/src/hope_planner_cpp/` | Responsibility |
| --- | --- |
| `src/planner_node.cpp`, `include/hope_planner_cpp/planner_node.hpp` | Default legacy / opt-in V2 dispatch inside the existing executable. |
| `src/integrated_planner_runtime.cpp` | Typed observations, real base stream, bounded worker, publication fences, execution guards and shadow diagnostics. |
| `src/return_eligibility.cpp` | Local observed/predicted receiver-bounce-to-plane eligibility independent of game history. |
| `src/validated_return_planner.cpp` | Shared-kernel inverse shooting, explicit paddle reconstruction and first outgoing landing checks. |
| `include/hope_planner_cpp/integrated_command_lifecycle.hpp` | Semantic wire IDs, monotonic revisions, freshness, producer freeze and overflow. |
| `config/integrated_planner.shadow.yaml`, `launch/integrated_planner.launch.py` | Separate single-node shadow profile; legacy defaults retained. |
| `scripts/integrated_preflight.py` | Read-only readiness and discovered input/QoS report. |
| `test/test_integrated_return.cpp` | Eligibility, impact, inverse, lifecycle, actual decoder and actor observation builder tests. |
| `test/integrated_producer_chain.py`, `test/integrated_runner_probe.cpp` | Independently paced raw MCT fixtures through actual installed adapters/node/C++ receiver. |
| `scripts/flight_prediction_validation/verify_integrated.sh` | Reproduction entry point. |

The producer addition is in
`hope_ws/src/hope_bringup/scripts/optitrack_mct_relay`: optional configured
`ball_calibration_id` is retained in typed observations. The existing base
adapter and runner implementation are unchanged.

## Executed evidence

Verification uses the user's **`distrobox enter hope`** Ubuntu 24.04 / ROS Jazzy
environment, GCC 13.3 and Eigen. The VS Code Snap environment initially selected
a different Podman store; commands explicitly restore normal user XDG data and
configuration paths. No `nohope` checkout or existing PR was modified. The
integration branch descends from the exact minimal pin, and upstream remains
`8d599e7d21e196275cf426488555e2648e8fd789`.

The pinned baseline passes 13/13 native CTest groups. The integrated native
suite passes 14/14 groups; an independent ASan/UBSan build also passes 14/14.
Final installed results: **hope_msgs 2/2, planner 37/37, bringup 24/26**. Bringup
retains exactly 3,086 flake8 and 37 pep257 issues, matching the minimal baseline.
All four scored chains pass: 150 Hz coverage **39/41 (95.12%)**, 300 Hz coverage
**80/82 (97.56%)**, with both plane policies. Actual decoder P95 is 2.43–2.65 ms
in this localhost envelope.

The outgoing reference covers 144/144 accepted contact cases with independent
DOP853 dynamics/events and analytic paddle closure, using <=1 mm / .5 ms
budgets. Exact final counts, source/binary/config hashes, runtime measurements
and commands are recorded in `integrated_planner_acceptance.json`.

The installed 150/300 Hz tests cover FOLLOW_POSE and ON_REFRESH, original
source timestamps, unavailable orientation, pre-bounce UNKNOWN-history
commands, missing base, wrong frame/recovery, real wire callbacks, retained
legacy relay outputs and absence of live command publishers. Synthetic survey
metadata is labeled synthetic. The actual camera driver and cross-host network
are excluded. Command epochs are fixed before the scored run: the first two
flights after 100 ms acquisition and before 130 ms lead, excluding deliberate
physical frame faults and required-base outages; unknown game history remains
included. Raw source intervals, eligible timestamps, traces and decoded arrays
are retained rather than deleting rejected/missing rows.

The installed launch/preflight was exercised with no inputs and reported
`BASE_UNAVAILABLE`, UNKNOWN optional game context and zero commands. Actual
runner `--inspect-policy` loads both available artifacts: bundled 110-D
`rally_v14` and the alternate 110-D `continuous_rally_sim2real_v3` export.
Weights are test pins, not planner dependencies. Native tests call the real
110-D observation builder and real schema-2 mailbox.

The first full ROS run exposed a raw-scenario **fixture pipe deadlock** on this
host: the parent waited for the independent producer to exit before draining
its timestamp report. The fixture now drains that pipe, and all eight cases
pass. Ignored/repeated context no longer unconditionally revokes standalone
V2 outputs. The unmodified pinned baseline fails this new regression; the
fixed node passes. The regression verifies ignored messages cause zero
revocations and a real accepted semantic change still causes one.

Bringup flake8/pep257 failures are inherited and remain visible. They are not
converted into passes or hidden by the reproduction entry point. Historical
minimal receipts remain historical; this document does not overwrite their
forecast-only scope or claim independent real-ball accuracy.

## Remaining requirements and promotion limits

The implementation is useful for a reviewable **draft PR**, but it is not the
entire V3 deliverable. The acceptance JSON explicitly marks remaining clauses:

- A normal-runner, three-arm closed-loop comparison with measured actual racket
  contact, orientation, first landing and stability is not delivered. The actual
  runner and native simulator build, and policy inspection passes; those builds
  do not constitute a closed-loop outcome. The existing formal Gate3 wrapper
  adds `--gate3-force-every-flight` in its shared-plane path, contrary to this
  plan's normal-admission requirement. That wrapper was not relabeled as a
  qualifying integrated run. The V2 + legacy-return evaluation arm is also not
  implemented in the runtime.
- Live command authority, cross-host source-to-receiver qualification and a
  runner-acknowledged commitment/cancellation protocol are not implemented.
  Live mode fails startup. The producer's 120 ms freeze does not establish
  when the real actor committed. Actual mailbox tests demonstrate that invalid
  packets do not cancel an already retained flight.
- The measured producer-chain envelope is position-only incoming data on one
  synthetic trajectory. All integrated spin/source/load/profile combinations,
  exact library-vs-ROS cutoff equivalence and
  full operational launch migration/rollback remain incompletely qualified.
  A separate 35 ms slow-worker negative test passed: zero expired commands,
  211 superseded jobs, one running plus one pending job.
- Independent held-out physical flights, surveyed tool/frame/plane/clock
  receipts, paddle calibration and operator activation are unavailable/not run.
  Outgoing POSITION_ONLY is an explicit effective model, not a claim that the
  real post-impact ball has zero spin.

These are explicit remaining software/qualification gaps, not hardware-data
exemptions for missing software. Do not promote this branch based solely on
native convergence, successful decoder callbacks, or a policy loading receipt.

## Reproduction and rollback

Read `integrated_planner_input_contract.md` / `.json` and
`integrated_planner_input_migration.md`. Run the verification entry point inside
`hope`, with a fresh absolute evidence directory. It builds native and installed
ROS paths and retains every package-test result. Use an isolated local ROS
domain and shadow topics; no hardware launcher is needed.

Legacy numerical modules, packers, command types and original YAMLs remain
byte-pinned. The source scope test allows only the exact backend-dispatch and
destructor hooks in the legacy node; it still hashes the remaining legacy node
body against upstream. Restore the original planner/packetizer profiles and
restart the receiver together with its selected schema-2 producer when rolling
back a publisher session.
