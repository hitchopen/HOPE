# PR85 integration checkpoint

PR85 was updated with ordinary merge `e7947b9` against upstream `nightly_built` at `91aecabfd44cb258ac998158e724be9017d883a7`. This checkpoint preserves legacy and V2 functionality; timing qualification remains open. The merge is an ordinary two-parent merge, with no history rewrite or upstream merge.

## Pins and conflict resolution

- Original PR head: `47296db5653c6dda7d397d7a82153f69fa4ed4fb`.
- Common ancestor: `0f9f41e9e3bb822ca7f262860ba3a4e13db84e26`.
- Native merge: **18 real conflicts** (12 content, 6 add/add); stage blobs, index and conflict diff retained in the evidence.
- Existing HOPE and nohope worktrees were preserved. Current deployment/Foxglove files have no remaining diff from the selected target.
- Every shared auto-merge is listed in the JSON companion. The planner entry point differs from upstream only by opt-in V2 dispatch/ownership/destruction; legacy stays the default.
- Keep upstream `rcl_interfaces`, session-home installation, runtime-plane tests, `max_sample_gap_s`, `single_flight_only`, packer/receiver headers and deployment profiles. Link the new `test_planner_runtime_plane` target to the V2 runtime because it compiles the shared entry point.
- Keep V2 physics and shadow configuration separate from upstream legacy PPQ004 overrides. V2 does not add support for runner schemas 4/5 or shared-HOME runtime controls. The existing schema-2 absolute contact deadline and height conversion pass against current receiver headers.

| Conflicted path | Resolution |
| --- | --- |
| `docs/operations/foxglove_lifecycle.md` | Consolidated operations node, stopped migration, early base relay and late Runner gateway. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `docs/operations/vrpn_pants_profiles.md` | One shell-v3 CMTracker import asset, preserving runtime V4 profile aliases. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/README.md` | Current operations ownership, native gateway and stopped deployment instructions. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/a3/hope_lifecycle_supervisor.py` | LifecycleModule under operations, bounded work, draining and ball-model verification. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/extensions/hope-a3-console/src/HopeA3Console.tsx` | Current HOME, hit-speed/plane controls, connection and stopped ball-model selection. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/extensions/hope-a3-console/src/styles.css` | Current Console control styles. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/extensions/hope-a3-console/tests/operatorProfile.test.cjs` | Updated Console operator-profile contract assertions. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/helpers/hope-laptop-container-runtime` | Preserve current peer list and capture launch behavior. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/helpers/hope-lifecycle` | Independent native base relay, late gateway, ball-model verification and selected serve CSV. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/tests/test_field_operations.py` | Current field/CSV/ball-model and lifecycle ordering regressions. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/tests/test_mocap_setup.py` | Variable peer list and gateway lifecycle test compatibility. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/tests/test_operator_profiles.py` | LifecycleModule, ball-model verification and Mac/Linux dispatch. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `foxglove/tests/test_runner_transport_start.py` | Gateway startup health and exact legacy-session cleanup. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `hope_ws/src/hope_bringup/CMakeLists.txt` | Upstream VRPN shell-v3 asset regression registration. Retain upstream asset tests plus V2 full-rate relay registration. |
| `hope_ws/src/hope_bringup/launch/avatar_pro_hope_bridge.launch.py` | Stopped-only ball-physics override for legacy packetizer; V2 shadow launch remains separate. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `hope_ws/src/hope_bringup/test/test_planner_full_rate_relay.py` | Preserve 300/360 Hz original exposures and side transforms. Keep feature superset: upstream 300/360 Hz preservation plus 150 Hz, source stamps, orientation authority, V2 observation validity. |
| `hope_ws/src/hope_bringup/test/test_vrpn_pants_relay.py` | Actual 300 Hz VRPN stream and no cached ball republishing on body callbacks. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |
| `scripts/macos/operator-dispatch.py` | Retain reconnect-laptop and ball-model-laptop allowlisted actions. Retain current upstream: this path has no V2-era change; earlier inherited implementation is superseded. |

## Executed verification

- Both clean ROS builds passed, including all targets compiling `planner_node.cpp`, replay/export tools, standalone forecaster and installed messages.
- Native C++: 19/19 groups, including the selected upstream receiver, restart, retention and session-home tests.
- Upstream ROS: messages 2/2, planner 19/19, bringup 24/26. Merged: messages 2/2, 44/44 selected planner functional groups and 6/6 installed producer/decoder profiles, bringup 25/27. The exhaustive repeated matrix is Step 2, not this checkpoint.
- Legacy: 701/701 exact matches against the current upstream target. Historical comparisons keep their original pin and receipts; hashes were not forced back to obsolete source.
- V2: ten recorded input streams produce exact state/event/covariance/readiness/generation outputs before and after merging, excluding compute duration. All 144 outgoing command rows match exactly; the outgoing sweep binary hashes also match.
- Deployment regressions: 83 tests and 148 subtests passed.
- The source-scope guard uses `test/fixtures/pr85_upstream_sources.json`, which records the explicit historical-to-current SHA mapping. Original historical fixture hashes and receipts remain unchanged.
- Bringup lint remains failing. After formatting only the touched test argument list, there are no added normalized findings relative to current upstream (file/rule/exact source-message retained, line/column coordinates ignored). Pep257 is unchanged; flake8 removes several findings. Actual colcon test-result exit is 1 for both trees.
- An initial wrapper invocation ran discovery from the wrong directory; it was corrected before the selected run. Initial XML tests could not fetch the external ROS schema; both trees were rerun using the existing local schema catalog. These setup failures are not passing qualification runs.

## Timing and delivery

The two prior failing profiles were run once on the integrated build before performance changes. Both passed; recovery coverage was 160/160 and the direct producer profile was 157/160. This does not close the historical 92.5% and 91.25% failures or replace the five-repetition campaign. Full software qualification remains false.

GitHub API requests are blocked by the environment proxy; the public PR page and fetched PR ref confirm the recorded target/head and open status. GitHub generated test merge `e79ca44b8b93529439affbf314c02f48d1af9f23`, with the pinned upstream and checkpoint as parents and exactly the checkpoint tree. This independently demonstrates textual mergeability; API check/policy status remains unavailable. No PR draft-state change, upstream merge, live output, simulation or robot actuation is requested or performed.
