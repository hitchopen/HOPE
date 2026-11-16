# 2026-11-08 — Gate3 infrastructure repair and guarded continuation

Existing evaluation worktree/branch `HOPE-stationary-gru-eval` /
`feature/stationary-gru-mujoco-eval`; base `9d207e53b559d3f48b485f8f91f79c4977226099`.
Initial worktree clean. No applicable AGENTS.md found. Personal HOPE origin is
`https://github.com/1786089174zth-cell/HOPE.git` (fetch/push). Fetch fails DNS;
live remote divergence remains unknown. No merge/reset/main/training edits.

The failed `formal-20261108-123341`, parent formal log, all model packages,
checkpoint and original installed binaries remain untouched. No weights, model
configuration, simulator source, physical parameters, serve ledger or acceptance
limits changed. The exact v2 actor remains SHA256
`2b2a029b450fb8601ecf34eefdbd3df6ab9f8597a33593dc9b7d6419fcd34d27`,
YAML `160f282fdb95d4ca5d0df5222897a203150152248fcc364f8ba723086357f3fc`;
checkpoint `744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0`.

## Findings and implementation

**Lock:** inherited `model21800_hardware.yaml` selects
`/var/lib/hope-lifecycle/planner_wire_identity.state`; actual lock is that path
plus `.lock`. Historical log discarded errno, so its original errno/holder
cannot be reconstructed. Native exceptions now distinguish open/flock, preserve
errno, exact path and effective UID/GID. `pp_wire_identity_probe.py` uses the same
open flags, nonblocking diagnostic flock, lstat ownership/mode/ancestors and
`/proc/locks`/PID command inspection. It never deletes state, truncates the counter,
changes permissions or signals a holder. It can create the lock file with the
same O_CREAT/0600 semantics when allowed, but never writes identity state.
Its timestamps/visibility explicitly describe the current container namespace,
not a retroactively inferred historical owner. Desktop continuation captures
this diagnosis before the new build/test; sandbox cannot access hope.

The hardware setting also seeds wire flight IDs with a persistent U52 range,
whereas the reference scorer requires physical/packet/Runner identities1..N.
The documented Planner default (`wire_identity_state_file=''`) is appropriate
for a new isolated simulation process, not a live hardware restart. The generic
Gate3 simulation profile now explicitly selects that existing default; hardware
configuration/persistent state and hardware replay checks are unchanged.
No Planner restart within a rally is allowed. Global runtime conflict checks,
fresh Runner admission, and isolated ROS domain77 in the continuation maintain
the session boundary. No command IDs are rewritten in scoring. Native focused
tests cover persistent restart/rollback separately from fresh simulation IDs.

**HOME:** `PlannerEngageStep_` previously checked HOME against an empty snapshot
before `has_valid`, incorrectly labelling no received command as
`session_home_mismatch`. It now handles no-command first and retains all HOME
checks for genuine commands. New per-command diagnostics record schema, epoch,
frame, actual/expected X, HOME XY/plane and actual/expected calibration/world IDs.
Wire command/revision IDs are joined exactly to the Planner CSV; the explicitly
labelled local receiver epoch is not misrepresented as a Planner boot nonce.
Native tests exercise the real shared HOME pack/parse/mailbox, schema4 target,
epoch, frame, world/calibration and freshness rejection. Live acceptance is
still a required preflight proof, not inferred from these tests.

**Plane:** saved env config has FH/BH X support[0.44,0.70]; ONNX metadata and
strict native inspection exactly match. Existing `PpPolicy` selects each box's
midpoint0.570. The planner profile uses the same rule and schema4 commands use
immutable HOME_x+0.570 in calibrated world X (not a table-height correction).
The old standalone V14 checker instead hard-coded0.580. Legacy checking remains
strictly0.580 without the new option. Only the generic fixed-HOME path supplies
its actual native inspection; it must match both observed Runner planes to
the independently derived plane within the same1e-6m tolerance. Tests reject
0.580 for this0.570 candidate. No model target is shifted, and0.580 is not claimed
outside its training support. This is model-bound generic Gate3 geometry, not
a claim of the professor model21800's identical actor/target configuration.
The remaining Build1 posture and other reference checks are retained.

**Plant CLI:** the tree contained a628-line older report tool but a launcher
requiring support-frame, torso-HOLD and collision checks. No compatible newer
local tool was found. New `pp_gate3_plant_contracts.py` implements these required
checks from the professor's archived
`docs/gates/model14600_gate3_30balls_20261013/pp_mujoco_plant_report.json` contracts:
last strict pre-MOTION plant anchor (max age0.25s), anchored world foot/support
geometry, yaw-zeroed HOME frame, actual mode3/level0 HOLD windows with0.12s trim
and0.20s minimum, and instrumented table/net/self collision counters. All CLI
limits passed by the original launcher are retained and evaluated. Missing
columns, nonfinite data, stale anchors and coverage gaps fail closed. This is
a source implementation of the archived contracts, not a claim of byte-identical
upstream report code. HOLD limits apply per eligible window. Optional-reach
qualification is explicitly unavailable/fail-closed if requested/nonzero; this
model's inspected profile disables optional reach, so no permission is fabricated.
Existing idle smoothness definitions, deadbands and limits are unchanged.

**Runtime guard:** the managed engine records its own Planner PID atomically.
It checks startup and monitors the Planner through the evaluation and flight
draining. A crash becomes infrastructure failure and stops only owned process
groups. A separate post-drain marker distinguishes intentional cleanup from a
crash during evaluation, avoiding a false failure during offline reporting.
On a Planner crash the guard first signals only the managed engine so its trap
stops future launches and drains existing flights; owned force cleanup is a
recorded fallback only if that bounded cleanup exceeds30s.

## Checks actually run; no simulation rerun

50 selected focused Python tests pass (report metrics/violations/missing data,
real lock holder probing, plane strictness, preflight guards, watchdog cleanup,
and unchanged formal100 vs separately labelled six-clean-serve prefix).
One pre-existing static-source assertion in
`test_model21800_wrapper_requires_native_no_ball_smoothness_gate` fails because
it searches for an old `_spin_until` implementation string. Both that test and
the ball launcher are byte-identical to base HEAD; baseline proof is archived.
It was excluded from the50 selected checks, not changed or claimed passing.
The actual20s idle remains covered by composition and runtime preflight checks.

Host g++ compiled/executed production identity/HOME-header tests: PASS, including
exact open errno2 on a deliberately nonexistent private fixture parent,
persistent rollback reservation and HOME identity rejection. This is not the
hardware path's observed errno. The desktop stage additionally compiles/tests
the actual `Schema2Packer` with Eigen and checks fresh flight IDs1..6.
Python syntax, shell syntax and whitespace checks pass.

The exact previously rejected plant CLI options now run successfully against
archived raw data into a **new** output, exit1 for measured behavior (not parser2):

| Measured archived behavior | Measurement | Unchanged limit |
|---|---:|---:|
| Idle command reversal rate, left hip pitch |15.495Hz|8Hz|
| Idle tracking RMS, waist pitch |0.234332rad|0.15rad|
| Support yaw peak/end |14.867/14.850deg|12/5deg|
| Support midpoint end displacement |0.062252m|0.03m|
| Foot anchor end displacement |0.115256m|0.03m|
| Foot heading end error |13.838deg|3deg|
| Maximum signed foot width |0.326739m|0.31m|
| Torso HOLD pitch median/peak |3.115/9.363deg|3/8deg|

Collision ledger passes with zero table/net/self contact rows. Reanalysis does
not amend the old verdict or establish a cause for these behaviors. Command
reversal rate is not physical sway frequency. New observation analysis of the
same archive passes the strict inspected0.570 plane check while retaining the
other failed checks. No physics/targets/actions/thresholds were tuned.

## Desktop continuation and current blocker

```bash
python3 "$HOME/workspace/HOPE-stationary-gru-eval/a3_deploy/a3_deploy_example/scripts/run_model19000_gate3_repair.py"
```

This is the complete resumable build/preflight/formal workflow, not a check-only
handoff. It verifies original artifact/runtime hashes and unchanged simulator
source/install, diagnoses the original lock path in hope, reuses existing mutable
build caches to rebuild changed Planner/Runner into **separate** installs/dist,
reruns full native parity and inspection, and performs Gate3 `--check`. No install
or dependency upgrade occurs. Old installs and failed evidence are hash-checked
before/after; source/binary/output bindings, commands and timings are retained.

It then runs an explicitly unscored first-six-clean-serve prefix, preserving20s
idle, clean timing, tail, no inter-ball resets/rescue, real physics and draining.
Structural preflight requires live Planner through drain, valid command audit,
an actual accepted/engaged/completed flight, matching HOME epoch/frame/calibration/
world/X across Planner and Runner, complete trajectories and all required report
evidence. Missing proof holds formal100. Measured behavioral report failures
remain visible; the preflight never claims certification. Only after structural
proof succeeds does the launcher run the unchanged fixed-home-100 scenario and
report actual `run.json.certification_pass`. It does not rerun finalized policy
failures to seek a pass. Dedicated ROS_DOMAIN_ID77 isolates the owned chain;
existing shared-runtime conflict/lock protections remain.

Actual sandbox invocation stops before all runtime stages: Podman exit1,
`chmod /run/user/1001/libpod: read-only file system`; launcher exit2.
Therefore **container-path errno/ownership/holder diagnosis, rebuilt native
execution, live six-ball preflight and new formal100 are pending**. No new
certification result is claimed. No permission/security workaround is attempted.

Evidence root:
`~/workspace/hope-stationary-gru-evaluations/20261108-model19000-gate3-infrastructure-repair/`.
Contains preservation inventories, exact archived-reanalysis command/log/JSON,
`strike-plane-contract-audit.json`, baseline assertion proof, host native test,
repair source binding and failed container-access command/log/receipt. No old
formal file or model package was overwritten. Training checkout is unchanged.
Normal local feature-branch commits/push only; DNS failures leave publication
unverified. No main/force/PR/release/professor writes or hardware connection.
