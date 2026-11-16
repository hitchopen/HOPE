# 2026-11-08 — supported 26-ball infrastructure preflight

Existing `feature/stationary-gru-mujoco-eval`, base18d5ef0, personal HOPE origin.
Initial worktree clean. The six-ball preflight override was an implementation
error: the shared launcher requires an even count>=26. Its guard is unchanged.

`run_gate3.py --integration-preflight` now requires the existing canonical
`config/gate3/scenarios/fixed-home.yaml` (26 balls), uses its resolved environment
verbatim and never overrides the count, timing or thresholds. This includes six
clean serves followed by20 rapid serves, original20s initial idle and final tail.
The separate formal call still selects `fixed-home-100.yaml` unchanged.

`run_model19000_gate3_repair.py` uses the standard26 scene for preflight and
checks scene hash, scheduled/actual counts, per-ball telemetry IDs,26 planner
audit rows, valid commands, matching Planner/Runner HOME and command identities,
actual swing engagement/completion, report evidence and complete flight draining.
Infrastructure status, execution readiness, behavioral outcomes and certification
are separate fields. Falls/early termination are labelled behavioral; misses and
measured plant-threshold failures are retained rather than called infrastructure
failures. Formal100 remains held without the required execution/readiness proof.

## Resume without rebuilding unchanged native components

Old `repair-source-binding.json` and six-ball command/log/receipt/run directories
are untouched. New orchestration uses `repair-source-binding-26.json` and distinct
`integration-preflight-26-*`, `gate3-check-26-resume-*`, profile-check and proof
names. The failed attempt
`integration-preflight-20261108-164659` and its wrapper files have a before/after
SHA256 inventory outside Git. No model, checkpoint, physics file, scoring limit,
shared-launcher guard, native source or training file changed.

Verified native stages may migrate only across this three-file Python
orchestration/test correction. Reuse requires their original completed receipt,
unchanged native/config/model sources, exact generated argv equality and every
output hash. A native source/config change or changed command/output invalidates
reuse; evaluation stages cannot migrate from the six-ball source binding.
The new binding does not overwrite the old binding or claim old evidence came
from new code.

The actual plan-only run verified these skips:

* Original hardware-lock diagnosis.
* Native identity contract test.
* Planner build and Runner build.
* Native sequential/reset parity and native inspection.

It plans fresh container access, Gate3 check and profile verification, then the
26-ball preflight and readiness proof. Only if ready does it run formal100.
No compilation is required for this Python-only correction. Original native
parity remains320 steps, action max error1.90735e-6, hidden/reference max0.

```bash
python3 "$HOME/workspace/HOPE-stationary-gru-eval/a3_deploy/a3_deploy_example/scripts/run_model19000_gate3_repair.py"
```

Optional `--plan-only` verifies reuse and prints stages without executing builds
or simulation. Default execution remains the complete guarded workflow, not a
check-only handoff. Stage progress/timings and exact commands remain archived.

## Verification and observed status

48 focused tests pass across the repair, fixed-HOME profile and Gate3 composition
tests. They cover canonical26 versus unchanged formal100, rejection of6, complete
protocol environment equality, readiness gates, behavioral-fall classification,
and cache invalidation on native-source/command/output changes. The real shared
entry composition tests include the supported26 sequence. Whitespace checks pass.

The dry-run plan executed no native build, ROS/Planner process or MuJoCo
simulation. There is **no new simulation or certification result**. The failed
six-ball attempt remains a pre-simulation infrastructure failure. Model19000's
checkpoint/ONNX/YAML and all previous results remain unchanged. Evidence is in
`~/workspace/hope-stationary-gru-evaluations/20261108-model19000-gate3-infrastructure-repair/`,
including `six-ball-attempt-preservation-before-26.json` and `resume-26-plan.json`.
Normal commits/push only to the same personal feature branch; no main, force,
professor writes, dependency upgrades, hardware or training operations.
