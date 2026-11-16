# 2026-11-08 — model19000 native GRU/reference implementation

Evaluation worktree: `~/workspace/HOPE-stationary-gru-eval`, existing branch
`feature/stationary-gru-mujoco-eval`, base
`822c0c4263d96d29635bba57feb59e87473904d7`. Initial worktree was clean.
No applicable AGENTS.md was found in parents or relevant subtrees. Origin fetch
and push are `https://github.com/1786089174zth-cell/HOPE.git`; no URL mismatch.
The cached origin has only the training branch; live `ls-remote`/fetch fail with
`Could not resolve host: github.com`. Remote evaluation-branch divergence is
unknown; no remote integration or force push is attempted.

Training worktree is separately verified at
`33c609022efbb4e85fe6a58a5a8947b0fbd46c76`,
`feature/stationary-rally-gru`, clean and unchanged. Run identity remains
`gru-formal-2b8c09938a`, checkpoint iteration19000.

## Implementation

The prior metadata-only candidate and all original model19000 files are
unchanged. New sibling:
`a3_deploy/a3_deploy_example/models/model_19000_native_v2/`.

* Original checkpoint SHA256:
  `744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0`.
* Original actor SHA256:
  `f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda`.
* New reference-bearing ONNX SHA256:
  `2b2a029b450fb8601ecf34eefdbd3df6ab9f8597a33593dc9b7d6419fcd34d27`
  (1,435,445 bytes).
* New deploy.yaml SHA256:
  `160f282fdb95d4ca5d0df5222897a203150152248fcc364f8ba723086357f3fc`
  (10,654 bytes).

Production `PpOnnxPolicy` now delegates explicit GRU graphs to `PpExplicitGru`.
It owns the110-float hidden state, validates graph interfaces/results, caches
one actor advance per policy tick, rejects changed duplicate calls and treats
reference queries as read-only. Main Runner resets state on genuine policy
session admission; SHADOW/MOTION transitions inside the session, HOLD, serve
handoffs and recovery do not reset it. Legacy static-stand bypass is explicitly
unsupported for GRU; Gate3 already uses `--policy-native`. Existing feed-forward
Run behavior is preserved. Inspection reports recurrent support and zero
advances. Unsupported recurrent interfaces fail closed.

The wrapper retains every original graph field/node/initializer byte and
appends genuine reference tensors, Gather clock nodes and actual exported
metadata. Metadata comes from the real `attach_onnx_metadata` function applied
to saved config/checkpoint managers, not fabricated fields. Exporter,
observation-contract helper and motion selector are unchanged from the frozen
training source. Provenance/config/software/source hashes are in the manifest.

References use pinned FH/BH NPZs (SHA256
`a6c68513720b12b168379cd6fa13f8b77607b4fa0bf7e828c4e1d81eda6f2094`,
`67d04e13deeed068bdb003e379e18330dcd29210d280188fab7af26c0764eaac`),
107+109frames,50Hz, saved tracked-body indices, raw clip-world wxyz frames,
and frozen strike phases0.3868/0.4444. Strike frames41/155. Standard integer
lookup and native clock clamping remain unchanged; no interpolation/dummy
outputs/model21800 references. Six reference arrays are embedded, so runtime
needs only the two new policy files. Nominal defaults/wire damping stay distinct
from historical host randomized defaults/total damping. Original observation
normalization, action decoding, feedback and passive-head behavior remain owned
by their existing graph/adapters.

## Verification actually executed

* Graph export with existing host venv: pass.320 sequential original/wrapped
  ORT1.31.1 inferences, real archived observations, nonzero initial h and resets32/
  192: max action0, hidden0; atol2e-5/rtol2e-4.
* All six reference streams match actual selected NPZ arrays exactly at
  -1,0,1,106,107,215,216,226; maximum error0. Original graph bytes retained.
* Compiled `native_parity.cpp` with g++ and existing ORT C++1.19.2 SDK, using
  the production-shared state class.320 steps pass: max action1.90735e-6,
  hidden0, reference0. Nonzero state, reset repeat, reference-query/duplicate
  no-advance, clock-handoff carry, invalid calls and all216 native clock frames
  pass. This host build deliberately excludes `HOPE_VERIFY_FULL_POLICY`:
  **it is component evidence, not a full Runner loading result**.
* YAML/settings comparison: all settings equal original nominal deployment
  YAML except new provenance/hash;31-slot bijection/order,110-D observations,
  0.02s control, default/scale/kp/wire-kd/safe-limit metadata binding pass.
  Largest tested value mismatch4.57e-9. YAML parses and ORT validates graph.
*47 files across original package/prior candidate byte-preserved, checkpoint
  rehashed after work. Training checkout remains clean. No old result changed.
* Both Python scripts parse; `git diff --check` passes.

## Full native build/inspection/formal status

Actual launcher invocation exits2 at `container-access`; Podman exit1:
`set sticky bit on: chmod /run/user/1001/libpod: read-only file system`.
No permissions/security changes, no backend initialization, no hardware,
training, dependency installation or simulation. No existing Runner binary
was relabelled as the new build. **Full Runner build/inspection, full-loader
parity/feed-forward regression and formal Gate3 remain NOT RUN**, certification
unknown, not failed policy measurements.

Unlike the previous check-only handoff, new `run_native_gate3.py` implements
the complete guarded sequence: existing hope access; separate ROS closure,
native x86 package/full-loader parity target and current instrumented MuJoCo
build/install; full native sequential/reset/reference/duplicate tests plus legacy
feed-forward fixture; actual new binary `--inspect-policy`; documented Gate3
`--check`; then real headless unchanged fixed-home-100. No host evaluator fallback.
It preserves initial idle/clean/rapid/tail timing, no rescue, concurrency/drain
and acceptance thresholds. Existing Gate3 global conflict/lock/owned PID cleanup
remains authoritative.900s external timeout sends SIGINT for bounded cleanup.
Finalized failed certification is retained, not automatically retried or tuned.

Stage receipts bind source content and output/library hashes, skip only verified
completed stages, record timing/commands/logs, and fail before subsequent stages.
Fresh run directories retain failed checks. Actual native binaries, runtime
configuration, source bindings and formal snapshots/reports remain outside Git.
Source changes after a build require a fresh output directory. No package
upgrade or system-permission workaround is included.

Evidence: `~/workspace/hope-stationary-gru-evaluations/20261108-model19000-native-v2/`,
especially `candidate-verification.json`, `native-state-parity.json`,
`configuration-and-preservation-check.json`, preserved-file inventories,
`.f32` observation/state/reference fixtures, compiled native component,
`source-binding.json`, `container-access.command.json`, `.receipt.json`, `.log`.
Candidate README documents the exact desktop command and all hash/contract details:

```bash
python3 "$HOME/workspace/HOPE-stationary-gru-eval/a3_deploy/a3_deploy_example/models/model_19000_native_v2/run_native_gate3.py"
```

Code/bundle/docs are committed on the existing feature branch only. Publication
uses normal Git and hooks; network failures leave commits local and must not be
described as a verified remote publication. No PR/release/main/professor writes
or Google Drive handoff changes. A successful future simulation still would not
establish real-robot performance.
