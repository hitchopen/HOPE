# Fixed-home-100 reference-sequence GRU diagnostic

This is an adapted diagnostic, not formal Gate3 certification. The reference revision, full threshold matrix and source receipts are in [the audit](GRU_FIXED_HOME_100_AUDIT.md). The retired custom three-second protocol must not be used.

## Frozen policy and execution status

The latest complete checkpoint when selected at 2026-11-05 22:19:19 PST was **model_29500.pt**, from W&B run `gru-formal-2b8c09938a`, source commit `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`. Its SHA-256 is `b53a031b6d2dacceeebe7301412d3d2c5fc456f445c6aebb357e4159871683d0`. Stable descriptor metadata, source/copy hashes and ZIP CRCs were checked. Saved agent, environment and resolved task configurations were copied and hashed in `snapshot.json`.

All artifacts are outside Git in `/home/tianhe1/workspace/hope-stationary-gru-evaluations/20261105-221919-fixed-home-gru-diagnostic`. The pipeline freezes this policy for the entire diagnostic; later training checkpoints do not change it.

**Execution is blocked before export and live ROS preflight:** Podman reports `chmod /run/user/1001/libpod: read-only file system`. Attempt `attempt-SXJmGLZa` stopped immediately. No model29500 ONNX bundle, sequential parity result, actual planner preflight, 100-serve result or diagnostic video exists yet. There is no fallback to an old bundle. The model5300 unit-render fixture is explicitly not diagnostic evidence.

## Concurrent lifecycle and planner data flow

`concurrent_physical_balls.py` manages nine independent vendor ball bodies. A rapid handoff releases only the planner's incoming pointer; it never parks or changes the outgoing ball. Each ball retains its own contacts, trajectory and reference physical-evidence accumulator. Slots become reusable only after an actual floor terminal or bounded table-rest terminal. A full pool waits and records cadence deviation rather than truncating a flight. Every launched ball is drained; an unsafe or timed-out drain is unresolved, never successful.

Only the current incoming ball's measured pose is published at 200 Hz to the actual C++ ROS planner, which estimates its velocity. Robot base measurements are also published. Schema-2 planner target, velocity and deadline are decoded with timestamp/identity checks and at least 40 ms lead. A constant translation `[.5,.7625,.76]` converts reference table coordinates to host world coordinates. The target supplies the existing actor target columns and countdown; no actor inputs or actions are added. Normalization, joint mapping, 20 ms control, 1 ms physics and persistent GRU state remain unchanged. Initial idle uses a neutral expired target; a real accepted planner command supplies subsequent targets. Command revisions are logged.

The ROS bridge uses localhost discovery, domain 187, a unique namespace and graph guards; it launches only its owned planner process. It does not launch hardware nodes or command bridges. Wall/source clock skew and host pacing are bounded at 50 ms; violations stop the preflight rather than silently running stale planning.

## Preserved protocol and failure handling

The official resolver supplies the exact audited 100-row ledger: six clean slots, then 94 rapid slots, original mixed sequence and lane coordinates. Initial idle is 20 s; flight times are six times 2.5 s, 93 times 1.15 s, then 2.5 s; pauses are six times 1 s followed by zero. The last launch is nominally 147.95 s after start; nominal slots finish at 150.45 s. A final tail of at least 3 s and natural flight drain follow completion. No stock 12 s episode timeout is applied.

There is one initialization reset and no robot or actor-memory reset between balls. A fall, safety or numerical failure ends the rally and stops launches. Remaining safe physics drains with the last controls held; there is no rescue or new actor action. Unlaunched, interrupted and unresolved serves remain separate and in the primary 100-serve denominator. A separate five-shot integration preflight covers clean and rapid handoffs before the scored process starts. Wall limits are 150 s preflight, 900 s scored and 2700 s for the entire desktop pipeline.

## Scoring and remaining differences

The unmodified reference `PhysicalEvidenceAccumulator` and side-joining logic score qualified contact and landing evidence, using the audited 84 global, 41 per-side, 49 assigned-side and 5/6 rate thresholds. Raw MuJoCo contacts are reported separately. Nominal lane groups do not replace actual planner side assignment. Per-ball CSV/JSON, planner diagnostics, event and trajectory streams, actor inputs/actions/hidden-state evidence, cadence and summary are saved. Summaries include all scheduled serves, explicit conditional denominators, streaks, the reset-free segment and post-failure exclusions.

Remaining differences prevent formal equivalence:

- The host GRU ABI is `obs,h_in -> actions,h_out`; the native runner does not implement this recurrent state contract. Native engage/preempt/completion certification is unavailable.
- Host geometry, soft-contact solver and established aerodynamic stepping differ from the native plant. Native `mujoco_sim_module.cc` applies fitted contact impulses and repositions the ball outside the racket/table. The host instead retains actual MuJoCo contact dynamics without a collision-time velocity overwrite or snap. Consequently raw contact may fail strict reference edge-velocity qualification. Thresholds are not relaxed.
- Host ball-slot lifecycle uses measured floor/table-rest termination rather than native flight packets. Pool waits and final drain are explicit, not assumed equivalent.
- Recovery reuses the reference revocable 0.3 s dwell criteria (position .05 m, EMA speed .12 m/s with alpha .2, tilt 12 degrees, yaw 5 degrees, pelvis .8 m). Its completion clock is a documented host proxy based on actual planner deadline plus the saved .5 s post-strike duration; native completion/READY equivalence is not established.
- Direct-pose schema-2 ROS transport and translated host HOME/frames differ from native transport and shared lifecycle. Reference native plant tracking and complete reports remain unavailable.

## Verification and desktop command

56 lifecycle/regression/reference tests and five command-contract tests passed. They cover independent handoff state, planner selection, full-pool behavior, natural slot reuse, contact isolation, end-of-rally drain and unchanged recurrent contracts. Unit fixtures use the previously verified model5300 bundle only. A 15-frame offline rendering smoke test passed; it contains no model29500 inference or actual planner session.

Run from a normal Ubuntu desktop terminal:

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261105-221919-fixed-home-gru-diagnostic/run-reference-diagnostic.sh"
```

The launcher prints checkpoint29500 and the new bundle path, validates hashes, exports inside existing grasping, independently checks loaded actor initializers, runs sequential ONNX/PyTorch parity and reset checks, builds the isolated existing planner in hope, runs a separate preflight, then the bounded diagnostic only if checks pass. It stops on errors. It makes no dependency changes. Training remains untouched.

Video rendering happens after the planner exits, from saved actual qpos and measured trajectories, avoiding capture latency in the live planner clock. This is an actual-state replay, not new simulated scoring. After a scored recording exists, replay it with the same command plus `--replay`. Optional `--view` opens a live viewer during a new bounded run; X11 security is never changed. No previously recorded movie is substituted if the selected attempt fails.

## 2026-11-06 alignment hold

The subsequent desktop preflight attempt-z8SGbdOv exported/verified model29500 and ran all five physical serves with the actual planner; all missed. Its evidence supersedes the earlier preparation-only status above. [Alignment investigation](GRU_STRIKE_ALIGNMENT_AUDIT.md) documents exact reconstructed geometry, the demonstrated prediction/soft-contact mismatch, the corrected actor-countdown cap, and the held100-run. Use the new `--alignment-preflight` option for the same five-serve live repeat; default scored continuation is blocked by the alignment hold. No after-fix live result is available in the sandbox.
