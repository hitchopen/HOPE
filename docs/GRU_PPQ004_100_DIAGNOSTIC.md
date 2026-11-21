# Authorized PPQ004 fixed-home-100 reference-sequence diagnostic

This is a diagnostic, not formal Gate3 certification. The user authorized100 serves after inspection of corrected-mode preflight `attempt-8aUCdxAm`. The legacy alignment-hold file remains as protection against older/default soft-contact launch paths; only explicit `--impact-100`, selecting ppq004 and scored-only, bypasses it after the infrastructure gate passes. It is not a generic hold override.

## Preflight findings

Independent checks of configuration, physical events, planner audit CSV, raw transport and policy trace passed:

- Frozen model29500 / gru-formal-2b8c09938a and matching ONNX identities.
- Explicit ppq004 mode and archived profile SHA256 `df55e82102e0d5624732ea7ea03f6d98053f45d7f036367e3df3551a1c88a6f9`; actual positive-force table contacts used the capped-slip handler and changed no positions.
- All five actual planner commands were received on time and consumed in34/34/34/35/34 positive-countdown actor ticks. Exact target-minus-current-base, world velocity and unclipped deadline fields matched. GRU reset count stayed1.
- Monotonic exposure stamps, max transport timestamp/reply skew5.090ms (<50ms); planner logs show zero drops/deadline misses/logger drops. Normal owned-process shutdown, no ERROR/FATAL/traceback; launcher completed cleanly.
- Peak two concurrent balls; five natural ground terminals; complete1kHz physical telemetry, empty remaining-live list, full final tail, zero runtime/numerical errors, falls or inter-ball resets. The validator does not use video or integration_preflight_pass as proof.

Physical hitting evidence is **5 balls with raw positive-force racket contact,0 reference-qualified contacts,0 legal landings,0 misses,0 recovery dwell passes**. These counts come independently from events and physical evidence and agree with summary. Infrastructure readiness is not successful hitting and does not relax qualification thresholds.

## Explicit full-run selection

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261105-221919-fixed-home-gru-diagnostic/run-reference-diagnostic.sh" --impact-100
```

This selects `phases=(scored)`, `table_model=ppq004`,100 scheduled balls. It does not pass `--preflight` to the evaluator and does not use old soft-table physics. Before container/runtime entry it prints checkpoint, bundle, table-impact mode, profile path/hash and scheduled count, verifies frozen inputs, and independently revalidates preflight8aUCdxAm into the new attempt directory. Errors stop the pipeline without fallback. Subsequent existing actor parity/reset and focused lifecycle checks remain in the launcher.

The audited immutable `fixed-home-100.yaml` resolver supplies every row:20s initial idle, six clean2.5s flights with1s pauses, then93 rapid1.15s flights and the reference final2.5s closure flight, no rapid pauses. Last launch nominal147.95s, slots finish150.45s, followed by at least3s completion/recovery tail and physical drain. Independent outgoing bodies survive handoffs. Robot and actor state carry throughout. Fall ends launches/rally; unlaunched serves are explicit, safe remaining flights drain with last controls held, and no reset/rescue occurs. The stock12s training timeout stays disabled only in this evaluation. Scored wall bound900s; total desktop pipeline2700s.

## Outputs and scoring

A fresh outside-Git `attempt-*/scored` stores exact ledger/configuration, per-ball CSV/JSON, original planner and transport diagnostics, contact/event/trajectory traces, actor inputs/actions/hidden evidence, strict physical-evidence report and summary. Reporting explicitly includes raw and qualified contacts, legal landings, net contacts, resolved misses, failed returns, planner failures/on-time commands, recovery dwell, fall/reset count, unlaunched and unresolved/interrupted outcomes. All100 scheduled serves remain the primary denominator. Full per-ball evidence preserves event counts and terminal positions; diagnostic aggregate counts describe balls with each outcome.

Recorded qpos/video rendering remains outside the timing-critical planner loop. Once simulation ends or falls, the launcher renders recorded actual states to `scored/diagnostic.mp4`, checks video decoding and updates the scored replay pointer only if that file exists. A fall/error remains an evaluation failure even when a partial video is rendered. No old video is substituted.

The reference scorer and recovery helpers are unchanged: global84 qualified contacts/legal/eligible landings, per-side49 assigned and41 qualified outcomes with5/6 rates; revocable .3s recovery criteria are preserved. No thresholds are lowered. Remaining differences are explicit: host table impact is1ms contact-edge quantized without native position snaps; racket contact remains MuJoCo soft mesh contact rather than native fitted velocity overwrite; completion/recovery timing uses a host deadline+.5s proxy, not native runner engage/preempt/completion; host GRU recurrent ABI and ROS direct-pose adapter differ from the native two-file runner. Therefore no formal Gate3 verdict is claimed, even if individual diagnostic rates improve.

## Actual execution status

Attempt `attempt-aJMqoQHA` printed model29500, ppq004, phase scored, scheduled100 and the correct profile hash; its independent infrastructure gate passed. It then stopped before simulation with `chmod /run/user/1001/libpod: read-only file system`. No100-serve per-ball outcomes or video exist yet. The desktop command above performs the remaining execution. No weights, training settings, dependencies, hardware connection or X11 security changes were made; training checkout remains clean.

Evidence: `/home/tianhe1/workspace/hope-stationary-gru-evaluations/20261106-ppq004-reference-100/` contains the hashed infrastructure audit, exact selected-source snapshot, launch receipt,13 passing gate/lifecycle/countdown tests and status. Five gate tests prove wrong physics, absent actor delivery, incomplete drain or planner errors fail; the real zero-qualified-contact preflight passes infrastructure even with its integration_preflight_pass flag set false in an isolated test copy.

Checkpoint SHA256: `b53a031b6d2dacceeebe7301412d3d2c5fc456f445c6aebb357e4159871683d0`. ONNX SHA256: `f50b0de8c79a83c3e48868d47bc8b922094353aebdea01540ec70b662541a58b`.
