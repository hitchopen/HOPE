# Physical accuracy evidence

**Gate P: NOT RUN — independent recordings/annotations are unavailable.**
No dataset IDs or physical scores are supplied. Historical same-sensor C3D and
synthetic DOP853 fixtures do not satisfy independent physical truth.

`compare_recordings.py` runs public HOPE V2/progressive/fixed-one-shot ball replay
executables on the same acquisition samples/cutoffs. It writes `matched.jsonl`,
`errors.csv`, `tables.json`, `manifest.json`, and three figure families per spin,
event and shot-type stratum. One-shot contributes only actual trigger points;
there is no continuous one-shot curve. Missing epochs are retained explicitly in
new-revision coverage, which is not the lifetime availability of an earlier
one-shot prediction. Common-valid-pair error comparisons and paired whole-shot
bootstrap are separate from each method's own valid-output statistics.

Required input JSON structure (values below describe fields, not a dataset):

```text
frame_id: "table"
clock_mapping_id: qualified source mapping ID
registration_receipt_sha256: independently reviewed table/plane/clock receipt hash
independent_truth: true only with independent evidence
code_pins: {legacy_one_shot, progressive_batch, v2_ckf}: exact source commits
flights[]:
  session_id, ball_id, flight_id, direction (+1/-1), shot_type, split="held_out"
  complete_history: explicitly supported boundary/history status
  observations[]: source_ns(int64), position[3], omega_valid, omega_rad_s[3]
                  optional common source spin_mode, independent of model quality
  information_cutoffs_ns[]: actual shared acquisition stamps (default all)
  b1_ns, b2_ns: independently annotated occurrence times when available
  truth_events[]: event_id, kind, ordinal for table bounce, truth_available,
                  position[3], velocity[3], arrival_ns, optional post_velocity[3]
```

The runner matches physical identities and event order, never nearest forecast
error. For B1/B2, it associates by independently annotated chronological ordinal;
plane predictions use each method's actual selected prediction. Legacy B1/B2
exports are forward diagnostics from its actual estimated state, not additional
one-shot triggers or commands. Missing event truth is excluded from error scores
and counted separately, rather than treated as automatic failure/pass.

Use `--allow-synthetic` only to test tooling: every figure is watermarked, and the
receipt remains `NOT_RUN_SYNTHETIC_TOOLING`. The unit test exercises all three
figure files and matching/trigger checks. Raw private captures stay outside Git.

Figures:

1. `01_plane_error_vs_lead.png`: Y/Z/3D position, velocity and arrival errors at
   actual source epochs, with method/sample/shot counts.
2. `02_bounce_aligned_error.png`: prediction epochs relative to true B1/B2.
3. `03_coverage_latency_uncertainty.png`: revision/target coverage, offline compute
   latency and conditional uncertainty; actual ROS latency is measured separately.

No physical quality threshold is declared passed by the tool. Independent-data
scores require interpretation of matched subsets, mode/phase strata, rejection
rates and the documented unmodelled errors before adoption decisions.
