# Original SuperGate research scripts

These are byte-identical source files from the 2026-11-08/09 SuperGate research
workspace. `SOURCE_MANIFEST.json` records each original relative path and SHA256,
including the original outer/mixed-long datasets now stored in `../data/`.
The scripts have not been rewritten to use the portable runner. Python parsing,
shell syntax and source/data hashes are checked without executing their workloads.

## Entry points and related sources

| Purpose | Original files |
| --- | --- |
| Physical flight integration and seed grids | `flight_kernel.py`, `build_physical_grid*.py`, `bank_eval.py` |
| Initial origin/angle suite | `build_super_gate3.py`, `validate_super_gate3.py` |
| Main factorial suite and freezing | `build_super_factorial.py`, `fill_factorial.py`, `freeze_super_dataset.py` |
| Earlier factorial draft | `build_super_factorial_draft.py` (historical draft, not the frozen suite) |
| Slow-tail construction | `build_slow_tail.py` |
| Original full-runner suite orchestration | `prepare_gate3_suite.py`, `run_gate3_suite.py`, `run_gate3.py`, `pp_gate3_ball_launcher.py`, `run_isolated.py`, `isolated_entry.py`, `hitter-suite.sh`, `rally-suite.sh` |
| Native batch revisions | `internal_batch/`, `internal_batch_v2/`, `internal_batch_v6/`, `internal_batch_v8/`: original Python drivers, C++ backend, physics and clock headers |
| Outer generation | `planner-range-v1/solve_outer.py`, `planner-range-v1/make_data.py` |
| Mixed-long construction and support sweep | `planner-range-v1/range_sweep.py` (`mixed_data()`), `planner-range-v1/support-*.yaml` |
| Holdout construction used by mixed-long | `best-v2-research/make_data.py` |
| Result analysis | `analyze_super_batch.py`, `analyze_detailed.py`, `report_detailed.py`, `finalize_super_evidence.py`, `planner-range-v1/extended_analysis.py`, `foot-recovery-v1/analyze_batch.py` |

The original constructor inputs (`expanded-horizontal-ledger.json`,
`super-gate3-ledger.json`, `super-factorial-requested.json`,
`super-factorial-selected.json`, `geometry-audit.json`, `branch-gate3/fixed-home.yaml`)
are included. The generation attempts are small construction records, not
per-timestep simulation traces. Every copied file is hash-verified against its
source; frozen outer/mixed-long manifests are preserved without regeneration.

## Running and interpretation

For a new evaluation, use the portable `../build.py` and `../run.py` instructions
in the [main README](../README.md). Both new datasets load through `--dataset`.

Original scripts retain absolute workstation paths, old `agi/` repository
layout, model names, fixed CPU affinities and historical directory assumptions.
Some execute work at import time, write outputs or expect an existing result to
finish. They are a source snapshot, not an installation command or self-contained
historical runtime. To use an original entry point, first work from a copy and
adapt its paths/dependencies. Generation additionally uses SciPy/Numba; analysis
uses pandas/matplotlib. Supply matching policy weights, XML/assets, ready states,
runtime sources/native libraries and any referenced earlier inputs/results.
Those model packages, binaries and run-result directories are not included here.
The local Git attributes allow original extra EOF blank lines to preserve hashes.
The included support profiles are offline test inputs, not deployment commands.

The historical outer/mixed-long sweep calls `internal_batch_v6/run.py` with
`PP_GATE3_SUPPORT_PROFILE` and evaluates continuous sequences. The portable
runner currently uses the diagnostic v2 backend with its documented fixed
admission profile. Loading the same questions through that runner does not
reproduce the v6 sweep or its reported outcome. Likewise, the original extended
analysis uses its own fall thresholds; do not conflate it with the portable
summary's fall indicators. Original native drivers can write raw time series;
only the portable runner defaults to no raw traces.

Outer covers 67 of 81 requested cells (402 shots), with 14 generation holes
listed in `planner-range-v1/coverage-holes.json`. Mixed-long contains 402 outer,
486 fresh holdout and 312 main shots, shuffled into six 200-shot sequences. Its
range selection was informed by outcomes; it is empirical sampled simulation,
not an independent validation set or proof of hardware qualification.
