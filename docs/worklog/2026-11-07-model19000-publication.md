# 2026-11-07 — model19000 alongside the final actor

Repository: `1786089174zth-cell/HOPE`; existing branch `feature/stationary-gru-mujoco-eval`; base `44945d12c4354e68d80020af868dff07a88b0f0a`. Explicit user instruction authorizes this same feature branch only. No applicable AGENTS.md found. Origin fetch/push both `https://github.com/1786089174zth-cell/HOPE.git`. Initial local working tree was clean. `git fetch origin` failed with `Could not resolve host: github.com`; live remote changes cannot be inspected until DNS works. No remote changes were integrated from stale refs; no force push or hook bypass is permitted.

Added a self-contained model19000 package and dedicated launcher; existing model29999 files/launcher remain byte-identical to the base. Shared README/comparison documentation links both. No training or evaluation rerun, ROS rebuild, dependency change, video alteration, hardware connection or new branch.

## Identity and artifacts

Run `gru-formal-2b8c09938a`; training source `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`; exact iteration19000, no substitution. Local archive `../hope-stationary-gru-evaluations/20261106-eight-checkpoint-comparison/model_19000`; original diagnostic `attempt-kaub5j2j`. Original ONNX, parity vectors and actor-load verification reused unchanged. Runtime contract compared against model29999 before copying its portable configuration/runtime. Only provenance paths and publication metadata are adapted in the original model19000 manifest; no actor weights change.

- Checkpoint SHA256: `744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0` (recomputed from full local PT).
- ONNX SHA256: `f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda` (recomputed; matches archived actual initializer verification).
- Actor parameter fingerprint: `d3c3ac40dc977606a0126687cd03bb2177a6eeee2e5bd88469abf78a3f6e86c8` (archived verification, not a fresh Torch reload).
- ZIP SHA256: `8a374c710297fca2b65b4ad109b2170586fb5ba6ffe06cec8e4ce4df9ac98f4d`; 1,202,639 bytes.
- PPQ004 profile SHA256: `df55e82102e0d5624732ea7ea03f6d98053f45d7f036367e3df3551a1c88a6f9`.

The original >100MiB PT remains local, not committed. Existing eight videos are reused without copies/reencoding. Per-file package manifest/SHA256SUMS include runtime/config hashes. Private full YAML/logging configuration remains local; immutable configuration hashes are included.

## Verification

Existing host `../nohope/.venv/bin/python`, unchanged dependencies:

```bash
python -m pytest -q hope_training/whole_body_tracking/tests/test_published_model19000.py hope_training/whole_body_tracking/tests/test_published_model29999.py
python hope_training/whole_body_tracking/scripts/run_model19000_mujoco.py --verify-only --output-root ../hope-stationary-gru-evaluations/20261107-model19000-publication
```

Six tests passed. Extracted ZIP runs standalone outside the checkout:64 sequential steps and exact zero-reset repeat passed. The new launcher rejects a model29999 bundle before starting anything. Fresh host64-step parity max absolute action error `2.86102294921875e-06`, hidden error `4.6193599700927734e-07`; finite actor/controller mappings and reset checks passed. Evidence: `../hope-stationary-gru-evaluations/20261107-model19000-publication/model19000-ibgbgsyr/`.

Recorded diagnostic results are unchanged:100 launched/resolved,100 raw contacts,0 qualified contacts,0 legal landings,1 fall at152.96s after flights resolved in the final recovery tail,0 unlaunched/unresolved/resets. Reference-sequence diagnostic, not formal Gate3 or real-robot certification. Concurrent balls, persistent GRU, no-rescue fall behavior, PPQ004 mode and scoring remain unchanged.

Full diff/staged files, package/video hashes and sizes, links and excluded-artifact scan are checked before commit. Model29999 and training checkout remain unchanged. Normal fetch/push is attempted without force or hook bypass; network errors are retained outside Git in the dated evidence directory. A remote publication is not claimed without a successful push.
