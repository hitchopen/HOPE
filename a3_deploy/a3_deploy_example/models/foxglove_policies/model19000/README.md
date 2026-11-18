# Model 19000 GRU

Foxglove `model19000`: explicit-GRU receiver, run `gru-formal-2b8c09938a`, checkpoint iteration 19000
(native v2 handoff of 2026-11-07, integration review delivered 2026-11-14).

- `v2/policy/`: ONNX (`obs[1,110]`, `h_in[1,1,110]`, `time_step[1,1]` → `actions[1,31]`, `h_out` + embedded
  references) and its matched `deploy.yaml`. Keep the two files together.
- `v2/planner/model19000.json`: HDU Planner profile, `planner_profile(inspection, support_reference='live_base_v1')`
  from the native Runner inspection in `v2/inspection.yaml` (same generator as `model14200best`).

Receive only: there is no matched serve CSV, so the bundle is `receive_only: true` and OptiTrack only (Kernel is
refused). It needs a Runner with `pp_explicit_gru.hpp` (hidden state carried per control tick, reset at policy
entry, requires `--policy-native`). This is asset publication; physical behaviour is not qualified.
