# Model 26500

Foxglove `model26500`: feed-forward `ball_clock_110_v1` / `rally_final_v2` / `fixed_session_anchor_v1` receiver
(checkpoint SHA256 in `manifest.json`), installed 2026-11-15 as a separate option.

- `v1/policy/`: ONNX and its matched `deploy.yaml`; keep them together.
- `v1/planner/model26500.json`: HDU Planner profile from `planner_profile(inspection, support_reference='live_base_v1')`.

Receive only (no matched serve CSV), OptiTrack only. Runs on the field Runner already used by `model14200best`.
Local Gate3 fixed-home 26: 26/26 contacts and legal returns, 0 falls, but certification FAIL (yaw rate after
swings, actuator saturation peak 1.62x, HOME support frame, idle smoothness). Physical behaviour not qualified.
