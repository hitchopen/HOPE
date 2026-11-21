# Eight-checkpoint reference-sequence diagnostic (not formal Gate3)

Run `gru-formal-2b8c09938a`; training source `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`. Exact requested checkpoints were available; no substitutions. Same fixed-home-100 ledger/seed0, 20s initial idle, PPQ004 actual table-contact law, persistent robot/GRU state and fall-ending rule. Videos are original unchanged bytes, 800×500,20fps,1× playback and identical camera. The first serve is at **00:20** in every published clip; archives retain the idle. `ffplay -ss 20 <video>` skips it without editing the archive.

| Checkpoint | Launched / resolved | Raw contact balls | Qualified | Legal | Falls (time) | Unlaunched | Termination | Video |
|---|---:|---:|---:|---:|---|---:|---|---|
| 1000 | 2 / 2 | 0 | 0 | 0 | 1 (23.76s) | 98 | fall before third serve | [MP4](../../assets/gru-formal-2b8c09938a/model_1000.mp4) |
| 5000 | 100 / 100 | 33 | 0 | 0 | 0 | 0 | completed sequence | [MP4](../../assets/gru-formal-2b8c09938a/model_5000.mp4) |
| 10000 | 100 / 100 | 100 | 0 | 0 | 1 (153.14s) | 0 | fall in final recovery tail | [MP4](../../assets/gru-formal-2b8c09938a/model_10000.mp4) |
| 15000 | 100 / 100 | 99 | 0 | 0 | 1 (153.40s) | 0 | fall in final recovery tail | [MP4](../../assets/gru-formal-2b8c09938a/model_15000.mp4) |
| 19000 | 100 / 100 | 100 | 0 | 0 | 1 (152.96s) | 0 | fall in final recovery tail | [MP4](../../assets/gru-formal-2b8c09938a/model_19000.mp4) |
| 23000 | 100 / 100 | 100 | 0 | 0 | 0 | 0 | completed sequence | [MP4](../../assets/gru-formal-2b8c09938a/model_23000.mp4) |
| 27000 | 2 / 2 | 1 | 0 | 0 | 1 (24.22s) | 98 | fall before third serve | [MP4](../../assets/gru-formal-2b8c09938a/model_27000.mp4) |
| 29999 | 100 / 100 | 100 | 0 | 0 | 0 | 0 | completed sequence | [MP4](../../assets/gru-formal-2b8c09938a/model_29999.mp4) |

`COMPLETE_100` means100 scheduled serves launched and resolved without terminating the rally; it does **not** mean100 successful returns. Every reference-qualified contact/legal count and longest success streak is0. The three late falls occurred after all100 flights resolved but before the final recovery tail ended. The two early falls launched/resolved2 and left98 unlaunched; no reset/rescue filled the missing count. All unresolved counts are0. No infrastructure failures are relabelled as policy falls.

Raw contact means a measured positive-force MuJoCo racket contact; strict reference qualification also checks edge velocities/flight/landing evidence. Actual first-return plane markers or raw table impacts do not replace reference qualification. Training virtual legal-return metrics are analytical at-strike task metrics and are not these physical outcomes.

[Machine-readable results, exact hashes and video sizes](comparison.json). Each checkpoint directory preserves the original summary, per-ball CSV/JSON, strict physical evidence, video receipt and weight/parity/reset verification. Original large state/trajectory/ROS traces and PT checkpoints remain in the local comparison archive; these are not required to play the published videos or load the final actor.

## Source provenance

The videos predate publication commits. [Historical input hashes](historical-evaluated-source-hashes.json) preserve evaluated source/binary hashes with root aliases, plus the SHA256 of the original input manifest. [Historical snapshot](historical-source/) contains the unchanged batch/export-verification/render snapshot. Portable publication entry points, manifests and paths were added afterward; no old video is claimed to have been produced by the publication commit. See the dated publication worklog for final code/artifact commit separation.

## Physics and lifecycle limits

Table impacts apply the archived PPQ004 capped-slip shell-inertia law only at a real positive-force finite table contact, with no position snap. Its first-step response replaces the soft table impulse; that ball's table constraint is suppressed only until natural separation (bounded at50ms) to prevent double impact. Timing is1ms quantized. Racket contacts remain actual MuJoCo soft mesh contacts; native fitted racket impulses/position snaps are not reproduced. This is one reason raw contacts cannot be promoted to qualified success.

Completion/recovery uses a host deadline+.5s proxy and strict revocable .3s dwell criteria, not native runner lifecycle telemetry. The GRU has explicit persistent hidden state and no side channel; its ABI is incompatible with the stock feed-forward native runner. Exact nominal robot state, rather than training randomization/noisy mocap/PhysX drives, is used. This is **not formal Gate3 certification**, an identical-training-physics claim, or a successful real-robot deployment claim.

## Published recurrent actors

Exact [model_19000 bundle and commands](../../../a3_deploy/a3_deploy_example/models/model_19000/README.md) and [final model_29999 bundle and commands](../../../a3_deploy/a3_deploy_example/models/model_29999/README.md) are available side by side. The new model_19000 package reuses its archived verified ONNX/parity vectors; no evaluation or video was rerun. Runtime, planner, PPQ004 physics, serve ledger, persistent hidden-state rules and scoring match the published evaluator. Model_19000 ends on its recorded152.96s fall in the final tail;100 resolved balls is not100 legal returns. Model_29999 files and launcher are unchanged.
