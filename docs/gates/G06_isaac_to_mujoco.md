# G06 Isaac-To-MuJoCo Parity

> For commands checked against the current checkout on 2026-10-11, use
> [deploy_and_gate3.md](../operations/deploy_and_gate3.md). Dated reports and
> legacy wrappers below retain their original scope; they do not replace the
> current two-file scene workflow or establish a new run's verdict.

Status: Partial (parity procedure operational and used to gate the 2026-07-02 sim-to-real; formal per-checkpoint acceptance thresholds still to be recorded)

## 2026-07-18 RallyV13 step 4: fail-closed export and isolated Gate3 plumbing

RallyV13 now has a deploy contract, but still has no checkpoint or ONNX. The exporter recognizes
V13 by its live `post_swing_xlock` term (not by `hasattr` on the inherited-but-disabled V12 waist
term), validates all V13 reward parameters against the resolved saved run and stamps recipe
`rally_v13`, version `1`, runtime `rally_final_v2`, status `gate3_candidate`. The Python MuJoCo
evaluator and C++ loader require the exact V13 metadata. They reject missing fields and also reject
the obsolete V11/V12 `hitter_pure_joint_qdes_max_blend` and
`hitter_pure_waist_qdes_saturation` keys.

An isolated source runtime config and versioned wrapper now exist:

- `a3_runtime_config.pingpong.v13_candidate.yaml` points only to
  `assets/a3_runtime/models/model_v13_candidate.onnx`;
- `pp_gate3_rally_v13.sh` requires that packaged config and a loader-validated V13 model;
- the wrapper reuses V12's exact 12 serves and geometry, selects report mode `rally_v13`, and runs
  the planner audit under contract `rally_v13` before the common engine.

Host verification passes: 38 focused training/export/MuJoCo contract tests, 21 planner/report
tests, 3 C++ runtime-pairing tests, shell/Python syntax, V13 report auto-discovery, and standalone
compilation of the ping-pong main translation unit. The V13 planner wrapper preflight passes the
runner-harness layer with 12/12 serves and two positive-main transitions, while raw planner
velocity coverage and physical contact/landing remain explicitly unproved. The full CMake target
is currently blocked before the changed ping-pong code by the host's missing `zmq.hpp`; the changed
translation units compile independently. No export, package, MuJoCo Gate3 or promotion was run.

## 2026-07-18 RallyV13 steps 1–3: failure manifest, pre-export eval gate and training task

V12 `model_12700` is now an immutable negative baseline. The machine-readable manifest is
`hope_training/whole_body_tracking/analysis/rally_v12_gate3_failure_manifest.json`; it pins the
checkpoint/ONNX/report hashes, all observed per-serve/per-joint failures, V13 selection margins and
the unchanged final `159/159` strict requirement. The exact 12-serve Isaac input is separately
frozen in `analysis/rally_v13_gate3_sequence.json` and SHA-pinned by the manifest.

`eval_deterministic.py +json_out=...` now emits checkpoint SHA, distribution, exact metrics and
all action-column pre-clamp q-des counts/maxima against Isaac soft and official hard limits. Q-des
is split into whole-tail, exact/pre/post and FH/BH scopes. With
`+gate3_sequence_spec=analysis/rally_v13_gate3_sequence.json`, MotionCommand and
RacketTargetCommand use synchronized cyclic serve indices rather than random clip/target draws and
the report includes all 12 per-serve rows. The eval-only sequence fields default empty and are not
present in the training YAML.

Before any V13 export, run core/planner/mix plus the sequence report and require:

```bash
python3 scripts/check_rally_v13_acceptance.py \
  /tmp/v13_core.json /tmp/v13_planner.json /tmp/v13_mix.json /tmp/v13_gate3_sequence.json \
  --json-out /tmp/v13_checkpoint_acceptance.json
```

The checker fails on a missing distribution/sequence, SHA mismatch, insufficient env/step/tail
coverage, missing/non-finite metric, any hard q-des clip, any per-serve miss, or mixed checkpoint
hashes. It does not export. RallyV13 training code is ready but unexecuted; G06 remains `Partial`.
Raw planner velocity coverage and physical contact/landing remain unresolved and must not be
conflated with the demo-substituted deterministic sequence.

The host hard-limit parity test needs the ignored generated Isaac A3 URDF. Restore/check it with
`scripts/prepare_a3_isaac_asset.py --force` then `--check` as documented in
`docs/operations/setup_local_sync.md`; the authoritative comparison targets remain the tracked
official `a3_pingpong.xml` and `pp_joint_limits.hpp`.

## 2026-07-18 RallyV12 `model_12700` deterministic selection and strict Gate3

The operator stopped the fresh V12 run at `model_12700`. Selection followed the required order
rather than using the WandB curve: `model_8000/10700/10900/12100/12700` were each evaluated with
512 environments, seed 0, 1200 steps, tail 400 and zero action noise on `core`, `planner` and `mix`.
The reproducible evaluator form (repeat for each checkpoint and distribution) is:

```bash
hope_isaac_py scripts/eval_deterministic.py task=HOPEPingPongHitterPureRallyV12 algo=ppo \
  headless=true num_envs=512 +seed=0 +steps=1200 +tail=400 '+noise_scales=[0.0]' \
  +vel_eval_distribution=mix checkpoint=<run>/model_12700.pt \
  motion_file=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz \
  motion_file_2=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz
```

`model_12700` had the strongest worst-distribution exact composite/side rates (`0.9921/0.9886`),
the smallest exact strike-x maximum (`0.0184 m`) and the smallest average exact waist-clamp
fraction (`0.0105`). Core/planner/mix composite was `0.9960/0.9935/0.9921` and READY release was
`0.9889/0.9929/0.9882`. It was not tail-clean: exact waist excess still reached `0.7001 rad`,
idle-wrist reference error reached about `0.38 rad`, right-elbow worst reached `1.6242 rad`, and
post-swing heading worst reached `47.43 deg`. Those values were retained as risks rather than
silently averaged away.

The exact selected checkpoint SHA256 is
`3e727fae1d906215767363815a111243d464e49296b450a9a1b4c66bf0fe8bc8`. It was exported with task
V12 and the exact face-fixed FH/BH clips using:

```bash
TASK=HOPEPingPongHitterPureRallyV12 \
FH=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz \
BH=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz \
bash scripts/export_onnx_hitter_pure.sh <run> model_12700.pt
```

The formal `<run>/exported/policy.onnx` SHA256 is
`d972d76179c741b844117a2597b1469d33221e4f8b05379b53ef8c3c2f3513ea`. Fail-closed metadata
validation confirms 110-D actor, recipe `rally_v12` version `1`, runtime `rally_final_v2`,
deployment status `gate3_candidate`, and all three V12 residual stamps. C++/Python parity passes
with max deltas `3.58e-7` (zero input) and `1.19e-6` (seeded); the isolated V12 candidate package
builds and contains the same ONNX bytes.

Strict Gate3 was run in the Jazzy `hope` distrobox (the host's ROS Lyrical cannot load the
Jazzy-built MuJoCo plugin):

```bash
distrobox enter hope -- bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /home/dongc1/workspace/Hope_v11/a3_deploy/a3_deploy_example && bash scripts/pp_gate3_rally_v12.sh'
```

The result is **FAIL: 141 checks passed, 18 failed**. Lifecycle/plant evidence is healthy: 12/12
serves engage and reach the completion/recovery proxy, zero falls/rescues/stale-base events occur,
all READY flags are true, and the synchronized MuJoCo plant passes (`z=[0.974,1.068] m`, tilt peak
`12.7 deg`, both-feet contact `0.989`, zero actuator-saturation rows). ONNX parity plus the passing
plant rule out export corruption and a gross Gate3 plant failure as the current cause.

Strict policy-quality failures are:

- aggregate q-des clipping: 9/111 sampled audit statuses are nonzero; `left_ankle_pitch 44/4327 @ 0.3636 rad`,
  `left_ankle_roll 6 @ 0.2043`, `left_shoulder_roll 16 @ 0.1151`, `right_ankle_pitch 10 @ 0.0785`,
  `right_hip_roll 7 @ 0.0640`, and `waist_pitch 74 @ 0.0274`; waist roll has no reported hit;
- forehand right-elbow contact p90 fails on swings 1/2/6/7/8/9 (`1.404-1.465 rad` vs `1.35`);
- strike station-x fails on swings 4/11/12 (`0.032/0.036/0.038 m` vs `0.03`), and the conductor's
  whole-interval `0.05 m` x-lock fails on serves 6/9/12 with peak/end `0.076 m`;
- post-swing mean heading fails on swings 4/5/10/11 (`10.5-12.7 deg` vs `10 deg`); swing 5 also
  fails station-y (`0.121 m`) and narrowly fails pre-swing mean/p90 speed (`0.201/0.311 m/s`);
- idle left-wrist roll/pitch/yaw ranges now all pass (`0.297/0.297/0.288 rad`).

Relative to V11 `model_11800`, V12 materially fixes the intended wrist tail, reduces station-x
failures from six swings to three, and reduces waist clipping from pitch/roll
`391/78` ticks at `0.2708/0.1511 rad` to pitch-only `74` ticks at `0.0274 rad`. It does not make all
joint requests deploy-clean: the dominant clipping moved to ankle/shoulder joints, while new
forehand-elbow and post-heading failures make the total strict result worse (`150/9` to `141/18`).
The Isaac deterministic reports already showed the elbow/heading worst-case risk, so this is not
evidence that MuJoCo alone invented it.

Complete immutable run artifacts are under
`/home/dongc1/.cache/hope_v12_eval/gate3_model12700_20260718/`; the 15 Isaac evaluation logs are
under `/home/dongc1/.cache/hope_v12_eval/isaac/`. The ball source is still fake ROS input: only
7/12 geometric contact candidates exist, physical contact and landing are unmeasured, and 12/12
proxy completion must not be reported as a real hit/return rate. G06 remains `Partial`; this ONNX is
simulation-only and is not promoted to the generic runtime or hardware.

## 2026-07-18 RallyV11 paired-dynamics and strict Gate3 result

The V11 candidate does **not** pass promotion, but the original attribution to a broken MuJoCo plant
was wrong. `eval_deterministic.py` now captures a no-DR, no-corruption, no-termination one-environment
trace containing the exact observation, deterministic action, post-clamp target and full Isaac plant
state at 50 Hz. The replay has two deliberately different meanings:

- `open-loop-target` applies the recorded Isaac target sequence after the two plants have begun to
  differ. Its 200-tick result diverges (`joint RMSE 0.1178 rad`, root max `1.3717 m`, body RMSE
  `0.7939 m`, orientation max `87.06 deg`). This is a useful sensitivity diagnostic, but it removes
  policy feedback and therefore does **not** prove that the Gate3 plant is wrong.
- `closed-loop-policy` reruns the exact exported ONNX against each plant's current state and the same
  captured external command stream. With the actual Gate3 explicit-Euler plant contract (1 ms
  MuJoCo step, explicit PD, preserved damping/frictionloss, friction `1.5`, optimized feet), the
  500-tick/10 s comparison passes: joint RMSE `0.039937 rad`, root max `0.027126 m`, body RMSE
  `0.017579 m`, orientation max `9.240829 deg`, and foot-contact agreement `100%`. Artifact:
  `/home/dongc1/.cache/hope_v11_closed_gate3_plant_500.json`.

Reproduction (inside `grasping`, after sourcing `setup_train_env.sh`):

```bash
hope_isaac_py scripts/eval_deterministic.py task=HOPEPingPongHitterPureRallyV11 algo=ppo \
  headless=true num_envs=1 +seed=0 +paired_steps=500 \
  +paired_trace_out=/home/dongc1/.cache/hope_v11_pair_500_material1_v3.npz \
  checkpoint=/home/dongc1/workspace/HOPE/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_v11/2026-07-17_07-16-59_rally_v11_gate3_ready_footwork_fresh_exportfix/model_11400.pt \
  motion_file=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz \
  motion_file_2=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz
```

Replay from the worktree root:

```bash
/home/dongc1/.conda/envs/hope-motion-py310/bin/python \
  hope_training/whole_body_tracking/scripts/mujoco_paired_dynamics_replay.py \
  --trace /home/dongc1/.cache/hope_v11_pair_500_material1_v3.npz \
  --onnx a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64/models/model_11400_v11.onnx \
  --control-mode closed-loop-policy --pd-mode explicit --mujoco-sim-dt 0.001 \
  --integrator euler --contact-friction 1.5 --noslip-iterations 3 \
  --json-out /home/dongc1/.cache/hope_v11_closed_gate3_plant_500.json
```

Checkpoint selection was then repeated in the required order rather than inferring a winner from
the training dashboard. Training-curve local peaks `model_10700`, `model_11800`, and `model_12300`
were compared with the deployed `model_11400` baseline in deterministic Isaac. The first 256-env
pass exposed a reporting issue: the aggregate exact-strike EMA had enough samples, while each
FH/BH accumulator fluctuated around `exact_success_min_count=50`; guard-generated zeros were being
averaged as real per-side failures. The evaluator now reports global/FH/BH raw EMA counts, verifies
that global equals the clip partition, recomputes final per-clip rates from the raw counters, and
prints `NaN` rather than zero when a clip lacks evidence. Standard V11 qualification uses 512 envs.

`model_11800` was the best diagnostic candidate and received 512-env, seed-0, 1200-step evaluation
on all three velocity distributions. Final raw-EMA FH/BH composite was `0.9980/0.9944` on core,
`0.9875/0.9918` on planner, and `0.9882/0.9882` on production mix; aggregate-vs-clip rate delta was
zero and each clip had about `93-96` decayed samples. Exact position error was `2.10-2.25 cm`, READY
release `0.9953-0.9971`, and pre-strike fall `0.64-0.74%`. This proves the candidate can strike and
release across the trained command components, but it is not promotion-clean: planner tails reached
READY heading `45.96 deg`, raw joint q-des `3.1532 rad`, left-wrist reference error `0.4857 rad`, and
right-elbow position `1.5749 rad`. Isaac base-x error at exact strike was already `0.0293 m` on mix
and `0.0302 m` on planner, essentially the Gate3 per-swing limit of `0.03 m`; the later MuJoCo
station-x misses are therefore a reproduced policy margin problem, not evidence that the harness
invented a new x drift.

The exact diagnostic export is `model_11800_v11_diag.onnx`, SHA256
`6bfdc36eea13be93471426b815c1d865eca53b91924f221db7db2d5f11a6092b`. Its C++/Python ONNX parity
passes (`2.38e-7` zero input, `7.15e-7` seeded), and the candidate x86 package builds. The subsequent
strict Gate3 correctly exits `1`: runner proxy is `12/12` with zero falls/rescues/stale-base warnings,
all four main transitions and both positive-main transitions occur, and the synchronized MuJoCo
plant passes (`z=[1.023,1.068] m`, tilt peak `9.0 deg`, both feet `0.993`, zero control-saturation
rows). Heading and elbow checks all pass for this checkpoint. The remaining nine failures are:

- aggregate q-des clamp evidence: waist pitch `391/4328` ticks, max discarded request `0.2708 rad`;
  waist roll `78/4328`, max `0.1511 rad`; plus three small ankle hits;
- idle left-wrist roll/yaw ranges `0.402/0.357 rad` (pitch `0.301 rad` passes);
- station-x strike error on swings 3, 4, 5, 8, 9 and 10 (`0.030-0.036 m` vs `0.03 m`).

This is an improvement over `model_11400` (`147 pass / 12 fail` to `150 pass / 9 fail`, materially
smaller waist clipping, no heading/elbow failure), but not a Gate3 pass. Complete artifacts are under
`/home/dongc1/.cache/hope_v11_eval/gate3_model11800_20260718/`.

For comparison, the corrected `model_11400` 12-serve strict Gate3 run kept the real deploy loop
upright: 12/12 engagement,
12/12 completed/recovered proxy, zero falls/rescues/stale-base warnings, all four commanded main
station transitions achieved, and MuJoCo MOTION `z=[1.024,1.069] m`, tilt peak `10.5 deg`, both-feet
contact `0.993`, zero actuator-saturation rows. It still fails policy-quality checks: substantial
waist q-des clipping (`waist_pitch 514/4332`, max discarded request `0.4064 rad`; `waist_roll
109/4332`, max `0.3658 rad`), idle left-wrist roll/pitch range `0.483/0.369 rad`, strike station
errors on multiple swings, one post-heading failure, and one forehand elbow contact p90 `1.367 rad`.
The conductor's only x-lock miss was `0.050000536 m` against `0.05 m`; the harness now applies a
fixed `1e-6 m` arithmetic tolerance, not a threshold change. An earlier repeat reached `0.05135 m`
and still fails. Raw planner velocities also remain outside the ONNX velocity boxes; `--demo`
substitution makes the runner rehearsal executable but does not turn that into planner-contract
certification.

Conclusion: there is no current evidence that the Gate3 MuJoCo plant is the root cause of V11's
failure. The V11 soft repair terms are present in training/export metadata, and changing from
`model_11400` to independently selected `model_11800` predictably improves heading, elbow and clamp
evidence, but no evaluated checkpoint converges every deploy tail tightly enough. Proxy completion
is not physical ball contact or landing; both remain unmeasured.

2026-07-11 FinalV3 geometry note: the new training/runtime contract uses ONNX-baked plane x `0.70`
and FH/BH reach-y `-0.04/+0.10`; the Gate3 harness now spawns at x `0.33` for fixed world hit plane
x `1.03`, and its host planner audit passes 10/10 envelopes. A trained checkpoint and MuJoCo run do
not yet exist, so this is synchronized code/harness evidence, not a completed G06 result.

## Goal

Test whether a policy learned in Isaac can be replayed or approximated in MuJoCo.

This gate is the sim-to-sim bridge before real deployment.

## Inputs

- Isaac-trained policy ONNX from G05 (exported with the full metadata contract).
- MuJoCo A3 model from G04 (`a3_pingpong.xml`).
- Shared joint order and observation/action contract (`docs/interfaces/policy_observation_action.md`).

## Outputs

- Replay/evaluation procedure with Isaac-exact metrics.
- Cross-sim metrics and known mismatch list.
- Decision on which MuJoCo configuration is deploy-faithful.

## Related Directories

- `hope_training/whole_body_tracking/scripts/mujoco_eval_onnx.py` — the parity evaluator.
- `a3_deploy/a3_deploy_example/` — active deploy tree: `MUJOCO_VALIDATION_RUNBOOK.md`, `SIM_DEPLOY_REHEARSAL.md`, `SIM_FIDELITY_NOTE_FOR_AGI.md`.
- `a3_deploy/A3_MuJoCo_Sim/` — vendor AimRT MuJoCo sim (the explicit-PD subscriber lives here).
- `agi/code_deployment/a3_deploy_example/` — older vendor reference subset.

## Operation Docs

- [../../QUICKSTART_A3_ISAAC.md](../../QUICKSTART_A3_ISAAC.md)
- [../operations/build_and_test.md](../operations/build_and_test.md)
- [../../a3_deploy/a3_deploy_example/README.md#验证流程](../../a3_deploy/a3_deploy_example/README.md#验证流程)

## Acceptance Criteria

- The same action ordering is verified in both simulators.
- The exported deploy ONNX (not a re-export) runs in MuJoCo with the training observation rebuilt exactly.
- Divergence sources are documented: contact, latency, actuator, timestep, observation delay, model mismatch.
- Exact-strike metrics from Isaac are reproduced in MuJoCo and recorded per accepted checkpoint.

## Current State

### 2026-07-15 RallyV10 `model_10500` — selected export and deploy parity complete

- The operator selected
  `.../2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058/model_10500.pt`
  (SHA256 `4d4a1ab90e41a75c113f607fab99836a56d833ee65c38516f3a0252a906d28cc`).
  The promotion source is only `<run>/exported/policy.onnx`, staged as
  `a3_deploy/a3_deploy_example/assets/a3_runtime/models/model_10500_v10.onnx`; both have SHA256
  `fa4277801bc6bb605e0312881173c06cbb7545b403533d6226c7db11a501ec5f`. The run-root
  auto-export is recipe-6/runtime-v1 and is not this candidate.
- The durable runtime config points at `model_10500_v10.onnx`, the x86 package was rebuilt from
  it, and the exact durable asset passed `scripts/pingpong_parity/run_parity.sh`: zero-input max
  delta `4.77e-07`, seeded max delta `1.07e-06`, finite/bounded output, ONNX box-center contract
  pass and policy-callback pass. This is the checkpoint-specific parity record; broader physical
  MuJoCo/promotion criteria remain open, so G06 remains `Partial`.

- V10 keeps the 110-D actor tensor but is exported as `training_recipe=rally_v10`, recipe **7**
  (6 intentionally skipped), and `hitter_pure_runtime_contract=rally_final_v2`. Runtime v2 is a
  capability boundary: C++ and MuJoCo both require the bidirectional V2/V10 pairing, exact saved-run
  metadata, union/core/planner boxes, mix `0.75`, ramp `96000`, shared plane `0.58`, and recovery
  fields. A recipe-6 export, V10 on runtime-v1, missing component or drifted value fails closed.
- MuJoCo random evaluation samples the actual converged `75% planner + 25% core` support rather
  than the union's untrained Cartesian corners. It also mirrors the saved `50% same / 20% small /
  30% main` station-y transition mixture. `--targets-center` remains valid because both union
  centers are proven inside the corresponding planner component. Recipe-7 eval refuses box CLI
  overrides that would make a G06 result non-contractual.
- Gate3 uses only `scripts/pp_gate3_rally_v10.sh`: `x_hit=1.02`, shared plane `0.58`, station
  `x=0.44`, zero BH delta, x-lock `0.05`, and its own solver-checked 12-serve bytes. V8/V9 use
  `pp_gate3_rally_v8.sh`; the bare engine fails closed without a versioned profile. The V10 report
  checks idle left-wrist range and absolute right-elbow contact p90; a sparse capture falls back to
  the nearest strike tick instead of producing a NaN false failure.
- Reproducible host checks:

  ```bash
  python -m pytest -q \
    hope_training/whole_body_tracking/tests/test_mujoco_runtime_v2_contract.py \
    a3_deploy/a3_deploy_example/tests/test_pp_rally_report.py
  cmake --build a3_deploy/a3_deploy_example/build/a3_pkg_x86_64 --target run_tests -j2
  a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64/run_tests \
    --gtest_filter='PpRuntimeContract.*:PpVelocityGate.*:PpStaticHandoff.*'
  ```

  Focused results are Python `24/24` plus five fail-closed subtests and C++ `10/10`. The selected
  recipe-7 ONNX now has the checkpoint-specific deploy parity record above; full physical MuJoCo
  acceptance and the complete per-checkpoint metric table remain open, so G06 remains `Partial`.
  The full host C++ binary builds and reports `173/181` passed, `3` skipped and `5` failed; the
  five are the known backend-off callback/timing cases (four) plus the missing local FK XML asset
  (one), while all ping-pong runtime-contract tests pass. A native `PpOnnxPolicy` smoke loads the
  preserved recipe-5/runtime-v1 V9 ONNX (`obs_dim=110`) and rejects the initial V10
  recipe-6/runtime-v1 ONNX at construction, as required. The active run's read-only export
  preflight passes against `model_2800.pt`; it creates no ONNX and is not promotion evidence.

### 2026-07-11 RallyFinalV3 pre-deploy contract — implemented, awaiting a trained ONNX

The earlier Gate3-derived per-side edge aims are superseded for the repaired FinalV3. The real-system
contract uses venue contact height `[0.98,1.26] m` and HITTER opponent-half-center returns. Gate3 now
uses center `land_y=0`, `dt=0.50` for both sides and a contact-height-matched fake sweep; the host audit
passes raw velocity/z/physical landing `5/5` per side, station `10/10`, and all nine 20-35 cm
transitions. This is harness consistency only: promotion still requires a trained checkpoint and
physical AGI closed-loop success.

- Runtime now takes both scripted racket position and velocity centers from the ONNX per-clip
  boxes; it no longer mixes a new model with hard-coded FinalV2 centers. FinalV3 ONNX loading also
  requires recipe version 3, hold `40,60`, station mixture/explicit side, passive-head semantics,
  physical-only termination, and immutable motion/receipt/env hashes.
- Planner side selection is explicit and based on the predicted contact relative to corrected live
  base/previous station, with a `0.04 m` Schmitt hysteresis; it is not inferred from racket `vy`.
  The current planner and Gate3 harness use the neutral opponent-half-center aim
  `land_y=0, dt=0.50` for both sides. `pp_planner_envelope_audit.py` reports raw velocity/z and
  physical landing coverage `5/5` per side, runner station coverage `10/10`, and all nine commanded
  transitions in `0.20-0.35 m`. These are verification fixtures; real ball position history, table
  origin and localized base pose remain the runtime inputs, and hardware tuning may change the
  planner without redefining the policy's trained command envelope.
- Gate reporting maps each engage to the runner's explicit chronological side and uses V3-specific
  ready/post windows. Export/train identity and checkpoint provenance are fail-closed, including
  renamed run directories and dual Hydra identity overrides. The x86 deploy package rebuild and
  C++/Python parity gate pass on the current source.
- This is preflight evidence only. No FinalV3 checkpoint/ONNX exists yet, so Gate3 has not been run
  on V3 and G06 remains `Partial`. Promotion still requires physical MuJoCo closed-loop repeated
  `move -> settle -> hit -> recover`, not a planner-envelope or single-strike pass.

### 2026-07-11 FinalV2 `model_8900` result — FAIL, mixed runner/task causes

The staged deploy asset was verified against
`.../2026-07-10_18-17-32_hitter_rally_final_v2_fresh_s1/model_8900.pt` before these runs.
The documented strict command (`20` serves, no rescue, readiness required, eight transitions,
`--vel-box-center --hold-recover 2.5 --gate-x-max 0.15`) failed after two serves:
`engaged=0`, `returned=0`, one fall, zero reached transitions, and pre-fall max x error `0.097 m`.
This is not clean model-only evidence: the first pending command appeared at `tts=1.45 s` and expired
before release; on the second command the runner cleared the pending walk on `too_late/stale` and
immediately returned a still-moving robot to the static stand. The run also emitted one stale-base
warning, which violates the strict localization requirement. Artifact:
`/home/dongc1/hope_eval_final/g3_model8900_official_strict_20260711_030855`.

The independent Gate3A result is the clean task/plant negative control because it has no ball,
planner, swing, or motion-video dependence. It reached only `5/12` READY transitions; latency p95
was `4.04 s` (required `<=0.80 s`), max x error `0.093 m` (required `<=0.04 m`), and max heading
`25.4 deg`. Full-state checks all failed except the no-swing invariant: measured ankle roll,
hip-yaw, sole tilt/yaw, ready-dwell near-floor slip, raw ankle clamp, and the absolute raw ankle-qdes
envelope. Representative worst values were right hip-yaw p95 `0.742 rad`, right foot-yaw p95
`41.8 deg`, right ready-dwell near-floor slip p95 `1.184 m/s`, and right raw ankle safe-envelope
violation rate `26.3%`. Artifact:
`/home/dongc1/hope_eval_final/g3a_station_only_2026-07-11_030251`.

A diagnostic `--hold-recover 20` run suppressed the post-swing static handoff and survived four
serves with zero falls, confirming a material runner-handoff contribution. It still achieved only
`2/4` engages and one accepted return proxy, with max x error `0.326 m`; the segmented swings failed
ready/post speed and yaw-rate checks. Artifact:
`/home/dongc1/hope_eval_final/g3_model8900_hold20_ablation_20260711_030242`.
Therefore `model_8900` is not promotable, and neither a Gate3-harness-only fix nor a new backhand
clip is sufficient. Gate3 still does not measure physical paddle-ball contact or landing.

2026-07-10 RallyFinalV2 **Gate3A station-only** implementation (ready for a 40-60-trained V2
candidate; the initial 25-50 `model_700` is diagnostic only):

- Added an opt-in native-runner `--station-only` mode and a dedicated no-ball/no-swing harness:
  `a3_deploy/a3_deploy_example/scripts/pp_gate3a_station_only.sh`. It exercises the production
  x/y/speed/0.12 s dwell readiness machine, prints `READY`, and returns before level 1 can be
  released. Default rally behavior is unchanged when the flag is absent. The report requires zero
  engage and swing-complete events and every measured runner trace row to remain at level 0.
- The scripted station sequence is
  `+0.20 FH, -0.275 BH, +0.35 BH, -0.20 FH, +0.275 FH, -0.35 FH`, repeated twice. It covers both
  sides and all three distance bands while returning to y=0 every six transitions. Certification
  requires 12/12 READY events, each latency `<=1.0 s`, latency p95 `<=0.80 s`, max x error
  `<=0.04 m`, whole-move heading `<=15 deg`, pelvis z `>=0.95 m`, and zero falls. A cumulative
  2 cm station/side dwell anchor resets readiness if a planner target walks during the dwell, so
  many individually small refinements cannot release a stale station.
- Full-state certification is fail-closed. `pp_gate3a_full_state.py` joins exact pelvis poses with
  the native runner measured-joint/q-des trace and runs FK through the plant MJCF. It gates measured
  ankle roll, hip yaw, sole tilt/yaw, **ready-dwell** near-floor slip, raw deploy-hard-clamp use, and
  raw ankle q-des outside the absolute `+/-0.20 rad` V2 envelope. Approach near-floor velocity is
  reported but not gated as slip because intentional lateral stepping must remain free. Sole
  contact remains honestly labelled a kinematic near-floor proxy, not a contact-force measurement.
- Certification additionally requires ONNX metadata
  `hitter_pure_training_recipe=rally_final_v2`, version `2`, and hold range `40,60`. A V1 ONNX or an
  initial 25-50 V2 export is rejected by default; `PP_G3A_ALLOW_V1=1` permits diagnostics only and can
  never produce PASS. Recipe/version remain V2/2 because the runtime/task schema is unchanged; the
  dedicated hold field is the fail-closed timer-revision discriminator.
- The initial fresh V2 `model_700` has not passed Gate3A. A seed-0 Isaac deterministic
  zero-weight-update A/B changed only the evaluation timer from 25-50 to 40-60 and improved ready
  release `0.7205 -> 0.8665`; the three 20-25/25-30/30-35 cm bins changed from
  `0.7589/0.7182/0.5921` to `0.8816/0.8619/0.8333`, while latency p95 reached `0.80 s` and composite
  improved `0.3659 -> 0.4206`. The longer ready exposure also revealed unsafe tails (hand clearance
  minimum `0.0145 m`, foot-orientation max `2.213 rad`, raw ankle q-des max `1.238 rad`) and remains
  below the `0.95` release gate. This justifies training the deploy-relevant timer distribution, not
  promoting the zero-shot checkpoint; a 40-60-resumed policy must pass Gate3A before strikes.
- The final-code V1 diagnostic on `model_13000` is preserved at
  `/home/dongc1/hope_eval_final/g3a_station_only_2026-07-10_162504/pp_gate3a_report.json`.
  It is a non-certifying FAIL: 1/12 READY (first +0.20 m FH in 0.74 s), then the -0.275 m BH
  transition timed out; zero swing, zero fall, max x error `0.0315 m`, min pelvis z `1.030 m`, but
  whole-move heading reached `20.5 deg`. Full-state coverage was 100% (502 samples) and independently
  failed ankle/hip/sole/raw-q-des gates: left/right measured ankle-roll max `0.311/0.323 rad`, left
  hip-yaw deviation p95 `0.806 rad`, left foot-yaw p95 `41.9 deg`, right sole-tilt p95 `11.7 deg`,
  right ready-dwell near-floor slip p95 `0.207 m/s`, and raw absolute-envelope violation rates
  `14.0%/5.9%`. This is the expected negative control: V1 can begin a lateral move but has not learned
  repeatable frozen-ready move/settle behavior.

After the operator trains and exports a V2 checkpoint, stage it through the normal sync/build path,
then run Gate3A before any strike gate:

```bash
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example
PP_VIEWER=0 bash scripts/pp_gate3a_station_only.sh
'
```

Only a certifying V2 `PASS` advances to the strict 20-serve Gate3. The old checkpoint may be replayed
only as a named diagnostic by adding `PP_G3A_ALLOW_V1=1`.

2026-07-10 RallyFinal gate extension (implemented; `model_13000` diagnostic below, no passing candidate):

Later `model_13000` AGI audit (2026-07-10; **diagnostic FAIL, not certification**):

- The saved run with `serves=12`, `engaged=1`, `returned=0`, `falls=0`, drift `0.042 m`, max x error
  `0.059 m`, and min motion z `1.017 m` confirms excellent single-swing plant/x-lock behavior. Its
  only swing is square at contact (`5.9 deg`) but finishes near `19 deg` and then remains `21-23 deg`;
  the correct `20 deg` runner safety gate rejects subsequent serves. The task fix is the bounded
  ready+post `rally_heading_debt` documented in G05, not relaxing the runner gate.
  Re-running the saved 110-D CSV through the extended report quantifies the blind spot: ready heading
  mean/max `18.1/20.3 deg`, post heading `16.9/19.7 deg`, and ready/post root yaw rate
  `0.302/0.205 rad/s`, all failing the new recovery gates while contact heading still passes.
  The same report also retains the independent raw-action sanity failure (`max |action|=30.36` versus
  the current `<12` check); q-des clamping limits its immediate plant effect, but it must be localized
  and cleared before deployment promotion rather than hidden by the zero-fall result.
- The run was not strict RallyFinal evidence: its JSON says `readiness_required=false`, required station
  transitions `0`, and observed in-band transitions `0`. It therefore cannot prove the requested
  lateral move/settle/repeat behavior despite the zero-fall result.
- The claim “119 velocity-box rejects” is not supported by the preserved raw logs. They contain one
  generic `REJECT(110)`, caused by forehand target z `1.22 m` exceeding `[0.67,0.97]+/-0.05 m`, zero
  literal `OUT-OF-BAND` velocity events, six throttled `REJECT yawed` lines, and 13 `PLANNER: yawed`
  status samples. A generic reject line always prints `vel=(...)`; only `OUT-OF-BAND` diagnoses a
  per-axis velocity-box failure. The one engaged command `(+0.885,+0.794,+0.657) m/s` is about
  `0.165 m/s` below two exact forehand box bounds but remains inside the runner's `0.30 m/s` margin.
- The earlier `rclcpp::Waitable` plugin error was a host ROS Lyrical/Jazzy ABI mix, not a persistent
  simulator blocker. Running from the documented `hope` distrobox supplies the matching ROS Jazzy
  stack. The staged dist ONNX and the specified run export have identical SHA256
  `20f556e950a0a8b56f2fea7ffaa101f6a3df1b97b2c3d3526389d50497ba2aeb`; its 110-D
  `rally_final_v1`, fixed-plane, station-step and per-side box metadata were all verified before test.
- Two strict no-rescue Gate3 repeats (viewer and headless) both failed **before engage** on the first
  approximately 24.5 cm planner station change. Viewer: `serves=1, engaged=0, falls=1`, min pelvis z
  `0.092 m`; headless: `serves=2, engaged=0, falls=1`, min z `0.094 m`. In the headless repeat the
  base reached y `+0.217 m` toward the `+0.249 m` station while remaining within about `0.048 m` x
  error before the fall, but it never satisfied readiness and no clip clock was released. The
  approximately `1.1 m` terminal x error in each JSON is post-fall sliding and must not be mislabeled
  as a pre-strike forward lunge. Both runs recorded `0/8` required station transitions and 0% return.
  Preserved artifacts:
  `/home/dongc1/hope_eval_final/g3_strict_2026-07-10_153x_model13000` and
  `/home/dongc1/hope_eval_final/g3_strict_headless_2026-07-10_154356_model13000`.
- The visual foot concern is confirmed by joint/FK forensic data, not dismissed as viewer noise.
  During frozen-ready, left/right raw ankle-roll targets exceed the deploy hard limits on about
  `83%/97%` of ticks; pooled hip-yaw reference deviation p95 is `0.593 rad`, and the right sole's
  world tilt is p95/max `16.1/18.4 deg`. The stable one-swing diagnostic likewise logged right
  ankle-roll command clamping on `79%` of ticks. The policy is exploiting desired-position clipping:
  measured-q `joint_limit` can pass while raw q-des repeatedly asks for an inverted/rolled foot.
  Promotion must therefore gate raw ankle-roll clamp fraction (target <=5%), phase-aware hip-yaw and
  ankle-roll envelopes, and loaded-foot sole tilt, not foot translation slip alone.
- The independent in-repo full-state MuJoCo path did run after the metric extension, using exact
  training-box targets, explicit deploy-faithful PD, both clips, and 200 control steps. `model_13000`
  completed 2/2 swings with zero falls and exact composite `1.0`, yet ready heading was
  `19.35 deg` mean / `26.91 deg` max and post heading `19.39/23.03 deg` with yaw rates
  `0.605/0.230 rad/s`. This short diagnostic is not a full gate, but it reproduces the recovery failure
  without the G3 planner's edge velocity and validates the new MuJoCo metric path. Output:
  `/tmp/rally_heading_metric_smoke`.

- `mujoco_eval_onnx.py` now consumes optional ONNX metadata
  `hitter_pure_station_y_step_range`, mirrors the stateful wrap-to-wrap station sampler, and reports
  the RallyFinal full-state metrics: exact base/station errors and speed, pre/post base speed,
  post leg RMS speed and contact-foot slip, contact pelvis/torso heading and torso yaw rate,
  ready/post pelvis heading mean+max and root yaw rate, and backhand
  racket-to-left-hand/forearm minimum distance. Exact-strike CSV rows carry the same raw quantities.
- HitterPure ONNX export now accepts `TASK=...`; V1 **must** export with
  `TASK=HOPEPingPongHitterPureRallyFinal` and V2 with
  `TASK=HOPEPingPongHitterPureRallyFinalV2`, so fixed reach, y box and station-step metadata are not
  silently replaced by base HitterPure values. V2 additionally carries fail-closed
  recipe/version/40-60-hold metadata. The export shell also checks the saved run `params/env.yaml`,
  so re-exporting an initial 25-50 checkpoint under the current task cannot relabel its provenance.
  `play.py` and `eval_deterministic.py` honor the separate local `motion_file_2` argument; eval
  failures return nonzero.
- The AGI rally runner now recognizes only the explicit ONNX
  `hitter_pure_runtime_contract=rally_final_v1` marker plus complete station metadata, keeps the clip
  clock at windup while a predicted station is pending, and releases after x/y error, filtered base
  speed, and dwell readiness pass. The conductor requires that evidence, 20 continuous serves,
  return proxy >=80%, zero falls/rescues, and at least eight reached 0.20–0.35 m station transitions.
  `pp_rally_report.py` also requires the 110-D observation trace to contain every returned swing.
- Candidate MuJoCo commands, after Isaac export:

  ```bash
  cd hope_training/whole_body_tracking
  FH="$PWD/artifacts/hope_forehand_hopex/motion.npz"
  BH="$PWD/artifacts/hope_backhand_hopex/motion.npz"
  RUN="$PWD/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_final_v2/DATE_TIME_RUN_NAME"
  CKPT="$RUN/model_N.pt"
  TASK=HOPEPingPongHitterPureRallyFinalV2 FH="$FH" BH="$BH" \
    bash scripts/export_onnx_hitter_pure.sh "$RUN" "$(basename "$CKPT")"

  ../.venv-motion/bin/python scripts/mujoco_eval_onnx.py \
    --onnx "$RUN/exported/policy.onnx" \
    --motion-files "$FH" "$BH" --noise-scales 0.0 --steps 6000 \
    --pd-mode implicit --reset-mode multiswing --hold-steps-range 0 0 \
    --out-dir "$RUN/eval_mujoco_implicit"

  ../.venv-motion/bin/python scripts/mujoco_eval_onnx.py \
    --onnx "$RUN/exported/policy.onnx" \
    --motion-files "$FH" "$BH" --noise-scales 0.0 --steps 6000 \
    --pd-mode explicit --keep-passive --deploy-faithful \
    --df-hold-steps 50 --df-rest-steps 75 --df-clips both \
    --out-dir "$RUN/eval_mujoco_agi_explicit"
  ```

- Candidate full-state thresholds: exact composite >=0.95 overall / >=0.93 backhand; exact racket
  position <=0.075 m and velocity <=0.50 m/s; base-x excursion p90 <=0.10 m and max <=0.15 m;
  exact station-y error <=0.10 m; pre/exact/post base speed <=0.20/0.20/0.25 m/s; post contact-foot
  slip <=0.05 m/s; hand/forearm clearance >=0.15/0.12 m; pelvis/torso heading <=15 deg at contact;
  ready/post pelvis heading <10 deg mean and <15 deg max with root yaw rate <=0.20 rad/s; zero falls
  and deploy-faithful completion =1.00 overall and for both clips. Station-step min/max must remain
  within `[0.20,0.35] m`.
  The live 110-D trace additionally gates per-swing pre/post speed window means at 0.20/0.25 m/s and
  p90 at 0.30/0.35 m/s; strict runs permit zero stale-base warnings unless a deliberate dropout case
  was requested. The 2026-07-10 foot audit adds mandatory full ready/windup coverage: pre-fall
  max `|base_x-station_x| <= 0.04 m`, ready/windup `|vx|` p90 `<=0.15 m/s`, raw ankle-roll q-des
  clamp fraction `<=5%`, ankle-roll reference error p95/max `<=0.15/0.25 rad`, hip-yaw reference
  error p95 `<=0.41 rad` (target `<=0.30 rad`), and loaded-foot sole tilt p95/max `<=5/10 deg`.
  These need a full-state/contact trace; the 110-D actor CSV alone cannot honestly measure contact
  or sole pose.
  These thresholds are provisional until the clean 18400 baseline and an actual Final checkpoint are
  recorded side-by-side.
  `scoreboard_eval.py` returns nonzero for a child protocol/runtime failure and records these fields,
  but does not yet enforce the provisional numeric thresholds; promotion requires an explicit row-by-row
  comparison recorded here.
- Closed-loop AGI command (after model sync/build):

  ```bash
  cd a3_deploy/a3_deploy_example
  PP_SERVES=20 PP_MIN_RETURN_RATE=0.8 PP_ALLOW_RESCUE=0 PP_MAX_RESCUES=0 \
  PP_REQUIRE_READY=1 PP_MIN_STATION_TRANSITIONS=8 \
  PP_XLOCK_THRESH=0.15 \
  PP_EXTRA_ARGS="--demo --hold-recover 2.5 --gate-x-max 0.15" \
    bash scripts/pp_gate3_rally.sh
  python3 scripts/pp_rally_report.py /tmp/pp_obs.csv /tmp/pp_rally_report.json
  ```

- Important limitation: Isaac `strike_composite_success_exact` is kinematic racket-target success.
  The current AGI fake-ball rally marks `RETURNED` as engage + swing-complete + recover; it does not
  simulate/measure physical paddle-ball contact or landing. Therefore neither number alone proves the
  user's final hit/return criterion. Final acceptance still requires an instrumented physical-ball
  MuJoCo path or the real mocap/planner closed loop recording actual contacts/returns, alongside zero
  lunge/slip/self-hit/collapse. G06 remains Partial.

Done (2026-06-27 → 2026-07-02, recorded 2026-07-03):

- The parity procedure exists and is battle-tested: `scripts/mujoco_eval_onnx.py` loads the exact
  exported deploy ONNX, reads the whole actuator contract from ONNX metadata (joint_names,
  default_joint_pos, action_scale, kp/kd, body_names — fails loudly if missing), auto-detects the
  175-D deploy-parity vs 180-D legacy obs contract, rebuilds the Isaac actor observation in MuJoCo
  (same frame math; the deploy-honest racket-target reframe is verified by
  `scripts/realsensor_obs_reference.py`), and reproduces Isaac's exact-strike metrics
  (pos/vel/normal pass, composite, hit-speed error, velocity attainment) with per-clip
  forehand/backhand breakdowns and per-step CSVs.
- The dominant divergence source was isolated to actuator PD integration: with the same ONNX and
  byte-identical `a3_pingpong.xml`, MuJoCo with `implicitfast` + kd in `dof_damping` (Isaac
  `ImplicitActuator` equivalent) is stable with clean swings, while the AGI deploy sim's
  explicit-Euler PD path (`joint_actuator_subscriber.cc`, MJCF without an integrator attribute,
  passive damping not zeroed) diverges within ~0.1 s. Switching only the PD integration moved
  hit-speed error 0.61 → 0.31 m/s and velocity attainment 0.35 → 0.88. One-flag reproduction:
  `--pd-mode implicit` vs `--pd-mode explicit --keep-passive`. See
  `a3_deploy/a3_deploy_example/SIM_FIDELITY_NOTE_FOR_AGI.md`.
- Current verdict stance (2026-07-02): implicit PD remains the Isaac-faithful cross-check, but the
  binding pre-hardware gate is the AGI explicit clipped-PD MuJoCo run ("falls in MuJoCo = falls on
  the real robot"). The deployed policy was fine-tuned to survive it
  (`launch_explicitpd_ft.sh`, exported via `export_onnx_explicitpd.sh`).
- A deploy-faithful episode protocol exists: `--deploy-faithful` mirrors the C++ runner
  (nominal-stand start, windup hold with pinned time_to_strike, one full clip per swing, rest
  between swings, no teleports, absolute fall terminations only), reporting swing completion rates
  and time-to-fall.
- A documented validation flow with an acceptance-criteria table exists:
  `a3_deploy/a3_deploy_example/MUJOCO_VALIDATION_RUNBOOK.md` (rate ~50 Hz, sync stable, infer < 20 ms,
  projected gravity sanity, bounded actions, neck passive).

Not done:

- Formal per-checkpoint acceptance: the metric thresholds and the numbers for the currently shipped
  checkpoint (`model_p4_deployparity` / explicitpd_ft `model_25700`) are not yet pasted into this
  gate as an accepted record.
- (Fixed 2026-07-03, branch `audit-leftover-fixes`.) `eval_realsensor_hopex.sh` /
  `export_onnx_explicitpd.sh` now resolve their own location and take `HOPE_EVAL_*` /
  `HOPE_EXPORT_*` env overrides, and `mujoco_eval_onnx.py` resolves strike phases as CLI >
  ONNX `clip_strike_phases` metadata > built-in legacy `(0.36, 0.50)` (plus a
  `clip_seg_lengths`-vs-npz mismatch warning). The `--onnx`/`--motion-files` defaults still point
  at a legacy run — pass current artifacts explicitly.
- No decision recorded on MuJoCo as a training backend (currently it is a validation/dry-run stage
  only).

## Risks

- A policy can appear valid in Isaac but fail in MuJoCo because of actuator/contact mismatch — this
  happened (explicit-PD divergence) and cost significant time before the root cause was isolated.
- Evaluating with the script's stale defaults silently tests the wrong contract; always pass the
  checkpoint's own clips/phases.

## Next Steps

1. Record the accepted sim2sim numbers for the shipped checkpoint (implicit cross-check + explicit
   clipped-PD gate + `--deploy-faithful` protocol) in this gate.
2. When the mocap→planner bridge lands, extend the MuJoCo rehearsal to consume live
   `/racket/command` targets instead of sampled planner-equivalents
   (`docs/operations/run_shared_interface_rehearsal.md`).
