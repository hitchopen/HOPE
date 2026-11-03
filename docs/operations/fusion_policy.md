# Fusion policy and matched serve CSV

Fusion is a separate experimental policy selection (`modelfusion`). It requires
PPQ004, OptiTrack, verified policy/deploy bytes on both endpoints, and the exact
matched `v23-fusion.csv`. Selection and START use the existing stopped-state and
cross-endpoint integrity checks. Adding source support does not install or select
a model on a robot.

## USB artifact adaptation

The 2026-11-04 USB artifact `policy_fusion_17400_13800/artifacts/fusion_ablation.onnx`
is an untrained composite: model13800 legs, model17400 waist/arms, and zero passive
head output. Its SHA256 is
`c3c5ae5a2261f46ddf74d25d42ce016a258744ff849ee04dffc0c126fd92cf1b`.
It has only `obs -> actions`, so it is not a complete Runner export.

`a3_deploy/a3_deploy_example/scripts/prepare_fusion_policy.py` requires the exact
reviewed USB actor and original 13800/17400 policy/deploy pairs. It preserves the
Fusion actor computation, adds model13800 reference outputs and the shared
runtime contract, retains both source provenances, and updates deploy.yaml to the
new ONNX hash. It removes single-checkpoint training attribution from the
composite. Both actors receive the same previous applied fused action. It checks
128 observations against the USB actor, both source actors (including the passive
head mask), and all reference outputs before writing a new policy directory.

```sh
python3 a3_deploy/a3_deploy_example/scripts/prepare_fusion_policy.py \
  --fusion /path/to/fusion_ablation.onnx \
  --source13800 /path/to/model_13800_policy \
  --source17400 /path/to/model17400/policy \
  --output /new/path/fusion-policy
```

Dependencies are ONNX, ONNX Runtime, NumPy and PyYAML. Run native inspection with
the intended Runner's `--inspect-policy --policy-dir /new/path/fusion-policy`.
The expected interface is `ball_clock_110_v1`, 110 observations, 31 actions,
`step_dt=0.02`, fixed session HOME and a common 0.57 m relative hit plane.

## Whole-body 12000/13800 candidate

`prepare_fusion_12000_13800.py` reproduces the tested whole-body mixture exactly.
Every active joint uses the same mixture ratio, with shared applied-action
feedback and unchanged PD gains. `--method weights` is available for research;
the tested weight interpolations underperformed and should not be deployed.

```sh
python3 a3_deploy/a3_deploy_example/scripts/prepare_fusion_12000_13800.py \
  --wrapped-usb /new/path/fusion-policy --source12000 /path/to/model_12000 \
  --share13800 0.25 --method actions --output /new/path/whole-body-fusion
```

The fixed mixture is a precursor. Reproduce the earlier phase-conditioned
candidate with:

```sh
python3 a3_deploy/a3_deploy_example/scripts/prepare_fusion_phase.py \
  --whole-body-policy /new/path/whole-body-fusion \
  --output /new/path/phase-fusion
```

Use `/new/path/phase-fusion` for that candidate's separate CSV derivation. The `--recovery early` experiment
returned worse torso yaw and target execution and is retained only for research.
The original USB hard splice remains separately reproducible for comparison.

## CSV generation and validation

`adapt_fusion_serve.py` samples the selected Fusion's own native-controller
15-second WAIT rollout. The final second of measured posture and support torque
defines the policy-ready command; the established +0.04 rad ankle compensation is
then revalidated. This does not splice the parent policies' ready poses.

```sh
python3 a3_deploy/a3_deploy_example/scripts/adapt_fusion_serve.py \
  --policy-dir /new/path/fusion-policy \
  --library /path/to/serve_probe.so --cycles 3 \
  --output /new/path/fusion-serve
```

Use a probe compiled against the current native controller with the simulation
clock enabled. The script checks Normal and Kernel repeats, yaw=1 rad, and branch
pitch-rate disturbances of +/-0.1 rad/s. Each repeat holds WAIT/Stand for 15 s.
The output includes policy/deploy/CSV hashes and exact handoff command continuity.
`prepare_serve_policy_bundle.py --model modelfusion` accepts only the matching
CSV name, reproducible recipe, passing serve simulation receipt and strict native
policy/serve inspection. This staging command does not deploy hardware.

## Direct single-MLP repair

The repository's `v23-fusion.*` pair now belongs to the research candidate
`13800_clock_dx0.045`, ONNX SHA256
`89de4971efbb2524e2ea616aaa4b2af1bb64c3c3d1231c97b464872ad9d242ec`.
It retains the original 110 -> 512 -> 256 -> 128 -> 31 ELU MLP, all reference
outputs, PD gains and action feedback. It changes 1,024 of 225,055 parameters:

```
b0' = b0 + 0.045 * W0[:, 103]
W0'[:, 109] = W0[:, 109] - (0.045 / 0.82) * W0[:, 103]
```

This is equivalent to an internal forward-target compensation of
`0.045 * (1 - time_to_strike / 0.82)` metres. The external Planner target,
contact physics, evaluation target and action ABI stay unchanged. The correction
is zero in WAIT (time_to_strike=0.82). This is an interpretable local compensation,
not proof of a universal weakness or a newly learned skill.

```sh
python3 a3_deploy/a3_deploy_example/scripts/prepare_fusion_weight_edit.py \
  --source13800 /path/to/model_13800_policy --method clock --dx 0.045 \
  --output /new/path/weight-edited-fusion
```

Use that output for `adapt_fusion_serve.py`. The builder verifies exact source
policy/deploy hashes, changes only the stated first-layer entries, rejects an
existing output directory, and verifies 128 numerical samples. Its measured
maximum action/WAIT equivalence error was 1.91e-6; reference error was zero.
Inherited checkpoint metadata describes the source training run, while
`provenance.weight_edit` records the later weight edit. Reproduction matched both
policy and deploy files byte for byte. All five serve/WAIT tests and native
staging checks passed; staging did not install anything.

On the original 26-ball scene, this candidate produced 26 contacts, 26 legal
landings and zero falls. FH/BH original-target deadline FK error medians were
2.09/0.67 cm, versus 3.86/3.63 cm for model13800. Recovery torso yaw was 10.09
degrees versus model13800's 10.10 and model12000's 14.72. However, FH velocity
error increased from 0.166 to 0.226 m/s; BH was 0.317 versus 0.325 m/s. Formal
Gate3 remains FAIL, including real posture/wrist/heading issues. Do not deploy.

A separately frozen 26-ball scene perturbed serve positions and velocities without
changing physics or thresholds. Model13800 versus the edited candidate yielded
FH 4.48 -> 3.13 cm and BH 2.88 -> 0.72 cm deadline FK error medians. Recovery
torso yaw was 10.33 -> 10.94 degrees, so yaw did not improve in that scene.
This is one paired simulation run, not a repeated confidence estimate or physical
qualification. All formal reports remained FAIL.

The earlier phase-conditioned whole-body mixture remains reproducible above.
It had 2.31/2.56 cm position errors and 12.52 degrees recovery torso yaw. Constant
forward-bias and heading-gain edits improved some position results but failed
the yawed CSV transition test. Those variants are retained only as failed
experiments. Ridge output-layer repairs and supervised whole-body distillation
were also tried; these are explicitly fitting/training methods and must not be
presented as reproducing MIT's no-training experiment.

A newly fused actor must get its own regenerated CSV,
recipe and receipt even if the installed CSV basename remains `v23-fusion.csv`.
Never reuse the original CSV solely because a candidate has the same model label.

## Joint position, velocity and face-normal assessment

Target execution means all three quantities, not only position or its forward
component. `analyze_target_execution.py` samples position, velocity and the
physical racket face normal at the same original Planner strike deadline. It
retains signed XYZ errors, FH/BH medians, P95 and maxima. It rejects missing
deadline brackets, gaps over 50 ms, invalid normals and nonfinite measurements.
It never fills a missing deadline with another flight's samples.

```sh
python3 a3_deploy/a3_deploy_example/scripts/analyze_target_execution.py \
  --run baseline=/path/to/baseline-run --run candidate=/path/to/candidate-run \
  --output /new/path/target-pvn.json
```

In the current Planner, `racket_normal = unit(racket_velocity)`. The 110-input
actor receives target position in slots 103:106 and target velocity in 106:109;
it has no independent normal input. The normal direction is therefore available
through velocity under this specific Planner contract. That does not guarantee
the policy actually achieves the requested face orientation. An independently
specified normal would require a new observation/target contract and compatible
policy; changing this MLP's weights alone cannot expose an unobserved input.

The physical face normal is the racket site's +Y axis. For the two-sided paddle,
the primary orientation error is `acos(abs(dot(unit(actual), unit(target))))`;
the tool also retains the directed angle and incoming-side-aligned angle.
Otherwise a correct backhand face plane can appear almost 180 degrees wrong.
Runner velocity is a 20 ms backward difference; the optional plant derivative
is reported separately, never silently substituted into the same column.

The previous clock edit is not a joint p/v/n winner. On the original native
26-ball scene, FH/BH face-plane angle medians were 2.21/2.14 degrees versus
3.21/3.32 for 13800, but FH velocity error increased as described above. On the
additional scene, BH angle increased from 2.08 to 2.24 degrees. On 69 matched
recorded-state short rollouts, its overall position/velocity/normal medians were
3.31 cm / 0.353 m/s / 2.65 degrees versus 13800's
4.16 cm / 0.322 m/s / 2.46 degrees. These are different evaluation layers and
must be compared within the same layer, not pooled as physical A/B evidence.

The weight builder also supports bounded joint position/velocity compensation:

```sh
python3 a3_deploy/a3_deploy_example/scripts/prepare_fusion_weight_edit.py \
  --source13800 /path/to/model_13800_policy --method joint \
  --position-delta 0.0360402735 -0.0118378937 0.0153324824 \
  --velocity-delta 0.0951160792 0.0133211149 -0.0731396380 \
  --output /new/path/joint-research-policy
```

This is an experimental search direction, not a recommended deployment. For
`delta = [dp_x, dp_y, dp_z, dv_x, dv_y, dv_z]`, it applies
`d = W0[:,103:109] @ delta`, `b0 += d`, `W0[:,109] -= d/0.82`.
The actual face-plane residual participates in the local optimization even
though the architecture has no separate normal slot. The search uses 16 recorded
development cases and then evaluates all 69; the remaining 53 were seen during
earlier research, so they are not an independent holdout. Search residual scales
and weights are not Gate3 acceptance thresholds. No physics, target, PD gains,
runtime frame transforms or formal certification thresholds are changed.

The joint candidate was tested on both 26-ball scenes. On the original scene,
FH/BH position, Runner velocity and face-plane angle medians were
1.97/1.42 cm, 0.219/0.276 m/s and 2.38/0.66 degrees. On the additional scene
they were 2.80/1.63 cm, 0.283/0.306 m/s and 2.65/0.61 degrees. The extra scene's
13800 baseline was 4.48/2.88 cm, 0.301/0.378 m/s and 2.79/2.08 degrees.
Compared with the clock edit, this trades some backhand position accuracy for
lower backhand velocity and normal errors; forehand normal is worse. Recovery
torso yaw is 10.49/10.67 degrees in the two scenes, versus 13800's 10.10/10.33.
It is not a uniform improvement or a deployment recommendation.

All five joint-candidate CSV/WAIT cases and local native policy/serve staging
passed. The source builder's graph exactly matches the research candidate, with
1.91e-6 maximum numerical action/WAIT error and zero reference-output error;
the original clock build still reproduces byte for byte. Metadata changes the
whole ONNX hash, so only the exact tested research hash is paired with its CSV
and receipts. The repository's existing clock-policy/CSV pair is retained;
the joint candidate is a separate comparison. Both formal Gate3 runs failed.

## Known qualification boundary (2026-11-04)

The original USB hard splice passed native loading and all five serve/WAIT
simulation cases, but **fell in continuous-ball Gate3**. Do not deploy that
candidate. Serve/WAIT success does not qualify sustained hitting or recovery.

Offline research also covers whole-body action mixtures, ordinary and
neuron-aligned parameter interpolation, small parameter deltas around 13800,
countdown-dependent whole-body mixtures, and a 13800/12000/17400 three-teacher
mixture. Mixtures and hand-defined edits do not establish a newly trained
capability; supervised repair/distillation and simulation-based parameter
fitting are labeled separately in the research records. Keep the
unaltered source models and compare identical recorded states/targets and the
same physical Gate3 scene. Do not change contact physics, gains, target geometry
or certification thresholds to make a candidate pass.

Gate3 results, including failed criteria, own the continuous-ball verdict. Neither
offline FK error, finite network output, successful selection nor a PASSIVE
startup establishes physical hitting qualification. The research phase above
performed no robot installation, restart, selection or motion. The subsequent
user-authorized experimental deployment is recorded separately below.

## 13800 Astra experimental deployment (2026-11-04)

At the user's request, the exact joint candidate was installed under the separate
model ID `model13800astra`, displayed as **13800 Astra**. It does not replace
`model13800`. The original 13800 catalog entries and all catalog-hashed files
were checked unchanged after installation on both HDU and MDU. Its existing
`v23-13800.csv` remains paired with original 13800.

The generic joint builder reproduces the same weights and graph, but its research
metadata differs from the installed Astra artifact. For exact ONNX and deploy
bytes (including provenance), use the fixed preset; it verifies both expected
SHA256 values and refuses an existing output directory:

```sh
python3 a3_deploy/a3_deploy_example/scripts/prepare_model13800_astra.py \
  --source13800 /path/to/model_13800_policy --output /new/path/model13800-astra
```

The Astra identity is:

| Artifact | SHA256 |
| --- | --- |
| ONNX | `fa9f8470fab43da30d9884afee0a207f6e1eb6dedfdf8b98b7f2d2e5302a5a9d` |
| deploy.yaml | `0570621dbb1ba92c77aa8429d4eaec752dc51d962a264eecd1cd04876cc5e10f` |
| v23-13800-astra.csv | `b52352d127ad6e147d8355544ef953c59625b60a276eee1470bf4af54ac77f39` |

`v23-13800-astra.csv` is a rename of this exact joint candidate's tested CSV;
its bytes, policy pair and five passing serve/WAIT simulation cases are unchanged.
The clock candidate's `v23-fusion.csv` remains a separate research artifact.
`prepare_serve_policy_bundle.py --model model13800astra` requires the Astra
filename, reproducible recipe and matching receipts. Use its independent Console
button with PPQ004 and OptiTrack, through the existing stopped-state selection
checks. Never point original 13800 at the Astra policy or CSV.

Installed locations and rollback records:

- Astra MDU: `/agibot/a3_deploy_model13800_astra_pvn_fa9f8470`
- Original 13800 MDU: `/agibot/a3_deploy_model13800_serve_20261103`
- HDU profile: `/var/lib/hope-lifecycle/policy-models/model13800astra.json`
- HDU backup: `/var/lib/hope-lifecycle/model13800-astra-backup-20261104-fa9f8470`
- MDU backup: `/agibot/model13800-astra-backup-20261104-fa9f8470`
- Console: `hopeopen.hope-a3-console` version `1.9.13`

Managed STOP collected the preceding 13800 session. Managed selection and START
then launched Astra as session `model21800_20261103T211223Z`. Acceptance observed
`START_COMPLETE_RUNNER_PASSIVE`, lifecycle RUNNING and at least 30 seconds of
PASSIVE with advancing Runner state; alive, session match and native-policy
flags were true, command-fault latch false, and base state fresh/valid. All six
HAL/IMU readiness checks passed. The active process policy, deploy, CSV, Runner
and runtime-config hashes were checked. The existing hardware Runner, gains,
training target frame and watchdog/fall guards were retained. No serve or hitting
command was issued; the serve bridge reported no startup gripper command.

Source validation passed 49 policy-selection tests, 25 Console operator-profile
tests, TypeScript checking and the production build. These checks and PASSIVE
startup establish installation and controlled startup only. **Both formal Gate3
runs remain FAIL**, and physical hitting is not qualified. ELink functional
readiness also remained unverified due to missing required telemetry channels.
The earlier contact/landing counts do not override these boundaries.

To return to original 13800, use managed STOP, select **13800** and verify
`v23-13800.csv`, then managed START and PASSIVE checks. Keep both deployments;
do not restore an entire old catalog over a running session.

## Upstream source port

The 2026-11-04 HOPE branch preserves main's newer policy/catalog checks and native
operations service. Original 13800 selection/CSV support is included as a
prerequisite to keep Astra independently selectable. All deployment and physical
observations above are historical field results, not acceptance of this source port.
No policy is installed or selected by applying these changes. The original
13800, Fusion clock candidate and Astra joint candidate use three distinct IDs,
CSV names and artifact hashes. Formal Gate3 remains FAIL for research candidates.
