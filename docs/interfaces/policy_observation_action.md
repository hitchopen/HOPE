# Policy Observation And Action

## RallyV13 training/eval/deploy contract (2026-07-18; no artifact yet)

RallyV13 keeps the V12 110-D actor observation and 31-action joint order exactly. There is no new
actor feature, runner projection, hard READY gate or termination. The changed action-side training
contract is that every pre-clamp q-des column is measured against the official A3 hard joint range,
with a 5% per-side safety inset and a range-normalized top-4 Huber barrier. This supersedes both the
single-maximum whole-body debt and the waist-only V12 patch; the downstream SDK/MuJoCo hard clamp
remains unchanged.

Deterministic eval reports the mapping by joint name and never infers it from column position. The
host contract test verifies all 31 official URDF limits against `a3_pingpong.xml` and the C++
`pp_joint_limits.hpp` table. Eval-only `eval_clip_sequence` and `eval_gate3_sequence` configuration
are empty by default and never enter a training config or actor observation.

After a checkpoint passes the frozen four-report selector, the exporter may stamp recipe
`rally_v13`, version `1`, runtime `rally_final_v2`, status `gate3_candidate`. The exact V13-only
fields are:

- `hitter_pure_all_joint_qdes_barrier=official_hard,0.0500,0.0300,4,0.7500,all31`
- `hitter_pure_right_elbow_extension=1.2500,0.1200,0.3000,0.1200,forehand_only`
- `hitter_pure_strike_x_gate_margin=0.0150,0.0200,0.0400,0.5000,1.2500`
- `hitter_pure_post_swing_xlock=0.0300,0.0400,0.1000,1.5500,1.0000,1.2500`
- `hitter_pure_heading_debt=0.1000,0.5000,1.5500,1.0000,1.5000`
- `hitter_pure_ready_deadline=0.1000,0.1000,0.2000,0.1200,classes1+2+3`

V13 also retains V12's exact idle-left-wrist stamp. It deliberately does not carry
`hitter_pure_joint_qdes_max_blend` or `hitter_pure_waist_qdes_saturation`; exporter, Python
MuJoCo evaluator and native loader fail closed if either obsolete key appears. These fields are
provenance/compatibility checks only and do not add observations or a runner-side clamp.

## RallyV12 contract (2026-07-18)

RallyV12 does not change the deploy wire: actor observations remain the ordered 110-D
`hitter_pure` tensor below and actions remain 31 joint targets. The three V12 additions are
training rewards and deterministic-eval diagnostics, not actor observations or runner-side action
corrections.

A V12 ONNX is accepted only with the inherited complete V11/runtime-v2 contract plus:

- `hitter_pure_training_recipe=rally_v12`
- `hitter_pure_training_recipe_version=1`
- `hitter_pure_runtime_contract=rally_final_v2`
- `hitter_pure_deployment_status=gate3_candidate`
- `hitter_pure_strike_x_gate_margin=0.0150,0.0200,0.0400`
- `hitter_pure_idle_left_wrist_debt=0.0200,0.1200,0.0500,0.2500,0.2500,0.7500,outward`
- `hitter_pure_waist_qdes_saturation=soft_limit,0.1000,1.0000`

Exporter, Python MuJoCo evaluator and C++ loader validate these values exactly and fail closed on a
missing/mismatched field. The recipe marker is candidate provenance, not hardware approval.

## RallyV11 runtime/evidence additions (2026-07-18)

- The actor tensor remains 110 observations and 31 actions; this change does not add or reorder an
  actor term.
- A V11 ONNX must carry recipe `rally_v11`, recipe version `1`, runtime contract
  `rally_final_v2`, READY deadline/stance, positive-main-step, passive-head and recovery metadata.
  The C++ loader rejects incomplete or inconsistent metadata.
- The runner records planner status, lifecycle sequence/event/reason, pending-shot identity,
  strike time, time-to-strike, valid-command age and base freshness beside each observation row.
  These are diagnostic columns, not actor inputs.
- Gate3 carries robot localization through a process-isolated
  `hope_base_pose_flat_relay`: best-effort `PoseStamped` in, reliable
  `/a3/base_pose_flat=[1,valid,x,y,z,qw,qx,qy,qz]` out. Marker-local offsets are rotated by the
  normalized pose quaternion; invalid positions publish `valid=0`. The planner retains its legacy
  internal publisher by default, but Gate3 starts it with `publish_base_flat=false` so a long
  Stage2/3 solve cannot starve the runner's 0.2 s localization freshness contract.
- `eval_deterministic.py +paired_trace_out=...` records the exact actor observation, raw action,
  post-clamp joint target and full Isaac plant state. MuJoCo parity consumes the joint target; it
  does not independently rerun the policy in `open-loop-target`; `closed-loop-policy` instead runs
  the exported ONNX against each plant's current state and is the valid plant-attribution check.
- Normal deterministic eval reports the aggregate exact-strike EMA sample count, summed FH/BH
  count, individual clip counts, and the aggregate-minus-partition composite-rate delta. Per-clip
  rates are reconstructed from final raw counters only after the clip reaches
  `exact_success_min_count`; otherwise they are `NaN`, not a guard-generated zero. V11 checkpoint
  qualification uses 512 envs so both sides remain above the 50-sample guard.
- Runner trace CSV now includes `clamp_viol_<joint>` for all 31 backend slots, defined as signed
  `pre_clamp_q_des - published_q_des`. `[clamp-audit]` lines report cumulative
  `joint=hits/total_ticks/max_abs_violation_rad`; this is diagnostic evidence and does not change
  the joint-limit clamp or any Gate threshold.

Status: Implemented (deploy-parity contract, sim-to-real verified 2026-07-02; realigned 2026-07-03)

## HITTER-Compatible Contract

The baseline policy keeps the HITTER-style separation:

- Planner provides target racket position, target racket velocity, target racket normal, and time to strike.
- WBC policy combines planner target, robot state, and previous action.
- Policy outputs desired joint positions.

## Actor Observation (implemented): 175-D deploy-parity

Source of truth: `HOPEPingPongDeployParityAgibotA3EnvCfg` in `hope_env_cfg.py`
(`HOPEPingPongRealSensor` is a backward-compat alias). The canonical layout with per-term hardware
sources is codified and asserted in
`hope_training/whole_body_tracking/scripts/realsensor_obs_reference.py` and checked by
`scripts/verify_realsensor.py`; the C++ deploy runner builds the identical layout
(`pp_obs_builder.hpp`) and auto-detects 175 vs 180 from the ONNX input dim.

| # | Term | Dim | On-hardware source |
| --- | --- | --- | --- |
| 0 | `command` | 62 | reference clip future joint pos/vel (baked into the ONNX) |
| 1 | `motion_anchor_ori_b` | 6 | relative orientation ref-vs-robot (IMU + clip); no base position |
| 2 | `base_ang_vel` | 3 | pelvis IMU gyro |
| 3 | `joint_pos` | 31 | joint encoders (q − default_q) |
| 4 | `joint_vel` | 31 | joint encoders (dq) |
| 5 | `actions` | 31 | previous action |
| 6 | `projected_gravity` | 3 | pelvis IMU (gravity in base frame), Unoise ±0.05 |
| 7 | `racket_target_pos_b` | 3 | REFRAMED: `yaw(base)⁻¹ · (target_w − racket_FK_w)`, Unoise ±0.02 |
| 8 | `racket_target_vel_w` | 3 | planner desired racket velocity (world) |
| 9 | `time_to_strike` | 1 | swing clock (from clip strike phase) |
| 10 | `swing_type` | 1 | forehand `+1` / backhand `−1` (planner) |

Total = 175. Removed vs the legacy 180-D `full` layout:

- `motion_anchor_pos_b` (3) — reference torso position error; needs the world base pose.
- `base_target_pos_b` (2) — base-repositioning target; needs the world base pose.

The racket-target reframe makes the term base-position-free: because `quat_rotate_inverse` is linear
in its vector argument, `R⁻¹(target − racket) = (target rel base) − (racket rel base)` for ANY base
position — verified numerically in `realsensor_obs_reference.py`.

Notes:

- Desired racket normal is NOT an actor observation (HITTER Table I: normal is a reward target
  only). `base_lin_vel` is critic-only. `swing_type` is included because the default task trains
  one unified forehand+backhand policy.
- Rationale for base-position freedom: the mocap DOES stream the robot base pose during play
  (300 Hz, `/UCB_P1/pose` — see the deploy-available signal set below), but that VRPN link is not
  bridged into the deploy runner, and independence from it is a deliberate robustness choice. The
  earlier "no localizer on the real A3" wording was inaccurate.
- The legacy 180-D `full` contract (`task=HOPEPingPong`, model_15200 lineage) is kept for
  comparison only; it depends on world-base-position terms and is not deploy-honest. The deploy
  runner still accepts 180-D ONNX for legacy checkpoints.

## HitterPure / RallyFinal Actor Observation: 110-D

`HOPEPingPongHitterPureRallyFinal` keeps the existing `hitter_pure` 110-D actor contract exactly, so
the clean `model_18400` actor/critic can warm-start strictly. No new actor observation was added for
settle, slip, clearance, or torso shaping; those are reward/evaluation-only simulation quantities.

| # | Term | Dim | Deploy source |
| --- | --- | ---: | --- |
| 0 | `base_ang_vel` | 3 | pelvis IMU gyro |
| 1 | `joint_pos` | 31 | encoders, `q-default_q` |
| 2 | `joint_vel` | 31 | encoders |
| 3 | `actions` | 31 | previous policy action |
| 4 | `projected_gravity` | 3 | pelvis IMU orientation |
| 5 | `base_forward_xy` | 2 | pelvis orientation / mocap base heading |
| 6 | `base_target_delta_xy` | 2 | runner station minus localized base xy |
| 7 | `racket_target_rel_base` | 3 | planner target minus localized base position |
| 8 | `racket_target_vel_w` | 3 | planner |
| 9 | `time_to_strike` | 1 | runner clip clock |

Total = 110. There is no reference-joint stream, swing-type scalar, desired racket normal, actual
racket FK state, root linear velocity, or ball state in the actor. The target face normal remains a
training reward/critic target derived from desired racket velocity, as in HitterPure.

RallyFinal changes the command distribution, not the tensor layout:

- FinalV3 station x is fixed at the environment/spawn x and its station-relative strike plane is
  x `0.70 m` (rounded V7 FH/BH reach `0.724/0.680 m`); older HitterPure models remain x `0.51 m`;
- FinalV3 runner and task use `station_y = racket_target_y - reach_y[clip]` exactly with
  reach-y `-0.04/+0.10 m` (older HitterPure models retain their ONNX-baked offsets);
- repaired FinalV3 uses the venue mocap floor-frame contact-height band `[0.98,1.26] m` for both
  sides and HITTER opponent-half-center velocity envelopes (FH `x[0.25,1.35], y[0,0.35],
  z[0.10,1.00]`; BH `x[0.25,1.35], y[-0.15,0.12], z[0.10,1.00]`). These are planner/WBC interface
  commands derived from position-history state estimation; no ball state enters the actor;
- station y lies in `[-0.35,0.35] m`; consecutive training wraps command `0.20-0.35 m` lateral
  changes; the runtime planner may request smaller changes, but must remain within the trained
  absolute station/target envelope;
- idle/rest timing is a runner contract: clamp the clock/target while waiting, release only when the
  runtime readiness gate passes. Training contains no V5 hold or arrival-gated clock machinery.
- localized base position is live deploy input; a stale stream blocks new engage and leaves a committed
  swing on the last measured pose, rather than substituting a reference-motion base pose.

The ONNX carries `actor_obs_contract=hitter_pure`, `actor_obs_total_dim=110`,
`hitter_pure_pos_range_per_clip`, `hitter_pure_vel_range_per_clip`,
`hitter_pure_base_target_range`, and (for RallyFinal) `hitter_pure_station_y_step_range` plus
`hitter_pure_runtime_contract=rally_final_v1`. A RallyFinal checkpoint must be exported with
`TASK=HOPEPingPongHitterPureRallyFinal`; the native runner requires the marker and the complete
position/velocity/base/step metadata together, derives the station from the baked fixed-reach box
centers, and fails rather than substituting another task's boxes.

`RallyFinalV2` does not change this 110-D tensor or the runtime wire behavior, so it deliberately keeps
`hitter_pure_runtime_contract=rally_final_v1`. It adds audit-only ONNX metadata
`hitter_pure_training_recipe=rally_final_v2`, `hitter_pure_training_recipe_version=2`, and
`hitter_pure_ready_hold_steps_range=40,60`. Export validation requires all three for a V2 task and
checks the same timer in the saved run config; an old RallyFinal/model_13000 or initial 25-50 V2 ONNX
therefore cannot be mislabeled as having the current deploy-ready clamp training. The recipe stays
`rally_final_v2` / version `2` because the observation, action and runtime schema did not change; the
dedicated hold field versions this exact timer hyperparameter fail closed.

`RallyFinalV3` also remains 110-D, but makes the two head channels passive end-to-end: head q-des is
fixed at the articulation/deploy default and the two applied previous-action slots observed by the
actor/critic are exactly zero. The actor still receives no ball state, hold flag, root linear
velocity, clearance distance, foot contact, torso pose, or privileged settle signal. Planner swing
side is an external runtime state-machine choice, not a new actor scalar; the fixed FH/BH target
reach already disambiguates the actor target.

V3 ONNX metadata adds recipe `rally_final_v3` / version `3`, fixed hold `40,60`, station mixture
`0.15,0.25,0.03,0.12,0.20,0.35`, explicit-side/passive-head/physical-termination markers, validator
profile, and hashes of both motions, the validator receipt and resolved training env. Train, export
and C++ load all fail closed on mismatches. For the approved V7 compatibility recipe, measured
contact states remain diagnostics while the deployed position/velocity boxes intentionally stay the
proven FinalV2/HitterPure command geometry; runtime scripted target centers are derived from those
metadata boxes rather than hard-coded constants.

`RallyV8`, `RallyV9` and `RallyV10` keep this 110-D actor tensor unchanged. Their recipe/runtime
pairs are `rally_v8/4 + rally_final_v1`, `rally_v9/5 + rally_final_v1`, and
`rally_v10/7 + rally_final_v2`; recipe 6 was intentionally skipped and is invalid. Runtime v2 is
the deploy capability marker for component-aware velocity gating, so V10 and v2 must appear
together. An old v1 loader rejects the model instead of treating the Cartesian union as trained
support. V9/V10 retain passive head joints, applied-zero head last-action feedback and the
reference-feet/no-z-only-wrist termination contract. V10 additionally requires:

- `hitter_pure_left_wrist_reference=hold_frame0_swing_current`;
- `hitter_pure_pre_settle_t_max=1.1000`;
- `hitter_pure_yaw_settle=first_order`;
- `hitter_pure_joint_qdes_max_blend=0.5000`.
- `hitter_pure_strike_x_drift=0.0200,0.0500,1.1000,1.5500,huber`;
- `hitter_pure_right_elbow_extension=1.3000,0.1500,0.2500,0.1000`;
- `hitter_pure_fixed_plane_x=0.5800`;
- `hitter_pure_vel_core_range_per_clip` and
  `hitter_pure_vel_planner_range_per_clip` (two six-value clip boxes each);
- `hitter_pure_vel_planner_mix_prob=0.7500` and
  `hitter_pure_vel_range_ramp_steps=96000`;
- `hitter_pure_vel_curriculum=v13_core25+planner_q05_q95_75,96000`.

These fields describe training and validation semantics; they add no actor input and change no
action decoding. Before launching Isaac, export also verifies the checkpoint run's resolved
`params/env.yaml` contains the exact V10 reward functions, values, left-only joint set and disabled
z-only wrist termination. Export, MuJoCo and C++ then fail closed on recipe 6, a V10/v1 mismatch,
or missing/drifted component metadata, preventing a normal V9 checkpoint from being relabeled as
the V10 recovery policy. The final union envelope remains a coarse speed-safety bound; engage and
mid-swing refinement accept only velocities contained by the core or planner component as a whole.

## Critic (privileged) Observation (implemented)

The critic group is unchanged between full and deploy-parity (~318-D) and is never deployed. Base
`PrivilegedCfg` terms: `command` 62, `motion_anchor_pos_b` 3, `motion_anchor_ori_b` 6, `body_pos`
42 (14 tracked bodies × 3), `body_ori` 84 (14 × 6), `base_lin_vel` 3, `base_ang_vel` 3,
`joint_pos` 31, `joint_vel` 31, `actions` 31 — all noise-free. HOPE additions: `base_target_pos_b`
2, `racket_target_pos_b` 3, `racket_target_vel_w` 3, `racket_target_normal_w` 3, `time_to_strike`
1, plus sim-only actual racket FK state `racket_pos_b` 3, `racket_lin_vel_w` 3, `racket_normal_w`
3, and `episode_time_left` 1. Privileged sim-only info is allowed here by the training contract.

## Action

- `ActionsCfg.joint_pos` with `joint_names=[.*]`, `use_default_offset=True`.
- Dimension = **31** in training AND in the deployed ping-pong runner: the ONNX outputs 31 actions
  (incl. `head_yaw_joint`/`head_pitch_joint`); the runner scatters them to the 31 SDK slots
  (`MakeA3Layout31`) and overrides the neck slots post-decode to `q=0, kp=40, kd=2`. The 29-DOF
  `ExpandToBackend` view belongs to AGI's official reference runner, not the HOPE ping-pong path.
  See [joint_order.md](joint_order.md).
- Decoder target = `action * action_scale + default_angle`, per-joint
  `action_scale = 0.25 * effort_limit / stiffness`; q_des clamped to MJCF joint limits; published
  as `{q_des, kp, kd}` to the implicit-PD backend (`dq_des = tau_ff = 0`).

## ONNX Metadata Contract

The whole obs/action contract travels with the model. `scripts/play.py` bakes into the ONNX:
`joint_names`, `default_joint_pos`, `action_scale`, `joint_stiffness` (kp), `joint_damping` (kd),
`body_names`, and the clip clock keys `clip_seg_lengths` / `clip_strike_phases`. Consumers
hard-validate at load and fail on any missing key, non-bijective joint map, obs input other than
[1,110]/[1,175]/[1,177]/[1,180], or any kp/kd ≤ 0 (zero-gain guard): the C++ runner (`pp_onnx_policy.hpp`), the
MuJoCo evaluator (`mujoco_eval_onnx.py`), and the contract diff tool
(`scripts/inspect_a3_deploy_contract.py`).

## Deploy-Available Signal Set

What the real system can observe (team contract, 2026-07):

- Robot-side (always): joint encoders (q, dq), pelvis IMU (quat, gyro), torso IMU (orientation),
  previous action.
- Mocap during PLAY (ChingMu/VRPN, 300 Hz): robot base (pelvis) pose, ball position. Ball
  rotation/spin is planned for the physics-modeling phase — not yet measured.
- Mocap during DATA-COLLECTION/physics-calibration only: additionally racket pose, table 4
  corners, net 2 corners.

Rules: actor observations must be built only from deploy-available signals; rewards run in sim
only and may use privileged state; the critic may be privileged. The current 175-D actor uses only
robot-side signals + planner targets (no mocap terms at all). NOTE: when the mocap→planner loop is
bridged, the HOPE-world → robot-frame target transform will need the mocap base pose at the
interface boundary even though the actor obs does not — that transform currently has no owner and
must be designed with the bridge (see G07 Next Steps).

## Table-Tennis Physics Scene Observation (experimental)

`HOPE-TableTennis-AgibotA3-v0` is a G04/G08 physics/visualization scene, not the accepted deployment
WBC policy. Its current actor observation is robot proprioception plus ball state in the base frame:
`base_lin_vel`, `base_ang_vel`, `projected_gravity`, `joint_pos_rel`, `joint_vel_rel`,
`last_action`, `ball_pos_b`, `ball_vel_b` (position/velocity only — no spin). The critic mirrors
these without noise. If this scene becomes a returner baseline, record dimensions, normalization,
reward targets, and deploy compatibility here first.

## Contract Knobs

- Control rate: 50 Hz, verified (sim dt 0.005 × decimation 4, `cfg/base/sim_base.yaml`).
- Training target source: `target_mode: uniform` with per-clip 3-D position and velocity boxes
  centered on the reference blade strike state (`pos_range_per_clip` / `vel_range_per_clip` in
  `cfg/task/HOPEPingPongDeployParity.yaml`); `strike_phase_per_clip: [0.47, 0.333]` on the
  re-grounded `_hopex` clips. The fixed `x=0.4` hit plane with (y,z)-only sampling is superseded.
- `racket.target_mode: reference_perturbed` remains a non-default option (it was
  the default on the pre-merge `rsi-on-wrap-progress-fix` branch): the target
  center is computed from each imitated clip's own strike-frame racket FK state
  (position, velocity, face normal), and the long-run distribution widens
  through success-gated perturbations. Either mode changes target generation
  only, not the observation/action tensor contract.
- Clip wrap for HOPE ping-pong does not teleport the robot mid-episode: the
  target and reference clip/time resample, but the policy must physically carry
  the body between swings (`MotionCommandCfg.wrap_teleport` defaults to false;
  the HOPE task YAMLs keep it explicit as `motion.wrap_teleport: false`). True episode resets
  still use reference-state initialization, except for the `stand_start_prob`
  fraction of envs that start from the default stand pose.
- Normalization: per-term Unoise with `enable_corruption` on the policy group (values in the actor
  table above).

## Update Rule

Any observation, action, scaling, normalization, latency, command frequency, or joint-order change must update this file and the relevant gate docs.
