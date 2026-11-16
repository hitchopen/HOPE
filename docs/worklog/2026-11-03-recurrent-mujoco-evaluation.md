# 2026-11-03 StationaryRally GRU CPU MuJoCo evaluation

- Base training commit: `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.
  Repository origin is `https://github.com/1786089174zth-cell/HOPE.git`.
  The clean training branch matched its fetched remote (0/0 divergence).
  Implementation is isolated in worktree `~/workspace/HOPE-stationary-gru-eval`
  on `feature/stationary-gru-mujoco-eval`. No existing feed-forward runtime or
  training source/configuration was edited, no dependencies installed, no push.
- Frozen evaluation checkpoint: `model_5300.pt`, SHA-256
  `e41296298ee9da9ec590a3751758fdfa495cc0913ca0f44d7fd4b9502c5dd816`.
  The snapshot and original `params/` remain outside Git at
  `~/workspace/hope-stationary-gru-evaluations/20261103-121753/`.
  Training run is `gru-formal-2b8c09938a`, PID 689032, tmux session
  `hope-gru-formal-2b8c09938a`; it continued throughout this task.
- `scripts/export_recurrent_mujoco.py` uses existing grasping PyTorch/RSL-RL/
  ONNX libraries on CPU, with no Isaac or SimulationApp imports. It strictly
  loads the recurrent policy, exports actual actor normalization/GRU/MLP and
  passive-head masking, and derives action/plant settings from the snapshot.
  This actor's normalizer is identity, confirmed from config and state keys.
  Hidden-state IO is explicit; checkpoint training-batch memory is never an
  evaluation initial condition.
- The opt-in host `recurrent_policy.py` and `scripts/mujoco_eval_recurrent.py`
  use the existing host MuJoCo venv. Memory carries across target changes and
  resets only with evaluation episode reset. Saved joint order, 110-D layout,
  nominal actuator parameters, affine safe action decode and delay FIFO are
  checked. Control is 20 ms with four 5 ms physics steps; total damping is
  counted once. The nominal MuJoCo implicit drive is an approximation of PhysX,
  not a claim of exact plant/task reproduction. See the explicit limits in
  `docs/RECURRENT_MUJOCO_EVAL.md` and the bundle manifest.
- Parity: 64 consecutive real-observation inputs, arbitrary nonzero initial
  memory, zero-memory episode reset at step 32. PyTorch explicit-vs-stateful
  oracle action error was `9.54e-7`; ONNX-vs-PyTorch max action error `2.38e-6`
  and hidden-state error `3.58e-7`. All outputs finite. ONNX tolerances are
  `atol=2e-5`, `rtol=2e-4` for float32 fused-kernel ordering and recurrent
  accumulation. Complete observation fixture, per-joint actuator force,
  action clamp/delay, saved racket mount (2 micrometers), and virtual-table
  placement/collision assertions passed.
- Final host tests: **9 passed, 1 skipped**. Three new recurrent tests verify
  carried memory/reset replay, nonfinite-input rejection without memory
  mutation, and bundle hash rejection. Existing action-adapter tests passed.
  Legacy ONNX export/metadata tests skip without the optional host `onnx`
  package; no dependency was installed to remove that skip.
- An initial headless/viewer attempt inherited the vendor Gate3 table at x=0
  with robot collisions. The actual saved virtual table starts at x=0.5.
  Those contaminated behavior results are **excluded** and retained separately
  in the external evidence directory. The fix moves visual geometry to the
  saved task frame and disables table/net contacts. The first viewer's SIGINT
  was followed by a native shutdown fault; the implementation now joins its
  viewer thread before releasing model/data. The complete final viewer run
  exited 0. These issues do not diagnose the earlier unrelated Isaac crash.
- Corrected headless smoke: 8 simulated seconds, both sides, zero falls,
  no numerical errors/warnings, hidden state retained across several targets.
- Final visible test on desktop `:1`: software rendering + CPU ONNX inference,
  **90.08 wall seconds**, 51.62 simulated seconds, 2,581 control steps, exit 0.
  It observed 13 forehand and 13 backhand reference strikes. There were **zero
  falls**, four episode-timeout resets (plus initial reset), and zero MuJoCo
  numerical warnings. Timeouts are quantized to control ticks.
- Final nominal-scene behavior: peak tilt 3.71 degrees, peak drift 0.0435 m;
  contact-foot speed p95 0.0314 m/s and peak 0.1825 m/s. Median target-position
  error at the reference strike clock was 0.1188 m FH / 0.0912 m BH; median
  racket speed 1.876 / 1.481 m/s; median velocity-target error 0.321 / 0.384 m/s.
  Recovery's position/speed-only READY proxy was true throughout its samples.
  Command reversals remain present, including larger initial reset transients;
  finite trajectories alone do not establish absence of oscillation.
- Scene semantics: green FH / blue BH target and velocity-reference markers,
  red measured racket marker, virtual table. No physical-ball contact or legal
  landing was measured. This is not a Gate 3 qualification. It omits training
  randomization, noisy/stressed mocap, full adaptive/replay/interruptible
  scheduling, and reward/reference-body termination managers.
- Commands, bundle, configs, parity inputs, tests, traces and reports are saved
  outside Git under
  `~/workspace/hope-stationary-gru-evaluations/20261103-recurrent-implementation/`.
  `reopen-model5300.sh` reopens this exact frozen actor and evaluation scenario.
  The documentation explains snapshotting and verifying another compatible
  saved checkpoint before viewing it.

## Reference-target visibility follow-up

- Inspection confirmed that the evaluator has a strike point, desired racket
  velocity, and strike clock, but no evaluated ball trajectory. Its existing
  small sphere was a target, while the vendor ball meshes are hidden off-court.
  No flight, bounce, contact, or return path was synthesized.
- Enlarged the target to radius 0.04 m, added a yellow crosshair, a 3-D
  `REFERENCE STRIKE TARGET` label, and a persistent overlay with side/clock and
  an explicit statement that ball flight/contact/bounce is not simulated.
  Policy inputs, inference, state, actuator settings, and the camera are unchanged.
- Four recurrent/visibility tests pass. The new test confirms that marker
  position remains the supplied strike point on both sides of clock zero and
  that drawing does not mutate target position, plant qpos, or action history.
  The repeated eight-second headless trace is byte-identical to the previous
  validated trace: SHA-256
  `10a02c0b07e79f82eb1d77f5613dec9abc9cfb67ffa2e392cb3266b9aeab76b7`.
- Default-camera desktop screenshot and bounded viewer-check evidence are in
  `~/workspace/hope-stationary-gru-evaluations/20261103-reference-marker/`.
  Its `reopen.sh` opens the updated reference-target visualization.

## Physical-ball follow-up

- Work remains isolated in `feature/stationary-gru-mujoco-eval` and its separate
  worktree. The training checkout's HEAD and clean working-tree status were
  checked read-only; no training source, process, dependency or hardware action
  was changed. Pre-existing staged evaluator additions were retained.
- Reused the existing vendor ball, racket mesh, table, net and floor geometry;
  the MuJoCo restitution/damping helper; and a NumPy port of the training
  `physical_ball.back_integrate_incoming` / `virtual_ball.rk4_step` construction.
  Inspected the saved task, bank fields, physics profile, Isaac instrument,
  `mujoco_pingpong_scene.py`, `mujoco_eval_onnx.py`, and serve-app components.
  Their legacy no-spin serves/policy controller were not substituted for the
  saved recurrent policy. Isaac's code-driven impulse/blade-plane snap was
  excluded because it is incompatible with actual collision-only evaluation.
- The same verified model_5300 bundle is used without export or weight changes.
  Target inputs remain bank strike position relative to base, desired racket
  velocity in world coordinates, and countdown. Incoming velocity/spin drive
  the ball launch only and never become actor inputs. Normalization, named-joint
  decode, 20 ms control, delay FIFO and episode-only GRU reset remain unchanged.
- The SHA-checked PPQ004 profile supplies the 3.4 g, 20 mm, shell-inertia ball,
  gravity, quadratic drag, linear Magnus, table contact and velocity-dependent
  paddle restitution. Net/floor materials come from the existing no-spin
  profile, explicitly recorded as a fallback. The reconstructed serve is the
  final post-bounce segment, at most 0.6 s, truncated above table+R+5 mm or at a
  40 m/s reverse-speed guard; the bank cannot identify the earlier incoming
  bounce. Actual flight is never snapped to the racket or changed at strike time.
- An initial 5 ms collision smoke exceeded the ball speed bound and is excluded.
  Final physical mode uses 1 ms collision steps (20 per actor tick) and bounded
  direct-format contact stiffness 100000. The soft-contact model approximates
  normal restitution and Coulomb spin transfer; it is not fitted tangential
  impulse parity. A vertical table drop measured normal-speed ratio 0.9503
  against configured 0.9373. The training virtual capture geometry and the
  vendor approximately 162 mm racket mesh also differ; documented in the guide.
- **19 tests pass**: recurrent/reset/action regressions, unchanged physical-mode
  observation/action contract, both-side reverse-serve roundtrips and truncation,
  positive-force contacts on all four surfaces with bounded response, drawing
  without state mutation, contact-gated legal outcomes, and censored incomplete
  shots. Final 64-step frozen actor/oracle verification passes in physical mode.
- Live desktop launch was attempted and failed with GLFW error 65550,
  `X11: Failed to open display :1`; `xdpyinfo` also cannot open it. Direct X11 Unix-socket connection also returns
  `PermissionError(1, Operation not permitted)` with readable Xauthority. No live
  desktop run is claimed. Existing EGL software rendering and ffmpeg provide
  the 60-second visible recording without installing dependencies. Camera
  lookat [1.35,0,0.85], distance 6.2, azimuth 115, elevation -24 includes the
  robot/net/both halves. Orange is actual incoming flight; cyan starts at
  actual racket force contact; dashed grey is collision-free reference flight.
- Evidence, source snapshots, seed 0, configuration/source hashes, verification,
  event/shot/trace logs and `reopen.sh` are outside Git under
  `~/workspace/hope-stationary-gru-evaluations/20261103-physical-ball/`.
  `recorded-60s/` is the authoritative final evaluation; earlier smoke and
  supplementary headless runs are excluded from its result claim.
- Authoritative recorded run: **60.000 simulated/video seconds**, 1,200 frames at
  20 fps, 800x500; software rendering took 297.41 wall seconds. Seed 0, 3,000
  control ticks, 15 FH + 15 BH questions. **20 positive-force racket contact
  onsets, 10 misses, 8 legal returns (2 FH / 6 BH), 2 net contacts, 19 total table
  contacts, 23 floor contacts**. Two hit shots were interrupted before resolving;
  they remain explicitly incomplete. No falls; four timeout resets plus initial
  reset; zero reported MuJoCo warnings or numerical failures. Peak active-ball
  speed 5.6573 m/s. `trace.csv` is byte-identical to the supplementary headless
  trace, confirming that offscreen visualization did not alter the rollout.
- MP4 duration/frame count was checked with ffprobe, the final outgoing-flight
  frame was inspected, and full video decoding passed. Final source snapshots
  and exact commands are retained with evidence. Changes are staged in the
  evaluation worktree; no commit or push was made, and the training checkout
  remains clean at `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.

## Latest formal-run checkpoint request (21:34–21:37)

- Selected the latest published checkpoint, `model_9100.pt`, from the same
  `gru-formal-2b8c09938a` run directory at 21:34:46 local time. Saved pickle
  metadata confirms iteration 9100 and that run ID without executing pickle.
  The trainer publishes through a temporary file and atomic `os.replace`.
  Copied from one descriptor with unchanged inode/size/mtime, rehashed source
  and destination, and checked every PyTorch ZIP member CRC successfully.
- New independent snapshot/evidence directory:
  `~/workspace/hope-stationary-gru-evaluations/20261103-213446-latest-gru-formal-2b8c09938a/`.
  Checkpoint SHA-256:
  `5c34fb55d22bbf827b9b6138f6b0a8c32fa9ab1255ce78a9b3385edb7b09dc80`.
  All three params YAML hashes match model_5300. Training checkout remains clean
  at `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`; evaluator source hashes also
  match the authoritative model_5300 physical evaluation. No environment tuning.
- Export was attempted through the existing grasping/Distrobox CPU pipeline but
  blocked before Python: Podman cannot chmod `/run/user/1001/libpod` on the
  sandbox's read-only runtime filesystem. The host venv lacks torch, onnx,
  tensordict and rsl_rl. No container was created, dependency installed, runtime
  security changed, bundle written, or model_5300 artifact overwritten.
- Consequently no new actor-load/parity result or model_9100 evaluation is
  claimed. `status.json` explicitly records NOT RUN. Prepared desktop-terminal
  scripts outside Git: existing exporter invocation, independent checkpoint-vs-
  ONNX MLP/GRU initializer verification and old/new actor fingerprints, unchanged
  manifest-contract assertions, parity/reset/collision tests, 60-second seed-0
  physical video/evaluation, comparison, and training-progress checks.
  Shell syntax and Python compilation checks pass; export-dependent execution
  remains unverified. The guarded `reopen-model9100.sh` points only to the new
  bundle and refuses to run before export/verification succeeds.
- Training was not stopped or signaled. Its existing console log advanced from
  iteration 9104 at snapshot to 9120 at 21:37:06, proving continued progress.
  The source checkpoint, config identities, blocked command, baseline metrics,
  and exact desktop commands are recorded in this new evidence directory.

## 2026-11-05 latest physical-ball viewer preparation

- Re-inspected the same formal run; latest published checkpoint at selection
  was **model_29100.pt**, not model_9100. Snapshot directory:
  `~/workspace/hope-stationary-gru-evaluations/20261105-211651-gru-formal-latest/`.
  Saved metadata confirms iteration 29100, schema 3 and run
  `gru-formal-2b8c09938a`. SHA-256:
  `0dcc9be06c35dc1a4a43df86d624b36cb358a97c9d534947991b03569b5c79d0`.
  Snapshot copied from one descriptor with unchanged inode/size/mtime; source
  and copy hashes match, all ZIP member CRCs pass, and saved params match the
  earlier model_5300 configuration. Training checkout remains clean at 33c6090.
- Training console log advanced from iteration **29123 to 29132** during this
  task; no stop/restart/signals or training writes were performed.
- Sandbox attempt of the complete launcher passed snapshot/config/evaluator
  source-hash preflight, then stopped on Podman's read-only
  `/run/user/1001/libpod` runtime error, before Python/export. No new bundle,
  loaded-weight verification, parity/reset result, headless result or visible
  evaluation is claimed. These are explicitly pending in `status.json`.
- Prepared one desktop-terminal entry point, `desktop-view-model29100.sh`, in
  the new evidence directory. It prints selected iteration and new bundle
  path, rejects existing output directories, requires the existing grasping
  container (never offers to create/pull one), exports on CPU through the
  existing recurrent pipeline, independently matches ONNX MLP/GRU initializers
  to the snapshot, and asserts unchanged policy/environment manifest fields.
  Then it runs 64-step sequential/reset parity, the recurrent/physical/action
  tests and an eight-second seed-0 headless check before opening the physical
  viewer for at most 60 simulated / 120 wall seconds. It inherits desktop X11
  credentials without modifying security and stops on every pipeline error.
  No old bundle fallback is possible. The existing evaluator provides actual
  contact/miss flight and trail visualization; no evaluator code was changed.
- Shell syntax and Python compilation checks pass. Scripts, exact snapshot
  identities, preflight/blocker log and before/after training evidence remain
  outside Git in the new directory. Worklog update is staged; no commit/push,
  dependency change, hardware connection or prior-artifact overwrite occurred.

## 2026-11-05 professor fixed-home-100 reference audit

- User steered the pending planner evaluation to the professor's existing
  fixed-home-100 Gate3 scenario. No custom/scored run had occurred. The previous
  shuffled/long-slot protocol and its launch command are superseded; its launcher
  and draft driver now stop before runtime actions. Draft code/evidence is kept
  for development history, not passed off as a reference benchmark.
- Read all four requested documents/scenario/tests and followed the official
  resolver, fixed-home wrapper, conductor, recovery, physical and plant-report
  implementation. Verified live public `hitchopen/HOPE` main through GitHub at
  `7990f18c2c1ae560c9e0708dca10228b9f2cc610` (#119). The requested four files and
  six additional resolver/scoring helpers were downloaded at that SHA; their
  Git blob hashes and byte equality to this checkout were checked. The original
  `dongc1/nohope` fork's local cached upstream/main is `c7986ee5...`; its live
  HEAD could not be verified (shell DNS error, connected API404), not conflated
  with the successfully verified public upstream.
- Saved the exact official 100-serve ledger/environment outside Git at
  `~/workspace/hope-stationary-gru-evaluations/20261105-gate3-reference-audit-rkea78z8/`.
  Sequence: six clean A B C D E F, then 4x the 20-row rapid cycle plus its first
  14 rows. Counts A/B=20 each and C/D/E/F=15 each; nominal 50 lanes per side but
  no forced side labels. Timing: 6x2.5s, 93x1.15s, final2.5s, with six1s pauses,
  one20s initial MOTION idle, >=3s post-completion tail, and natural-flight drain.
  Nominal slot sum after first launch is130.45s, not a total runtime cap.
- Existing reference unit tests: **32 passed**, including scene composition and
  continuous-flight lifecycle. Composition-only mock inspection captured exact
  inherited thresholds without launching native Runner/ROS/MuJoCo. This is not
  a recurrent native-inspection or physical benchmark result.
- Audit found substantive gaps: native policy IO is obs/time_step (or generic
  single-input obs_dict), not this GRU's obs/h_in + h_out; host manager disables
  return bodies and cannot preserve every old flight during rapid handoffs;
  host READY/landing/scoring lacks full native lifecycle and plant evidence.
  Neither dimensions=110 nor a successful host rollout establishes native ABI
  compatibility. No metadata spoofing, recurrent reset-per-ball workaround,
  trajectory truncation, threshold relaxation or formal Gate3 claim is allowed.
- Reference requires100 engage/complete,94 rapid with93 preempt+1native closure,
  >=84global qualified contacts/legal landings, side floors/rates and full
  natural terminal evidence. Recovery is strict/reversible continuous0.3s with
  XY<=.05m, speed<=.12m/s, tilt<=12deg, yaw<=5deg, pelvis>=.8m. Audit also flags
  a wrapper comment/code mismatch: tracking-error RMS>.15rad actually fails the
  plant report, despite the shell calling it diagnostic. Executable behavior
  was retained. Complete threshold matrix and adaptation boundaries are in
  `docs/GRU_FIXED_HOME_100_AUDIT.md`.
- Training checkout remains clean/read-only; its log was progressing at29380
  during this audit. No signal, restart, package/hardware/security operation or
  scored evaluation was performed. Any future host-based adaptation must be
  labelled reference-sequence GRU diagnostic; formal Gate3 remains NOT_PROVEN.

### 2026-11-05 concurrent reference-sequence diagnostic preparation

- Implemented nine independent physical balls, current-incoming-only actual planner feed, per-ball evidence/trails, natural endpoint reuse and bounded drain. Rapid handoffs preserve outgoing flights. Fall ends launches/rally without robot or hidden-state reset/rescue; unlaunched serves remain explicit. Original official ledger, timing, 20 s idle, final tail and strict scoring thresholds retained.
- Added isolated ROS adapter, timestamp/frame/deadline checks, separate rapid-handoff preflight, bounded diagnostic, offline actual-state video renderer and fail-fast desktop pipeline. Native fitted impulse/snap physics and lifecycle/recovery differences are documented in `docs/GRU_FIXED_HOME_DIAGNOSTIC.md`; no formal Gate3 claim.
- Froze run gru-formal-2b8c09938a model29500 at22:19:19 PST, SHA256 b53a031b6d2dacceeebe7301412d3d2c5fc456f445c6aebb357e4159871683d0, source33c609022efbb4e85fe6a58a5a8947b0fbd46c76. Stable copy/config/hash receipts are outside Git in `20261105-221919-fixed-home-gru-diagnostic`.
- 56 focused lifecycle/regression/reference tests plus5 command-contract tests passed. Model5300 was only a unit fixture. Offline renderer smoke produced15 frames without actor/planner inference and is explicitly not the requested diagnostic video.
- Attempt attempt-SXJmGLZa stopped before export: Podman chmod /run/user/1001/libpod is read-only. New actor-load/parity/reset checks, live planner preflight and100 serves have NOT RUN. No new bundle or diagnostic video exists. Desktop launcher performs these in order and never falls back to an old bundle.
- Read-only training observation at22:44:19 PST showed iteration29702 with log updated22:44:17; training checkout clean at33c6090. No training signals/settings/dependencies/hardware changes.

### 2026-11-06 spatial/temporal alignment investigation (100 serves held)

- Investigated actual-planner model29500 preflight attempt-z8SGbdOv, same IDs1,2,7,8,9. Frozen checkpoint SHA b53a031b6d2dacceeebe7301412d3d2c5fc456f445c6aebb357e4159871683d0; ONNX f50b0de8c79a83c3e48868d47bc8b922094353aebdea01540ec70b662541a58b. Saved original source/hash receipts before implementation changes. No target, serve, physics or deadline moved.
- Recorded-action reconstruction at1kHz matches all679 saved qpos frames exactly. Independent scratch forward computes actual finite-mesh surface distances and full racket orientations. Both clean balls cross x=.58 at173.830ms before planned strike, z1.277201 vs predicted1.051297. At deadline ball is33.769cm past target x. FH/BH closest surface gaps .137797/.144367m, at21.063/24.560s. Target-site errors at deadline .070041/.035889m; normal errors4.163/6.235deg. Pre-bounce planner-fit position agrees within .15micrometres; known native fitted table impulse vs host soft contact is a demonstrated prediction/plant mismatch. Detailed trace in docs/GRU_STRIKE_ALIGNMENT_AUDIT.md.
- Fixed demonstrated .65s actor countdown cap: training env has clip:null and .35-.95s prep range. It affected only two ticks per serve (25.060ms,5.060ms), not the173.830ms ball mismatch. Removed unused bridge close acknowledgement causing baseline teardown EPIPE. No arbitrary timing compensation, geometry/physics/target change, actor input/action injection or recurrent reset. Frozen config causal_idle=false; did not enable newer optional idle-clock behavior.
- Added focused contract/interpolation tests and forensic alignment/video tools.33 selected tests pass with the actual frozen29500 bundle; identity/hash check passes, annotated baseline video decodes. Full command/raw message/obs slices/frames/deadlines and per-ball1ms evidence saved outside Git in 20261106-alignment-model29500. Video is baseline actual-state reconstruction, not an after-fix live planner result.
- Original100 launcher held using HOLD-100-SERVE-ALIGNMENT.txt. Added --alignment-preflight mode (preflight only, actual planner, same five serves) with automatic alignment report/video. Repeat attempt-2DdNYIJp blocked before container entry: Podman /run/user/1001/libpod read-only. No live after-fix result or100 diagnostic. Exact desktop repeat command in audit.
- Training checkout remains clean/untouched. Read-only final log now shows29999 and W&B synced-summary at2026-11-05 23:26:18 PST; model_29999.pt exists. No training process signals/restarts/settings/dependencies/hardware operations; retained29500 for this comparison.

### 2026-11-06 actual C++ trajectory / table-impact diagnosis

- Input is actual-planner corrected-countdown preflight attempt-vT5mOho1, same frozen29500 and IDs1,2,7,8,9. All679 qpos frames reconstruct exactly; zero capped-countdown ticks. All five recorded strike predictions/deadlines reproduced exactly using instrumented existing C++predict_with_spin, compiled with existing c++/Eigen headers only. Raw traces/CSV/JSON are outside Git in20261106-trajectory-impact-model29500.
- Timestamp/frame/gravity/drag/radius/table-plane/spin checks pass; pre-bounce position difference <=.0545mm, velocity <=.00455m/s. Consequential divergence starts at table impact t20.60184/20.602. Planner instantaneous v_x -2.35667->-1.37530, spin_y -73.60266; host10ms soft-contact separation v_x -2.20329, spin_y -8.95279. Host copied restitution/friction into soft constraints but never implemented archived a_t=.41642145 capped-slip law. This explains34cm deadline mismatch and changed Magnus lift.
- Added explicit table_impact_model=ppq004 evaluation mode using archived law only after positive-force real finite-geometry table contact; overwrites that first-step response, no qpos/target/deadline changes, no racket override, no GRU reset. Suppresses only that ball's duplicate table constraint until natural separation;50ms guard fails closed. Old soft mode preserved and old artifacts untouched. Physics revision is explicit, not a claim of unchanged native/Isaac physics or policy improvement.
- Isolated ball/table-only regression reproduces all old errors then reduces same-deadline errors to[-.001181,0,-.005329]m (~5.46mm) with only the law changed. Its racket-disabled unit setup is NOT a live actor/planner preflight and scores no success. Real preflight keeps all racket contacts.43 focused tests pass, including7 map/contact and3 replay tests;7 impact tests rechecked after bounded-separation guard.
- Added actual C++ versus1kHz measured-state checkpoints at source, pre/post predicted and actual bounce, separation, actual strike-plane crossing and deadline. Added explicit --impact-preflight (only same five serves) with automatic alignment/comparison/video; --replay-preflight [attempt-name] checks phase/policy/receipt and never falls back to scored video. Pointer defaults to existingvT5mOho1 until a new preflight video exists.
- Live repeat attempt-FlGiKewV blocked before container entry by Podman chmod /run/user/1001/libpod read-only. No live after-map result or100 serves. HOLD-100-SERVE-ALIGNMENT.txt remains. Exact desktop repeat/replay commands, root-cause equations, parameters and limitations in docs/GRU_TABLE_IMPACT_DIAGNOSIS.md. Training checkout clean/untouched; no dependency/hardware/X11-security changes.

- Exact-attempt desktop replay now reads the preflight request pointer set before runtime checks and supplies its basename explicitly; it cannot replay an earlier video after a failed new launch. Archived law YAML and C++ SHA256 recorded in the impact audit. Frozen29500 and100 hold unchanged.

- Desktop preflight launcher now prints and validates the active PPQ004 profile SHA256 alongside the selected impact mode before runtime entry. --impact-preflight still selects only preflight with frozen29500; exact-attempt replay remains explicit.

### 2026-11-06 user-authorized corrected-mode100-serve diagnostic

- Independently audited attempt8aUCdxAm config/profile, positive-force PPQ004 table events, actual C++ planner audit, raw bridge trace, exact actor observation fields, current-ball ownership, monotonic stamps,5.090ms max skew, final tail and every natural terminal. Five commands consumed34/34/34/35/34 ticks, zero resets/falls/errors/remaining flights, peak concurrency2. Infra PASS independently of video/integration_pass flag. Hitting remains5 raw contacts,0 qualified,0 legal,0 recovery; no success inference.
- Added explicit --impact-100 selecting only scored/ppq004/100, printed checkpoint+mode+profilehash+count, and strict archived-preflight gate before runtime. It selectively supersedes the legacy alignment hold only for this authorized corrected path; old/default soft runs remain held. No --preflight flag reaches the scored evaluator.13 gate/lifecycle/countdown tests passed; malformed physics/actor delivery/drain/planner errors fail closed.
- Added reporting-only planner failure, unresolved, resolved miss and failed-return fields plus falls/mode/profile/count metadata and CSV details. Reference contact/landing/recovery logic and thresholds unchanged. Same audited100 ledger,20s idle,6clean+rapid sequence, outgoing concurrency, nointerball resets/rescue, fall ends rally preserved. Artifacts/video remain outside Git in unique attempt directories.
- Attempt-aJMqoQHA gate passed and printed29500/ppq004/scored/100/hash df55e821..., then Podman runtime read-only error blocked before simulation. No100 outcomes or video exist yet. ONE exact desktop command --impact-100 in docs/GRU_PPQ004_100_DIAGNOSTIC.md. Outside-Git readiness/audit/source/blockedlaunch evidence in20261106-ppq004-reference-100. Training checkout untouched/clean; no weights, thresholds, dependencies or hardware changes.

### Read-only audit of completed attempt-eqQciohc (lane/side repetition)

- No evaluation, MuJoCo, actor or planner run repeated. All original attempt files SHA256-verified unchanged before/after. Official scene expansion compared to per-ball rows, actual launch events and first measured ball states:100/100 match positions/velocities/order/timing; lane countsA20 B20 C15 D15 E15 F15, nominalACE50/BDF50.
- Actual wire/planner/adapter/row labels agree50FH/50BH;100valid/accepted/applied,0rejected/unassigned. All100 actor target fields matched realcommands. Actor110-D contract has NO side input. Per-ball CSV/JSON include world/table target, exact actor translation-only target and full body/yaw coordinates using recorded base quaternion at first consumption. No posthoc stroke labels assigned.
- Six practical world strike clusters (3per side), x=.58,z~1.051297; FH y[-.33,-.26] span7cm, BH[.05,.15] span10cm.24exact float tuples reflect <=1.001micrometre z jitter, not broadtarget diversity. Actor/body relative coordinates vary withlivepose; fullbody lateral targets negative50FH/positive50BH.
- Intended return target fixedbothsides P[2.055,-.7625,0] ->W[2.555,0,.76], singlepoint. Actual99postrackettablepositions distinct (49FH/50BH), separately from100incomingbounces and100floor terminals. Original0referencelegal/qualified unchanged. Arithmetic inversion of recordedStage3 command gives uniquelynominal .5s returntime100/100 (maxsurfacegoal residual5.46e-7m); no alternative-duration evidence.
- SessionHOME side reference P y=-.764826203709406 is constant100rows; unique rawbasepose pairs it withP x=-.525439454232806 at20.42s, W HOME[-.025439454232806,-.002326203709406]. Dedicatedschema2 HOMEcolumns NaN are exportscope, notmissingreference. Relative split-.25,hysteresis0,W boundary-.252326203709406. Fixedxhit=.08P,bhdelta0,followrobotfalse,yfilteroff; targettranslationexact,no observedclamp/replacement. Narrowreference lanes explainpointrepetition; balancedrequests do not certifybalancedexecutedmotorstyles.
- Saved full100-row report, interactiveHTMLscatter/table, JSON/CSV, arithmeticinference and source-artifacthash receipt outsideGit in20261106-completed-100-lane-audit. No sequence/results/policy/sourceimplementation changes.

### Eight checkpoint comparison preparation (user-authorized eight bounded runs)

- Located exact1000,5000,10000,15000,19000,23000,27000,29999 in same gru-formal-2b8c09938a. Stable copied all8 weights/configs outsideGit in20261106-eight-checkpoint-comparison; source/copy SHA256, ZIP CRC, iteration/schema/run metadata checked. No substitutions/duplicates; no valid matching external bundle cache found. Training checkout unchanged.
- Added generic actual-weight/ONNX initializer verifier, bounded reset-repeat check, uniform offline annotated renderer and sequential resumable batch. Uses existing recurrent export/evaluator/PPQ004/concurrent lifecycle/scoring and existing ROS planner install; no rebuild/install, no per-model regression suite. Each model exports/verifies/evaluates/renders/saves before next. Initial idle20s archived; fixed800x500/20fps/1x camera, checkpoint/serve/actual planner side, cumulative contact/qualified/legal counts, purple reference vsorange/cyanactual, fall/unresolved labels. Actual reference paths generated from recorded planner state and validated against logged commands.
- Hash-protected completion skips and saved-evaluation render resume prevent unnecessary reruns. Infrastructure failures stop/preserve attempts and remain separate from policy falls. Clean falls end rally without reset/rescue and are rendered with unlaunched counts. Local index HTML/JSON updates each result with requested metrics and exact first-serve timestamp.
-3focused overlay/failure/resume tests passed; offline20-frame renderer-only smoke and existing29500reset-repeat check passed.100existing recorded planner-reference paths reproduce their original commands, no policy/simulator rerun. Smoke is not one of8requestedvideos.
- Check-only validates all frozen model/config/source inputs. Actual batch attempt model1000/attempt-60fgmqil stops before export/eval on sandbox Podman chmod /run/user/1001/libpod read-only.0/8 requested videos exist; exact one-command desktop resume in docs/GRU_EIGHT_CHECKPOINT_VIDEOS.md and externalREADME. No training/hardware/dependency/X11-security changes.
