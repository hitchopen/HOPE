# Systematic incoming-trajectory mismatch: model29500

The 100-serve diagnostic remains on hold. This investigation uses the frozen model29500 from `gru-formal-2b8c09938a` and corrected-countdown actual-planner preflight `attempt-vT5mOho1/preflight`, IDs1,2,7,8,9. All five missed. The countdown cap is absent (zero affected ticks). No actor weights, normalization, actions, strike targets, time offsets, serve rows or training files changed.

## Root cause

The host's **soft MuJoCo table constraint is not the planner's PPQ004 table-impact map**. The profile's `a_t=.41642144985036184` was never used by the host impact response. Copying normal restitution and friction into MuJoCo `solref`/friction parameters did not implement that fitted slip/spin law. The host produces much less tangential impulse and much less spin. That changes horizontal arrival time and the subsequent Magnus acceleration.

The actual C++ `TrajectoryPredictor::predict_with_spin` was instrumented with state-print calls only and compiled using existing host c++ plus an already-present Eigen header tree. No package or dependency was installed. Its original strike-plane result reproduces **all five recorded planner outputs exactly**, including deadline and position. A separate continuation of the same model is used only to inspect simultaneous states after that plane; it is not an alternative planner command.

Recorded-action reconstruction again matches all679 saved qpos frames exactly. Actual geometry, velocity and world spin are sampled at1ms. Before the first bounce the maximum position discrepancy is **0.0545mm**, and velocity discrepancy is <=.00455m/s. Small estimator/discretization differences exist immediately, but the large error begins at the table bounce: first >1mm position discrepancy is t20.602s for serve1, as the planner instantaneously bounces while the host begins a10ms soft contact. The source timestamp is not displaced by174ms.

## Matched timestamps and geometry

All vectors below use host MuJoCo world W: +x toward the far table, +y robot left, +z up; metres, m/s and rad/s. Planner table frame P is W minus [.5,.7625,.76]; same axes, no rotation. Incoming sensor stamps are actual simulator exposure times in Unix nanoseconds. C++ source time is1793956540.0419905 (simulation20.420000076). It is not the later command receipt/producer time. The first actor consumption is at20.44s, with an unclipped .67505985s countdown. The command has one absolute deadline conversion, no duplicate height or HOME offset.

Actual table top is W z=.76, centre W [1.87,0,.75], half-size [1.37,.7625,.01]; near/far edges .5/3.24. Ball radius .02, so centre contact is z=.78 in W or .02 in P. The active C++ spin-aware predictor uses z=radius, **not** its inactive legacy point-ball z=0 branch. Both implementations agree on this geometry. No net or racket contact precedes the incoming table bounce.

Both use gravity [0,0,-9.81], drag .12453915600992894 and Magnus .00399279130564425, acceleration `g - kd*|v|*v + km*(omega cross v)`. No fluid drag/viscosity is enabled in MuJoCo. Ball mass .0034kg and inertia `(2/3)*mass*radius^2` agree. Freejoint armature/damping/frictionloss are zero. C++ uses1ms constant-acceleration position/forward-Euler velocity stepping; host uses1ms implicitfast MuJoCo stepping. These numerical differences are small before impact and cannot explain34cm.

The five serves have true zero incoming spin. The planner's unavailable-spin path permits a zero prior (`future_contact_require_spin=false`); audit `spin_valid=0`, `future_contact_spin_used=0`, but the selected physical future-contact branch still uses the venue table law and generated post-bounce spin/Magnus. Its zero prior agrees with actual incoming spin, so this is not an unmodelled incoming-spin error.

## Detailed checkpoints, representative forehand shot1

| Event | Simulator time | Actual W position | Planner W position |
| --- | ---: | --- | --- |
| Exact estimator input source stamp |20.420000076|[1.736972738,-.26,1.277919994]|[1.736973022,-.26,1.277920226]|
|1ms before predicted bounce|20.600839542|[1.291489771,-.26,.783539155]|[1.291459102,-.26,.783493915]|
|Predicted impact instant (pre/post position continuous)|20.601839542|[1.289132465,-.26,.780045167]|[1.289101931,-.26,.780000000]|
|Before host's first contact solve|20.602000000|[1.288754249,-.26,.779484324]|[1.288881061,-.26,.780526335]|
|After first host contact solve|20.603000000|[1.286410572,-.26,.776089713]|[1.287505424,-.26,.783797728]|
|Host separated from table|20.613000000|[1.263833194,-.26,.784999863]|[1.273729903,-.26,.815871806]|
|Actual incoming x=.58 crossing|20.941230231|[.580000000,-.26,1.277200708]|[.813293562,-.26,1.263299147]|
|Planned x=.58 crossing / absolute strike deadline|21.115059853|[.242310365,-.26,1.100734580]|[.580000113,-.26,1.051296899]|

The planned deadline is Unix1793956540.7370503. Ball position discrepancy there is approximately [-.33768975,0,+.04943768]m. The actual x=.58 crossing is173.830ms early and22.590cm above the announced point. The backhand differs only in y and launch time; all five matched-timestamp state rows, velocities, spin and errors are saved in CSV/JSON.

| Impact stage | Actual velocity | Planner velocity | Actual / planner spin y |
| --- | --- | --- | --- |
|Immediately before predicted contact|[-2.35731,0,-3.49399]|[-2.35667,0,-3.49808]|~0 /0|
|Immediately after predicted contact (same instant)|host still approaching|[-1.37530,0,+3.27886]|~0 /-73.60266|
|First host contact step completed (20.603)|[-2.34368,0,-3.39461]|[-1.37571,0,+3.26532]|-.90273 /-73.60266|
|Host separation (20.613)|[-2.20329,0,+3.26528]|[-1.37916,0,+3.14918]|-8.95279 /-73.60266|

An instantaneous map and a finite-duration soft constraint have no common single post-impact timestamp; both the rising edge and complete separation are shown to avoid treating the host's still-downward first contact sample as a completed bounce.

## Actual impact laws and supported correction

The archived profile and active C++ `ball_dynamics.hpp` specify, for shell inertia coefficient kappa=2/3 and radius R=.02:

```text
slip = [vx - R*omega_y, vy + R*omega_x, 0]
J_over_mass = min(a_t*norm(slip), mu*(1+e)*abs(vz))
delta_v = -J_over_mass * slip/norm(slip)     # zero for zero slip
v_xy += delta_v_xy
v_z = -e*v_z
omega += [delta_v_y, -delta_v_x, 0] / (kappa*R)
```

Parameters are `a_t=.41642144985036184`, `mu=.218520906499864`, `e=.9373319241133231`, b_t=0. Racket parameters are not part of this correction. The original host instead used pair `solref=[-100000,-13.026023532949354]`, friction [.2185209065,.2185209065,.005,.0001,.0001], condim3, default solimp [.9,.95,.001,.5,2], pyramidal cone, noslip_iterations3. This provides no `a_t` map. Its normal rebound is close to the intended restitution, but tangential impulse/spin are far too small.

Added **explicit evaluation mode `--table-impact-model ppq004`**. The default `soft` mode remains reproducible for old artifacts. The new mode uses precisely the archived table law when MuJoCo reports a real finite-geometry table contact with positive normal force, an approaching ball and vertical normal. It replaces the first-step engine velocity response using the saved pre-step velocity/spin. It changes ball velocity/spin only, never qpos, target, strike time, robot/GRU state or racket collision dynamics. Each ball's table pair is suppressed only until its centre exits the same1ms-quantized overlap, preventing a duplicate soft impulse; a50ms failure-to-separate guard fails closed. Events record pre/post velocity/spin, replaced engine velocity, normal impulse in N·s, contact force, penetration and `position_modified=false`.

This is a **documented table-physics revision correcting an unimplemented profile law**, not policy improvement or a hidden timing calibration. It is not formal Gate3 physics equivalence: native software also snaps positions and applies a fitted racket impulse, whereas the host never snaps and retains real MuJoCo racket collisions. Training itself used virtual at-strike targets, not a physically trained incoming table bounce; this aligns planner and documented table-law semantics without claiming full Isaac equivalence. The remaining1ms integration/contact discretization is explicit.

## Before/after evidence and verification limits

- All five original C++ predictions reproduced exactly; actual reconstruction679/679 qpos frames exact; countdown error absent.
- 43 focused tests passed with the frozen29500 bundle, including seven map/contact tests and three explicit replay tests. The final impact guard was rechecked with all seven impact tests passing.
- In the **isolated ball/table regression**, the unchanged old soft law reproduces each [-.33769,0,+.04944]m error. Switching only the impact law gives about [-.001181,0,-.005329]m at the same deadline, norm5.46mm, for every serve. The unit test disables the racket pair to isolate table dynamics; it runs no actor or live ROS session and scores no return. The actual preflight retains the racket unchanged. The residual was not tuned away.
- Tests verify the archived impulse against the actual C++ numeric example, spin/slip/friction bounds and energy, real positive-force contacts before applying it, zero position/robot-velocity changes inside the handler, independent concurrent balls, one map application per bounce, pair restoration and no impulse without contact. They do not guarantee future policy contacts or legal returns.
- The attempted live same-five repeat `attempt-FlGiKewV` stopped before container entry: `chmod /run/user/1001/libpod: read-only file system`. **No after-map actual-planner preflight has run in the sandbox.** The100 hold stays present. No result is silently substituted.

## Desktop commands and explicit preflight replay

Run the same five serves with the supported table-map correction, actual planner and frozen policy:

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261105-221919-fixed-home-gru-diagnostic/run-reference-diagnostic.sh" --impact-preflight
```

This mode runs only preflight, never scored serves. It prints model/table identities, validates source/weight hashes and parity, runs focused checks, then records the real preflight. Afterwards it reconstructs surface measurements, traces the actual C++ prediction, saves simultaneous trajectory comparisons and renders actual-state videos. If an actual racket contact occurs, post-contact prediction comparisons are explicitly marked as such; an incoming crossing prevented by that contact is not fabricated.

Explicitly replay the existing corrected-countdown baseline preflight:

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261105-221919-fixed-home-gru-diagnostic/run-reference-diagnostic.sh" --replay-preflight attempt-vT5mOho1
```

After a new recording, `--replay-preflight` without an attempt uses `latest-recorded-preflight.txt`, which is updated only for an existing recorded preflight. Selection verifies phase, iteration, checkpoint SHA, ONNX identity and video receipt. Missing video, wrong phase/policy, path traversal or absent pointer fails. There is no scored-video fallback. Legacy `--replay` remains explicitly scored-only. `--alignment-preflight` retains the old soft-table mode for reproducing the baseline.

## Evidence and identity

Artifacts are outside Git in `/home/tianhe1/workspace/hope-stationary-gru-evaluations/20261106-trajectory-impact-model29500`:

- `before-alignment/alignment.json`: reconstruction validation, raw commands, actor fields and surface measurements.
- `before-comparison-complete/checkpoints.csv`, `comparison.json`, per-shot1ms streams and original/continued actual C++ traces.
- `trajectory-comparison.svg`: actual preflight, C++ prediction and clearly labelled isolated after-map regression.
- `ball-only-map-check.json`, `after-ball-only-comparison.json`: unit-only before/after evidence, no planner/policy success claim.
- `before-source/`, `corrected-source/`, `corrected-source-hashes.json`, `cpp-trace-receipt.json`, instrumentation sources and exact commands.
- `focused-tests.log`, `impact-final-tests.log`, `status.json`; failed live-repeat log remains in its original attempt directory.

Checkpoint SHA256 `b53a031b6d2dacceeebe7301412d3d2c5fc456f445c6aebb357e4159871683d0`; ONNX SHA256 `f50b0de8c79a83c3e48868d47bc8b922094353aebdea01540ec70b662541a58b`; run gru-formal-2b8c09938a, iteration29500, source33c609022efbb4e85fe6a58a5a8947b0fbd46c76. Agent/env/task config hashes are unchanged in the frozen session's `snapshot.json`. Training checkout remains clean and untouched. No dependencies, hardware or X11 security settings changed.

### Exact-attempt replay and law hashes

The preflight-only launcher records `latest-requested-preflight.txt` before container checks. Passing that attempt basename explicitly to `--replay-preflight` replays precisely that attempt; missing video fails instead of opening an earlier recording. The archived PPQ004 capped-slip shell-inertia law is from `configs/ball_physics_ppq004_training.yaml`, SHA256 `df55e82102e0d5624732ea7ea03f6d98053f45d7f036367e3df3551a1c88a6f9`. Its C++ implementation `hope_ws/src/hope_planner_cpp/include/hope_planner_cpp/ball_dynamics.hpp` has SHA256 `e7c2c0a9a8ba70b474d3d2a26e0e5c5a593f95b6a5771f9a3a37f8586f6a86be`.
