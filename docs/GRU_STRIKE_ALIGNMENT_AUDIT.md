# model29500 strike alignment diagnosis

The 100-serve run is held. This diagnoses the actual-planner preflight `attempt-z8SGbdOv/preflight`, using the same frozen checkpoint and the five scheduled serves (IDs 1, 2, 7, 8, 9). It is not Gate3 certification. No training checkout, checkpoint, normalization, joint/action mapping, physics, target location, serve, or deadline was changed.

## Findings

The dominant demonstrated mismatch is **planner future-bounce prediction versus the host soft-contact ball dynamics**, not a duplicate table-height transform. Both clean serves cross x=.58 approximately **173.830 ms before** the planner deadline, at z=1.277201 rather than its predicted z=1.051297 (22.590 cm higher). At the planned deadline the ball has travelled **33.769 cm past** the target in x. The policy tracks the commanded point considerably more closely than this. Removing a constant timing offset would not correct the height mismatch and is not an allowed fix.

A separate demonstrated adapter error capped the actor countdown at .65 s. Frozen `env.yaml` exposes `time_to_strike` with `clip: null`, `scale: null`; saved preparation sampling is .35–.95 s. The first actual planner observation should be .675060091 s, not .65 s. This affects two control ticks per serve (25.060 ms and 5.060 ms errors), ten ticks in this preflight. The cap is removed without changing the absolute deadline, target or physics. Its impact on live outcomes has not yet been measured; it does not account for the 173.830 ms physical crossing error.

The baseline also ended with a bridge `BrokenPipeError` after normal evaluator completion: the host sends a one-way close and closes, while the bridge tried to acknowledge it. The unused acknowledgement is removed. This teardown error did not cause the misses.

## Coordinate and timing contract

All measurements below use **host MuJoCo world W**, metres and seconds: +x from robot toward the opponent, +y toward robot left, +z up. Floor z=0, table surface z=.76, near edge x=.5, net x=1.87. Planner table frame P has the same axis directions and no rotation: `p_W = p_P + [.5,.7625,.76]`. Velocities and angular orientations are not translated. Spin is rad/s.

The schema-2 publisher has already added `policy_z_offset=.76` to command z. The host therefore adds **only [.5,.7625,0]** to the command position. Sensors subtract the full table offset; the planner robot-pose callback restores the .76 height for its actor convention. `marker_to_base_xyz` defaults to [0,0,0]. No root/HOME translation is subsequently added to the target. The frozen root XY [-.018148432,.003765902] is captured as the host station anchor once; only actor HOME-delta fields use it. The actual current base position is subtracted once in actor target fields. No yaw rotation, axis inversion, centimetre scaling or second table-height offset belongs in this 110-D world-relative contract.

Planner exposure stamps, producer stamps and absolute strike deadline are Unix seconds. `deadline_sim = deadline_unix - unix_anchor`, then `actor_tts = deadline_sim - observation_sim_t`. Producer TTS is a consistency check, not a second offset to add. The control loop samples the accepted command at the next 20 ms boundary. The original .65 cap is the only detected countdown modification; there is no fixed .174 s adapter latency correction. ROS direct-pose mode and exact simulator base state still differ from training's delayed/noisy target/mocap views.

## Representative clean serves

The recorded actions were reconstructed with the identical plant and launch/release events at 1 ms. **All 679 saved qpos frames match exactly (maximum error 0)**. No actor inference or planner substitution occurred in this reconstruction. At each exact deadline, qpos is interpolated across the enclosing 1 ms interval and quaternions normalized; the bracket is saved. Closest distance is a global 1 ms sampled minimum until the ball's natural terminal, not an analytically continuous minimum. It uses `mj_geomDistance` between the actual ball sphere and finite convex racket collision mesh.

| Quantity | Forehand shot 1 | Backhand shot 2 |
| --- | --- | --- |
| Planned point W | [0.580000, -0.260000, 1.051297] | [0.580000, 0.050000, 1.051297] |
| Absolute strike Unix s | 1793955062.178895950 | 1793955065.678895950 |
| Absolute strike UTC | 2026-11-06T08:51:02.178896+00:00 | 2026-11-06T08:51:05.678896+00:00 |
| Strike simulator s | 21.115060091 | 24.615060091 |
| Actual ball centre W at strike | [0.242310, -0.260000, 1.100734] | [0.242310, 0.050000, 1.100734] |
| Actual racket site W at strike | [0.566350, -0.325407, 1.072304] | [0.564637, 0.017969, 1.056396] |
| Collision mesh origin W at strike | [0.561163, -0.327541, 1.068572] | [0.570035, 0.018526, 1.060388] |
| Nearest racket surface point W at strike | [0.508950, -0.290975, 1.116490] | [0.517506, 0.014769, 1.117613] |
| Collision face normal W at strike | [0.782303, 0.126283, 0.609962] | [-0.793034, 0.116015, -0.598028] |
| Racket site target error | 0.070041 m | 0.035889 m |
| Unsigned normal / desired-velocity angle | 4.163 degrees | 6.235 degrees |
| Closest sphere-surface / mesh gap | 0.137797 m | 0.144367 m |
| Closest centre / mesh distance | 0.157797 m | 0.164367 m |
| Closest timestamp simulator s | 21.063000 | 24.560000 |
| Closest timestamp Unix s | 1793955062.126835823 | 1793955065.623835802 |

The full 3×3 rotation matrices for both collision geom and wrist-mounted racket site are in `baseline/alignment.json`. Matrices map local axes into W. The compiled mesh thin axis is local x; it agrees with the training/site local y normal (dot product 1). The mesh is two-sided; the backhand's raw normal has the opposite sign. Full thickness is about .014 m, lateral half extents about .076–.077 m. The site is about 6.7 mm from the mesh origin; neither a point site nor an infinite plane is used for collision-distance measurements. The ball radius is .02 m. Zero raw racket contacts and strictly positive mesh separation agree, so this is not a missed contact-report callback.

## One command end to end (forehand shot 1)

Original schema-2 message (unchanged in the evidence):

```json
[2.0, 1.0, 1.0, 0.08, -1.0224999999993447, 1.0512966468602754, 1.7330208438306294, 0.1778352642901003, 1.192808174781422, 0.6945912837982178, 1793955062.178896, 0.0, 1793955061.0, 484304547.0, 1.0, 1.0, 1.0, 37.0, 0.17999982833862305]
```

Fields: [schema,valid,side, position(3),velocity(3),producer_TTS,absolute_deadline,frame_code,producer_sec,producer_nsec,command_seq,flight_id,revision_id,sample_count,span]. Frame code is 0. The position [.08,-1.0225,1.051296647] already includes world-height offset; XY translation yields [.58,-.26,1.051296647]. Desired racket velocity is [1.733020844,.177835264,1.192808175] m/s, not incoming-ball velocity.

Exposure-source simulation time is 20.420000076 s. The estimated ball W position [1.736973049,-.26,1.277920247] differs from actual [1.736972934,-.26,1.277920143] by only about 0.15 micrometres at the nearest 1 ms source sample. The sensor transform and pre-bounce fit therefore agree; divergence develops in future contact prediction. The planner commits once before the actual table bounce, so its future-bounce error is not revised from post-bounce measurements in this protocol.

The command is received/applied by the adapter at simulation 20.425 s. It passes producer/flight-identity, consistency and minimum-lead checks. Deadline is simulator 21.115060091 s. The first actor consumption is at 20.44 s; true countdown is .675060091 s, but the baseline supplied .65 s. At 20.46 s it supplied .65 instead of .655060091; later ticks are unmodified. The correction supplies the true signed countdown, including negative follow-through time.

Exact first-consumption fields before normalization (the complete 110-vector is saved in JSON):

- forehand at t=20.440000: current base W=[-0.025424, -0.002331, 1.034368]; HOME delta obs[101:103]=[0.007276, 0.006097]; target-minus-base obs[103:106]=[0.605424, -0.257669, 0.016928]; target velocity obs[106:109]=[1.733021, 0.177835, 1.192808]; obs[109]=0.649999976.
- backhand at t=23.940000: current base W=[-0.045900, 0.060811, 0.990129]; HOME delta obs[101:103]=[0.027752, -0.057045]; target-minus-base obs[103:106]=[0.625900, -0.010811, 0.061167]; target velocity obs[106:109]=[1.716691, -0.033869, 1.182406]; obs[109]=0.649999976.

The frozen recurrent bundle applies its existing saved observation normalization. Its hash and actor arrays are unchanged. The actor has no side/ball-position input; side is implicit in the target. Neither sensor ball state nor a new normal channel is injected into the actor.

## Physics and reachability attribution

The planner uses the documented PPQ004 capped-slip table impulse (`table_contact` in `ball_dynamics.hpp`): slip gain .41642145, friction cap .21852091, restitution .93733192, shell inertia 2/3. The host loads those restitution/friction values into soft contact parameters, but a MuJoCo friction constraint is not that fitted impulse law. Actual shot1 pre-contact velocity near t20.602 is [-2.3571,0,-3.4953]; after separation at t20.620 it is [-2.1966,0,3.1850]. In contrast the planner predicts velocity at its strike plane [-1.2747,0,-2.0719], whereas actual plane-crossing velocity is about [-1.9903,0,-.167]. Free-flight drag/Magnus and gravity settings are unchanged; prediction error is concentrated after the table bounce. No fitted impulse overwrite or ball snap was added to obtain agreement.

Both intended positions and desired velocities lie within the frozen per-side rectangular training ranges, and the .675 s lead lies inside the .35–.95 s sampled preparation range. Rectangular membership is not a dynamic reachability certificate. The policy's baseline target tracking is imperfect (FH 7.0 cm, BH 3.6 cm), and long idle/recurrent history plus exact versus noisy/delayed observations remain transfer differences. The frozen config has `causal_idle: false`; the newer source's optional 1.2 s idle-clock helper must **not** be enabled just to improve this older checkpoint. No startup target or idle clock was changed. The evidence does not establish that the policy could hit a correctly predicted ball, but establishes that the observed actual ball does not arrive at the announced tuple.

## Changes, verification and repeat status

Only the proven countdown cap and orderly bridge-close race were fixed. Added measurement/visualization scripts do not alter policy/physics. Before-source files, original input hashes, original preflight artifacts and the original video remain preserved. Corrected source and hash receipts are separate in `20261106-alignment-model29500/corrected-source`.

33 focused tests passed with the frozen model29500 bundle; the three new tests verify unclipped signed countdown, exactly-once height/deadline conversion and bounded timestamp interpolation. Bundle identity check passes. The new annotated video decodes successfully. It is a verified actual-state reconstruction of the baseline, not an after-fix live run.

The attempted actual-planner repeat `attempt-2DdNYIJp` stopped before container entry with `chmod /run/user/1001/libpod: read-only file system`. Therefore **no live after-fix preflight or before/after improvement claim is available**. A normal desktop terminal must perform the remaining live repeat. It uses the same frozen bundle, same five serves/order, scheduling, physics, carried GRU/robot state and actual ROS planner, and generates per-ball alignment and annotated video automatically. It can never enter the scored phase:

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261105-221919-fixed-home-gru-diagnostic/run-reference-diagnostic.sh" --alignment-preflight
```

Open the baseline alignment video:

```bash
xdg-open "$HOME/workspace/hope-stationary-gru-evaluations/20261106-alignment-model29500/baseline/alignment.mp4"
```

`HOLD-100-SERVE-ALIGNMENT.txt` prevents the original full launcher from proceeding into scored serves. No threshold was relaxed. Next engineering decision is to reconcile the intended planner/plant contact-model contract explicitly; neither a tuned timing offset nor an artificial successful collision is justified.

## Frozen identity and training status

Run `gru-formal-2b8c09938a`; checkpoint model29500; source commit `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.

- Checkpoint SHA256: `b53a031b6d2dacceeebe7301412d3d2c5fc456f445c6aebb357e4159871683d0`.
- ONNX SHA256: `f50b0de8c79a83c3e48868d47bc8b922094353aebdea01540ec70b662541a58b`.
- Agent SHA256: `770dbc5f9ccba22c0957b0afc14178da8a3e45eafd3a66f9ee1596e30215a015`.
- Environment SHA256: `a9d28f88d1f98130f10b7de1979e9941330d1a1a1ab58041c77f5498ba2ccc90`.
- Resolved task SHA256: `5fcbf22da9b99edb9919f28faca3f887c6901e9deb106a8683f10f5e54006ada`.

Read-only training evidence now shows final iteration29999, model_29999.pt and the W&B final synced-summary log at23:26:18 PST on Nov5. It is no longer correct to say it is still progressing; no signal, stop/restart or checkout change was made by this investigation. The training checkout remains clean. The requested model29500 stays frozen despite the later checkpoint.

## Subsequent trajectory-impact diagnosis

The corrected-countdown desktop preflight attempt-vT5mOho1 completed with zero countdown-cap errors and the same five misses. [The impact diagnosis](GRU_TABLE_IMPACT_DIAGNOSIS.md) traces the actual C++ prediction at matched timestamps, proves divergence at the table impact, and implements an explicit archived PPQ004 table-law mode without position changes. It documents the isolated model-regression result and the still-blocked live same-five repeat. The100 hold remains in place. Preflight replay is now explicitly phase-selected, without scored-video fallback.
