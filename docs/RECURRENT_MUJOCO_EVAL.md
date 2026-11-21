# StationaryRally GRU MuJoCo inspection

This opt-in path runs the **saved GRU actor**, with explicit recurrent memory,
using CPU ONNX Runtime. The existing feed-forward exporter, reference runner,
and feed-forward physical-ball evaluator are unchanged. Default mode is a
nominal-plant inspection with PPQ004 target/reference markers. The opt-in
`--physical-ball` mode below measures actual contacts and return landings; neither
mode is a hardware qualification.

## Export a frozen checkpoint without starting Isaac

Use the existing PyTorch/ONNX libraries inside `grasping`. Source this worktree's
launcher, but do not import the training package or start `SimulationApp`.
`CUDA_VISIBLE_DEVICES=''` affects only the export process, not active training.
Replace `SNAPSHOT` and `BUNDLE` below with paths outside Git. The snapshot must
contain `model_N.pt` and its original `params/{agent,env,task_resolved}.yaml`.

```bash
distrobox enter grasping
if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi
cd "$HOME/workspace/HOPE-stationary-gru-eval/hope_training/whole_body_tracking"
source setup_train_env.sh
CUDA_VISIBLE_DEVICES='' hope_isaac_py scripts/export_recurrent_mujoco.py \
  --checkpoint "$SNAPSHOT/model_N.pt" \
  --expected-sha256 "$CHECKPOINT_SHA256" --output-dir "$BUNDLE"
```

The exporter reconstructs the installed RSL-RL recurrent policy and strictly
loads its complete state dictionary on CPU. The graph uses its actual actor
normalizer, GRU, and MLP, plus the saved passive-head mask. Observation input is
`obs[1,110]`; actor memory is `h_in[1,1,110]` / `h_out[1,1,110]`; output is
`actions[1,31]` in the checkpoint's **articulation order**. Neither critic memory
nor training-batch actor memory initializes a fresh evaluation episode.

The frozen model_5300 has **identity** actor normalization: the saved policy
config disables it and the state dictionary has no normalizer buffers. The
legacy runner's `empirical_normalization: true` field alone does not establish
that actor normalization is active. If another compatible checkpoint has
learned actor normalization, its strictly loaded normalizer is embedded.

## Verify, smoke, and view on the host

The host MuJoCo venv needs no PyTorch. Use the already installed venv:

```bash
cd "$HOME/workspace/HOPE-stationary-gru-eval"
PY="$HOME/workspace/nohope/.venv/bin/python"
SCRIPT=hope_training/whole_body_tracking/scripts/mujoco_eval_recurrent.py
"$PY" "$SCRIPT" --bundle "$BUNDLE" --output-dir "$EVIDENCE/verification" --verify-only
"$PY" "$SCRIPT" --bundle "$BUNDLE" --output-dir "$EVIDENCE/headless" --seconds 8
DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 MUJOCO_GL=glfw \
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
"$PY" "$SCRIPT" --bundle "$BUNDLE" --output-dir "$EVIDENCE/visible" \
  --view --realtime --seconds 120 --wall-seconds 90
```

The desktop display must be accessible to the logged-in lab user. Software
rendering avoids adding rendering work to the training GPU. Green markers
represent forehand targets; blue markers represent backhand targets. The red
marker follows the racket, and the target arrow shows desired velocity. There
is no collision-based success claim. The renderer is closed and its thread
joined at the time bound or on Ctrl-C.

The target is now an enlarged sphere with a yellow crosshair and the 3-D label
`REFERENCE STRIKE TARGET`. A persistent overlay identifies the side and strike
clock, and states that no ball flight/contact/bounce is simulated. The bank
provides a strike point and desired racket velocity, **not a time-indexed ball
trajectory**. The marker stays at that supplied point; no flight is invented
from the velocity or countdown. Vendor off-court ball meshes remain hidden.

The 64-step verification sequence starts with arbitrary **nonzero** actor
memory and resets it to zero at step 32. Every action and hidden output is
compared to the independent installed PyTorch policy's stateful inference.
Float32 ONNX comparisons use `atol=2e-5, rtol=2e-4`, allowing fused-kernel
ordering and accumulated GRU rounding, while the PyTorch explicit-state/oracle
comparison uses `1e-6`. The frozen model measured action error `2.38e-6` and
hidden-state error `3.58e-7` across the sequence. Finite-state checks, complete
110-column observation fixtures, joint-name/actuator-force mapping, affine
action decoding, delay FIFO, and virtual-table collision/placement checks
must pass before visualization.

Host regression tests with an exported bundle:

```bash
HOPE_GRU_EVAL_BUNDLE="$BUNDLE" PYTHONPATH=a3_deploy/a3_deploy_example/reference \
"$PY" -m pytest -q \
  hope_training/whole_body_tracking/tests/test_recurrent_mujoco.py \
  hope_training/whole_body_tracking/tests/test_action_adapter_parity.py \
  hope_training/whole_body_tracking/tests/test_onnx_joint_order_gate.py
```

## Scope and differences from training

- Actor observations retain the saved hitter_pure layout, world-frame targets,
  joint order, defaults, raw previous-action feedback, and passive-head masking.
  The manifest records the original observation functions. Perfect-state base
  measurements replace training mocap latency, noise, dropout, and stress.
- The saved action offset, scales, safe limits, and nominal stiffness/damping,
  armature, friction, and effort limits come from checkpoint/configuration,
  never model_21800. Nominal total damping is used **once** in the MuJoCo affine
  implicit drive. MuJoCo `implicitfast` at 5 ms with four physics steps per
  20 ms action approximates the training implicit drive; it is not PhysX.
  PhysX contact/friction and drive velocity-limit behavior are not identical.
- The plant uses nominal MJCF masses/COM and saved nominal PD gains. It does not
  replay training mass/COM/friction randomization, pushes, or the 6144-env cohort.
  Evaluation episodes alternate zero- and one-control-tick action delay to
  inspect both ends of the saved delay contract. FIFO seeding matches training.
- The pinned training bank and physics profile hashes are checked. Bank rows
  alternate forehand/backhand, with pre-strike time at the saved range midpoint
  and post-strike time at the saved 0.5 s value, plus an explicit 1 s inspection
  recovery interval. This replaces the adaptive sampler, curriculum, motion
  replay, and interruptible next-ball scheduling, not the actor architecture.
- Default evaluation starts PRE. `--initial-hold-seconds 4` instead inspects
  the task's long-HOLD start duration. Neither deterministic start reproduces
  the full training reset mixture. Episodes end at the saved 12 s timeout or
  an inspection fall guard (pelvis below 0.8 m or tilt above 60 degrees).
  GRU memory is zeroed **only** then, not on ball/target changes.
- The saved task's virtual table starts at x=0.5 m and is station-centered.
  Vendor Gate3 table/net collision is disabled and the visual geometry moved
  to that frame. Hidden vendor balls remain parked off-court with collisions
  disabled. Training reward/reference-body termination managers are not copied.

`results.json` and `trace.csv` record falls, episode resets, side, bank row,
reference strike timing, height/tilt, drift, contact-foot speed, racket speed,
target/velocity errors, command oscillation, and a **position/speed-only READY
proxy**. That proxy does not include the full training READY criteria. A
reference-strike sample is within one 20 ms control step of the target clock
crossing zero; it is not ball contact.

## Select another compatible checkpoint

Keep active training untouched. Select a published `model_N.pt`, never a hidden
`.tmp-PID` file. The pinned trainer writes through a temporary file and publishes
with atomic `os.replace`, so a single open of a final filename cannot see a
partially saved file. Copy that file and its original `params/` into a **new**
directory outside Git, compute `sha256sum`, and use its hash in the export
command. Re-export into a new bundle directory, repeat `--verify-only` and the
short headless smoke, and only then view. The exporter rejects incompatible
task/policy contracts or a checkpoint hash mismatch. Do not transfer saved
6144-env hidden states into the single fresh episode.

## Physical-ball evaluation (model_5300)

Add `--physical-ball` to the recurrent evaluator. The frozen actor, normalization,
110-D observation builder, named-joint action decode, 20 ms control interval,
action-delay FIFO and episode-only GRU resets are retained. Physical mode uses
20 x 1 ms MuJoCo steps per action instead of 4 x 5 ms: the original step was too
coarse for stable thin-racket/table contacts. This changes the nominal plant's
integration, not the actor/control clock. No actor inputs or action interventions
were added. The table/net are ball-only collision surfaces, so they still cannot
shove the robot.

### Incoming trajectory and actor target conversion

The saved, SHA-checked PPQ004 training bank jointly supplies
`target_position_offset`, `target_velocity`, `incoming_velocity`, `incoming_spin`
and `intended_landing_xy`. Stationary rows command `station_xy = [0,0]`.
The strike point is the sampled bank position in the session/world frame;
`target_velocity` is the desired **racket** velocity, not ball velocity. The
actor's final seven observation columns remain strike position relative to the
base (103:106), desired racket velocity in world coordinates (106:109), and
countdown (109). Ball position/velocity/spin, contact truth, landing outcome and
predicted path never enter actor observations. This is an oracle bank-target
experiment, not a vision/planner estimation evaluation.

`recurrent_physical_ball.incoming_serve` ports the pure RK4 flight law and bounded
reverse-time construction from `tracking/mdp/{virtual_ball,physical_ball}.py` to
NumPy so the installed host venv needs no torch or Isaac imports. It starts at the
bank's contact position, incoming velocity and spin, integrates backward at 1 ms
for at most 0.6 s, and rejects a step below table surface + ball radius + 5 mm or
above 40 m/s. The ball is launched only at the resulting effective lead time;
forward flight thereafter belongs to MuJoCo. Fractional launch-tick correction
advances that initial state to the next 1 ms tick. It never moves the ball onto
the racket or applies an impulse at the strike clock. Roundtrip tests cover both
sides and truncated serves.

This reconstructs the **final post-bounce incoming segment only**. The bank does
not identify pre-bounce states; the PPQ004 profile explicitly says so. It does
not invent a prior opponent serve or an incoming table bounce. Incoming-flight
prediction is drawn dashed grey. The simulated incoming path is solid orange;
after actual racket contact it is cyan. A yellow cross is the strike reference,
and the FH/BH label reports incoming/hit/miss. The broad camera contains the robot,
net, and both table halves. The path and camera also work in offscreen video.

### Physics and truth criteria

The saved `configs/ball_physics_ppq004_training.yaml` hash is checked before use:
ball mass 3.4 g, radius 20 mm, shell inertia coefficient 2/3, gravity 9.81 m/s²,
quadratic drag `k_d=0.12453915600992894` and Magnus
`k_m=0.00399279130564425`. The runtime uses the training virtual model's linear
Magnus law, not the separate saturating fit noted in the YAML. Spin is converted
from free-joint local coordinates into world coordinates for each aero force.
There is no extra spin-decay torque in free flight.

The existing vendor dynamic ball, table, net, floor and
`right_racket_collision` mesh are reused. The table occupies x=[0.5,3.24],
y=[-0.7625,0.7625], top z=0.76; net x=1.87, top z=0.9125. Explicit ball pairs
are the only collision authority. Other robot links and the vendor palm-support
pair are excluded. The scene's racket face is approximately 162 mm across,
whereas the Isaac instrument's diagnostic disc uses radius 75 mm. The saved
racket-site mount is still checked to 2 micrometres; no contact geometry is
repositioned to improve hit rate.

The existing `mujoco_pingpong_scene._dampratio_from_restitution` maps restitution
to damping ratio. Direct-format `solref` uses normalized stiffness 100000 and
damping `2*zeta*sqrt(100000)`, avoiding the excessive stiffness generated by
positive-format time constants at high restitution. The paddle's normal
coefficient is evaluated from its documented `0.759*exp(-0.0441*|u_n|)` at the
actual relative contact speed. Table restitution is 0.9373319241. Coulomb friction
uses PPQ004 safety caps (table 0.2185209065, paddle 0.5). Net/floor use the existing
no-spin profile's restitution 0.1/0.4 and friction 0.5/0.8 because the pinned
training profile has no net/floor material block.

These MuJoCo soft Coulomb contacts **approximate** the fitted normal restitution;
they do not reproduce the fitted angle/spin-dependent tangential impulse. The
Isaac physical instrument's code-driven velocity rewrite and blade-plane snap
are deliberately not used. The active formal model_5300 training configuration
has `physical_ball=false`, so actual contact is a new evaluation demand, not
training physics parity or a hardware qualification.

A hit requires positive MuJoCo normal contact force on the racket mesh. Contact
onset events include pre/post velocity, force, penetration, position and time.
The return must cross the net in +x with ball-radius clearance, have no net
contact, and its first descending table-height crossing/first table contact must
be on the opponent half **with a real table contact**. Exactly one racket contact
is required for a legal return. A plane crossing alone or floor bounce never
qualifies. This is a single-return metric, not a full competition referee.
Landings remain in MuJoCo world/session coordinates. Terminal floor contacts
end the flight and the ball parks off-court; new serves and episode resets are
explicitly logged. Unresolved shots interrupted by episode/evaluation end are
reported separately instead of silently counted as successes.

### Reopen and evidence

The frozen evaluation's command is saved at:

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261103-physical-ball/reopen.sh"
```

It opens the live viewer on desktop `:1`, seed 0, model_5300, with a fresh evidence
output directory. This sandbox cannot open X11 (`X11: Failed to open display :1`),
and a direct Unix-socket connection to `/tmp/.X11-unix/X1` returns
`PermissionError(1, Operation not permitted)` despite readable Xauthority.
The visible deliverable is an offscreen MP4, not a claim that the live desktop
viewer ran. CPU software rendering uses the already installed EGL and ffmpeg:

```bash
cd "$HOME/workspace/HOPE-stationary-gru-eval"
MUJOCO_GL=egl LIBGL_ALWAYS_SOFTWARE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
"$HOME/workspace/nohope/.venv/bin/python" \
  hope_training/whole_body_tracking/scripts/mujoco_eval_recurrent.py \
  --physical-ball --seed 0 --seconds 60 \
  --bundle "$HOME/workspace/hope-stationary-gru-evaluations/20261103-recurrent-implementation/bundle" \
  --output-dir /tmp/hope-physical-evaluation \
  --video /tmp/hope-physical-evaluation.mp4
```

`physical_config.json` saves the seed, complete frozen manifest, actual integration
step, physics profile, contact settings, source hashes, and exact invocation.
`ball_events.json`, `physical_shots.json`, `trace.csv` and `results.json` retain
contacts, misses, landings, legal returns, falls and reset reasons. The original
manifest's historical virtual-only limitation is superseded by the explicit
physical-mode limitations in `results.json`; the frozen bundle itself is unchanged.

Run the new collision/serve tests alongside the earlier recurrent/action tests:

```bash
HOPE_GRU_EVAL_BUNDLE="$HOME/workspace/hope-stationary-gru-evaluations/20261103-recurrent-implementation/bundle" \
"$HOME/workspace/nohope/.venv/bin/python" -m pytest -q \
  hope_training/whole_body_tracking/tests/test_recurrent_physical_ball.py \
  hope_training/whole_body_tracking/tests/test_recurrent_mujoco.py \
  hope_training/whole_body_tracking/tests/test_action_adapter_parity.py
```

The prior differences from training still apply: nominal rather than randomized
plant, perfect rather than noisy/delayed mocap, simplified alternating bank-row
scheduler and deterministic reset, omitted reward/termination managers, and
MuJoCo rather than PhysX. No formal training checkout, process, dependencies,
checkpoint, hardware connection or training command is modified by this evaluator.
