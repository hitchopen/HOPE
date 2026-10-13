# ONNX → deploy → robot: the full runbook

Everything between an exported `policy.onnx` and a robot returning a ball. Run the sections in
order; each one is cheap compared to the next, which is the whole reason for the order.

Fill in the variables in §1 once and paste them into every shell. Nothing below contains an
address, a checkpoint name, or a threshold from someone else's setup — those are yours to supply.

---

## 0. The one control path

```
   your training run
        │ .pt
        ▼  §2 export
   policy.onnx  (metadata baked in: obs contract + dim, action dim, rate, joint order)
        │
        ▼  §4 sync + parity      §5 build
   deploy package  ──────────────────────────────────────┐
        │  same binary everywhere                        │
        ├──────────────► §6/§7 MuJoCo sim                │
        └──────────────► §8/§9/§10 robot                 │
                                                          │
   input side (plain ROS 2, no control code):             │
     mocap / fake ball → planner → racket target ─────────┘
```

The runner is the only thing that talks to motors. The ROS 2 side is input only: it produces a
racket target and (if your policy needs it) a base pose. Everything else is the same binary
against a different backend.

---

## 1. Set up your variables

```bash
# --- training side ---
RUN=                    # run directory containing model_*.pt and exported/
CKPT="$RUN/model_XXXX.pt"
ONNX="$RUN/exported/policy.onnx"
MOTIONS=()              # reference clips your policy trained on, if any

# --- deploy side ---
ASSET=                  # short name for the deployed model, e.g. my_policy_v1
CFG=src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml
PYBIN=                  # python with onnxruntime + numpy

# --- hardware ---
ONBOARD=user@onboard-ip     # runs the planner; also the jump host
ROBOT=user@robot-ip         # runs the runner
DEPLOY=/path/on/robot       # where the package lands
DOMAIN=                     # ROS domain, identical on every machine
```

### What this runbook assumes about your policy

| Assumption | If yours differs |
|---|---|
| Feed-forward: obs vector in, action vector out, fixed rate | Recurrent policies need §4 parity replayed as a *sequence* with identical hidden-state reset |
| Actions are joint targets realized by PD | Torque/Cartesian output changes the action adapter only |
| Needs world-frame base position | Fully proprioceptive → skip every `--oracle-pelvis` / base-pose step and the mocap base bridge |
| An upstream planner supplies the target | End-to-end policy → §7 collapses into the policy; keep the closed-loop run |
| The robot steps to reach the ball | Stationary base → ignore station/drift checks, keep the heading step in §10 |
| Carries a reference clock (motion clips) | No clock → ignore `--motion-files` and clip metadata everywhere |
| Two swing sides selected per ball | Single behaviour → drop side checks; more modes → repeat them per mode |

---

## 2. Export the ONNX

Run the preflight first; it checks provenance without launching the simulator or touching the
existing file.

```bash
cd hope_training/whole_body_tracking
PP_EXPORT_PREFLIGHT_ONLY=1 TASK=<your task> \
  bash scripts/export_onnx_hitter_pure.sh "$RUN" "$(basename "$CKPT")"

# then the real export
TASK=<your task> bash scripts/export_onnx_hitter_pure.sh "$RUN" "$(basename "$CKPT")"
```

**expect:** `$RUN/exported/policy.onnx` exists; the exporter exits nonzero on incomplete or
inconsistent metadata.

**The export must bake everything the runner needs to interpret the model** — observation contract
name and dimension, action dimension, control rate, joint order, plus whatever geometry and timing
constants your policy was trained against. A runner that loads a model it does not fully understand
produces plausible numbers and wrong behaviour, with no error anywhere. Both sides fail closed:
exporter refuses to write, runner refuses to load.

Two rules that are not optional:

- **The exporter reads the config saved with that run**, not your current source tree. Otherwise
  re-exporting an old checkpoint after a refactor silently yields a different model with the same
  name.
- **If your policy uses reference data, bake the exact data it trained on.** Verify against that
  run's own log, not a directory listing.

Pin the file by hash now. This is its identity for the rest of the runbook:

```bash
sha256sum "$ONNX" | tee /tmp/policy.sha256
```

Training pipelines leave more than one `.onnx` around — an auto-export next to the checkpoint, an
older manual one. They are not interchangeable and share an extension.

---

## 3. Policy-only check in MuJoCo (host, no simulator licence needed)

Evaluate **the ONNX**, not the checkpoint, under deploy conditions: realistic start pose, no
teleports, fall-only termination, deployment's PD arrangement.

```bash
cd hope_training/whole_body_tracking
"$PYBIN" scripts/mujoco_eval_onnx.py \
  --onnx "$ONNX" \
  --motion-files "${MOTIONS[@]}" \
  --noise-scales 0.0 0.05 \
  --pd-mode implicit \
  --deploy-faithful \
  --steps 6000
```

**expect:** zero falls across seeds, task completion on your own terms.

**fail →** if it falls here it will fall on the robot. Do not continue.

Flags that matter and why:

| Flag | Why |
|---|---|
| `--deploy-faithful` | Removes the training helpers: nominal-stand start, no teleports, fall-only termination |
| `--pd-mode implicit` | Match how your robot's controller actually closes the loop. A mismatch here is the classic "worked in sim, oscillates on the robot" |
| `--noise-scales` | A policy that only works on clean observations will not survive real sensors |
| `--steps` | Run long. Failures that matter appear after many strikes, not the first three |

Measure what your task needs, but these categories transfer: falls, task success, drift from the
standing position per action, stability *between* actions, and recovery after the action. Idle
instability is easy to miss in a short run and dominates real sessions.

Two habits: fix the protocol before comparing candidates and record it next to the numbers; and
never promote on numbers produced by a debugging escape hatch.

> Kinematic success ≠ physical success. If this scores whether the racket reached a pose, it has
> not shown a ball would clear the net and land in. Know which you measured.

---

## 4. Sync into the deploy tree + parity

```bash
cd a3_deploy/a3_deploy_example
PYBIN="$PYBIN" bash scripts/sync_pingpong_model.sh "$ONNX" "$ASSET" "$CFG"
```

This does three things: stages the model as a durable asset under
`assets/a3_runtime/models/$ASSET.onnx`, repoints `onnx.model_path` in `$CFG`, and runs the parity
gate.

**expect:** `PARITY: PASS`.

**Never hand-copy a model into `dist/`** — the next build wipes it, and a build output is not a
certifiable artifact. Stage into sources; let the build populate the package.

Prove the bytes survived the copy:

```bash
sha256sum "$ONNX" "assets/a3_runtime/models/$ASSET.onnx"   # must be identical
```

### The parity gate on its own

```bash
MODEL="assets/a3_runtime/models/$ASSET.onnx" PYBIN="$PYBIN" \
  bash scripts/pingpong_parity/run_parity.sh
```

Feeds byte-identical observations to the C++ runner front-end and to Python, and compares actions
element-wise. **This is the highest-value test in the pipeline and the one most often skipped.**
Between training and deployment the observation is rebuilt by different code, in a different
language, months apart — term order, signs, frames, joint order, units, normalization, staleness.
None of it raises an error; all of it produces a policy that runs and behaves wrong.

Agreement should be at float32 precision (`~1e-6` element-wise). Widening the tolerance to make it
pass converts your best diagnostic into decoration.

Reading a failure:

| Pattern | Cause |
|---|---|
| A few elements wrong, rest exact | Joint order, or a mis-indexed obs term |
| All wrong, similar magnitude | Normalization applied on one side only |
| Sign-flipped groups | Frame convention (handedness, axis direction) |
| Small constant offset | A term one tick stale on one side |
| Exact at rest, wrong in motion | A velocity, phase, or timing term |
| First frame exact, diverges over a rollout | `last_action` feedback wired differently |

Wire this into whatever runs on every change. It is fast and needs no robot.

---

## 5. Build the package

Build **after** sync — sync updates the sources, the build repopulates `dist/` from them. Reversed,
you ship the previous model with the current config, which looks entirely correct.

```bash
cd a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --runtime-cfg "$CFG"
```

**expect:** `dist/a3_deploy_x86_64/` with the runner binary, `models/`, `config/`.

Verify the package, not your source tree:

```bash
grep model_path dist/a3_deploy_x86_64/config/$(basename "$CFG")
sha256sum "assets/a3_runtime/models/$ASSET.onnx" "dist/a3_deploy_x86_64/models/$ASSET.onnx"
file dist/a3_deploy_x86_64/a3_deploy_onnx_ref_pingpong
```

**Rebuild after ANY change** to the runner, the model, or the config. `dist/` is never rebuilt
automatically. A stale binary silently ignores new flags and new metadata and presents as a policy
regression — when a hardware result disagrees with a sim result, suspect this before the policy.

If your runner reads compile-time constants for reachability or target boxes, confirm they still
match your model's trained envelope, and rebuild if you change them.

---

## 6. Policy-only in the official sim

Same binary, simulated robot, scripted targets. No planner yet.

```bash
# terminal A — the simulator
bash scripts/run_sim.sh

# terminal B — the runner, policy only
bash scripts/pp_freebase_watch.sh --single-swing        # add --oracle-pelvis if your
                                                        # policy needs real base feedback
```

**expect:** stands, holds, completes an action, recovers, holds again. No falls.

**fail →** back to §3. If §3 passed and this fails, the difference is the runner, not the policy —
re-check §4 parity.

---

## 7. Closed loop in sim

Now with a live target source: planner in, runner acting, consequences feeding the next decision.

```bash
# runner in planner mode, against the sim
taskset -c 4-7 ./run_a3_pingpong.sh --planner --policy-native --start passive --official-stand
```

Boot log **must** show the target source is the planner, the localization mode your policy needs,
both input subscribers alive, and the clip layout from ONNX metadata. Anything else means a stale
binary — go back to §5.

What to prove here:

- **Handoff works.** Targets arrive, are accepted or rejected with a stated reason, and the decision
  to act fires at a sensible moment.
- **Rejections are correct.** Deliberately feed impossible targets — too late, out of reach, stale,
  malformed. Every one ends in a held stand.
- **Sequences work.** Act, recover, act again. Designs that fail on hardware usually fail in the
  recovery, not the action.
- **Measure the lead time your policy needs.** The real number becomes the serving rule in §10.

> Simulated contact is not physical contact, and a sim without ball physics makes a "successful"
> strike a kinematic claim only. Write down which one you measured.

---

## 8. Cross-compile, ship, verify on the robot

Build for the robot's architecture (this may need Docker on the host rather than a container):

```bash
cd a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch rockchip --runtime-cfg "$CFG"

# verify BEFORE shipping
sha256sum "dist/a3_deploy_rockchip/models/$ASSET.onnx"     # == /tmp/policy.sha256
grep model_path "dist/a3_deploy_rockchip/config/$(basename "$CFG")"
file dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong   # must be the target arch
test -x dist/a3_deploy_rockchip/run_a3_pingpong.sh
```

Ship (the robot is typically reached through the onboard machine):

```bash
rsync -azP -e "ssh -J $ONBOARD" dist/a3_deploy_rockchip/ "$ROBOT:$DEPLOY/"
```

Verify **on the robot**, not on your laptop:

```bash
ssh -J "$ONBOARD" "$ROBOT"
cd "$DEPLOY"
sha256sum "models/$ASSET.onnx"                  # == /tmp/policy.sha256
grep model_path "config/$(basename "$CFG")"
file a3_deploy_onnx_ref_pingpong
```

**fail →** re-ship. Do not proceed on the theory that it is probably fine.

Re-ship after any change. Mocap-side changes do not require it — the runner only consumes what the
planner and the base bridge publish.

---

## 9. Shadow on the robot — real inputs, zero motion

The first time the artifact meets the robot it must not be able to move it.

```bash
ssh -J "$ONBOARD" "$ROBOT"
source /absolute/path/to/robot_env.sh
cd "$DEPLOY"
export A3_PINGPONG_RUNTIME_CFG="$PWD/config/$(basename "$CFG")"

SESSION=hope_$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "/tmp/hope/$SESSION/shadow"

taskset -c 4-7 ./run_a3_pingpong.sh \
  --planner --policy-native --dry-run --start shadow \
  --trace-csv "/tmp/hope/$SESSION/shadow/trace.csv" \
  --obs-csv   "/tmp/hope/$SESSION/shadow/obs.csv" \
  --session-id "$SESSION" \
  > >(tee -a "/tmp/hope/$SESSION/shadow/runner.log") 2>&1
```

Before feeding it anything, confirm the startup banner:

- model path and hash actually loaded — matching `/tmp/policy.sha256`
- observation contract and dimension
- localization source (mocap, not a placeholder) and command topic, with publishers present
- control rate

**fail →** banner mismatch means the artifact on the robot is not what you built. Stop, re-ship.

Now serve real balls. Watch which the runner engages on and which it rejects, and confirm both are
correct. Tune your *serving* here, not the robot. This step is free and it is where the session is
won.

If your policy needs external localization, confirm here that the base stream is live, in the right
frame and units, and not stale — a policy fed a frozen base position engages confidently and swings
at nothing.

### The planner side

The planner runs on the onboard machine and subscribes to the ball only; its strike plane is a fixed
number you set. The robot's base pose goes to the **runner**, not the planner. Getting this
backwards is the most common first-session wiring error.

```bash
ssh "$ONBOARD"
source /opt/ros/jazzy/setup.bash && source ~/hope_ws/install/setup.bash
export ROS_DOMAIN_ID="$DOMAIN"

# HARD GATE: prove mocap reaches this machine before starting anything
ros2 topic echo --once "$BALL_POSES_TOPIC"  # set to the configured ball poses topic

ros2 run hope_planner hope_planner_node --ros-args \
  --params-file /absolute/path/to/planner_params.yaml \
  -p x_hit:="$STRIKE_PLANE_X" \
  -p swing_side_split_y:="$SWING_SIDE_SPLIT_Y" \
  -p target_land_x:="$TARGET_LAND_X" -p target_land_y:="$TARGET_LAND_Y"
```

Toss a ball and check the command against the room, not against your expectations: is the target
position where you would put a racket; does the side match where the ball went; does time-to-strike
count *down*; does the task id increment once per ball.

If `ping` works but a topic is invisible across machines, it is DDS, not mocap — see
[Appendix A](#appendix-a--dds-across-a-venue-network).

---

## 10. Motion on the robot

### Hand over motor control ⚠

**The robot must be hoisted or supported.** Stopping the controller that owns the motors releases
whatever was holding the robot, and nothing holds it again until your runner reaches MOTION.

```bash
# 1. stop the default control stack that owns the motors at boot
# 2. confirm the shared middleware broker your runner needs is STILL ALIVE
# 3. start the low-level hardware layer, if it was not part of what you stopped (blocks; own terminal)
```

Write this sequence and its reverse down before the session. Do not improvise it with a hoisted
robot in front of you. Restore the normal stack afterwards.

### Run

```bash
ssh -J "$ONBOARD" "$ROBOT"
cd "$DEPLOY"
export A3_PINGPONG_RUNTIME_CFG="$PWD/config/$(basename "$CFG")"
mkdir -p "/tmp/hope/$SESSION/real"

taskset -c 4-7 ./run_a3_pingpong.sh \
  --planner --policy-native --start passive --official-stand \
  --trace-csv "/tmp/hope/$SESSION/real/trace.csv" \
  --obs-csv   "/tmp/hope/$SESSION/real/obs.csv" \
  --session-id "$SESSION" \
  > >(tee -a "/tmp/hope/$SESSION/real/runner.log") 2>&1
```

Confirm the banner again before pressing anything.

### The arm ritual

```text
1.  robot on its marked spot, standing still, squared up, facing the opponent
2.  's'  STAND   — verify it is genuinely weight-bearing before continuing
3.  wait ~2 s, completely still — the runner aligns heading here; moving or
                 leaning rotates EVERY subsequent target
4.  'm'  MOTION  — now, and only now, serve
    'p'  PASSIVE — the abort, available at every instant including step 3
```

Step 3 is the one people skip. Skipping it is invisible until targets come out rotated and the robot
swings at nothing.

If your runner captures a reference position at startup, **where the robot stands when you start it
is that reference for the whole run.** Mark the spot. Repositioning without restarting leaves the
policy holding a reference that no longer matches the world.

### Serving

- **Respect the lead time you measured in §7.** A policy that needs time to plan and move will
  correctly refuse anything faster. Serve slow and high until you have a baseline.
- **Start narrow** — one placement, one mode, repeatedly. A failure with one variable has one
  candidate cause.
- **One action per point.** Reposition between points unless you trained post-action homing.
- **Long runs of rejections are information**, not malfunction — usually your geometry disagrees
  with where the balls actually are. Go back to §9's planner checks.

### Collect and restore

```bash
rsync -az -e "ssh -J $ONBOARD" "$ROBOT:/tmp/hope/$SESSION/" "logs/$SESSION/robot/"
rsync -az "$ONBOARD:/tmp/hope/$SESSION/" "logs/$SESSION/onboard/"
```

Keep logs, traces, artifact hashes, and a `ros2 bag record` of the input topics — the input
recording is the highest-value artifact, because it replays the session offline with no robot and
no rig. Note the clock offsets between machines before comparing timestamps across them.

When something goes wrong, work backwards in order: did the input arrive, was the target sane, did
the runner accept it, did the action execute. Out of order is how sessions get blamed on the policy.

Restore the robot's normal control stack.

---

## Safety

- A human's hand stays on the abort at all times. A person, watching, finger on the key.
- The §10 handover **drops the robot**. Hoisted or supported, feet off-load. Never free-standing.
- Shadow (§9) before motion (§10). Always.
- Rejection is the safe outcome. A robot that stands still and refuses balls is a working robot with
  wrong geometry.
- Your vendor's motor protection, joint limits, comms timeouts, and e-stop stay authoritative.
- Keep the swing volume clear of people. Stand behind or beside the robot, never in the strike plane.

---

## Appendix A — DDS across a venue network

Many venue access points do not forward multicast between wireless clients. ROS 2 discovery is
multicast by default, so every node works locally and nothing crosses machines — while `ping`
succeeds the whole time. **A successful ping proves unicast routing and nothing else.**

Prove the data path: publish on one machine, echo on another. If that fails while ping succeeds,
configure unicast peers. Save a Fast DDS profile listing this machine's local interfaces:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>static_peer_udp</transport_id>
        <type>UDPv4</type>
        <maxInitialPeersRange>32</maxInitialPeersRange>
        <interfaceWhiteList>
          <interface>REPLACE_WITH_LOCAL_INTERFACE</interface>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="static_peer" is_default_profile="true">
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports><transport_id>static_peer_udp</transport_id></userTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

Export before launching **every** process that crosses machines — including the `ros2 topic echo`
you verify with, or you will "confirm" a problem that exists only in your verification shell:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/abs/path/profile.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=/abs/path/profile.xml
export ROS_STATIC_PEERS="<peer-ip>;<peer-ip>"      # every remote machine
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="$DOMAIN"
```

- List every peer on **every** machine, not just one direction.
- Take the interface from `ip route get <peer>`, not from habit. A machine bridging two networks
  needs both listed.
- Keep the domain modest: the last probed port is `7400 + 250·domain + 10 + 2·(maxInitialPeers − 1)`
  and must stay under 65535. Exceeding it fails like a firewall problem.

---

## Appendix B — what your runner must provide

Independent of what your policy does:

| Mode | Behaviour |
|---|---|
| Passive | Zero gains, limp. Startup state and abort target |
| Stand | Stable standing pose, weight-bearing, no policy actuation |
| Shadow | Full pipeline — obs, policy, decision logic — publishing **nothing** |
| Motion | The policy drives the robot |

- **An always-available abort** to passive from any state, including mid-action.
- **A fall guard**: down for longer than a short threshold → passive, automatically.
- **A startup banner**: model path and hash, obs contract and dimension, every input source and
  whether it is live, control rate. On screen, not inferred.
- **Fail closed**: missing metadata, unexpected obs dimension, stale command, hash mismatch → refuse
  to run. A runner that starts anyway and behaves subtly differently costs far more.
- **Every rejection path ends in a held stand** — never a partial or improvised action.

---

## Quick reference

| Step | Where | Command |
|---|---|---|
| §2 export | training env | `PP_EXPORT_PREFLIGHT_ONLY=1 TASK=… bash scripts/export_onnx_hitter_pure.sh "$RUN" "$CKPT"` then without the preflight var |
| §3 policy check | host | `"$PYBIN" scripts/mujoco_eval_onnx.py --onnx "$ONNX" --deploy-faithful --pd-mode implicit --steps 6000` |
| §4 sync + parity | deploy env | `PYBIN="$PYBIN" bash scripts/sync_pingpong_model.sh "$ONNX" "$ASSET" "$CFG"` → `PARITY: PASS` |
| §4 parity alone | deploy env | `MODEL=… PYBIN=… bash scripts/pingpong_parity/run_parity.sh` |
| §5 build x86 | deploy env | `bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --runtime-cfg "$CFG"` |
| §6 policy-only sim | deploy env | `bash scripts/run_sim.sh` + `bash scripts/pp_freebase_watch.sh --single-swing` |
| §7 closed loop | deploy env | `./run_a3_pingpong.sh --planner --policy-native --start passive --official-stand` |
| §8 build target | host (Docker) | `bash scripts/build_a3_deploy_pkg.sh --arch rockchip --runtime-cfg "$CFG"` |
| §8 ship | host | `rsync -azP -e "ssh -J $ONBOARD" dist/a3_deploy_rockchip/ "$ROBOT:$DEPLOY/"` |
| §9 shadow | robot | `./run_a3_pingpong.sh --planner --policy-native --dry-run --start shadow` |
| §10 motion | robot | `./run_a3_pingpong.sh --planner --policy-native --start passive --official-stand` |

---

## Related

- [run_pingpong_end_to_end.md](run_pingpong_end_to_end.md) — one project's fully instantiated
  version of this runbook, with its actual addresses, checkpoints, and pass criteria
- run_v17_r1_fixed3_hardware_trial.md (historical document; not included in this checkout) — the isolated
  V17-r1 `model_16600` fixed-station qualification procedure, including the OptiTrack/NatNet
  masked-MODELDEF preflight and three field-observed-return objective
- [../interfaces/policy_observation_action.md](../interfaces/policy_observation_action.md) — the
  observation and action contract
- [build_and_test.md](build_and_test.md) — building the workspace and the deploy package
