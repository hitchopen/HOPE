# GRU physical-ball evaluation with the live ROS planner

**SUPERSEDED DRAFT — do not run this custom protocol.** The user selected the
professor fixed-home-100 reference. See [the compatibility audit](GRU_FIXED_HOME_100_AUDIT.md).
The old launcher and draft driver are blocked; the protocol below is retained
only as development history, not an approved or executed benchmark.

This opt-in evaluation uses the existing C++ `hope_planner_cpp_node` in the
validated **hope** ROS 2 Jazzy Distrobox and the existing host
`~/workspace/nohope/.venv` for MuJoCo/ONNX Runtime. Recurrent export still runs
CPU-only in **grasping**. Training is never started, resumed, stopped or signaled.
No hardware launcher, runner, bridge, mocap relay, or robot topic is used.

## Data flow and responsibility

`planner_rally_protocol.py` generates only physical **serve initial conditions**.
They reuse clean-serve position/velocity cases from
`a3_deploy/a3_deploy_example/config/gate3/scenarios/fixed-home-100.yaml`.
`PhysicalBall` supplies existing MuJoCo gravity/aero/material/collision behavior;
its existing force-contact detection remains the physical authority.

The new host driver, `mujoco_eval_planner_rally.py`, passes actual measured ball
poses/orientations at 200 Hz through a private Unix socket to
`planner_ros_eval_bridge.py` inside hope. The bridge publishes those measurements
as ROS PoseArray and measured pelvis PoseStamped. The actual C++ planner fits
incoming motion, predicts the strike, and performs its Stage 3 racket solve.
Its schema-2 output is translated into the GRU's strike point, desired racket
velocity, and countdown. Actor outputs go through the existing action decoder,
plant and actual racket/ball collision. There are no bank strike targets, CSV
question fixtures, synthesized successful returns, or action interventions.
Before the first planner command the actor receives an explicit neutral idle
input (current racket position, zero velocity, time=-1); this is not scored as a
planner command. Afterwards the last real planner target remains with its
expired countdown until a new live command arrives. No new positive strike
countdown is created without a live planner command. The first valid command
per serve is latched, and all further messages are logged with their applied flag.

This uses the C++ node's explicitly supported **direct `/poses` legacy A/B input**,
not its production immutable flight-packet/packetizer pipeline. Its native
post-net one-shot logic is enabled; the separate packetizer is not launched.
Schema 2 is selected explicitly because the GRU harness has no native shared-HOME
schema-4 runner. No fixture target override is enabled. This evaluates that
specific live-planner mode, not a production packetizer or hardware qualification.

## Frames, time and policy contract

All units are SI. The planner table frame occupies x=[0,2.74], y=[-1.525,0],
surface z=0. MuJoCo world = planner position + **[0.5,0.7625,0.76]**.
Ball and base input subtract this translation. Orientations/velocities share axes.
The planner's schema-2 publisher already adds `policy_z_offset=0.76`, so output
positions add **[0.5,0.7625,0]**, avoiding a double height offset. Fixed planner
hit plane x=0.08 becomes world x=0.58. Side split is y=-0.25 relative to the
station HOME, with zero hysteresis for these separately specified FH/BH lanes.

The actual saved 110-D observation, embedded normalization, action/joint mapping,
passive head mask, action clamp/delay, 20 ms control and carried GRU state are
preserved. Physical mode retains 1 ms engine steps. Target velocity is desired
**racket** velocity from the planner, not measured ball velocity. Countdown is
planner absolute deadline minus the simulation's Unix-epoch clock, capped at the
existing 0.65 s pre-strike display/input maximum. No extra actor inputs are added.

The planner uses system Unix wall time for deadline rejection, independent of
ROS `/clock`. Source stamps are therefore monotonic simulated exposure times
anchored once to Unix wall time, paced in real time. Both host and ROS bridge
reject clock lag/skew >50 ms; the host rejects malformed schema, stale flight
identity, inconsistent producer/countdown/deadline timestamps, and commands with
less than 40 ms remaining at receipt. A slow host aborts preflight instead of
publishing a misleading benchmark with stretched physical time. This strict
real-time capacity has not been demonstrated from the blocked sandbox.

## Frozen 100-serve protocol

FH/BH serve classes refer to the specified incoming lanes relative to the fixed
station anchor; the actual planner-selected swing side is recorded independently.

The exact policy snapshot, ONNX bundle verification, protocol JSON and SHA-256,
planner parameters, source/config hashes and every attempt live outside Git.
The frozen protocol is selected before any scored run:

- Seed **0**, exactly **50 FH and 50 BH**, shuffled together reproducibly.
- Existing clean serves in planner table coordinates: x=2.4, z=0.49,
  velocity=(-3,0,2.2) m/s, spin=(0,0,0) rad/s. FH y is one of
  {-1.0225,-1.0575,-1.0925}; BH y is one of {-0.7125,-0.6575,-0.6125}.
  Translation gives world x=2.9, z=1.25 and the stationary table's lane positions.
  Launch speed is 3.72 m/s, within the documented 1–7 m/s, <=15 rev/s venue
  envelope. These are serve states, not prescribed strike positions/times.
- Two seconds initial warmup; one serve every **5 simulated seconds**, first
  launch at t=2, last at t=497. Total scored duration **502 simulated seconds**.
  No overlap or READY-based retry/admission; all scheduled failures count.
- Scored wall timeout **900 seconds**, native process timeout 915 s; outer
  launcher bound **2100 s** includes export/build/preflight. ROS bridge lifetime
  has its own 930 s cap. Preflight is separately capped at 90 s.
- Separate two-serve FH/BH integration preflight, fresh planner/process/robot/GRU.
  It must demonstrate valid physical deliveries, on-time planner commands and
  resolved physical outcomes with validated clock/frame/schema contracts and at
  least one positive-countdown policy observation from each live command. Hits
  or legal returns are not a preflight quality gate. No retry or parameter tuning
  is performed to make it pass. Failure prevents creation of scored results.

Ball spawning only rewrites the ball's initial state and bookkeeping. Robot
qpos/qvel, the action-delay FIFO, anchor and actor memory persist across serves.
The baseline evaluator's 12-second episode timeout would invalidate continuity;
this driver does not call that scheduler and **disables that timeout only for
this evaluation**. It resets robot and hidden state only on pelvis z<0.8 m,
tilt>60 degrees, nonfinite state or MuJoCo numerical warning, logging each event.
The fixed serve schedule then continues; a reset never silently drops a serve.
All other plant/action settings are inherited. Nominal zero action-delay cohort
is retained throughout a reset-free episode; full safety resets alternate zero
and saved maximum delay, matching the earlier evaluator reset cohorts.

## Physical score and denominators

For every one of the 100 scheduled serves, including those not started after an
infrastructure abort, JSON/CSV retain these independent fields:

- **Valid delivery:** actual incoming net crossing with radius clearance, actual
  own-half table contact before racket contact, then arrival at the world x=.58
  plane or actual racket contact, without incoming net contact. Merely spawning
  or crossing a policy reference time is insufficient.
- **Planner on time:** valid real schema-2 output for this serve, accepted with
  >=40 ms lead, correct timestamp/identity contract. Invalid, absent, late and
  unapplied messages remain in command/ROS diagnostic traces.
- **Contact:** positive MuJoCo force on the racket mesh. Net/table/floor contacts
  are likewise physical events. Counts distinguish contact serves from events.
- **Legal return:** exactly one racket contact before the first return landing;
  outgoing +x net crossing with center z>0.9325 and |y|<=0.7625; no net contact
  before landing; first landing x in (1.87,3.24], |y|<=0.7625; and a real table
  contact within 80 ms of the interpolated surface crossing. A plane crossing
  alone or floor bounce never qualifies. Net-post bypasses are excluded.
- A first actual return table contact or floor/out-of-play event resolves the
  outcome. A slot ending before resolution is **interrupted/unresolved**, never
  successful. A physical resolution snapshot is stored before any automatic
  off-court parking. Later events remain logged. A fall after an already legal
  physical return retains that physical fact but fails end-to-end success.
- **End-to-end success:** valid delivery AND on-time planner AND physical legal
  return AND resolved AND no fall/numerical reset in that serve. Primary rate
  denominator is **all 100**; FH/BH denominators are **50 each**. Also report
  end-to-end successes / valid deliveries and physical legal returns / contact
  serves, with integer numerators/denominators, including zero-denominator cases.
- **Recovery/READY proxy:** measured anchor drift<=0.1 m and base planar
  speed<=0.2 m/s continuously for 0.3 s after physical resolution, with no reset.
  These are the existing position/speed proxy plus a documented dwell, not the
  full training/hardware READY contract. First proxy completion time is saved.
  It does not gate the next serve or inject control actions.

Summary includes misses, failed returns, invalid deliveries, planner failures,
reset events (including warmup), longest end-to-end success streak, and contiguous
reset-free serve segments. Unstarted serves remain unresolved in the primary
100 denominator. No rate from an interrupted run is presented as a completed
100-serve benchmark.

## Isolation, launch and evidence

ROS discovery is localhost-only, domain **187**, with a unique namespace for each
attempt/phase. The graph is checked before planning and every second; unrelated
nodes cause an abort. Only the C++ planner and this adapter are allowed. All ball,
base and command topics are remapped into the evaluation namespace; hardware
command bridges are never launched. The bridge socket is mode 0600 inside a fresh private directory under
`~/.cache/hope-rally-*` (short enough for Unix-socket path limits, shared with
Distrobox through home). Its exact path is recorded in the attempt directory. Cleanup
signals only the owned bridge PID after checking its /proc command line, and that
bridge cleans up its own planner child. No global pkill/tmux/training controls.

Run from the normal Ubuntu desktop terminal:

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261105-212509-planner-100/run-100-serves.sh" --view
```

The script prints the frozen checkpoint iteration and exact bundle path. It uses
only existing environments/dependencies, builds the two ROS packages into a new
isolated user build/install tree, verifies checkpoint-to-ONNX MLP/GRU initializer
identity and 64-step PyTorch/ONNX parity/reset behavior, then runs preflight before
the scored 100. It refuses unverified bundle reuse. Repeating the same command
reuses only this explicitly verified frozen policy/protocol and creates a new
attempt directory, preserving prior evidence. No X11 security changes are made.
For headless evaluation omit `--view`. Software viewer rendering is optional and
must satisfy the same preflight clock bound. Video is not enabled by default
because preserving actual planner wall deadlines takes priority; sensor/event
traces and the visible repetition command remain available.

Each attempt saves complete per-serve CSV/JSON, actual event JSONL, ball/base
sensor and planner diagnostics, planner audit CSV, all actor observations and
hidden-state/reset traces, parameters, exact process argv, build/export logs and
summary. The frozen session also saves source hashes, checkpoint/config hashes,
protocol and script files. No trained weights, ROS build products or attempt
artifacts enter Git. Keep source frozen between preflight and scored evaluation.

Known differences remain: MuJoCo soft contacts versus fitted tangential impulse,
Stage 3's documented omission of outgoing Magnus, perfect simulated mocap at
200 Hz rather than real 360 Hz noisy transport, legacy direct-pose planner mode,
nominal plant without training randomization, the partial READY proxy and the
explicit long continuous evaluation instead of training reset/scheduler logic.

Sandbox preparation passed focused host tests and a two-side physical serve
geometry unit check. It could not enter the existing containers because Podman
cannot chmod `/run/user/1001/libpod` on the read-only runtime filesystem. That
geometry check is **not** the actual-planner integration preflight. The new policy
export, ROS adapter preflight and scored 100-serve evaluation remain pending the
desktop launcher; no live-planner success rate is claimed from preparation alone.
