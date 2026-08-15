# Planner interface

The planner turns a stream of motion-capture ball positions into a per-strike racket target
for the policy. It is a no-spin, continuous planner: it predicts where the incoming ball
crosses a virtual hit plane, chooses forehand or backhand, and publishes a racket target
position, velocity, face normal, and strike timing.

There is one supported ROS runtime implementation:

| Package | Role |
|---------|------|
| `hope_ws/src/hope_planner_cpp` (C++) | Production and Gate 3 Planner, with the flight packetizer, audit logger, batch physics estimator, and `/planner/diagnostics`. |

`hope_ws/src/hope_planner` is excluded from colcon with `COLCON_IGNORE`. Its
Python source is retained only as an offline numerical reference; do not launch
it or add it to a production build.

## Data flow

```
mocap ball positions (/poses, ball at index 0)
  -> hope_ball_flight_packetizer (immutable /ball/flight_packet)
  -> C++ batch position/velocity estimate
  -> no-spin trajectory prediction (gravity + drag + table bounce)
  -> virtual hit plane crossing (fixed x_hit by default)
  -> forehand/backhand selection
  -> outgoing-shot solve (landing target on the opponent half)
  -> RacketCommand + /racket/command_flat (+ diagnostics)
```

- Every incoming mocap sample feeds the Laptop-side packetizer. It freezes one
  complete incoming-flight packet; the C++ Planner solves that immutable packet
  once and the Runner freezes the accepted command when the swing engages.
- The ball model is no-spin: `[x, y, z, vx, vy, vz]` with gravity, measured drag, and
  measured table/paddle restitution. The shipped parameters are a real venue fit
  (`drag_k = 0.1261`, see `configs/ball_physics_venue.yaml` and the fitting tools under
  `hope_training/ball_physics_fit/`); planner, fake-ball publisher, and evaluator must use
  matching values or every command arrives invalid/late.
- An optional **spin shadow** estimator exists in the C++ planner
  (`spin_shadow_enabled`, default **false**; modes include `venue_grip_magnus`). It is an
  observability/diagnostics channel fed by `/ball/pose` rigid-body orientation — the
  published command remains the no-spin solution.
- The virtual hit plane is **fixed** by default (`x_hit_follow_robot: false`) — the
  x-locked HITTER convention: the robot walks to a commanded station behind a fixed plane.
  Per-side plane offsets are supported for backhand reach.

## Forehand / backhand selection

The planner predicts the ball's lateral (`y`) position where it crosses the hit plane and
compares it to the configured split (with hysteresis so alternating rallies don't flap).
A ball toward the paddle side (`-y`) is taken forehand (`swing_sign = +1`); the other side
backhand (`-1`). The side is chosen per incoming ball, carried on the wire as
`swing_sign`, consumed by the runner's engage machine — and **never observed by the
policy** (see [POLICY_INTERFACE.md](POLICY_INTERFACE.md)).

## Wire contract

### `hope_msgs/RacketCommand` (default `/racket/command`)

The rich ROS message, used by tooling, gates, and the MuJoCo closed loop:

```
std_msgs/Header header
geometry_msgs/Point position               # target racket position, world frame, m
geometry_msgs/Vector3 velocity             # target racket velocity, world frame, m/s
geometry_msgs/Vector3 normal               # desired racket face normal at contact
float64 strike_time                        # absolute strike wall time, s
float64 time_to_strike                     # seconds until the strike
geometry_msgs/Vector3 ball_velocity_outgoing  # predicted outgoing ball velocity
bool valid                                 # command is currently actionable
bool clears_net                            # predicted outgoing shot clears the net
bool bypasses_net_posts                    # predicted shot passes outside the net posts
int32 predicted_bounces                    # incoming-trajectory bounce count
```

### Flat topics (`std_msgs/Float64MultiArray`) — the hardware wire

The C++ runner on the robot's motion unit deliberately subscribes **core `std_msgs`
flats** instead of `hope_msgs` (no custom rosidl typesupport needed in the aarch64
cross-build). Layouts are pinned in
`a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_planner_input.hpp`;
element `[0]` is always a schema tag:

- **`/racket/command_flat`** — schema 2 (19 doubles):
  `[0]=2, [1]=valid, [2]=swing_sign, [3..5]=pos, [6..8]=vel, [9]=time_to_strike,
  [10]=absolute strike wall time, [11]=frame_code, [12..13]=producer stamp,
  [14]=command_seq, [15]=flight_id, [16]=revision_id, [17]=estimator sample count,
  [18]=estimator span`. (Schema 1, ≥11 doubles, remains accepted for legacy tooling.)
- **`/a3/base_pose_flat`** — schema 2 (16 doubles, authoritative for the deploy line):
  `[0]=2, [1]=valid, [2]=sequence, [3..4]=source stamp, [5..7]=base position,
  [8..11]=base quaternion wxyz, [12]=tracking quality, [13]=flags,
  [14..15]=calibration receipt ids`. Published by the process-isolated
  `hope_base_pose_flat_relay` from mocap `/P1/pose`; flags carry
  tracking/quaternion/extrinsic/world-calibration validity bits.
- **`/serve/ball_state_flat`** — pre-serve ball state (≥11 doubles) for the runner's
  scripted-serve mode.

Freshness is enforced with local monotonic receipt age; the schema-2 absolute strike
deadline is converted once into the receiver's monotonic domain so wall-clock corrections
never move a deadline.

## Continuous rallies

Each incoming ball is a new **flight** (`flight_id`); pre-strike refinements increment
`revision_id` for the same flight. After a strike the planner moves to the next flight;
the robot is never reset between flights, and all four side transitions (FH→FH, FH→BH,
BH→FH, BH→BH) occur naturally across a rally.

## Configuration

The production Planner config is
`hope_ws/src/hope_planner_cpp/config/model21800_hardware.yaml`: virtual hit
plane (`x_hit`), side split/hysteresis, landing target, solve rate and venue-fit
physics. The Laptop packet boundary is configured by
`model21800_flight_packetizer.yaml`. Gate 3 and Foxglove use these same files.
