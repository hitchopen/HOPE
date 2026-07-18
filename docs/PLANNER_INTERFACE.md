# Planner interface

The planner turns a stream of motion-capture ball positions into a per-strike racket target
for the policy. It is a no-spin, continuous, fixed-station planner: it predicts where the
incoming ball crosses a fixed strike plane, chooses forehand or backhand, and publishes a
racket target position, velocity, time-to-strike, and swing side.

## Data flow

```
mocap ball positions (/poses)
  -> position/velocity estimate
  -> no-spin trajectory prediction
  -> fixed strike plane
  -> forehand/backhand split
  -> fixed opponent-half landing target
  -> RacketCommand (position, velocity, time_to_strike, swing_side)
```

- Every incoming mocap sample feeds the estimator; the (more expensive) trajectory solve runs
  at **at most 50 Hz**.
- The first timestamped sample seeds position; a second sample at a different timestamp
  enables the velocity estimate; after that the planner publishes directly.
- The ball model is no-spin: state is `[x, y, z, vx, vy, vz]` with gravity, measured drag, and
  measured table/paddle restitution from `configs/ball_physics.yaml`. There is no spin
  estimation, no Magnus force, and no per-side adaptive strike plane.

## Continuous rallies

Each incoming ball opens a new **task**:

- `task_id` — a new unique id per incoming ball.
- `task_revision` — increments monotonically as the pre-strike trajectory estimate is refined
  for the *same* ball.
- `swing_side` — chosen once when the task opens and held constant for that task.

After a strike the planner opens the next task for the next ball. The robot is never reset
between tasks. All four adjacent side transitions (FH→FH, FH→BH, BH→FH, BH→BH) occur naturally
across a rally.

## Forehand / backhand selection

The planner predicts the ball's lateral (`y`) position where it crosses the fixed strike
plane and compares it to `swing_side_split_y` (with optional small hysteresis):

```
crossing_y <  swing_side_split_y  -> FOREHAND (+1)
crossing_y >= swing_side_split_y  -> BACKHAND (-1)
```

A ball arriving **below** the split (toward the paddle side, `-y`) is taken forehand; a ball
at or above the split is taken backhand. Boundary cases, with `split = swing_side_split_y`
and hysteresis `h` (`prev` = the previous task's side):

| `crossing_y`            | `prev`     | selected side |
|-------------------------|------------|---------------|
| `< split`               | none       | FOREHAND      |
| `= split` or `> split`  | none       | BACKHAND      |
| `<= split + h`          | FOREHAND   | FOREHAND (sticky) |
| `> split + h`           | FOREHAND   | BACKHAND      |
| `>= split - h`          | BACKHAND   | BACKHAND (sticky) |
| `< split - h`           | BACKHAND   | FOREHAND      |

This convention is implemented in `hope_planner/side_selection.py` and pinned by
`hope_planner/test/test_side_selection.py`. There is no higher-level shot selection, side
optimization, or opponent adaptation. `swing_side` is a formal field of the message — it is
no longer inferred downstream from the target's Y sign.

## `RacketCommand.msg`

Published on the racket-command topic (default `/racket/command`), consumed by the runner:

```
std_msgs/Header header
uint64 task_id
uint32 task_revision
int8 FOREHAND=1
int8 BACKHAND=-1
int8 swing_side
geometry_msgs/Point position          # target racket position, world frame, m
geometry_msgs/Vector3 velocity        # target racket velocity, world frame, m/s
float64 time_to_strike                # seconds until the strike
```

The message carries only what the policy needs. There is intentionally no `valid`/`reason`/
failure flag, no outgoing-ball prediction, no net/bounce prediction, no confidence, and no
diagnostics. The planner does not maintain a readiness or failure state — if the incoming
data is insufficient it simply has not published yet.

## Configuration

`hope_ws/src/hope_planner/config/hope_planner.yaml` holds the public parameters: the fixed
strike-plane position, `swing_side_split_y` (and hysteresis), the fixed opponent-half landing
target, the prediction horizon, and the solve rate. Ball physics is read from the shared
`configs/ball_physics.yaml`.
