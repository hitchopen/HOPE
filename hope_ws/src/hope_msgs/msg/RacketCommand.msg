# Racket target command published by the HOPE PingPong planner.
#
# One task_id is created per incoming ball. Pre-strike updates keep the same
# task_id and increase task_revision. swing_side is chosen once per task_id and
# stays fixed within it. All fields refer to the world frame (+x forward,
# +y left, +z up; metres, seconds).
std_msgs/Header header

# Identifies the incoming ball this command belongs to (new ball -> new id).
uint64 task_id
# Monotonically increasing within a task_id as the pre-strike plan is refined.
uint32 task_revision

# swing_side values.
int8 FOREHAND=1
int8 BACKHAND=-1
# Selected swing side for this task (FOREHAND or BACKHAND). Formal side channel.
int8 swing_side

# Target racket position at the strike, world frame (m).
geometry_msgs/Point position
# Target racket velocity at the strike, world frame (m/s).
geometry_msgs/Vector3 velocity
# Remaining time until the strike (s).
float64 time_to_strike
