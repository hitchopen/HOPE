# Frames

HOPE uses the ROS 2 REP 103 axis convention.

## Table World Frame

- Origin: near-side left corner of the table surface.
- X: toward the opponent, along the table length.
- Y: left, along the table width.
- Z: up.
- Table surface height: 0.76 m above the floor.

## Robot Frame

- `base_link` or the robot root body is estimated from mocap in deployment.
- Isaac Lab training uses the simulated root body `pelvis_link` for A3.
- The right arm holds the paddle in the current A3 starter task.

## Racket Frame

- The A3 starter uses `right_wrist_yaw_Link` as the wrist body.
- The racket center body is `pingpang_red_Link`.
- The configured racket face normal is local +Y for the red/forehand face.

No public v1 task depends on live racket mocap. Racket pose is inferred through
robot kinematics.
