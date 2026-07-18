# Replacing the motion clips

Training imitates two motion clips. The ones shipped under
`hope_training/motions/preprocessed/` are **reference examples only** — short, smooth,
physically-neutral placeholders that let the loader and shape checks pass. They are **not**
performance-tuned. Replace them with your own retargeted forehand/backhand motions before training a
policy you intend to deploy.

This page documents the file format so you can produce your own. There is deliberately **no** motion
validator, scorer, receipt, or qualification step — the loader reads the fixed format below, and
ordinary file/shape errors surface naturally.

## The two clips

```
hope_training/motions/preprocessed/hope_forehand.npz   (clip 0, swing_side +1)
hope_training/motions/preprocessed/hope_forehand.yaml
hope_training/motions/preprocessed/hope_backhand.npz   (clip 1, swing_side -1)
hope_training/motions/preprocessed/hope_backhand.yaml
```

Each clip should cover one full swing: **ready → strike → follow-through → recoverable end pose**.
Keep both files' `fps`, joint order, and tracked-body list identical to the schema below; the loader
validates that the arrays match.

## `.npz` schema

All arrays are `float32`, retargeted to the Agibot A3 and expressed in the shared world frame
(+x forward, +y left, +z up). `F` is the frame count.

| Key | Shape | Meaning |
|-----|-------|---------|
| `fps` | scalar | frames per second (e.g. 50) |
| `joint_pos` | `(F, 31)` | joint positions, in the [31-DOF joint order](POLICY_INTERFACE.md#joint-order) |
| `joint_vel` | `(F, 31)` | joint velocities, same order |
| `body_pos_w` | `(F, 14, 3)` | world positions of the 14 tracked bodies |
| `body_quat_w` | `(F, 14, 4)` | world orientations (quaternion, **wxyz**) |
| `body_lin_vel_w` | `(F, 14, 3)` | world linear velocities of the tracked bodies |
| `body_ang_vel_w` | `(F, 14, 3)` | world angular velocities of the tracked bodies |

The 14 tracked bodies are stored in this exact order (index 0 is the root, index 7 is the anchor the
imitation reward aligns to):

```
 0 pelvis_link         (root)        7 torso_Link          (anchor)
 1 left_hip_roll_Link                8 left_shoulder_roll_Link
 2 left_knee_Link                    9 left_elbow_Link
 3 left_ankle_roll_Link             10 left_wrist_yaw_Link
 4 right_hip_roll_Link              11 right_shoulder_roll_Link
 5 right_knee_Link                  12 right_elbow_Link
 6 right_ankle_roll_Link            13 right_wrist_yaw_Link
```

The loader raises a clear error if the tracked-body count does not match, so keep this list and its
order in sync with the YAML sidecar and the robot asset.

## `.yaml` sidecar

The sidecar describes the clip's phase structure and racket convention (see
`hope_forehand.yaml` for a complete example):

- `name`, `swing_side` (`+1` forehand / `-1` backhand)
- `fps`, `frame_count`, `frame_time_s`, `duration_s`
- `strike_frame`, `strike_phase` (fraction of the clip at the strike)
- `ready_interval_frames`, `follow_through_end_frame`, `recover_end_frame`
- `joint_order` (the 31 joint names, in order)
- `tracked_bodies`, `anchor_body`, `root_body`
- `racket_link`, `racket_body`, `mount_offset_xyz` (wrist → racket-centre offset in the wrist frame),
  `blade_normal_axis`, `blade_normal_sign` (the public blade-face convention)

## Producing your own clips

Record or synthesize a forehand and a backhand swing, retarget them to the A3's 31 DOF, resample to a
fixed `fps`, and write the arrays above into a `.npz` plus a matching `.yaml`. Point training at them
either by placing them at the default paths, or via the CLI:

```bash
python scripts/train.py task=HOPEPingPong \
    motion_file=/path/to/your_forehand.npz \
    motion_file_2=/path/to/your_backhand.npz
```

(or set `commands.motion.motion_file` in `cfg/task/HOPEPingPong.yaml`). The retargeting pipeline
itself is out of scope for this repository — any tool that emits the schema above works.
