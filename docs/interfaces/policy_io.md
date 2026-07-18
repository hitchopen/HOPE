# Policy IO

This file records the stable public contract for the starter branch. Exact
tensor sizes can change if you edit the Isaac Lab environment config, but the
high-level contract should stay consistent.

## Training Motion File

`scripts/train.py` accepts:

```bash
motion_file=/path/to/motion.npz
```

The `.npz` must contain:

- `fps`
- `joint_pos`
- `joint_vel`
- `body_pos_w`
- `body_quat_w`
- `body_lin_vel_w`
- `body_ang_vel_w`

`scripts/create_smoke_motion.py` generates a tiny local file with this schema.

## Actions

The A3 starter policy outputs joint position targets for the active A3 joint
order in `docs/interfaces/joint_order.md`. The Isaac Lab articulation applies
these through configured implicit actuators.

## Observations

The tracking task includes robot proprioception and motion-reference command
features from the BeyondMimic-style environment. The ping-pong task additionally
contains racket target command terms. See the Isaac Lab env configs under:

```text
hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/tasks/
```

## A3 Deployment Reference IO

The optional Agibot A3 deployment example under `agibot/code_deployment/` includes
an ONNX runtime reference. It is not required for the Isaac Lab smoke-training
path, but it documents the policy/runtime contract used by the A3 body-drive
example.

The reference deployment code builds a 1570-float `obs_dict` input:

```text
[   0 ..  579]  command_multi_future_nonflat
[ 580 ..  639]  motion_anchor_ori_b_mf_nonflat
[ 640 ..  669]  base_ang_vel over 10 history steps
[ 670 ..  959]  joint_pos over 10 history steps
[ 960 .. 1249]  joint_vel over 10 history steps
[1250 .. 1539]  previous actions over 10 history steps
[1540 .. 1569]  gravity_dir over 10 history steps
```

The deployment backend exposes a 31-DOF A3 command/state layout, including neck
joints. The reference policy view is 29 DOF and skips neck/head joints. The
deployment helper expands 29-DOF policy outputs back into the 31-DOF backend
layout before publishing body-drive commands.

Teams can replace the included Agibot reference motions, tokenizer metadata,
and exported policy artifacts with their own trained policies when moving
beyond the Isaac smoke path.

## Logging

The public smoke command uses:

```bash
logger=tensorboard
```

WandB registry/artifact logging is optional.
