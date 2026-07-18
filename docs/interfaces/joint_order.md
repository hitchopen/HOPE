# A3 Joint Order

The public A3 starter uses 31 active joints, excluding hands. This order matches
`hope_training/config/joint_order_agibot_a3.yaml` and
`whole_body_tracking.robots.agibot_a3.AGIBOT_A3_JOINT_NAMES`.

```text
waist_yaw_joint
waist_roll_joint
waist_pitch_joint
head_yaw_joint
head_pitch_joint
left_shoulder_pitch_joint
left_shoulder_roll_joint
left_shoulder_yaw_joint
left_elbow_joint
left_wrist_roll_joint
left_wrist_pitch_joint
left_wrist_yaw_joint
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_joint
right_wrist_roll_joint
right_wrist_pitch_joint
right_wrist_yaw_joint
left_hip_pitch_joint
left_hip_roll_joint
left_hip_yaw_joint
left_knee_joint
left_ankle_pitch_joint
left_ankle_roll_joint
right_hip_pitch_joint
right_hip_roll_joint
right_hip_yaw_joint
right_knee_joint
right_ankle_pitch_joint
right_ankle_roll_joint
```

Retargeted CSV inputs should use this column order after the root
position/quaternion columns unless you also update the loader configuration.

## Deployment Backend And Policy View

The optional A3 deployment example uses the same 31-DOF backend layout for
state and command vectors:

```text
[0..2]   waist
[3..4]   neck/head
[5..11]  left arm
[12..18] right arm
[19..24] left leg
[25..30] right leg
```

The reference deployment policy view is 29 DOF. It skips the two neck/head
joints and keeps this order:

```text
[0..2]   waist
[3..9]   left arm
[10..16] right arm
[17..22] left leg
[23..28] right leg
```

Use the deployment helpers `ExtractPolicyView()` and `ExpandToBackend()` when
bridging between these layouts. Do not publish a 29-DOF policy vector directly
to the 31-DOF body-drive command topics.
