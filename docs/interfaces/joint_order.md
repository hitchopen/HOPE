# A3 joint order

One canonical 31-DOF joint order is shared by training, the exported ONNX
policy, and the deploy runner. The source of truth is
[`hope_training/config/joint_order_agibot_a3.yaml`](../../hope_training/config/joint_order_agibot_a3.yaml);
the C++ deploy runner carries the same list (and the official hard-limit table)
under `a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/`.
The authoritative contract (with the full indexed table) is
[POLICY_INTERFACE.md — Joint order](../POLICY_INTERFACE.md#joint-order).

The observation `joint_pos` / `joint_vel` / `last_action` slices, the ONNX
`raw_action[31]` output, and the joint-position targets written to the robot all
use this exact index order:

```text
 0  waist_yaw_joint              16  right_wrist_roll_joint
 1  waist_roll_joint             17  right_wrist_pitch_joint
 2  waist_pitch_joint            18  right_wrist_yaw_joint
 3  head_yaw_joint    (passive)  19  left_hip_pitch_joint
 4  head_pitch_joint  (passive)  20  left_hip_roll_joint
 5  left_shoulder_pitch_joint    21  left_hip_yaw_joint
 6  left_shoulder_roll_joint     22  left_knee_joint
 7  left_shoulder_yaw_joint      23  left_ankle_pitch_joint
 8  left_elbow_joint             24  left_ankle_roll_joint
 9  left_wrist_roll_joint        25  right_hip_pitch_joint
10  left_wrist_pitch_joint       26  right_hip_roll_joint
11  left_wrist_yaw_joint         27  right_hip_yaw_joint
12  right_shoulder_pitch_joint   28  right_knee_joint
13  right_shoulder_roll_joint    29  right_ankle_pitch_joint
14  right_shoulder_yaw_joint     30  right_ankle_roll_joint
15  right_elbow_joint
```

- `head_yaw_joint` / `head_pitch_joint` (indices **3–4**) are passive at deploy:
  the runtime holds the neck at its nominal pose regardless of the actor output,
  but the columns still exist so every vector is length 31.
- The racket is mounted on the right wrist.

## Enforcement

A permuted joint enumeration would silently permute every observation and action
column, so the order is bound and checked at the artifact boundary:

| Stage | Check |
|-------|-------|
| Train (`scripts/train.py`) | Uses the articulation order of the selected Isaac asset; the published task and motion sidecars record the validated 31-column training order. |
| Export (`scripts/export_onnx.py`) | Verifies the order, then embeds it (with the runtime-contract metadata) in the ONNX and the exported deploy manifest. |
| Deploy (C++ loader) | Validates the embedded metadata — joint order, contract name, recipe fields, and the hard-limit table cross-checked against the official A3 URDF — and **fails closed** on any mismatch. |

If your A3 asset enumerates joints differently, fix the URDF/USD (or update the
canonical order everywhere at once); do not remap columns ad hoc. Verify against
the real robot before deploying, e.g. `ros2 topic echo /joint_states --once`.
