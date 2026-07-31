# A3 Assets

This branch includes the public Agibot A3 materials used by the HOPE starter.

For the Isaac Lab quickstart, teams only need the source URDF package and the
asset preparation script. The rest of `agibot/` is Agibot-provided reference
material for teams studying deployment or optional MuJoCo/AimRT simulation.

## What Each A3 Area Means

| Path | Required for quickstart? | Role |
|------|--------------------------|------|
| `agibot/URDF/A3T2.5-URDF-std-pingpang/` | Yes | Source A3 ping-pong URDF, meshes, joint config, and metadata. |
| `hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py` | Yes | Copies the source URDF package into the Isaac Lab Python package and rewrites mesh paths for local loading. |
| `hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/` | Generated locally | Derived Isaac-ready copy. It is ignored by git and can be regenerated. |
| `hope_training/config/joint_order_agibot_a3.yaml` | Yes | Canonical public A3 policy joint order. |
| `agibot/code_deployment/` | No | Agibot A3 deployment example for ONNX policy runtime, body-drive I/O, and an optional experimental [fixed vendor-arm serve](agibot/code_deployment/a3_deploy_example/SERVE_SCRIPT_DEPLOY.md). |
| `agibot/A3_MuJoCo_Sim/` | No | Agibot MuJoCo/AimRT simulation reference. Not required for Isaac smoke training. |

## Source URDF

The source package is:

```text
agibot/URDF/A3T2.5-URDF-std-pingpang/
```

It contains the A3 ping-pong URDF, mesh files, joint-name config, and runtime
metadata, plus Agibot's source authoring helpers.

This is the racket-equipped A3 variant. The URDF uses
`right_hand_pingpang_Link` and fixed `pingpang_red_Link` /
`pingpang_black_Link` racket bodies. The broader `agibot/URDF/` bundle also
contains Agibot's non-racket A3 source variant for reference; the Isaac starter
uses the racket-equipped package above.

## Isaac Lab Prepared Copy

Isaac Lab loads the prepared asset from:

```text
hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/
```

Generate it with:

```bash
python3 hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py --force
python3 hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py --check
```

The script copies the URDF package into the Python package asset directory,
writes `urdf/model.urdf`, and rewrites mesh paths from `package://.../meshes`
to relative `../meshes/...` paths that Isaac Lab can resolve from a fresh
clone.

The prepared copy is intentionally git-ignored because it is derived from the
source URDF. Regenerate it locally after cloning.

## Joint Order

The A3 active-policy joint order is documented in:

```text
hope_training/config/joint_order_agibot_a3.yaml
```

It is mirrored in code by `whole_body_tracking.robots.agibot_a3.AGIBOT_A3_JOINT_NAMES`
and, on the deploy side, by
`a3_deploy/a3_deploy_example/reference/a3_deploy_onnx_ref_pingpong/joint_order.py`.
See [docs/POLICY_INTERFACE.md](docs/POLICY_INTERFACE.md) for the full
observation/action contract that consumes it.

Use that order for retargeted CSV columns and policy action/observation
contracts unless you intentionally change the robot configuration.

## Deployment Example

The Agibot A3 deployment example is under:

```text
agibot/code_deployment/
```

This area is optional for the Isaac quickstart. It is useful after teams have
exported policies and want to study Agibot's body-drive state/command topics,
runtime configuration, and deployment packaging examples. It also contains a
narrow fixed-motion vendor-arm serve example; that route is real-robot
experimental code, not an Isaac dependency or a certified deployment path.

## MuJoCo / AimRT Reference

The Agibot MuJoCo/AimRT reference project is under:

```text
agibot/A3_MuJoCo_Sim/
```

This is included as Agibot-provided reference material. The v1 quickstart does
not claim a validated MuJoCo RL training backend; the supported first run is
still Isaac Lab asset load, table-tennis scene smoke test, and PPO smoke
training.
