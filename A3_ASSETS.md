# A3 Assets

This branch includes the public Agibot A3 materials used by HOPE.

For the Isaac Lab quickstart, teams only need the source URDF package and the
asset preparation script. The complete hardware/reference-software inventory,
printing units and A3-specific setup guidance live in
[agibot/README.md](agibot/README.md). This page owns the cross-workspace asset
paths, Isaac preparation and joint-order contract.

## What Each A3 Area Means

| Path | Required for quickstart? | Role |
|------|--------------------------|------|
| `agibot/URDF/A3T2.5-URDF-std-pingpang/` | Yes | Source A3 ping-pong URDF, meshes, joint config, and metadata. |
| `hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py` | Yes | Copies the source URDF package into the Isaac Lab Python package and rewrites mesh paths for local loading. |
| `a3_deploy/URDF/` | No | Optional override location for your own vendor-supplied A3 URDF copy (`--source-root`, see its [README](a3_deploy/URDF/README.md)). |
| `hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/` | Generated locally | Derived Isaac-ready copy. It is ignored by git and can be regenerated. |
| `hope_training/config/joint_order_agibot_a3.yaml` | Yes | Canonical public A3 policy joint order. |
| [agibot/](agibot/README.md) (other assets) | No | Mounting hardware, serving gripper STL, clock synchronization and vendor deploy/simulation references; see the folder guide for selection and usage. |
| `a3_deploy/` | No (deploy path) | HOPE's deploy line: the native C++ runner, packaging, and the AimRT MuJoCo sim copy the deploy scripts drive (`a3_deploy/A3_MuJoCo_Sim/`). See [docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md). |
| `apps/a3_mujoco_serve/` | No | Self-contained [MuJoCo → DLS IK → CSV → high-level A3 application](apps/a3_mujoco_serve/README.md), including the fully A3-tested PR #18 reference motion and [demo video](apps/a3_mujoco_serve/assets/validated/pr18_a3_serve_demo.mp4). |

## Source URDF

The source package is:

```text
agibot/URDF/A3T2.5-URDF-std-pingpang/
```

This is the racket-equipped A3 variant used by the Isaac starter. For package
contents, the non-racket alternative and software boundaries, see
[the A3 folder guide](agibot/README.md#robot-models-and-software-boundaries).

## Isaac Lab Prepared Copy

On a new workstation, first create the verified `grasping` environment by
following [docs/DISTROBOX_SETUP.md](docs/DISTROBOX_SETUP.md). Run the commands
below inside that Distrobox after deactivating Conda and sourcing
`setup_train_env.sh`; do not use the host `python3` for Isaac asset work.

Isaac Lab loads the prepared asset from:

```text
hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/
```

Generate it with (from `hope_training/whole_body_tracking/` — the script lives in that
package's `scripts/`):

```bash
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi
source setup_train_env.sh
hope_isaac_py scripts/prepare_a3_isaac_asset.py --force
hope_isaac_py scripts/prepare_a3_isaac_asset.py --check
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
and, on the deploy side, by the C++ runner's name-scattered joint map
(`a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_joint_map.hpp`)
plus the SDK joint mapping carried in each export's `deploy.yaml`.
See [docs/POLICY_INTERFACE.md](docs/POLICY_INTERFACE.md) for the full
observation/action contract that consumes it.

Use that order for retargeted CSV columns and policy action/observation
contracts unless you intentionally change the robot configuration.

## Deployment Example

See [the A3 software map](agibot/README.md#robot-models-and-software-boundaries)
for the vendor example, HOPE runner and serving application. Their build and
operational instructions remain in the respective component guides.

## Hip Marker Shell (mocap calibration)

See [pelvis marker hardware](agibot/README.md#pelvis-marker-hardware) for the
original ten-marker shell, the retained 12-station v3 shell and their distinct
calibration data. The shared frame contract is in
[docs/interfaces/frames.md](docs/interfaces/frames.md).

## MuJoCo / AimRT Reference

See [the A3 software map](agibot/README.md#robot-models-and-software-boundaries)
for the vendor simulation reference versus HOPE's closed-loop rehearsal copy.
