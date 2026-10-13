# Restore local runtime assets

Install [the documented environments](../DISTROBOX_SETUP.md) first. Public runtime
sources live in `a3_deploy/`, robot and mocap assets in `agibot/`, and the OptiTrack
adapter in `NatNet2ROS2/`. See the [asset inventory](../../A3_ASSETS.md).

| Runtime input | Restore or prepare |
| --- | --- |
| Published receiving policy | The tracked `a3_deploy/a3_deploy_example/models/model_21800/policy` contains its ONNX and matching deploy YAML. |
| Supplied test policy | Keep `exported/policy.onnx` and `params/deploy.yaml` from the same export; select the directory explicitly when packaging. |
| Xbox HumanLike assets | The public builder includes `assets/a3_runtime/teleop_humanlike` by default; use an explicit override or `--without-teleop` as appropriate. |
| Rockchip/Thor sysroot | Export the target vendor sysroot with the corresponding script under `a3_deploy/a3_deploy_example/scripts`; these archives are machine-specific and ignored. |
| x86 ONNX Runtime and build dependencies | Run `setup_a3_env.sh` in the deployment project inside `hope`, then use the [build guide](build_and_test.md). |
| Mocap shell and Motive asset | The tracked [17-station release](../../agibot/mocap_sticker_shell_v3/README.md) includes printing files, transforms and `UCB_P1`. |
| Venue addresses and frames | Set current network endpoints and the venue's Motive-to-table transform. Default live localization does not require a pinned asset file or a measured receipt. |
| Session evidence | Record Laptop/HDU/MDU data under a unique session, collect after stop, and retain hashes with the [distributed recording workflow](foxglove_lifecycle.md). |

Generated `build/`, `dist/`, ROS installations, raw logs and experimental policies
remain local. Stage hardware packages in a new slot and select that slot only
while stopped; copying files alone does not start Runner. Follow
[deployment and Gate 3](deploy_and_gate3.md) for the complete sequence.

Training assets and recipes remain covered by the existing
[public Isaac quickstart](../../QUICKSTART_A3_ISAAC.md).
