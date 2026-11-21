# model_29999: final StationaryRally GRU actor

[Download self-contained actor ZIP](model_29999_gru_bundle.zip) · [ONNX](policy/exported/actor.onnx) · [Manifest](policy/exported/manifest.json) · [Normalization/joints/actions/timing](policy/params/runtime.json) · [Final unchanged video](../../../../docs/assets/gru-formal-2b8c09938a/model_29999.mp4) · [Eight-model results](../../../../docs/evaluations/gru-formal-2b8c09938a/README.md)

Run: `gru-formal-2b8c09938a`; final iteration29999. Training source: `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.

- Original checkpoint SHA256: `6e671c56d1de5ead2a7553f489bf346bb372957d87f0bcc30854e2931e472c88`.
- Original ONNX SHA256: `c361831725a4fd42661a7b6f46c738000120e248bae06e28f9e05195da3e96d3`.
- Actor parameter fingerprint: `14991289489893892f1e655ac253911b0cce0f11ee273374867e499aa27f721b`.

The221MB PT checkpoint remains in the local comparison snapshot and original training archive: it exceeds GitHub's100MiB ordinary-file limit and is unnecessary for actor inference. No alternate actor or model21800/29500 is substituted. ONNX bytes and learned normalization remain unchanged. Full original training-configuration hashes are retained; only portable runtime settings are distributed. No credentials or private logging configuration are included.

## Standalone actor use

The ZIP includes actor.onnx, manifest, parity vectors, minimal runtime Python modules, pinned dependency lists and configuration. It does not include a Python environment. The observed verified environment is Python3.12, NumPy2.5.4, ONNX Runtime1.31.1; MuJoCo3.17.0 and PyYAML6.0.3 are needed only for evaluation. FFmpeg/libass provides video playback/rendering. Use an existing compatible environment; on a new machine the dependency lists document what to provision separately.

After unpacking the ZIP, verify without Torch, ROS or this repository:

```bash
python policy/exported/verify.py
```

From the repository root:

```bash
python a3_deploy/a3_deploy_example/models/model_29999/policy/exported/verify.py
```

Inputs are `obs:float32[1,110]` and `h_in:float32[1,1,110]`; outputs `actions:float32[1,31]` and `h_out:float32[1,1,110]`. Observation normalization is **identity**, verified from saved config/state; the graph owns normalization and passive-head masking. The manifest/runtime JSON specify every joint, observation slice, affine action scale/offset/clamp, delay, nominal actuator parameter and20ms control timing. Physical evaluation uses20×1ms steps, while the saved training metadata remains4×5ms; this explicit evaluation override is documented rather than hidden.

Zero hidden state once at episode start; feed h_out into the next invocation across every serve/handoff/recovery step. Do not initialize from training-batch memory or reset per ball. Reset only at a genuine new episode; this diagnostic ends on a fall instead of resetting. There is no actor FH/BH input: the actual planner's continuous strike position/velocity/deadline feed the existing110-D observation.

## MuJoCo visualization and100-ball diagnostic

Run these commands from a clone of this feature branch's repository root. Set `HOPE_PYTHON` to an existing host MuJoCo Python and `HOPE_PLANNER_BINARY` / `HOPE_ROS_SETUP` to an existing compatible ROS planner build. Defaults below are repository-relative install paths. No automatic package installation, build, hardware launch or training is performed.

```bash
export HOPE_PYTHON="${HOPE_PYTHON:-python3}"
export HOPE_PLANNER_BINARY="${HOPE_PLANNER_BINARY:-$PWD/hope_ws/install/hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node}"
export HOPE_ROS_SETUP="${HOPE_ROS_SETUP:-$PWD/hope_ws/install/setup.bash}"
```

Visible bounded100-ball evaluation:

```bash
"$HOPE_PYTHON" hope_training/whole_body_tracking/scripts/run_model29999_mujoco.py --view --record-video
```

Headless100-ball diagnostic, rendering actual recorded states only after the planner exits:

```bash
"$HOPE_PYTHON" hope_training/whole_body_tracking/scripts/run_model29999_mujoco.py --record-video
```

Playback of the original published final video, skipping the20s idle:

```bash
"$HOPE_PYTHON" hope_training/whole_body_tracking/scripts/run_model29999_mujoco.py --replay
```

The ROS planner is launched in the existing `hope` Distrobox, localhost-only domain187 with a unique namespace and a graph guard. Use `--ros-box NAME` or `--ros-host` / `--ros-domain` for a documented environment override. Only simulated ball/base poses reach that planner; no hardware command bridge is launched. Outputs go to a fresh `~/hope-evaluations/model29999-*` directory; `--output-root` overrides it. A fall exits the rally and records unlaunched serves; an exit code1 may therefore describe a recorded policy fall, not100 completed returns. Inspect summary/error fields.

A new machine needs an existing host environment with `requirements-mujoco.txt`, FFmpeg, and a ROS2 Jazzy environment with built hope_msgs and hope_planner_cpp. Existing validated builds should be reused. If a build does not exist, the repository's standard isolated ROS build is:

```bash
distrobox enter hope -- bash -lc 'source /opt/ros/jazzy/setup.bash; colcon --log-base "$PWD/hope_ws/log" build --base-paths "$PWD/hope_ws/src/hope_msgs" "$PWD/hope_ws/src/hope_planner_cpp" --build-base "$PWD/hope_ws/build" --install-base "$PWD/hope_ws/install"'
```

This is a prerequisite command for a new installation, not something this publication task ran. See [Distrobox setup](../../../../docs/DISTROBOX_SETUP.md) for provisioning. Missing environments/builds fail clearly; no old policy is loaded. The native two-file/hardware runner does not implement this recurrent hidden-state ABI and must not be pointed at this actor as a feed-forward policy.

## Observed result and limits

Final original attempt `attempt-p0vi0jjj`:100 launched/resolved,100 raw-contact balls,0 qualified contacts,0 reference legal landings,0 falls/resets,0 unlaunched/unresolved, longest success streak0. `COMPLETE_100` means completion, not100 successful returns. The20s initial idle is preserved; first serve is00:20. All eight videos are unchanged, not recompressed.

The runtime uses the corrected PPQ004 table-impact profile SHA256 `df55e82102e0d5624732ea7ea03f6d98053f45d7f036367e3df3551a1c88a6f9`, actual MuJoCo paddle mesh collisions, concurrent outgoing balls and the audited fixed-home-100 timing. Table response is1ms quantized without pose snapping. Paddle impulses differ from native fitted impacts. Training used virtual at-strike legal-return calculations, not these measured contacts and legal landings. Host recovery completion remains a deadline+.5s proxy rather than native lifecycle. This is a **reference-sequence diagnostic, not formal Gate3**, and not evidence of successful real-robot deployment.

## Existing validated lab workspace (no rebuild)

For the recorded sibling-checkout layout, these copy-paste settings reuse the existing host venv and ROS build. The venv directory name does not change this repository's HOPE fork identity. Other installations should set these same configurable variables to their own existing paths.

```bash
export HOPE_PYTHON="$PWD/../nohope/.venv/bin/python"
export HOPE_ROS_SETUP="$PWD/../hope-stationary-gru-evaluations/20261105-221919-fixed-home-gru-diagnostic/ros-install/setup.bash"
export HOPE_PLANNER_BINARY="$(dirname "$HOPE_ROS_SETUP")/hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node"
"$HOPE_PYTHON" hope_training/whole_body_tracking/scripts/run_model29999_mujoco.py --view --record-video
```

For headless100-ball evaluation, run the last command without `--view`. These paths are configurable workspace-relative locations, not required personal home paths. Full initial idle and no-rescue fall behavior remain unchanged.
