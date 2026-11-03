# Run and understand the V2 prediction simulations

Start with the offline conversion/MuJoCo workflow below. It needs no ROS, GPU, camera, ONNX policy or robot connection. Then use the ROS producer-chain test to exercise delivery, packet formation, expiry and the actual decoder.

[Rendered graphs and downloadable HTML/SVG/CSV](https://github.com/timothyypark/HOPE/tree/evidence/v2-simulation-verification-20261030/visual-guide/plots) show three different questions:

1. **Prediction evolution:** actual V2 waypoints after 100 ms versus 180 ms of observations, compared with independent simulated truth. In the selected measured-spin example, maximum forecast error drops from 0.4744 mm to 0.03967 mm. These are two prefixes of one synthetic flight, not a general convergence guarantee.
2. **Contact accuracy:** predicted contact position/time across all 162 prefixes. There are 159 commands and three justified early abstentions; all 81 physical scenarios have a supported cutoff. Maximum contact error is 0.4933 mm and 0.05491 ms.
3. **Outgoing landing:** the existing racket command is interpreted using its impact law and replayed in the supplied A3 MuJoCo scene. Maximum landing error is 3.6477 mm, against a declared 20 mm limit.

These measurements use noiseless synthetic inputs and pinned physics. Unknown-spin cases use a zero-spin plant with withheld orientation. They do not establish accuracy on arbitrary hidden spin or sensor noise. Good numeric accuracy also does not establish reliable ROS delivery: the recorded campaign still has 899/900 usable incoming commands and unresolved historical losses.

## 1. Get the code and native dependencies

The commands below target Ubuntu 24.04 / Python 3.12 (including Ubuntu in WSL2). Use a Linux container for macOS. The implementation branch is in the fork; it is not merged into upstream `nightly_built`.

```bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev libgtest-dev \
  python3-venv python3-pip libgl1 libglfw3

git clone --branch feature/v2-incoming-validity-continuous-dropin \
  https://github.com/timothyypark/HOPE.git
cd HOPE
export HOPE_REPO="$(pwd)"
export HOPE_RUN="$(dirname "$HOPE_REPO")/hope-sim-run-$(date +%Y%m%d-%H%M%S)"
export HOPE_PKG="$HOPE_REPO/hope_ws/src/hope_planner_cpp"
export HOPE_SCRIPTS="$HOPE_PKG/scripts/flight_prediction_validation"
mkdir -p "$HOPE_RUN"
git rev-parse HEAD > "$HOPE_RUN/source-commit.txt"

python3 -m venv "$HOPE_RUN/venv"
source "$HOPE_RUN/venv/bin/activate"
python -m pip install 'numpy==1.26.4' 'scipy==1.11.4' \
  'matplotlib>=3.8,<4' 'mujoco==3.3.7'
python -m pip freeze > "$HOPE_RUN/python-environment.txt"
```

Keep outputs outside the checkout, as shown. Each simulation command requires a **new** output directory to retain failed runs rather than overwrite them. The supplied model XML and meshes must be present under `a3_deploy/A3_MuJoCo_Sim`; retain the repository layout. PyTorch checkpoints are not used by this workflow.

## 2. Build and run the production-component conversion probe

```bash
cmake -S "$HOPE_PKG/test/native" -B "$HOPE_RUN/native" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOPE_RUN/native" --target hope_incoming_conversion_probe -j2

python "$HOPE_SCRIPTS/incoming_conversion_simulation.py" \
  --probe "$HOPE_RUN/native/hope_incoming_conversion_probe" \
  --output "$HOPE_RUN/conversion" --waypoints
```

Expected reference result: `PASS`, 162 checks, 159 commands, three early uncertain abstentions, 81/81 covered physical scenarios. The probe runs the actual V2 coordinator, adapter, existing racket planner and schema-4/5 decoder. The independent Python plant supplies truth; this is not a plot of the estimator against itself.

Inspect:

| File | Contents |
|---|---|
| `conversion/manifest.json` | Scenario grid, tolerances, input/probe hashes |
| `conversion/observations.txt` | Exact observations supplied to C++ |
| `conversion/probe.csv` | Contact, incoming/racket/outgoing velocity, normal, schema and decoder results |
| `conversion/waypoints.csv` | Actual production forecast points at each source cutoff |
| `conversion/result.json` | Every check, error metric and abstention reason |

The grid covers 5/10/12 m/s, unknown/zero/measured spin, three lateral positions, three selected planes and 100/180 ms observation cutoffs. The probe injects the selected plane/HOME; runtime plane selection is tested separately by ROS. Time cutoffs are measured from the synthetic launch, not from crossing the net. They are unrelated to the 30 ms wall-clock publication/expiry diagnostics.

To build and run all 25 native C++ test groups as well:

```bash
cmake --build "$HOPE_RUN/native" -j2
ctest --test-dir "$HOPE_RUN/native" --output-on-failure
```

## 3. Replay the converted returns in MuJoCo and create graphs

```bash
python "$HOPE_SCRIPTS/mujoco_conversion_replay.py" \
  --repo "$HOPE_REPO" --conversion "$HOPE_RUN/conversion" \
  --output "$HOPE_RUN/mujoco"

python "$HOPE_SCRIPTS/plot_incoming_accuracy.py" \
  --conversion "$HOPE_RUN/conversion" --mujoco "$HOPE_RUN/mujoco" \
  --output "$HOPE_RUN/plots"
```

Expected reference result: 159/159 MuJoCo cases pass. Open `plots/index.html` in a browser; it embeds its images and needs no server. Individual PNG/SVG figures, `errors.csv` and `summary.json` are beside it. The plotter verifies that MuJoCo used the same command receipt as the conversion run. The numerical limits are displayed, not changed by plotting.

This calls the supplied `apps/a3_mujoco_serve/a3_serve/mujoco_scene.py::A3ServeScene.simulate_outgoing_ball`. The robot is pinned and the ball starts at the converted contact, with outgoing velocity independently reconstructed from the actual racket command. It verifies ball flight, net clearance and landing; it does **not** run a dynamic robot swing or resolve the incoming ball hitting a moving racket. The replay records an explicit drag overlay of `0.12861635416914574`, replacing the serve app's different default fit for this comparison.

All three steps run headlessly. A graphical MuJoCo viewer and EGL configuration are unnecessary for these checks. The source's separate serve application can inspect the model and generate its own IK/collision serve replay:

```bash
python -m pip install -e "$HOPE_REPO/apps/a3_mujoco_serve"
hope-a3-serve inspect --config "$HOPE_REPO/apps/a3_mujoco_serve/config/serve.json"
# Optional separate serve experiment; this is NOT the incoming V2 return loop:
hope-a3-serve generate --config "$HOPE_REPO/apps/a3_mujoco_serve/config/serve.json" \
  --output "$HOPE_RUN/standalone-serve"
```

That serve workflow has its own no-spin model, IK and validation contract. Its success must not be counted as validation of an incoming V2-driven racket collision.

## 4. Exercise the ROS chain locally

Install [ROS 2 Jazzy on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) first. Use a separate shell, or deactivate the offline venv before building/testing ROS so Python and ROS packages use the same interpreter. Set the `HOPE_REPO`, `HOPE_RUN`, `HOPE_PKG` and `HOPE_SCRIPTS` variables to the paths used above if opening a new shell.

```bash
deactivate  # only if the offline venv is currently active
sudo apt-get install -y python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-matplotlib python3-pytest python3-yaml libxml2-utils \
  ros-jazzy-ros-base ros-jazzy-eigen3-cmake-module \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest \
  ros-jazzy-ament-lint-common ros-jazzy-diagnostic-msgs \
  ros-jazzy-rosidl-default-generators ros-jazzy-tf2-ros \
  ros-jazzy-rmw-fastrtps-cpp
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd "$HOPE_REPO/hope_ws"
MAKEFLAGS=-j2 colcon --log-base "$HOPE_RUN/ros-log" build \
  --base-paths src ../NatNet2ROS2/src/motion_capture_tracking_interfaces \
  --build-base "$HOPE_RUN/ros-build" --install-base "$HOPE_RUN/ros-install" \
  --packages-select motion_capture_tracking_interfaces hope_msgs hope_planner_cpp hope_bringup \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source "$HOPE_RUN/ros-install/setup.bash"

/usr/bin/python3 "$HOPE_PKG/test/v2_incoming_producer_chain.py" \
  --repo "$HOPE_REPO" --build "$HOPE_RUN/ros-build/hope_planner_cpp" \
  --bringup "$HOPE_RUN/ros-install/hope_bringup/lib/hope_bringup" \
  --rate 150 --mode fixed --spin zero --output "$HOPE_RUN/ros-first-run"
```

This command launches the synthetic camera, real relays, packetizer, planner, HOME fixture and actual C++ decoder. It offers ten independent flights and saves every offered frame, packet, trace and decoded command. Do not launch a second copy of the shadow stack alongside this test. Try `--rate 300` or `360`, `--mode per_ball` or `stable_per_ball`, and `--spin unknown` with a new output directory each time.

While it runs, a second sourced shell on the same ROS domain can inspect:

```bash
ros2 topic echo /hope/shadow/v2_incoming/prediction
ros2 topic echo /hope/shadow/v2_incoming/trajectory
ros2 topic echo /hope/shadow/v2_incoming/command_flat
ros2 topic echo /hope/shadow/v2_incoming/revocation
```

`trajectory` contains full waypoints at up to 50 Hz; it is separate from the one-shot command. Keep receipt/trace files as the accuracy evidence: terminal polling is not a complete lifecycle observer. Preserve failed runs. An empty command stream can mean input registration/HOME is absent, a justified abstention, expiry, or the unresolved delivery defect; read `result.json`, `raw.json`, `planner.tsv` and `relay.tsv` before deciding.

For a broader local run:

```bash
colcon --log-base "$HOPE_RUN/ros-test-log" test \
  --build-base "$HOPE_RUN/ros-build" --install-base "$HOPE_RUN/ros-install" \
  --packages-select hope_msgs hope_planner_cpp hope_bringup --executor sequential
colcon test-result --test-result-base "$HOPE_RUN/ros-build" --verbose
```

The full `verify_integrated.sh` additionally runs sanitizers, historical-source parity, 250 campaign trials and the actual ONNX Runner comparison. It requires a clean full-history checkout, a selected-upstream baseline build for lint comparison, the policy Python dependencies and `ORT_ROOT` pointing to ONNX Runtime's C++ distribution. It clears `PYTHONPATH`; supply MuJoCo dependencies via `HOPE_SIMULATION_PYTHONPATH`. It is not a quickstart substitute for the commands above. See the retained wrapper and input hashes in the evidence branch to reproduce the complete qualification environment.

## 5. Connect your own simulator

The existing offline adapter is the working MuJoCo integration for this milestone. A live MuJoCo-to-ROS bridge and a policy-driven collision loop are additional integration work; launching the planner alone does not create them.

Use this progression:

1. **Coordinate conversion.** The planner's table frame has playing surface `z=0`, `+x` toward the opponent and `y` spanning `[-1.525,0]` for the reference table. The supplied scene uses `p_world = p_table + scene.offset`, where `offset = [near_edge_x_mujoco, table_width/2, table_height]`. Use `scene.to_table()` for the inverse. Read these values from the selected scene config.
2. **Independent observations.** Export MuJoCo ball positions at chosen acquisition times. Feed positions/orientation availability to the estimator, and retain the simulator's unexposed truth separately for scoring. Vary noise, missing frames and spin in new named scenarios; do not reuse the noiseless accuracy claim for those tests.
3. **Offline custom prefixes.** `hope_incoming_conversion_probe` reads a header `case_id plane_mode plane_x sample_count`, then `source_ns x y z orientation_angle_about_y orientation_available` for every sample. It can consume multiple cases. Units are seconds/nanoseconds, metres and radians; plane modes are 0/1/2. This fixture supports Y-axis rotation, so use the typed ROS interface for general quaternion cases. Match its synthetic clock and monotonically increasing sample times; `conversion/observations.txt` is a concrete input example.
4. **ROS transport.** Adapt the tested `source()` in `test/v2_incoming_producer_chain.py` to read simulator samples. It publishes `NamedPoseArrayV2` to `/optitrack/poses`, with `Ball` and `UCB_P1`, and waits for HOME/reference readiness. Retain both the raw packetizer and typed-observation paths; feeding only `/hope/ball/observation_v2` does not exercise the complete managed command path. The launch file starts planner/packetizer only and expects input publishers, relays and HOME/base authority to exist separately.
5. **Clock contract.** The supplied ROS test is paced at real time using wall-clock acquisition timestamps. Bare MuJoCo `data.time` is elapsed simulation time, not that epoch. The tested launch does not establish a `/clock`-driven contract simply by setting `use_sim_time`. Use offline prefixes for faster-than-real-time physics; establish a consistent clock/mapping/expiry design before a paused or accelerated live ROS loop.
6. **Consume the target.** Decode schema 4/5 with the existing receiver. Check validity, source/expiry, HOME/plane, flight identity and revocation before accepting a new target. The flat target is not a joint trajectory; the existing policy/IK/controller still owns the conversion to joint motion. The offline probe exposes the racket normal explicitly for impact reconstruction.
7. **Separate the next validation level.** To prove simulated racket contact, feed the decoded target to that controller, step robot dynamics and ball together, and record actual contact impulse, contact time and landing. The current outgoing-ball replay and stand-state Runner tests do not already supply this proof.

In the recorded probe, decoded target z includes a 0.76 m policy offset; the MuJoCo replay instead starts from table-frame `rx,ry,rz` and adds `scene.offset` once. Applying both offsets to the same point would double-count table height. Follow the provided adapter rather than copying flat positions into the scene without a frame conversion.

## What was checked for this guide

The extraction of the independent Python plant preserves its function body exactly. The new offline run succeeded without ROS Python imports, passed 162/162 checks, and produced command CSV and waypoint CSV byte-identical to the prior retained run. A fresh supplied-scene MuJoCo run passed 159/159. The graph script generated PNG, SVG, a standalone HTML report and CSV metrics from those same receipts. A fresh 150 Hz/fixed-plane/zero-spin ROS producer-chain run passed with ten decoded commands, using the previously built unchanged runtime. Production prediction, command eligibility, expiry and racket-planning code were not changed for these graphs.

These commands were exercised using the existing Ubuntu 24.04/Jazzy validation environment (GCC 13.3, NumPy 1.26.4, SciPy 1.11.4, distribution Matplotlib 3.6.3, MuJoCo 3.3.7). The pip quickstart allows newer Matplotlib for Python 3.12 wheel availability; save its resolved versions as shown. A fresh OS/ROS installation was not performed for this guide.
