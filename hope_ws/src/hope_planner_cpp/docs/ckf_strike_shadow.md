# CKF strike shadow integration

This optional pipeline adds continuous CKF ball forecasts and one best-effort
racket decision per detected incoming flight. Public base:
`6616b1df795b6b5db242435fafc394295e0b45cf` (`nightly_built`).
The existing production planner, packetizer, adaptive scheduler, physics algorithms,
messages, policy and runner are unchanged. Launching this pipeline starts only the
two new ROS nodes; it does not start hardware or a runner.

`/poses` feeds the CKF and an independent X-only flight detector. The CKF publishes
compact `/ball/forecast/v1/event` and full `/ball/forecast/v1/trajectory`
`BallForecast` messages. A measured negative-X crossing of the fixed net at
1.37 m establishes the integer acquisition-time cutoff: net time + 50 ms.
A 1-ms timer commits after a 10-ms receive grace, even without further poses.
Only observations/forecasts sourced at or before that cutoff can be used.

Selection keeps the newest useful current/retained CKF crossing, then tries a
full CKF state rollout, the existing public batch estimator/predictor, and a sparse
observation fit. It calls public `RacketTargetPlanner` and `Schema2Packer` unchanged.
The selector publishes exactly one typed decision per detected flight on
`/planner/shadow_racket_command`; inability to construct a finite future target
produces `valid=false`. Optional `/planner/shadow_racket_command_flat` carries
valid schema-2 packets only. Output remapping is rejected. Neither command output
can be redirected to production topics. Racket position receives the existing
+0.76-m policy Z offset once; velocities, ball state and normals do not.

Copy `config/ckf_strike_shadow.yaml` to a session file. Confirm the acquisition
clock, pose index, table-frame registration ID and base world ID in both nodes.
Use ROS system time for flat wall deadlines. Provide registered schema-2 base
samples on `/a3/base_pose_flat`. At an idle boundary, call the forecaster's
`/hope_hybrid_trajectory_node/freeze_x_hit` Trigger service to freeze
`x_hit = base_x + 0.58 m`; subsequent base movement does not move that plane.
A known test plane is exclusively synthetic evidence. If both forecast streams
are absent, observation fallback requires explicitly confirming the same frozen
plane and its generation/world/calibration in the selector's fallback parameters.
Side selection needs accepted same-session base Y or an explicit `nominal_base_y_m`.

The selector requires finite `hit_y_min_table_m < hit_y_max_table_m`; missing,
partial or nonfinite bounds refuse startup. Supply team-approved **table-frame
incoming-contact** Y limits, not landing-target limits. Both endpoints are inclusive.
Out-of-range contact produces one invalid typed decision with
`reason=hit_y_out_of_range` diagnostics, no Stage-3 call and no flat packet or
sequence increment. It does not halt the session or suppress compact/full forecasts;
the next independent flight can still qualify. Bounds stay fixed as the base moves.
Shipped YAML leaves both values NaN. All test ranges are synthetic, not deployment
recommendations; approved real bounds and randomized base-start envelope are pending.

Optional `/ball/hybrid/context` uses `HybridTrajectoryContext`. Configure the
selector's `context_evidence_source_id` for authoritative RESET/history evidence.
Unknown history, stale estimates, missing uncertainty and short positive lead
are reported as shadow quality limitations. Frame, clock, ownership and plane
faults fail closed with an invalid decision for a pending flight. A registered
explicit RESET is required after continuity faults. A BEGIN alone does not rearm
physical selection; observed outgoing motion on the opponent side followed by an
incoming return does. History labels are advisory, not verified legal eligibility.

```bash
source /opt/ros/jazzy/setup.bash
cd hope_ws
colcon build --packages-up-to hope_planner_cpp --executor sequential
source install/setup.bash
ros2 launch hope_planner_cpp ckf_strike_shadow.launch.py config:=/absolute/session.yaml
```

Run the four focused checks, including the generated ROS smoke and actual parser:

```bash
ctest --test-dir build/hope_planner_cpp --output-on-failure \
  -R 'test_strike_flight_detector|test_strike_target_selector|test_shadow_runner_input|ckf_strike_shadow_ros'
colcon test --packages-select hope_planner_cpp --event-handlers console_direct+
colcon test-result --test-result-base build/hope_planner_cpp --verbose
```

The native test uses this checkout's `pp_planner_input.hpp` and pure lifecycle
helpers; only that test target needs C++20. The production planner remains C++17.
The smoke generates tiny synthetic observations at runtime, stops immediately
after the net straddle, checks compact/full forecasts, exact typed/flat values,
one-shot identity, parser acceptance and absence of production publishers.
No private repository or saved dataset is required.

The CKF retains its original explicit drag/restitution/grip coefficients in the
new node/config. Public batch fallback and Stage 3 use current public defaults;
these differ from the historical private defaults. No coefficient was retuned.

This is shadow compatibility evidence. Real table/world/base/actor registration
is unverified; a `table` header is not proof of the runner's physical `world` frame.
Retained/sparse estimates can be inaccurate, and a positive deadline does not
qualify policy clip timing or reachability. Current public receivers reject
non-increasing producer sequences, including rollback after restart: coordinate
producer/runner session lifecycle before reconnecting. Hardware, policy execution
and full C++ ONNX integration are untested. No production integration is enabled.

Sweet-spot interface confirmation from Tien-Chun/the policy owner is pending:
paddle center versus contact/attachment reference; actual table/world registration
and single Z offset; normal direction and velocity frame/units; any required
geometric adapter. Current code preserves Stage 3's contact-position target,
unchanged normal/velocity, and the single outgoing-position Z offset. These tests
prove existing adapter behavior, not compatibility with a revised reward. No new
transform or reward is inferred. The deployment owner must freeze after the base
settles and before BEGIN; production planner Y enforcement is a separate handoff.
