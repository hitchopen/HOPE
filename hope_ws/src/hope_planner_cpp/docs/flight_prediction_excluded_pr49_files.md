# Historical PR49 files excluded

All 24 historical paths below remain on their original source branches. No equivalent selector/command runtime is copied under another name. The original production tree is retained.

- `hope_ws/src/hope_msgs/msg/BallForecast.msg`
- `hope_ws/src/hope_msgs/msg/BallForecastPhysicsEvent.msg`
- `hope_ws/src/hope_msgs/msg/BallForecastWaypoint.msg`
- `hope_ws/src/hope_msgs/msg/HybridTrajectoryContext.msg`
- `hope_ws/src/hope_msgs/msg/HybridTrajectoryIdentity.msg`
- `hope_ws/src/hope_planner_cpp/config/ckf_strike_shadow.yaml`
- `hope_ws/src/hope_planner_cpp/docs/ckf_strike_shadow.md`
- `hope_ws/src/hope_planner_cpp/include/hope_planner_cpp/ball_flight_state_machine.hpp`
- `hope_ws/src/hope_planner_cpp/include/hope_planner_cpp/hybrid_trajectory_coordinator.hpp`
- `hope_ws/src/hope_planner_cpp/include/hope_planner_cpp/hybrid_trajectory_node.hpp`
- `hope_ws/src/hope_planner_cpp/include/hope_planner_cpp/strike_flight_detector.hpp`
- `hope_ws/src/hope_planner_cpp/include/hope_planner_cpp/strike_target_selector.hpp`
- `hope_ws/src/hope_planner_cpp/launch/ckf_strike_shadow.launch.py`
- `hope_ws/src/hope_planner_cpp/src/ball_flight_state_machine.cpp`
- `hope_ws/src/hope_planner_cpp/src/hybrid_trajectory_coordinator.cpp`
- `hope_ws/src/hope_planner_cpp/src/hybrid_trajectory_main.cpp`
- `hope_ws/src/hope_planner_cpp/src/hybrid_trajectory_node.cpp`
- `hope_ws/src/hope_planner_cpp/src/strike_flight_detector.cpp`
- `hope_ws/src/hope_planner_cpp/src/strike_target_selector.cpp`
- `hope_ws/src/hope_planner_cpp/src/strike_target_selector_node.cpp`
- `hope_ws/src/hope_planner_cpp/test/ckf_strike_shadow_smoke.py`
- `hope_ws/src/hope_planner_cpp/test/test_shadow_runner_input.cpp`
- `hope_ws/src/hope_planner_cpp/test/test_strike_flight_detector.cpp`
- `hope_ws/src/hope_planner_cpp/test/test_strike_target_selector.cpp`

V2 uses ShotContext/FlightPipeline, six Flight*V2/BallObservationV2 physical interfaces and its independent graph. No legacy hybrid or strike-selector targets/messages/tests are registered. Only return_region physical side/history/config hashes remain as individually inventoried shared helpers. Runtime source, linked binaries/libraries, ROS publication/remap fences and the complete upstream diff have separate gates.
