# Runner-owned session HOME contract

This change is opt-in through `--shared-session-home` and requires a calibrated
110-D ball-clock policy, external base localization, and normal Planner mode.
Legacy Runner behavior remains the default. This is the producer/wire-contract
batch; the Planner consumer and plane UI are submitted separately.

Runner captures HOME at policy entry from measured base position, calibration ID
and world-frame ID. Re-entering policy captures a new epoch. Ordinary body drift
and a mode change never redefine HOME. A fresh receipt heartbeat is published at
`/hope/runner/session_home_flat` and relayed, without timestamp substitution, to
`/hope/runner/session_home_hdu_flat`. Install the matching AimRT topic config.

Schema 4 checks the HOME epoch and fixed plane. Schema 5 additionally freezes the
per-flight reference/mode. Old epochs, wrong frames/planes and same-flight
retargeting are rejected. A posture-stable heartbeat requires a continuous fresh
waiting window. Receipt capability is not hardware qualification.

Offline validation on 2026-10-21: 16 wire/receipt tests passed; the native ONNX
policy probe passed capture, re-entry, drift, stale HOME, wrong plane and immutable
dynamic-flight tests using the historical model21800 policy snapshot. Both Runner
main and AimRT backend passed C++20 syntax checks with ROS/AimRT enabled. No
backend was started and no actuator command was published by the probe.

To reproduce the standalone tests, compile `unit_tests/test_session_home_contract.cpp`
and `unit_tests/test_pp_racket_target_input.cpp` with C++20, Eigen and gtest. Compile
`include/a3_pingpong/test/pp_shared_home_test.cpp` with Eigen, ONNX Runtime and
yaml-cpp, then pass the compatible `policy.onnx` and `params/deploy.yaml` as its
two arguments. Do not use a different policy ABI as an acceptance substitute.
