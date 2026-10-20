# Reproduce validation

Use a clean public HOPE checkout, Ubuntu 24.04 / ROS 2 Jazzy, GCC 13.3, CMake 3.28, Python 3.12, Eigen3, GTest, SciPy, NumPy, Matplotlib, colcon and standard ament test/lint packages. No private nohope source or stale overlay is needed. Current run uses ros:jazzy-ros-base digest sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410.

From repository root:

```bash
bash hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify.sh /tmp/hope-v2-review
source /tmp/hope-v2-review/install/setup.bash
python3 hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/production_parity.py --repo "$PWD" --output /tmp/production-parity.json
python3 hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/benchmark_cadence.py --executable /tmp/hope-v2-review/install/hope_planner_cpp/lib/hope_planner_cpp/hope_flight_prediction_v2_node --smoke hope_ws/src/hope_planner_cpp/test/flight_prediction_v2_smoke.py --output /tmp/cadence
cmake -S hope_ws/src/hope_planner_cpp/test/native -B /tmp/hope-v2-sanitize -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='-O1 -g1 -fsanitize=address,undefined -fno-omit-frame-pointer' -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build /tmp/hope-v2-sanitize -j2
ASAN_OPTIONS=detect_leaks=1 ctest --test-dir /tmp/hope-v2-sanitize --output-on-failure
```

`verify.sh` preserves all test failures, including inherited lint debt. It records process exit statuses, pins and clean package-specific builds; it does not suppress assertions or import prior branch artifacts. Exact current-run receipts and paths are in flight_prediction_acceptance.json. Optional 50/100/150/latest full-stream experiments retain their individual results. Compact eligibility/coverage, source age, receipt quantiles and full-stream rates are reported separately.

If download.ros.org XML schema lookup is blocked, use an XML catalog pointing to the official ros-infrastructure/rep xsd/package_format3.xsd and adjacent package_common.xsd at the recorded upstream Git commit; retain XML validation. Never disable TLS/signature/schema checks to obtain a green result.

Raw current-instance logs live in /workspace/.cloud-setup/v2-evidence. The baseline uses a separate upstream archive/build/install. Receipt paths distinguish inherited failures, initial incomplete-fixture/setup failures and corrected final runs. Physical accuracy stays NOT_RUN without independently annotated held-out trajectories and qualified clock/frame/plane receipts.

Current fresh output root is `/workspace/.cloud-setup/v2-evidence/fixed-final`; current full-suite log is `independent-final-all-tests.log`, load logs are `cadence-fixed-final/`, sanitizer log is `sanitize-fixed.log`. Runtime build pin and later fixture-only validation pin are recorded separately. The default ROS scenario group includes all eight direction/rate/plane-policy cases using an independent source process. Raw `gap-regression-before.log` and `gap-regression-after.log` preserve the deterministic failure/fix. Use `--gtest_filter=FlightV2Motion.*SourceGap*` on the native test binary to isolate it.
