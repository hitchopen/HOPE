# Flight prediction validation receipt

Final clean validation is pending. This receipt is updated only from executed
commands and test results. Physical prediction accuracy requires independent
recordings and is NOT RUN. Production adoption is NOT ATTEMPTED; the pipeline
remains shadow-only.

Reproduce the complete build and test gate from a clean shell in the HOPE distrobox:

```bash
source /opt/ros/jazzy/setup.bash
hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify.sh /tmp/hope-v2-clean-validation
```

The mocap interfaces are built from this repository, so relay tests do not depend
on an old overlay. The complete gate intentionally retains existing bringup lint
failure status. Focused ROS/numerical/transport tests are registered in normal
CTest collection. The optional native CMake entry point under `test/native` tests
the same libraries without generated ROS messages; it is supplementary evidence.
