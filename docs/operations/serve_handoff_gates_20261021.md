# Serve-to-policy measured readiness

The segmented Normal Play handoff now allows maximum joint position error
0.20 rad, joint speed 0.60 rad/s, tilt 0.24 rad, and gyro norm 0.40 rad/s.
Each is twice the prior tolerance. The final segment still requires five
consecutive qualifying measurements and retains the one-second timeout.
Kernel Mode keeps its existing return-to-Stand path.

Validation: 60 C++ tests passed across serve controller, command transition,
runner control and segment suites. The new cases accept a measurement outside
all old limits, reject each independently exceeded new limit, and verify the
five-sample requirement. These are source-level tests, not a new physical
balance or handoff qualification. This PR preparation did not deploy or restart
software or execute any robot commands.
