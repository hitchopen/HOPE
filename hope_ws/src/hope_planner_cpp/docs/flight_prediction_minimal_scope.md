# Minimal V2 flight scope

This branch starts from hitchopen/HOPE nightly_built `8d599e7d21e196275cf426488555e2648e8fd789`.
It selectively ports physical V2 forecasting from `9118a491df658fa7e68f042581af48890ec8b0b8`.
The source comparison contains 42 commits and 113 files; those historical counts do not describe this branch.

Raw qualified observation → ShotContext → six-state CKF + TrustedSpin → shared forward kernel → nominal physical events → registered +X plane → V2 prediction/trajectory/packet.

V2 does not select landing points, forehand/backhand, racket normals or velocities; it does not pack or publish commands, change scheduling or execute robot motion. Observed outgoing balls and caller-supplied hypothetical *forward* trajectories are supported. Inverse shot selection is outside scope.

Legacy production Stage 2, distinct Nakashima/venue modes, Stage 3, estimator, scheduler, planner, packetizer, replay, defaults and retired Python implementation retain upstream bytes. Strict V2 angular alias checks live in FlightSpinEstimator, not the original SpinEstimator.
BallSample appends V2 provenance fields; EffectiveSpin is a separate type. Existing field order/defaults and command definitions are retained, but **dependent C++ packages must be rebuilt**; binary ABI compatibility with older objects is not claimed.

See the complete changed-file/source inventory and excluded-file register. Synthetic registration/tests establish software behavior, not live clock/frame qualification or physical accuracy.
