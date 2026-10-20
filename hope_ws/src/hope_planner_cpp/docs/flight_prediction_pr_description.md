# Add isolated V2 ball-flight forecasting and shared forward physics (no racket planning)

Adds a shadow-only physical ball forecast to hitchopen/HOPE:nightly_built, selectively ported from the scoped V2 source. The historical PR49 hybrid forecaster, strike detector/selector, messages and command-preview tests are excluded. Production Stage 2, all five physics modes, Stage 3 and scheduling preserve upstream behavior.

The new graph is observation → shot lifecycle/CKF/trusted spin → forward physics/events → registered plane → versioned physical prediction/trajectory/checkpoint. It never chooses a racket action or changes a production command path. Six physical interfaces add exact source/arrival times, optional qualified spin and conditional uncertainty. Public C++ sample structs grow; rebuild dependent packages.

Review flight_prediction_minimal_scope_manifest.json for every file, flight_prediction_excluded_pr49_files.md for omissions, and flight_prediction_acceptance.json for current code/config pins and executed gates. This document is a ready-to-post description; no PR is created by the implementation task.

Validation and limitations: see the current acceptance receipt for exact counts rather than historical 33/33 or 50-case claims. Strict production parity covers 18 racket fixtures, 540 cases spanning all five predictor modes and 140 spin/batch samples. Real-ball accuracy is NOT_RUN. Legacy-method comparison supports only registered fixed-plane sessions; historical APIs do not export bounce/apex estimates. ROS synthetic load is a software/resource test, not qualification for live deployment.

Reproduce using flight_prediction_validation.md. No merge, robot execution or physical-accuracy/adoption claim is part of this change.
