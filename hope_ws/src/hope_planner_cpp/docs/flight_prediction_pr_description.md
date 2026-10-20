# Add isolated V2 ball-flight forecasting and shared forward physics (no racket planning)

Adds a shadow-only physical ball forecast to hitchopen/HOPE:nightly_built, selectively ported from the scoped V2 source. The historical PR49 hybrid forecaster, strike detector/selector, messages and command-preview tests are excluded. Production Stage 2, all five physics modes, Stage 3 and scheduling preserve upstream behavior.

The new graph is observation → shot lifecycle/CKF/trusted spin → forward physics/events → registered plane → versioned physical prediction/trajectory/checkpoint. It never chooses a racket action or changes a production command path. Six physical interfaces add exact source/arrival times, optional qualified spin and conditional uncertainty. Public C++ sample structs grow; rebuild dependent packages.

Review flight_prediction_minimal_scope_manifest.json for every file, flight_prediction_excluded_pr49_files.md for omissions, and flight_prediction_acceptance.json for current code/config pins and executed gates. This document is a ready-to-post description; no PR is created by the implementation task.

Validation and limitations: see the current acceptance receipt for exact counts rather than historical 33/33 or 50-case claims. Strict production parity covers 18 racket fixtures, 540 cases spanning all five predictor modes and 140 spin/batch samples. Real-ball accuracy is NOT_RUN. Legacy-method comparison supports only registered fixed-plane sessions; historical APIs do not export bounce/apex estimates. ROS synthetic load is a software/resource test, not qualification for live deployment.

Reproduce using flight_prediction_validation.md. No merge, robot execution or physical-accuracy/adoption claim is part of this change.

## Current review pins and gates

Upstream base: `8d599e7d21e196275cf426488555e2648e8fd789`. Source: `9118a491df658fa7e68f042581af48890ec8b0b8`. Tested code: `dcd17786f5b32afa56da63713c5cd32c3dffdd78`; following review commits change documentation/evidence only. Branch: `feature/flight-state-prediction-v2-minimal`.

Clean build passed; planner 30/30 groups, messages 2/2, native sanitizers 13/13, and all 50 scoped software IDs pass. Bringup is 24/26 with exactly the upstream flake8/pep257 issue multiset; one unsupported standalone replay mode is skipped on both branches. Production parity is exact across 701 fixtures/samples. The 50 Hz full-output profile passes at nominal 150/300 Hz (100.0%/99.14% compact coverage, receipt P95 <2 ms). Unrestricted full output at 300 Hz fails coverage (91.52%) and is optional. Gate P remains NOT_RUN.

PR #49 is closed/unmerged at 84fc446 and its history is not the base of this branch. No existing source branch was rewritten and nohope is unchanged. See the adjacent manifest/engineering report and committed evidence for the final file inventory and all limitations.
