# Shared HOME and per-flight plane selection

Review/merge order: continuous fitting and 405 ms deadline -> Runner HOME wire
contract -> this Planner/UI consumer. Ball data and ball-selection software are
separate reviews and do not change the reference-frame contract.

Managed normal play now starts Runner with `--shared-session-home`, and the
model21800 hardware config uses schema 4 and `plane_reference_mode: fixed`.
Install Runner, its AimRT topic config/transport relay, Planner, helper and YAML
as one coordinated version. Kernel mode still replaces the normal-play argument
list and does not opt into HOME. Generic Planner defaults remain schema 2.
The Berkeley/main physical parameters are preserved.

Runner captures HOME at policy entry. Planner rejects missing, old or
calibration/world-mismatched receipts. Refresh x_hit is rejected for schema 4;
re-enter policy to capture HOME again. A PASSIVE session has no captured HOME.

The control endpoint exposes `/hope/planner/set_plane_mode` with one atomic
string parameter named `mode`. Generic parameter writes remain disabled. Console
confirms changes by reading fresh diagnostics and distinguishes requested mode
from the last published command:

- `fixed`: HOME X plus the ONNX policy offset (default).
- `per_ball`: current fresh body X plus the same policy offset, sampled once.
- `stable_per_ball`: update only on the Runner's continuous 200 ms waiting/posture
  window; otherwise keep the last committed reference. This gate does not measure
  foot loading.

Changes apply to the next flight. Already published targets and an in-progress
swing are immutable. Dynamic modes require an upgraded Runner capability and
fresh body pose. Restarting Planner restores the YAML default.

Validation on 2026-10-21: Release build passed; all 19 CTest targets passed across
a full run plus the focused runtime-plane rerun. The synthetic runtime-mode
fixture was changed from an unphysical flat low arc to a ballistic arc that stays
above the table across all test planes, avoiding accidental dependence on field
bounce coefficients. Tests cover runtime mode changes, duplicate packets, stale
body, legacy Runner rejection and new HOME policy offsets. Console build and 18
frontend tests passed; 33 helper/assets tests and 520 subtests passed. These are
source/offline checks; no deployment, robot operation or physical qualification
was performed for these PRs.
