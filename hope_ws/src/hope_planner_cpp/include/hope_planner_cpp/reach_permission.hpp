#pragma once

#include <string>

#include "hope_planner_cpp/types.hpp"

namespace hope_planner_cpp {

inline constexpr const char* kDisabledReachPermissionContract =
    "disabled_zero_v1";
inline constexpr const char* kScreenedReachPermissionContract =
    "staged_support_intent_training_v3";
inline constexpr const char* kScreenedTargetTupleContract =
    "fixed_home_staged_support_intent_bank_v3_axis_envelope_"
    "prefilter_not_certification_v1";
inline constexpr const char* kTargetTupleFrameContract =
    "table_world_xy_minus_immutable_session_home_xy_actor_world_z_v1";
// Fail-closed engineering allowance for independent HOME/target sensing; not
// a bank-receipt certification or correlated-support expansion.
inline constexpr double kSessionHomeRelativeXSensorToleranceM = 0.0015;

struct ReachPermissionConfig {
  bool enabled = false;
  std::string contract = kDisabledReachPermissionContract;
  double minimum_tts_s = 0.60;
  double fh_level1_abs_y_m = 0.275;
  double fh_level2_abs_y_m = 0.300;
  double bh_level1_abs_y_m = 0.090;
  double bh_level2_abs_y_m = 0.120;
  double fh_min_relative_y_m = -0.350;
  double fh_max_relative_y_m = -0.250;
  double bh_min_relative_y_m = -0.140;
  double bh_max_relative_y_m = 0.180;
};

struct ReachPermission {
  double reach_level = 0.0;
  // Actual moving foot: Level 1 unloads the target-opposite foot; Level 2
  // permits the target-side foot to make the screened optional microstep.
  double swing_foot_sign = 0.0;
  // False rejects the complete target. It must never be downgraded to the
  // ordinary (0, 0) label while the same OOD racket tuple remains valid.
  bool target_admitted = true;
};

// Throws at startup if an enabled producer could drift from the training-bank
// classifier, or if a disabled producer does not explicitly select zero-only.
void validate_reach_permission_config(const ReachPermissionConfig& config);

// Pure target-tuple classifier.  No plant, contact, foot, load, or outcome
// signal is available to this API. Non-finite/missing input rejects the whole
// target (`target_admitted=false`); it is never downgraded to ordinary (0, 0).
ReachPermission classify_reach_permission(
    const ReachPermissionConfig& config,
    double target_world_y,
    double immutable_session_home_y,
    double swing_sign,
    double producer_tts_s) noexcept;

bool legal_reach_permission(const ReachPermission& permission) noexcept;

bool revision_preserves_reach_geometry_cell(
    const ReachPermission& latched,
    const ReachPermission& candidate) noexcept;

bool target_tuple_inside_axis_prefilter(
    const ReachPermissionConfig& config,
    const Vec3& target_position_actor_world,
    const Vec3& target_velocity_world,
    const Vec3& target_normal_world,
    double immutable_session_home_x,
    double immutable_session_home_y,
    double swing_sign) noexcept;

}  // namespace hope_planner_cpp
