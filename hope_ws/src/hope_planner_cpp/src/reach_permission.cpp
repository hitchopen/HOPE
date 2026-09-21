#include "hope_planner_cpp/reach_permission.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace hope_planner_cpp {
namespace {

void require_exact(double configured, double expected, const char* name) {
  if (!std::isfinite(configured) || configured != expected) {
    throw std::invalid_argument(
        std::string(name) + " must exactly match the pinned Schema28 classifier (" +
        std::to_string(expected) + ")");
  }
}

}  // namespace

void validate_reach_permission_config(const ReachPermissionConfig& config) {
  if (!config.enabled) {
    if (config.contract != kDisabledReachPermissionContract) {
      throw std::invalid_argument(
          "disabled reach permission requires contract disabled_zero_v1");
    }
    return;
  }
  if (config.contract != kScreenedReachPermissionContract) {
    throw std::invalid_argument(
        "enabled reach permission requires the screened target-tuple contract");
  }
  require_exact(config.minimum_tts_s, 0.60, "reach_permission_min_tts_s");
  require_exact(
      config.fh_level1_abs_y_m, 0.275,
      "reach_permission_fh_level1_abs_y_m");
  require_exact(
      config.fh_level2_abs_y_m, 0.300,
      "reach_permission_fh_level2_abs_y_m");
  require_exact(
      config.bh_level1_abs_y_m, 0.090,
      "reach_permission_bh_level1_abs_y_m");
  require_exact(
      config.bh_level2_abs_y_m, 0.120,
      "reach_permission_bh_level2_abs_y_m");
  require_exact(
      config.fh_min_relative_y_m, -0.350,
      "reach_permission_fh_min_relative_y_m");
  require_exact(
      config.fh_max_relative_y_m, -0.250,
      "reach_permission_fh_max_relative_y_m");
  require_exact(
      config.bh_min_relative_y_m, -0.140,
      "reach_permission_bh_min_relative_y_m");
  require_exact(
      config.bh_max_relative_y_m, 0.180,
      "reach_permission_bh_max_relative_y_m");
}

ReachPermission classify_reach_permission(
    const ReachPermissionConfig& config,
    double target_world_y,
    double immutable_session_home_y,
    double swing_sign,
    double producer_tts_s) noexcept {
  if (!config.enabled) {
    return {};
  }
  if (!std::isfinite(target_world_y) ||
      !std::isfinite(immutable_session_home_y) || !std::isfinite(swing_sign) ||
      !std::isfinite(producer_tts_s) ||
      (swing_sign != -1.0 && swing_sign != 1.0)) {
    return {0.0, 0.0, false};
  }

  const double relative_y = target_world_y - immutable_session_home_y;
  const bool target_admitted = swing_sign > 0.0
      ? (relative_y >= config.fh_min_relative_y_m &&
         relative_y < config.fh_max_relative_y_m)
      : (relative_y >= config.bh_min_relative_y_m &&
         relative_y <= config.bh_max_relative_y_m);
  if (!target_admitted) {
    return {0.0, 0.0, false};
  }
  if (producer_tts_s < config.minimum_tts_s) {
    return {};
  }
  const double lateral = std::fabs(relative_y);
  const double target_sign = relative_y < 0.0 ? -1.0 : 1.0;
  if (swing_sign > 0.0) {
    if (lateral >= config.fh_level2_abs_y_m) return {2.0, target_sign};
    if (lateral >= config.fh_level1_abs_y_m) return {1.0, -target_sign};
  } else {
    if (lateral >= config.bh_level2_abs_y_m) return {2.0, target_sign};
    if (lateral >= config.bh_level1_abs_y_m) return {1.0, -target_sign};
  }
  return {};
}

bool legal_reach_permission(const ReachPermission& permission) noexcept {
  return permission.target_admitted &&
         ((permission.reach_level == 0.0 &&
           permission.swing_foot_sign == 0.0) ||
          ((permission.reach_level == 1.0 ||
            permission.reach_level == 2.0) &&
           (permission.swing_foot_sign == -1.0 ||
            permission.swing_foot_sign == 1.0)));
}

bool revision_preserves_reach_geometry_cell(
    const ReachPermission& latched,
    const ReachPermission& candidate) noexcept {
  return latched.target_admitted && candidate.target_admitted &&
         latched.reach_level == candidate.reach_level &&
         latched.swing_foot_sign == candidate.swing_foot_sign;
}

bool target_tuple_inside_axis_prefilter(
    const ReachPermissionConfig& config,
    const Vec3& target_position_actor_world,
    const Vec3& target_velocity_world,
    const Vec3& target_normal_world,
    double immutable_session_home_x,
    double immutable_session_home_y,
    double swing_sign) noexcept {
  if (!config.enabled) return true;
  if ((swing_sign != -1.0 && swing_sign != 1.0) ||
      !target_position_actor_world.allFinite() ||
      !target_velocity_world.allFinite() ||
      !target_normal_world.allFinite() ||
      !std::isfinite(immutable_session_home_x) ||
      !std::isfinite(immutable_session_home_y)) {
    return false;
  }
  const bool fh = swing_sign > 0.0;
  const std::array<double, 6> velocity_box = fh
      ? std::array<double, 6>{1.26, 2.25, -0.21, 0.65, 0.62, 1.60}
      : std::array<double, 6>{1.50, 2.17, -0.47, 0.39, 0.70, 1.35};
  const std::array<double, 6> normal_box = fh
      ? std::array<double, 6>{0.74, 0.92, -0.10, 0.29, 0.38, 0.64}
      : std::array<double, 6>{0.78, 0.93, -0.21, 0.18, 0.35, 0.62};
  const double relative_x =
      target_position_actor_world.x() - immutable_session_home_x;
  if (std::fabs(relative_x - 0.58) >
          kSessionHomeRelativeXSensorToleranceM ||
      target_position_actor_world.z() < 0.98 ||
      target_position_actor_world.z() > 1.26) {
    return false;
  }
  const ReachPermission y_permission = classify_reach_permission(
      config, target_position_actor_world.y(), immutable_session_home_y,
      swing_sign, config.minimum_tts_s);
  if (!y_permission.target_admitted) return false;
  for (int axis = 0; axis < 3; ++axis) {
    if (target_velocity_world[axis] < velocity_box[2 * axis] ||
        target_velocity_world[axis] > velocity_box[2 * axis + 1] ||
        target_normal_world[axis] < normal_box[2 * axis] ||
        target_normal_world[axis] > normal_box[2 * axis + 1]) {
      return false;
    }
  }
  return std::fabs(target_normal_world.squaredNorm() - 1.0) <= 2.0e-3;
}

}  // namespace hope_planner_cpp
