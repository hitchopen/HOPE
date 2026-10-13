#pragma once

#include "hope_planner_cpp/types.hpp"

#include <cmath>
#include <limits>

namespace hope_planner_cpp {

struct TargetTurnDecision {
  bool allowed = false;
  double base_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  double target_bearing_deg = std::numeric_limits<double>::quiet_NaN();
  double turn_deg = std::numeric_limits<double>::quiet_NaN();
  const char* reason = "not_evaluated";
};

// This is the horizontal angle to the contact point, not a body-yaw command
// or a racket-normal constraint. Both vectors must use the same world frame.
inline TargetTurnDecision check_target_turn(
    const Vec3& target, const Vec3& base,
    const Eigen::Quaterniond& orientation, bool base_valid,
    double max_turn_deg = 40.0) noexcept {
  TargetTurnDecision result;
  if (!std::isfinite(max_turn_deg) || max_turn_deg <= 0.0 || max_turn_deg > 180.0) {
    result.reason = "target_turn_limit_invalid";
    return result;
  }
  if (!base_valid || !base.allFinite() || !orientation.coeffs().allFinite() ||
      orientation.norm() < 0.5 || orientation.norm() > 1.5) {
    result.reason = "target_turn_base_unavailable";
    return result;
  }
  if (!target.allFinite()) {
    result.reason = "target_turn_position_invalid";
    return result;
  }
  const Vec3 forward = orientation.normalized() * Vec3::UnitX();
  const Vec3 delta = target - base;
  if (forward.head<2>().squaredNorm() < 1.0e-12 ||
      delta.head<2>().squaredNorm() < 1.0e-12) {
    result.reason = "target_turn_direction_undefined";
    return result;
  }
  constexpr double degrees = 180.0 / 3.14159265358979323846;
  const double yaw = std::atan2(forward.y(), forward.x());
  const double bearing = std::atan2(delta.y(), delta.x());
  result.base_yaw_deg = yaw * degrees;
  result.target_bearing_deg = bearing * degrees;
  result.turn_deg = std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw)) * degrees;
  // Include exactly +/-40 degrees; allow only roundoff at the boundary.
  result.allowed = std::abs(result.turn_deg) <= max_turn_deg + 1.0e-12;
  result.reason = result.allowed ? "target_turn_allowed" : "target_turn_exceeds_limit";
  return result;
}

}  // namespace hope_planner_cpp
