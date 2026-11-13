#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace hope_planner_cpp {

// Runner publishes [version, valid, home_x, home_y] from its MOTION anchor.
inline double runner_home_y_from_flat(const std::vector<double>& values) noexcept {
  if (values.size() != 4 || values[0] != 1.0 || values[1] != 1.0 ||
      !std::isfinite(values[2]) || !std::isfinite(values[3]))
    return std::numeric_limits<double>::quiet_NaN();
  return values[3];
}

// Select a supported interception plane; never enlarge the actor's strict box.
// The small inward rounding margin avoids a world->Home subtraction crossing
// the upper bound again. An already supported interior plane is unchanged.
inline double supported_home_x_hit(double requested, double home_x,
                                   const std::vector<double>& support) noexcept {
  if (support.empty()) return requested;
  if (support.size() != 2 || !std::isfinite(requested) || !std::isfinite(home_x) ||
      !std::isfinite(support[0]) || !std::isfinite(support[1]) ||
      support[1] - support[0] <= 2e-6)
    return std::numeric_limits<double>::quiet_NaN();
  const double relative = requested - home_x;
  if (relative >= support[0] + 1e-6 && relative <= support[1] - 1e-6)
    return requested;
  return home_x + std::clamp(relative, support[0] + 1e-6, support[1] - 1e-6);
}

inline bool valid_swing_side_support_ranges(
    const std::vector<double>& forehand, const std::vector<double>& backhand) noexcept {
  if (forehand.empty() && backhand.empty()) return true;
  const auto valid = [](const std::vector<double>& band) {
    return band.size() == 2 && std::isfinite(band[0]) && std::isfinite(band[1]) &&
           band[0] <= band[1];
  };
  return valid(forehand) && valid(backhand);
}

// Bounds and intercept must use the same reference frame. A side which cannot
// reach the target must never be retained merely because of hysteresis. Empty
// bands preserve the legacy selector for policies without a support contract.
inline double select_swing_sign_with_support(
    double intercept_y, double reference_y, double split_y, double hysteresis_y,
    double previous_sign, const std::vector<double>& forehand,
    const std::vector<double>& backhand) noexcept {
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(intercept_y) || !std::isfinite(reference_y) ||
      !std::isfinite(split_y) || !std::isfinite(hysteresis_y) || hysteresis_y < 0.0 ||
      !valid_swing_side_support_ranges(forehand, backhand)) return invalid;
  const double relative_y = intercept_y - reference_y;
  if (!forehand.empty()) {
    const bool fh = relative_y >= forehand[0] && relative_y <= forehand[1];
    const bool bh = relative_y >= backhand[0] && relative_y <= backhand[1];
    if (!fh && !bh) return invalid;
    if (fh != bh) return fh ? 1.0 : -1.0;
  }
  if (hysteresis_y == 0.0) return relative_y < split_y ? 1.0 : -1.0;
  if (previous_sign > 0.5) return relative_y > split_y + hysteresis_y ? -1.0 : 1.0;
  if (previous_sign < -0.5) return relative_y < split_y - hysteresis_y ? 1.0 : -1.0;
  return relative_y < split_y ? 1.0 : -1.0;
}

}  // namespace hope_planner_cpp
