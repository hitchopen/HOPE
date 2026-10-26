#pragma once

#include <cmath>
#include <limits>

namespace hope_planner_cpp {

// Frozen venue bounds, in the same metre/table frame as Planner ball input.
struct BallYFilter {
  bool enabled = false;
  double min_y = std::numeric_limits<double>::quiet_NaN();
  double max_y = std::numeric_limits<double>::quiet_NaN();

  bool valid_bounds() const noexcept {
    return std::isfinite(min_y) && std::isfinite(max_y) && min_y < max_y;
  }

  // nullptr means admitted. Never clamp an unsafe contact back into the lane.
  const char* rejection(double observed_y, double contact_y) const noexcept {
    if (!enabled) return nullptr;
    if (!valid_bounds()) return "ball_y_filter_bounds_invalid";
    if (!std::isfinite(observed_y) || !std::isfinite(contact_y))
      return "ball_y_filter_position_invalid";
    if (observed_y < min_y || observed_y > max_y)
      return "ball_y_filter_observed_outside";
    if (contact_y < min_y || contact_y > max_y)
      return "ball_y_filter_contact_outside";
    return nullptr;
  }
};

}  // namespace hope_planner_cpp
