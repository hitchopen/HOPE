#pragma once

#include <cmath>
#include <limits>
#include <utility>

namespace hope_planner_cpp {

// Process startup is not the playing-session boundary.  This tiny state object intentionally has
// only one mutation API whose name encodes the lifecycle: the first finite solve-time base XY is
// immutable for swing-side, support-intent and complete target-tuple classification.
struct SessionHomeReference {
  bool set = false;
  double x = 0.0;
  double y = 0.0;

  // An explicit playing-session boundary may be established before the first
  // ball solve.  The caller owns the lifecycle guard that forbids rewriting
  // HOME after a flight has entered the Planner.
  bool freeze_xy_at_session_boundary(
      double session_home_x, double session_home_y) noexcept {
    if (!std::isfinite(session_home_x) || !std::isfinite(session_home_y)) {
      return false;
    }
    x = session_home_x;
    y = session_home_y;
    set = true;
    return true;
  }

  std::pair<double, double> resolve_xy_on_solve(
      double live_base_x, double live_base_y) noexcept {
    if (!set && std::isfinite(live_base_x) && std::isfinite(live_base_y)) {
      x = live_base_x;
      y = live_base_y;
      set = true;
    }
    return set ? std::pair{x, y} : std::pair{live_base_x, live_base_y};
  }
};

// Pure Schmitt selector shared by the node and its lifecycle/venue tests.  A
// missing HOME is not a geometric origin: it must fail closed rather than
// silently classifying a world-Y lane relative to zero.
inline double select_swing_sign_with_hysteresis(
    double intercept_y,
    double immutable_home_y,
    double split_y,
    double hysteresis_y,
    double previous_sign) noexcept {
  if (!std::isfinite(intercept_y) || !std::isfinite(immutable_home_y) ||
      !std::isfinite(split_y) || !std::isfinite(hysteresis_y) ||
      hysteresis_y < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double relative_y = intercept_y - immutable_home_y;
  const double low = split_y - hysteresis_y;
  const double high = split_y + hysteresis_y;
  if (previous_sign > 0.5) {
    return relative_y > high ? -1.0 : 1.0;
  }
  if (previous_sign < -0.5) {
    return relative_y < low ? 1.0 : -1.0;
  }
  return relative_y < split_y ? 1.0 : -1.0;
}

}  // namespace hope_planner_cpp
