#include "hope_planner_cpp/post_net_one_shot.hpp"

#include <algorithm>
#include <cmath>

namespace hope_planner_cpp {

PostNetOneShot::PostNetOneShot(
    double net_x, double commit_delay_s) noexcept
    : net_x_(std::isfinite(net_x) ? net_x : 1.37),
      commit_delay_s_(std::max(
          0.0, std::isfinite(commit_delay_s) ? commit_delay_s : 0.05)) {}

void PostNetOneShot::reset() noexcept {
  have_previous_ = false;
  previous_ = BallSample{};
  armed_ = true;
  active_ = false;
  committed_ = false;
  net_cross_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
  commit_source_time_s_ = std::numeric_limits<double>::quiet_NaN();
}

PostNetOneShotEvent PostNetOneShot::observe(
    const BallSample& sample) noexcept {
  PostNetOneShotEvent event;
  if (!std::isfinite(sample.source_time_s) ||
      !sample.position.allFinite()) {
    return event;
  }

  if (have_previous_ &&
      sample.source_time_s > previous_.source_time_s) {
    const double previous_x = previous_.position.x();
    const double current_x = sample.position.x();

    // An outgoing crossing re-arms the next opponent return. This is phase
    // bookkeeping only; it never inspects confidence, balance, or freshness.
    if (!armed_ && previous_x <= net_x_ && current_x > net_x_) {
      armed_ = true;
      active_ = false;
      committed_ = false;
    }

    if (armed_ && previous_x > net_x_ && current_x <= net_x_) {
      const double denominator = previous_x - current_x;
      const double fraction = denominator > 1.0e-12
          ? std::clamp((previous_x - net_x_) / denominator, 0.0, 1.0)
          : 1.0;
      net_cross_source_time_s_ = previous_.source_time_s +
          fraction * (sample.source_time_s - previous_.source_time_s);
      commit_source_time_s_ =
          net_cross_source_time_s_ + commit_delay_s_;
      ++flight_sequence_;
      armed_ = false;
      active_ = true;
      committed_ = false;
      event.net_crossed = true;
    }
  }

  previous_ = sample;
  have_previous_ = true;

  if (active_ && !committed_ &&
      std::isfinite(commit_source_time_s_) &&
      sample.source_time_s + 1.0e-12 >= commit_source_time_s_) {
    event.commit_due = true;
  }
  event.flight_sequence = flight_sequence_;
  event.net_cross_source_time_s = net_cross_source_time_s_;
  event.commit_source_time_s = commit_source_time_s_;
  return event;
}

}  // namespace hope_planner_cpp
