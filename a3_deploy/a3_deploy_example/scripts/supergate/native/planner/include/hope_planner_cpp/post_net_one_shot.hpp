#pragma once

#include "hope_planner_cpp/types.hpp"

#include <cstdint>
#include <limits>

namespace hope_planner_cpp {

struct PostNetOneShotEvent {
  bool net_crossed = false;
  bool commit_due = false;
  std::uint64_t flight_sequence = 0;
  double net_cross_source_time_s =
      std::numeric_limits<double>::quiet_NaN();
  double commit_source_time_s =
      std::numeric_limits<double>::quiet_NaN();
};

// A deterministic task-phase trigger, not an admission gate. Every incoming
// (+X to -X) net crossing arms exactly one fixed-time Planner solve. Ball
// samples continue to feed the estimator before and after this trigger.
class PostNetOneShot {
 public:
  PostNetOneShot(double net_x, double commit_delay_s) noexcept;

  void reset() noexcept;
  PostNetOneShotEvent observe(const BallSample& sample) noexcept;
  void mark_committed() noexcept { committed_ = true; }

  bool active() const noexcept { return active_; }
  bool committed() const noexcept { return committed_; }
  std::uint64_t flight_sequence() const noexcept { return flight_sequence_; }
  double net_cross_source_time_s() const noexcept {
    return net_cross_source_time_s_;
  }
  double commit_source_time_s() const noexcept {
    return commit_source_time_s_;
  }
  double commit_delay_s() const noexcept { return commit_delay_s_; }

 private:
  double net_x_ = 1.37;
  double commit_delay_s_ = 0.05;
  bool have_previous_ = false;
  BallSample previous_;
  bool armed_ = true;
  bool active_ = false;
  bool committed_ = false;
  std::uint64_t flight_sequence_ = 0;
  double net_cross_source_time_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double commit_source_time_s_ =
      std::numeric_limits<double>::quiet_NaN();
};

}  // namespace hope_planner_cpp
