#pragma once

#include "hope_planner_cpp/types.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <limits>

namespace hope_planner_cpp {

// Non-recursive robust physical fit.  Every estimate is rebuilt from the
// current raw-position window; no previous state, covariance, Kalman gain, or
// source-gap restart participates in the result.
class BatchPhysicsEstimator {
 public:
  BatchPhysicsEstimator(BallPhysics physics, EstimatorConfig config);

  void reset() noexcept;
  // Start a new incoming net-to-racket flight without discarding the raw
  // history already collected before the net. This clears only contact-epoch
  // bookkeeping so one physical table bounce remains persistent per flight.
  void begin_flight() noexcept;
  void push(const BallSample& sample) noexcept;
  BallState estimate() noexcept;

  std::size_t sample_count() const noexcept { return sample_count_; }
  double sample_span_s() const noexcept;
  bool bounce_detected() const noexcept { return bounce_detected_; }
  bool bounce_transition_active() const noexcept;
  bool bounce_epoch_active() const noexcept { return flight_bounce_seen_; }
  const BallSample* latest_sample() const noexcept;

 private:
  using State6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;

  bool initial_state(State6& state) const noexcept;
  bool simulate_positions(
      const State6& initial,
      std::array<Vec3, kMaxEstimatorSamples>& positions,
      State6* latest_state = nullptr) const noexcept;
  bool propagate(State6& state, double duration_s) const noexcept;
  bool apply_table_bounce(State6& state) const noexcept;
  void append(const BallSample& sample) noexcept;
  void drop_prefix(std::size_t count) noexcept;
  void trim_to_window() noexcept;

  BallPhysics physics_;
  EstimatorConfig config_;
  std::array<BallSample, kMaxEstimatorSamples> samples_{};
  std::size_t sample_count_ = 0;
  static constexpr std::size_t kNoBounceIndex =
      std::numeric_limits<std::size_t>::max();
  // Index of the measured local minimum. State propagation is pre-contact
  // through this sample and applies the velocity jump before the next sample.
  std::size_t bounce_index_ = kNoBounceIndex;
  double last_bounce_source_time_s_ =
      -std::numeric_limits<double>::infinity();
  bool bounce_detected_ = false;
  bool flight_bounce_seen_ = false;
  bool post_bounce_only_ = false;

  // Reused scratch space: seven trajectories = baseline + one forward
  // finite-difference trajectory per state component.
  std::array<std::array<Vec3, kMaxEstimatorSamples>, 7> predicted_{};
};

}  // namespace hope_planner_cpp
