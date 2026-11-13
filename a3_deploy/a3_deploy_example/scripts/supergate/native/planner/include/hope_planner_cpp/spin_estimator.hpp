#pragma once

#include "hope_planner_cpp/types.hpp"

#include <array>
#include <cstddef>

namespace hope_planner_cpp {

// Causal angular-velocity estimator for the OptiTrack ball quaternion.
//
// Motive can change a rigid body's marker assignment for a single frame.  A
// marker relock produces one impossible quaternion delta but does not make the
// following deltas unusable: a fixed body-frame orientation offset cancels in
// q_k * inverse(q_{k-1}).  We therefore reject only the impossible increment
// instead of clearing the whole window.  The resulting quality values are
// audit data; no value here is an admission/release gate for the command path.
class SpinEstimator {
 public:
  explicit SpinEstimator(SpinEstimatorConfig config = {});

  void reset() noexcept;
  void push(const BallSample& sample) noexcept;
  SpinEstimate estimate() const noexcept;

 private:
  struct Increment {
    double end_time_s = 0.0;
    double dt_s = 0.0;
    Vec3 rotation_vector_rad = Vec3::Zero();
    bool retained = false;
  };

  void append(const Increment& increment) noexcept;
  void trim(double now_s) noexcept;

  static constexpr std::size_t kCapacity = kMaxEstimatorSamples;
  SpinEstimatorConfig config_;
  std::array<Increment, kCapacity> increments_{};
  std::size_t increment_count_ = 0;
  Eigen::Quaterniond previous_orientation_ = Eigen::Quaterniond::Identity();
  double previous_time_s_ = 0.0;
  bool have_previous_ = false;
};

}  // namespace hope_planner_cpp
