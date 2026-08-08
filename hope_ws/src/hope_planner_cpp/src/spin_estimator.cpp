#include "hope_planner_cpp/spin_estimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace hope_planner_cpp {
namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

template <std::size_t Capacity>
double median_prefix(std::array<double, Capacity> values, std::size_t count) {
  if (count == 0) {
    return 0.0;
  }
  const auto end = values.begin() + static_cast<std::ptrdiff_t>(count);
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(count / 2);
  std::nth_element(values.begin(), middle, end);
  const double upper = *middle;
  if (count % 2 != 0) {
    return upper;
  }
  const double lower = *std::max_element(values.begin(), middle);
  return 0.5 * (lower + upper);
}

}  // namespace

SpinEstimator::SpinEstimator(SpinEstimatorConfig config)
    : config_(std::move(config)) {}

void SpinEstimator::reset() noexcept {
  increment_count_ = 0;
  previous_orientation_ = Eigen::Quaterniond::Identity();
  previous_time_s_ = 0.0;
  have_previous_ = false;
}

void SpinEstimator::append(const Increment& increment) noexcept {
  if (increment_count_ < kCapacity) {
    increments_[increment_count_++] = increment;
    return;
  }
  std::move(increments_.begin() + 1, increments_.end(), increments_.begin());
  increments_.back() = increment;
}

void SpinEstimator::trim(double now_s) noexcept {
  const double cutoff = now_s - std::max(0.0, config_.window_s);
  std::size_t first = 0;
  while (first < increment_count_ && increments_[first].end_time_s <= cutoff) {
    ++first;
  }
  if (first == 0) {
    return;
  }
  std::move(
      increments_.begin() + static_cast<std::ptrdiff_t>(first),
      increments_.begin() + static_cast<std::ptrdiff_t>(increment_count_),
      increments_.begin());
  increment_count_ -= first;
}

void SpinEstimator::push(const BallSample& sample) noexcept {
  try {
    if (!sample.orientation_valid || !std::isfinite(sample.source_time_s) ||
        !sample.orientation.coeffs().allFinite() ||
        sample.orientation.norm() < 1.0e-9) {
      reset();
      return;
    }
    Eigen::Quaterniond orientation = sample.orientation.normalized();
    if (!have_previous_) {
      previous_orientation_ = orientation;
      previous_time_s_ = sample.source_time_s;
      have_previous_ = true;
      return;
    }
    if (sample.source_time_s <= previous_time_s_) {
      return;
    }
    const double dt = sample.source_time_s - previous_time_s_;
    if (dt > config_.max_gap_s) {
      increment_count_ = 0;
      previous_orientation_ = orientation;
      previous_time_s_ = sample.source_time_s;
      return;
    }
    if (orientation.coeffs().dot(previous_orientation_.coeffs()) < 0.0) {
      orientation.coeffs() *= -1.0;
    }
    Eigen::Quaterniond relative = orientation * previous_orientation_.conjugate();
    relative.normalize();
    if (relative.w() < 0.0) {
      relative.coeffs() *= -1.0;
    }
    const Vec3 vector = relative.vec();
    const double sin_half = vector.norm();
    Vec3 rotation_vector = Vec3::Zero();
    if (sin_half >= 1.0e-12) {
      const double angle = 2.0 * std::atan2(sin_half, relative.w());
      rotation_vector = (angle / sin_half) * vector;
    }
    const double rate_rev_s = rotation_vector.norm() / (dt * kTwoPi);
    const bool retained = std::isfinite(rate_rev_s) &&
                          rate_rev_s <= config_.max_rev_s;
    append(Increment{sample.source_time_s, dt, rotation_vector, retained});
    trim(sample.source_time_s);
    // Always adopt the new reference.  A fixed marker-permutation offset then
    // cancels from the next relative quaternion instead of poisoning 100 ms.
    previous_orientation_ = orientation;
    previous_time_s_ = sample.source_time_s;
  } catch (...) {
    reset();
  }
}

SpinEstimate SpinEstimator::estimate() const noexcept {
  SpinEstimate output;
  try {
    if (!have_previous_ || increment_count_ == 0) {
      return output;
    }
    std::array<Vec3, kCapacity> rates{};
    std::array<double, kCapacity> dt_weights{};
    std::array<double, kCapacity> x_values{};
    std::array<double, kCapacity> y_values{};
    std::array<double, kCapacity> z_values{};
    std::size_t retained_count = 0;
    double retained_dt = 0.0;
    double total_dt = 0.0;
    Vec3 total_rotation = Vec3::Zero();
    double total_rotation_norm = 0.0;
    for (std::size_t index = 0; index < increment_count_; ++index) {
      const auto& increment = increments_[index];
      total_dt += std::max(0.0, increment.dt_s);
      if (!increment.retained || increment.dt_s <= 0.0) {
        ++output.rejected_increments;
        continue;
      }
      const Vec3 rate = increment.rotation_vector_rad / increment.dt_s;
      if (!rate.allFinite()) {
        ++output.rejected_increments;
        continue;
      }
      rates[retained_count] = rate;
      dt_weights[retained_count] = increment.dt_s;
      x_values[retained_count] = rate.x();
      y_values[retained_count] = rate.y();
      z_values[retained_count] = rate.z();
      ++retained_count;
      retained_dt += increment.dt_s;
      total_rotation += increment.rotation_vector_rad;
      total_rotation_norm += increment.rotation_vector_rad.norm();
    }
    output.retained_increments = retained_count;
    output.sample_span_s = total_dt;
    output.retained_time_fraction = config_.window_s > 0.0
        ? std::clamp(retained_dt / config_.window_s, 0.0, 1.0)
        : 0.0;
    output.coherence = total_rotation_norm > 1.0e-12
        ? std::clamp(total_rotation.norm() / total_rotation_norm, 0.0, 1.0)
        : 0.0;
    if (retained_count < config_.min_increments) {
      output.reason = "spin_insufficient_increments";
      return output;
    }
    if (retained_dt < config_.min_span_s) {
      output.reason = "spin_insufficient_span";
      return output;
    }
    Vec3 omega(
        median_prefix(x_values, retained_count),
        median_prefix(y_values, retained_count),
        median_prefix(z_values, retained_count));
    const double huber_delta = std::max(0.0, config_.huber_delta_rev_s) * kTwoPi;
    for (int iteration = 0; iteration < std::max(1, config_.robust_iterations);
         ++iteration) {
      Vec3 numerator = Vec3::Zero();
      double denominator = 0.0;
      for (std::size_t index = 0; index < retained_count; ++index) {
        const double residual = (rates[index] - omega).norm();
        const double robust_weight = huber_delta > 0.0 && residual > huber_delta
            ? huber_delta / residual
            : 1.0;
        const double weight = dt_weights[index] * robust_weight;
        numerator += weight * rates[index];
        denominator += weight;
      }
      if (!(denominator > 0.0)) {
        output.reason = "spin_zero_weight";
        return output;
      }
      omega = numerator / denominator;
    }
    if (!omega.allFinite()) {
      output.reason = "spin_nonfinite";
      return output;
    }
    output.omega_rad_s = omega;
    output.valid = true;
    output.reason = "spin_valid";
    return output;
  } catch (...) {
    output.reason = "spin_exception";
    return output;
  }
}

}  // namespace hope_planner_cpp
