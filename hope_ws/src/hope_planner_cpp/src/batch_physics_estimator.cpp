#include "hope_planner_cpp/batch_physics_estimator.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>

namespace hope_planner_cpp {
namespace {

bool finite_vec(const Vec3& value) noexcept {
  return value.allFinite();
}

}  // namespace

BatchPhysicsEstimator::BatchPhysicsEstimator(
    BallPhysics physics, EstimatorConfig config)
    : physics_(std::move(physics)), config_(std::move(config)) {
  config_.window_s = std::max(0.02, config_.window_s);
  config_.min_span_s = std::clamp(config_.min_span_s, 0.0, config_.window_s);
  config_.min_samples = std::clamp<std::size_t>(config_.min_samples, 6, kMaxEstimatorSamples);
  config_.huber_delta_m = std::max(1.0e-6, config_.huber_delta_m);
  config_.recency_half_life_s = std::max(0.0, config_.recency_half_life_s);
  config_.robust_iterations = std::clamp(config_.robust_iterations, 1, 6);
  config_.integration_dt_s = std::clamp(config_.integration_dt_s, 1.0e-4, 0.01);
  config_.bounce_min_reversal_m = std::max(0.0, config_.bounce_min_reversal_m);
  config_.bounce_min_excursion_m = std::max(0.0, config_.bounce_min_excursion_m);
  config_.bounce_confirmation_samples = std::clamp<std::size_t>(
      config_.bounce_confirmation_samples, 1, 8);
  config_.bounce_confirmation_max_span_s =
      std::max(0.001, config_.bounce_confirmation_max_span_s);
  config_.bounce_sparse_confirmation_min_span_s =
      std::max(0.0, config_.bounce_sparse_confirmation_min_span_s);
  config_.bounce_sparse_confirmation_excursion_m =
      std::max(config_.bounce_min_excursion_m,
               config_.bounce_sparse_confirmation_excursion_m);
  config_.bounce_refractory_s = std::max(0.0, config_.bounce_refractory_s);
}

void BatchPhysicsEstimator::reset() noexcept {
  sample_count_ = 0;
  begin_flight();
}

void BatchPhysicsEstimator::begin_flight() noexcept {
  bounce_index_ = kNoBounceIndex;
  last_bounce_source_time_s_ = -std::numeric_limits<double>::infinity();
  bounce_detected_ = false;
  flight_bounce_seen_ = false;
  post_bounce_only_ = false;
}

bool BatchPhysicsEstimator::bounce_transition_active() const noexcept {
  return bounce_index_ != kNoBounceIndex &&
         bounce_index_ + 1 < sample_count_;
}

const BallSample* BatchPhysicsEstimator::latest_sample() const noexcept {
  return sample_count_ == 0 ? nullptr : &samples_[sample_count_ - 1];
}

double BatchPhysicsEstimator::sample_span_s() const noexcept {
  if (sample_count_ < 2) {
    return 0.0;
  }
  return std::max(0.0, samples_[sample_count_ - 1].source_time_s - samples_[0].source_time_s);
}

void BatchPhysicsEstimator::append(const BallSample& sample) noexcept {
  if (sample_count_ == kMaxEstimatorSamples) {
    std::move(samples_.begin() + 1, samples_.end(), samples_.begin());
    --sample_count_;
    if (bounce_index_ == 0) {
      bounce_index_ = kNoBounceIndex;
    } else if (bounce_index_ != kNoBounceIndex) {
      --bounce_index_;
    }
  }
  samples_[sample_count_++] = sample;
}

void BatchPhysicsEstimator::drop_prefix(std::size_t count) noexcept {
  count = std::min(count, sample_count_);
  if (count == 0) {
    return;
  }
  std::move(samples_.begin() + static_cast<std::ptrdiff_t>(count),
            samples_.begin() + static_cast<std::ptrdiff_t>(sample_count_),
            samples_.begin());
  sample_count_ -= count;
  if (bounce_index_ < count) {
    if (flight_bounce_seen_) {
      post_bounce_only_ = true;
    }
    bounce_index_ = kNoBounceIndex;
  } else if (bounce_index_ != kNoBounceIndex) {
    bounce_index_ -= count;
  }
}

void BatchPhysicsEstimator::trim_to_window() noexcept {
  if (sample_count_ < 2) {
    return;
  }
  const double cutoff = samples_[sample_count_ - 1].source_time_s - config_.window_s;
  std::size_t first = 0;
  while (first + 1 < sample_count_ && samples_[first].source_time_s < cutoff) {
    ++first;
  }
  if (first > 0) {
    drop_prefix(first);
  }
  // Once fewer than three pre-contact samples remain, the piecewise seed is
  // underdetermined. By then the retained post-contact segment spans almost
  // the full window, so discard only the obsolete contact marker and fit that
  // outgoing segment directly. This avoids an artificial initial_fit_failed
  // interval without carrying recursive state.
  if (bounce_index_ != kNoBounceIndex && bounce_index_ + 1 < 3) {
    drop_prefix(bounce_index_ + 1);
  }
}

void BatchPhysicsEstimator::push(const BallSample& sample) noexcept {
  bounce_detected_ = false;
  if (!std::isfinite(sample.source_time_s) || !finite_vec(sample.position)) {
    return;
  }

  if (sample_count_ > 0 &&
      sample.source_time_s <= samples_[sample_count_ - 1].source_time_s) {
    // A non-increasing source clock cannot define derivatives.  This is a
    // source-epoch boundary, not a gap-duration restart.
    reset();
  }

  append(sample);
  trim_to_window();

  const std::size_t confirmation = config_.bounce_confirmation_samples;
  // A legal incoming flight contains at most one table contact on the robot
  // side. Keep that contact epoch for the whole flight instead of allowing a
  // later noisy local minimum to switch the model a second time.
  if (!flight_bounce_seen_ && sample_count_ >= confirmation + 2) {
    const std::size_t maximum_post_samples = std::min(
        confirmation, sample_count_ - 1 - confirmation);
    std::size_t confirmed_post_samples = 0;
    for (std::size_t post_samples = 1;
         post_samples <= maximum_post_samples; ++post_samples) {
      const std::size_t center_index = sample_count_ - 1 - post_samples;
      const std::size_t before_index = center_index - confirmation;
      const BallSample& center = samples_[center_index];
      const double center_z = center.position.z();
      const double pre_span_s =
          center.source_time_s - samples_[before_index].source_time_s;
      const double post_span_s =
          samples_[sample_count_ - 1].source_time_s - center.source_time_s;
      bool center_is_window_minimum = true;
      for (std::size_t index = before_index; index < sample_count_; ++index) {
        if (index != center_index && samples_[index].position.z() <= center_z) {
          center_is_window_minimum = false;
          break;
        }
      }
      const double post_excursion_m =
          samples_[sample_count_ - 1].position.z() - center_z;
      const bool dense_confirmation = post_samples >= confirmation;
      const bool sparse_gap_confirmation =
          post_span_s >= config_.bounce_sparse_confirmation_min_span_s &&
          post_excursion_m >= config_.bounce_sparse_confirmation_excursion_m;
      const double since_bounce_s =
          center.source_time_s - last_bounce_source_time_s_;
      const bool center_min =
          (dense_confirmation || sparse_gap_confirmation) &&
          center_z <= config_.bounce_center_z_max_m &&
          center_is_window_minimum &&
          samples_[center_index - 1].position.z() - center_z >=
              config_.bounce_min_reversal_m &&
          samples_[center_index + 1].position.z() - center_z >=
              config_.bounce_min_reversal_m &&
          samples_[before_index].position.z() - center_z >=
              config_.bounce_min_excursion_m &&
          post_excursion_m >= config_.bounce_min_excursion_m &&
          pre_span_s <= config_.bounce_confirmation_max_span_s &&
          post_span_s <= config_.bounce_confirmation_max_span_s &&
          since_bounce_s >= config_.bounce_refractory_s;
      if (center_min) {
        confirmed_post_samples = post_samples;
        break;
      }
    }
    if (confirmed_post_samples > 0) {
      // Keep the complete causal pre-contact window. A second contact cannot
      // share one six-state fit with the first, so retain only the flight that
      // starts immediately after the previous contact before marking the new
      // split. A normal 360 Hz stream uses five post samples; after a source
      // gap, elapsed time plus a large rise can confirm the same V shape with
      // fewer retained samples. No post-contact warm-up/reset is introduced.
      if (bounce_index_ != kNoBounceIndex) {
        drop_prefix(bounce_index_ + 1);
      }
      // drop_prefix above can shift the just-confirmed center index.
      bounce_index_ = sample_count_ - 1 - confirmed_post_samples;
      last_bounce_source_time_s_ = samples_[bounce_index_].source_time_s;
      bounce_detected_ = true;
      flight_bounce_seen_ = true;
      post_bounce_only_ = false;
    }
  }
}

bool BatchPhysicsEstimator::initial_state(State6& state) const noexcept {
  const std::size_t fit_count = bounce_transition_active()
      ? bounce_index_ + 1
      : sample_count_;
  if (fit_count < 3) {
    return false;
  }
  const double t0 = samples_[0].source_time_s;
  std::array<double, kMaxEstimatorSamples> weights{};
  for (std::size_t i = 0; i < fit_count; ++i) {
    const double age_s =
        samples_[fit_count - 1].source_time_s - samples_[i].source_time_s;
    weights[i] = config_.recency_half_life_s > 0.0
        ? std::exp2(-std::max(0.0, age_s) / config_.recency_half_life_s)
        : 1.0;
  }

  Eigen::Vector2d cx = Eigen::Vector2d::Zero();
  Eigen::Vector2d cy = Eigen::Vector2d::Zero();
  Eigen::Vector3d cz = Eigen::Vector3d::Zero();
  // Robust linear X/Y and quadratic Z provide a deterministic seed for the
  // physical Gauss-Newton fit. This seed is recomputed from the raw window on
  // every solve and carries no recursive state.
  for (int iteration = 0; iteration < 3; ++iteration) {
    Eigen::Matrix2d normal_xy = Eigen::Matrix2d::Zero();
    Eigen::Matrix3d normal_z = Eigen::Matrix3d::Zero();
    Eigen::Vector2d bx = Eigen::Vector2d::Zero();
    Eigen::Vector2d by = Eigen::Vector2d::Zero();
    Eigen::Vector3d bz = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < fit_count; ++i) {
      const double t = samples_[i].source_time_s - t0;
      const double t2 = t * t;
      const Eigen::Vector2d row_xy(1.0, t);
      const Eigen::Vector3d row_z(1.0, t, t2);
      const double weight = weights[i];
      normal_xy.noalias() += weight * row_xy * row_xy.transpose();
      normal_z.noalias() += weight * row_z * row_z.transpose();
      bx.noalias() += weight * row_xy * samples_[i].position.x();
      by.noalias() += weight * row_xy * samples_[i].position.y();
      bz.noalias() += weight * row_z * samples_[i].position.z();
    }
    const auto xy_ldlt = normal_xy.ldlt();
    const auto z_ldlt = normal_z.ldlt();
    if (xy_ldlt.info() != Eigen::Success || z_ldlt.info() != Eigen::Success) {
      return false;
    }
    cx = xy_ldlt.solve(bx);
    cy = xy_ldlt.solve(by);
    cz = z_ldlt.solve(bz);
    if (!cx.allFinite() || !cy.allFinite() || !cz.allFinite()) {
      return false;
    }
    if (iteration == 2) {
      break;
    }
    for (std::size_t i = 0; i < fit_count; ++i) {
      const double t = samples_[i].source_time_s - t0;
      const Vec3 fitted(
          cx[0] + cx[1] * t,
          cy[0] + cy[1] * t,
          cz[0] + cz[1] * t + cz[2] * t * t);
      const double residual = (fitted - samples_[i].position).norm();
      const double robust = residual <= config_.huber_delta_m
          ? 1.0
          : config_.huber_delta_m / std::max(residual, 1.0e-12);
      const double age_s =
          samples_[fit_count - 1].source_time_s - samples_[i].source_time_s;
      const double recency = config_.recency_half_life_s > 0.0
          ? std::exp2(-std::max(0.0, age_s) / config_.recency_half_life_s)
          : 1.0;
      weights[i] = robust * recency;
    }
  }
  state << cx[0], cy[0], cz[0], cx[1], cy[1], cz[1];
  return state.allFinite();
}

bool BatchPhysicsEstimator::propagate(State6& state, double duration_s) const noexcept {
  if (!state.allFinite() || !std::isfinite(duration_s) || duration_s < 0.0) {
    return false;
  }
  Vec3 p = state.head<3>();
  Vec3 v = state.tail<3>();
  double remaining = duration_s;
  while (remaining > 1.0e-12) {
    const double h = std::min(config_.integration_dt_s, remaining);
    const double speed = v.norm();
    const Vec3 acceleration = -physics_.drag_k * speed * v + physics_.gravity;
    p += v * h + 0.5 * acceleration * h * h;
    v += acceleration * h;
    remaining -= h;
    if (!finite_vec(p) || !finite_vec(v)) {
      return false;
    }
  }
  state.head<3>() = p;
  state.tail<3>() = v;
  return true;
}

bool BatchPhysicsEstimator::apply_table_bounce(State6& state) const noexcept {
  if (!state.allFinite()) {
    return false;
  }
  Vec3 velocity = state.tail<3>();
  const Vec3 tangential(velocity.x(), velocity.y(), 0.0);
  const double tangential_speed = tangential.norm();
  const double raw_impulse_speed =
      std::max(0.0, physics_.table_tangential_gain) * tangential_speed;
  const double friction_cap =
      std::max(0.0, physics_.table_friction_cap_mu) *
      (1.0 + physics_.restitution_v) * std::abs(velocity.z());
  const double impulse_speed = std::min(raw_impulse_speed, friction_cap);
  if (tangential_speed > 1.0e-12) {
    velocity.head<2>() -=
        (impulse_speed / tangential_speed) * tangential.head<2>();
  }
  velocity.z() = -physics_.restitution_v * velocity.z();
  state.tail<3>() = velocity;
  return state.allFinite();
}

bool BatchPhysicsEstimator::simulate_positions(
    const State6& initial,
    std::array<Vec3, kMaxEstimatorSamples>& positions,
    State6* latest_state) const noexcept {
  State6 state = initial;
  positions[0] = state.head<3>();
  double current_t = samples_[0].source_time_s;
  for (std::size_t i = 1; i < sample_count_; ++i) {
    if (bounce_transition_active() && i == bounce_index_ + 1 &&
        !apply_table_bounce(state)) {
      return false;
    }
    const double duration = samples_[i].source_time_s - current_t;
    if (!propagate(state, duration)) {
      return false;
    }
    positions[i] = state.head<3>();
    current_t = samples_[i].source_time_s;
  }
  if (latest_state != nullptr) {
    *latest_state = state;
  }
  return true;
}

BallState BatchPhysicsEstimator::estimate() noexcept {
  BallState output;
  output.sample_count = sample_count_;
  output.sample_span_s = sample_span_s();
  output.bounce_transition_used = bounce_transition_active();
  output.bounce_epoch_active = flight_bounce_seen_;
  if (output.bounce_transition_used) {
    output.bounce_source_time_s = samples_[bounce_index_].source_time_s;
    output.pre_bounce_samples = bounce_index_ + 1;
    output.post_bounce_samples = sample_count_ - output.pre_bounce_samples;
  } else if (flight_bounce_seen_) {
    output.bounce_source_time_s = last_bounce_source_time_s_;
    output.post_bounce_samples = post_bounce_only_ ? sample_count_ : 0;
  }
  if (sample_count_ < config_.min_samples ||
      output.sample_span_s + 1.0e-12 < config_.min_span_s) {
    output.reason = "estimator_not_ready";
    return output;
  }

  State6 state;
  if (!initial_state(state)) {
    output.reason = "initial_fit_failed";
    return output;
  }

  constexpr std::array<double, 6> perturbation{
      1.0e-4, 1.0e-4, 1.0e-4, 1.0e-3, 1.0e-3, 1.0e-3};

  for (int iteration = 0; iteration < config_.robust_iterations; ++iteration) {
    if (!simulate_positions(state, predicted_[0])) {
      output.reason = "physics_fit_diverged";
      return output;
    }
    for (int column = 0; column < 6; ++column) {
      State6 shifted = state;
      shifted[column] += perturbation[static_cast<std::size_t>(column)];
      if (!simulate_positions(shifted, predicted_[static_cast<std::size_t>(column + 1)])) {
        output.reason = "physics_jacobian_diverged";
        return output;
      }
    }

    Matrix6 normal = Matrix6::Zero();
    State6 gradient = State6::Zero();
    for (std::size_t i = 0; i < sample_count_; ++i) {
      const Vec3 residual = predicted_[0][i] - samples_[i].position;
      const double residual_norm = residual.norm();
      const double weight = residual_norm <= config_.huber_delta_m
          ? 1.0
          : config_.huber_delta_m / std::max(residual_norm, 1.0e-12);
      const double age_s =
          samples_[sample_count_ - 1].source_time_s - samples_[i].source_time_s;
      const double recency_weight = config_.recency_half_life_s > 0.0
          ? std::exp2(-std::max(0.0, age_s) / config_.recency_half_life_s)
          : 1.0;
      const double combined_weight = weight * recency_weight;
      Eigen::Matrix<double, 3, 6> jacobian;
      for (int column = 0; column < 6; ++column) {
        jacobian.col(column) =
            (predicted_[static_cast<std::size_t>(column + 1)][i] - predicted_[0][i]) /
            perturbation[static_cast<std::size_t>(column)];
      }
      normal.noalias() += combined_weight * jacobian.transpose() * jacobian;
      gradient.noalias() += combined_weight * jacobian.transpose() * residual;
    }
    normal.diagonal().array() += 1.0e-8;
    const auto ldlt = normal.ldlt();
    if (ldlt.info() != Eigen::Success) {
      output.reason = "physics_normal_singular";
      return output;
    }
    const State6 step = ldlt.solve(gradient);
    if (!step.allFinite()) {
      output.reason = "physics_step_nonfinite";
      return output;
    }
    state -= step;
    if (!state.allFinite()) {
      output.reason = "physics_state_nonfinite";
      return output;
    }
    if (step.norm() < 1.0e-7) {
      break;
    }
  }

  if (!simulate_positions(state, predicted_[0])) {
    output.reason = "physics_final_diverged";
    return output;
  }
  double squared_sum = 0.0;
  double residual_max = 0.0;
  for (std::size_t i = 0; i < sample_count_; ++i) {
    const double residual = (predicted_[0][i] - samples_[i].position).norm();
    squared_sum += residual * residual;
    residual_max = std::max(residual_max, residual);
  }

  State6 latest;
  if (!simulate_positions(state, predicted_[0], &latest)) {
    output.reason = "physics_latest_diverged";
    return output;
  }
  output.position = latest.head<3>();
  output.velocity = latest.tail<3>();
  output.source_time_s = samples_[sample_count_ - 1].source_time_s;
  output.residual_rms_m = std::sqrt(squared_sum / static_cast<double>(sample_count_));
  output.residual_max_m = residual_max;
  output.valid = output.position.allFinite() && output.velocity.allFinite() &&
                 std::isfinite(output.source_time_s);
  output.reason = output.valid ? "estimate_valid" : "estimate_nonfinite";
  return output;
}

}  // namespace hope_planner_cpp
