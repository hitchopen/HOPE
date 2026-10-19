#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/ball_dynamics.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hope_planner_cpp {
namespace {

bool finite_vec(const Vec3& value) noexcept {
  return value.allFinite();
}

}  // namespace

BatchPhysicsEstimator::BatchPhysicsEstimator(
    BallPhysics physics, EstimatorConfig config, SpinEstimatorConfig spin_config)
    : physics_(std::move(physics)), config_(std::move(config)),
      spin_config_(std::move(spin_config)), trusted_spin_(spin_config_, config_.spin_min_coherence) {
  validate_ball_physics(physics_);
  if (!std::isfinite(config_.spin_min_coherence) ||
      config_.spin_min_coherence < 0.0 || config_.spin_min_coherence > 1.0)
    throw std::invalid_argument("invalid estimator spin coherence");
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
  if (!std::isfinite(config_.post_bounce_min_span_s) ||
      config_.post_bounce_min_span_s < 0.0 ||
      (config_.post_bounce_min_span_s > 0.0 &&
       (config_.post_bounce_min_span_s < 0.02 ||
        config_.post_bounce_min_span_s > config_.window_s)) ||
      !std::isfinite(config_.post_bounce_exclusion_s) ||
      config_.post_bounce_exclusion_s < 0.0) {
    throw std::invalid_argument("invalid post-contact estimator window");
  }
}

void BatchPhysicsEstimator::reset() noexcept {
  sample_count_ = 0;
  trusted_spin_.reset();
  contacts_.clear();
  begin_flight();
}

void BatchPhysicsEstimator::begin_flight() noexcept {
  contacts_.clear();
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
    std::move(spin_at_sample_.begin()+1,spin_at_sample_.end(),spin_at_sample_.begin());
    --sample_count_;
    if (bounce_index_ == 0) {
      bounce_index_ = kNoBounceIndex;
    } else if (bounce_index_ != kNoBounceIndex) {
      --bounce_index_;
    }
  }
  spin_at_sample_[sample_count_] = config_.spin_enabled ? trusted_spin_.observe(sample) : EffectiveSpin{};
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
  std::move(spin_at_sample_.begin()+count,spin_at_sample_.begin()+sample_count_,spin_at_sample_.begin());
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

  if(sample_count_>0) {
    const auto& previous=samples_[sample_count_-1];
    if(sample.frame_id!=previous.frame_id || sample.calibration_id!=previous.calibration_id || sample.ball_id!=previous.ball_id ||
        sample.clock_domain!=previous.clock_domain || sample.clock_mapping_id!=previous.clock_mapping_id)reset();
  }
  if (sample_count_ > 0 &&
      sample.source_time_s <= samples_[sample_count_ - 1].source_time_s) {
    // Duplicate/reordered transport is rejected, not a physical shot boundary.
    return;
  }

  append(sample);
  trim_to_window();

  const std::size_t confirmation = config_.bounce_confirmation_samples;
  // A legal incoming flight contains at most one table contact on the robot
  // side. Keep that contact epoch for the whole flight instead of allowing a
  // later noisy local minimum to switch the model a second time.
  if (sample_count_ >= confirmation + 2) {
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
      bounce_index_ = sample_count_ - 1 - confirmed_post_samples;
      const auto& center=samples_[bounce_index_];
      const auto id=std::max(contacts_.empty()?std::uint64_t{0}:contacts_.back().id,
                             trusted_spin_.value().last_contact_id)+1;
      if(contacts_.size()<64)contacts_.push_back({center.source_time_s,center.source_time_ns,id});
      else return; // bounded physical-shot ledger
      if(config_.spin_enabled && bounce_index_>0) {
        const auto& before=samples_[bounce_index_-1];
        const double dt=center.source_time_s-before.source_time_s;
        Vec3 velocity=(center.position-before.position)/dt;
        trusted_spin_.contact(id,center.source_time_ns,velocity,physics_);
        // Repair only held lineage after this now-confirmed contact; newer measurements stand.
        for(std::size_t j=bounce_index_+1;j<sample_count_;++j) {
          auto& spin=spin_at_sample_[j];
          if(spin.usable() && spin.measurement_time_ns<=center.source_time_ns && spin.last_contact_id<id) {
            ModelState model;model.velocity=velocity;model.spin=spin;apply_model_contact(model,physics_);
            spin=model.spin;spin.last_contact_id=id;
          }
        }
      }
      last_bounce_source_time_s_ = samples_[bounce_index_].source_time_s;
      bounce_detected_ = true;
      flight_bounce_seen_ = true;
      post_bounce_only_ = false;
    }
  }
}

bool BatchPhysicsEstimator::initial_state(State6& state) const noexcept {
  std::size_t fit_count=sample_count_;
  for(const auto& c:contacts_) if(sample_count_ && c.time_s>=samples_[0].source_time_s) {
    fit_count=0;while(fit_count<sample_count_ && samples_[fit_count].source_time_s<=c.time_s+1e-12)++fit_count;
    break;
  }
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

void BatchPhysicsEstimator::prepare_spin() noexcept {
  pre_spin_={};post_spin_={};
  if(sample_count_ && config_.spin_enabled) {
    pre_spin_.effective=spin_at_sample_[0];
    if(pre_spin_.effective.usable()) {pre_spin_.omega_rad_s=*pre_spin_.effective.omega;pre_spin_.valid=true;pre_spin_.coherence=1;}
  }
}
bool BatchPhysicsEstimator::propagate(State6& state,double duration,const Vec3& omega) const noexcept {
  ModelState model;model.position=state.head<3>();model.velocity=state.tail<3>();
  if(omega.allFinite() && omega.squaredNorm()>0)model.spin.omega=omega;
  for(double t=0;t<duration-1e-12;) {const double h=std::min(config_.integration_dt_s,duration-t);integrate_ball(model,h,physics_);t+=h;}
  state.head<3>()=model.position;state.tail<3>()=model.velocity;return state.allFinite();
}
bool BatchPhysicsEstimator::apply_table_bounce(State6& state,Vec3& omega) const noexcept {
  ModelState model;model.velocity=state.tail<3>();if(omega.squaredNorm()>0)model.spin.omega=omega;
  apply_model_contact(model,physics_);state.tail<3>()=model.velocity;
  omega=model.spin.usable()?*model.spin.omega:Vec3::Zero();return state.allFinite();
}
bool BatchPhysicsEstimator::simulate_positions(const State6& initial,
    std::array<Vec3,kMaxEstimatorSamples>& positions,State6* latest,Vec3* latest_spin) const noexcept {
  if(!sample_count_)return false;
  ModelState model;model.position=initial.head<3>();model.velocity=initial.tail<3>();model.spin=spin_at_sample_[0];
  positions[0]=model.position;
  auto advance=[&](double duration) {
    for(double t=0;t<duration-1e-12;) {const double h=std::min(config_.integration_dt_s,duration-t);integrate_ball(model,h,physics_);t+=h;}
  };
  for(std::size_t i=1;i<sample_count_;++i) {
    double t=samples_[i-1].source_time_s;
    for(const auto& c:contacts_) if(c.time_s>=t-1e-12 && c.time_s<samples_[i].source_time_s-1e-12 &&
        (c.time_s>samples_[i-1].source_time_s+1e-12 || i==1 || c.time_s==samples_[i-1].source_time_s)) {
      advance(std::max(0.,c.time_s-t));apply_model_contact(model,physics_);model.spin.last_contact_id=c.id;t=c.time_s;
    }
    advance(samples_[i].source_time_s-t);
    positions[i]=model.position;
    // A later measurement affects only intervals starting at its source time.
    const auto& measured=spin_at_sample_[i];
    if(measured.usable() && measured.mode==EffectiveSpinMode::MEASURED)model.spin=measured;
  }
  if(latest){latest->head<3>()=model.position;latest->tail<3>()=model.velocity;}
  if(latest_spin)*latest_spin=model.spin.usable()?*model.spin.omega:Vec3::Zero();
  return model.position.allFinite() && model.velocity.allFinite();
}

BallState BatchPhysicsEstimator::estimate() noexcept {
  if (config_.post_bounce_min_span_s > 0.0 && flight_bounce_seen_ &&
      sample_count_ > 0) {
    const double cutoff = last_bounce_source_time_s_ +
        config_.post_bounce_exclusion_s;
    std::size_t first = 0;
    while (first < sample_count_ && samples_[first].source_time_s <= cutoff)
      ++first;
    if (sample_count_ - first >= config_.min_samples &&
        samples_[sample_count_ - 1].source_time_s - samples_[first].source_time_s +
                1.0e-12 >= config_.post_bounce_min_span_s) {
      EstimatorConfig post_config = config_;
      post_config.min_span_s = config_.post_bounce_min_span_s;
      post_config.post_bounce_min_span_s = 0.0;  // no recursive contact fit
      BatchPhysicsEstimator post(physics_, post_config, spin_config_);
      post.restore_spin_checkpoint(spin_at_sample_[first]);
      for (std::size_t i = first; i < sample_count_; ++i) post.push(samples_[i]);
      BallState measured = post.estimate();
      if (measured.valid && !measured.bounce_epoch_active) {
        measured.bounce_epoch_active = true;
        measured.bounce_source_time_s = last_bounce_source_time_s_;
        measured.pre_bounce_samples = 0;
        measured.post_bounce_samples = measured.sample_count;
        measured.reason = "post_bounce_free_flight";
        return measured;
      }
    }
  }
  BallState output;
  output.spin_model_enabled = config_.spin_enabled;
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

  prepare_spin();
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
  Vec3 latest_spin;
  if (!simulate_positions(state, predicted_[0], &latest, &latest_spin)) {
    output.reason = "physics_latest_diverged";
    return output;
  }
  output.position = latest.head<3>();
  output.velocity = latest.tail<3>();
  output.physics_spin = bounce_transition_active() && post_spin_.valid ? post_spin_ : pre_spin_;
  output.physics_spin.effective=trusted_spin_.value();
  output.physics_spin.valid=output.physics_spin.effective.usable();
  output.physics_spin.coherence=output.physics_spin.valid?1:0;
  output.physics_spin.omega_rad_s = latest_spin;
  if(output.physics_spin.effective.usable())output.physics_spin.effective.omega=latest_spin;
  if (bounce_transition_active() && !post_spin_.valid && pre_spin_.valid)
    output.physics_spin.reason = "spin_propagated_through_observed_contact";
  output.source_time_s = samples_[sample_count_ - 1].source_time_s;
  output.residual_rms_m = std::sqrt(squared_sum / static_cast<double>(sample_count_));
  output.residual_max_m = residual_max;
  output.valid = output.position.allFinite() && output.velocity.allFinite() &&
                 std::isfinite(output.source_time_s);
  output.reason = output.valid ? "estimate_valid" : "estimate_nonfinite";
  return output;
}

}  // namespace hope_planner_cpp
