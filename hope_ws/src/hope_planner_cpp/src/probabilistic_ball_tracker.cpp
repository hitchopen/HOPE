#include "hope_planner_cpp/probabilistic_ball_tracker.hpp"
#include "hope_planner_cpp/probabilistic_physics.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace hope_planner_cpp {
namespace {
bool joint_psd(const Matrix6 &state, const Eigen::Matrix<double, 6, 3> &cross,
               const Matrix3 &bias) {
  Matrix9 p;
  p << state, cross, cross.transpose(), bias;
  if (!p.allFinite())
    return false;
  Eigen::SelfAdjointEigenSolver<Matrix9> eig((p + p.transpose()).eval() * .5,
                                             Eigen::EigenvaluesOnly);
  return eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() >= -1e-10;
}
} // namespace

ProbabilisticBallTracker::ProbabilisticBallTracker(ProbabilisticConfig c)
    : config_(std::move(c)) {
  config_.validate();
}
FullTrackerCheckpoint ProbabilisticBallTracker::export_checkpoint() const {
  return {belief_,
          bias_mean_,
          state_bias_covariance_,
          bias_covariance_,
          consecutive_rejections_,
          innovation_sum_squared_,
          noise_model_fingerprint(config_)};
}
void ProbabilisticBallTracker::restore_checkpoint(
    const FullTrackerCheckpoint &c) {
  if (c.noise_fingerprint != noise_model_fingerprint(config_) || !c.valid())
    throw std::invalid_argument(
        "invalid or incompatible full tracker checkpoint");
  belief_ = c.belief;
  bias_mean_ = c.bias_mean;
  state_bias_covariance_ = c.state_bias_covariance;
  bias_covariance_ = c.bias_covariance;
  consecutive_rejections_ = c.consecutive_rejections;
  innovation_sum_squared_ = c.innovation_sum_squared;
}
void ProbabilisticBallTracker::reset() {
  belief_ = StateBelief{};
  bias_mean_.setZero();
  state_bias_covariance_.setZero();
  bias_covariance_.setZero();
  consecutive_rejections_ = 0;
  innovation_sum_squared_ = 0;
}
void ProbabilisticBallTracker::initialize(const BallSample &s) {
  reset();
  belief_.initialized = true;
  belief_.mean.head<3>() = s.position;
  belief_.mean.tail<3>() = config_.velocity_prior_mean;
  belief_.covariance.setZero();
  belief_.covariance.topLeftCorner<3, 3>().diagonal() =
      (config_.measurement_std_m.array().square() +
       config_.measurement_bias_std_m.array().square())
          .matrix();
  bias_covariance_.diagonal() =
      config_.measurement_bias_std_m.array().square().matrix();
  state_bias_covariance_.topRows<3>() = -bias_covariance_;
  belief_.covariance.bottomRightCorner<3, 3>().diagonal() =
      config_.velocity_prior_std.array().square().matrix();
  belief_.source_time_s = belief_.first_measurement_time_s =
      belief_.last_measurement_time_s = s.source_time_s;
  belief_.source_time_ns = belief_.last_measurement_time_ns = s.source_time_ns;
  belief_.receipt_steady_ns = s.receipt_steady_ns;
  belief_.receipt_wall_ns = s.receipt_wall_ns;
  belief_.observation_count = 1;
}
void ProbabilisticBallTracker::propagate(double time) {
  while (belief_.source_time_s + 1e-12 < time) {
    const double dt = std::min(.01, time - belief_.source_time_s);
    std::optional<PropagationAudit> audit;
    if (audit_sink_) {
      audit.emplace();
      audit->before = export_checkpoint(); audit->duration_s = dt;
      audit->nominal = belief_.mean;
      for (double t = 0.; t < dt - 1e-12;) {
        const double h = std::min(config_.integration_dt_s, dt - t);
        const auto event = physical_step(audit->nominal, h, config_);
        if (event.bounced && !std::isfinite(audit->nominal_contact_offset_s))
          audit->nominal_contact_offset_s = t + event.fraction * h;
        t += h;
      }
    }
    const Matrix6 prior_covariance = positive_covariance(belief_.covariance);
    Matrix6 root = prior_covariance.llt().matrixL();
    const auto prior_cross = state_bias_covariance_;
    std::array<State6, 12> sigma;
    State6 mean = State6::Zero();
    double contacts = 0;
    for (int i = 0; i < 12; ++i) {
      sigma[i] =
          belief_.mean + (i < 6 ? 1. : -1.) * std::sqrt(6.) * root.col(i % 6);
      if (audit_sink_) audit->members[i].initial = sigma[i];
      double remaining = dt;
      bool bounced = false;
      while (remaining > 1e-12) {
        double h = std::min(remaining, config_.integration_dt_s);
        const State6 step_initial = audit_sink_ ? sigma[i] : State6::Zero();
        const auto event = physical_step(sigma[i], h, config_);
        bounced = event.bounced || bounced;
        if (audit_sink_ && event.bounced) {
          auto &member = audit->members[i];
          if (!member.impulses) {
            member.contact_offset_s = dt - remaining + event.fraction * h;
            member.contact_pre = free_flight(step_initial, event.fraction * h, config_.physics);
            member.contact_pre[2] = config_.physics.radius;
            member.contact_post = event.contact;
          }
          ++member.impulses;
        }
        remaining -= h;
      }
      if (audit_sink_) audit->members[i].final = sigma[i];
      contacts += bounced ? 1. : 0.;
      mean += sigma[i] / 12.;
    }
    Matrix6 covariance =
        process_covariance(dt, config_.acceleration_spectral_density);
    for (const auto &point : sigma) {
      State6 d = point - mean;
      covariance.noalias() += d * d.transpose() / 12.;
    }
    covariance.bottomRightCorner<3, 3>().diagonal() +=
        (contacts / 12.) *
        config_.bounce_velocity_std.array().square().matrix();
    if (config_.measurement_bias_std_m.maxCoeff() > 0.) {
      const double rho = std::exp(-dt / config_.measurement_bias_tau_s);
      Eigen::Matrix<double, 6, 6> transition = Matrix6::Identity();
      transition.topRightCorner<3, 3>() = Matrix3::Identity() * dt;
      state_bias_covariance_ = rho * transition * state_bias_covariance_;
      bias_covariance_ =
          rho * rho * bias_covariance_ +
          (1. - rho * rho) * config_.measurement_bias_std_m.array()
                                 .square()
                                 .matrix()
                                 .asDiagonal()
                                 .toDenseMatrix();
      bias_mean_ *= rho;
    }
    const Matrix6 next_covariance = positive_covariance(covariance);
    if (config_.measurement_bias_std_m.maxCoeff() > 0. &&
        !joint_psd(next_covariance, state_bias_covariance_, bias_covariance_)) {
      // The old constant-velocity cross transition can disagree with the
      // nonlinear cubature marginal at contact and make the joint indefinite.
      // Recombine cross moments from the SAME positive-weight cubature pairs.
      // C(y,b)=C(y,x) P(x)^-1 C(x,b); the conditional bias residual is PSD.
      Matrix6 propagated_prior_cross = Matrix6::Zero();
      for (int i = 0; i < 12; ++i) {
        const State6 initial =
            (i < 6 ? 1. : -1.) * std::sqrt(6.) * root.col(i % 6);
        propagated_prior_cross.noalias() +=
            (sigma[i] - mean) * initial.transpose() / 12.;
      }
      state_bias_covariance_ = std::exp(-dt / config_.measurement_bias_tau_s) *
                               propagated_prior_cross *
                               prior_covariance.ldlt().solve(prior_cross);
      ++belief_.numerical_cross_repairs;
      if (!joint_psd(next_covariance, state_bias_covariance_, bias_covariance_))
        throw std::runtime_error("invalid cubature joint covariance");
    }
    if (!mean.allFinite())
      throw std::runtime_error("nonfinite propagated mean");
    belief_.mean = mean;
    belief_.covariance = next_covariance;
    belief_.source_time_s += dt;
    if (audit_sink_) {
      audit->predicted = export_checkpoint();
      audit->process_noise = process_covariance(dt, config_.acceleration_spectral_density);
      audit->bounce_noise.bottomRightCorner<3, 3>().diagonal() =
          (contacts / 12.) * config_.bounce_velocity_std.array().square().matrix();
      // A diagnostic consumer failure must never reset or change the tracker.
      try { audit_sink_(*audit); }
      catch (...) { ++audit_failures_; }
    }
  }
  belief_.source_time_s = time;
}
TrackerUpdate ProbabilisticBallTracker::advance_time_checked(double time) {
  if (!belief_.initialized || !std::isfinite(time) ||
      time <= belief_.source_time_s)
    return {};
  if (time - belief_.last_measurement_time_s > config_.max_tracking_gap_s) {
    reset();
    return {true, false, false, true, false, "tracking_timeout"};
  }
  propagate(time);
  return {true, false, false, false, false, "gap_propagation"};
}
TrackerUpdate ProbabilisticBallTracker::observe_checked(const BallSample &s) {
  if (!std::isfinite(s.source_time_s) || !s.position.allFinite())
    return {false, false, false, false, true, "nonfinite_measurement"};
  if (!belief_.initialized) {
    initialize(s);
    return {true, true, true, false, false, "initial_observation"};
  }
  double dt = s.source_time_s - belief_.source_time_s;
  if (dt < -config_.clock_reset_backstep_s) {
    initialize(s);
    return {true, true, true, false, false, "source_epoch_reset"};
  }
  if (s.source_time_s <= belief_.last_measurement_time_s || dt < -1e-12)
    return {false, false, false, false, true, "duplicate_or_reordered"};
  if (s.source_time_s - belief_.last_measurement_time_s >
      config_.max_tracking_gap_s) {
    initialize(s);
    return {true, true, true, false, false, "tracking_gap_restart"};
  }
  propagate(s.source_time_s);
  belief_.source_time_ns = s.source_time_ns;
  belief_.receipt_steady_ns = s.receipt_steady_ns;
  belief_.receipt_wall_ns = s.receipt_wall_ns;
  if (config_.measurement_bias_std_m.maxCoeff() > 0.) {
    const Vec3 innovation = s.position - belief_.mean.head<3>() - bias_mean_;
    const Matrix3 R =
        config_.measurement_std_m.array().square().matrix().asDiagonal();
    const Matrix3 S = belief_.covariance.topLeftCorner<3, 3>() +
                      state_bias_covariance_.topRows<3>() +
                      state_bias_covariance_.topRows<3>().transpose() +
                      bias_covariance_ + R;
    const auto solve = S.ldlt();
    if (solve.info() != Eigen::Success || !S.allFinite() || !solve.isPositive())
      throw std::runtime_error("invalid innovation covariance");
    belief_.innovation_squared = innovation.dot(solve.solve(innovation));
    innovation_sum_squared_ += innovation.squaredNorm();
    belief_.innovation_max_m =
        std::max(belief_.innovation_max_m, innovation.norm());
    belief_.innovation_rms_m = std::sqrt(
        innovation_sum_squared_ /
        double(belief_.observation_count + belief_.rejected_measurements));
    if (belief_.innovation_squared > config_.innovation_gate_squared) {
      ++belief_.rejected_measurements;
      if (++consecutive_rejections_ >= config_.reacquire_rejections) {
        initialize(s);
        return {true, true, true, false, true, "reacquired_after_rejections"};
      }
      return {true, false, false, false, true, "innovation_outlier"};
    }
    using Matrix9 = Eigen::Matrix<double, 9, 9>;
    using Matrix93 = Eigen::Matrix<double, 9, 3>;
    using Matrix39 = Eigen::Matrix<double, 3, 9>;
    Matrix9 joint = Matrix9::Zero();
    joint.topLeftCorner<6, 6>() = belief_.covariance;
    joint.topRightCorner<6, 3>() = state_bias_covariance_;
    joint.bottomLeftCorner<3, 6>() = state_bias_covariance_.transpose();
    joint.bottomRightCorner<3, 3>() = bias_covariance_;
    Matrix39 H = Matrix39::Zero();
    H.leftCols<3>() = Matrix3::Identity();
    H.rightCols<3>() = Matrix3::Identity();
    const Matrix93 gain =
        joint * H.transpose() * solve.solve(Matrix3::Identity());
    belief_.mean += gain.topRows<6>() * innovation;
    bias_mean_ += gain.bottomRows<3>() * innovation;
    const Matrix9 A = Matrix9::Identity() - gain * H;
    joint = A * joint * A.transpose() + gain * R * gain.transpose();
    joint = .5 * (joint + joint.transpose()).eval();
    belief_.covariance = positive_covariance(joint.topLeftCorner<6, 6>());
    state_bias_covariance_ = joint.topRightCorner<6, 3>();
    bias_covariance_ = joint.bottomRightCorner<3, 3>();
    belief_.last_measurement_gap_s =
        s.source_time_s - belief_.last_measurement_time_s;
    belief_.last_measurement_time_s = s.source_time_s;
    belief_.source_time_ns = belief_.last_measurement_time_ns =
        s.source_time_ns;
    belief_.receipt_steady_ns = s.receipt_steady_ns;
    belief_.receipt_wall_ns = s.receipt_wall_ns;
    ++belief_.observation_count;
    consecutive_rejections_ = 0;
    return {true, true, false, false, false, "correlated_measurement_update"};
  }
  const Vec3 innovation = s.position - belief_.mean.head<3>();
  const Matrix3 R =
      config_.measurement_std_m.array().square().matrix().asDiagonal();
  const Matrix3 S = belief_.covariance.topLeftCorner<3, 3>() + R;
  const auto solve = S.ldlt();
  belief_.innovation_squared = innovation.dot(solve.solve(innovation));
  innovation_sum_squared_ += innovation.squaredNorm();
  belief_.innovation_max_m =
      std::max(belief_.innovation_max_m, innovation.norm());
  belief_.innovation_rms_m = std::sqrt(
      innovation_sum_squared_ /
      double(belief_.observation_count + belief_.rejected_measurements));
  if (belief_.innovation_squared > config_.innovation_gate_squared) {
    ++belief_.rejected_measurements;
    if (++consecutive_rejections_ >= config_.reacquire_rejections) {
      initialize(s);
      return {true, true, true, false, true, "reacquired_after_rejections"};
    }
    return {true, false, false, false, true, "innovation_outlier"};
  }
  const Eigen::Matrix<double, 6, 3> gain =
      belief_.covariance.leftCols<3>() * solve.solve(Matrix3::Identity());
  belief_.mean += gain * innovation;
  Matrix6 A = Matrix6::Identity();
  A.leftCols<3>() -= gain;
  belief_.covariance = positive_covariance(
      A * belief_.covariance * A.transpose() + gain * R * gain.transpose());
  belief_.last_measurement_gap_s =
      s.source_time_s - belief_.last_measurement_time_s;
  belief_.last_measurement_time_s = s.source_time_s;
  belief_.source_time_ns = belief_.last_measurement_time_ns = s.source_time_ns;
  belief_.receipt_steady_ns = s.receipt_steady_ns;
  belief_.receipt_wall_ns = s.receipt_wall_ns;
  ++belief_.observation_count;
  consecutive_rejections_ = 0;
  return {true, true, false, false, false, "measurement_update"};
}
TrackerUpdate ProbabilisticBallTracker::observe(const BallSample &s) {
  try {
    auto result = observe_checked(s);
    if (belief_.initialized &&
        (!belief_.mean.allFinite() ||
         !joint_psd(belief_.covariance, state_bias_covariance_,
                    bias_covariance_)))
      throw std::runtime_error("invalid posterior joint covariance");
    return result;
  } catch (const std::exception &) {
    reset();
    return {true, false, false, true, true, "numerical_failure"};
  }
}
TrackerUpdate ProbabilisticBallTracker::advance_time(double time) {
  try {
    return advance_time_checked(time);
  } catch (const std::exception &) {
    reset();
    return {true, false, false, true, false, "numerical_failure"};
  }
}
} // namespace hope_planner_cpp
