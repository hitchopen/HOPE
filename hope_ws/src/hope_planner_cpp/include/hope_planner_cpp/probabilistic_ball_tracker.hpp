#pragma once
#include "hope_planner_cpp/fixed_lag_types.hpp"
#include <array>
#include <functional>
namespace hope_planner_cpp {
// Optional read-only offline diagnostics. A null sink leaves the original
// arithmetic and estimator behavior unchanged. No truth enters this interface.
struct CubatureMemberAudit {
  State6 initial = State6::Zero(), final = State6::Zero();
  State6 contact_pre = State6::Zero(), contact_post = State6::Zero();
  double contact_offset_s = NAN;
  unsigned impulses = 0;
};
struct PropagationAudit {
  FullTrackerCheckpoint before, predicted;
  std::array<CubatureMemberAudit, 12> members;
  State6 nominal = State6::Zero();
  double duration_s = 0., nominal_contact_offset_s = NAN;
  Matrix6 process_noise = Matrix6::Zero(), bounce_noise = Matrix6::Zero();
};
// Positive-weight spherical-radial cubature filter (sigma-point UKF family).
// Keeps Gaussian state and correlated-bias moments for the nominal forecast.
class ProbabilisticBallTracker {
public:
  explicit ProbabilisticBallTracker(ProbabilisticConfig config);
  TrackerUpdate observe(const BallSample &sample);
  TrackerUpdate advance_time(double source_time_s);
  void reset();
  void set_effective_spin(const EffectiveSpin& spin) {
    config_.effective_spin=spin;
    if(!belief_.effective_spin.usable() || !spin.usable() || spin.measurement_time_ns>belief_.effective_spin.measurement_time_ns ||
       spin.last_contact_id>belief_.effective_spin.last_contact_id)belief_.effective_spin=spin;
  }
  void set_propagation_audit_sink(std::function<void(const PropagationAudit &)> sink) {
    audit_sink_ = std::move(sink);
  }
  std::size_t propagation_audit_failures() const { return audit_failures_; }
  FullTrackerCheckpoint export_checkpoint() const;
  void restore_checkpoint(const FullTrackerCheckpoint &);
  const StateBelief &belief() const { return belief_; }

private:
  TrackerUpdate observe_checked(const BallSample &);
  TrackerUpdate advance_time_checked(double);
  void initialize(const BallSample &sample);
  void propagate(double source_time_s);
  std::function<void(const PropagationAudit &)> audit_sink_;
  std::size_t audit_failures_ = 0;
  ProbabilisticConfig config_;
  StateBelief belief_;
  Vec3 bias_mean_ = Vec3::Zero();
  Eigen::Matrix<double, 6, 3> state_bias_covariance_ =
      Eigen::Matrix<double, 6, 3>::Zero();
  Matrix3 bias_covariance_ = Matrix3::Zero();
  std::size_t consecutive_rejections_ = 0;
  double innovation_sum_squared_ = 0.;
};
} // namespace hope_planner_cpp
