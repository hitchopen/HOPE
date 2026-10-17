#include "hope_planner_cpp/probabilistic_physics.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace hope_planner_cpp {
const char *name(RallyContext c) {
  switch (c) {
  case RallyContext::kServeReturn:
    return "serve_return";
  case RallyContext::kNormalReturn:
    return "normal_return";
  default:
    return "unknown";
  }
}
const char *name(BallMotionState c) {
  switch (c) {
  case BallMotionState::kIncoming:
    return "incoming";
  case BallMotionState::kOutgoing:
    return "outgoing";
  default:
    return "uncertain";
  }
}
const char *name(PlannerBallState s) {
  switch (s) {
  case PlannerBallState::kServe: return "SERVE";
  case PlannerBallState::kReturnServe: return "RETURN_SERVE";
  case PlannerBallState::kIncoming: return "INCOMING";
  case PlannerBallState::kOutgoing: return "OUTGOING";
  }
  return "OUTGOING";
}
void ProbabilisticConfig::validate() const {
  const double values[]{
      integration_dt_s,        trajectory_dt_s,   horizon_s,
      max_tracking_gap_s,      gap_warning_s,     clock_reset_backstep_s,
      innovation_gate_squared, direction_window_s,
      measurement_bias_tau_s};
  for (double v : values)
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument(
          "nonpositive/nonfinite probabilistic configuration");
  if (!measurement_bias_std_m.allFinite() ||
      (measurement_bias_std_m.array() < 0).any() ||
      !measurement_std_m.allFinite() ||
      (measurement_std_m.array() <= 0).any() ||
      !velocity_prior_std.allFinite() ||
      (velocity_prior_std.array() <= 0).any() ||
      !velocity_prior_mean.allFinite() || !bounce_velocity_std.allFinite() ||
      (bounce_velocity_std.array() < 0).any() ||
      !std::isfinite(acceleration_spectral_density) ||
      acceleration_spectral_density < 0 || !std::isfinite(x_hit) ||
      !std::isfinite(min_tracking_z_m) || !std::isfinite(direction_sigma) ||
      direction_sigma < 0 || !std::isfinite(direction_speed_mps) ||
      direction_speed_mps < 0 || ensemble_size < 12 || ensemble_size > 1024 ||
      direction_confirmations < 1 || reacquire_rejections < 1 ||
      max_rollout_bounces < 1 || integration_dt_s > .01 ||
      trajectory_dt_s < integration_dt_s || horizon_s > 10 ||
      normal_expected_bounces < 0 || serve_expected_bounces < 0 ||
      !std::isfinite(physics.drag_k) || physics.drag_k < 0 ||
      !physics.gravity.allFinite() || !std::isfinite(physics.radius) ||
      physics.radius <= 0 || !std::isfinite(physics.restitution_v) ||
      physics.restitution_v < 0 || physics.restitution_v > 1.5 ||
      !std::isfinite(physics.table_tangential_gain) ||
      physics.table_tangential_gain < 0 || physics.table_tangential_gain > 1 ||
      !std::isfinite(physics.table_friction_cap_mu) ||
      physics.table_friction_cap_mu < 0 || !std::isfinite(table.length) ||
      table.length <= 0 || !std::isfinite(table.width) || table.width <= 0 ||
      !std::isfinite(table.y_max))
    throw std::invalid_argument("invalid probabilistic parameters");
}
Matrix6 positive_covariance(const Matrix6 &p) {
  Eigen::SelfAdjointEigenSolver<Matrix6> eig((p + p.transpose()) * .5);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite())
    throw std::runtime_error("nonfinite covariance");
  return eig.eigenvectors() * eig.eigenvalues().cwiseMax(1e-12).asDiagonal() *
         eig.eigenvectors().transpose();
}
State6 free_flight(const State6 &s, double dt, const BallPhysics &p) {
  const Vec3 v = s.tail<3>();
  const Vec3 a = p.gravity - p.drag_k * v.norm() * v;
  const Vec3 vm = v + .5 * dt * a;
  State6 next;
  next.head<3>() = s.head<3>() + dt * vm;
  next.tail<3>() = v + dt * (p.gravity - p.drag_k * vm.norm() * vm);
  return next;
}
ContactStep physical_step(State6 &s, double dt, const ProbabilisticConfig &c) {
  ContactStep event;
  State6 next = free_flight(s, dt, c.physics);
  const double z = c.physics.radius;
  if (s[5] < 0 && next[2] <= z && s[2] >= 0.) {
    const double denom = s[2] - next[2];
    const double f =
        denom > 1e-12 ? std::clamp((s[2] - z) / denom, 0., 1.) : 0.;
    State6 hit = free_flight(s, f * dt, c.physics);
    if (hit[0] >= 0 && hit[0] <= c.table.length && hit[1] <= c.table.y_max &&
        hit[1] >= c.table.y_max - c.table.width) {
      hit[2] = z;
      const double speed = hit.segment<2>(3).norm();
      const double impulse =
          std::min(c.physics.table_tangential_gain * speed,
                   c.physics.table_friction_cap_mu *
                       (1 + c.physics.restitution_v) * std::abs(hit[5]));
      if (speed > 1e-12)
        hit.segment<2>(3) *= (1 - impulse / speed);
      hit[5] *= -c.physics.restitution_v;
      event = {true, f, hit};
      next = free_flight(hit, (1 - f) * dt, c.physics);
    }
  }
  s = next;
  return event;
}
Matrix6 process_covariance(double dt, double q) {
  Matrix6 p = Matrix6::Zero();
  p.topLeftCorner<3, 3>().diagonal().setConstant(q * dt * dt * dt / 3.);
  p.bottomRightCorner<3, 3>().diagonal().setConstant(q * dt);
  p.topRightCorner<3, 3>().diagonal().setConstant(q * dt * dt / 2.);
  p.bottomLeftCorner<3, 3>() = p.topRightCorner<3, 3>();
  return p;
}
} // namespace hope_planner_cpp
