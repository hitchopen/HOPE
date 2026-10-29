#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/ball_dynamics.hpp"
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>

namespace hope_planner_cpp {

RacketTargetPlanner::RacketTargetPlanner(
    BallPhysics physics, PlannerConfig config, TableParams table)
    : physics_(std::move(physics)), config_(std::move(config)), table_(std::move(table)) {}

bool RacketTargetPlanner::valid_parameters() const noexcept {
  const std::array<double, 22> values{
      physics_.drag_k, physics_.magnus_k, physics_.radius,
      config_.ball_inertia_coeff, config_.paddle_tangential_gain,
      config_.paddle_angle_gain, config_.paddle_friction_cap_mu,
      config_.restitution_racket, config_.restitution_exp_g1, config_.restitution_exp_g2,
      config_.return_restitution_min, config_.return_restitution_max,
      config_.return_dt_integrate, config_.return_numeric_epsilon,
      config_.max_predict_time_s, config_.return_fd_relative_step,
      config_.return_position_tolerance_m, config_.return_time_tolerance_s,
      config_.return_max_step_mps, config_.return_net_clearance_margin,
      table_.net_height, table_.net_overhang};
  for (double value : values) if (!std::isfinite(value)) return false;
  for (double time : config_.return_alternative_flight_times)
    if (!std::isfinite(time) || time <= 0.0) return false;
  return physics_.gravity.allFinite() && physics_.drag_k >= 0.0 && physics_.radius > 0.0 &&
      config_.ball_inertia_coeff > 0.0 && config_.paddle_friction_cap_mu >= 0.0 &&
      config_.restitution_racket > -1.0 && config_.return_restitution_min >= 0.0 &&
      config_.return_restitution_max >= config_.return_restitution_min &&
      config_.return_dt_integrate > 0.0 && config_.return_numeric_epsilon > 0.0 &&
      config_.max_predict_time_s > 0.0 && config_.return_fd_relative_step > 0.0 &&
      config_.return_position_tolerance_m > 0.0 && config_.return_time_tolerance_s > 0.0 &&
      config_.return_max_step_mps > 0.0 && config_.return_net_clearance_margin >= 0.0 &&
      config_.return_newton_iterations > 0 && config_.return_line_search_iterations > 0 &&
      config_.return_seed_iterations >= 0 && std::isfinite(table_.width) && table_.width > 0.0 &&
      std::isfinite(table_.net_x) && std::isfinite(table_.y_max);
}

void RacketTargetPlanner::rk4_step(Vec3& position, Vec3& velocity,
                                  const Vec3& omega, double h) const noexcept {
  const Vec3 a1 = ball_acceleration(physics_, velocity, omega);
  const Vec3 v2 = velocity + 0.5 * h * a1;
  const Vec3 a2 = ball_acceleration(physics_, v2, omega);
  const Vec3 v3 = velocity + 0.5 * h * a2;
  const Vec3 a3 = ball_acceleration(physics_, v3, omega);
  const Vec3 v4 = velocity + h * a3;
  const Vec3 a4 = ball_acceleration(physics_, v4, omega);
  position += (h / 6.0) * (velocity + 2.0 * v2 + 2.0 * v3 + v4);
  velocity += (h / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
}

ReturnPrediction RacketTargetPlanner::evaluate_return(
    const StrikeTarget& strike, const Vec3& racket_velocity) const noexcept {
  ReturnPrediction result;
  const double epsilon = config_.return_numeric_epsilon;
  if (!valid_parameters() || !strike.ball_position.allFinite() ||
      !strike.ball_velocity.allFinite() || !strike.ball_spin_rad_s.allFinite() ||
      !racket_velocity.allFinite() || racket_velocity.norm() <= epsilon ||
      strike.ball_position.z() <= physics_.radius) return result;
  // Velocity-normal ABI specifies the face plane; contact chooses its incoming side.
  result.racket_normal = racket_velocity / (racket_velocity.norm() + epsilon);
  Vec3 normal = result.racket_normal;
  if ((strike.ball_velocity - racket_velocity).dot(normal) > 0.0) normal = -normal;
  const Vec3 relative = strike.ball_velocity +
      strike.ball_spin_rad_s.cross(-physics_.radius * normal) - racket_velocity;
  const double normal_speed = relative.dot(normal);
  const Vec3 tangent = relative - normal_speed * normal;
  const double tangent_speed = tangent.norm();
  const double restitution = std::clamp(
      config_.restitution_exp_g1 * std::exp(config_.restitution_exp_g2 * std::abs(normal_speed)),
      config_.return_restitution_min, config_.return_restitution_max);
  const double cosine = std::abs(normal_speed) /
      (std::hypot(tangent_speed, normal_speed) + epsilon);
  const double impulse = std::min(
      std::max(0.0, (config_.paddle_tangential_gain + config_.paddle_angle_gain * cosine) * tangent_speed),
      config_.paddle_friction_cap_mu * (1.0 + restitution) * std::abs(normal_speed));
  Vec3 tangent_delta = Vec3::Zero();
  if (tangent_speed > epsilon) tangent_delta = -impulse * tangent / (tangent_speed + epsilon);
  result.outgoing_velocity = strike.ball_velocity -
      (1.0 + restitution) * normal_speed * normal + tangent_delta;
  result.outgoing_spin_rad_s = strike.ball_spin_rad_s - normal.cross(tangent_delta) /
      (config_.ball_inertia_coeff * physics_.radius);
  Vec3 position = strike.ball_position;
  Vec3 velocity = result.outgoing_velocity;
  if (!velocity.allFinite() || !result.outgoing_spin_rad_s.allFinite()) return result;
  double elapsed = 0.0;
  while (config_.max_predict_time_s - elapsed > 1.0e-12) {
    const double h = std::min(config_.return_dt_integrate, config_.max_predict_time_s - elapsed);
    Vec3 next_position = position, next_velocity = velocity;
    rk4_step(next_position, next_velocity, result.outgoing_spin_rad_s, h);
    if (!next_position.allFinite() || !next_velocity.allFinite()) return result;
    const bool lands = position.z() > physics_.radius && next_position.z() <= physics_.radius;
    const double landing_fraction = lands
        ? std::clamp((position.z() - physics_.radius) / (position.z() - next_position.z()), 0.0, 1.0)
        : 1.0;
    if (!result.net_crossed && position.x() < table_.net_x && next_position.x() >= table_.net_x) {
      const double fraction = (table_.net_x - position.x()) / (next_position.x() - position.x());
      if (fraction <= landing_fraction) {
        result.net_position = position + fraction * (next_position - position);
        result.net_crossed = true;
      }
    }
    if (lands) {
      result.bounce_position = position + landing_fraction * (next_position - position);
      result.bounce_position.z() = physics_.radius;
      result.bounce_velocity = velocity + landing_fraction * (next_velocity - velocity);
      result.flight_time_s = elapsed + landing_fraction * h;
      result.valid = true;
      return result;
    }
    position = next_position;
    velocity = next_velocity;
    elapsed += h;
  }
  return result;
}

Vec3 RacketTargetPlanner::initial_racket_velocity(
    const StrikeTarget& strike, const Vec3& target_land, double duration_s) const noexcept {
  const Vec3 target(target_land.x(), target_land.y(), physics_.radius);
  const Vec3 outgoing = (target - strike.ball_position) / duration_s - 0.5 * physics_.gravity * duration_s;
  const Vec3 delta = outgoing - strike.ball_velocity;
  if (!delta.allFinite() || delta.norm() < 1.0e-6) return Vec3::Zero();
  Vec3 normal = delta.normalized();
  if (normal.x() < 0.0) normal = -normal;
  if (normal.x() <= 1.0e-6) normal = (normal + Vec3::UnitX()).normalized();
  const double incoming_n = strike.ball_velocity.dot(normal), outgoing_n = outgoing.dot(normal);
  double restitution = config_.restitution_racket;
  double speed = (outgoing_n + restitution * incoming_n) / (1.0 + restitution);
  for (int i = 0; i < config_.return_seed_iterations; ++i) {
    restitution = std::clamp(config_.restitution_exp_g1 *
        std::exp(config_.restitution_exp_g2 * std::abs(incoming_n - speed)),
        config_.return_restitution_min, config_.return_restitution_max);
    speed = (outgoing_n + restitution * incoming_n) / (1.0 + restitution);
  }
  return speed * normal;
}

bool RacketTargetPlanner::solve_racket(const StrikeTarget& strike, const Vec3& target_land,
                                      double duration_s, Vec3& racket_velocity,
                                      ReturnPrediction& prediction) const noexcept {
  if (!std::isfinite(duration_s) || duration_s <= 0.0 || !target_land.allFinite()) return false;
  racket_velocity = initial_racket_velocity(strike, target_land, duration_s);
  const auto residual = [&](const ReturnPrediction& p) -> Vec3 {
    return Vec3(p.bounce_position.x() - target_land.x(),
                p.bounce_position.y() - target_land.y(), p.flight_time_s - duration_s);
  };
  const auto converged = [&](const Vec3& error) {
    return error.head<2>().norm() <= config_.return_position_tolerance_m &&
        std::abs(error.z()) <= config_.return_time_tolerance_s;
  };
  const Vec3 scales(config_.return_position_tolerance_m, config_.return_position_tolerance_m,
                    config_.return_time_tolerance_s);
  for (int iteration = 0; iteration < config_.return_newton_iterations; ++iteration) {
    prediction = evaluate_return(strike, racket_velocity);
    if (!prediction.valid) return false;
    const Vec3 error = residual(prediction);
    if (converged(error)) return true;
    Eigen::Matrix3d jacobian;
    for (int axis = 0; axis < 3; ++axis) {
      Vec3 perturbed = racket_velocity;
      const double epsilon = config_.return_fd_relative_step * std::max(std::abs(perturbed[axis]), 1.0);
      perturbed[axis] += epsilon;
      const ReturnPrediction candidate = evaluate_return(strike, perturbed);
      if (!candidate.valid) return false;
      jacobian.col(axis) = (residual(candidate) - error) / epsilon;
    }
    const Eigen::FullPivLU<Eigen::Matrix3d> lu(jacobian);
    if (!lu.isInvertible()) return false;
    Vec3 step = lu.solve(error);
    if (!step.allFinite()) return false;
    if (step.norm() > config_.return_max_step_mps) step *= config_.return_max_step_mps / step.norm();
    bool accepted = false;
    double alpha = 1.0;
    const double score = error.cwiseQuotient(scales).norm();
    for (int attempt = 0; attempt < config_.return_line_search_iterations; ++attempt) {
      const Vec3 trial = racket_velocity - alpha * step;
      const ReturnPrediction candidate = evaluate_return(strike, trial);
      if (candidate.valid && residual(candidate).cwiseQuotient(scales).norm() < score) {
        racket_velocity = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) return false;
  }
  prediction = evaluate_return(strike, racket_velocity);
  return prediction.valid && converged(residual(prediction));
}

void RacketTargetPlanner::check_net_clearance(const ReturnPrediction& prediction,
                                            bool& clears_net, bool& bypasses_posts) const noexcept {
  clears_net = false;
  bypasses_posts = false;
  if (!prediction.net_crossed) return;
  bypasses_posts = prediction.net_position.y() < table_.y_max - table_.width - table_.net_overhang ||
      prediction.net_position.y() > table_.y_max + table_.net_overhang;
  clears_net = !bypasses_posts && prediction.net_position.z() >
      table_.net_height + physics_.radius + config_.return_net_clearance_margin;
}

RacketCommand RacketTargetPlanner::plan(const StrikeTarget& strike, const Vec3& target_land,
                                      double delta_t_flight_s) const noexcept {
  RacketCommand command;
  command.position = strike.ball_position;
  command.strike_source_time_s = strike.strike_source_time_s;
  command.target_land = target_land;
  command.predicted_bounces = strike.predicted_bounces;
  if (!strike.valid) { command.reason = strike.reason; return command; }
  if (!valid_parameters()) { command.reason = "full_return_parameters_invalid"; return command; }
  if (!strike.ball_position.allFinite() || !strike.ball_velocity.allFinite() ||
      !strike.ball_spin_rad_s.allFinite() || !std::isfinite(strike.strike_source_time_s) ||
      !target_land.allFinite() || !std::isfinite(delta_t_flight_s) || delta_t_flight_s <= 0.0) {
    command.reason = "full_return_input_invalid";
    return command;
  }
  bool found = false;
  for (std::size_t attempt = 0; attempt <= config_.return_alternative_flight_times.size(); ++attempt) {
    const double duration = attempt == 0 ? delta_t_flight_s : config_.return_alternative_flight_times[attempt - 1];
    Vec3 racket_velocity;
    ReturnPrediction prediction;
    if (!solve_racket(strike, target_land, duration, racket_velocity, prediction)) continue;
    bool clear = false, bypass = false;
    check_net_clearance(prediction, clear, bypass);
    if (!found || clear) {
      command.velocity = racket_velocity;
      command.normal = prediction.racket_normal;
      command.outgoing_ball_velocity = prediction.outgoing_velocity;
      command.outgoing_ball_spin_rad_s = prediction.outgoing_spin_rad_s;
      command.predicted_landing = prediction.bounce_position;
      command.return_flight_time_s = prediction.flight_time_s;
      command.clears_net = clear;
      command.bypasses_net_posts = bypass;
      command.valid = true;
      command.reason = "command_valid";
      found = true;
    }
    if (clear) break;
  }
  if (!found) command.reason = "full_return_solve_failed";
  return command;
}

}  // namespace hope_planner_cpp
