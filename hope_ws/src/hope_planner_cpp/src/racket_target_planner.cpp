#include "hope_planner_cpp/racket_target_planner.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>

namespace hope_planner_cpp {

RacketTargetPlanner::RacketTargetPlanner(
    BallPhysics physics, PlannerConfig config, TableParams table)
    : physics_(std::move(physics)), config_(std::move(config)), table_(std::move(table)) {}

bool RacketTargetPlanner::integrate_free_flight(
    const Vec3& initial_position,
    const Vec3& initial_velocity,
    double duration_s,
    Vec3& final_position,
    Vec3& final_velocity) const noexcept {
  if (!initial_position.allFinite() || !initial_velocity.allFinite() ||
      !std::isfinite(duration_s) || duration_s < 0.0) {
    return false;
  }
  Vec3 position = initial_position;
  Vec3 velocity = initial_velocity;
  double remaining = duration_s;
  while (remaining > 1.0e-12) {
    const double h = std::min(config_.integrate_dt_s, remaining);
    const Vec3 acceleration =
        -physics_.drag_k * velocity.norm() * velocity + physics_.gravity;
    position += velocity * h + 0.5 * acceleration * h * h;
    velocity += acceleration * h;
    remaining -= h;
    if (!position.allFinite() || !velocity.allFinite()) {
      return false;
    }
  }
  final_position = position;
  final_velocity = velocity;
  return true;
}

bool RacketTargetPlanner::integrate_free_flight_batch(
    const Vec3& initial_position,
    const std::array<Vec3, 4>& initial_velocity,
    double duration_s,
    std::array<Vec3, 4>& final_position,
    std::array<Vec3, 4>& final_velocity) const noexcept {
  if (!initial_position.allFinite() || !std::isfinite(duration_s) || duration_s < 0.0) {
    return false;
  }
  final_position.fill(initial_position);
  final_velocity = initial_velocity;
  double remaining = duration_s;
  while (remaining > 1.0e-12) {
    const double h = std::min(config_.integrate_dt_s, remaining);
    for (std::size_t i = 0; i < final_velocity.size(); ++i) {
      const Vec3 acceleration =
          -physics_.drag_k * final_velocity[i].norm() * final_velocity[i] + physics_.gravity;
      final_position[i] += final_velocity[i] * h + 0.5 * acceleration * h * h;
      final_velocity[i] += acceleration * h;
      if (!final_position[i].allFinite() || !final_velocity[i].allFinite()) {
        return false;
      }
    }
    remaining -= h;
  }
  return true;
}

bool RacketTargetPlanner::compute_outgoing_velocity(
    const Vec3& strike_position,
    const Vec3& target_land,
    double flight_time_s,
    Vec3& outgoing_velocity) const noexcept {
  if (!strike_position.allFinite() || !target_land.allFinite() ||
      !std::isfinite(flight_time_s) || flight_time_s <= 0.0) {
    return false;
  }
  Vec3 velocity =
      (target_land - strike_position) / flight_time_s -
      0.5 * physics_.gravity * flight_time_s;
  if (physics_.drag_k == 0.0) {
    outgoing_velocity = velocity;
    return velocity.allFinite();
  }

  for (int iteration = 0; iteration < 12; ++iteration) {
    const Vec3 epsilon = 1.0e-4 * velocity.cwiseAbs().cwiseMax(Vec3::Ones());
    std::array<Vec3, 4> velocity_batch{velocity, velocity, velocity, velocity};
    for (int axis = 0; axis < 3; ++axis) {
      velocity_batch[static_cast<std::size_t>(axis + 1)][axis] += epsilon[axis];
    }
    std::array<Vec3, 4> final_positions{};
    std::array<Vec3, 4> final_velocities{};
    if (!integrate_free_flight_batch(
            strike_position, velocity_batch, flight_time_s,
            final_positions, final_velocities)) {
      return false;
    }
    const Vec3 residual = final_positions[0] - target_land;
    if (residual.norm() < 1.0e-5) {
      outgoing_velocity = velocity;
      return true;
    }
    Eigen::Matrix3d jacobian;
    for (int axis = 0; axis < 3; ++axis) {
      jacobian.col(axis) =
          (final_positions[static_cast<std::size_t>(axis + 1)] - final_positions[0]) /
          epsilon[axis];
    }
    const Eigen::FullPivLU<Eigen::Matrix3d> lu(jacobian);
    if (!lu.isInvertible()) {
      return false;
    }
    const Vec3 step = lu.solve(residual);
    if (!step.allFinite()) {
      return false;
    }
    velocity -= step;
    if (!velocity.allFinite()) {
      return false;
    }
  }
  outgoing_velocity = velocity;
  return velocity.allFinite();
}

void RacketTargetPlanner::compute_racket_velocity(
    const Vec3& incoming,
    const Vec3& outgoing,
    Vec3& racket_velocity,
    Vec3& racket_normal) const noexcept {
  const Vec3 delta = outgoing - incoming;
  if (!delta.allFinite() || delta.norm() < 1.0e-6) {
    racket_velocity.setZero();
    racket_normal = Vec3(1.0, 0.0, 0.0);
    return;
  }
  racket_normal = delta.normalized();
  if (racket_normal.x() < 0.0) {
    racket_normal = -racket_normal;
  }
  if (racket_normal.x() <= 1.0e-6) {
    racket_normal += Vec3(1.0, 0.0, 0.0);
    racket_normal.normalize();
  }

  const double outgoing_normal = outgoing.dot(racket_normal);
  const double incoming_normal = incoming.dot(racket_normal);
  double restitution = config_.restitution_racket;
  double racket_normal_speed =
      (outgoing_normal + restitution * incoming_normal) / (1.0 + restitution);
  for (int iteration = 0; iteration < 3; ++iteration) {
    const double approach = std::abs(incoming_normal - racket_normal_speed);
    restitution = std::clamp(
        config_.restitution_exp_g1 *
            std::exp(config_.restitution_exp_g2 * approach),
        0.2, 0.95);
    racket_normal_speed =
        (outgoing_normal + restitution * incoming_normal) / (1.0 + restitution);
  }
  racket_velocity = racket_normal_speed * racket_normal;
}

bool RacketTargetPlanner::free_flight_position_at_x(
    const Vec3& initial_position,
    const Vec3& initial_velocity,
    double target_x,
    Vec3& position_at_x) const noexcept {
  if (std::abs(initial_position.x() - target_x) <= 1.0e-9) {
    position_at_x = initial_position;
    return true;
  }
  const double direction = target_x > initial_position.x() ? 1.0 : -1.0;
  if (direction * initial_velocity.x() <= 0.0) {
    return false;
  }
  Vec3 position = initial_position;
  Vec3 velocity = initial_velocity;
  const int max_steps = static_cast<int>(config_.max_predict_time_s / config_.integrate_dt_s);
  for (int step = 0; step < max_steps; ++step) {
    const Vec3 acceleration =
        -physics_.drag_k * velocity.norm() * velocity + physics_.gravity;
    const Vec3 next_position =
        position + velocity * config_.integrate_dt_s +
        0.5 * acceleration * config_.integrate_dt_s * config_.integrate_dt_s;
    const Vec3 next_velocity = velocity + acceleration * config_.integrate_dt_s;
    if ((position.x() - target_x) * (next_position.x() - target_x) <= 0.0) {
      const double dx = next_position.x() - position.x();
      const double fraction = std::clamp(
          std::abs(dx) < 1.0e-12 ? 0.0 : (target_x - position.x()) / dx,
          0.0, 1.0);
      position_at_x = position + fraction * (next_position - position);
      return position_at_x.allFinite();
    }
    position = next_position;
    velocity = next_velocity;
  }
  return false;
}

void RacketTargetPlanner::check_net_clearance(
    const Vec3& strike_position,
    const Vec3& outgoing_velocity,
    bool& clears_net,
    bool& bypasses_posts) const noexcept {
  clears_net = false;
  bypasses_posts = false;
  if (outgoing_velocity.x() <= 0.0) {
    return;
  }
  Vec3 at_net;
  if (!free_flight_position_at_x(
          strike_position, outgoing_velocity, table_.net_x, at_net)) {
    return;
  }
  const double y_min = -table_.width - table_.net_overhang;
  const double y_max = table_.net_overhang;
  bypasses_posts = at_net.y() < y_min || at_net.y() > y_max;
  if (!bypasses_posts) {
    clears_net = at_net.z() > table_.net_height + 0.03;
  }
}

RacketCommand RacketTargetPlanner::plan(
    const StrikeTarget& strike,
    const Vec3& target_land,
    double delta_t_flight_s) const noexcept {
  RacketCommand command;
  command.position = strike.ball_position;
  command.strike_source_time_s = strike.strike_source_time_s;
  command.target_land = target_land;
  command.predicted_bounces = strike.predicted_bounces;
  if (!strike.valid) {
    command.reason = strike.reason;
    return command;
  }

  Vec3 outgoing;
  if (!compute_outgoing_velocity(
          strike.ball_position, target_land, delta_t_flight_s, outgoing)) {
    command.reason = "outgoing_velocity_solve_failed";
    return command;
  }
  Vec3 racket_velocity;
  Vec3 racket_normal;
  compute_racket_velocity(
      strike.ball_velocity, outgoing, racket_velocity, racket_normal);
  bool clears = false;
  bool bypasses = false;
  check_net_clearance(strike.ball_position, outgoing, clears, bypasses);

  if (!clears) {
    constexpr std::array<double, 5> alternatives{0.4, 0.6, 0.35, 0.7, 0.3};
    for (const double candidate_time : alternatives) {
      Vec3 candidate_outgoing;
      if (!compute_outgoing_velocity(
              strike.ball_position, target_land, candidate_time,
              candidate_outgoing)) {
        continue;
      }
      bool candidate_clears = false;
      bool candidate_bypasses = false;
      check_net_clearance(
          strike.ball_position, candidate_outgoing,
          candidate_clears, candidate_bypasses);
      if (candidate_clears) {
        outgoing = candidate_outgoing;
        compute_racket_velocity(
            strike.ball_velocity, outgoing, racket_velocity, racket_normal);
        clears = true;
        bypasses = candidate_bypasses;
        break;
      }
    }
  }

  command.velocity = racket_velocity;
  command.normal = racket_normal;
  command.outgoing_ball_velocity = outgoing;
  command.clears_net = clears;
  command.bypasses_net_posts = bypasses;
  command.valid = command.position.allFinite() && command.velocity.allFinite() &&
                  command.normal.allFinite() && outgoing.allFinite() &&
                  std::isfinite(command.strike_source_time_s);
  command.reason = command.valid ? "command_valid" : "stage3_nonfinite";
  return command;
}

}  // namespace hope_planner_cpp
