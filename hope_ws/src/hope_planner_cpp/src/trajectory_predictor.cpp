#include "hope_planner_cpp/trajectory_predictor.hpp"

#include <algorithm>
#include <cmath>

namespace hope_planner_cpp {

TrajectoryPredictor::TrajectoryPredictor(
    BallPhysics physics, PlannerConfig config, TableParams table)
    : physics_(std::move(physics)), config_(std::move(config)), table_(std::move(table)) {}

double TrajectoryPredictor::prediction_horizon_s(
    const BallState& state, double x_hit) const noexcept {
  const double base = std::max(0.0, config_.max_predict_time_s);
  if (!config_.adaptive_predict_horizon) {
    return base;
  }
  const double distance_x = state.position.x() - x_hit;
  const double incoming_speed_x = -state.velocity.x();
  if (distance_x <= 0.0 || incoming_speed_x <= 1.0e-6) {
    return base;
  }
  return std::max(base, config_.max_predict_time_cap_s);
}

StrikeTarget TrajectoryPredictor::predict(
    const BallState& state, double x_hit) const noexcept {
  StrikeTarget output;
  output.ball_position = state.position;
  output.ball_velocity = state.velocity;
  output.strike_source_time_s = state.source_time_s;
  output.reason = "prediction_running";
  if (!state.valid || !state.position.allFinite() || !state.velocity.allFinite() ||
      !std::isfinite(state.source_time_s) || !std::isfinite(x_hit)) {
    output.reason = "invalid_estimate";
    return output;
  }
  if (state.velocity.x() >= 0.0) {
    output.reason = "not_incoming";
    return output;
  }

  const double dt = config_.integrate_dt_s;
  if (!std::isfinite(dt) || dt <= 0.0) {
    output.reason = "invalid_integrator_dt";
    return output;
  }
  const double horizon = prediction_horizon_s(state, x_hit);
  const int max_steps = std::max(0, static_cast<int>(horizon / dt));
  const double half_dt_sq = 0.5 * dt * dt;
  const double table_y_lo = table_.y_max - table_.width;

  double px = state.position.x();
  double py = state.position.y();
  double pz = state.position.z();
  double vx = state.velocity.x();
  double vy = state.velocity.y();
  double vz = state.velocity.z();
  double time_s = state.source_time_s;
  int bounces = 0;

  for (int step = 0; step < max_steps; ++step) {
    const double previous_x = px;
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    const double ax = -physics_.drag_k * speed * vx + physics_.gravity.x();
    const double ay = -physics_.drag_k * speed * vy + physics_.gravity.y();
    const double az = -physics_.drag_k * speed * vz + physics_.gravity.z();

    double vx_new = vx + ax * dt;
    double vy_new = vy + ay * dt;
    double vz_new = vz + az * dt;
    double px_new = px + vx * dt + ax * half_dt_sq;
    double py_new = py + vy * dt + ay * half_dt_sq;
    double pz_new = pz + vz * dt + az * half_dt_sq;
    time_s += dt;

    bool bounce_this_step = false;
    double px_bounce = px;
    double py_bounce = py;
    double vx_post = vx;
    double vy_post = vy;
    double vz_post = vz;
    double remaining_dt = dt;

    if (pz_new < 0.0 && vz_new < 0.0) {
      const bool on_table =
          -physics_.radius <= px_new && px_new <= table_.length + physics_.radius &&
          table_y_lo - physics_.radius <= py_new &&
          py_new <= table_.y_max + physics_.radius;
      if (on_table) {
        const double dz = pz - pz_new;
        const double fraction = std::clamp(
            dz > 1.0e-9 ? pz / dz : 0.5, 0.0, 1.0);
        px_bounce = px + fraction * (px_new - px);
        py_bounce = py + fraction * (py_new - py);
        const double bounce_dt = fraction * dt;
        const double vx_at_bounce = vx + ax * bounce_dt;
        const double vy_at_bounce = vy + ay * bounce_dt;
        const double vz_at_bounce = vz + az * bounce_dt;
        vx_post = physics_.restitution_h * vx_at_bounce;
        vy_post = physics_.restitution_h * vy_at_bounce;
        vz_post = -physics_.restitution_v * vz_at_bounce;

        remaining_dt = (1.0 - fraction) * dt;
        const double post_speed = std::sqrt(
            vx_post * vx_post + vy_post * vy_post + vz_post * vz_post);
        const double ax_post = -physics_.drag_k * post_speed * vx_post + physics_.gravity.x();
        const double ay_post = -physics_.drag_k * post_speed * vy_post + physics_.gravity.y();
        const double az_post = -physics_.drag_k * post_speed * vz_post + physics_.gravity.z();
        const double half_remaining_sq = 0.5 * remaining_dt * remaining_dt;
        px_new = px_bounce + vx_post * remaining_dt + ax_post * half_remaining_sq;
        py_new = py_bounce + vy_post * remaining_dt + ay_post * half_remaining_sq;
        pz_new = vz_post * remaining_dt + az_post * half_remaining_sq;
        vx_new = vx_post + ax_post * remaining_dt;
        vy_new = vy_post + ay_post * remaining_dt;
        vz_new = vz_post + az_post * remaining_dt;
        ++bounces;
        bounce_this_step = true;
      } else {
        pz_new = std::max(0.0, pz_new);
      }
    }

    if (previous_x > x_hit && px_new <= x_hit && vx_new < 0.0) {
      double py_cross = 0.0;
      double pz_cross = 0.0;
      double vx_cross = 0.0;
      double vy_cross = 0.0;
      double vz_cross = 0.0;
      double t_cross = 0.0;
      if (bounce_this_step) {
        const double dx_arc = px_bounce - px_new;
        const double fraction = std::clamp(
            std::abs(dx_arc) > 1.0e-9 ? (px_bounce - x_hit) / dx_arc : 0.5,
            0.0, 1.0);
        py_cross = py_bounce + fraction * (py_new - py_bounce);
        pz_cross = fraction * pz_new;
        vx_cross = vx_post + fraction * (vx_new - vx_post);
        vy_cross = vy_post + fraction * (vy_new - vy_post);
        vz_cross = vz_post + fraction * (vz_new - vz_post);
        t_cross = time_s - remaining_dt + fraction * remaining_dt;
      } else {
        const double dx_step = px - px_new;
        const double fraction = std::clamp(
            std::abs(dx_step) > 1.0e-9 ? (px - x_hit) / dx_step : 0.5,
            0.0, 1.0);
        py_cross = py + fraction * (py_new - py);
        pz_cross = pz + fraction * (pz_new - pz);
        vx_cross = vx + fraction * (vx_new - vx);
        vy_cross = vy + fraction * (vy_new - vy);
        vz_cross = vz + fraction * (vz_new - vz);
        t_cross = time_s - dt + fraction * dt;
      }
      output.ball_position = Vec3(x_hit, py_cross, pz_cross);
      output.ball_velocity = Vec3(vx_cross, vy_cross, vz_cross);
      output.strike_source_time_s = t_cross;
      output.predicted_bounces = bounces;
      const bool dead_ball = pz_cross < 0.05 && vz_cross < 0.0;
      output.valid = !dead_ball;
      output.reason = dead_ball ? "dead_ball" : "prediction_valid";
      return output;
    }

    px = px_new;
    py = py_new;
    pz = pz_new;
    vx = vx_new;
    vy = vy_new;
    vz = vz_new;
  }

  output.ball_position = Vec3(px, py, pz);
  output.ball_velocity = Vec3(vx, vy, vz);
  output.strike_source_time_s = time_s;
  output.predicted_bounces = bounces;
  if (state.position.x() <= x_hit) {
    output.reason = "no_hit_plane_crossing";
  } else if (px > x_hit) {
    output.reason = "prediction_horizon_exceeded";
  } else {
    output.reason = "prediction_invalid";
  }
  return output;
}

Vec3 TrajectoryPredictor::flight_acceleration(
    const Vec3& velocity,
    const Vec3& omega_rad_s,
    bool use_magnus) const noexcept {
  Vec3 acceleration =
      -physics_.drag_k * velocity.norm() * velocity + physics_.gravity;
  if (use_magnus) {
    acceleration += physics_.magnus_k * omega_rad_s.cross(velocity);
  }
  return acceleration;
}

void TrajectoryPredictor::apply_nakashima_bounce(
    const Vec3& velocity_in,
    const Vec3& omega_in,
    Vec3* velocity_out,
    Vec3* omega_out) const noexcept {
  if (velocity_out == nullptr || omega_out == nullptr) {
    return;
  }
  const double radius = physics_.radius;
  if (!(radius > 0.0)) {
    *velocity_out = Vec3(
        physics_.restitution_h * velocity_in.x(),
        physics_.restitution_h * velocity_in.y(),
        -physics_.restitution_v * velocity_in.z());
    *omega_out = omega_in;
    return;
  }
  const Vec3 contact_velocity(
      velocity_in.x() - radius * omega_in.y(),
      velocity_in.y() + radius * omega_in.x(),
      0.0);
  const double tangential_speed = contact_velocity.head<2>().norm();
  double alpha = 0.0;
  if (tangential_speed >= 1.0e-9) {
    const double sliding_alpha =
        physics_.nakashima_friction_mu * (1.0 + physics_.restitution_v) *
        std::abs(velocity_in.z()) / tangential_speed;
    alpha = 1.0 - 2.5 * sliding_alpha > 0.0
        ? sliding_alpha
        : 0.4;
  }
  *velocity_out = Vec3(
      velocity_in.x() - alpha * contact_velocity.x(),
      velocity_in.y() - alpha * contact_velocity.y(),
      -physics_.restitution_v * velocity_in.z());
  const double gain = 3.0 * alpha / (2.0 * radius);
  *omega_out = Vec3(
      omega_in.x() - gain * contact_velocity.y(),
      omega_in.y() + gain * contact_velocity.x(),
      omega_in.z());
}

void TrajectoryPredictor::apply_venue_grip_bounce(
    const Vec3& velocity_in,
    const Vec3& omega_in,
    Vec3* velocity_out,
    Vec3* omega_out) const noexcept {
  if (velocity_out == nullptr || omega_out == nullptr) {
    return;
  }
  const double radius = physics_.radius;
  constexpr double inertia_coefficient = 2.0 / 3.0;
  if (!(radius > 0.0)) {
    *velocity_out = Vec3(
        physics_.restitution_h * velocity_in.x(),
        physics_.restitution_h * velocity_in.y(),
        -physics_.restitution_v * velocity_in.z());
    *omega_out = omega_in;
    return;
  }

  // Exact static-table specialization of
  // ball_physics_fit/contact_model.py::predict_contact and
  // physical_ball.py::predict_table_contact:
  //   u = v + omega x (-R ez)
  //   |dv_t| = min(a_t |u_t|, mu (1+e) |u_n|)
  //   dw = -(1/(cR)) ez x dv_t
  const Vec3 contact_velocity(
      velocity_in.x() - radius * omega_in.y(),
      velocity_in.y() + radius * omega_in.x(),
      velocity_in.z());
  const Vec3 tangential(contact_velocity.x(), contact_velocity.y(), 0.0);
  const double tangential_speed = tangential.norm();
  const double raw_impulse_speed =
      std::max(0.0, physics_.table_tangential_gain) * tangential_speed;
  const double friction_cap =
      std::max(0.0, physics_.table_friction_cap_mu) *
      (1.0 + physics_.restitution_v) * std::abs(contact_velocity.z());
  const double impulse_speed = std::min(raw_impulse_speed, friction_cap);
  Vec3 delta_v_t = Vec3::Zero();
  if (tangential_speed > 1.0e-12) {
    delta_v_t = -(impulse_speed / tangential_speed) * tangential;
  }
  const Vec3 delta_v_n(
      0.0, 0.0,
      -(1.0 + physics_.restitution_v) * contact_velocity.z());
  const Vec3 normal(0.0, 0.0, 1.0);
  const Vec3 delta_omega =
      -(1.0 / (inertia_coefficient * radius)) * normal.cross(delta_v_t);
  *velocity_out = velocity_in + delta_v_t + delta_v_n;
  *omega_out = omega_in + delta_omega;
}

StrikeTarget TrajectoryPredictor::predict_with_spin(
    const BallState& state,
    double x_hit,
    const Vec3& omega_rad_s,
    SpinPhysicsMode mode) const noexcept {
  if (mode == SpinPhysicsMode::kLegacyNoSpin) {
    return predict(state, x_hit);
  }

  StrikeTarget output;
  output.ball_position = state.position;
  output.ball_velocity = state.velocity;
  output.strike_source_time_s = state.source_time_s;
  output.reason = "prediction_running";
  if (!state.valid || !state.position.allFinite() || !state.velocity.allFinite() ||
      !omega_rad_s.allFinite() || !std::isfinite(state.source_time_s) ||
      !std::isfinite(x_hit)) {
    output.reason = "invalid_estimate";
    return output;
  }
  if (state.velocity.x() >= 0.0) {
    output.reason = "not_incoming";
    return output;
  }

  const double dt = config_.integrate_dt_s;
  if (!std::isfinite(dt) || dt <= 0.0) {
    output.reason = "invalid_integrator_dt";
    return output;
  }
  const double horizon = prediction_horizon_s(state, x_hit);
  const int max_steps = std::max(0, static_cast<int>(horizon / dt));
  const double table_y_lo = table_.y_max - table_.width;
  const bool use_magnus =
      mode == SpinPhysicsMode::kNakashimaBounceAndMagnus ||
      mode == SpinPhysicsMode::kVenueGripBounceAndMagnus;
  const bool use_venue_grip =
      mode == SpinPhysicsMode::kVenueGripBounce ||
      mode == SpinPhysicsMode::kVenueGripBounceAndMagnus;

  Vec3 position = state.position;
  Vec3 velocity = state.velocity;
  Vec3 omega = omega_rad_s;
  double time_s = state.source_time_s;
  int bounces = 0;

  for (int step = 0; step < max_steps; ++step) {
    const Vec3 previous_position = position;
    const Vec3 previous_velocity = velocity;
    const Vec3 acceleration = flight_acceleration(velocity, omega, use_magnus);
    Vec3 velocity_new = velocity + acceleration * dt;
    Vec3 position_new = position + velocity * dt + 0.5 * acceleration * dt * dt;
    time_s += dt;

    bool bounce_this_step = false;
    Vec3 bounce_position = position;
    Vec3 post_velocity = velocity;
    double remaining_dt = dt;
    // OptiTrack reports the ball center.  Contact therefore happens at
    // z=radius, not at the point-ball z=0 plane retained by the legacy path.
    const double contact_z = std::max(0.0, physics_.radius);
    if (position_new.z() < contact_z && velocity_new.z() < 0.0) {
      const bool on_table =
          -physics_.radius <= position_new.x() &&
          position_new.x() <= table_.length + physics_.radius &&
          table_y_lo - physics_.radius <= position_new.y() &&
          position_new.y() <= table_.y_max + physics_.radius;
      if (on_table) {
        const double dz = position.z() - position_new.z();
        const double fraction = std::clamp(
            dz > 1.0e-9 ? (position.z() - contact_z) / dz : 0.5,
            0.0, 1.0);
        bounce_position = position + fraction * (position_new - position);
        bounce_position.z() = contact_z;
        const double bounce_dt = fraction * dt;
        const Vec3 velocity_at_bounce = velocity + acceleration * bounce_dt;
        Vec3 post_omega = omega;
        if (use_venue_grip) {
          apply_venue_grip_bounce(
              velocity_at_bounce, omega, &post_velocity, &post_omega);
        } else {
          apply_nakashima_bounce(
              velocity_at_bounce, omega, &post_velocity, &post_omega);
        }
        omega = post_omega;

        remaining_dt = (1.0 - fraction) * dt;
        const Vec3 post_acceleration =
            flight_acceleration(post_velocity, omega, use_magnus);
        position_new = bounce_position + post_velocity * remaining_dt +
                       0.5 * post_acceleration * remaining_dt * remaining_dt;
        velocity_new = post_velocity + post_acceleration * remaining_dt;
        ++bounces;
        bounce_this_step = true;
      } else {
        position_new.z() = std::max(0.0, position_new.z());
      }
    }

    if (previous_position.x() > x_hit && position_new.x() <= x_hit &&
        velocity_new.x() < 0.0) {
      Vec3 crossing_position = Vec3::Zero();
      Vec3 crossing_velocity = Vec3::Zero();
      double crossing_time = 0.0;
      if (bounce_this_step) {
        const double dx = bounce_position.x() - position_new.x();
        const double fraction = std::clamp(
            std::abs(dx) > 1.0e-9
                ? (bounce_position.x() - x_hit) / dx
                : 0.5,
            0.0, 1.0);
        crossing_position =
            bounce_position + fraction * (position_new - bounce_position);
        crossing_velocity =
            post_velocity + fraction * (velocity_new - post_velocity);
        crossing_time = time_s - remaining_dt + fraction * remaining_dt;
      } else {
        const double dx = previous_position.x() - position_new.x();
        const double fraction = std::clamp(
            std::abs(dx) > 1.0e-9
                ? (previous_position.x() - x_hit) / dx
                : 0.5,
            0.0, 1.0);
        crossing_position =
            previous_position + fraction * (position_new - previous_position);
        crossing_velocity =
            previous_velocity + fraction * (velocity_new - previous_velocity);
        crossing_time = time_s - dt + fraction * dt;
      }
      crossing_position.x() = x_hit;
      output.ball_position = crossing_position;
      output.ball_velocity = crossing_velocity;
      output.strike_source_time_s = crossing_time;
      output.predicted_bounces = bounces;
      const bool dead_ball =
          crossing_position.z() < 0.05 && crossing_velocity.z() < 0.0;
      output.valid = !dead_ball;
      output.reason = dead_ball ? "dead_ball" : "prediction_valid";
      return output;
    }

    position = position_new;
    velocity = velocity_new;
  }

  output.ball_position = position;
  output.ball_velocity = velocity;
  output.strike_source_time_s = time_s;
  output.predicted_bounces = bounces;
  if (state.position.x() <= x_hit) {
    output.reason = "no_hit_plane_crossing";
  } else if (position.x() > x_hit) {
    output.reason = "prediction_horizon_exceeded";
  } else {
    output.reason = "prediction_invalid";
  }
  return output;
}

}  // namespace hope_planner_cpp
