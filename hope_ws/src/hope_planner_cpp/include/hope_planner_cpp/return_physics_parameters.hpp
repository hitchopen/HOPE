#pragma once

#include "hope_planner_cpp/types.hpp"

namespace hope_planner_cpp {

// One key mapping shared by ROS declarations, replay YAML loading and tests.
// Readers are strict: absent coefficients do not fall back to compiled values.
template <class DoubleReader, class IntReader, class VectorReader>
void read_return_physics(BallPhysics& physics, PlannerConfig& config,
                         DoubleReader number, IntReader integer, VectorReader vector) {
  physics.radius = number("ball_radius");
  physics.gravity = Vec3(0.0, 0.0, -number("gravity_mps2"));
  config.ball_inertia_coeff = number("ball_inertia_coeff");
  config.restitution_racket = number("restitution_racket");
  config.restitution_exp_g1 = number("restitution_exp_g1");
  config.restitution_exp_g2 = number("restitution_exp_g2");
  config.paddle_tangential_gain = number("paddle_tangential_gain");
  config.paddle_angle_gain = number("paddle_angle_gain");
  config.paddle_friction_cap_mu = number("paddle_friction_cap_mu");
  config.return_restitution_min = number("return_restitution_min");
  config.return_restitution_max = number("return_restitution_max");
  config.return_dt_integrate = number("return_dt_integrate");
  config.return_numeric_epsilon = number("return_numeric_epsilon");
  config.return_fd_relative_step = number("return_fd_relative_step");
  config.return_position_tolerance_m = number("return_position_tolerance_m");
  config.return_time_tolerance_s = number("return_time_tolerance_s");
  config.return_max_step_mps = number("return_max_step_mps");
  config.return_net_clearance_margin = number("return_net_clearance_margin");
  config.return_newton_iterations = integer("return_newton_iterations");
  config.return_line_search_iterations = integer("return_line_search_iterations");
  config.return_seed_iterations = integer("return_seed_iterations");
  config.return_alternative_flight_times = vector("return_alternative_flight_times");
}

}  // namespace hope_planner_cpp
