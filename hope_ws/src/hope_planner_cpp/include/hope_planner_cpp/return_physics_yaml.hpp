#pragma once

#include "hope_planner_cpp/return_physics_parameters.hpp"
#include <yaml-cpp/yaml.h>

namespace hope_planner_cpp {

inline void load_return_physics_yaml(const std::string& path, BallPhysics& physics,
                                     PlannerConfig& config) {
  const auto parameters = YAML::LoadFile(path)["hope_planner"]["ros__parameters"];
  read_return_physics(physics, config,
      [&](const char* key) { return parameters[key].as<double>(); },
      [&](const char* key) { return parameters[key].as<int>(); },
      [&](const char* key) { return parameters[key].as<std::vector<double>>(); });
}

inline void load_ball_profile_yaml(const std::string& path, BallPhysics& physics) {
  const auto parameters = YAML::LoadFile(path)["hope_planner"]["ros__parameters"];
  physics.drag_k = parameters["drag_k"].as<double>();
  physics.magnus_k = parameters["magnus_k"].as<double>();
  physics.restitution_h = parameters["restitution_h"].as<double>();
  physics.restitution_v = parameters["restitution_v"].as<double>();
  physics.table_tangential_gain = parameters["table_tangential_gain"].as<double>();
  physics.table_friction_cap_mu = parameters["table_friction_cap_mu"].as<double>();
}

inline void load_return_timing_yaml(const std::string& path, PlannerConfig& config) {
  const auto parameters = YAML::LoadFile(path)["hope_planner"]["ros__parameters"];
  config.target_land = Vec3(parameters["target_land_x"].as<double>(),
                            parameters["target_land_y"].as<double>(), 0.0);
  config.delta_t_flight = parameters["delta_t_flight"].as<double>();
  config.integrate_dt_s = parameters["dt_integrate"].as<double>();
  config.max_predict_time_s = parameters["max_predict_time"].as<double>();
}

}  // namespace hope_planner_cpp
