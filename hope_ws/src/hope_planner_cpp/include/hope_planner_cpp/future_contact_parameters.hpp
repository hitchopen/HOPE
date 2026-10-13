#pragma once

#include "hope_planner_cpp/future_contact_predictor.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hope_planner_cpp {

inline FutureContactConfig declare_future_contact_config(rclcpp::Node& node) {
  FutureContactConfig c;
  c.spin_enabled = node.declare_parameter<bool>("future_contact_spin_enabled", false);
  c.magnus_enabled = node.declare_parameter<bool>("future_contact_magnus_enabled", false);
  c.tangential_gain = node.declare_parameter<double>("future_contact_spin_tangential_gain", c.tangential_gain);
  c.omega_scale = node.declare_parameter<double>("future_contact_spin_omega_scale", c.omega_scale);
  c.min_coherence = node.declare_parameter<double>("future_contact_spin_min_coherence", c.min_coherence);
  return c;
}

inline SpinEstimatorConfig declare_spin_estimator_config(rclcpp::Node& node) {
  SpinEstimatorConfig c;
  c.window_s = node.declare_parameter<double>("spin_window_s", 0.10);
  c.min_span_s = node.declare_parameter<double>("spin_min_span_s", 0.05);
  c.max_gap_s = node.declare_parameter<double>("spin_max_gap_s", 0.05);
  c.max_rev_s = node.declare_parameter<double>("spin_max_rev_s", 20.0);
  c.huber_delta_rev_s = node.declare_parameter<double>("spin_huber_delta_rev_s", 2.0);
  c.min_increments = static_cast<std::size_t>(std::max(1, static_cast<int>(node.declare_parameter<int>("spin_min_increments", 3))));
  c.robust_iterations = std::max(1, static_cast<int>(node.declare_parameter<int>("spin_robust_iterations", 5)));
  return c;
}

}  // namespace hope_planner_cpp
