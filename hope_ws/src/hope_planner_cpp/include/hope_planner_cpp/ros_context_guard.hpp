#pragma once

#include <rclcpp/context.hpp>
#include <exception>
#include <utility>

namespace hope_planner_cpp {
// ROS can close its context between a timer's admission and a graph query.
// Suppress only exceptions after shutdown; active-context failures still fail.
template <class Callback>
void with_active_ros_context(const rclcpp::Context::SharedPtr &context, Callback &&callback) {
  if (!context->is_valid()) return;
  try {
    std::forward<Callback>(callback)();
  } catch (const std::exception &) {
    if (context->is_valid()) throw;
  }
}
}  // namespace hope_planner_cpp
