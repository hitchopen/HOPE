#pragma once

#include "hope_planner_cpp/ball_y_filter.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace hope_planner_cpp {
inline visualization_msgs::msg::MarkerArray ball_y_filter_markers(
    const BallYFilter& filter, const builtin_interfaces::msg::Time& stamp,
    double table_length = 2.74, double table_y_min = -1.525, double table_y_max = 0.0) {
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray result;
  for (int id = 0; id < 5; ++id) {
    Marker m;
    m.header.frame_id = "world";
    m.header.stamp = stamp;
    m.ns = "planner_ball_y_filter";
    m.id = id;
    m.action = filter.valid_bounds() ? Marker::ADD : Marker::DELETE;
    m.pose.orientation.w = 1.0;
    m.lifetime.sec = 3;  // Expire when Planner stops; do not leave a false live lane.
    m.color.r = filter.enabled ? 1.0F : 0.55F;
    m.color.g = filter.enabled ? 0.65F : 0.55F;
    m.color.b = filter.enabled ? 0.12F : 0.55F;
    m.color.a = filter.enabled ? 0.8F : 0.35F;
    const double y = id % 2 == 0 ? filter.min_y : filter.max_y;
    if (id < 2) {
      m.type = Marker::CUBE;
      m.pose.position.x = table_length * 0.5;
      m.pose.position.y = y;
      m.pose.position.z = 0.014;
      m.scale.x = table_length;
      m.scale.y = 0.018;
      m.scale.z = 0.02;
    } else if (id < 4) {
      m.type = Marker::TEXT_VIEW_FACING;
      m.pose.position.x = -0.12;
      m.pose.position.y = y;
      m.pose.position.z = 0.14;
      m.scale.z = 0.075;
      std::ostringstream label;
      label << (id == 2 ? "Y min " : "Y max ") << std::fixed << std::setprecision(3)
            << y << " m" << (filter.enabled ? "" : " (OFF)");
      m.text = label.str();
    } else {
      const double lo = std::max(filter.min_y, table_y_min);
      const double hi = std::min(filter.max_y, table_y_max);
      m.type = Marker::CUBE;
      m.pose.position.x = table_length * 0.5;
      m.pose.position.y = (lo + hi) * 0.5;
      m.pose.position.z = 0.005;
      m.scale.x = table_length;
      m.scale.y = std::max(0.001, hi - lo);
      m.scale.z = 0.004;
      m.color.r = 0.15F; m.color.g = 0.9F; m.color.b = 0.55F; m.color.a = 0.12F;
      if (!filter.enabled || hi <= lo) m.action = Marker::DELETE;
    }
    result.markers.push_back(m);
  }
  return result;
}
}  // namespace hope_planner_cpp
