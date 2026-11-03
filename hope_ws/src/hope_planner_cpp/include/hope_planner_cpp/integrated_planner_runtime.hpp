#pragma once
#include <memory>
namespace rclcpp { class Node; }
namespace hope_planner_cpp {
// Owns callbacks and one bounded worker inside PlannerNode; never a ROS node.
class IntegratedPlannerRuntime {
public:
  explicit IntegratedPlannerRuntime(rclcpp::Node &);
  ~IntegratedPlannerRuntime();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace hope_planner_cpp
