#include "hope_planner_cpp/planner_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hope_planner_cpp::PlannerNode>();
  std::unique_ptr<rclcpp::Executor> executor;
  if (node->get_parameter("prediction_backend").as_string() == "v2" ||
      node->get_parameter("prediction_backend").as_string() == "v2_incoming") {
    // V2 callbacks share one mutually exclusive group; solving already runs
    // on its bounded worker. Avoid extra executor handoffs on this path.
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  } else {
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 3);
  }
  executor->add_node(node);
  executor->spin();
  executor->remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
