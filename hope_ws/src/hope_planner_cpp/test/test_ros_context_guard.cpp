#include "hope_planner_cpp/ros_context_guard.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

TEST(RosContextGuard, ShutdownBetweenAdmissionAndGraphQueryDoesNotAbort) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("shutdown_guard", rclcpp::NodeOptions().context(context));
  bool reached = false;
  EXPECT_NO_THROW(hope_planner_cpp::with_active_ros_context(context, [&] {
    reached = true;
    context->shutdown("regression: signal raced diagnostic callback");
    // This actual ROS graph operation threw in the retained failing trial.
    node->count_publishers("/shutdown_guard_test");
  }));
  EXPECT_TRUE(reached);
  EXPECT_THROW(node->count_publishers("/shutdown_guard_test"), std::runtime_error);
  hope_planner_cpp::with_active_ros_context(context, [] { FAIL() << "callback after shutdown"; });
}

TEST(RosContextGuard, ActiveContextFailureRemainsVisible) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  EXPECT_THROW(hope_planner_cpp::with_active_ros_context(context, [] {
    throw std::runtime_error("real failure");
  }), std::runtime_error);
  context->shutdown("test complete");
}
