#include "hope_planner_cpp/planner_node.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace hope_planner_cpp {
namespace {

using namespace std::chrono_literals;

std::filesystem::path temporary_path(const std::string& suffix) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
      ("hope_planner_node_one_shot_" + std::to_string(::getpid()) + "_" +
       std::to_string(stamp) + suffix);
}

TEST(PlannerNodeOneShot, PostNetFlightProducesOnlyOneCommandAndSolveRow) {
  ::setenv("ROS_DOMAIN_ID", "228", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  int argc = 0;
  rclcpp::init(argc, nullptr);

  const auto csv_path = temporary_path(".csv");
  rclcpp::NodeOptions options;
  options.append_parameter_override("input_qos_depth", 64);
  options.append_parameter_override("post_net_one_shot_enabled", true);
  options.append_parameter_override("post_net_commit_delay_s", 0.05);
  options.append_parameter_override(
      "post_net_future_bounce_tangential_gain", 0.075);
  options.append_parameter_override("debug_csv_path", csv_path.string());
  options.append_parameter_override("debug_session_id", "post_net_one_shot_test");
  options.append_parameter_override("spin_shadow_enabled", false);

  auto planner = std::make_shared<PlannerNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("one_shot_test_io");
  std::atomic<int> command_messages{0};
  std::mutex command_mutex;
  std::vector<double> one_shot_packet;
  auto command_subscription =
      io_node->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/racket/command_flat", rclcpp::QoS(10).reliable(),
          [&command_messages, &command_mutex, &one_shot_packet](
              const std_msgs::msg::Float64MultiArray::SharedPtr message) {
            std::lock_guard<std::mutex> lock(command_mutex);
            one_shot_packet = message->data;
            ++command_messages;
          });
  auto publisher = io_node->create_publisher<geometry_msgs::msg::PoseArray>(
      "/poses", rclcpp::QoS(rclcpp::KeepLast(128))
                    .best_effort()
                    .durability_volatile());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(planner);
  executor.add_node(io_node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);

  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto start_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  constexpr std::int64_t kStepNs = 2'777'778;
  for (int index = 0; index < 180; ++index) {
    const auto stamp_ns = start_ns + index * kStepNs;
    geometry_msgs::msg::PoseArray message;
    message.header.stamp.sec = static_cast<std::int32_t>(
        stamp_ns / 1'000'000'000LL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(
        stamp_ns % 1'000'000'000LL);
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.55 - 0.006 * index;
    pose.position.y = -0.55 + 0.0002 * index;
    pose.position.z = 0.45;
    pose.orientation.w = 1.0;
    message.poses.push_back(pose);
    publisher->publish(message);
    executor.spin_some(5ms);
    std::this_thread::sleep_for(1ms);
  }
  const auto drain_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < drain_deadline) {
    executor.spin_some(10ms);
    std::this_thread::sleep_for(2ms);
  }

  executor.remove_node(io_node);
  executor.remove_node(planner);
  publisher.reset();
  command_subscription.reset();
  io_node.reset();
  planner.reset();
  rclcpp::shutdown();
  EXPECT_EQ(command_messages.load(), 1);
  {
    std::lock_guard<std::mutex> lock(command_mutex);
    ASSERT_EQ(one_shot_packet.size(), 19U);
    EXPECT_DOUBLE_EQ(one_shot_packet[0], 2.0);
    ASSERT_DOUBLE_EQ(one_shot_packet[1], 1.0);
    EXPECT_DOUBLE_EQ(one_shot_packet[16], 1.0);
  }

  std::ifstream input(csv_path);
  ASSERT_TRUE(input.good());
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
  EXPECT_NE(line.find("trajectory_epoch"), std::string::npos);
  EXPECT_NE(line.find("snapshot_sequence"), std::string::npos);
  EXPECT_NE(line.find("segment_boundary_reason"), std::string::npos);
  const auto header_fields = std::count(line.begin(), line.end(), ',') + 1;
  int solve_rows = 0;
  std::string solve_row;
  while (std::getline(input, line)) {
    if (!line.empty()) {
      EXPECT_EQ(std::count(line.begin(), line.end(), ',') + 1, header_fields);
      solve_row = line;
      ++solve_rows;
    }
  }
  EXPECT_EQ(solve_rows, 1);
  EXPECT_NE(solve_row.find(",initial_incoming,"), std::string::npos);

  std::error_code error;
  std::filesystem::remove(csv_path, error);
}

}  // namespace
}  // namespace hope_planner_cpp
