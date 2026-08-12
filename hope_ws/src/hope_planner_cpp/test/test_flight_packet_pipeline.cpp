#include "hope_planner_cpp/flight_packetizer_node.hpp"
#include "hope_planner_cpp/planner_node.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <gtest/gtest.h>
#include <hope_msgs/msg/ball_flight_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace hope_planner_cpp {
namespace {

using namespace std::chrono_literals;

TEST(FlightPacketPipeline, ThreeTransmitsProduceOnePlannerCommand) {
  ::setenv("ROS_DOMAIN_ID", "229", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  int argc = 0;
  rclcpp::init(argc, nullptr);

  rclcpp::NodeOptions packetizer_options;
  packetizer_options.append_parameter_override("session_id", "pipeline_test");
  packetizer_options.append_parameter_override(
      "producer_instance_id", "laptop_test_instance");
  packetizer_options.append_parameter_override(
      "retransmit_delays_ms", std::vector<std::int64_t>{0, 5, 15});
  auto packetizer = std::make_shared<FlightPacketizerNode>(packetizer_options);

  rclcpp::NodeOptions planner_options;
  planner_options.append_parameter_override("flight_packet_input_enabled", true);
  planner_options.append_parameter_override("post_net_one_shot_enabled", true);
  planner_options.append_parameter_override("spin_shadow_enabled", false);
  auto planner = std::make_shared<PlannerNode>(planner_options);

  auto io_node = std::make_shared<rclcpp::Node>("flight_packet_pipeline_io");
  std::atomic<int> packet_messages{0};
  std::atomic<int> command_messages{0};
  std::atomic<bool> exact_source_stamps{true};
  std::mutex packet_mutex;
  std::set<std::string> payload_hashes;
  std::set<std::uint8_t> transmit_indices;
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto start_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() + 123;
  constexpr std::int64_t kStepNs = 2'777'778;
  std::set<std::int64_t> published_source_stamps;
  for (int index = 0; index < 180; ++index) {
    published_source_stamps.insert(start_ns + index * kStepNs);
  }
  auto packet_subscription =
      io_node->create_subscription<hope_msgs::msg::BallFlightPacket>(
          "/ball/flight_packet", rclcpp::QoS(rclcpp::KeepLast(8))
              .best_effort().durability_volatile(),
          [&packet_messages, &exact_source_stamps, &packet_mutex,
           &payload_hashes, &transmit_indices, &published_source_stamps](
              const hope_msgs::msg::BallFlightPacket::SharedPtr message) {
            std::lock_guard<std::mutex> lock(packet_mutex);
            payload_hashes.insert(message->payload_hash);
            transmit_indices.insert(message->transmit_index);
            for (const auto& sample : message->samples) {
              if (published_source_stamps.count(
                      sample.exposure_unix_stamp_ns) == 0) {
                exact_source_stamps.store(false);
              }
            }
            ++packet_messages;
          });
  auto command_subscription =
      io_node->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/racket/command_flat", rclcpp::QoS(10).reliable(),
          [&command_messages](
              const std_msgs::msg::Float64MultiArray::SharedPtr) {
            ++command_messages;
          });
  auto pose_publisher =
      io_node->create_publisher<geometry_msgs::msg::PoseArray>(
          "/poses", rclcpp::QoS(rclcpp::KeepLast(128))
              .best_effort().durability_volatile());

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4);
  executor.add_node(packetizer);
  executor.add_node(planner);
  executor.add_node(io_node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while (pose_publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(pose_publisher->get_subscription_count(), 0U);

  for (int index = 0; index < 180; ++index) {
    const auto stamp_ns = start_ns + index * kStepNs;
    geometry_msgs::msg::PoseArray message;
    message.header.stamp.sec = static_cast<std::int32_t>(
        stamp_ns / 1'000'000'000LL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(
        stamp_ns % 1'000'000'000LL);
    message.header.frame_id = "world";
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.70 - 0.006 * index;
    pose.position.y = -0.55 + 0.0002 * index;
    pose.position.z = 0.45;
    pose.orientation.w = 1.0;
    message.poses.push_back(pose);
    pose_publisher->publish(message);
    executor.spin_some(5ms);
    std::this_thread::sleep_for(1ms);
  }

  const auto drain_deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < drain_deadline) {
    executor.spin_some(10ms);
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(packet_messages.load(), 3);
  EXPECT_EQ(command_messages.load(), 1);
  EXPECT_TRUE(exact_source_stamps.load());
  {
    std::lock_guard<std::mutex> lock(packet_mutex);
    EXPECT_EQ(payload_hashes.size(), 1U);
    EXPECT_EQ(transmit_indices, (std::set<std::uint8_t>{0, 1, 2}));
  }

  executor.remove_node(io_node);
  executor.remove_node(planner);
  executor.remove_node(packetizer);
  pose_publisher.reset();
  command_subscription.reset();
  packet_subscription.reset();
  io_node.reset();
  planner.reset();
  packetizer.reset();
  rclcpp::shutdown();
}

}  // namespace
}  // namespace hope_planner_cpp
