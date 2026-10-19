#include "hope_planner_cpp/planner_node.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <sstream>
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

struct TurnCase { double yaw; bool flat; bool send_pose; bool expected_valid; bool stale = false; bool source_stale = false; bool y_filter = false; double y_min = -0.7; double y_max = -0.3; const char* y_reason = nullptr; bool runtime_update = false; bool runtime_enabled = true; double updated_min = std::numeric_limits<double>::quiet_NaN(); double updated_max = std::numeric_limits<double>::quiet_NaN(); bool reject_stale = true; };
class PlannerNodeTurnGuard : public ::testing::TestWithParam<TurnCase> {
  void TearDown() override { if (rclcpp::ok()) rclcpp::shutdown(); }
};
TEST_P(PlannerNodeTurnGuard, BothOutputsRejectTheWholeCommand) {
  const auto c = GetParam();
  ::setenv("ROS_DOMAIN_ID", "229", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  int argc = 0;
  rclcpp::init(argc, nullptr);

  const auto csv_path = temporary_path(".csv");
  rclcpp::NodeOptions options;
  options.append_parameter_override("target_turn_reject_stale_pose", c.reject_stale);
  options.append_parameter_override("input_qos_depth", 64);
  options.append_parameter_override("post_net_one_shot_enabled", true);
  options.append_parameter_override("post_net_commit_delay_s", 0.05);
  options.append_parameter_override(
      "post_net_future_bounce_tangential_gain", 0.075);
  options.append_parameter_override("debug_csv_path", csv_path.string());
  options.append_parameter_override("debug_session_id", "post_net_one_shot_test");
  options.append_parameter_override("spin_shadow_enabled", false);
  options.append_parameter_override("swing_side_reference_mode", c.send_pose ? "session_home_v1" : "live_base_v1");
  options.append_parameter_override("robot_pose_topic", c.flat ? "" : "/test/session_home_pose");
  options.append_parameter_override("base_pose_flat_input_topic", c.flat ? "/test/base_flat" : "");
  options.append_parameter_override("publish_typed_command", true);

  options.append_parameter_override("ball_y_filter_enabled", c.y_filter);
  options.append_parameter_override("ball_y_filter_min_m", c.y_min);
  options.append_parameter_override("ball_y_filter_max_m", c.y_max);

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
  hope_msgs::msg::RacketCommand typed;
  std::atomic<int> typed_count{0};
  auto typed_subscription = io_node->create_subscription<hope_msgs::msg::RacketCommand>(
      "/racket/command", rclcpp::QoS(10).reliable(),
      [&typed, &typed_count](const hope_msgs::msg::RacketCommand::SharedPtr msg) {
        typed = *msg; ++typed_count;
      });
  auto flat_publisher = io_node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/test/base_flat", rclcpp::QoS(10).reliable());
  auto publisher = io_node->create_publisher<geometry_msgs::msg::PoseArray>(
      "/poses", rclcpp::QoS(rclcpp::KeepLast(128))
                    .best_effort()
                    .durability_volatile());
  auto base_publisher = io_node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/test/session_home_pose", rclcpp::QoS(rclcpp::KeepLast(10))
                                     .best_effort()
                                     .durability_volatile());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(planner);
  executor.add_node(io_node);

  if (c.runtime_update) {
    auto client = io_node->create_client<rcl_interfaces::srv::SetParametersAtomically>(
        "/hope/planner/set_ball_y_filter");
    ASSERT_TRUE(client->wait_for_service(2s));
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    request->parameters = {rclcpp::Parameter("enabled", c.runtime_enabled).to_parameter_msg(),
        rclcpp::Parameter("min_m", std::isfinite(c.updated_min) ? c.updated_min : c.y_min).to_parameter_msg(),
        rclcpp::Parameter("max_m", std::isfinite(c.updated_max) ? c.updated_max : c.y_max).to_parameter_msg()};
    auto future = client->async_send_request(request);
    ASSERT_EQ(executor.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->result.successful);
  }

  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while ((publisher->get_subscription_count() == 0 ||
          (c.flat ? flat_publisher->get_subscription_count() : base_publisher->get_subscription_count()) == 0) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  ASSERT_GT(c.flat ? flat_publisher->get_subscription_count() : base_publisher->get_subscription_count(), 0U);

  // Gate3 publishes a process-start origin before reset/placement. That
  // transport packet must not become HOME. No ball solve happens until after
  // the authoritative placement packet at y=-0.7625.
  bool initial_pose = true;
  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.pose.orientation.w = std::cos(c.yaw * 3.14159265358979323846 / 360.0);
  base_pose.pose.orientation.z = std::sin(c.yaw * 3.14159265358979323846 / 360.0);
  base_pose.pose.position.x = -0.58;
  base_pose.pose.position.y = 0.0;
  if (c.send_pose && (!c.stale || initial_pose)) {
    if (c.flat) {
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      std_msgs::msg::Float64MultiArray flat;
      flat.data = {2,1,1, double(now_ns / 1000000000LL - (c.source_stale ? 1 : 0)), double(now_ns % 1000000000LL),
        base_pose.pose.position.x, base_pose.pose.position.y, 0.95,
        base_pose.pose.orientation.w,0,0,base_pose.pose.orientation.z,0,47,1,1};
      flat_publisher->publish(flat);
    } else base_publisher->publish(base_pose);
  }
  executor.spin_some(20ms);
  base_pose.pose.position.y = -0.7625;
  if (c.send_pose && (!c.stale || initial_pose)) {
    if (c.flat) {
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      std_msgs::msg::Float64MultiArray flat;
      flat.data = {2,1,1, double(now_ns / 1000000000LL - (c.source_stale ? 1 : 0)), double(now_ns % 1000000000LL),
        base_pose.pose.position.x, base_pose.pose.position.y, 0.95,
        base_pose.pose.orientation.w,0,0,base_pose.pose.orientation.z,0,47,1,1};
      flat_publisher->publish(flat);
    } else base_publisher->publish(base_pose);
  }
  executor.spin_some(20ms);

  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto start_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  constexpr std::int64_t kStepNs = 2'777'778;
  initial_pose = false;
  if (c.stale) std::this_thread::sleep_for(250ms);
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
    base_pose.header.stamp = message.header.stamp;
    if (c.source_stale) --base_pose.header.stamp.sec;
    if (c.send_pose && (!c.stale || initial_pose)) {
    if (c.flat) {
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      std_msgs::msg::Float64MultiArray flat;
      flat.data = {2,1,1, double(now_ns / 1000000000LL - (c.source_stale ? 1 : 0)), double(now_ns % 1000000000LL),
        base_pose.pose.position.x, base_pose.pose.position.y, 0.95,
        base_pose.pose.orientation.w,0,0,base_pose.pose.orientation.z,0,47,1,1};
      flat_publisher->publish(flat);
    } else base_publisher->publish(base_pose);
  }
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
  typed_subscription.reset();
  flat_publisher.reset();
  publisher.reset();
  base_publisher.reset();
  command_subscription.reset();
  io_node.reset();
  planner.reset();
  rclcpp::shutdown();
  EXPECT_EQ(command_messages.load(), 1);
  {
    std::lock_guard<std::mutex> lock(command_mutex);
    ASSERT_EQ(one_shot_packet.size(), 19U);
    EXPECT_DOUBLE_EQ(one_shot_packet[0], 2.0);
    ASSERT_DOUBLE_EQ(one_shot_packet[1], c.expected_valid ? 1.0 : 0.0);
    ASSERT_EQ(typed_count.load(), 1);
    EXPECT_EQ(typed.valid, c.expected_valid);
    if (!c.expected_valid) {
      for (int i = 3; i <= 10; ++i) EXPECT_DOUBLE_EQ(one_shot_packet[i], 0.0);
      EXPECT_DOUBLE_EQ(typed.position.x, 0.0);
      EXPECT_DOUBLE_EQ(typed.position.y, 0.0);
      EXPECT_DOUBLE_EQ(typed.velocity.x, 0.0);
      EXPECT_DOUBLE_EQ(typed.time_to_strike, 0.0);
    } else {
    // The incoming lane is around y=-0.52. Relative to the solve-time HOME
    // (-0.7625) it is BH; relative to the startup origin it would be FH.
    EXPECT_DOUBLE_EQ(one_shot_packet[2], -1.0);
    EXPECT_DOUBLE_EQ(one_shot_packet[16], 1.0);
    }
  }

  std::ifstream input(csv_path);
  ASSERT_TRUE(input.good());
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
  EXPECT_NE(line.find("trajectory_epoch"), std::string::npos);
  EXPECT_NE(line.find("snapshot_sequence"), std::string::npos);
  EXPECT_NE(line.find("segment_boundary_reason"), std::string::npos);
  const std::string header = line;
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
  EXPECT_NE(solve_row.find(c.y_reason ? c.y_reason : c.expected_valid ? "target_turn_allowed" :
      (c.source_stale ? "target_turn_base_source_stale" : c.stale ? "target_turn_base_stale" :
       (c.send_pose ? "target_turn_exceeds_limit" : "target_turn_base_unavailable"))), std::string::npos);

  const auto fields = [](const std::string& row) {
    std::vector<std::string> values;
    std::istringstream stream(row);
    std::string value;
    while (std::getline(stream, value, ',')) values.push_back(value);
    return values;
  };
  const auto names = fields(header);
  const auto values = fields(solve_row);
  const auto metric = [&](const std::string& name) {
    const auto it = std::find(names.begin(), names.end(), name);
    EXPECT_NE(it, names.end()) << name;
    return it != names.end() && static_cast<std::size_t>(it - names.begin()) < values.size()
        ? std::stod(values[it - names.begin()]) : NAN;
  };
  EXPECT_DOUBLE_EQ(metric("target_turn_reject_stale_pose"), c.reject_stale ? 1.0 : 0.0);
  EXPECT_DOUBLE_EQ(metric("base_pose_age_exceeded_commands_total"), (c.stale || c.source_stale) ? 1.0 : 0.0);
  EXPECT_DOUBLE_EQ(metric("pose_age_limit_ms"), 200.0);
  EXPECT_DOUBLE_EQ(metric("base_receipt_stale_rejections_total"), (c.stale && c.reject_stale) ? 1.0 : 0.0);
  EXPECT_DOUBLE_EQ(metric("base_source_stale_rejections_total"), (c.source_stale && c.reject_stale) ? 1.0 : 0.0);
  if (c.send_pose) {
    EXPECT_GT(metric("base_received_total"), 0.0);
    EXPECT_DOUBLE_EQ(metric("base_invalid_total"), 0.0);
    if (c.stale) { EXPECT_GT(metric("base_age_ms"), 200.0); }
    if (c.source_stale) {
      EXPECT_GT(metric("base_source_age_at_receipt_ms"), 900.0);
      EXPECT_GT(metric("base_source_age_ms"), 900.0);
      EXPECT_LT(metric("base_age_ms"), 200.0);
    }
  } else {
    EXPECT_DOUBLE_EQ(metric("base_received_total"), 0.0);
    EXPECT_TRUE(std::isnan(metric("base_source_age_at_receipt_ms")));
  }

  std::error_code error;
  std::filesystem::remove(csv_path, error);
}

TEST(PlannerBodyTiming, HealthReportsWithoutBallsOrRecording) {
  ::setenv("ROS_DOMAIN_ID", "229", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  int argc = 0;
  rclcpp::init(argc, nullptr);
  rclcpp::NodeOptions options;
  options.append_parameter_override("robot_pose_topic", "/test/timing_pose");
  options.append_parameter_override("base_pose_flat_input_topic", "");
  options.append_parameter_override("debug_csv_path", "");
  auto planner = std::make_shared<PlannerNode>(options);
  auto io = std::make_shared<rclcpp::Node>("body_timing_test_io");
  auto publisher = io->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/test/timing_pose", rclcpp::SensorDataQoS());
  bool observed = false;
  auto subscription = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/planner/diagnostics", rclcpp::QoS(10),
      [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        for (const auto& status : message->status) {
          if (status.name != "hope_planner_cpp/realtime") continue;
          const auto metric = [&](const std::string& name) {
            for (const auto& value : status.values) {
              if (value.key == name) return std::stod(value.value);
            }
            return static_cast<double>(NAN);
          };
          if (!(metric("base_received_total") > 0)) continue;
          observed = true;
          EXPECT_GT(metric("base_source_age_at_receipt_ms"), 900.0);
          EXPECT_GT(metric("base_source_age_ms"), 900.0);
          EXPECT_DOUBLE_EQ(metric("pose_age_limit_ms"), 200.0);
          EXPECT_DOUBLE_EQ(metric("target_turn_evaluations_total"), 0.0);
          EXPECT_DOUBLE_EQ(metric("base_source_stale_rejections_total"), 0.0);
        }
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(io);
  const auto until = std::chrono::steady_clock::now() + 3s;
  while (!observed && std::chrono::steady_clock::now() < until) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = io->get_clock()->now() - rclcpp::Duration::from_seconds(1.0);
    pose.pose.orientation.w = 1.0;
    publisher->publish(pose);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(observed);
  executor.remove_node(io);
  executor.remove_node(planner);
  planner.reset();
  io.reset();
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(PoseAndFlat, PlannerNodeTurnGuard, ::testing::Values(
    [] { TurnCase c{0, true, true, true}; c.stale = true; c.reject_stale = false; return c; }(),
    [] { TurnCase c{0, true, true, true}; c.source_stale = true; c.reject_stale = false; return c; }(),
    [] { TurnCase c{0, true, false, false}; c.reject_stale = false; return c; }(),
    TurnCase{0,false,true,true}, TurnCase{90,false,true,false},
    TurnCase{-90,false,true,false}, TurnCase{0,false,false,false},
    TurnCase{0,true,true,true}, TurnCase{90,true,true,false},
    TurnCase{-90,true,true,false}, TurnCase{0,true,false,false},
    TurnCase{0,false,true,false,true}, TurnCase{0,true,true,false,true},
    TurnCase{0,false,true,false,false,true}, TurnCase{0,true,true,false,false,true},
    TurnCase{0,false,true,true,false,false,true},
    TurnCase{0,true,true,true,false,false,true},
    TurnCase{0,true,true,false,false,false,true,-1.39,-0.56,"ball_y_filter_observed_outside"},
    TurnCase{0,true,true,false,false,false,true,-0.50,-0.30,"ball_y_filter_observed_outside"},
    TurnCase{0,true,true,false,false,false,true,-0.55,-0.51,"ball_y_filter_contact_outside"},
    TurnCase{0,true,true,false,false,false,false,-1.39,-0.56,"ball_y_filter_observed_outside",true,true},
    TurnCase{0,true,true,true,false,false,true,-1.39,-0.56,nullptr,true,false},
    TurnCase{0,true,true,true,false,false,true,-1.39,-0.56,nullptr,true,true,-0.7,-0.3}));
}  // namespace
}  // namespace hope_planner_cpp
