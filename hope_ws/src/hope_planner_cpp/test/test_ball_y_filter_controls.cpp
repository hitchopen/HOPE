#include "hope_planner_cpp/planner_node.hpp"
#include "hope_planner_cpp/ball_y_filter_state.hpp"
#include "hope_planner_cpp/ball_y_filter_markers.hpp"
#include <gtest/gtest.h>
#include <unistd.h>

namespace hope_planner_cpp {
namespace {
using namespace std::chrono_literals;
using Service = rcl_interfaces::srv::SetParametersAtomically;
std::shared_ptr<Service::Request> request(bool enabled, double lo, double hi) {
  auto r = std::make_shared<Service::Request>();
  r->parameters = {rclcpp::Parameter("enabled", enabled).to_parameter_msg(),
      rclcpp::Parameter("min_m", lo).to_parameter_msg(),
      rclcpp::Parameter("max_m", hi).to_parameter_msg()};
  return r;
}

TEST(BallYFilterControls, AtomicServiceValidationAndPersistence) {
  setenv("ROS_DOMAIN_ID", "229", 1); setenv("ROS_LOCALHOST_ONLY", "1", 1);
  int argc = 0; rclcpp::init(argc, nullptr);
  const auto path = (std::filesystem::temp_directory_path() /
      ("ball_y_filter_control_" + std::to_string(getpid()) + ".state")).string();
  std::filesystem::remove(path);
  rclcpp::NodeOptions options;
  options.append_parameter_override("ball_y_filter_state_file", path);
  options.append_parameter_override("ball_y_filter_enabled", true);
  options.append_parameter_override("ball_y_filter_min_m", -1.4);
  options.append_parameter_override("ball_y_filter_max_m", -0.55);
  auto planner = std::make_shared<PlannerNode>(options);
  auto io = std::make_shared<rclcpp::Node>("filter_control_test");
  auto client = io->create_client<Service>("/hope/planner/set_ball_y_filter");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner); executor.add_node(io);
  ASSERT_TRUE(client->wait_for_service(2s));
  auto call = [&](const std::shared_ptr<Service::Request>& r) {
    auto future = client->async_send_request(r);
    if (executor.spin_until_future_complete(future, 2s) != rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("service timed out");
    return future.get()->result.successful;
  };
  EXPECT_TRUE(call(request(false, -1.2, -0.6)));
  auto saved = load_ball_y_filter(path, {});
  EXPECT_FALSE(saved.enabled); EXPECT_DOUBLE_EQ(saved.min_y, -1.2); EXPECT_DOUBLE_EQ(saved.max_y, -0.6);
  EXPECT_FALSE(call(request(true, -0.6, -1.2)));
  EXPECT_FALSE(call(request(true, -0.6, -0.6)));
  EXPECT_FALSE(call(request(true, std::numeric_limits<double>::quiet_NaN(), -0.6)));
  auto duplicate = request(true, -1.1, -0.7); duplicate->parameters.push_back(duplicate->parameters[0]);
  EXPECT_FALSE(call(duplicate));
  auto missing = request(true, -1.1, -0.7); missing->parameters.pop_back(); EXPECT_FALSE(call(missing));
  auto type = request(true, -1.1, -0.7); type->parameters[1].value.type = 2; EXPECT_FALSE(call(type));
  auto unknown = request(true, -1.1, -0.7); unknown->parameters[1].name = "arbitrary"; EXPECT_FALSE(call(unknown));
  saved = load_ball_y_filter(path, {});
  EXPECT_FALSE(saved.enabled); EXPECT_DOUBLE_EQ(saved.min_y, -1.2);
  EXPECT_TRUE(call(request(true, -1.1, -0.7)));
  saved = load_ball_y_filter(path, {});
  EXPECT_TRUE(saved.enabled); EXPECT_DOUBLE_EQ(saved.min_y, -1.1); EXPECT_DOUBLE_EQ(saved.max_y, -0.7);
  executor.remove_node(planner); planner.reset();
  planner = std::make_shared<PlannerNode>(options); executor.add_node(planner);
  visualization_msgs::msg::MarkerArray markers;
  auto sub = io->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/hope/planner/y_filter_markers", rclcpp::QoS(1).reliable().transient_local(),
      [&](visualization_msgs::msg::MarkerArray::SharedPtr m) { markers = *m; });
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (markers.markers.empty() && std::chrono::steady_clock::now() < deadline)
    executor.spin_some(20ms);
  ASSERT_EQ(markers.markers.size(), 5U);
  EXPECT_DOUBLE_EQ(markers.markers[0].pose.position.y, -1.1);
  EXPECT_DOUBLE_EQ(markers.markers[1].pose.position.y, -0.7);
  EXPECT_EQ(markers.markers[4].action, visualization_msgs::msg::Marker::ADD);
  executor.remove_node(planner); executor.remove_node(io);
  sub.reset(); client.reset(); planner.reset(); io.reset(); rclcpp::shutdown();
  std::filesystem::remove(path);
}

TEST(BallYFilterControls, MarkerGeometryOffStateAndExpiry) {
  auto m = ball_y_filter_markers({true,-1.4,-0.55}, {});
  ASSERT_EQ(m.markers.size(), 5U);
  EXPECT_DOUBLE_EQ(m.markers[0].scale.x, 2.74);
  EXPECT_EQ(m.markers[0].header.frame_id, "world");
  EXPECT_GT(m.markers[0].pose.position.z, 0.0);
  EXPECT_EQ(m.markers[0].lifetime.sec, 3);
  EXPECT_DOUBLE_EQ(m.markers[2].pose.position.y, -1.4);
  EXPECT_NE(m.markers[2].text.find("-1.400 m"), std::string::npos);
  EXPECT_DOUBLE_EQ(m.markers[4].scale.y, 0.85);
  m = ball_y_filter_markers({false,-1.4,-0.55}, {});
  EXPECT_EQ(m.markers[4].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_NE(m.markers[2].text.find("OFF"), std::string::npos);
  m = ball_y_filter_markers({}, {});
  for (const auto& marker : m.markers) EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETE);
}

TEST(BallYFilterControls, CorruptReceiptAndFailedSaveFailClosed) {
  const auto path = (std::filesystem::temp_directory_path() /
      ("ball_y_filter_bad_" + std::to_string(getpid()) + ".state")).string();
  { std::ofstream out(path); out << "1 1 -0.5 -1.4"; }
  EXPECT_THROW(load_ball_y_filter(path, {}), std::runtime_error);
  std::filesystem::remove(path);
  EXPECT_THROW(save_ball_y_filter(path + "/missing/state", {true,-1.4,-0.5}), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path));
}
}  // namespace
}  // namespace hope_planner_cpp
