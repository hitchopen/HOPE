// Isolated native publisher: avoid rclpy/GIL scheduling in the 300 Hz test.
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
int main(int argc, char **argv) {
  const auto *domain = std::getenv("ROS_DOMAIN_ID");
  const auto *local = std::getenv("ROS_LOCALHOST_ONLY");
  if (!domain || std::string(domain) != "219" || !local || std::string(local) != "1") return 3;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("operations_ingress_fixture");
  const auto malformed = node->declare_parameter("malformed_input", std::string{});
  if (!malformed.empty()) {
    const std::string invalid_utf8(1, char(0xff));
    if (malformed == "service") {
      using Service = rcl_interfaces::srv::SetParameters;
      auto client = node->create_client<Service>("/hope/field/command");
      if (!client->wait_for_service(std::chrono::seconds(5))) return 5;
      auto request = std::make_shared<Service::Request>();
      rcl_interfaces::msg::Parameter parameter;
      parameter.name = "request";
      parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      parameter.value.string_value = invalid_utf8;
      request->parameters.push_back(parameter);
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(5)) !=
          rclcpp::FutureReturnCode::SUCCESS) return 6;
      const auto response = result.get();
      if (response->results.size() != 1 || response->results[0].successful) return 7;
    } else if (malformed == "qualification" || malformed == "qualification_oversize") {
      auto pub = node->create_publisher<std_msgs::msg::String>("/hope/ntp/qualification", 10);
      auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!pub->get_subscription_count() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (!pub->get_subscription_count()) return 8;
      std_msgs::msg::String message;
      message.data = malformed == "qualification" ? invalid_utf8 : std::string(6*1024*1024, char(1));
      for (int i=0;i<3;++i) { pub->publish(message); std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
    } else return 9;
    rclcpp::shutdown(); return 0;
  }
  const bool players = node->declare_parameter("players_only", false);
  const bool benchmark = node->declare_parameter("benchmark", false);
  const int rate = node->declare_parameter("rate_hz", 300);
  const double duration = node->declare_parameter("duration_s", 1.5);
  if(rate<=0||rate>1000||duration<=0||duration>120)return 4;
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("/poses", rclcpp::SensorDataQoS());
  auto p1 = node->create_publisher<geometry_msgs::msg::PoseStamped>("/hope/mocap/players/UCB_P1", rclcpp::SensorDataQoS());
  auto p2 = node->create_publisher<geometry_msgs::msg::PoseStamped>("/hope/mocap/players/UCB_P2", rclcpp::SensorDataQoS());
  auto base_pub=node->create_publisher<std_msgs::msg::Float64MultiArray>("/a3/base_pose_flat",10);
  auto runner_pub=node->create_publisher<std_msgs::msg::Float64MultiArray>("/hope/runner/state_hdu_flat",10);
  auto command_pub=node->create_publisher<std_msgs::msg::Float64MultiArray>("/racket/command_flat",rclcpp::SensorDataQoS());
  const auto matched = [&] { return players ? p1->get_subscription_count() && p2->get_subscription_count()
                                           : publisher->get_subscription_count() > 0; };
  const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!matched() && std::chrono::steady_clock::now() < timeout)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (!matched()) return 2;
  geometry_msgs::msg::PoseArray message;
  message.header.frame_id = "world";
  message.poses.resize(1);
  auto &p = message.poses.front().position;
  p.x = 1.; p.y = -.7; p.z = 1.2;
  geometry_msgs::msg::PoseStamped player1, player2;
  player1.header.frame_id = player2.header.frame_id = "hope_table_global";
  player1.pose.position.x = 3.1; player1.pose.position.y = .2; player1.pose.position.z = 1.2;
  player2.pose.position.x = -.5; player2.pose.position.y = -.3; player2.pose.position.z = 1.1;
  player1.pose.orientation.z = player2.pose.orientation.z = .6;
  player1.pose.orientation.w = player2.pose.orientation.w = .8;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < duration*rate && rclcpp::ok(); ++i) {
    // Managed START evidence is throttled to 30 Hz by the production relay.
    // players_only retains the separate full-rate overload regression.
    if (players || (benchmark && i % std::max(1,rate/30)==0)) {
      player1.header.stamp = player2.header.stamp = node->now();
      p1->publish(player1); p2->publish(player2);
    }
    if(!players || benchmark) {
      message.header.stamp = node->now();
      publisher->publish(message);
    }
    if(benchmark) {
      auto now=node->now();auto stamp=now.nanoseconds();
      std_msgs::msg::Float64MultiArray base,runner,command;
      base.data={2,1,double(i+1),double(stamp/1000000000LL),double(stamp%1000000000LL),-.5,.2,.8,1,0,0,0,1,47,123,234};base_pub->publish(base);
      if(i%std::max(1,rate/50)==0) {
        runner.data={2,42,double(i+1),3,1,1,0,2,0,1,0,0,0,-1,-1,0,0,0,0,0,123};runner_pub->publish(runner);
      }
      if(i%std::max(1,rate/30)==0) {
        command.data={2,1,-1,.3,-.4,.8,4,1,2,.3,now.seconds()+.3,0,double(stamp/1000000000LL),double(stamp%1000000000LL),double(i+1),6,7,14,.04};command_pub->publish(command);
      }
    }
    std::this_thread::sleep_until(start + std::chrono::nanoseconds((i + 1) * 1000000000LL / rate));
  }
  rclcpp::shutdown();
}
