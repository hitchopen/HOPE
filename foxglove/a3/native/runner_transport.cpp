// Late-started HDU DDS participant. No command generation, timer replay or
// timestamp replacement: Runner remains the sole body-command authority.
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using Array = std_msgs::msg::Float64MultiArray;
using Steady = std::chrono::steady_clock;

class RunnerTransport final : public rclcpp::Node {
 public:
  RunnerTransport()
      : Node("hope_runner_transport_relay",
             rclcpp::NodeOptions().start_parameter_services(false)
                                  .start_parameter_event_publisher(false)) {
    const std::array<std::string, 14> names = {
        "remote_state_topic", "local_state_topic", "local_control_topic",
        "remote_control_topic", "local_teleop_input_topic", "remote_teleop_input_topic",
        "remote_teleop_state_topic", "local_teleop_state_topic",
        "remote_mocap_safety_topic", "local_mocap_safety_topic",
        "remote_operator_challenge_topic", "local_operator_challenge_topic",
        "local_operator_echo_topic", "remote_operator_echo_topic"};
    const std::array<std::string, 14> defaults = {
        "/hope/runner/state_flat", "/hope/runner/state_hdu_flat",
        "/hope/runner/control_request_hdu_flat", "/hope/runner/control_request_flat",
        "/hope/runner/teleop_input_hdu_flat", "/hope/runner/teleop_input_flat",
        "/hope/runner/teleop_state_flat", "/hope/runner/teleop_state_hdu_flat",
        "/hope/runner/mocap_safety_flat", "/hope/runner/mocap_safety_hdu_flat",
        "/hope/runner/operator_challenge_flat", "/hope/runner/operator_challenge_hdu_flat",
        "/hope/runner/operator_echo_hdu_flat", "/hope/runner/operator_echo_flat"};
    std::set<std::string> unique;
    for (size_t i = 0; i < names.size(); ++i) {
      topics_[i] = declare_parameter(names[i], defaults[i]);
      if (topics_[i].empty() || topics_[i][0] != '/' || !unique.insert(topics_[i]).second)
        throw std::invalid_argument("Relay requires fourteen distinct absolute topics");
    }
    for (size_t route = 0; route < publishers_.size(); ++route) {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(route < 2 ? 10 : 1)).reliable().durability_volatile();
      publishers_[route] = create_publisher<Array>(topics_[2 * route + 1], qos);
      subscriptions_[route] = create_subscription<Array>(
          topics_[2 * route], qos, [this, route](Array::ConstSharedPtr message) {
            // Copy every data and layout field unchanged, including stale input.
            // The native Runner validates freshness/session/sequence itself.
            publishers_[route]->publish(*message);
            ++counts_[route];
            if (route == 0) last_state_ = Steady::now();
          });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { report(); });
    RCLCPP_INFO(get_logger(), "Native Runner transport ready; payloads unchanged; no startup command");
  }

 private:
  void report() {
    const auto now = Steady::now();
    const double age = std::chrono::duration<double>(now - last_state_).count();
    const auto state_pubs = count_publishers(topics_[0]);
    const auto state_subs = count_subscribers(topics_[1]);
    const auto control_pubs = count_publishers(topics_[2]);
    const auto control_subs = count_subscribers(topics_[3]);
    const bool healthy = counts_[0] > 0 && age <= 1.0 && state_pubs >= 1 &&
                         state_subs >= 2 && control_pubs >= 1 && control_subs >= 1;
    // Report transitions immediately; limit unchanged status logs to 0.2 Hz.
    if (!reported_ || healthy != previous_healthy_ || now - last_report_ >= std::chrono::seconds(5)) {
      RCLCPP_INFO(get_logger(),
          "RUNNER TRANSPORT %s state_received=%llu state_published=%llu state_age=%.3fs "
          "remote_state_publishers=%zu local_state_subscribers=%zu "
          "local_control_publishers=%zu remote_control_subscribers=%zu "
          "control_received=%llu control_published=%llu teleop_input=%llu teleop_state=%llu",
          healthy ? "HEALTHY" : "WAITING", static_cast<unsigned long long>(counts_[0]),
          static_cast<unsigned long long>(counts_[0]), age, state_pubs, state_subs,
          control_pubs, control_subs, static_cast<unsigned long long>(counts_[1]),
          static_cast<unsigned long long>(counts_[1]), static_cast<unsigned long long>(counts_[2]),
          static_cast<unsigned long long>(counts_[3]));
      reported_ = true;
      previous_healthy_ = healthy;
      last_report_ = now;
    }
  }
  std::array<std::string, 14> topics_;
  std::array<rclcpp::Publisher<Array>::SharedPtr, 7> publishers_;
  std::array<rclcpp::Subscription<Array>::SharedPtr, 7> subscriptions_;
  std::array<uint64_t, 7> counts_{};
  rclcpp::TimerBase::SharedPtr timer_;
  Steady::time_point last_state_ = Steady::now(), last_report_ = Steady::now();
  bool reported_ = false, previous_healthy_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RunnerTransport>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("hope_runner_transport_relay"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
