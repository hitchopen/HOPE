// One late-started HDU DDS participant for Runner transport.
// The Runner participant must exist before this explicit-peer reader starts.
// No command generation, timer replay or timestamp replacement.
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using Array = std_msgs::msg::Float64MultiArray;
using Steady = std::chrono::steady_clock;

class TransportGateway final : public rclcpp::Node {
 public:
  TransportGateway()
      : Node("hope_transport_gateway",
             rclcpp::NodeOptions().start_parameter_services(false)
                                  .start_parameter_event_publisher(false)) {
    operations_node_ = declare_parameter("operations_node", std::string("/hope_operations"));
    if (operations_node_.empty() || operations_node_[0] != '/' || operations_node_.back() == '/')
      throw std::invalid_argument("operations_node must be a fully qualified ROS node name");
    const std::array<std::string, 10> names = {
        "remote_state_topic", "local_state_topic", "local_control_topic",
        "remote_control_topic", "local_teleop_input_topic", "remote_teleop_input_topic",
        "remote_teleop_state_topic", "local_teleop_state_topic",
        "remote_session_home_topic", "local_session_home_topic"};
    const std::array<std::string, 10> defaults = {
        "/hope/runner/state_flat", "/hope/runner/state_hdu_flat",
        "/hope/runner/control_request_hdu_flat", "/hope/runner/control_request_flat",
        "/hope/runner/teleop_input_hdu_flat", "/hope/runner/teleop_input_flat",
        "/hope/runner/teleop_state_flat", "/hope/runner/teleop_state_hdu_flat",
        "/hope/runner/session_home_flat", "/hope/runner/session_home_hdu_flat"};
    std::set<std::string> unique;
    for (size_t i = 0; i < names.size(); ++i) {
      topics_[i] = declare_parameter(names[i], defaults[i]);
      if (topics_[i].empty() || topics_[i][0] != '/' || !unique.insert(topics_[i]).second)
        throw std::invalid_argument("Gateway requires ten distinct absolute topics");
    }
    unique.clear();
    for (const auto& topic : topics_) {
      if (!unique.insert(get_node_topics_interface()->resolve_topic_name(topic)).second)
        throw std::invalid_argument("Gateway topics must remain distinct after ROS remapping");
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
    RCLCPP_INFO(get_logger(),
        "Native Runner transport gateway started; operations_node=%s; payloads unchanged; "
        "no startup command; health covers transport only, RUNNER_VERIFY remains authoritative",
        operations_node_.c_str());
  }

 private:
  struct GraphSnapshot {
    size_t state_pubs = 0, state_subs = 0, control_pubs = 0, control_subs = 0;
    size_t operations_readers = 0, operations_writers = 0;
  };

  size_t operations_endpoints(const std::vector<rclcpp::TopicEndpointInfo>& endpoints) const {
    size_t count = 0;
    for (const auto& endpoint : endpoints) {
      auto name = endpoint.node_namespace();
      if (name.empty() || name.back() != '/') name += '/';
      name += endpoint.node_name();
      // Counts alone can be satisfied by Foxglove or a diagnostic echo. Require
      // the actual operations node and its compatible state/control endpoints.
      if (name == operations_node_ && endpoint.topic_type() == "std_msgs/msg/Float64MultiArray" &&
          endpoint.qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable &&
          endpoint.qos_profile().durability() == rclcpp::DurabilityPolicy::Volatile)
        ++count;
    }
    return count;
  }

  void refresh_graph() {
    // Use resolved topic names so ROS remapping cannot make the graph check
    // inspect a different topic from the actual wire path.
    graph_.state_pubs = count_publishers(subscriptions_[0]->get_topic_name());
    graph_.state_subs = count_subscribers(publishers_[0]->get_topic_name());
    graph_.control_pubs = count_publishers(subscriptions_[1]->get_topic_name());
    graph_.control_subs = count_subscribers(publishers_[1]->get_topic_name());
    graph_.operations_readers = operations_endpoints(
        get_subscriptions_info_by_topic(publishers_[0]->get_topic_name()));
    graph_.operations_writers = operations_endpoints(
        get_publishers_info_by_topic(subscriptions_[1]->get_topic_name()));
  }

  void report() {
    auto now = Steady::now();
    // Graph discovery can allocate endpoint vectors. Query on alternate
    // 500 ms reports (nominally 1 Hz), but never reuse an overdue snapshot
    // after a delayed timer. Keep the single executor: counters and receipt
    // times remain serialized with all five forwarding callbacks.
    if (report_ticks_++ % 2 == 0 || now - last_graph_query_ >= std::chrono::seconds(1)) {
      refresh_graph();
      last_graph_query_ = Steady::now();
    }
    now = Steady::now();
    const double age = std::chrono::duration<double>(now - last_state_).count();
    // This is transport evidence only. Payload validity/session/mode admission
    // is still checked by lifecycle RUNNER_VERIFY and the native Runner.
    const bool eligible = counts_[0] > 0 && age <= 1.0 && graph_.state_pubs >= 1 &&
                          graph_.operations_readers >= 1 && graph_.operations_writers >= 1 &&
                          graph_.control_subs >= 1;
    if (!eligible) {
      consecutive_healthy_ = 0;
    } else if (consecutive_healthy_ == 0) {
      consecutive_healthy_ = 1;
      candidate_state_count_ = counts_[0];
    } else if (consecutive_healthy_ == 1 && counts_[0] > candidate_state_count_) {
      // Two reports of one cached packet are not evidence of a live stream.
      // Require another forwarded packet before opening the START gate.
      consecutive_healthy_ = 2;
    }
    const bool healthy = consecutive_healthy_ >= 2;
    // Report transitions immediately; limit unchanged status logs to 0.2 Hz.
    if (!reported_ || healthy != previous_healthy_ || now - last_report_ >= std::chrono::seconds(5)) {
      std::ostringstream state_age;
      if (counts_[0] == 0) state_age << "unseen";
      else state_age << std::fixed << std::setprecision(3) << age << 's';
      RCLCPP_INFO(get_logger(),
          "TRANSPORT GATEWAY %s readiness=transport_only healthy_evaluations=%u "
          "state_received=%llu state_published=%llu state_age=%s "
          "remote_state_publishers=%zu local_state_subscribers=%zu "
          "local_control_publishers=%zu remote_control_subscribers=%zu "
          "operations_state_readers=%zu operations_control_writers=%zu "
          "control_received=%llu control_published=%llu teleop_input=%llu teleop_state=%llu "
          "session_home=%llu",
          healthy ? "HEALTHY" : "WAITING", consecutive_healthy_,
          static_cast<unsigned long long>(counts_[0]),
          static_cast<unsigned long long>(counts_[0]), state_age.str().c_str(),
          graph_.state_pubs, graph_.state_subs, graph_.control_pubs, graph_.control_subs,
          graph_.operations_readers, graph_.operations_writers,
          static_cast<unsigned long long>(counts_[1]),
          static_cast<unsigned long long>(counts_[1]), static_cast<unsigned long long>(counts_[2]),
          static_cast<unsigned long long>(counts_[3]),
          static_cast<unsigned long long>(counts_[4]));
      reported_ = true;
      previous_healthy_ = healthy;
      last_report_ = now;
    }
  }
  std::string operations_node_;
  std::array<std::string, 10> topics_;
  std::array<rclcpp::Publisher<Array>::SharedPtr, 5> publishers_;
  std::array<rclcpp::Subscription<Array>::SharedPtr, 5> subscriptions_;
  std::array<uint64_t, 5> counts_{};
  GraphSnapshot graph_;
  uint64_t report_ticks_ = 0, candidate_state_count_ = 0;
  unsigned consecutive_healthy_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  Steady::time_point last_state_ = Steady::now(), last_report_ = Steady::now();
  Steady::time_point last_graph_query_{};
  bool reported_ = false, previous_healthy_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TransportGateway>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("hope_transport_gateway"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
