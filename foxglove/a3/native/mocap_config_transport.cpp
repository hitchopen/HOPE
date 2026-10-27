// Fresh HDU DDS participant; no configuration authority and no cached heartbeat.
#include "mocap_config_transport.hpp"
#include "network_addresses.hpp"
#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace hope_mocap_config {
Addresses network_addresses() {
  // Equivalent to the former `ip -j -4 address show` projection: sorted
  // (interface name, IPv4 address), excluding lo but INCLUDING all scopes.
  Addresses addresses;
  for (const auto &a : hope_network::ipv4_addresses(hope_network::Scope::All))
    addresses.emplace_back(a.name, a.address);
  std::sort(addresses.begin(), addresses.end()); return addresses;
}

class Transport final : public rclcpp::Node {
 public:
  explicit Transport(AddressReader reader)
      : Node("hope_mocap_config_transport", rclcpp::NodeOptions().start_parameter_services(false)),
        reader_(std::move(reader)), addresses_(reader_()) {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    publisher_ = create_publisher<std_msgs::msg::String>("/hope/lifecycle/config/mocap_transport", qos);
    subscription_ = create_subscription<std_msgs::msg::String>("/hope/lifecycle/config/mocap", qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        // No JSON decoding, normalization, timestamp changes, or retained copy.
        publisher_->publish(*message);
      });
    if (std::string(publisher_->get_topic_name()) == subscription_->get_topic_name())
      throw std::invalid_argument("Configuration relay input and output must remain distinct after remapping");
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      if (reader_() != addresses_)
        throw std::runtime_error("Network addresses changed; recreate config transport DDS");
    });
    RCLCPP_INFO(get_logger(), "Native mocap config transport: original snapshots only; no changes or cached heartbeats");
  }
 private:
  AddressReader reader_;
  const Addresses addresses_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int run(int argc, char **argv, AddressReader reader) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // Deliberately single-threaded: timer failures unwind to this clean nonzero
    // exit, never out of a worker thread. systemd recreates only this participant.
    rclcpp::spin(std::make_shared<Transport>(std::move(reader)));
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("hope_mocap_config_transport"), "%s", error.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
}  // namespace hope_mocap_config
