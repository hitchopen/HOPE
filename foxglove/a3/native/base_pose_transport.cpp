// Independent early HDU base transport. This participant survives Runner
// transport restarts and preserves the Laptop's original source freshness.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using Array = std_msgs::msg::Float64MultiArray;
using Steady = std::chrono::steady_clock;

// Only discard structurally valid poses. Explicit invalids and malformed
// packets must still reach the consumer's fail-closed validation.
static bool StaleValidPose(const Array& message, double now) {
  const auto& a = message.data;
  if (a.size() < 16 || a[0] != 2. || a[1] != 1.) return false;
  for (size_t i = 2; i < 16; ++i)
    if (!std::isfinite(a[i])) return false;
  for (const auto i : {2, 3, 4, 13, 14, 15})
    if (a[i] < 0. || a[i] > 4503599627370495. || std::floor(a[i]) != a[i])
      return false;
  if (a[3] == 0. || a[4] >= 1.e9 || a[14] == 0. || a[15] == 0. ||
      (static_cast<uint64_t>(a[13]) & 3U) != 3U || a[12] <= 0. || a[12] > 1.)
    return false;
  const double norm = std::hypot(std::hypot(a[8], a[9]), std::hypot(a[10], a[11]));
  if (norm < .5 || norm > 1.5) return false;
  // Same 100 ms source-age bound as Runner. Never restamp, replay, or
  // manufacture valid=0 for a delayed but otherwise valid pose.
  return now - (a[3] + a[4] * 1.e-9) > .100;
}

class BasePoseTransport final : public rclcpp::Node {
 public:
  BasePoseTransport()
      : Node("hope_base_pose_transport_relay",
             rclcpp::NodeOptions().start_parameter_services(false)
                                  .start_parameter_event_publisher(false)) {
    const auto input = declare_parameter("input_topic", std::string("/a3/base_pose_laptop_flat"));
    const auto output = declare_parameter("output_topic", std::string("/a3/base_pose_flat"));
    if (input.empty() || output.empty() || input[0] != '/' || output[0] != '/' || input == output)
      throw std::invalid_argument("Base transport requires two distinct absolute topics");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    publisher_ = create_publisher<Array>(output, qos);
    // Reject a remapping that would feed this node's output back into itself.
    if (std::string(publisher_->get_topic_name()) ==
        get_node_topics_interface()->resolve_topic_name(input))
      throw std::invalid_argument("Base transport input/output resolve to the same topic");
    subscription_ = create_subscription<Array>(input, qos, [this](Array::ConstSharedPtr message) {
      ++received_;
      last_receipt_ = Steady::now();
      const double now = std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      if (StaleValidPose(*message, now)) {
        ++stale_dropped_;
        return;
      }
      publisher_->publish(*message);  // Every forwarded field stays unchanged.
      ++published_;
    });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      // Until a real packet arrives there is no receipt age to measure.
      std::ostringstream age;
      if (received_ == 0) age << "unseen";
      else age << std::fixed << std::setprecision(3)
               << std::chrono::duration<double>(Steady::now() - last_receipt_).count() << 's';
      RCLCPP_INFO(get_logger(), "BASE TRANSPORT received=%llu published=%llu base_age=%s stale_dropped=%llu",
                  static_cast<unsigned long long>(received_),
                  static_cast<unsigned long long>(published_), age.str().c_str(),
                  static_cast<unsigned long long>(stale_dropped_));
    });
    RCLCPP_INFO(get_logger(), "Independent native base transport started; payloads unchanged; no timer replay");
  }

 private:
  rclcpp::Publisher<Array>::SharedPtr publisher_;
  rclcpp::Subscription<Array>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t received_ = 0;
  uint64_t published_ = 0;
  uint64_t stale_dropped_ = 0;
  Steady::time_point last_receipt_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BasePoseTransport>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("hope_base_pose_transport_relay"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
