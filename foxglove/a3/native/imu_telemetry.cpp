// High-rate IMU ingestion stays native. The Python safety/health node receives
// only bounded-rate samples. CLOCK_MONOTONIC is shared with Python on this HDU.
#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

static double monotonic_seconds() {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    throw std::runtime_error("CLOCK_MONOTONIC unavailable");
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static std::set<std::string> interfaces() {
  ifaddrs* addresses = nullptr;
  if (getifaddrs(&addresses) != 0) throw std::runtime_error("Cannot inspect DDS interfaces");
  std::set<std::string> result;
  for (auto* p = addresses; p; p = p->ifa_next) {
    if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET || (p->ifa_flags & IFF_LOOPBACK)) continue;
    const auto ip = reinterpret_cast<const sockaddr_in*>(p->ifa_addr)->sin_addr.s_addr;
    result.insert(std::string(p->ifa_name) + ':' + std::to_string(if_nametoindex(p->ifa_name)) + ':' + std::to_string(ip));
  }
  freeifaddrs(addresses);
  return result;
}

class ImuTelemetry final : public rclcpp::Node {
 public:
  ImuTelemetry()
      : Node("hope_imu_telemetry", rclcpp::NodeOptions().start_parameter_services(false)
                                                        .start_parameter_event_publisher(false)) {
    const auto input = declare_parameter<std::string>("imu_topic", "/ros2/body_drive/pelvis_imu/data");
    const auto output = declare_parameter<std::string>("sample_topic", "/hope/internal/imu_latency_sample");
    const double hz = declare_parameter("publish_hz", 20.0);
    const bool clock_topics = declare_parameter("publish_clock_topics", false);
    const double stale_after = declare_parameter("stale_after_s", 0.5);
    if (!std::isfinite(hz) || hz <= 0 || hz > 100)
      throw std::invalid_argument("publish_hz must be in (0,100]");
    if (!std::isfinite(stale_after) || stale_after <= 0)
      throw std::invalid_argument("stale_after_s must be positive and finite");
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(output, rclcpp::QoS(1).reliable());
    if (clock_topics) {
      latency_publisher_ = create_publisher<std_msgs::msg::Float64>("/hope/clock/message_latency_ms", 10);
      fresh_publisher_ = create_publisher<std_msgs::msg::Bool>("/hope/clock/message_fresh", 10);
      text_publisher_ = create_publisher<std_msgs::msg::String>("/hope/clock/message_text", 10);
    }
    // The high-rate reader has one fixed wait set and reuses its message.
    // Keep it out of the node executor: rebuilding/scanning all executor
    // entities for every IMU packet was itself a measured CPU hotspot.
    receive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = receive_group_;
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        input, rclcpp::SensorDataQoS().keep_last(1),
        [](sensor_msgs::msg::Imu::ConstSharedPtr) {}, options);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this, input, stale_after] {
      std_msgs::msg::Float64MultiArray sample;
      bool pending;
      {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        sample.data = {1.0, latency_ms_, received_s_};
        pending = pending_;
        pending_ = false;
      }
      // Audit samples are only emitted for a new receipt. The public display
      // heartbeat evaluates age independently and goes false after source loss.
      if (pending) publisher_->publish(sample);
      if (fresh_publisher_) {
        const double age = monotonic_seconds() - sample.data[2];
        std_msgs::msg::Bool fresh;
        fresh.data = sample.data[2] > 0 && age >= 0 && age <= stale_after && std::isfinite(sample.data[1]);
        fresh_publisher_->publish(fresh);
        std_msgs::msg::String text;
        if (fresh.data) {
          std_msgs::msg::Float64 latency;
          latency.data = sample.data[1];
          latency_publisher_->publish(latency);
          char number[64];
          std::snprintf(number, sizeof(number), "%+.3f", latency.data);
          text.data = "A3 ROS clock - " + input + " header stamp = " + number + " ms";
        } else {
          text.data = "NO FRESH TIMESTAMP | " + input;
        }
        text_publisher_->publish(text);
      }
    });
    interfaces_ = interfaces();
    network_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      if (interfaces() != interfaces_)
        throw std::runtime_error("Network interfaces changed; recreate native IMU DDS");
    });
    RCLCPP_INFO(get_logger(), "Native IMU telemetry: %s -> %s at max %.1f Hz", input.c_str(), output.c_str(), hz);
    receiver_ = std::thread([this] { receive(); });
  }
  ~ImuTelemetry() override {
    stopping_.store(true);
    if (receiver_.joinable()) receiver_.join();
  }
 private:
  void receive() {
    try {
      rclcpp::WaitSet wait_set;
      rclcpp::SubscriptionWaitSetMask mask;
      mask.include_events = false;
      mask.include_intra_process_waitable = false;
      wait_set.add_subscription(subscription_, mask);
      sensor_msgs::msg::Imu message;
      rclcpp::MessageInfo info;
      while (!stopping_.load() && rclcpp::ok()) {
        const auto ready = wait_set.wait(std::chrono::milliseconds(100));
        if (ready.kind() != rclcpp::WaitResultKind::Ready || !subscription_->take(message, info)) continue;
        const double receipt = monotonic_seconds();
        const auto stamp = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
                           message.header.stamp.nanosec;
        const double latency = stamp > 0 ? (get_clock()->now().nanoseconds() - stamp) / 1e6
                                        : std::numeric_limits<double>::quiet_NaN();
        std::lock_guard<std::mutex> lock(sample_mutex_);
        latency_ms_ = latency;
        received_s_ = receipt;
        pending_ = true;
      }
    } catch (const std::exception& error) {
      if (!stopping_.load() && rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "IMU reader failed: %s", error.what());
        // Fail stale without restarting the independent safety monitor.
        // Exit nonzero below so systemd reconstructs this DDS participant.
        reader_failed_.store(true);
        rclcpp::shutdown();
      }
    }
  }
 public:
  bool reader_failed() const { return reader_failed_.load(); }
 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr receive_group_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fresh_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr network_timer_;
  std::set<std::string> interfaces_;
  std::thread receiver_;
  std::mutex sample_mutex_;
  std::atomic<bool> stopping_{false}, reader_failed_{false};
  double latency_ms_ = 0.0, received_s_ = 0.0;
  bool pending_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ImuTelemetry>();
    rclcpp::spin(node);
    if (node->reader_failed()) return 1;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("hope_imu_telemetry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
