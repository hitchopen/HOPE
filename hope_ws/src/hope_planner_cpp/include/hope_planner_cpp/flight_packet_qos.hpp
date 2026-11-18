#pragma once

#include <hope_msgs/msg/ball_flight_packet.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace hope_planner_cpp {

constexpr std::size_t kFlightPacketHistoryDepth = 8;
constexpr std::int64_t kFlightPacketMaxAgeNs = 100'000'000;
constexpr std::int64_t kFlightPacketFutureToleranceNs = 50'000'000;

inline rclcpp::QoS flight_packet_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(kFlightPacketHistoryDepth))
      .reliable().durability_volatile()
      .lifespan(rclcpp::Duration(std::chrono::nanoseconds(kFlightPacketMaxAgeNs)));
}

// DDS lifespan starts again on every publish. These original timestamps do not:
// retries must never revive an old flight, including after a reliable backlog.
inline bool flight_packet_transport_fresh(
    std::int64_t freeze_ns, std::int64_t last_source_ns,
    std::int64_t now_ns) noexcept {
  const auto fresh = [now_ns](std::int64_t stamp) {
    return stamp > 0 && stamp >= now_ns - kFlightPacketMaxAgeNs &&
        stamp <= now_ns + kFlightPacketFutureToleranceNs;
  };
  return fresh(freeze_ns) && fresh(last_source_ns);
}

// No DDS call runs while the queue lock is held. Even a blocked middleware
// writer cannot block mocap admission, grow memory, or preserve cancelled work.
class BoundedFlightPacketSender {
 public:
  using Packet = hope_msgs::msg::BallFlightPacket;
  using Send = std::function<void(Packet)>;
  using WallClock = std::function<std::int64_t()>;

  explicit BoundedFlightPacketSender(Send send, WallClock wall_clock)
      : send_(std::move(send)), wall_clock_(std::move(wall_clock)),
        worker_([this] { run(); }) {}
  ~BoundedFlightPacketSender() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      queue_.clear();
    }
    wake_.notify_one();
    worker_.join();
  }

  void set_gate(std::uint64_t generation, bool allowed) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_ || !allowed) queue_.clear();
    generation_ = generation;
    allowed_ = allowed;
  }

  bool enqueue(Packet packet, std::uint64_t generation) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || !allowed_ || generation != generation_) return false;
      if (queue_.size() == kFlightPacketHistoryDepth) queue_.pop_front();
      queue_.push_back({std::move(packet), generation,
          std::chrono::steady_clock::now()});
    }
    wake_.notify_one();
    return true;
  }

  std::size_t pending_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

 private:
  struct Pending {
    Packet packet;
    std::uint64_t generation;
    std::chrono::steady_clock::time_point queued_at;
  };
  void run() {
    for (;;) {
      Pending next;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
        if (stopping_) return;
        next = std::move(queue_.front());
        queue_.pop_front();
        if (!allowed_ || next.generation != generation_) continue;
      }
      if (next.packet.samples.empty() ||
          std::chrono::steady_clock::now() - next.queued_at >
              std::chrono::nanoseconds(kFlightPacketMaxAgeNs) ||
          !flight_packet_transport_fresh(next.packet.freeze_wall_unix_ns,
              next.packet.samples.back().exposure_unix_stamp_ns, wall_clock_())) {
        continue;
      }
      // A write already admitted to DDS cannot be recalled; the remote reader
      // checks the original timestamps again before admission and before solve.
      send_(std::move(next.packet));
    }
  }

  Send send_;
  WallClock wall_clock_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Pending> queue_;
  std::uint64_t generation_ = 0;
  bool allowed_ = false;
  bool stopping_ = false;
  std::thread worker_;
};

}  // namespace hope_planner_cpp
