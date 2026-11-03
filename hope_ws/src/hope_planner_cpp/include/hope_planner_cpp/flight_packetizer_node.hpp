#pragma once

#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/laptop_play_state.hpp"
#include "hope_planner_cpp/local_play_state_status.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <hope_msgs/msg/ball_flight_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace hope_planner_cpp {

class FlightPacketizerNode final : public rclcpp::Node {
 public:
  explicit FlightPacketizerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  int local_status_port() const { return local_status_ ? local_status_->port() : 0; }

 private:
  struct PendingTransmit {
    hope_msgs::msg::BallFlightPacket message;
    std::int64_t due_steady_ns = 0;
    std::uint64_t generation = 0;
  };

  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr message);
  void prepare_snapshot(
      const TrajectorySnapshot& snapshot,
      const std::string& frame_id);
  void transmit_packet(const hope_msgs::msg::BallFlightPacket& packet);
  void retransmit_due();
  bool sync_play_state();
  void publish_play_state();
  void publish_gated_pose(const geometry_msgs::msg::PoseArray& message, std::int64_t source_ns);
  void publish_copy(
      const hope_msgs::msg::BallFlightPacket& base,
      std::uint8_t transmit_index);
  void write_audit(
      const hope_msgs::msg::BallFlightPacket& message,
      const char* event);

  static std::int64_t steady_now_ns() noexcept;
  static std::int64_t wall_now_ns() noexcept;

  IncomingTrajectoryConfig trajectory_config_;
  CommitPredictionConfig prediction_config_;
  IncomingTrajectory trajectory_;
  LaptopPlayState play_state_;
  std::unique_ptr<BatchPhysicsEstimator> state_estimator_;
  SpinEstimator state_spin_;
  std::uint64_t state_generation_ = 0;
  double fit_bounce_time_ = -1e9;
  double forecast_time_ = 0;
  std::int64_t state_publish_ns_ = 0;
  std::string last_state_text_;
  using DisplayKey = std::tuple<std::string, PlayState, std::string, std::string,
      std::uint64_t, bool, bool, bool, bool, bool, int, int, bool, std::uint64_t, bool>;
  std::optional<DisplayKey> last_state_key_;
  std::int64_t last_forwarded_pose_ns_ = 0;
  std::uint64_t local_intent_revision_ = 0;
  std::string last_frame_id_ = "world";
  double hit_plane_stamp_s_ = 0.0;
  double last_hit_plane_x_ = 0.15;
  std::string session_id_;
  std::string producer_instance_id_;
  std::string packet_topic_;
  int ball_pose_index_ = 0;
  std::vector<std::int64_t> retransmit_delays_ms_;
  std::atomic<std::uint64_t> sample_sequence_{0};
  std::deque<PendingTransmit> pending_transmits_;
  std::mutex pending_mutex_;
  std::ofstream audit_stream_;
  std::mutex audit_mutex_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hit_plane_subscription_;
  rclcpp::Publisher<hope_msgs::msg::BallFlightPacket>::SharedPtr packet_publisher_;
  std::unique_ptr<LocalPlayStateStatus> local_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr gated_pose_publisher_;
  rclcpp::TimerBase::SharedPtr retransmit_timer_;
};

}  // namespace hope_planner_cpp
