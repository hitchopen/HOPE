#pragma once

#include "hope_planner_cpp/incoming_trajectory.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <hope_msgs/msg/ball_flight_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace hope_planner_cpp {

class FlightPacketizerNode final : public rclcpp::Node {
 public:
  explicit FlightPacketizerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  struct PendingTransmit {
    hope_msgs::msg::BallFlightPacket message;
    std::int64_t due_steady_ns = 0;
  };

  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr message);
  void publish_snapshot(
      const TrajectorySnapshot& snapshot,
      const std::string& frame_id);
  void retransmit_due();
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
  std::string last_frame_id_ = "world";
  double hit_plane_stamp_s_ = 0.0;
  double last_hit_plane_x_ = 0.15;
  std::string session_id_;
  std::string producer_instance_id_;
  std::string packet_topic_;
  int ball_pose_index_ = 0;
  std::vector<std::int64_t> retransmit_delays_ms_;
  std::atomic<std::uint64_t> sample_sequence_{0};
  std::atomic<std::int64_t> last_source_stamp_ns_{0};
  std::deque<PendingTransmit> pending_transmits_;
  std::mutex pending_mutex_;
  std::ofstream audit_stream_;
  std::mutex audit_mutex_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hit_plane_subscription_;
  rclcpp::Publisher<hope_msgs::msg::BallFlightPacket>::SharedPtr packet_publisher_;
  rclcpp::TimerBase::SharedPtr retransmit_timer_;
};

}  // namespace hope_planner_cpp
