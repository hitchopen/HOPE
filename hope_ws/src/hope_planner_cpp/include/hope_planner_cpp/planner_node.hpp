#pragma once

#include "hope_planner_cpp/audit_logger.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/flight_packet.hpp"
#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/post_net_one_shot.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"
#include "hope_planner_cpp/spsc_ring.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hope_msgs/msg/ball_flight_packet.hpp>
#include <hope_msgs/msg/racket_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace hope_planner_cpp {

class PlannerNode final : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PlannerNode() override;

 private:
  struct BaseSnapshot {
    Vec3 position = Vec3::Zero();
    bool valid = false;
    std::int64_t receipt_steady_ns = 0;
    std::int64_t source_stamp_ns = 0;
  };

  struct CalibrationSample {
    std::int64_t receipt_steady_ns = 0;
    double base_x = 0.0;
  };

  void ball_callback(const geometry_msgs::msg::PoseArray::SharedPtr message);
  void flight_packet_callback(
      const hope_msgs::msg::BallFlightPacket::SharedPtr message);
  void base_flat_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void solver_loop() noexcept;
  void publish_solve(
      const BallSample& latest_sample,
      const BallState& state,
      const StrikeTarget& strike,
      const RacketCommand& command,
      double swing_sign,
      const SolveAudit& audit,
      const SpinShadowAudit& spin_shadow,
      const PostNetOneShotEvent& one_shot_event,
      std::int64_t solve_finished_steady_ns) noexcept;
  void publish_diagnostics() noexcept;
  void poll_calibration_request() noexcept;
  std::pair<bool, std::string> calibrate_x_hit() noexcept;
  void add_base_sample(double base_x, std::int64_t receipt_steady_ns) noexcept;
  void set_base_snapshot(const BaseSnapshot& snapshot) noexcept;
  BaseSnapshot base_snapshot() const noexcept;
  bool try_take_flight_packet(TrajectorySnapshot& snapshot) noexcept;
  bool has_pending_flight_packet() const noexcept;
  std::size_t flight_packet_queue_depth() const noexcept;
  double select_swing_sign(double intercept_y, double base_y) noexcept;
  double active_x_hit() const noexcept;

  static std::int64_t steady_now_ns() noexcept;
  static std::int64_t wall_now_ns() noexcept;

  BallPhysics physics_;
  TableParams table_;
  PlannerConfig planner_config_;
  EstimatorConfig estimator_config_;
  SpinEstimatorConfig spin_estimator_config_;
  IncomingTrajectoryConfig incoming_trajectory_config_;
  std::unique_ptr<BatchPhysicsEstimator> estimator_;
  std::unique_ptr<SpinEstimator> spin_estimator_;
  std::unique_ptr<TrajectoryPredictor> predictor_;
  std::unique_ptr<TrajectoryPredictor> post_net_predictor_;
  std::unique_ptr<RacketTargetPlanner> target_planner_;
  std::unique_ptr<IncomingTrajectory> incoming_trajectory_;

  SpscRing<BallSample, kInputRingCapacity> input_ring_;
  LatestSnapshotMailbox snapshot_mailbox_;
  FlightPacketDeduplicator flight_packet_deduplicator_{256};
  mutable std::mutex flight_packet_mutex_;
  std::deque<TrajectorySnapshot> flight_packet_queue_;
  std::mutex wake_mutex_;
  std::condition_variable wake_condition_;
  std::thread solver_thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> estimator_reset_requested_{false};
  std::atomic<bool> trajectory_reset_requested_{false};

  mutable std::mutex base_mutex_;
  BaseSnapshot base_;
  std::mutex calibration_mutex_;
  std::deque<CalibrationSample> calibration_samples_;

  Schema2Packer schema2_packer_;
  std::unique_ptr<AuditLogger> audit_logger_;
  std::unique_ptr<AuditLogger> flight_packet_audit_logger_;

  rclcpp::CallbackGroup::SharedPtr ball_callback_group_;
  rclcpp::CallbackGroup::SharedPtr base_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ball_subscription_;
  rclcpp::Subscription<hope_msgs::msg::BallFlightPacket>::SharedPtr
      flight_packet_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr flat_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr serve_ball_publisher_;
  rclcpp::Publisher<hope_msgs::msg::RacketCommand>::SharedPtr typed_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;

  int ball_pose_index_ = 0;
  int input_qos_depth_ = 64;
  double solve_period_s_ = 0.033;
  double expected_mocap_hz_ = 360.0;
  double policy_z_offset_ = 0.76;
  double x_hit_bh_delta_ = 0.0;
  bool x_hit_follow_robot_ = false;
  double x_hit_offset_ = 0.65;
  double x_hit_min_ = -0.30;
  double x_hit_max_ = 0.30;
  std::atomic<double> x_hit_fh_{0.15};
  std::atomic<bool> x_hit_calibrated_{false};
  bool require_x_hit_calibration_audit_ = false;
  double x_hit_calibration_offset_ = 0.58;
  double x_hit_calibration_window_s_ = 0.5;
  double x_hit_calibration_max_age_s_ = 0.2;
  int x_hit_calibration_min_samples_ = 10;
  double x_hit_calibration_max_span_m_ = 0.01;
  std::string x_hit_request_file_;
  std::string x_hit_status_file_;
  std::string session_id_;
  bool post_net_one_shot_enabled_ = true;
  bool flight_packet_input_enabled_ = false;
  std::string flight_packet_topic_ = "/ball/flight_packet";
  double post_net_commit_delay_s_ = 0.05;
  double post_net_future_bounce_tangential_gain_ = 0.075;

  double swing_side_split_y_ = -0.25;
  double swing_side_hysteresis_y_ = 0.04;
  double last_swing_sign_ = 0.0;
  double target_land_y_fh_ = std::numeric_limits<double>::quiet_NaN();
  double target_land_y_bh_ = std::numeric_limits<double>::quiet_NaN();
  double delta_t_flight_fh_ = std::numeric_limits<double>::quiet_NaN();
  double delta_t_flight_bh_ = std::numeric_limits<double>::quiet_NaN();

  bool publish_flat_command_ = true;
  bool publish_base_flat_ = false;
  bool publish_serve_ball_flat_ = true;
  std::vector<double> marker_to_base_xyz_{0.0, 0.0, 0.0};
  bool spin_shadow_enabled_ = false;
  SpinPhysicsMode spin_shadow_mode_ =
      SpinPhysicsMode::kVenueGripBounceAndMagnus;
  std::string spin_shadow_mode_name_ = "venue_grip_magnus";

  std::atomic<std::uint64_t> received_samples_{0};
  std::atomic<std::uint64_t> present_samples_{0};
  std::atomic<std::uint64_t> missing_samples_{0};
  std::atomic<std::uint64_t> ring_drops_{0};
  std::atomic<std::uint64_t> out_of_order_samples_{0};
  std::atomic<std::uint64_t> solver_input_samples_{0};
  std::atomic<std::uint64_t> solver_coalesced_samples_{0};
  std::atomic<std::uint64_t> maximum_solver_batch_samples_{0};
  std::atomic<std::uint64_t> trajectory_epoch_{0};
  std::atomic<int> incoming_phase_{
      static_cast<int>(IncomingPhase::kSeekIncoming)};
  std::atomic<std::uint64_t> trajectory_source_resets_{0};
  std::atomic<std::uint64_t> flight_packets_received_{0};
  std::atomic<std::uint64_t> flight_packets_accepted_{0};
  std::atomic<std::uint64_t> flight_packets_duplicate_{0};
  std::atomic<std::uint64_t> flight_packets_conflict_{0};
  std::atomic<std::uint64_t> flight_packets_invalid_{0};
  std::atomic<std::uint64_t> solve_count_{0};
  std::atomic<std::uint64_t> valid_count_{0};
  std::atomic<std::uint64_t> spin_valid_count_{0};
  std::atomic<std::uint64_t> spin_shadow_valid_count_{0};
  std::atomic<std::uint64_t> solver_deadline_miss_count_{0};
  std::atomic<std::uint64_t> sample_sequence_{0};
  std::atomic<std::int64_t> last_accepted_source_stamp_ns_{0};
  std::atomic<std::int64_t> last_source_age_at_callback_ns_{0};
  std::atomic<std::int64_t> last_ball_receipt_steady_ns_{0};
  std::atomic<std::int64_t> maximum_ball_gap_ns_{0};
  std::uint64_t health_previous_received_ = 0;
  std::uint64_t health_previous_solves_ = 0;
  std::int64_t health_previous_steady_ns_ = 0;
};

}  // namespace hope_planner_cpp
