#pragma once
#include "hope_planner_cpp/hybrid_trajectory_coordinator.hpp"
#include "hope_planner_cpp/hit_plane_manager.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <hope_msgs/msg/hybrid_trajectory_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <hope_msgs/msg/ball_forecast.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
namespace hope_planner_cpp {
class HybridTrajectoryNode : public rclcpp::Node {
public:
  explicit HybridTrajectoryNode(const rclcpp::NodeOptions & = rclcpp::NodeOptions());
private:
  void observation(const geometry_msgs::msg::PoseArray &);
  void context(const hope_msgs::msg::HybridTrajectoryContext &);
  void tick();
  bool publish_forecast(const ForecastUpdate &, bool compact);
  rclcpp::Publisher<hope_msgs::msg::BallForecast>::SharedPtr forecast_pub_, forecast_event_pub_;
  void activate_plane(std::int64_t now);
  void diagnostics();
  ProbabilisticConfig config_;
  ForecastNoiseConfig forecast_noise_;
  ReturnRegionPolicy policy_;
  std::unique_ptr<HitPlaneManager> plane_;
  HitPlaneSnapshot active_plane_;
  bool rally_active_=false;
  std::uint64_t rally_id_=0, projection_sequence_=0;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr freeze_service_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::unique_ptr<HybridTrajectoryCoordinator> coordinator_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr observation_sub_;
  rclcpp::Subscription<hope_msgs::msg::HybridTrajectoryContext>::SharedPtr context_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_, registration_id_, instance_, clock_domain_;
  bool frame_registered_ = false, source_clock_registered_ = false;
  int ball_index_ = 0;
  std::uint64_t sequence_ = 0;
  std::int64_t callback_ns_ = 0, ckf_complete_ns_ = 0;
  std::optional<std::int64_t> last_ros_ns_, last_steady_ns_;
  std::deque<hope_msgs::msg::HybridTrajectoryContext> pending_sim_context_;
  // Read-only integration accounting; never participates in estimator decisions.
  std::uint64_t observation_callbacks_=0, accepted_updates_=0;
  std::uint64_t compact_evaluated_=0, compact_published_=0, full_published_=0;
  std::int64_t last_accepted_ns_=0, last_forecast_ns_=0;
  std::int64_t last_message_build_ns_=0, last_publish_call_ns_=0;
  std::string last_invalidation_reason_="startup";
  double max_age_ = .030;
};
} // namespace hope_planner_cpp
