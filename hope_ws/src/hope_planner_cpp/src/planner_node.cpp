#include "hope_planner_cpp/planner_node.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace hope_planner_cpp {
namespace {

using namespace std::chrono_literals;

constexpr int kRequiredBaseFlags =
    (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5);

diagnostic_msgs::msg::KeyValue diagnostic_value(
    const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

std::string number_string(double value, int precision = 6) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

double ns_to_seconds(std::int64_t value) noexcept {
  return static_cast<double>(value) * 1.0e-9;
}

std::pair<SpinPhysicsMode, std::string> parse_spin_mode(
    const std::string& requested) {
  if (requested == "nakashima") {
    return {SpinPhysicsMode::kNakashimaBounce, requested};
  }
  if (requested == "nakashima_magnus") {
    return {SpinPhysicsMode::kNakashimaBounceAndMagnus, requested};
  }
  if (requested == "venue_grip") {
    return {SpinPhysicsMode::kVenueGripBounce, requested};
  }
  if (requested == "venue_grip_magnus") {
    return {SpinPhysicsMode::kVenueGripBounceAndMagnus, requested};
  }
  throw std::invalid_argument(
      "spin_shadow_mode must be nakashima, nakashima_magnus, "
      "venue_grip, or venue_grip_magnus");
}

}  // namespace

std::int64_t PlannerNode::steady_now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t PlannerNode::wall_now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hope_planner", options) {
  declare_parameter<std::string>("ball_rigid_body_name", "pingpong_ball");
  ball_pose_index_ = static_cast<int>(declare_parameter<int>("ball_pose_index", 0));
  input_qos_depth_ = std::clamp(
      static_cast<int>(declare_parameter<int>("input_qos_depth", 64)), 1, 1024);
  solve_period_s_ = std::max(0.001, declare_parameter<double>("solve_period_s", 0.033));
  expected_mocap_hz_ = std::max(1.0, declare_parameter<double>("expected_mocap_hz", 360.0));

  x_hit_fh_.store(declare_parameter<double>("x_hit", 0.15));
  x_hit_follow_robot_ = declare_parameter<bool>("x_hit_follow_robot", false);
  x_hit_offset_ = declare_parameter<double>("x_hit_offset", 0.65);
  x_hit_min_ = declare_parameter<double>("x_hit_min", -0.30);
  x_hit_max_ = declare_parameter<double>("x_hit_max", 0.30);
  x_hit_bh_delta_ = declare_parameter<double>("x_hit_bh_delta", 0.0);
  require_x_hit_calibration_audit_ =
      declare_parameter<bool>("require_x_hit_calibration", false);
  x_hit_calibration_offset_ = declare_parameter<double>("x_hit_calibration_offset", 0.58);
  x_hit_calibration_window_s_ =
      declare_parameter<double>("x_hit_calibration_window_s", 0.5);
  x_hit_calibration_max_age_s_ =
      declare_parameter<double>("x_hit_calibration_max_age_s", 0.2);
  x_hit_calibration_min_samples_ = static_cast<int>(
      declare_parameter<int>("x_hit_calibration_min_samples", 10));
  x_hit_calibration_max_span_m_ =
      declare_parameter<double>("x_hit_calibration_max_span_m", 0.01);
  x_hit_request_file_ =
      declare_parameter<std::string>("x_hit_calibration_request_file", "");
  x_hit_status_file_ =
      declare_parameter<std::string>("x_hit_calibration_status_file", "");

  swing_side_split_y_ = declare_parameter<double>("swing_side_split_y", -0.25);
  swing_side_hysteresis_y_ =
      std::max(0.0, declare_parameter<double>("swing_side_hysteresis_y", 0.04));
  target_land_y_fh_ = declare_parameter<double>(
      "target_land_y_fh", std::numeric_limits<double>::quiet_NaN());
  target_land_y_bh_ = declare_parameter<double>(
      "target_land_y_bh", std::numeric_limits<double>::quiet_NaN());
  delta_t_flight_fh_ = declare_parameter<double>(
      "delta_t_flight_fh", std::numeric_limits<double>::quiet_NaN());
  delta_t_flight_bh_ = declare_parameter<double>(
      "delta_t_flight_bh", std::numeric_limits<double>::quiet_NaN());

  physics_.drag_k = declare_parameter<double>("drag_k", 0.1261);
  physics_.magnus_k = declare_parameter<double>("magnus_k", 0.00444);
  physics_.restitution_h = declare_parameter<double>("restitution_h", 0.64);
  physics_.restitution_v = declare_parameter<double>("restitution_v", 0.9215);
  physics_.nakashima_friction_mu =
      declare_parameter<double>("nakashima_friction_mu", 0.25);
  physics_.table_tangential_gain =
      declare_parameter<double>("table_tangential_gain", 0.369);
  physics_.table_friction_cap_mu =
      declare_parameter<double>("table_friction_cap_mu", 2.0);
  planner_config_.restitution_racket =
      declare_parameter<double>("restitution_racket", 0.654);
  planner_config_.restitution_exp_g1 =
      declare_parameter<double>("restitution_exp_g1", 0.759);
  planner_config_.restitution_exp_g2 =
      declare_parameter<double>("restitution_exp_g2", -0.0441);
  planner_config_.x_hit = x_hit_fh_.load();
  planner_config_.target_land = Vec3(
      declare_parameter<double>("target_land_x", 2.055),
      declare_parameter<double>("target_land_y", -0.7625), 0.0);
  planner_config_.delta_t_flight = declare_parameter<double>("delta_t_flight", 0.50);
  planner_config_.integrate_dt_s = declare_parameter<double>("dt_integrate", 0.001);
  planner_config_.max_predict_time_s =
      declare_parameter<double>("max_predict_time", 2.0);
  planner_config_.adaptive_predict_horizon =
      declare_parameter<bool>("adaptive_predict_horizon", true);
  planner_config_.max_predict_time_cap_s =
      declare_parameter<double>("max_predict_time_cap", 3.0);
  table_.y_max = declare_parameter<double>("table_y_max", 0.0);
  table_.net_x = declare_parameter<double>("net_x", 1.37);
  post_net_one_shot_enabled_ =
      declare_parameter<bool>("post_net_one_shot_enabled", true);
  flight_packet_input_enabled_ =
      declare_parameter<bool>("flight_packet_input_enabled", false);
  flight_packet_topic_ = declare_parameter<std::string>(
      "flight_packet_topic", "/ball/flight_packet");
  if (flight_packet_input_enabled_ && !post_net_one_shot_enabled_) {
    post_net_one_shot_enabled_ = true;
    RCLCPP_WARN(
        get_logger(),
        "flight_packet_input_enabled requires one solve per immutable flight; "
        "post_net_one_shot_enabled was promoted to true");
  }
  post_net_commit_delay_s_ = std::max(
      0.0, declare_parameter<double>("post_net_commit_delay_s", 0.05));
  post_net_future_bounce_tangential_gain_ = std::max(
      0.0, declare_parameter<double>(
          "post_net_future_bounce_tangential_gain", 0.075));

  incoming_trajectory_config_.net_x = table_.net_x;
  incoming_trajectory_config_.commit_delay_s = post_net_commit_delay_s_;
  incoming_trajectory_config_.opponent_side_margin_m = std::max(
      0.0, declare_parameter<double>("incoming_opponent_side_margin_m", 0.05));
  incoming_trajectory_config_.incoming_speed_threshold_mps = std::max(
      0.01, declare_parameter<double>("incoming_speed_threshold_mps", 0.25));
  incoming_trajectory_config_.outgoing_speed_threshold_mps = std::max(
      0.01, declare_parameter<double>("outgoing_speed_threshold_mps", 0.25));
  incoming_trajectory_config_.source_gap_reset_s = std::max(
      0.02, declare_parameter<double>("incoming_source_gap_reset_s", 0.25));
  incoming_trajectory_config_.direction_fit_samples =
      static_cast<std::size_t>(std::max(
          3, static_cast<int>(declare_parameter<int>(
                 "incoming_direction_fit_samples", 4))));
  incoming_trajectory_config_.direction_confirmations =
      static_cast<std::size_t>(std::max(
          1, static_cast<int>(declare_parameter<int>(
                 "incoming_direction_confirmations", 2))));
  incoming_trajectory_config_.pre_roll_samples =
      static_cast<std::size_t>(std::max(
          6, static_cast<int>(declare_parameter<int>(
                 "incoming_pre_roll_samples", 24))));

  estimator_config_.window_s =
      declare_parameter<double>("batch_estimator_window_s", 0.18);
  incoming_trajectory_config_.estimator_window_s = estimator_config_.window_s;
  estimator_config_.min_span_s =
      declare_parameter<double>("batch_estimator_min_span_s", 0.08);
  estimator_config_.min_samples = static_cast<std::size_t>(std::max(
      6, static_cast<int>(declare_parameter<int>("batch_estimator_min_samples", 12))));
  estimator_config_.huber_delta_m =
      declare_parameter<double>("batch_estimator_huber_delta_m", 0.003);
  estimator_config_.recency_half_life_s =
      declare_parameter<double>("batch_estimator_recency_half_life_s", 0.0);
  estimator_config_.robust_iterations = static_cast<int>(
      declare_parameter<int>("batch_estimator_iterations", 3));
  estimator_config_.integration_dt_s = planner_config_.integrate_dt_s;
  estimator_config_.bounce_center_z_max_m =
      declare_parameter<double>("bounce_center_z_max", 0.20);
  estimator_config_.bounce_min_reversal_m =
      declare_parameter<double>("bounce_min_reversal_m", 0.00005);
  estimator_config_.bounce_min_excursion_m =
      declare_parameter<double>("bounce_min_excursion_m", 0.001);
  estimator_config_.bounce_confirmation_samples = static_cast<std::size_t>(std::max(
      1, static_cast<int>(
             declare_parameter<int>("bounce_confirmation_samples", 5))));
  estimator_config_.bounce_confirmation_max_span_s =
      declare_parameter<double>("bounce_confirmation_max_span_s", 0.05);
  estimator_config_.bounce_sparse_confirmation_min_span_s =
      declare_parameter<double>("bounce_sparse_confirmation_min_span_s", 0.012);
  estimator_config_.bounce_sparse_confirmation_excursion_m =
      declare_parameter<double>("bounce_sparse_confirmation_excursion_m", 0.005);
  estimator_config_.bounce_refractory_s =
      declare_parameter<double>("bounce_refractory_s", 0.12);

  // Legacy parameters are accepted only so the existing two YAML files can be
  // reused while field commands migrate.  They do not select a recursive
  // estimator or alter release behavior in this executable.
  declare_parameter<int>("fit_window", 37);
  declare_parameter<double>("fit_window_min_span_s", 0.0);
  declare_parameter<double>("fit_window_max_span_s", 0.15);
  declare_parameter<int>("horizontal_poly_order", 0);
  const bool requested_kalman = declare_parameter<bool>("use_kalman", false);
  declare_parameter<double>("estimator_q_accel_psd", 1.0);
  declare_parameter<double>("estimator_sigma_ar1_m", 0.0006);
  declare_parameter<double>("estimator_chi2_gate", 16.3);
  declare_parameter<double>("estimator_track_gap_s", 0.08);
  spin_shadow_enabled_ = declare_parameter<bool>("spin_shadow_enabled", false);
  const auto spin_mode = parse_spin_mode(declare_parameter<std::string>(
      "spin_shadow_mode", "venue_grip_magnus"));
  spin_shadow_mode_ = spin_mode.first;
  spin_shadow_mode_name_ = spin_mode.second;
  declare_parameter<std::string>("ball_orientation_topic", "/ball/pose");
  spin_estimator_config_.window_s =
      declare_parameter<double>("spin_window_s", 0.10);
  spin_estimator_config_.min_span_s =
      declare_parameter<double>("spin_min_span_s", 0.05);
  declare_parameter<double>("spin_gate_rev_s", 3.0);
  spin_estimator_config_.max_gap_s =
      declare_parameter<double>("spin_max_gap_s", 0.05);
  spin_estimator_config_.max_rev_s =
      declare_parameter<double>("spin_max_rev_s", 20.0);
  spin_estimator_config_.huber_delta_rev_s =
      declare_parameter<double>("spin_huber_delta_rev_s", 2.0);
  spin_estimator_config_.min_increments = static_cast<std::size_t>(std::max(
      1, static_cast<int>(declare_parameter<int>("spin_min_increments", 3))));
  spin_estimator_config_.robust_iterations = std::max(
      1, static_cast<int>(declare_parameter<int>("spin_robust_iterations", 5)));
  declare_parameter<bool>("publish_strike_spec", false);
  declare_parameter<double>("racket_speed_budget", 10.0);
  if (requested_kalman) {
    RCLCPP_WARN(
        get_logger(),
        "use_kalman=true was supplied by a legacy command but is ignored: "
        "this executable contains only batch_physics_cpp_no_ekf");
  }

  policy_z_offset_ = declare_parameter<double>("policy_z_offset", 0.76);
  publish_flat_command_ = declare_parameter<bool>("publish_flat_cmd", true);
  publish_base_flat_ = declare_parameter<bool>("publish_base_flat", false);
  publish_serve_ball_flat_ = declare_parameter<bool>("publish_serve_ball_flat", true);
  const int flat_schema = static_cast<int>(
      declare_parameter<int>("racket_flat_schema", 2));
  if (flat_schema != 2) {
    throw std::invalid_argument("hope_planner_cpp supports only the model_21800 schema-2 wire");
  }
  const std::string flat_topic =
      declare_parameter<std::string>("racket_flat_topic", "/racket/command_flat");
  const std::string base_flat_topic =
      declare_parameter<std::string>("base_flat_topic", "/a3/base_pose_flat");
  const std::string serve_ball_topic =
      declare_parameter<std::string>("serve_ball_flat_topic", "/serve/ball_state_flat");
  const bool publish_typed = declare_parameter<bool>("publish_typed_command", true);
  marker_to_base_xyz_ = declare_parameter<std::vector<double>>(
      "marker_to_base_xyz", std::vector<double>{0.0, 0.0, 0.0});
  if (marker_to_base_xyz_.size() != 3) {
    throw std::invalid_argument("marker_to_base_xyz must contain exactly three doubles");
  }

  session_id_ = declare_parameter<std::string>("debug_session_id", "");
  const std::string debug_path =
      declare_parameter<std::string>("debug_csv_path", "");
  declare_parameter<int>("debug_flush_rows", 64);
  audit_logger_ = std::make_unique<AuditLogger>(
      debug_path,
      "session_id,sample_seq,solve_seq,command_seq,flight_id,revision_id,"
      "source_time_s,source_age_at_publish_ms,strike_deadline_wall_s,"
      "producer_wall_s,receipt_steady_ns,solve_finish_steady_ns,wire_valid,reason,"
      "ball_x,ball_y,ball_z,estimator_samples,estimator_span_s,est_x,est_y,est_z,"
      "est_vx,est_vy,est_vz,fit_rms_m,fit_max_m,strike_x,strike_y,strike_z,"
      "strike_vx,strike_vy,strike_vz,racket_vx,racket_vy,racket_vz,swing_sign,tts_s,"
      "estimator_ms,stage2_ms,stage3_ms,total_ms,input_ring_depth,input_ring_drops,"
      "logger_queue_depth,logger_drops,base_valid,base_x,base_y,base_z,base_age_ms,"
      "estimator_kind,spin_shadow_enabled,spin_shadow_mode,spin_valid,spin_reason,"
      "spin_wx_rad_s,spin_wy_rad_s,spin_wz_rad_s,spin_magnitude_rev_s,spin_span_s,"
      "spin_retained_time_fraction,spin_coherence,spin_retained_increments,"
      "spin_rejected_increments,spin_shadow_valid,spin_shadow_reason,"
      "spin_shadow_strike_x,spin_shadow_strike_y,spin_shadow_strike_z,"
      "spin_shadow_strike_vx,spin_shadow_strike_vy,spin_shadow_strike_vz,"
      "spin_shadow_strike_time_s,spin_shadow_bounces,spin_shadow_delta_y_m,"
      "spin_shadow_delta_z_m,spin_shadow_delta_t_s,bounce_transition_used,"
      "bounce_source_time_s,pre_bounce_samples,post_bounce_samples,"
      "control_stage2_mode,source_age_at_callback_ms,callback_to_solve_ms,"
      "solver_batch_samples,solver_coalesced_samples,solver_input_samples_total,"
      "solver_coalesced_samples_total,out_of_order_samples_total,input_qos_depth,"
      "bounce_epoch_active,one_shot_enabled,one_shot_flight_seq,"
      "net_cross_source_time_s,commit_source_time_s,post_net_commit_delay_s,"
      "post_net_future_bounce_tangential_gain,trajectory_epoch,snapshot_sequence,"
      "segment_boundary_reason,segment_start_source_time_s,"
      "previous_segment_last_source_time_s,previous_tail_to_commit_ms,"
      "snapshot_published_total,snapshot_consumed_total,snapshot_superseded_total,"
      "flight_packet_input,packet_session_id,packet_producer_instance_id,"
      "packet_payload_hash,packet_transmit_index,packet_transmit_count,"
      "packet_transport_age_ms,packet_freeze_to_receive_ms,"
      "packet_received_total,packet_accepted_total,packet_duplicate_total,"
      "packet_conflict_total,packet_invalid_total,packet_queue_depth");
  std::string flight_packet_audit_path = declare_parameter<std::string>(
      "flight_packet_audit_csv_path", "");
  if (flight_packet_audit_path.empty() && !debug_path.empty()) {
    flight_packet_audit_path = debug_path + ".flight_packets.csv";
  }
  flight_packet_audit_logger_ = std::make_unique<AuditLogger>(
      flight_packet_audit_path,
      "receipt_wall_unix_ns,receipt_steady_ns,session_id,"
      "producer_instance_id,trajectory_epoch,flight_sequence,payload_hash,"
      "transmit_index,transmit_count,sample_count,last_exposure_unix_ns,"
      "source_age_ms,dedupe_result,accepted_for_solve,packet_queue_depth");

  estimator_ = std::make_unique<BatchPhysicsEstimator>(physics_, estimator_config_);
  spin_estimator_ = std::make_unique<SpinEstimator>(spin_estimator_config_);
  predictor_ = std::make_unique<TrajectoryPredictor>(physics_, planner_config_, table_);
  BallPhysics post_net_physics = physics_;
  // This effective future-contact coefficient is identified for the causal
  // pre-bounce one-shot prediction. The estimator keeps the venue coefficient
  // when an actual bounce has already been observed.
  post_net_physics.table_tangential_gain =
      post_net_future_bounce_tangential_gain_;
  post_net_predictor_ = std::make_unique<TrajectoryPredictor>(
      post_net_physics, planner_config_, table_);
  target_planner_ =
      std::make_unique<RacketTargetPlanner>(physics_, planner_config_, table_);
  incoming_trajectory_ = std::make_unique<IncomingTrajectory>(
      incoming_trajectory_config_);

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(input_qos_depth_))
                             .best_effort()
                             .durability_volatile();

  if (publish_flat_command_) {
    flat_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        flat_topic, command_qos);
  }
  if (publish_base_flat_) {
    base_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        base_flat_topic, command_qos);
  }
  if (publish_serve_ball_flat_) {
    serve_ball_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        serve_ball_topic, command_qos);
  }
  if (publish_typed) {
    typed_publisher_ = create_publisher<hope_msgs::msg::RacketCommand>(
        "/racket/command", command_qos);
  }
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/planner/diagnostics", rclcpp::QoS(1));

  ball_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  base_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions ball_options;
  ball_options.callback_group = ball_callback_group_;
  if (flight_packet_input_enabled_) {
    const auto packet_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                .best_effort().durability_volatile();
    flight_packet_subscription_ =
        create_subscription<hope_msgs::msg::BallFlightPacket>(
            flight_packet_topic_, packet_qos,
            std::bind(
                &PlannerNode::flight_packet_callback, this,
                std::placeholders::_1),
            ball_options);
  } else {
    ball_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/poses", input_qos,
        std::bind(&PlannerNode::ball_callback, this, std::placeholders::_1),
        ball_options);
  }

  const std::string base_input_topic =
      declare_parameter<std::string>("base_pose_flat_input_topic", "");
  if (!base_input_topic.empty()) {
    rclcpp::SubscriptionOptions base_options;
    base_options.callback_group = base_callback_group_;
    base_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        base_input_topic, command_qos,
        std::bind(&PlannerNode::base_flat_callback, this, std::placeholders::_1),
        base_options);
  }

  const std::string robot_pose_topic =
      declare_parameter<std::string>("robot_pose_topic", "");
  if (!robot_pose_topic.empty()) {
    rclcpp::SubscriptionOptions base_options;
    base_options.callback_group = base_callback_group_;
    robot_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        robot_pose_topic, input_qos,
        std::bind(&PlannerNode::robot_pose_callback, this, std::placeholders::_1),
        base_options);
  }

  calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "~/freeze_x_hit",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const auto result = calibrate_x_hit();
        response->success = result.first;
        response->message = result.second;
      });
  diagnostics_timer_ = create_wall_timer(1s, [this] { publish_diagnostics(); });
  if (!x_hit_request_file_.empty()) {
    calibration_timer_ = create_wall_timer(100ms, [this] { poll_calibration_request(); });
  }

  health_previous_steady_ns_ = steady_now_ns();
  solver_thread_ = std::thread(&PlannerNode::solver_loop, this);

  RCLCPP_INFO(
      get_logger(),
      "HOPE C++ planner started: estimator=batch_physics_cpp_no_ekf_persistent_bounce "
      "input_depth=%d solve_period=%.3f x_hit=%.3f schema=2 "
      "window=%.3f min_span=%.3f huber=%.4f recency_half_life=%.3f "
      "bounce_reversal=%.5f bounce_excursion=%.4f bounce_confirm=%zu "
      "bounce_sparse=(%.3f,%.3f) bounce_refractory=%.3f "
      "horizon=%.3f adaptive=%d spin_shadow=%d spin_mode=%s debug=%s "
      "post_net_one_shot=%d flight_packet_input=%d packet_topic=%s "
      "net_x=%.3f commit_delay=%.3f "
      "future_bounce_gain=%.3f incoming=(margin=%.3f vin=%.3f vout=%.3f "
      "fit=%zu confirm=%zu preroll=%zu gap=%.3f)",
      input_qos_depth_, solve_period_s_, x_hit_fh_.load(),
      estimator_config_.window_s, estimator_config_.min_span_s,
      estimator_config_.huber_delta_m, estimator_config_.recency_half_life_s,
      estimator_config_.bounce_min_reversal_m,
      estimator_config_.bounce_min_excursion_m,
      estimator_config_.bounce_confirmation_samples,
      estimator_config_.bounce_sparse_confirmation_min_span_s,
      estimator_config_.bounce_sparse_confirmation_excursion_m,
      estimator_config_.bounce_refractory_s,
      planner_config_.max_predict_time_s,
      planner_config_.adaptive_predict_horizon ? 1 : 0,
      spin_shadow_enabled_ ? 1 : 0,
      spin_shadow_mode_name_.c_str(),
      audit_logger_->enabled() ? "on" : "off",
      post_net_one_shot_enabled_ ? 1 : 0,
      flight_packet_input_enabled_ ? 1 : 0,
      flight_packet_topic_.c_str(),
      table_.net_x,
      post_net_commit_delay_s_,
      post_net_future_bounce_tangential_gain_,
      incoming_trajectory_config_.opponent_side_margin_m,
      incoming_trajectory_config_.incoming_speed_threshold_mps,
      incoming_trajectory_config_.outgoing_speed_threshold_mps,
      incoming_trajectory_config_.direction_fit_samples,
      incoming_trajectory_config_.direction_confirmations,
      incoming_trajectory_config_.pre_roll_samples,
      incoming_trajectory_config_.source_gap_reset_s);
  if (require_x_hit_calibration_audit_) {
    RCLCPP_WARN(
        get_logger(),
        "require_x_hit_calibration=true is audit-only in the no-gate C++ path; "
        "the configured x_hit remains executable before and after refresh");
  }
}

PlannerNode::~PlannerNode() {
  stop_requested_.store(true, std::memory_order_release);
  wake_condition_.notify_all();
  if (solver_thread_.joinable()) {
    solver_thread_.join();
  }
  if (audit_logger_) {
    audit_logger_->stop();
  }
}

void PlannerNode::ball_callback(
    const geometry_msgs::msg::PoseArray::SharedPtr message) {
  try {
    ++received_samples_;
    const auto receipt_steady = steady_now_ns();
    const auto previous = last_ball_receipt_steady_ns_.exchange(receipt_steady);
    if (previous > 0 && receipt_steady > previous) {
      const auto gap = receipt_steady - previous;
      auto maximum = maximum_ball_gap_ns_.load();
      while (gap > maximum &&
             !maximum_ball_gap_ns_.compare_exchange_weak(maximum, gap)) {
      }
    }
    if (ball_pose_index_ < 0 ||
        static_cast<std::size_t>(ball_pose_index_) >= message->poses.size()) {
      ++missing_samples_;
      return;
    }
    const auto& pose = message->poses[static_cast<std::size_t>(ball_pose_index_)];
    const auto& position = pose.position;
    BallSample sample;
    const auto source_stamp_ns =
        static_cast<std::int64_t>(message->header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(message->header.stamp.nanosec);
    sample.source_time_ns = source_stamp_ns;
    sample.source_time_s = static_cast<double>(source_stamp_ns) * 1.0e-9;
    sample.position = Vec3(position.x, position.y, position.z);
    Eigen::Quaterniond orientation(
        pose.orientation.w, pose.orientation.x,
        pose.orientation.y, pose.orientation.z);
    const double orientation_norm = orientation.norm();
    sample.orientation_valid = std::isfinite(orientation_norm) &&
        orientation_norm >= 0.5 && orientation_norm <= 1.5 &&
        orientation.coeffs().allFinite();
    if (sample.orientation_valid) {
      sample.orientation = orientation.normalized();
    }
    sample.receipt_steady_ns = receipt_steady;
    sample.receipt_wall_ns = wall_now_ns();
    if (!std::isfinite(sample.source_time_s) || !sample.position.allFinite()) {
      ++missing_samples_;
      return;
    }
    const auto previous_source =
        last_accepted_source_stamp_ns_.load(std::memory_order_acquire);
    if (source_stamp_ns <= previous_source) {
      const auto count =
          out_of_order_samples_.fetch_add(1, std::memory_order_relaxed) + 1;
      if (count == 1 || count % 250 == 0) {
        RCLCPP_WARN(
            get_logger(),
            "ignored duplicate/out-of-order Ball source stamp; latest estimator state "
            "remains executable (count=%llu source=%lld previous=%lld)",
            static_cast<unsigned long long>(count),
            static_cast<long long>(source_stamp_ns),
            static_cast<long long>(previous_source));
      }
      return;
    }
    last_accepted_source_stamp_ns_.store(source_stamp_ns, std::memory_order_release);
    last_source_age_at_callback_ns_.store(
        sample.receipt_wall_ns - source_stamp_ns, std::memory_order_release);
    sample.sequence = ++sample_sequence_;
    ++present_samples_;
    if (post_net_one_shot_enabled_ && incoming_trajectory_) {
      if (trajectory_reset_requested_.exchange(false)) {
        incoming_trajectory_->reset_phase(true);
      }
      const IncomingTrajectoryUpdate update = incoming_trajectory_->observe(sample);
      trajectory_epoch_.store(
          incoming_trajectory_->trajectory_epoch(), std::memory_order_release);
      incoming_phase_.store(
          static_cast<int>(incoming_trajectory_->phase()),
          std::memory_order_release);
      if (update.source_epoch_reset) {
        trajectory_source_resets_.fetch_add(1, std::memory_order_relaxed);
      }
      if (update.snapshot_ready) {
        snapshot_mailbox_.publish(update.snapshot);
        wake_condition_.notify_one();
      }
      return;
    }
    if (!input_ring_.try_push(sample)) {
      ++ring_drops_;
      return;
    }
    wake_condition_.notify_one();
  } catch (...) {
    ++missing_samples_;
  }
}

void PlannerNode::flight_packet_callback(
    const hope_msgs::msg::BallFlightPacket::SharedPtr message) {
  const auto receipt_steady_ns = steady_now_ns();
  const auto receipt_wall_ns = wall_now_ns();
  flight_packets_received_.fetch_add(1, std::memory_order_relaxed);
  const auto write_receive_audit = [this, &message, receipt_steady_ns,
                                    receipt_wall_ns](
      const char* result, bool accepted_for_solve,
      std::size_t queue_depth) noexcept {
    if (!flight_packet_audit_logger_ ||
        !flight_packet_audit_logger_->enabled()) {
      return;
    }
    try {
      const std::int64_t last_exposure_ns = message->samples.empty()
          ? 0
          : message->samples.back().exposure_unix_stamp_ns;
      const double source_age_ms = last_exposure_ns > 0
          ? (receipt_wall_ns - last_exposure_ns) * 1.0e-6
          : std::numeric_limits<double>::quiet_NaN();
      std::ostringstream row;
      row << std::setprecision(17)
          << receipt_wall_ns << ',' << receipt_steady_ns << ','
          << message->session_id << ',' << message->producer_instance_id << ','
          << message->trajectory_epoch << ',' << message->flight_sequence << ','
          << message->payload_hash << ','
          << static_cast<int>(message->transmit_index) << ','
          << static_cast<int>(message->transmit_count) << ','
          << message->samples.size() << ',' << last_exposure_ns << ','
          << source_age_ms << ',' << result << ','
          << (accepted_for_solve ? 1 : 0) << ',' << queue_depth;
      flight_packet_audit_logger_->enqueue(row.str());
    } catch (...) {
      // Receive auditing is deliberately non-authoritative. Never let a log
      // formatting failure alter packet acceptance or Planner output.
    }
  };
  try {
    if (message->schema_version != kBallFlightPacketSchemaVersion ||
        message->payload_hash_algorithm != kBallFlightPacketHashAlgorithm ||
        message->session_id.empty() || message->producer_instance_id.empty() ||
        message->trajectory_epoch == 0 || message->flight_sequence == 0 ||
        message->transmit_count == 0 || message->samples.empty() ||
        message->samples.size() > kMaxEstimatorSamples) {
      flight_packets_invalid_.fetch_add(1, std::memory_order_relaxed);
      write_receive_audit(
          "invalid_structure", false, flight_packet_queue_depth());
      RCLCPP_WARN(
          get_logger(),
          "ignored structurally invalid BallFlightPacket schema=%u samples=%zu "
          "session=%s producer=%s epoch=%llu flight=%llu",
          message->schema_version, message->samples.size(),
          message->session_id.c_str(), message->producer_instance_id.c_str(),
          static_cast<unsigned long long>(message->trajectory_epoch),
          static_cast<unsigned long long>(message->flight_sequence));
      return;
    }

    TrajectorySnapshot snapshot;
    snapshot.sample_count = message->samples.size();
    snapshot.trajectory_epoch = message->trajectory_epoch;
    snapshot.snapshot_sequence = message->flight_sequence;
    snapshot.segment_start_source_time_s =
        ns_to_seconds(message->segment_start_exposure_unix_ns);
    snapshot.previous_segment_last_source_time_s =
        message->previous_segment_last_exposure_unix_ns == 0
        ? std::numeric_limits<double>::quiet_NaN()
        : ns_to_seconds(message->previous_segment_last_exposure_unix_ns);
    snapshot.segment_boundary_reason = message->segment_boundary_reason;
    snapshot.one_shot.commit_due = true;
    snapshot.one_shot.flight_sequence = message->flight_sequence;
    snapshot.one_shot.net_cross_source_time_s =
        ns_to_seconds(message->net_cross_exposure_unix_ns);
    snapshot.one_shot.commit_source_time_s =
        ns_to_seconds(message->commit_exposure_unix_ns);
    snapshot.packet.present = true;
    snapshot.packet.session_id = message->session_id;
    snapshot.packet.producer_instance_id = message->producer_instance_id;
    snapshot.packet.payload_hash = message->payload_hash;
    snapshot.packet.frame_id = message->frame_id;
    snapshot.packet.trajectory_epoch = message->trajectory_epoch;
    snapshot.packet.flight_sequence = message->flight_sequence;
    snapshot.packet.freeze_wall_unix_ns = message->freeze_wall_unix_ns;
    snapshot.packet.publish_wall_unix_ns = message->publish_wall_unix_ns;
    snapshot.packet.receipt_wall_unix_ns = receipt_wall_ns;
    snapshot.packet.receipt_steady_ns = receipt_steady_ns;
    snapshot.packet.transmit_index = message->transmit_index;
    snapshot.packet.transmit_count = message->transmit_count;

    for (std::size_t i = 0; i < message->samples.size(); ++i) {
      const auto& input = message->samples[i];
      BallSample& sample = snapshot.samples[i];
      sample.source_time_ns = input.exposure_unix_stamp_ns;
      sample.source_time_s = ns_to_seconds(input.exposure_unix_stamp_ns);
      sample.position = Vec3(
          input.position.x, input.position.y, input.position.z);
      Eigen::Quaterniond orientation(
          input.orientation.w, input.orientation.x,
          input.orientation.y, input.orientation.z);
      sample.orientation_valid = input.orientation_valid;
      sample.orientation = sample.orientation_valid
          ? orientation
          : Eigen::Quaterniond::Identity();
      sample.receipt_steady_ns = receipt_steady_ns;
      sample.receipt_wall_ns = receipt_wall_ns;
      sample.sequence = ++sample_sequence_;
    }

    std::string validation_reason;
    if (!validate_flight_snapshot(snapshot, validation_reason)) {
      flight_packets_invalid_.fetch_add(1, std::memory_order_relaxed);
      write_receive_audit(
          "invalid_snapshot", false, flight_packet_queue_depth());
      RCLCPP_WARN(
          get_logger(), "ignored invalid BallFlightPacket %s: %s",
          flight_packet_identity_key(snapshot.packet).c_str(),
          validation_reason.c_str());
      return;
    }
    const std::string expected_hash =
        flight_packet_message_payload_hash(*message);
    if (expected_hash != message->payload_hash) {
      flight_packets_invalid_.fetch_add(1, std::memory_order_relaxed);
      write_receive_audit(
          "payload_hash_mismatch", false, flight_packet_queue_depth());
      RCLCPP_WARN(
          get_logger(),
          "ignored BallFlightPacket with payload hash mismatch key=%s got=%s expected=%s",
          flight_packet_identity_key(snapshot.packet).c_str(),
          message->payload_hash.c_str(), expected_hash.c_str());
      return;
    }

    const std::string identity_key = flight_packet_identity_key(snapshot.packet);
    std::size_t accepted_queue_depth = 0;
    {
      std::lock_guard<std::mutex> lock(flight_packet_mutex_);
      const FlightPacketDedupResult result = flight_packet_deduplicator_.observe(
          identity_key, message->payload_hash);
      if (result == FlightPacketDedupResult::kDuplicate) {
        flight_packets_duplicate_.fetch_add(1, std::memory_order_relaxed);
        write_receive_audit("duplicate", false, flight_packet_queue_.size());
        return;
      }
      if (result == FlightPacketDedupResult::kIdentityConflict) {
        flight_packets_conflict_.fetch_add(1, std::memory_order_relaxed);
        write_receive_audit(
            "identity_conflict", false, flight_packet_queue_.size());
        RCLCPP_WARN(
            get_logger(),
            "ignored conflicting BallFlightPacket identity=%s hash=%s",
            identity_key.c_str(), message->payload_hash.c_str());
        return;
      }
      flight_packet_queue_.push_back(std::move(snapshot));
      accepted_queue_depth = flight_packet_queue_.size();
    }

    received_samples_.fetch_add(message->samples.size(), std::memory_order_relaxed);
    present_samples_.fetch_add(message->samples.size(), std::memory_order_relaxed);
    const auto last_source_ns = message->samples.back().exposure_unix_stamp_ns;
    last_accepted_source_stamp_ns_.store(last_source_ns, std::memory_order_release);
    last_source_age_at_callback_ns_.store(
        receipt_wall_ns - last_source_ns, std::memory_order_release);
    last_ball_receipt_steady_ns_.store(receipt_steady_ns, std::memory_order_release);
    trajectory_epoch_.store(message->trajectory_epoch, std::memory_order_release);
    incoming_phase_.store(
        static_cast<int>(IncomingPhase::kWaitOutgoing),
        std::memory_order_release);
    flight_packets_accepted_.fetch_add(1, std::memory_order_relaxed);
    write_receive_audit("accepted", true, accepted_queue_depth);
    wake_condition_.notify_one();
  } catch (const std::exception& exception) {
    flight_packets_invalid_.fetch_add(1, std::memory_order_relaxed);
    write_receive_audit("exception", false, flight_packet_queue_depth());
    RCLCPP_ERROR(
        get_logger(), "BallFlightPacket callback failed: %s", exception.what());
  } catch (...) {
    flight_packets_invalid_.fetch_add(1, std::memory_order_relaxed);
    write_receive_audit("unknown_exception", false, flight_packet_queue_depth());
    RCLCPP_ERROR(get_logger(), "BallFlightPacket callback failed");
  }
}

bool PlannerNode::try_take_flight_packet(
    TrajectorySnapshot& snapshot) noexcept {
  std::lock_guard<std::mutex> lock(flight_packet_mutex_);
  if (flight_packet_queue_.empty()) return false;
  snapshot = std::move(flight_packet_queue_.front());
  flight_packet_queue_.pop_front();
  return true;
}

bool PlannerNode::has_pending_flight_packet() const noexcept {
  std::lock_guard<std::mutex> lock(flight_packet_mutex_);
  return !flight_packet_queue_.empty();
}

std::size_t PlannerNode::flight_packet_queue_depth() const noexcept {
  std::lock_guard<std::mutex> lock(flight_packet_mutex_);
  return flight_packet_queue_.size();
}

void PlannerNode::set_base_snapshot(const BaseSnapshot& snapshot) noexcept {
  std::lock_guard<std::mutex> lock(base_mutex_);
  base_ = snapshot;
}

PlannerNode::BaseSnapshot PlannerNode::base_snapshot() const noexcept {
  std::lock_guard<std::mutex> lock(base_mutex_);
  return base_;
}

void PlannerNode::add_base_sample(
    double base_x, std::int64_t receipt_steady_ns) noexcept {
  if (!std::isfinite(base_x) || receipt_steady_ns <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(calibration_mutex_);
  calibration_samples_.push_back(CalibrationSample{receipt_steady_ns, base_x});
  const auto cutoff = receipt_steady_ns - 2'000'000'000LL;
  while (!calibration_samples_.empty() &&
         calibration_samples_.front().receipt_steady_ns < cutoff) {
    calibration_samples_.pop_front();
  }
}

void PlannerNode::base_flat_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  static std::atomic<std::uint64_t> invalid_count{0};
  try {
    const auto& value = message->data;
    bool valid = value.size() == 16 && value[0] == 2.0 && value[1] == 1.0;
    if (valid) {
      const int flags = static_cast<int>(value[13]);
      valid = static_cast<double>(flags) == value[13] &&
              (flags & kRequiredBaseFlags) == kRequiredBaseFlags &&
              value[14] > 0.0 && value[15] > 0.0 &&
              value[3] > 0.0 && value[4] >= 0.0 && value[4] < 1.0e9;
    }
    Vec3 position = Vec3::Zero();
    if (valid) {
      position = Vec3(value[5], value[6], value[7]);
      const double quaternion_norm = std::sqrt(
          value[8] * value[8] + value[9] * value[9] +
          value[10] * value[10] + value[11] * value[11]);
      valid = position.allFinite() && std::isfinite(quaternion_norm) &&
              quaternion_norm >= 0.5 && quaternion_norm <= 1.5;
    }
    const auto receipt = steady_now_ns();
    BaseSnapshot snapshot;
    snapshot.valid = valid;
    snapshot.position = position;
    snapshot.receipt_steady_ns = receipt;
    if (valid) {
      snapshot.source_stamp_ns =
          static_cast<std::int64_t>(value[3]) * 1'000'000'000LL +
          static_cast<std::int64_t>(value[4]);
      add_base_sample(position.x(), receipt);
    } else if ((++invalid_count % 100U) == 1U) {
      RCLCPP_WARN(
          get_logger(),
          "authoritative schema-2 base packet is invalid; recorded as audit only, "
          "planner release is not blocked");
    }
    set_base_snapshot(snapshot);
  } catch (...) {
    BaseSnapshot snapshot;
    snapshot.receipt_steady_ns = steady_now_ns();
    set_base_snapshot(snapshot);
  }
}

void PlannerNode::robot_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message) {
  try {
    const auto& p = message->pose.position;
    const auto& q_msg = message->pose.orientation;
    Eigen::Quaterniond quaternion(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 0.5 ||
        quaternion.norm() > 1.5) {
      quaternion = Eigen::Quaterniond::Identity();
    } else {
      quaternion.normalize();
    }
    const Vec3 offset(marker_to_base_xyz_[0], marker_to_base_xyz_[1], marker_to_base_xyz_[2]);
    Vec3 position(p.x, p.y, p.z);
    position += quaternion * offset;
    position.z() += policy_z_offset_;
    BaseSnapshot snapshot;
    snapshot.position = position;
    snapshot.valid = position.allFinite();
    snapshot.receipt_steady_ns = steady_now_ns();
    snapshot.source_stamp_ns =
        static_cast<std::int64_t>(message->header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(message->header.stamp.nanosec);
    set_base_snapshot(snapshot);
    if (snapshot.valid) {
      add_base_sample(position.x(), snapshot.receipt_steady_ns);
    }

    if (base_publisher_) {
      std_msgs::msg::Float64MultiArray output;
      output.data = {
          1.0, snapshot.valid ? 1.0 : 0.0,
          snapshot.valid ? position.x() : 0.0,
          snapshot.valid ? position.y() : 0.0,
          snapshot.valid ? position.z() : 0.0,
          quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
      base_publisher_->publish(output);
    }
  } catch (...) {
  }
}

double PlannerNode::active_x_hit() const noexcept {
  if (x_hit_follow_robot_) {
    const BaseSnapshot snapshot = base_snapshot();
    if (snapshot.valid) {
      return std::clamp(snapshot.position.x() + x_hit_offset_, x_hit_min_, x_hit_max_);
    }
  }
  return x_hit_fh_.load(std::memory_order_acquire);
}

double PlannerNode::select_swing_sign(double intercept_y, double base_y) noexcept {
  const double relative_y = intercept_y - base_y;
  const double low = swing_side_split_y_ - swing_side_hysteresis_y_;
  const double high = swing_side_split_y_ + swing_side_hysteresis_y_;
  if (last_swing_sign_ > 0.5) {
    if (relative_y > high) {
      last_swing_sign_ = -1.0;
    }
  } else if (last_swing_sign_ < -0.5) {
    if (relative_y < low) {
      last_swing_sign_ = 1.0;
    }
  } else {
    last_swing_sign_ = relative_y < swing_side_split_y_ ? 1.0 : -1.0;
  }
  return last_swing_sign_;
}

void PlannerNode::solver_loop() noexcept {
  try {
    std::int64_t next_solve_ns = 0;
    BallSample latest_sample;
    bool solve_pending = false;
    std::size_t pending_input_samples = 0;
    PostNetOneShotEvent one_shot_event;
    std::uint64_t pending_snapshot_epoch = 0;
    std::uint64_t pending_snapshot_sequence = 0;
    double pending_segment_start_source_time_s =
        std::numeric_limits<double>::quiet_NaN();
    double pending_previous_segment_last_source_time_s =
        std::numeric_limits<double>::quiet_NaN();
    std::string pending_segment_boundary_reason = "none";
    FlightPacketMetadata pending_flight_packet;
    while (!stop_requested_.load(std::memory_order_acquire)) {
      {
        std::unique_lock<std::mutex> lock(wake_mutex_);
        wake_condition_.wait_for(lock, 5ms, [this] {
          return stop_requested_.load(std::memory_order_acquire) ||
                 input_ring_.size_approx() > 0 ||
                 snapshot_mailbox_.has_pending() ||
                 has_pending_flight_packet();
        });
      }
      if (stop_requested_.load(std::memory_order_acquire)) {
        break;
      }
      if (estimator_reset_requested_.exchange(false)) {
        estimator_->reset();
        spin_estimator_->reset();
      }

      std::size_t consumed_now = 0;
      if (flight_packet_input_enabled_ || post_net_one_shot_enabled_) {
        TrajectorySnapshot snapshot;
        const bool snapshot_available = flight_packet_input_enabled_
            ? try_take_flight_packet(snapshot)
            : snapshot_mailbox_.try_take(snapshot);
        if (snapshot_available &&
            snapshot.sample_count > 0 && snapshot.latest_sample()) {
          estimator_->reset();
          spin_estimator_->reset();
          for (std::size_t i = 0; i < snapshot.sample_count; ++i) {
            estimator_->push(snapshot.samples[i]);
            spin_estimator_->push(snapshot.samples[i]);
          }
          latest_sample = *snapshot.latest_sample();
          consumed_now = snapshot.sample_count;
          pending_input_samples = snapshot.sample_count;
          one_shot_event = snapshot.one_shot;
          solve_pending = true;
          // Snapshot metadata is copied into the per-solve audit below.
          // Only this immutable source-time window participates in the solve.
          one_shot_event.flight_sequence = snapshot.trajectory_epoch;
          pending_snapshot_epoch = snapshot.trajectory_epoch;
          pending_snapshot_sequence = snapshot.snapshot_sequence;
          pending_segment_start_source_time_s =
              snapshot.segment_start_source_time_s;
          pending_previous_segment_last_source_time_s =
              snapshot.previous_segment_last_source_time_s;
          pending_segment_boundary_reason = snapshot.segment_boundary_reason;
          pending_flight_packet = snapshot.packet;
        }
      } else {
        BallSample sample;
        while (input_ring_.try_pop(sample)) {
          latest_sample = sample;
          ++consumed_now;
          ++pending_input_samples;
          estimator_->push(sample);
          spin_estimator_->push(sample);
        }
        if (consumed_now > 0) solve_pending = true;
      }
      if (!solve_pending) {
        continue;
      }
      const auto solve_start_ns = steady_now_ns();
      if (!post_net_one_shot_enabled_ &&
          next_solve_ns > 0 && solve_start_ns < next_solve_ns) {
        continue;
      }
      if (!post_net_one_shot_enabled_) {
        next_solve_ns = solve_start_ns +
            static_cast<std::int64_t>(solve_period_s_ * 1.0e9);
      }

      SolveAudit audit;
      audit.input_samples_consumed = pending_input_samples;
      audit.input_samples_coalesced =
          pending_input_samples > 0 ? pending_input_samples - 1 : 0;
      audit.trajectory_epoch = pending_snapshot_epoch;
      audit.snapshot_sequence = pending_snapshot_sequence;
      audit.segment_start_source_time_s = pending_segment_start_source_time_s;
      audit.previous_segment_last_source_time_s =
          pending_previous_segment_last_source_time_s;
      audit.segment_boundary_reason = pending_segment_boundary_reason;
      audit.flight_packet = pending_flight_packet;
      solver_input_samples_.fetch_add(
          audit.input_samples_consumed, std::memory_order_relaxed);
      solver_coalesced_samples_.fetch_add(
          audit.input_samples_coalesced, std::memory_order_relaxed);
      auto maximum_batch = maximum_solver_batch_samples_.load();
      while (audit.input_samples_consumed > maximum_batch &&
             !maximum_solver_batch_samples_.compare_exchange_weak(
                 maximum_batch, audit.input_samples_consumed)) {
      }
      const auto estimator_start = steady_now_ns();
      const BallState state = estimator_->estimate();
      SpinShadowAudit spin_shadow;
      spin_shadow.enabled = spin_shadow_enabled_;
      spin_shadow.mode = spin_shadow_enabled_ ? spin_shadow_mode_name_ : "disabled";
      spin_shadow.spin = spin_estimator_->estimate();
      if (spin_shadow.spin.valid) {
        ++spin_valid_count_;
      }
      audit.estimator_ms = (steady_now_ns() - estimator_start) * 1.0e-6;

      StrikeTarget strike;
      strike.reason = state.reason;
      double swing_sign = last_swing_sign_;
      const auto stage2_start = steady_now_ns();
      TrajectoryPredictor* const control_predictor =
          post_net_one_shot_enabled_ && post_net_predictor_
          ? post_net_predictor_.get()
          : predictor_.get();
      if (state.valid) {
        // One-shot control uses the causally identified effective coefficient
        // for a future contact; the estimator keeps the venue coefficient for
        // an already observed bounce. Omega=0 remains the control contract.
        // Orientation, spin coherence and Magnus are shadow-only.
        strike = control_predictor->predict_with_spin(
            state, active_x_hit(), Vec3::Zero(),
            SpinPhysicsMode::kVenueGripBounce);
        if (strike.valid) {
          const BaseSnapshot base = base_snapshot();
          swing_sign = select_swing_sign(
              strike.ball_position.y(), base.valid ? base.position.y() : 0.0);
          if (swing_sign < 0.0 && x_hit_bh_delta_ != 0.0) {
            strike = control_predictor->predict_with_spin(
                state, active_x_hit() + x_hit_bh_delta_, Vec3::Zero(),
                SpinPhysicsMode::kVenueGripBounce);
          }
        }
      }
      if (spin_shadow_enabled_ && state.valid) {
        const double shadow_x_hit =
            active_x_hit() + (swing_sign < 0.0 ? x_hit_bh_delta_ : 0.0);
        // Missing/unusable orientation deliberately falls back to zero omega
        // inside the shadow only. It never suppresses or replaces `strike`.
        const Vec3 shadow_omega = spin_shadow.spin.valid
            ? spin_shadow.spin.omega_rad_s
            : Vec3::Zero();
        spin_shadow.strike = control_predictor->predict_with_spin(
            state, shadow_x_hit, shadow_omega, spin_shadow_mode_);
        if (spin_shadow.strike.valid) {
          ++spin_shadow_valid_count_;
        }
      } else {
        spin_shadow.strike.reason = spin_shadow_enabled_
            ? state.reason
            : "spin_shadow_disabled";
      }
      audit.stage2_ms = (steady_now_ns() - stage2_start) * 1.0e-6;

      Vec3 target_land = planner_config_.target_land;
      double flight_time = planner_config_.delta_t_flight;
      if (swing_sign > 0.0) {
        if (std::isfinite(target_land_y_fh_)) target_land.y() = target_land_y_fh_;
        if (std::isfinite(delta_t_flight_fh_)) flight_time = delta_t_flight_fh_;
      } else if (swing_sign < 0.0) {
        if (std::isfinite(target_land_y_bh_)) target_land.y() = target_land_y_bh_;
        if (std::isfinite(delta_t_flight_bh_)) flight_time = delta_t_flight_bh_;
      }

      const auto stage3_start = steady_now_ns();
      const RacketCommand command =
          target_planner_->plan(strike, target_land, flight_time);
      audit.stage3_ms = (steady_now_ns() - stage3_start) * 1.0e-6;
      const auto solve_finish_ns = steady_now_ns();
      audit.total_ms = (solve_finish_ns - solve_start_ns) * 1.0e-6;
      if (audit.total_ms > solve_period_s_ * 1000.0) {
        ++solver_deadline_miss_count_;
      }
      audit.reason = command.valid ? "command_valid" :
          (strike.valid ? command.reason : strike.reason);
      ++solve_count_;
      publish_solve(
          latest_sample, state, strike, command, swing_sign, audit, spin_shadow,
          one_shot_event, solve_finish_ns);
      solve_pending = false;
      pending_input_samples = 0;
      one_shot_event = PostNetOneShotEvent{};
      pending_snapshot_epoch = 0;
      pending_snapshot_sequence = 0;
      pending_segment_start_source_time_s =
          std::numeric_limits<double>::quiet_NaN();
      pending_previous_segment_last_source_time_s =
          std::numeric_limits<double>::quiet_NaN();
      pending_segment_boundary_reason = "none";
      pending_flight_packet = FlightPacketMetadata{};
    }
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "planner solver thread stopped: %s", exception.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "planner solver thread stopped by an unknown exception");
  }
}

void PlannerNode::publish_solve(
    const BallSample& latest_sample,
    const BallState& state,
    const StrikeTarget& strike,
    const RacketCommand& command,
    double swing_sign,
    const SolveAudit& audit,
    const SpinShadowAudit& spin_shadow,
    const PostNetOneShotEvent& one_shot_event,
    std::int64_t solve_finished_steady_ns) noexcept {
  try {
    // With /poses.header.stamp in the camera-exposure Unix epoch, Stage 2's
    // strike_source_time_s is already the true absolute crossing deadline.
    // Subtract the HDU wall time once at publication. This accounts for the
    // complete exposure->Motive->Laptop->HDU path and solver time without a
    // guessed fixed latency constant.
    const auto producer_wall_ns = wall_now_ns();
    const double producer_wall_s =
        static_cast<double>(producer_wall_ns) * 1.0e-9;
    const double strike_deadline_wall_s = command.valid
        ? command.strike_source_time_s
        : 0.0;
    const double time_to_strike_s = command.valid
        ? strike_deadline_wall_s - producer_wall_s
        : 0.0;
    const bool wire_valid = command.valid && std::isfinite(time_to_strike_s) &&
                            time_to_strike_s > 0.0;
    const auto identity = schema2_packer_.next_identity(
        wire_valid, solve_finished_steady_ns);
    const Schema2Packet packet = Schema2Packer::pack(
        wire_valid ? &command : nullptr,
        swing_sign,
        strike_deadline_wall_s,
        policy_z_offset_,
        producer_wall_ns,
        identity,
        state.sample_count,
        state.sample_span_s);
    if (packet.valid) {
      ++valid_count_;
    }

    if (flat_publisher_) {
      std_msgs::msg::Float64MultiArray output;
      output.data.assign(packet.values.begin(), packet.values.end());
      flat_publisher_->publish(output);
    }

    if (typed_publisher_) {
      hope_msgs::msg::RacketCommand output;
      output.header.stamp = get_clock()->now();
      output.header.frame_id = "world";
      if (command.valid) {
        output.position.x = command.position.x();
        output.position.y = command.position.y();
        output.position.z = command.position.z() + policy_z_offset_;
        output.velocity.x = command.velocity.x();
        output.velocity.y = command.velocity.y();
        output.velocity.z = command.velocity.z();
        output.normal.x = command.normal.x();
        output.normal.y = command.normal.y();
        output.normal.z = command.normal.z();
        output.strike_time = command.strike_source_time_s;
        output.time_to_strike = packet.valid ? time_to_strike_s : 0.0;
        output.ball_velocity_outgoing.x = command.outgoing_ball_velocity.x();
        output.ball_velocity_outgoing.y = command.outgoing_ball_velocity.y();
        output.ball_velocity_outgoing.z = command.outgoing_ball_velocity.z();
      }
      output.valid = packet.valid;
      output.clears_net = command.clears_net;
      output.bypasses_net_posts = command.bypasses_net_posts;
      output.predicted_bounces = command.predicted_bounces;
      typed_publisher_->publish(output);
    }

    if (serve_ball_publisher_) {
      std_msgs::msg::Float64MultiArray output;
      const bool valid = state.valid;
      output.data = {
          1.0, valid ? 1.0 : 0.0,
          valid ? state.position.x() : 0.0,
          valid ? state.position.y() : 0.0,
          valid ? state.position.z() + policy_z_offset_ : 0.0,
          valid ? state.velocity.x() : 0.0,
          valid ? state.velocity.y() : 0.0,
          valid ? state.velocity.z() : 0.0,
          valid ? state.source_time_s : 0.0,
          0.0,
          static_cast<double>(state.sample_count)};
      serve_ball_publisher_->publish(output);
    }

    if (audit_logger_ && audit_logger_->enabled()) {
      const BaseSnapshot base = base_snapshot();
      const double base_age_ms = base.receipt_steady_ns > 0
          ? (solve_finished_steady_ns - base.receipt_steady_ns) * 1.0e-6
          : std::numeric_limits<double>::quiet_NaN();
      const double source_age_at_publish_ms =
          (producer_wall_s - latest_sample.source_time_s) * 1.0e3;
      const double source_age_at_callback_ms =
          (static_cast<double>(latest_sample.receipt_wall_ns) * 1.0e-9 -
           latest_sample.source_time_s) * 1.0e3;
      const double callback_to_solve_ms =
          (solve_finished_steady_ns - latest_sample.receipt_steady_ns) * 1.0e-6;
      const double packet_transport_age_ms = audit.flight_packet.present
          ? (audit.flight_packet.receipt_wall_unix_ns * 1.0e-9 -
             latest_sample.source_time_s) * 1.0e3
          : std::numeric_limits<double>::quiet_NaN();
      const double packet_freeze_to_receive_ms = audit.flight_packet.present &&
              audit.flight_packet.freeze_wall_unix_ns > 0
          ? (audit.flight_packet.receipt_wall_unix_ns -
             audit.flight_packet.freeze_wall_unix_ns) * 1.0e-6
          : std::numeric_limits<double>::quiet_NaN();
      std::ostringstream row;
      row << std::setprecision(17)
          << session_id_ << ',' << latest_sample.sequence << ','
          << solve_count_.load() << ',' << identity.command_sequence << ','
          << (packet.valid ? identity.flight_id : 0) << ','
          << (packet.valid ? identity.revision_id : 0) << ','
          << latest_sample.source_time_s << ',' << source_age_at_publish_ms << ','
          << strike_deadline_wall_s << ',' << producer_wall_s << ','
          << latest_sample.receipt_steady_ns << ','
          << solve_finished_steady_ns << ',' << (packet.valid ? 1 : 0) << ','
          << audit.reason << ','
          << latest_sample.position.x() << ',' << latest_sample.position.y() << ','
          << latest_sample.position.z() << ',' << state.sample_count << ','
          << state.sample_span_s << ','
          << state.position.x() << ',' << state.position.y() << ',' << state.position.z() << ','
          << state.velocity.x() << ',' << state.velocity.y() << ',' << state.velocity.z() << ','
          << state.residual_rms_m << ',' << state.residual_max_m << ','
          << strike.ball_position.x() << ',' << strike.ball_position.y() << ','
          << strike.ball_position.z() << ',' << strike.ball_velocity.x() << ','
          << strike.ball_velocity.y() << ',' << strike.ball_velocity.z() << ','
          << command.velocity.x() << ',' << command.velocity.y() << ','
          << command.velocity.z() << ',' << swing_sign << ','
          << (packet.valid ? time_to_strike_s : 0.0) << ','
          << audit.estimator_ms << ',' << audit.stage2_ms << ',' << audit.stage3_ms << ','
          << audit.total_ms << ','
          << (post_net_one_shot_enabled_ ? 0 : input_ring_.size_approx()) << ','
          << ring_drops_.load() << ',' << audit_logger_->queue_depth() << ','
          << audit_logger_->dropped_rows() << ',' << (base.valid ? 1 : 0) << ','
          << base.position.x() << ',' << base.position.y() << ',' << base.position.z() << ','
          << base_age_ms << ",batch_physics_cpp_no_ekf_persistent_bounce,"
          << (spin_shadow.enabled ? 1 : 0) << ',' << spin_shadow.mode << ','
          << (spin_shadow.spin.valid ? 1 : 0) << ',' << spin_shadow.spin.reason << ','
          << spin_shadow.spin.omega_rad_s.x() << ','
          << spin_shadow.spin.omega_rad_s.y() << ','
          << spin_shadow.spin.omega_rad_s.z() << ','
          << spin_shadow.spin.omega_rad_s.norm() /
                 (2.0 * 3.14159265358979323846) << ','
          << spin_shadow.spin.sample_span_s << ','
          << spin_shadow.spin.retained_time_fraction << ','
          << spin_shadow.spin.coherence << ','
          << spin_shadow.spin.retained_increments << ','
          << spin_shadow.spin.rejected_increments << ','
          << (spin_shadow.strike.valid ? 1 : 0) << ','
          << spin_shadow.strike.reason << ','
          << spin_shadow.strike.ball_position.x() << ','
          << spin_shadow.strike.ball_position.y() << ','
          << spin_shadow.strike.ball_position.z() << ','
          << spin_shadow.strike.ball_velocity.x() << ','
          << spin_shadow.strike.ball_velocity.y() << ','
          << spin_shadow.strike.ball_velocity.z() << ','
          << spin_shadow.strike.strike_source_time_s << ','
          << spin_shadow.strike.predicted_bounces << ','
          << (spin_shadow.strike.ball_position.y() - strike.ball_position.y()) << ','
          << (spin_shadow.strike.ball_position.z() - strike.ball_position.z()) << ','
          << (spin_shadow.strike.strike_source_time_s - strike.strike_source_time_s) << ','
          << (state.bounce_transition_used ? 1 : 0) << ','
          << state.bounce_source_time_s << ',' << state.pre_bounce_samples << ','
          << state.post_bounce_samples << ','
          << (post_net_one_shot_enabled_
                  ? "post_net_one_shot_effective_bounce_zero_spin"
                  : "venue_grip_zero_spin")
          << ','
          << source_age_at_callback_ms << ',' << callback_to_solve_ms << ','
          << audit.input_samples_consumed << ',' << audit.input_samples_coalesced << ','
          << solver_input_samples_.load() << ','
          << solver_coalesced_samples_.load() << ','
          << out_of_order_samples_.load() << ',' << input_qos_depth_ << ','
          << (state.bounce_epoch_active ? 1 : 0) << ','
          << (post_net_one_shot_enabled_ ? 1 : 0) << ','
          << one_shot_event.flight_sequence << ','
          << one_shot_event.net_cross_source_time_s << ','
          << one_shot_event.commit_source_time_s << ','
          << post_net_commit_delay_s_ << ','
          << post_net_future_bounce_tangential_gain_ << ','
          << audit.trajectory_epoch << ',' << audit.snapshot_sequence << ','
          << audit.segment_boundary_reason << ','
          << audit.segment_start_source_time_s << ','
          << audit.previous_segment_last_source_time_s << ','
          << (std::isfinite(audit.previous_segment_last_source_time_s) &&
                      std::isfinite(one_shot_event.commit_source_time_s)
                  ? (one_shot_event.commit_source_time_s -
                     audit.previous_segment_last_source_time_s) * 1.0e3
                  : std::numeric_limits<double>::quiet_NaN())
          << ',' << snapshot_mailbox_.published() << ','
          << snapshot_mailbox_.consumed() << ','
          << snapshot_mailbox_.superseded() << ','
          << (audit.flight_packet.present ? 1 : 0) << ','
          << audit.flight_packet.session_id << ','
          << audit.flight_packet.producer_instance_id << ','
          << audit.flight_packet.payload_hash << ','
          << static_cast<int>(audit.flight_packet.transmit_index) << ','
          << static_cast<int>(audit.flight_packet.transmit_count) << ','
          << packet_transport_age_ms << ',' << packet_freeze_to_receive_ms << ','
          << flight_packets_received_.load() << ','
          << flight_packets_accepted_.load() << ','
          << flight_packets_duplicate_.load() << ','
          << flight_packets_conflict_.load() << ','
          << flight_packets_invalid_.load() << ','
          << flight_packet_queue_depth();
      audit_logger_->enqueue(row.str());
    }
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "failed to publish one planner solve");
  }
}

void PlannerNode::publish_diagnostics() noexcept {
  try {
    const auto now = steady_now_ns();
    const double elapsed = std::max(
        1.0e-6, (now - health_previous_steady_ns_) * 1.0e-9);
    const auto received = received_samples_.load();
    const auto solves = solve_count_.load();
    const double receive_rate = (received - health_previous_received_) / elapsed;
    const double solve_rate = (solves - health_previous_solves_) / elapsed;
    const double nominal_retention = receive_rate / expected_mocap_hz_;
    health_previous_received_ = received;
    health_previous_solves_ = solves;
    health_previous_steady_ns_ = now;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "hope_planner_cpp/realtime";
    status.hardware_id = "hdu";
    status.level = ring_drops_.load() == 0 && snapshot_mailbox_.superseded() == 0
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
        "batch_physics_cpp_no_ekf_persistent_bounce; diagnostics are audit-only";
    status.values.push_back(diagnostic_value("receive_hz", number_string(receive_rate, 2)));
    status.values.push_back(diagnostic_value("solve_hz", number_string(solve_rate, 2)));
    status.values.push_back(diagnostic_value(
        "nominal_callback_retention", number_string(nominal_retention, 4)));
    status.values.push_back(diagnostic_value("received", std::to_string(received)));
    status.values.push_back(diagnostic_value("present", std::to_string(present_samples_.load())));
    status.values.push_back(diagnostic_value("missing", std::to_string(missing_samples_.load())));
    status.values.push_back(diagnostic_value("ring_depth", std::to_string(input_ring_.size_approx())));
    status.values.push_back(diagnostic_value("ring_drops", std::to_string(ring_drops_.load())));
    status.values.push_back(diagnostic_value(
        "source_out_of_order", std::to_string(out_of_order_samples_.load())));
    status.values.push_back(diagnostic_value(
        "source_age_at_callback_ms",
        number_string(last_source_age_at_callback_ns_.load() * 1.0e-6, 3)));
    status.values.push_back(diagnostic_value(
        "solver_input_samples", std::to_string(solver_input_samples_.load())));
    status.values.push_back(diagnostic_value(
        "solver_coalesced_samples", std::to_string(solver_coalesced_samples_.load())));
    status.values.push_back(diagnostic_value(
        "maximum_solver_batch_samples",
        std::to_string(maximum_solver_batch_samples_.load())));
    status.values.push_back(diagnostic_value(
        "input_qos_depth", std::to_string(input_qos_depth_)));
    status.values.push_back(diagnostic_value(
        "input_mode", flight_packet_input_enabled_ ? "flight_packet" : "poses"));
    status.values.push_back(diagnostic_value(
        "flight_packet_topic", flight_packet_topic_));
    status.values.push_back(diagnostic_value(
        "flight_packet_queue_depth", std::to_string(flight_packet_queue_depth())));
    status.values.push_back(diagnostic_value(
        "flight_packets_received", std::to_string(flight_packets_received_.load())));
    status.values.push_back(diagnostic_value(
        "flight_packets_accepted", std::to_string(flight_packets_accepted_.load())));
    status.values.push_back(diagnostic_value(
        "flight_packets_duplicate", std::to_string(flight_packets_duplicate_.load())));
    status.values.push_back(diagnostic_value(
        "flight_packets_conflict", std::to_string(flight_packets_conflict_.load())));
    status.values.push_back(diagnostic_value(
        "flight_packets_invalid", std::to_string(flight_packets_invalid_.load())));
    status.values.push_back(diagnostic_value(
        "post_net_one_shot", post_net_one_shot_enabled_ ? "1" : "0"));
    status.values.push_back(diagnostic_value(
        "post_net_commit_delay_s", number_string(post_net_commit_delay_s_, 3)));
    status.values.push_back(diagnostic_value(
        "post_net_future_bounce_tangential_gain",
        number_string(post_net_future_bounce_tangential_gain_, 3)));
    status.values.push_back(diagnostic_value(
        "one_shot_flight_sequence",
        std::to_string(trajectory_epoch_.load())));
    status.values.push_back(diagnostic_value(
        "incoming_phase",
        incoming_phase_name(static_cast<IncomingPhase>(incoming_phase_.load()))));
    status.values.push_back(diagnostic_value(
        "trajectory_epoch", std::to_string(trajectory_epoch_.load())));
    status.values.push_back(diagnostic_value(
        "trajectory_source_resets",
        std::to_string(trajectory_source_resets_.load())));
    status.values.push_back(diagnostic_value(
        "snapshot_pending", snapshot_mailbox_.has_pending() ? "1" : "0"));
    status.values.push_back(diagnostic_value(
        "snapshot_published", std::to_string(snapshot_mailbox_.published())));
    status.values.push_back(diagnostic_value(
        "snapshot_consumed", std::to_string(snapshot_mailbox_.consumed())));
    status.values.push_back(diagnostic_value(
        "snapshot_superseded", std::to_string(snapshot_mailbox_.superseded())));
    status.values.push_back(diagnostic_value("valid_commands", std::to_string(valid_count_.load())));
    status.values.push_back(diagnostic_value(
        "spin_shadow_enabled", spin_shadow_enabled_ ? "1" : "0"));
    status.values.push_back(diagnostic_value("spin_shadow_mode", spin_shadow_mode_name_));
    status.values.push_back(diagnostic_value(
        "spin_valid_solves", std::to_string(spin_valid_count_.load())));
    status.values.push_back(diagnostic_value(
        "spin_shadow_valid_solves", std::to_string(spin_shadow_valid_count_.load())));
    status.values.push_back(diagnostic_value(
        "solver_deadline_misses", std::to_string(solver_deadline_miss_count_.load())));
    status.values.push_back(diagnostic_value(
        "ball_max_gap_ms", number_string(maximum_ball_gap_ns_.load() * 1.0e-6, 3)));
    status.values.push_back(diagnostic_value(
        "logger_drops", std::to_string(audit_logger_ ? audit_logger_->dropped_rows() : 0)));
    status.values.push_back(diagnostic_value(
        "x_hit_calibrated_audit", x_hit_calibrated_.load() ? "1" : "0"));
    status.values.push_back(diagnostic_value(
        "estimator_kind", "batch_physics_cpp_no_ekf_persistent_bounce"));
    status.values.push_back(diagnostic_value(
        "control_stage2_mode", "post_net_one_shot_effective_bounce_zero_spin"));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);

    RCLCPP_INFO(
        get_logger(),
        "planner health input=%.1fHz nominal-retention=%.1f%% solve=%.1fHz "
        "ring=%zu drops=%llu coalesced=%llu ooo=%llu source_age=%.2fms "
        "valid=%llu deadline_miss=%llu max_gap=%.2fms logger_drops=%llu "
        "trajectory=%llu phase=%s snapshots=%llu/%llu superseded=%llu "
        "packets=%llu/%llu duplicate=%llu conflict=%llu invalid=%llu queue=%zu",
        receive_rate, 100.0 * nominal_retention, solve_rate,
        input_ring_.size_approx(),
        static_cast<unsigned long long>(ring_drops_.load()),
        static_cast<unsigned long long>(solver_coalesced_samples_.load()),
        static_cast<unsigned long long>(out_of_order_samples_.load()),
        last_source_age_at_callback_ns_.load() * 1.0e-6,
        static_cast<unsigned long long>(valid_count_.load()),
        static_cast<unsigned long long>(solver_deadline_miss_count_.load()),
        maximum_ball_gap_ns_.load() * 1.0e-6,
        static_cast<unsigned long long>(
            audit_logger_ ? audit_logger_->dropped_rows() : 0),
        static_cast<unsigned long long>(trajectory_epoch_.load()),
        incoming_phase_name(static_cast<IncomingPhase>(incoming_phase_.load())),
        static_cast<unsigned long long>(snapshot_mailbox_.consumed()),
        static_cast<unsigned long long>(snapshot_mailbox_.published()),
        static_cast<unsigned long long>(snapshot_mailbox_.superseded()),
        static_cast<unsigned long long>(flight_packets_accepted_.load()),
        static_cast<unsigned long long>(flight_packets_received_.load()),
        static_cast<unsigned long long>(flight_packets_duplicate_.load()),
        static_cast<unsigned long long>(flight_packets_conflict_.load()),
        static_cast<unsigned long long>(flight_packets_invalid_.load()),
        flight_packet_queue_depth());
  } catch (...) {
  }
}

std::pair<bool, std::string> PlannerNode::calibrate_x_hit() noexcept {
  try {
    if (x_hit_follow_robot_) {
      return {false, "x_hit_follow_robot=true; fixed-plane refresh is not applicable"};
    }
    const auto now = steady_now_ns();
    const auto window_ns = static_cast<std::int64_t>(x_hit_calibration_window_s_ * 1.0e9);
    std::vector<double> values;
    std::int64_t newest = 0;
    {
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      for (const auto& sample : calibration_samples_) {
        if (sample.receipt_steady_ns >= now - window_ns) {
          values.push_back(sample.base_x);
          newest = std::max(newest, sample.receipt_steady_ns);
        }
      }
    }
    if (static_cast<int>(values.size()) < x_hit_calibration_min_samples_) {
      return {false, "NOT CALIBRATED: insufficient recent base samples"};
    }
    const double newest_age = (now - newest) * 1.0e-9;
    if (newest <= 0 || newest_age > x_hit_calibration_max_age_s_) {
      return {false, "NOT CALIBRATED: newest base sample is too old"};
    }
    const auto minimum_maximum = std::minmax_element(values.begin(), values.end());
    const double span = *minimum_maximum.second - *minimum_maximum.first;
    if (span > x_hit_calibration_max_span_m_) {
      return {false, "NOT CALIBRATED: base-X span exceeds calibration tolerance"};
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    const double base_x = *middle;
    const double x_hit = base_x + x_hit_calibration_offset_;
    if (!std::isfinite(x_hit)) {
      return {false, "NOT CALIBRATED: derived x_hit is non-finite"};
    }
    x_hit_fh_.store(x_hit, std::memory_order_release);
    x_hit_calibrated_.store(true, std::memory_order_release);
    estimator_reset_requested_.store(true, std::memory_order_release);
    trajectory_reset_requested_.store(true, std::memory_order_release);
    std::ostringstream message;
    message << std::fixed << std::setprecision(4)
            << "CALIBRATED audit refresh base_x=" << base_x
            << " + offset=" << x_hit_calibration_offset_
            << " -> x_hit=" << x_hit
            << "; samples=" << values.size()
            << " span=" << span
            << " newest_age=" << newest_age
            << "; refresh status does not block planner output";
    return {true, message.str()};
  } catch (const std::exception& exception) {
    return {false, std::string("NOT CALIBRATED: ") + exception.what()};
  } catch (...) {
    return {false, "NOT CALIBRATED: unknown error"};
  }
}

void PlannerNode::poll_calibration_request() noexcept {
  try {
    const std::filesystem::path request_path(x_hit_request_file_);
    if (!std::filesystem::exists(request_path)) {
      return;
    }
    std::ifstream request_stream(request_path);
    std::string request_id;
    std::getline(request_stream, request_id);
    request_stream.close();
    std::error_code error;
    std::filesystem::remove(request_path, error);
    if (request_id.empty()) request_id = "unknown";
    const auto result = calibrate_x_hit();
    if (result.first) {
      RCLCPP_INFO(get_logger(), "%s", result.second.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", result.second.c_str());
    }
    if (x_hit_status_file_.empty()) {
      return;
    }
    const std::filesystem::path status_path(x_hit_status_file_);
    const std::filesystem::path temporary = status_path.string() + ".tmp";
    if (status_path.has_parent_path()) {
      std::filesystem::create_directories(status_path.parent_path());
    }
    {
      std::ofstream status_stream(temporary, std::ios::out | std::ios::trunc);
      status_stream << "request=" << request_id << '\n'
                    << "success=" << (result.first ? 1 : 0) << '\n'
                    << "message=" << result.second << '\n';
    }
    std::filesystem::rename(temporary, status_path, error);
    if (error) {
      RCLCPP_WARN(get_logger(), "could not replace x_hit status file: %s", error.message().c_str());
    }
  } catch (const std::exception& exception) {
    RCLCPP_WARN(get_logger(), "x_hit request polling failed: %s", exception.what());
  }
}

}  // namespace hope_planner_cpp
