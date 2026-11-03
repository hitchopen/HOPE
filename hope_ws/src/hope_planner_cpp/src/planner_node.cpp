#include "hope_planner_cpp/planner_node.hpp"
#include "hope_planner_cpp/v2_managed_revocation.hpp"
#include "hope_planner_cpp/future_contact_parameters.hpp"
#include "hope_planner_cpp/hit_speed_scale_state.hpp"

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

std::string scale_string(double value) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
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
  const auto backend = declare_parameter<std::string>("prediction_backend", "legacy");
  if (backend == "v2") {
    integrated_runtime_ = std::make_unique<IntegratedPlannerRuntime>(*this);
    return;
  }
  incoming_backend_ = backend == "v2_incoming";
  if (backend != "legacy" && !incoming_backend_) throw std::invalid_argument("unknown prediction_backend");
  declare_parameter<std::string>("ball_rigid_body_name", "pingpong_ball");
  ball_pose_index_ = static_cast<int>(declare_parameter<int>("ball_pose_index", 0));
  input_qos_depth_ = std::clamp(
      static_cast<int>(declare_parameter<int>("input_qos_depth", 64)), 1, 1024);
  solve_period_s_ = std::max(0.001, declare_parameter<double>("solve_period_s", 0.033));
  expected_mocap_hz_ = std::max(1.0, declare_parameter<double>("expected_mocap_hz", 150.0));

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
  x_hit_freeze_session_home_ =
      declare_parameter<bool>("x_hit_freeze_session_home", false);
  x_hit_request_file_ =
      declare_parameter<std::string>("x_hit_calibration_request_file", "");
  x_hit_status_file_ =
      declare_parameter<std::string>("x_hit_calibration_status_file", "");

  swing_side_split_y_ = declare_parameter<double>("swing_side_split_y", -0.25);
  swing_side_hysteresis_y_ =
      std::max(0.0, declare_parameter<double>("swing_side_hysteresis_y", 0.04));
  swing_side_reference_mode_ = declare_parameter<std::string>(
      "swing_side_reference_mode", "live_base_v1");
  if (swing_side_reference_mode_ != "live_base_v1" &&
      swing_side_reference_mode_ != "session_home_v1") {
    throw std::invalid_argument(
        "swing_side_reference_mode must be live_base_v1 or session_home_v1");
  }
  if (x_hit_freeze_session_home_ &&
      swing_side_reference_mode_ != "session_home_v1") {
    throw std::invalid_argument(
        "x_hit_freeze_session_home requires "
        "swing_side_reference_mode=session_home_v1");
  }
  target_land_y_fh_ = declare_parameter<double>(
      "target_land_y_fh", std::numeric_limits<double>::quiet_NaN());
  target_land_y_bh_ = declare_parameter<double>(
      "target_land_y_bh", std::numeric_limits<double>::quiet_NaN());
  delta_t_flight_fh_ = declare_parameter<double>(
      "delta_t_flight_fh", std::numeric_limits<double>::quiet_NaN());
  delta_t_flight_bh_ = declare_parameter<double>(
      "delta_t_flight_bh", std::numeric_limits<double>::quiet_NaN());

  physics_.drag_k = declare_parameter<double>("drag_k", BallPhysics{}.drag_k);
  physics_.magnus_k = declare_parameter<double>("magnus_k", BallPhysics{}.magnus_k);
  physics_.restitution_h = declare_parameter<double>("restitution_h", BallPhysics{}.restitution_h);
  physics_.restitution_v = declare_parameter<double>("restitution_v", BallPhysics{}.restitution_v);
  physics_.nakashima_friction_mu =
      declare_parameter<double>("nakashima_friction_mu", 0.25);
  physics_.table_tangential_gain =
      declare_parameter<double>("table_tangential_gain", BallPhysics{}.table_tangential_gain);
  physics_.table_friction_cap_mu =
      declare_parameter<double>("table_friction_cap_mu", BallPhysics{}.table_friction_cap_mu);
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
          "post_net_future_bounce_tangential_gain", physics_.table_tangential_gain));
  post_net_future_bounce_restitution_v_ = declare_parameter<double>(
      "post_net_future_bounce_restitution_v", physics_.restitution_v);
  if (!std::isfinite(post_net_future_bounce_restitution_v_) ||
      post_net_future_bounce_restitution_v_ < 0.0 ||
      post_net_future_bounce_restitution_v_ > 1.0) {
    throw std::invalid_argument("post_net_future_bounce_restitution_v must be in [0, 1]");
  }

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
  estimator_config_.max_sample_gap_s = declare_parameter<double>("batch_max_sample_gap_s", 0.0);
  estimator_config_.single_flight_only = declare_parameter<bool>("batch_single_flight_only", false);
  incoming_trajectory_config_.estimator_max_sample_gap_s = estimator_config_.max_sample_gap_s;
  estimator_config_.min_span_s =
      declare_parameter<double>("batch_estimator_min_span_s", 0.08);
  estimator_config_.post_bounce_min_span_s =
      declare_parameter<double>("batch_post_bounce_min_span_s", 0.0);
  estimator_config_.post_bounce_exclusion_s =
      declare_parameter<double>("batch_post_bounce_exclusion_s", 0.008);
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
  spin_estimator_config_ = declare_spin_estimator_config(*this);
  future_contact_config_ = declare_future_contact_config(*this);
  estimator_config_.spin_enabled = future_contact_config_.magnus_enabled;
  estimator_config_.spin_min_coherence = future_contact_config_.min_coherence;
  declare_parameter<double>("spin_gate_rev_s", 3.0);
  declare_parameter<bool>("publish_strike_spec", false);
  declare_parameter<double>("racket_speed_budget", 10.0);
  if (requested_kalman) {
    RCLCPP_WARN(
        get_logger(),
        "use_kalman=true was supplied by a legacy command but is ignored: "
        "this executable contains only batch_physics_cpp_no_ekf");
  }

  policy_z_offset_ = declare_parameter<double>("policy_z_offset", 0.76);
  max_target_turn_deg_ = declare_parameter<double>("max_target_turn_deg", 40.0);
  if (!std::isfinite(max_target_turn_deg_) || max_target_turn_deg_ <= 0.0 ||
      max_target_turn_deg_ > 180.0) {
    throw std::invalid_argument("max_target_turn_deg must be finite and in (0, 180]");
  }
  target_turn_max_pose_age_s_ = declare_parameter<double>("target_turn_max_pose_age_s", 0.2);
  target_turn_reject_stale_pose_ = declare_parameter<bool>("target_turn_reject_stale_pose", true);
  if (!std::isfinite(target_turn_max_pose_age_s_) || target_turn_max_pose_age_s_ <= 0.0) {
    throw std::invalid_argument("target_turn_max_pose_age_s must be finite and positive");
  }
  publish_flat_command_ = declare_parameter<bool>("publish_flat_cmd", true);
  publish_base_flat_ = declare_parameter<bool>("publish_base_flat", false);
  publish_serve_ball_flat_ = declare_parameter<bool>("publish_serve_ball_flat", !incoming_backend_);
  racket_flat_schema_ = static_cast<int>(
      declare_parameter<int>("racket_flat_schema", 2));
  if (racket_flat_schema_ != 2 && racket_flat_schema_ != 3 && racket_flat_schema_ != 4) {
    throw std::invalid_argument("racket_flat_schema must be 2, 3, or 4");
  }
  const auto home_topic = declare_parameter<std::string>("session_home_input_topic", "");
  session_home_max_age_s_ = declare_parameter<double>("session_home_max_age_s", 0.5);
  if ((racket_flat_schema_ == 4 && home_topic.empty()) ||
      !std::isfinite(session_home_max_age_s_) || session_home_max_age_s_ <= 0.0 ||
      (racket_flat_schema_ == 4 && (x_hit_follow_robot_ || x_hit_freeze_session_home_ ||
          x_hit_bh_delta_ != 0.0)))
    throw std::invalid_argument("schema4 requires Runner HOME and a fixed station plane");
  if (racket_flat_schema_ == 4) {
    home_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        home_topic, rclcpp::QoS(10).reliable().durability_volatile(),
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          const auto update = [this, &msg] {
            std::lock_guard<std::mutex> lock(base_mutex_);
            shared_home_.accept(msg->data, steady_now_ns());
            return std::make_pair(shared_home_.value.epoch, base_.valid &&
                (!incoming_runtime_ || incoming_runtime_->reference_matches(base_.calibration_id, base_.world_id)));
          };
          if (incoming_runtime_) incoming_runtime_->update_reference(update);
          else update();
        });
  }
  reach_permission_config_.enabled =
      declare_parameter<bool>("reach_permission_enabled", false);
  reach_permission_config_.contract = declare_parameter<std::string>(
      "reach_permission_contract", kDisabledReachPermissionContract);
  const std::string screened_target_tuple_contract =
      declare_parameter<std::string>(
          "screened_target_tuple_contract", kScreenedTargetTupleContract);
  const std::string target_tuple_frame_contract =
      declare_parameter<std::string>(
          "target_tuple_frame_contract", kTargetTupleFrameContract);
  const double target_tuple_relative_x_sensor_tolerance_m =
      declare_parameter<double>(
          "target_tuple_relative_x_sensor_tolerance_m",
          kSessionHomeRelativeXSensorToleranceM);
  reach_permission_config_.minimum_tts_s = declare_parameter<double>(
      "reach_permission_min_tts_s", 0.60);
  reach_permission_config_.fh_level1_abs_y_m = declare_parameter<double>(
      "reach_permission_fh_level1_abs_y_m", 0.275);
  reach_permission_config_.fh_level2_abs_y_m = declare_parameter<double>(
      "reach_permission_fh_level2_abs_y_m", 0.300);
  reach_permission_config_.bh_level1_abs_y_m = declare_parameter<double>(
      "reach_permission_bh_level1_abs_y_m", 0.090);
  reach_permission_config_.bh_level2_abs_y_m = declare_parameter<double>(
      "reach_permission_bh_level2_abs_y_m", 0.120);
  reach_permission_config_.fh_min_relative_y_m = declare_parameter<double>(
      "reach_permission_fh_min_relative_y_m", -0.350);
  reach_permission_config_.fh_max_relative_y_m = declare_parameter<double>(
      "reach_permission_fh_max_relative_y_m", -0.250);
  reach_permission_config_.bh_min_relative_y_m = declare_parameter<double>(
      "reach_permission_bh_min_relative_y_m", -0.140);
  reach_permission_config_.bh_max_relative_y_m = declare_parameter<double>(
      "reach_permission_bh_max_relative_y_m", 0.180);
  validate_reach_permission_config(reach_permission_config_);
  if (reach_permission_config_.enabled &&
      (screened_target_tuple_contract != kScreenedTargetTupleContract ||
       target_tuple_frame_contract != kTargetTupleFrameContract ||
       target_tuple_relative_x_sensor_tolerance_m !=
           kSessionHomeRelativeXSensorToleranceM)) {
    throw std::invalid_argument(
        "enabled reach permission requires the exact screened target tuple "
        "and immutable HOME-XY frame contracts");
  }
  if (reach_permission_config_.enabled && racket_flat_schema_ != 3) {
    throw std::invalid_argument(
        "enabled reach permission requires racket_flat_schema=3");
  }
  if (reach_permission_config_.enabled &&
      swing_side_reference_mode_ != "session_home_v1") {
    throw std::invalid_argument(
        "enabled reach permission requires swing_side_reference_mode=session_home_v1");
  }
  if (reach_permission_config_.enabled &&
      (swing_side_split_y_ != -0.25 || swing_side_hysteresis_y_ != 0.0)) {
    throw std::invalid_argument(
        "enabled reach permission requires exact immutable-HOME "
        "side split=-0.25 and hysteresis=0.0");
  }
  const std::string question_fixture_csv = declare_parameter<std::string>(
      "question_fixture_csv", "");
  const bool question_fixture_simulator_only = declare_parameter<bool>(
      "question_fixture_simulator_only", false);
  const std::string question_fixture_contract = declare_parameter<std::string>(
      "question_fixture_contract", "");
  const std::string question_fixture_bank_sha256 = declare_parameter<std::string>(
      "question_fixture_bank_sha256", "");
  const std::string question_fixture_receipt_sha256 =
      declare_parameter<std::string>("question_fixture_receipt_sha256", "");
  const int question_fixture_expected_flights = static_cast<int>(
      declare_parameter<int>("question_fixture_expected_flights", 0));
  question_fixture_max_position_error_m_ = declare_parameter<double>(
      "question_fixture_max_position_error_m", 0.03);
  question_fixture_max_velocity_error_mps_ = declare_parameter<double>(
      "question_fixture_max_velocity_error_mps", 0.10);
  question_fixture_max_tts_error_s_ = declare_parameter<double>(
      "question_fixture_max_tts_error_s", 0.03);
  question_fixture_enabled_ = !question_fixture_csv.empty();
  if (question_fixture_enabled_) {
    if (!question_fixture_simulator_only || !flight_packet_input_enabled_ ||
        !post_net_one_shot_enabled_ || racket_flat_schema_ != 3 ||
        reach_permission_config_.enabled || !x_hit_freeze_session_home_ ||
        swing_side_reference_mode_ != "session_home_v1" ||
        swing_side_split_y_ != -0.25 || swing_side_hysteresis_y_ != 0.0 ||
        x_hit_fh_.load() != 0.08 || x_hit_bh_delta_ != 0.0 ||
        question_fixture_contract != kSchema31V5FixtureContract ||
        question_fixture_bank_sha256 != kSchema31V5BankSha256 ||
        question_fixture_receipt_sha256 != kSchema31V5ReceiptSha256 ||
        question_fixture_expected_flights != 26) {
      throw std::invalid_argument(
          "Schema31 v5 question fixture requires the exact simulator-only, "
          "26-flight, L0, immutable-HOME Schema3 contract");
    }
    if (!(question_fixture_max_position_error_m_ > 0.0 &&
          question_fixture_max_position_error_m_ <= 0.03 &&
          question_fixture_max_velocity_error_mps_ > 0.0 &&
          question_fixture_max_velocity_error_mps_ <= 0.10 &&
          question_fixture_max_tts_error_s_ > 0.0 &&
          question_fixture_max_tts_error_s_ <= 0.03)) {
      throw std::invalid_argument(
          "Schema31 v5 physical residual tolerances may not exceed "
          "0.03 m / 0.10 mps / 0.03 s");
    }
    question_fixture_ = std::make_unique<QuestionBankFixture>(
        QuestionBankFixture::load_csv(
            question_fixture_csv, question_fixture_contract,
            question_fixture_bank_sha256, question_fixture_receipt_sha256,
            static_cast<std::size_t>(question_fixture_expected_flights)));
  }
  const std::string flat_topic =
      declare_parameter<std::string>("racket_flat_topic", incoming_backend_
          ? std::string(V2IncomingRuntime::prefix)+"command_flat" : "/racket/command_flat");
  if (incoming_backend_ && (!flight_packet_input_enabled_ || !post_net_one_shot_enabled_ ||
      racket_flat_schema_ != 4 || question_fixture_enabled_ || publish_base_flat_ ||
      publish_serve_ball_flat_ || flat_topic != std::string(V2IncomingRuntime::prefix)+"command_flat"))
    throw std::invalid_argument("v2_incoming requires managed packet/schema4 and protected shadow command outputs");
  rcl_interfaces::msg::ParameterDescriptor plane_descriptor;
  plane_descriptor.read_only = true;
  plane_descriptor.description = "Initial plane mode; runtime changes use /hope/planner/set_plane_mode";
  const auto initial_plane_mode = parse_plane_mode(declare_parameter<std::string>(
      "plane_reference_mode", "fixed", plane_descriptor));
  runtime_plane_configurable_ = racket_flat_schema_ == 4 && post_net_one_shot_enabled_ && !question_fixture_enabled_;
  if (!runtime_plane_configurable_ && initial_plane_mode != PlaneMode::Fixed)
    throw std::invalid_argument("runtime plane modes require shared HOME and one-shot Planner");
  runtime_plane_.request(initial_plane_mode);
  plane_mode_service_ = create_service<rcl_interfaces::srv::SetParametersAtomically>(
      incoming_backend_ ? std::string(V2IncomingRuntime::prefix)+"set_plane_mode" : "/hope/planner/set_plane_mode",
      [this](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
             std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
        try {
          if (!runtime_plane_configurable_) throw std::invalid_argument("This Planner configuration does not support runtime plane modes");
          if (request->parameters.size() != 1 || request->parameters[0].name != "mode" ||
              request->parameters[0].value.type != 4)
            throw std::invalid_argument("Supply exactly one mode(string)");
          const auto mode = parse_plane_mode(request->parameters[0].value.string_value);
          { std::lock_guard<std::mutex> lock(base_mutex_);
            if (mode != PlaneMode::Fixed && shared_home_.value.epoch != 0 && !shared_home_.value.runtime_planes)
              throw std::invalid_argument("Runner upgrade required for dynamic planes");
          }
          runtime_plane_.request(mode);
          response->result.successful = true;
          response->result.reason = "Selected for the next flight; published targets and Runner state are unchanged";
        } catch (const std::exception& error) {
          response->result.successful = false;
          response->result.reason = error.what();
        }
      });

  rcl_interfaces::msg::ParameterDescriptor scale_descriptor;
  scale_descriptor.read_only = true;
  scale_descriptor.description = "Startup scale; live changes use /hope/planner/set_hit_speed_scale";
  const double startup_scale = declare_parameter<double>("hit_speed_scale", 1.0, scale_descriptor);
  hit_speed_scale_state_file_ = declare_parameter<std::string>("hit_speed_scale_state_file", "", scale_descriptor);
  const double initial_scale = question_fixture_enabled_ ? startup_scale :
      load_hit_speed_scale(hit_speed_scale_state_file_, startup_scale);
  if (!std::isfinite(initial_scale) || initial_scale < 0.0 ||
      (question_fixture_enabled_ && initial_scale != 1.0)) {
    throw std::invalid_argument("hit_speed_scale must be finite and nonnegative; exact fixtures require 1");
  }
  hit_speed_scale_.store(initial_scale);
  hit_speed_scale_service_ = create_service<rcl_interfaces::srv::SetParametersAtomically>(
      incoming_backend_ ? std::string(V2IncomingRuntime::prefix)+"set_hit_speed_scale" : "/hope/planner/set_hit_speed_scale",
      [this](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
             std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
        if (question_fixture_enabled_ || request->parameters.size() != 1 ||
            request->parameters[0].name != "scale" || request->parameters[0].value.type != 3 ||
            !std::isfinite(request->parameters[0].value.double_value) ||
            request->parameters[0].value.double_value < 0.0) {
          response->result.successful = false;
          response->result.reason = "Supply exactly one finite nonnegative scale(double); exact fixtures are locked";
          return;
        }
        // Serialize acceptance with publication: after this acknowledgement,
        // a new output cannot publish a scale sampled before the request.
        std::lock_guard<std::mutex> lock(hit_speed_scale_publish_mutex_);
        try {
          save_hit_speed_scale(hit_speed_scale_state_file_, request->parameters[0].value.double_value);
        } catch (const std::exception& error) {
          response->result.successful = false;
          response->result.reason = error.what();
          return;
        }
        hit_speed_scale_.store(request->parameters[0].value.double_value);
        response->result.successful = true;
        response->result.reason = "Applied to subsequent new targets; already published commands are unchanged";
      });

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
      "target_actor_x,target_actor_y,target_actor_z,target_nx,target_ny,target_nz,"
      "session_home_x,session_home_y,reach_level,swing_foot_sign,"
      "estimator_ms,stage2_ms,stage3_ms,total_ms,input_ring_depth,input_ring_drops,"
      "logger_queue_depth,logger_drops,base_valid,base_x,base_y,base_z,base_age_ms,"
      "swing_side_reference_mode,swing_side_reference_y,"
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
      "packet_conflict_total,packet_invalid_total,packet_queue_depth,"
      "question_fixture_active,question_fixture_contract,"
      "question_fixture_bank_sha256,question_fixture_receipt_sha256,"
      "question_fixture_bank_row_id,question_fixture_physical_strike_x,"
      "question_fixture_physical_strike_y,question_fixture_physical_strike_z,"
      "question_fixture_physical_strike_vx,question_fixture_physical_strike_vy,"
      "question_fixture_physical_strike_vz,question_fixture_physical_tts_s,"
      "question_fixture_position_error_max_m,"
      "question_fixture_velocity_error_max_mps,question_fixture_tts_error_s,"
      "question_fixture_tts_s,question_fixture_physical_match,"
      "target_turn_limit_deg,target_turn_allowed,target_turn_reason,"
      "base_yaw_deg,target_bearing_deg,target_turn_deg,post_net_future_bounce_restitution_v,base_source_age_ms,"
      "future_contact_spin_enabled,future_contact_spin_used,future_contact_spin_tangential_gain,"
      "future_contact_spin_omega_scale,future_contact_spin_min_coherence,"
      "future_contact_magnus_enabled,physics_drag_k,physics_magnus_k,physics_table_gain,physics_table_mu,physics_spin_reason,session_home_epoch,plane_mode,plane_reference_x,plane_world_x,plane_selection_reason,hit_speed_scale");
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

  estimator_ = std::make_unique<BatchPhysicsEstimator>(physics_, estimator_config_, spin_estimator_config_);
  spin_estimator_ = std::make_unique<SpinEstimator>(spin_estimator_config_);
  predictor_ = std::make_unique<TrajectoryPredictor>(physics_, planner_config_, table_);
  BallPhysics post_net_physics = physics_;
  // This effective future-contact coefficient is identified for the causal
  // pre-bounce one-shot prediction. The estimator keeps the venue coefficient
  // when an actual bounce has already been observed.
  post_net_physics.table_tangential_gain =
      post_net_future_bounce_tangential_gain_;
  post_net_physics.restitution_v = post_net_future_bounce_restitution_v_;
  post_net_predictor_ = std::make_unique<TrajectoryPredictor>(
      post_net_physics, planner_config_, table_);
  future_contact_predictor_ = std::make_unique<FutureContactPredictor>(
      post_net_physics, planner_config_, table_, future_contact_config_);
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
        incoming_backend_ ? std::string(V2IncomingRuntime::prefix)+"command" : "/racket/command", command_qos);
  }
  if (incoming_backend_ && ((flat_publisher_ && std::string(flat_publisher_->get_topic_name()) !=
      std::string(V2IncomingRuntime::prefix)+"command_flat") ||
      (typed_publisher_ && std::string(typed_publisher_->get_topic_name()) !=
      std::string(V2IncomingRuntime::prefix)+"command")))
    throw std::invalid_argument("v2_incoming shadow command outputs cannot be remapped");
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      incoming_backend_ ? std::string(V2IncomingRuntime::prefix)+"diagnostics" : "/planner/diagnostics", rclcpp::QoS(1));

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
  hit_plane_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      incoming_backend_ ? std::string(V2IncomingRuntime::prefix)+"hit_plane" : "/hope/planner/hit_plane",
      rclcpp::QoS(1).reliable().transient_local());
  diagnostics_timer_ = create_wall_timer(1s, [this] { publish_diagnostics(); });
  if (!x_hit_request_file_.empty()) {
    calibration_timer_ = create_wall_timer(100ms, [this] { poll_calibration_request(); });
  }

  RCLCPP_INFO(get_logger(), "Future contact model: tangential_gain=%.6f restitution_v=%.6f",
              post_net_future_bounce_tangential_gain_, post_net_future_bounce_restitution_v_);
  health_previous_steady_ns_ = steady_now_ns();
  if (incoming_backend_) incoming_runtime_ = std::make_unique<V2IncomingRuntime>(
      *this, physics_, table_, [this] { wake_condition_.notify_one(); }, [this] {
        const auto plane = incoming_preview();
        return std::make_pair(plane.selection.epoch, plane.selection.valid);
      }, [this](const V2StrikeCandidate& status) { return publish_incoming_revocation(status); });
  solver_thread_ = std::thread(&PlannerNode::solver_loop, this);

  RCLCPP_INFO(
      get_logger(),
      "Ball physics: drag_k=%.6f restitution_h=%.6f restitution_v=%.6f "
      "future_zero_spin_gain=%.6f observed_bounce_grip=%.6f magnus_shadow=%.8f",
      physics_.drag_k, physics_.restitution_h, physics_.restitution_v,
      post_net_future_bounce_tangential_gain_, physics_.table_tangential_gain,
      physics_.magnus_k);

  RCLCPP_INFO(
      get_logger(),
      "HOPE C++ planner started: estimator=%s "
      "input_depth=%d solve_period=%.3f x_hit=%.3f freeze_home=%d "
      "schema=%d reach=%s "
      "window=%.3f min_span=%.3f huber=%.4f recency_half_life=%.3f "
      "bounce_reversal=%.5f bounce_excursion=%.4f bounce_confirm=%zu "
      "bounce_sparse=(%.3f,%.3f) bounce_refractory=%.3f "
      "horizon=%.3f adaptive=%d spin_shadow=%d spin_mode=%s debug=%s "
      "post_net_one_shot=%d flight_packet_input=%d packet_topic=%s "
      "net_x=%.3f commit_delay=%.3f "
      "future_bounce_gain=%.3f incoming=(margin=%.3f vin=%.3f vout=%.3f "
      "fit=%zu confirm=%zu preroll=%zu gap=%.3f) question_fixture=%d",
      incoming_backend_ ? "v2_shared_ckf" : "batch_physics_cpp_no_ekf_persistent_bounce",
      input_qos_depth_, solve_period_s_, x_hit_fh_.load(),
      x_hit_freeze_session_home_ ? 1 : 0,
      racket_flat_schema_, reach_permission_config_.contract.c_str(),
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
      incoming_trajectory_config_.source_gap_reset_s,
      question_fixture_enabled_ ? 1 : 0);
  if (require_x_hit_calibration_audit_) {
    RCLCPP_WARN(
        get_logger(),
        "require_x_hit_calibration=true is audit-only in the no-gate C++ path; "
        "the configured x_hit remains executable before and after refresh");
  }
}

PlannerNode::~PlannerNode() {
  integrated_runtime_.reset();
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
      if (incoming_backend_ && !flight_packet_queue_.empty()) {
        flight_packets_invalid_.fetch_add(1, std::memory_order_relaxed);
        write_receive_audit("bounded_command_queue_full", false, flight_packet_queue_.size());
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
  if (incoming_runtime_ && !incoming_runtime_->packet_ready(
      flight_packet_queue_.front().latest_sample()->source_time_ns)) return false;
  snapshot = std::move(flight_packet_queue_.front());
  flight_packet_queue_.pop_front();
  return true;
}

bool PlannerNode::has_pending_flight_packet() const noexcept {
  std::lock_guard<std::mutex> lock(flight_packet_mutex_);
  return !flight_packet_queue_.empty() && (!incoming_runtime_ || incoming_runtime_->packet_ready(
      flight_packet_queue_.front().latest_sample()->source_time_ns));
}

std::size_t PlannerNode::flight_packet_queue_depth() const noexcept {
  std::lock_guard<std::mutex> lock(flight_packet_mutex_);
  return flight_packet_queue_.size();
}

void PlannerNode::set_base_snapshot(const BaseSnapshot& snapshot) noexcept {
  // A valid base packet at process startup is not a HOME receipt.  Gate3 publishes an origin
  // sample before reset/placement; latching here classified both physical lanes as FH.  HOME is
  // frozen at the first valid solve for legacy wire schemas. Schema 4 uses
  // the Runner policy-entry receipt for both plane and side choice.
  const auto update = [this, &snapshot] {
    std::lock_guard<std::mutex> lock(base_mutex_);
    base_ = snapshot;
    return std::make_pair(shared_home_.value.epoch, base_.valid &&
        (!incoming_runtime_ || incoming_runtime_->reference_matches(base_.calibration_id, base_.world_id)));
  };
  if (incoming_runtime_) incoming_runtime_->update_reference(update);
  else update();
}

IncomingPlane PlannerNode::incoming_preview() const {
  std::lock_guard<std::mutex> lock(base_mutex_);
  const auto now = steady_now_ns();
  PlaneSelection selection;
  selection.mode = runtime_plane_.requested();
  selection.epoch = shared_home_.value.epoch;
  selection.reference_x = shared_home_.value.x;
  const bool home_fresh = shared_home_.fresh(now,
      static_cast<std::int64_t>(session_home_max_age_s_ * 1e9), base_.calibration_id, base_.world_id);
  const long double source_age = static_cast<long double>(wall_now_ns()) - base_.source_stamp_ns;
  const bool body_fresh = base_.valid && base_.orientation_valid &&
      now - base_.receipt_steady_ns >= 0 && now - base_.receipt_steady_ns <= target_turn_max_pose_age_s_ * 1e9 &&
      source_age >= -1000000.L && source_age <= target_turn_max_pose_age_s_ * 1e9;
  selection.valid = home_fresh && base_.valid && incoming_runtime_->reference_matches(
      base_.calibration_id, base_.world_id);
  if (selection.mode != PlaneMode::Fixed) {
    selection.valid = selection.valid && body_fresh && shared_home_.value.runtime_planes;
    const auto last = runtime_plane_.last_command();
    if (last.valid && last.epoch == selection.epoch) selection.reference_x = last.reference_x;
    if (selection.mode == PlaneMode::PerBall || shared_home_.posture_stable)
      selection.reference_x = base_.position.x();
  }
  selection.plane_x = selection.reference_x + shared_home_.value.plane_x;
  selection.reason = selection.valid ? "managed_preview" : "reference_unavailable";
  const auto& contract = incoming_runtime_->contract();
  return incoming_plane(selection, contract.frame, contract.registration,
                        selection.mode != PlaneMode::Fixed, base_.source_stamp_ns);
}

bool PlannerNode::publish_incoming_revocation(const V2StrikeCandidate& status) noexcept {
  try {
    std::lock_guard<std::mutex> lock(hit_speed_scale_publish_mutex_);
    const auto identity = schema2_packer_.next_identity(false, steady_now_ns());
    const auto wire = managed_revocation(status, identity, wall_now_ns());
    if (!wire || !flat_publisher_) return false;
    std_msgs::msg::Float64MultiArray message; message.data = *wire;
    flat_publisher_->publish(message);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "incoming revocation publication failed: %s", e.what());
    return false;
  }
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
    std::uint64_t calibration_id = 0, world_id = 0;
    bool valid = value.size() == 16 && value[0] == 2.0 && value[1] == 1.0;
    valid = valid && std::isfinite(value[13]) && value[13] >= 0.0 &&
        value[13] <= static_cast<double>(std::numeric_limits<int>::max()) &&
        a3_deploy::SessionHomeReceipt::integer(value[14], calibration_id) &&
        a3_deploy::SessionHomeReceipt::integer(value[15], world_id);
    if (valid) {
      const int flags = static_cast<int>(value[13]);
      valid = static_cast<double>(flags) == value[13] &&
              (flags & kRequiredBaseFlags) == kRequiredBaseFlags &&
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
    snapshot.orientation_valid = valid;
    if (valid) {
      snapshot.calibration_id = calibration_id;
      snapshot.world_id = world_id;
      snapshot.orientation = Eigen::Quaterniond(value[8], value[9], value[10], value[11]).normalized();
    }
    snapshot.receipt_steady_ns = receipt;
    if (valid) {
      snapshot.source_stamp_ns =
          static_cast<std::int64_t>(value[3]) * 1'000'000'000LL +
          static_cast<std::int64_t>(value[4]);
      add_base_sample(position.x(), receipt);
    } else if ((++invalid_count % 100U) == 1U) {
      RCLCPP_WARN(
          get_logger(),
          "authoritative schema-2 base packet is invalid; "
          "target turn cannot be checked until a valid pose arrives");
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
    const bool orientation_valid = quaternion.coeffs().allFinite() &&
        quaternion.norm() >= 0.5 && quaternion.norm() <= 1.5;
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
    snapshot.orientation = quaternion;
    snapshot.orientation_valid = orientation_valid;
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
  if (racket_flat_schema_ == 4) return solve_plane_.plane_x;
  if (x_hit_follow_robot_) {
    const BaseSnapshot snapshot = base_snapshot();
    if (snapshot.valid) {
      return std::clamp(snapshot.position.x() + x_hit_offset_, x_hit_min_, x_hit_max_);
    }
  }
  return x_hit_fh_.load(std::memory_order_acquire);
}

double PlannerNode::select_swing_sign(
    double intercept_y, double base_x, double base_y) noexcept {
  double reference_y = base_y;
  if (swing_side_reference_mode_ == "session_home_v1" && racket_flat_schema_ == 4) {
    reference_y = solve_home_valid_ ? solve_home_.y : std::numeric_limits<double>::quiet_NaN();
  } else if (swing_side_reference_mode_ == "session_home_v1") {
    std::lock_guard<std::mutex> lock(base_mutex_);
    reference_y =
        session_home_reference_.resolve_xy_on_solve(base_x, base_y).second;
  }
  last_swing_side_reference_y_.store(reference_y, std::memory_order_relaxed);
  const double selected = select_swing_sign_with_hysteresis(
      intercept_y, reference_y, swing_side_split_y_,
      swing_side_hysteresis_y_, last_swing_sign_);
  if (selected == -1.0 || selected == 1.0) {
    last_swing_sign_ = selected;
  }
  return selected;
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
    TrajectorySnapshot incoming_packet;
    while (!stop_requested_.load(std::memory_order_acquire)) {
      {
        std::unique_lock<std::mutex> lock(wake_mutex_);
        wake_condition_.wait_for(lock, 5ms, [this] {
          return stop_requested_.load(std::memory_order_acquire) ||
                 input_ring_.size_approx() > 0 ||
                 snapshot_mailbox_.has_pending() ||
                 has_pending_flight_packet() || (incoming_runtime_ && incoming_runtime_->pending());
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
          if (incoming_runtime_) incoming_packet = snapshot;
          else {
            estimator_->reset();
            spin_estimator_->reset();
            for (std::size_t i = 0; i < snapshot.sample_count; ++i) {
              estimator_->push(snapshot.samples[i]);
              spin_estimator_->push(snapshot.samples[i]);
            }
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
        if (incoming_runtime_) incoming_runtime_->continuous(incoming_preview());
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

      if (racket_flat_schema_ == 4) {
        std::lock_guard<std::mutex> lock(base_mutex_);
        if (solve_home_.epoch != shared_home_.value.epoch) last_swing_sign_ = 0.0;
        solve_home_ = shared_home_.value;
        solve_posture_stable_ = shared_home_.posture_stable;
        solve_home_valid_ = base_.valid && shared_home_.fresh(solve_start_ns,
            static_cast<std::int64_t>(session_home_max_age_s_ * 1e9),
            base_.calibration_id, base_.world_id);
      }
      if (racket_flat_schema_ == 4) {
        const auto body = base_snapshot();
        const double age = (solve_start_ns - body.receipt_steady_ns) * 1e-9;
        const double source_age = (wall_now_ns() - body.source_stamp_ns) * 1e-9;
        const bool fresh = body.valid && body.orientation_valid && body.receipt_steady_ns > 0 &&
            body.source_stamp_ns > 0 && age >= 0 && age <= target_turn_max_pose_age_s_ &&
            source_age >= -0.05 && source_age <= target_turn_max_pose_age_s_;
        solve_plane_ = runtime_plane_.select(solve_home_.epoch, ++plane_flight_counter_,
            solve_home_.x, solve_home_.plane_x, body.position.x(), fresh,
            solve_home_.runtime_planes, solve_home_valid_ && solve_posture_stable_);
        solve_home_valid_ = solve_home_valid_ && solve_plane_.valid;
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
      BallState state = incoming_runtime_ ? BallState{} : estimator_->estimate();
      SpinShadowAudit spin_shadow;
      spin_shadow.enabled = spin_shadow_enabled_ && !incoming_runtime_;
      spin_shadow.mode = spin_shadow.enabled ? spin_shadow_mode_name_ : "disabled";
      spin_shadow.spin = incoming_runtime_ ? SpinEstimate{} :
          (state.spin_model_enabled ? state.physics_spin : spin_estimator_->estimate());
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
      if (incoming_runtime_) {
        const auto& contract = incoming_runtime_->contract();
        auto selected = solve_plane_;
        selected.valid = selected.valid && solve_home_valid_;
        strike = incoming_runtime_->packet(incoming_packet,
            incoming_plane(selected, contract.frame, contract.registration, false), state);
        if (strike.valid) {
          const auto base = base_snapshot();
          swing_sign = select_swing_sign(strike.ball_position.y(), base.position.x(), base.position.y());
        }
      } else if (state.valid && (racket_flat_schema_ != 4 || solve_home_valid_)) {
        strike = post_net_one_shot_enabled_
            ? future_contact_predictor_->predict(state, active_x_hit(), spin_shadow.spin)
            : control_predictor->predict_with_spin(state, active_x_hit(), Vec3::Zero(),
                                                  SpinPhysicsMode::kVenueGripBounce);
        if (strike.valid) {
          const BaseSnapshot base = base_snapshot();
          swing_sign = select_swing_sign(
              strike.ball_position.y(),
              base.valid ? base.position.x()
                         : std::numeric_limits<double>::quiet_NaN(),
              base.valid ? base.position.y()
                         : (swing_side_reference_mode_ == "session_home_v1"
                                ? std::numeric_limits<double>::quiet_NaN()
                                : 0.0));
          if (swing_sign < 0.0 && x_hit_bh_delta_ != 0.0) {
            strike = post_net_one_shot_enabled_
                ? future_contact_predictor_->predict(
                    state, active_x_hit() + x_hit_bh_delta_, spin_shadow.spin)
                : control_predictor->predict_with_spin(
                    state, active_x_hit() + x_hit_bh_delta_, Vec3::Zero(),
                    SpinPhysicsMode::kVenueGripBounce);
          }
        }
      }
      if (racket_flat_schema_ == 4 && !solve_home_valid_) strike.reason =
          solve_plane_.valid ? "runner_home_missing_stale_or_frame_mismatch" : solve_plane_.reason;
      if (!incoming_runtime_ && spin_shadow_enabled_ && state.valid && (racket_flat_schema_ != 4 || solve_home_valid_)) {
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
      RacketCommand command =
          target_planner_->plan(strike, target_land, flight_time);
      if (question_fixture_enabled_) {
        auto& fixture_audit = audit.question_fixture;
        fixture_audit.active = true;
        fixture_audit.contract = question_fixture_->contract();
        fixture_audit.bank_sha256 = question_fixture_->bank_sha256();
        fixture_audit.receipt_sha256 = question_fixture_->receipt_sha256();
        fixture_audit.physical_strike = strike;
        const QuestionBankFixtureRow* const fixture_row =
            question_fixture_->row_for_flight(audit.trajectory_epoch);
        if (fixture_row == nullptr) {
          command = RacketCommand{};
          command.reason = "question_fixture_flight_missing";
        } else {
          fixture_audit.bank_row_id = fixture_row->bank_row_id;
          fixture_audit.fixture_tts_s = fixture_row->time_to_strike_s;
          double immutable_home_x = std::numeric_limits<double>::quiet_NaN();
          double immutable_home_y = std::numeric_limits<double>::quiet_NaN();
          {
            std::lock_guard<std::mutex> lock(base_mutex_);
            if (session_home_reference_.set) {
              immutable_home_x = session_home_reference_.x;
              immutable_home_y = session_home_reference_.y;
            }
          }
          const Vec3 expected_physical_position(
              immutable_home_x + fixture_row->contact_position_offset.x(),
              immutable_home_y + fixture_row->contact_position_offset.y(),
              fixture_row->contact_position_offset.z() - policy_z_offset_);
          fixture_audit.physical_tts_s = strike.strike_source_time_s -
              static_cast<double>(wall_now_ns()) * 1.0e-9;
          fixture_audit.position_error_max_m =
              (strike.ball_position - expected_physical_position)
                  .cwiseAbs().maxCoeff();
          fixture_audit.velocity_error_max_mps =
              (strike.ball_velocity - fixture_row->incoming_velocity)
                  .cwiseAbs().maxCoeff();
          fixture_audit.tts_error_s = std::abs(
              fixture_audit.physical_tts_s - fixture_row->time_to_strike_s);
          const double expected_swing_sign = fixture_row->clip == 0 ? 1.0 : -1.0;
          fixture_audit.physical_match =
              strike.valid && expected_physical_position.allFinite() &&
              audit.trajectory_epoch == fixture_row->flight_id &&
              swing_sign == expected_swing_sign &&
              fixture_audit.position_error_max_m <=
                  question_fixture_max_position_error_m_ &&
              fixture_audit.velocity_error_max_mps <=
                  question_fixture_max_velocity_error_mps_ &&
              fixture_audit.tts_error_s <= question_fixture_max_tts_error_s_;
          if (fixture_audit.physical_match) {
            strike.ball_position = expected_physical_position;
            strike.ball_velocity = fixture_row->incoming_velocity;
            strike.predicted_bounces =
                fixture_audit.physical_strike.predicted_bounces;
            strike.valid = true;
            strike.reason = "question_fixture_exact_v5_row";
            command.position = expected_physical_position;
            command.velocity = fixture_row->target_velocity;
            command.normal = fixture_row->target_normal;
            command.outgoing_ball_velocity =
                fixture_row->predicted_outgoing_velocity;
            command.target_land = fixture_row->intended_landing;
            command.strike_source_time_s = strike.strike_source_time_s;
            command.predicted_bounces = strike.predicted_bounces;
            command.clears_net = true;
            command.bypasses_net_posts = false;
            command.valid = true;
            command.reason = "question_fixture_exact_v5_row";
          } else {
            command = RacketCommand{};
            command.reason = "question_fixture_physical_mismatch";
          }
        }
      }
      audit.stage3_ms = (steady_now_ns() - stage3_start) * 1.0e-6;
      const auto solve_finish_ns = steady_now_ns();
      audit.total_ms = (solve_finish_ns - solve_start_ns) * 1.0e-6;
      if (audit.total_ms > solve_period_s_ * 1000.0) {
        ++solver_deadline_miss_count_;
      }
      audit.reason = command.valid ? "command_valid" :
          (strike.valid ? command.reason : strike.reason);
      ++solve_count_;
      if (incoming_runtime_) {
        incoming_runtime_->command([&](V2StrikeCandidate& candidate) {
          WireIdentity identity;
          const bool valid = publish_solve(latest_sample, state, strike, command,
              swing_sign, audit, spin_shadow, one_shot_event, solve_finish_ns, &identity);
          candidate.command_sequence = identity.command_sequence;
          candidate.command_flight_id = identity.flight_id;
          candidate.command_revision_id = identity.revision_id;
          return valid;
        });
      } else publish_solve(latest_sample, state, strike, command, swing_sign, audit,
                           spin_shadow, one_shot_event, solve_finish_ns);
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

bool PlannerNode::publish_solve(
    const BallSample& latest_sample,
    const BallState& state,
    const StrikeTarget& strike,
    const RacketCommand& unscaled_command,
    double swing_sign,
    const SolveAudit& audit,
    const SpinShadowAudit& spin_shadow,
    const PostNetOneShotEvent& one_shot_event,
    std::int64_t solve_finished_steady_ns, WireIdentity* published_identity) noexcept {
  try {
    std::lock_guard<std::mutex> filter_lock(hit_speed_scale_publish_mutex_);
    const double hit_speed_scale = hit_speed_scale_.load();
    RacketCommand command = unscaled_command;
    command.velocity *= hit_speed_scale;
    if (!command.velocity.allFinite()) {
      command.valid = false;
      command.reason = "hit_speed_scale_overflow";
    }
    // With /poses.header.stamp in the camera-exposure Unix epoch, Stage 2's
    // strike_source_time_s is already the true absolute crossing deadline.
    // Subtract the HDU wall time once at publication. This accounts for the
    // complete exposure->Motive->Laptop->HDU path and solver time without a
    // guessed fixed latency constant.
    const auto producer_wall_ns = wall_now_ns();
    const double producer_wall_s =
        static_cast<double>(producer_wall_ns) * 1.0e-9;
    const bool exact_fixture_timing =
        audit.question_fixture.active && audit.question_fixture.physical_match &&
        std::isfinite(audit.question_fixture.fixture_tts_s);
    const double strike_deadline_wall_s = command.valid
        ? (exact_fixture_timing
               ? producer_wall_s + audit.question_fixture.fixture_tts_s
               : command.strike_source_time_s)
        : 0.0;
    const double time_to_strike_s = command.valid
        ? strike_deadline_wall_s - producer_wall_s
        : 0.0;
    const bool legal_swing_sign = swing_sign == -1.0 || swing_sign == 1.0;
    bool wire_valid = command.valid && legal_swing_sign &&
                      std::isfinite(time_to_strike_s) &&
                      time_to_strike_s > 0.0;
    const char* timing_reject_reason = !legal_swing_sign ? "invalid_swing_sign" :
        (!std::isfinite(time_to_strike_s) ? "strike_deadline_invalid" : "strike_deadline_expired");
    if (racket_flat_schema_ == 4) {
      std::lock_guard<std::mutex> lock(base_mutex_);
      wire_valid = wire_valid && solve_home_valid_ &&
          solve_home_.epoch == shared_home_.value.epoch && base_.valid &&
          shared_home_.fresh(steady_now_ns(),
              static_cast<std::int64_t>(session_home_max_age_s_ * 1e9),
              base_.calibration_id, base_.world_id);
    }
    const bool timing_valid = wire_valid;
    // Use one consistent pose for admission and audit. Reject the entire new
    // command; never clamp/rotate the target or alter an in-progress swing.
    const BaseSnapshot base = base_snapshot();
    TargetTurnDecision target_turn;
    const bool target_turn_evaluated = wire_valid;
    if (target_turn_evaluated) {
      target_turn = check_target_turn(
          command.position, base.position, base.orientation,
          base.valid && base.orientation_valid, max_target_turn_deg_);
      const double pose_age_s = (steady_now_ns() - base.receipt_steady_ns) * 1.0e-9;
      if (target_turn_reject_stale_pose_ && target_turn.allowed && (base.receipt_steady_ns <= 0 ||
          pose_age_s < 0.0 || pose_age_s > target_turn_max_pose_age_s_)) {
        target_turn.allowed = false;
        target_turn.reason = "target_turn_base_stale";
      }
      // A packet can be newly received after spending hundreds of ms in
      // Wi-Fi queues. Receipt age alone cannot establish current heading.
      const double source_pose_age_s = base.source_stamp_ns > 0
          ? (producer_wall_ns - base.source_stamp_ns) * 1.0e-9 : 0.0;
      if (target_turn_reject_stale_pose_ && target_turn.allowed && source_pose_age_s > target_turn_max_pose_age_s_) {
        target_turn.allowed = false;
        target_turn.reason = "target_turn_base_source_stale";
      }
      if (!target_turn_reject_stale_pose_ && target_turn.allowed &&
          (base.receipt_steady_ns <= 0 || pose_age_s < 0.0 ||
           pose_age_s > target_turn_max_pose_age_s_ || source_pose_age_s > target_turn_max_pose_age_s_)) {
        RCLCPP_WARN(get_logger(),
            "Planner body timing audit: reject_stale_pose=0 receipt_age_ms=%.3f "
            "source_age_ms=%.3f limit_ms=%.3f; pose age does not veto this command",
            pose_age_s * 1.0e3, source_pose_age_s * 1.0e3, target_turn_max_pose_age_s_ * 1.0e3);
      }
      if (!target_turn.allowed) {
        wire_valid = false;
        RCLCPP_WARN(get_logger(),
            "Planner command rejected: %s turn=%.3f deg limit=%.3f deg",
            target_turn.reason, target_turn.turn_deg, max_target_turn_deg_);
      }
    }
    ReachPermission candidate_permission;
    ReachPermission candidate_geometry_permission;
    double immutable_home_x = std::numeric_limits<double>::quiet_NaN();
    double immutable_home_y = std::numeric_limits<double>::quiet_NaN();
    Vec3 target_position_actor_world = Vec3::Constant(
        std::numeric_limits<double>::quiet_NaN());
    if (racket_flat_schema_ == 4) {
      immutable_home_x = solve_home_.x;
      immutable_home_y = solve_home_.y;
      target_position_actor_world = command.position + Vec3(0.0, 0.0, policy_z_offset_);
    }
    if (wire_valid && racket_flat_schema_ == 3) {
      {
        std::lock_guard<std::mutex> lock(base_mutex_);
        if (session_home_reference_.set) {
          immutable_home_x = session_home_reference_.x;
          immutable_home_y = session_home_reference_.y;
        }
      }
      // Re-check every revision against the receipt-bound target envelope.
      // Only the support permission is latched per flight; target admission
      // itself must never be bypassed by a later OOD revision.
      candidate_permission = classify_reach_permission(
          reach_permission_config_, command.position.y(), immutable_home_y,
          swing_sign, time_to_strike_s);
      candidate_geometry_permission = classify_reach_permission(
          reach_permission_config_, command.position.y(), immutable_home_y,
          swing_sign,
          std::max(time_to_strike_s, reach_permission_config_.minimum_tts_s));
      target_position_actor_world =
          command.position + Vec3(0.0, 0.0, policy_z_offset_);
      if (!candidate_permission.target_admitted ||
          !target_tuple_inside_axis_prefilter(
              reach_permission_config_, target_position_actor_world,
              command.velocity, command.normal, immutable_home_x,
              immutable_home_y,
              swing_sign)) {
        wire_valid = false;
      }
    }
    const auto identity = schema2_packer_.next_identity(
        wire_valid, solve_finished_steady_ns, racket_flat_schema_ == 4 && post_net_one_shot_enabled_);
    if (wire_valid && racket_flat_schema_ == 3) {
      if (identity.flight_id != reach_permission_flight_id_) {
        reach_permission_latched_ = candidate_permission;
        reach_permission_geometry_latched_ = candidate_geometry_permission;
        reach_permission_flight_id_ = identity.flight_id;
      } else if (!revision_preserves_reach_geometry_cell(
                     reach_permission_geometry_latched_,
                     candidate_geometry_permission)) {
        wire_valid = false;
      }
    }
    bool packet_valid = false;
    std::vector<double> packet_values;
    ReachPermission permission;
    if (racket_flat_schema_ == 3) {
      permission = wire_valid ? reach_permission_latched_ : ReachPermission{};
      const Schema3Packet packet = Schema3Packer::pack(
          wire_valid ? &command : nullptr,
          swing_sign,
          strike_deadline_wall_s,
          policy_z_offset_,
          producer_wall_ns,
          identity,
          state.sample_count,
          state.sample_span_s,
          permission);
      packet_valid = packet.valid;
      packet_values.assign(packet.values.begin(), packet.values.end());
    } else {
      const Schema2Packet packet = Schema2Packer::pack(
          wire_valid ? &command : nullptr,
          swing_sign,
          strike_deadline_wall_s,
          policy_z_offset_,
          producer_wall_ns,
          identity,
          state.sample_count,
          state.sample_span_s);
      packet_valid = packet.valid;
      packet_values.assign(packet.values.begin(), packet.values.end());
      if (racket_flat_schema_ == 4) {
        packet_values[0] = 4.0;
        packet_values.push_back(packet_valid ? static_cast<double>(solve_home_.epoch) : 0.0);
        if (solve_plane_.mode != PlaneMode::Fixed) {
          packet_values[0] = 5.0;
          packet_values.push_back(packet_valid ? solve_plane_.reference_x : 0.0);
          packet_values.push_back(static_cast<double>(solve_plane_.mode));
        }
      }
    }
    if (packet_valid) {
      if (published_identity) *published_identity = identity;
      last_command_hit_speed_scale_.store(hit_speed_scale);
      if (racket_flat_schema_ == 4) {
        auto published_plane = solve_plane_;
        published_plane.flight = identity.flight_id;
        runtime_plane_.commit(published_plane);
      }
      ++valid_count_;
    }

    if (flat_publisher_) {
      std_msgs::msg::Float64MultiArray output;
      output.data = packet_values;
      flat_publisher_->publish(output);
    }

    if (typed_publisher_) {
      hope_msgs::msg::RacketCommand output;
      output.header.stamp = get_clock()->now();
      output.header.frame_id = "world";
      if (packet_valid) {
        output.position.x = command.position.x();
        output.position.y = command.position.y();
        output.position.z = command.position.z() + policy_z_offset_;
        output.velocity.x = command.velocity.x();
        output.velocity.y = command.velocity.y();
        output.velocity.z = command.velocity.z();
        output.normal.x = command.normal.x();
        output.normal.y = command.normal.y();
        output.normal.z = command.normal.z();
        output.strike_time = strike_deadline_wall_s;
        output.time_to_strike = packet_valid ? time_to_strike_s : 0.0;
        output.ball_velocity_outgoing.x = command.outgoing_ball_velocity.x();
        output.ball_velocity_outgoing.y = command.outgoing_ball_velocity.y();
        output.ball_velocity_outgoing.z = command.outgoing_ball_velocity.z();
      }
      output.valid = packet_valid;
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
          << (packet_valid ? identity.flight_id : 0) << ','
          << (packet_valid ? identity.revision_id : 0) << ','
          << latest_sample.source_time_s << ',' << source_age_at_publish_ms << ','
          << strike_deadline_wall_s << ',' << producer_wall_s << ','
          << latest_sample.receipt_steady_ns << ','
          << solve_finished_steady_ns << ',' << (packet_valid ? 1 : 0) << ','
          << (command.valid && !timing_valid ? timing_reject_reason :
              (target_turn_evaluated && !target_turn.allowed
                  ? target_turn.reason : (unscaled_command.valid && !command.valid ? command.reason : audit.reason))) << ','
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
          << (packet_valid ? time_to_strike_s : 0.0) << ','
          << target_position_actor_world.x() << ','
          << target_position_actor_world.y() << ','
          << target_position_actor_world.z() << ','
          << command.normal.x() << ',' << command.normal.y() << ','
          << command.normal.z() << ',' << immutable_home_x << ','
          << immutable_home_y << ',' << permission.reach_level << ','
          << permission.swing_foot_sign << ','
          << audit.estimator_ms << ',' << audit.stage2_ms << ',' << audit.stage3_ms << ','
          << audit.total_ms << ','
          << (post_net_one_shot_enabled_ ? 0 : input_ring_.size_approx()) << ','
          << ring_drops_.load() << ',' << audit_logger_->queue_depth() << ','
          << audit_logger_->dropped_rows() << ',' << (base.valid ? 1 : 0) << ','
          << base.position.x() << ',' << base.position.y() << ',' << base.position.z() << ','
          << base_age_ms << ',' << swing_side_reference_mode_ << ','
          << last_swing_side_reference_y_.load(std::memory_order_relaxed)
          << ",batch_physics_cpp_no_ekf_persistent_bounce,"
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
                  ? (future_contact_config_.magnus_enabled ? "post_net_one_shot_physical_spin_magnus" :
                     future_contact_predictor_->uses_spin(spin_shadow.spin)
                      ? "post_net_one_shot_spin_conditioned_bounce"
                      : "post_net_one_shot_effective_bounce_zero_spin")
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
          << flight_packet_queue_depth() << ','
          << (audit.question_fixture.active ? 1 : 0) << ','
          << audit.question_fixture.contract << ','
          << audit.question_fixture.bank_sha256 << ','
          << audit.question_fixture.receipt_sha256 << ','
          << audit.question_fixture.bank_row_id << ','
          << audit.question_fixture.physical_strike.ball_position.x() << ','
          << audit.question_fixture.physical_strike.ball_position.y() << ','
          << audit.question_fixture.physical_strike.ball_position.z() << ','
          << audit.question_fixture.physical_strike.ball_velocity.x() << ','
          << audit.question_fixture.physical_strike.ball_velocity.y() << ','
          << audit.question_fixture.physical_strike.ball_velocity.z() << ','
          << audit.question_fixture.physical_tts_s << ','
          << audit.question_fixture.position_error_max_m << ','
          << audit.question_fixture.velocity_error_max_mps << ','
          << audit.question_fixture.tts_error_s << ','
          << audit.question_fixture.fixture_tts_s << ','
          << (audit.question_fixture.physical_match ? 1 : 0) << ','
          << max_target_turn_deg_ << ',' << (target_turn.allowed ? 1 : 0) << ','
          << target_turn.reason << ',' << target_turn.base_yaw_deg << ','
          << target_turn.target_bearing_deg << ',' << target_turn.turn_deg << ','
          << post_net_future_bounce_restitution_v_ << ','
          << (base.source_stamp_ns > 0 ? (producer_wall_ns-base.source_stamp_ns)*1.0e-6 :
              std::numeric_limits<double>::quiet_NaN()) << ','
          << (future_contact_config_.spin_enabled ? 1 : 0) << ','
          << (post_net_one_shot_enabled_ && future_contact_predictor_->uses_spin(spin_shadow.spin) ? 1 : 0) << ','
          << future_contact_config_.tangential_gain << ','
          << future_contact_config_.omega_scale << ',' << future_contact_config_.min_coherence << ','
          << future_contact_config_.magnus_enabled << ',' << physics_.drag_k << ','
          << physics_.magnus_k << ',' << physics_.table_tangential_gain << ','
          << physics_.table_friction_cap_mu << ',' << state.physics_spin.reason << ',' << (racket_flat_schema_ == 4 ? solve_home_.epoch : 0)
          << ',' << plane_mode_name(solve_plane_.mode) << ',' << solve_plane_.reference_x
          << ',' << solve_plane_.plane_x << ',' << solve_plane_.reason
          << ',' << hit_speed_scale;
      audit_logger_->enqueue(row.str());
    }
    return packet_valid;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "failed to publish one planner solve");
  }
  return false;
}

void PlannerNode::publish_diagnostics() noexcept {
  try {
    // Scheduling reference only; final contact selection and the 40 degree
    // guard remain in the solver. Send both-side front bound after calibration.
    std_msgs::msg::Float64MultiArray hit_plane;
    hit_plane.data = {1.0, static_cast<double>(wall_now_ns()) * 1e-9,
                     x_hit_follow_robot_ ? x_hit_max_
                         : x_hit_fh_.load() + std::max(0.0, x_hit_bh_delta_)};
    a3_deploy::SessionHomeMailbox home;
    bool home_fresh = false;
    if (racket_flat_schema_ == 4) {
      std::lock_guard<std::mutex> lock(base_mutex_);
      home = shared_home_;
      home_fresh = base_.valid && home.fresh(steady_now_ns(),
          static_cast<std::int64_t>(session_home_max_age_s_ * 1e9),
          base_.calibration_id, base_.world_id);
      hit_plane.data[2] = home.value.x + home.value.plane_x;
      if (runtime_plane_.requested() != PlaneMode::Fixed) {
        const auto last = runtime_plane_.last_command();
        if (last.valid && last.epoch == home.value.epoch)
          hit_plane.data[2] = std::max(hit_plane.data[2], last.plane_x);
        if (base_.valid) hit_plane.data[2] = std::max(hit_plane.data[2], base_.position.x() + home.value.plane_x);
      }
    }
    if (racket_flat_schema_ != 4 || home_fresh) hit_plane_publisher_->publish(hit_plane);
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
    status.message = incoming_backend_ ? "v2_shared_ckf; shadow commands; packet receive counters" :
        "batch_physics_cpp_no_ekf_persistent_bounce; diagnostics are audit-only";
    if (incoming_runtime_) {
      const auto counters = incoming_runtime_->counters();
      for (const auto& entry : std::vector<std::pair<std::string, std::uint64_t>>{
          {"incoming_observations", counters.offered}, {"incoming_qualified", counters.qualified},
          {"incoming_tracker_accepted", counters.tracker_accepted},
          {"incoming_tracker_rejected", counters.tracker_rejected},
          {"incoming_queries", counters.queries}, {"incoming_completed", counters.completed},
          {"incoming_obsolete", counters.obsolete}, {"incoming_commands", counters.commands},
          {"incoming_revoked", counters.revoked}, {"incoming_history", counters.history},
          {"incoming_checkpoints", counters.checkpoints},
          {"incoming_superseded", incoming_runtime_->superseded()}})
        status.values.push_back(diagnostic_value(entry.first, std::to_string(entry.second)));
    }
    status.values.push_back(diagnostic_value("hit_speed_scale", scale_string(hit_speed_scale_.load())));
    status.values.push_back(diagnostic_value("hit_speed_scale_configurable", question_fixture_enabled_ ? "false" : "true"));
    status.values.push_back(diagnostic_value("hit_speed_scale_persistent", !question_fixture_enabled_ && !hit_speed_scale_state_file_.empty() ? "true" : "false"));
    status.values.push_back(diagnostic_value("hit_speed_scale_last_command", scale_string(last_command_hit_speed_scale_.load())));
    const auto selected_plane = runtime_plane_.last_selection();
    const auto command_plane = runtime_plane_.last_command();
    const bool current_plane = home_fresh && command_plane.valid && command_plane.epoch == home.value.epoch;
    status.values.push_back(diagnostic_value("plane_mode_configurable", runtime_plane_configurable_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("plane_mode_requested", plane_mode_name(runtime_plane_.requested())));
    status.values.push_back(diagnostic_value("plane_mode_last_command", current_plane ? plane_mode_name(command_plane.mode) : "none"));
    status.values.push_back(diagnostic_value("plane_last_flight", current_plane ? std::to_string(command_plane.flight) : "0"));
    status.values.push_back(diagnostic_value("plane_reference_x", number_string(current_plane ? command_plane.reference_x : NAN, 6)));
    status.values.push_back(diagnostic_value("plane_world_x", number_string(current_plane ? command_plane.plane_x : NAN, 6)));
    status.values.push_back(diagnostic_value("plane_selection_reason", selected_plane.epoch == home.value.epoch ? selected_plane.reason : "waiting_for_ball"));
    status.values.push_back(diagnostic_value("plane_runtime_supported", home_fresh && home.value.runtime_planes ? "true" : "false"));
    status.values.push_back(diagnostic_value("plane_posture_stable", home_fresh && home.posture_stable ? "true" : "false"));
    status.values.push_back(diagnostic_value("session_home_source", racket_flat_schema_ == 4 ? "runner" : "planner_legacy"));
    status.values.push_back(diagnostic_value("session_home_epoch", std::to_string(home.value.epoch)));
    status.values.push_back(diagnostic_value("session_home_fresh", home_fresh ? "true" : "false"));
    status.values.push_back(diagnostic_value("session_home_x", number_string(home.value.x, 6)));
    status.values.push_back(diagnostic_value("session_home_y", number_string(home.value.y, 6)));
    status.values.push_back(diagnostic_value("session_home_plane_x", number_string(home.value.plane_x, 6)));
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
        "swing_side_reference_mode", swing_side_reference_mode_));
    status.values.push_back(diagnostic_value(
        "swing_side_reference_y",
        number_string(
            last_swing_side_reference_y_.load(std::memory_order_relaxed), 4)));
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
        "post_net_future_bounce_restitution_v",
        number_string(post_net_future_bounce_restitution_v_, 6)));
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
        "estimator_kind", estimator_config_.spin_enabled
            ? "batch_physics_cpp_spin_magnus_persistent_bounce"
            : "batch_physics_cpp_no_ekf_persistent_bounce"));
    status.values.push_back(diagnostic_value(
        "control_stage2_mode", future_contact_config_.magnus_enabled
            ? "post_net_one_shot_physical_spin_magnus"
            : future_contact_config_.spin_enabled
            ? "post_net_one_shot_spin_conditioned_with_zero_spin_fallback"
            : "post_net_one_shot_effective_bounce_zero_spin"));
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
  if (racket_flat_schema_ == 4) return {false, "Runner owns session HOME and ONNX strike plane; re-enter policy mode to recapture HOME"};
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
    double base_x = *middle;
    double base_y = std::numeric_limits<double>::quiet_NaN();
    double snapshot_age = std::numeric_limits<double>::quiet_NaN();
    if (x_hit_freeze_session_home_) {
      // This is the explicit playing-session boundary.  It is legal to retry
      // before the first flight, but HOME may never be rewritten after any
      // flight packet has entered the Planner.
      if (flight_packets_received_.load(std::memory_order_acquire) != 0U ||
          valid_count_.load(std::memory_order_acquire) != 0U) {
        return {
            false,
            "NOT CALIBRATED: session HOME freeze is forbidden after first flight"};
      }
      const BaseSnapshot snapshot = base_snapshot();
      snapshot_age = (now - snapshot.receipt_steady_ns) * 1.0e-9;
      if (!snapshot.valid || !snapshot.position.allFinite() ||
          snapshot.receipt_steady_ns <= 0 || snapshot_age < 0.0 ||
          snapshot_age > x_hit_calibration_max_age_s_) {
        return {
            false,
            "NOT CALIBRATED: session HOME base snapshot is invalid or stale"};
      }
      // Derive both values from this exact snapshot.  Using the historical
      // median for x_hit while freezing latest-pose HOME can violate the
      // Schema28 1.5 mm target-tuple receipt even when both are individually
      // reasonable measurements.
      base_x = snapshot.position.x();
      base_y = snapshot.position.y();
    }
    const double x_hit = base_x + x_hit_calibration_offset_;
    if (!std::isfinite(x_hit)) {
      return {false, "NOT CALIBRATED: derived x_hit is non-finite"};
    }
    if (x_hit_freeze_session_home_) {
      std::lock_guard<std::mutex> lock(base_mutex_);
      if (!session_home_reference_.freeze_xy_at_session_boundary(base_x, base_y)) {
        return {false, "NOT CALIBRATED: session HOME is non-finite"};
      }
      x_hit_fh_.store(x_hit, std::memory_order_release);
    } else {
      x_hit_fh_.store(x_hit, std::memory_order_release);
    }
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
            << (x_hit_freeze_session_home_
                    ? "; session_home=(" + number_string(base_x, 4) + "," +
                          number_string(base_y, 4) + ") snapshot_age=" +
                          number_string(snapshot_age, 4)
                    : "")
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
