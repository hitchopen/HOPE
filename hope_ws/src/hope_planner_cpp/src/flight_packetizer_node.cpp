#include "hope_planner_cpp/flight_packetizer_node.hpp"
#include "hope_planner_cpp/local_control_key.hpp"
#include "hope_planner_cpp/future_contact_parameters.hpp"

#include "hope_planner_cpp/flight_packet.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>

#include <unistd.h>

namespace hope_planner_cpp {
namespace {

using namespace std::chrono_literals;

std::int64_t seconds_to_ns(double value) noexcept {
  return std::isfinite(value)
      ? static_cast<std::int64_t>(std::llround(value * 1.0e9))
      : 0;
}

std::string default_instance_id(std::int64_t wall_ns) {
  std::ostringstream output;
  output << "packetizer-" << ::getpid() << '-' << wall_ns;
  return output.str();
}

IncomingTrajectoryConfig declare_trajectory_config(rclcpp::Node& node) {
  IncomingTrajectoryConfig config;
  config.net_x = node.declare_parameter<double>("net_x", 1.37);
  config.estimator_window_s =
      node.declare_parameter<double>("flight_window_s", 0.18);
  config.estimator_max_sample_gap_s =
      node.declare_parameter<double>("batch_max_sample_gap_s", 0.0);
  config.commit_delay_s =
      node.declare_parameter<double>("post_net_commit_delay_s", 0.05);
  config.adaptive.enabled = node.declare_parameter<bool>("adaptive_commit_enabled", false);
  config.adaptive.policy_lead_s = node.declare_parameter<double>("adaptive_policy_lead_s", 0.35);
  config.adaptive.transport_budget_s = node.declare_parameter<double>("adaptive_transport_budget_s", 0.10);
  config.adaptive.solve_budget_s = node.declare_parameter<double>("adaptive_solve_budget_s", 0.005);
  config.adaptive.policy_tick_s = node.declare_parameter<double>("adaptive_policy_tick_s", 0.02);
  config.adaptive.max_sample_age_s = node.declare_parameter<double>("adaptive_max_sample_age_s", 0.05);
  config.adaptive.evaluation_period_s = node.declare_parameter<double>("adaptive_evaluation_period_s", 0.01);
  config.opponent_side_margin_m =
      node.declare_parameter<double>("incoming_opponent_side_margin_m", 0.05);
  config.incoming_speed_threshold_mps =
      node.declare_parameter<double>("incoming_speed_threshold_mps", 0.25);
  config.outgoing_speed_threshold_mps =
      node.declare_parameter<double>("outgoing_speed_threshold_mps", 0.25);
  config.source_gap_reset_s =
      node.declare_parameter<double>("incoming_source_gap_reset_s", 0.25);
  config.direction_fit_samples = static_cast<std::size_t>(std::max(
      3, static_cast<int>(node.declare_parameter<int>(
             "incoming_direction_fit_samples", 4))));
  config.direction_confirmations = static_cast<std::size_t>(std::max(
      1, static_cast<int>(node.declare_parameter<int>(
             "incoming_direction_confirmations", 2))));
  config.pre_roll_samples = static_cast<std::size_t>(std::max(
      6, static_cast<int>(node.declare_parameter<int>(
             "incoming_pre_roll_samples", 24))));
  return config;
}

CommitPredictionConfig declare_prediction_config(rclcpp::Node& node) {
  CommitPredictionConfig config;
  config.future_contact = declare_future_contact_config(node);
  config.spin_estimator = declare_spin_estimator_config(node);
  config.observed_physics.drag_k = node.declare_parameter<double>("drag_k", BallPhysics{}.drag_k);
  config.observed_physics.magnus_k = node.declare_parameter<double>("magnus_k", BallPhysics{}.magnus_k);
  config.observed_physics.restitution_h = node.declare_parameter<double>("restitution_h", BallPhysics{}.restitution_h);
  config.observed_physics.restitution_v = node.declare_parameter<double>("restitution_v", BallPhysics{}.restitution_v);
  config.observed_physics.table_tangential_gain = node.declare_parameter<double>("table_tangential_gain", BallPhysics{}.table_tangential_gain);
  config.observed_physics.table_friction_cap_mu = node.declare_parameter<double>("table_friction_cap_mu", BallPhysics{}.table_friction_cap_mu);
  config.estimator.spin_enabled = config.future_contact.magnus_enabled;
  config.estimator.spin_min_coherence = config.future_contact.min_coherence;
  config.estimator.single_flight_only =
      node.declare_parameter<bool>("batch_single_flight_only", false);
  config.estimator.min_span_s =
      node.declare_parameter<double>("batch_estimator_min_span_s", 0.08);
  config.estimator.min_samples = static_cast<std::size_t>(std::max(
      6, static_cast<int>(node.declare_parameter<int>("batch_estimator_min_samples", 12))));
  config.estimator.huber_delta_m = node.declare_parameter<double>("batch_estimator_huber_delta_m", 0.003);
  config.estimator.recency_half_life_s =
      node.declare_parameter<double>("batch_estimator_recency_half_life_s", 0.0);
  config.estimator.robust_iterations = node.declare_parameter<int>("batch_estimator_iterations", 3);
  config.future_physics = config.observed_physics;
  config.future_physics.table_tangential_gain = node.declare_parameter<double>("post_net_future_bounce_tangential_gain", config.observed_physics.table_tangential_gain);
  config.future_physics.restitution_v = node.declare_parameter<double>("post_net_future_bounce_restitution_v", config.observed_physics.restitution_v);
  config.planner.x_hit = node.declare_parameter<double>("adaptive_scheduling_x_hit", 0.15);
  config.planner.adaptive_predict_horizon = true;
  config.estimator.post_bounce_min_span_s = node.declare_parameter<double>(
      "batch_post_bounce_min_span_s", 0.0);
  config.estimator.post_bounce_exclusion_s = node.declare_parameter<double>(
      "batch_post_bounce_exclusion_s", 0.008);
  return config;
}

PlayStateConfig declare_play_state_config(rclcpp::Node& node) {
  PlayStateConfig c;
  c.input_timeout_s = node.declare_parameter<double>("play_state_input_timeout_s", c.input_timeout_s);
  c.rally_min_speed_mps = node.declare_parameter<double>("play_state_rally_min_speed_mps", c.rally_min_speed_mps);
  c.rally_slow_confirm_s = node.declare_parameter<double>("play_state_rally_slow_confirm_s", c.rally_slow_confirm_s);
  c.direction_speed_mps = node.declare_parameter<double>("play_state_direction_speed_mps", c.direction_speed_mps);
  c.direction_confirm_s = node.declare_parameter<double>("play_state_direction_confirm_s", c.direction_confirm_s);
  c.launch_span_s = node.declare_parameter<double>("play_state_launch_span_s", c.launch_span_s);
  c.held_span_s = node.declare_parameter<double>("play_state_held_span_s", c.held_span_s);
  c.contact_height_m = node.declare_parameter<double>("play_state_contact_height_m", c.contact_height_m);
  if (!(c.input_timeout_s > 0 && c.input_timeout_s <= 1 && c.direction_speed_mps > 0 &&
        c.rally_min_speed_mps > 0 && c.rally_min_speed_mps <= 0.10 &&
        c.rally_slow_confirm_s >= 0.100 && c.rally_slow_confirm_s <= 1 &&
        c.direction_confirm_s > 0 && c.launch_span_s >= 0.03 && c.launch_span_s <= 0.2 &&
        c.held_span_s >= c.launch_span_s && c.held_span_s <= 0.24 && c.contact_height_m > 0 &&
        c.contact_height_m < 0.10)) throw std::invalid_argument("invalid play state thresholds");
  return c;
}

}  // namespace

std::int64_t FlightPacketizerNode::steady_now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t FlightPacketizerNode::wall_now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

FlightPacketizerNode::FlightPacketizerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hope_ball_flight_packetizer", options),
      trajectory_config_(declare_trajectory_config(*this)),
      prediction_config_(declare_prediction_config(*this)),
      trajectory_(trajectory_config_, prediction_config_),
      play_state_(declare_play_state_config(*this)),
      state_spin_(prediction_config_.spin_estimator) {
  auto estimator = prediction_config_.estimator;
  estimator.window_s = 0.10;
  estimator.min_span_s = 0.060;
  estimator.min_samples = 8;
  // Contacts are split by the persistent classifier ledger before this fit.
  estimator.single_flight_only = true;
  state_estimator_ = std::make_unique<BatchPhysicsEstimator>(
      prediction_config_.observed_physics, estimator, prediction_config_.spin_estimator);
  last_hit_plane_x_ = prediction_config_.planner.x_hit;
  const auto startup_wall_ns = wall_now_ns();
  session_id_ = declare_parameter<std::string>("session_id", "");
  if (session_id_.empty()) {
    session_id_ = "unspecified-" + std::to_string(startup_wall_ns);
  }
  producer_instance_id_ =
      declare_parameter<std::string>("producer_instance_id", "");
  if (producer_instance_id_.empty()) {
    producer_instance_id_ = default_instance_id(startup_wall_ns);
  }
  packet_topic_ = declare_parameter<std::string>(
      "flight_packet_topic", "/ball/flight_packet");
  const std::string pose_topic =
      declare_parameter<std::string>("input_pose_topic", "/poses");
  const auto gated_pose_topic = declare_parameter<std::string>("gated_pose_topic", "");
  if (gated_pose_topic == pose_topic) throw std::invalid_argument("gated pose output must differ from classifier input");
  ball_pose_index_ = static_cast<int>(
      declare_parameter<int>("ball_pose_index", 0));
  const int input_depth = std::clamp(
      static_cast<int>(declare_parameter<int>("input_qos_depth", 64)), 1, 1024);
  retransmit_delays_ms_ = declare_parameter<std::vector<std::int64_t>>(
      "retransmit_delays_ms", std::vector<std::int64_t>{0, 5, 15});
  if (retransmit_delays_ms_.empty()) retransmit_delays_ms_ = {0};
  std::sort(retransmit_delays_ms_.begin(), retransmit_delays_ms_.end());
  retransmit_delays_ms_.erase(
      std::remove_if(
          retransmit_delays_ms_.begin(), retransmit_delays_ms_.end(),
          [](std::int64_t delay) { return delay < 0; }),
      retransmit_delays_ms_.end());
  retransmit_delays_ms_.erase(
      std::unique(retransmit_delays_ms_.begin(), retransmit_delays_ms_.end()),
      retransmit_delays_ms_.end());
  if (retransmit_delays_ms_.empty()) retransmit_delays_ms_ = {0};
  if (retransmit_delays_ms_.size() > 255) {
    retransmit_delays_ms_.resize(255);
  }

  const std::string audit_path =
      declare_parameter<std::string>("debug_csv_path", "");
  if (!audit_path.empty()) {
    const std::filesystem::path path(audit_path);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
    audit_stream_.open(path, std::ios::out | std::ios::app);
    if (audit_stream_ && audit_stream_.tellp() == 0) {
      audit_stream_
          << "session_id,producer_instance_id,trajectory_epoch,flight_sequence,"
             "payload_hash,event,transmit_index,transmit_count,sample_count,"
             "first_exposure_unix_ns,last_exposure_unix_ns,net_cross_unix_ns,"
             "commit_unix_ns,freeze_wall_unix_ns,publish_wall_unix_ns\n";
    }
  }

  const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(input_depth))
                             .best_effort().durability_volatile();
  const auto packet_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                              .best_effort().durability_volatile();
  packet_publisher_ = create_publisher<hope_msgs::msg::BallFlightPacket>(
      packet_topic_, packet_qos);
  if (!gated_pose_topic.empty()) gated_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      gated_pose_topic, rclcpp::QoS(1).best_effort().durability_volatile());
  // Local display/button intent only, with no robot state subscriptions. Network I/O
  // stays on a separate bounded worker and cannot stall the packetizer.
  const auto status_bind = declare_parameter<std::string>("play_state_status_bind", "127.0.0.1");
  const auto status_port = declare_parameter<int>("play_state_status_port", 8768);
  const auto key_path=declare_parameter<std::string>("play_state_control_key_path", default_local_control_key_path());
  std::string desktop_key;
  try { desktop_key=load_local_control_key(key_path); }
  catch(const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Desktop ball controls unavailable: %s", e.what());
  }
  try { local_status_ = std::make_unique<LocalPlayStateStatus>(status_bind, status_port, producer_instance_id_, desktop_key); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Local ball status/intent unavailable: %s; ball gate remains active", e.what());
  }
  pose_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pose_topic, input_qos,
      std::bind(&FlightPacketizerNode::pose_callback, this,
                std::placeholders::_1));
  const std::string hit_plane_topic = declare_parameter<std::string>(
      "adaptive_hit_plane_topic", "/hope/planner/hit_plane");
  if (!hit_plane_topic.empty()) {
    hit_plane_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        hit_plane_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
          // Schema 1: [schema, publisher wall seconds, foremost contact x].
          // Use the calibrated front bound for both stroke sides. A stale or
          // out-of-order reference cannot relax the scheduling deadline.
          const double now = static_cast<double>(wall_now_ns()) * 1e-9;
          if (message->data.size() != 3 || message->data[0] != 1.0 ||
              !std::isfinite(message->data[1]) || !std::isfinite(message->data[2]) ||
              std::abs(message->data[2]) > 1.0 || message->data[1] <= hit_plane_stamp_s_ ||
              now - message->data[1] > 3.0 || message->data[1] - now > 0.5) return;
          hit_plane_stamp_s_ = message->data[1];
          last_hit_plane_x_ = message->data[2];
          play_state_.set_hit_plane(last_hit_plane_x_);
          trajectory_.set_prediction_x_hit(last_hit_plane_x_);
        });
  }
  publish_play_state();
  retransmit_timer_ = create_wall_timer(1ms, [this] {
    if (local_status_) for (const auto& intent : local_status_->take_intents()) {
      play_state_.local_intent(intent.action);
      local_intent_revision_ = intent.revision;
      sync_play_state();
    }
    play_state_.poll(steady_now_ns());
    sync_play_state();
    publish_play_state();
    const double now = static_cast<double>(wall_now_ns()) * 1e-9;
    if (now - hit_plane_stamp_s_ > 3.0 || hit_plane_stamp_s_ - now > 0.5) {
      hit_plane_stamp_s_ = 0;
      play_state_.clear_hit_plane();
      trajectory_.set_prediction_x_hit(std::max(prediction_config_.planner.x_hit,
                                               last_hit_plane_x_));
    }
    if (!play_state_.incoming_flight()) return;
    const auto update = trajectory_.poll(now);
    if (update.snapshot_ready) prepare_snapshot(update.snapshot, last_frame_id_);
    if (play_state_.may_forward()) retransmit_due();
  });

  RCLCPP_INFO(
      get_logger(),
      "Laptop Flight Packet producer started: input=%s output=%s session=%s "
      "producer=%s window=%.3fs scheduling=%s legacy_net_delay=%.3fs retries=%zu QoS=best_effort/KeepLast(1)",
      pose_topic.c_str(), packet_topic_.c_str(), session_id_.c_str(),
      producer_instance_id_.c_str(), trajectory_config_.estimator_window_s,
      trajectory_config_.adaptive.enabled ? "hit_deadline" : "fixed_post_net",
      trajectory_config_.commit_delay_s, retransmit_delays_ms_.size());
}

void FlightPacketizerNode::pose_callback(
    const geometry_msgs::msg::PoseArray::SharedPtr message) {
  if (ball_pose_index_ < 0 ||
      static_cast<std::size_t>(ball_pose_index_) >= message->poses.size()) {
    play_state_.input_unavailable("ball_pose_missing", steady_now_ns());
    sync_play_state(); publish_play_state();
    return;
  }
  const std::int64_t source_ns =
      static_cast<std::int64_t>(message->header.stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(message->header.stamp.nanosec);
  const auto receipt_wall_ns = wall_now_ns();
  if (source_ns <= 0 || message->header.stamp.nanosec >= 1'000'000'000U ||
      receipt_wall_ns - source_ns > 100'000'000LL || source_ns - receipt_wall_ns > 50'000'000LL) {
    play_state_.input_unavailable("source_timestamp_invalid_or_stale", steady_now_ns());
    sync_play_state(); publish_play_state(); return;
  }
  const auto& pose = message->poses[static_cast<std::size_t>(ball_pose_index_)];
  BallSample sample;
  sample.source_time_ns = source_ns;
  sample.source_time_s = static_cast<double>(source_ns) * 1.0e-9;
  sample.position = Vec3(pose.position.x, pose.position.y, pose.position.z);
  if (!sample.position.allFinite()) {
    play_state_.input_unavailable("nonfinite_position", steady_now_ns());
    sync_play_state(); publish_play_state(); return;
  }
  Eigen::Quaterniond orientation(
      pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
  const double norm = orientation.norm();
  sample.orientation_valid = orientation.coeffs().allFinite() &&
      std::isfinite(norm) && norm >= 0.5 && norm <= 1.5;
  if (sample.orientation_valid) sample.orientation = orientation.normalized();
  sample.receipt_steady_ns = steady_now_ns();
  sample.receipt_wall_ns = receipt_wall_ns;
  sample.sequence = ++sample_sequence_;
  const auto frame = message->header.frame_id.empty() ? "world" : message->header.frame_id;
  if (sample.sequence > 1 && frame != last_frame_id_ &&
      sample.source_time_s > play_state_.latest_time()) play_state_.reset("frame_changed");

  const bool accepted = play_state_.observe(sample);
  const bool new_generation = sync_play_state();
  if (!accepted) { publish_play_state(); return; }
  last_frame_id_ = frame;
  if (play_state_.state() == PlayState::INVALID || play_state_.waiting_for_launch()) {
    publish_play_state(); return;
  }
  if (!play_state_.sample_usable()) { publish_play_state(); return; }
  std::optional<TrajectorySnapshot> seeded_snapshot;
  if (new_generation) {
    // Replay only this admitted shot's candidate samples, preserving the net
    // crossing even when direction/launch confirmation finishes after it.
    for (const auto& seed : play_state_.samples()) {
      state_estimator_->push(seed); state_spin_.push(seed);
      const auto update = trajectory_.observe(seed);
      if (update.snapshot_ready) seeded_snapshot = update.snapshot;
    }
  } else {
    if (play_state_.last_bounce_time() != fit_bounce_time_) {
      state_estimator_->reset(); state_spin_.reset();
      fit_bounce_time_ = play_state_.last_bounce_time();
    }
    state_estimator_->push(sample); state_spin_.push(sample);
  }
  if (sample.source_time_s - forecast_time_ >= 0.020) {
    forecast_time_ = sample.source_time_s;
    auto physics = prediction_config_.future_physics;
    const auto fit = state_estimator_->estimate();
    auto spin = fit.spin_model_enabled ? fit.physics_spin : state_spin_.estimate();
    const auto& contact = prediction_config_.future_contact;
    physics.table_tangential_gain = contact.tangential_gain;
    if (!contact.magnus_enabled) physics.magnus_k = 0.0;
    spin.valid = spin.valid && contact.spin_enabled && spin.coherence >= contact.min_coherence;
    spin.omega_rad_s *= contact.omega_scale;
    play_state_.forecast(fit, physics, spin);
    sync_play_state();
  }
  publish_play_state();
  if (!play_state_.incoming_flight()) return;
  publish_gated_pose(*message, source_ns);
  if (seeded_snapshot) prepare_snapshot(*seeded_snapshot, last_frame_id_);
  if (new_generation) return;

  IncomingTrajectoryUpdate update = trajectory_.observe(sample);
  if (update.incoming_cancelled) {
    RCLCPP_INFO(
        get_logger(),
        "incoming cancelled before commit: confirmed outgoing motion "
        "epoch=%llu source=%.9f x=%.4f",
        static_cast<unsigned long long>(trajectory_.trajectory_epoch()),
        sample.source_time_s, sample.position.x());
  }
  if (update.snapshot_ready) {
    prepare_snapshot(
        update.snapshot,
        message->header.frame_id.empty() ? "world" : message->header.frame_id);
  }
}

void FlightPacketizerNode::publish_gated_pose(
    const geometry_msgs::msg::PoseArray& message, std::int64_t source_ns) {
  if (!play_state_.may_forward() || !gated_pose_publisher_ ||
      source_ns - last_forwarded_pose_ns_ < 6'600'000LL) return;
  // Base poses/heartbeats retain their independent, unchanged path.
  geometry_msgs::msg::PoseArray output;
  output.header = message.header;
  output.poses.push_back(message.poses[static_cast<std::size_t>(ball_pose_index_)]);
  gated_pose_publisher_->publish(output); last_forwarded_pose_ns_ = source_ns;
}

void FlightPacketizerNode::prepare_snapshot(
    const TrajectorySnapshot& snapshot,
    const std::string& frame_id) {
  if (!play_state_.incoming_flight() || snapshot.sample_count == 0 ||
      snapshot.samples[0].source_time_s < play_state_.boundary_time()) return;
  if (trajectory_config_.adaptive.enabled) {
    RCLCPP_INFO(get_logger(), "adaptive commit epoch=%llu reason=%s net_delay=%.6f sample_age=%.6f",
        static_cast<unsigned long long>(snapshot.trajectory_epoch), snapshot.commit_reason.c_str(),
        snapshot.one_shot.commit_source_time_s - snapshot.one_shot.net_cross_source_time_s,
        static_cast<double>(wall_now_ns()) * 1e-9 - snapshot.latest_sample()->source_time_s);
  }
  FlightPacketMetadata metadata;
  metadata.present = true;
  metadata.session_id = session_id_;
  metadata.producer_instance_id = producer_instance_id_;
  metadata.trajectory_epoch = snapshot.trajectory_epoch;
  metadata.flight_sequence = snapshot.one_shot.flight_sequence;
  metadata.frame_id = frame_id;
  metadata.freeze_wall_unix_ns = wall_now_ns();

  hope_msgs::msg::BallFlightPacket packet;
  packet.schema_version = kBallFlightPacketSchemaVersion;
  packet.session_id = metadata.session_id;
  packet.producer_instance_id = metadata.producer_instance_id;
  packet.trajectory_epoch = metadata.trajectory_epoch;
  packet.flight_sequence = metadata.flight_sequence;
  packet.payload_hash_algorithm = kBallFlightPacketHashAlgorithm;
  packet.frame_id = metadata.frame_id;
  packet.segment_boundary_reason = snapshot.segment_boundary_reason;
  packet.net_x = trajectory_config_.net_x;
  packet.post_net_delay_s = trajectory_config_.adaptive.enabled
      ? std::max(0.0, snapshot.one_shot.commit_source_time_s -
                         snapshot.one_shot.net_cross_source_time_s)
      : trajectory_config_.commit_delay_s;
  packet.segment_start_exposure_unix_ns =
      seconds_to_ns(snapshot.segment_start_source_time_s);
  packet.previous_segment_last_exposure_unix_ns =
      seconds_to_ns(snapshot.previous_segment_last_source_time_s);
  packet.net_cross_exposure_unix_ns =
      seconds_to_ns(snapshot.one_shot.net_cross_source_time_s);
  packet.commit_exposure_unix_ns =
      seconds_to_ns(snapshot.one_shot.commit_source_time_s);
  packet.freeze_wall_unix_ns = metadata.freeze_wall_unix_ns;
  packet.publish_wall_unix_ns = metadata.freeze_wall_unix_ns;
  packet.transmit_count = static_cast<std::uint8_t>(
      retransmit_delays_ms_.size());
  packet.samples.reserve(snapshot.sample_count);
  for (std::size_t i = 0; i < snapshot.sample_count; ++i) {
    const BallSample& sample = snapshot.samples[i];
    hope_msgs::msg::BallFlightSample output;
    output.exposure_unix_stamp_ns = sample.source_time_ns != 0
        ? sample.source_time_ns
        : seconds_to_ns(sample.source_time_s);
    output.position.x = sample.position.x();
    output.position.y = sample.position.y();
    output.position.z = sample.position.z();
    output.orientation_valid = sample.orientation_valid;
    if (sample.orientation_valid) {
      output.orientation.w = sample.orientation.w();
      output.orientation.x = sample.orientation.x();
      output.orientation.y = sample.orientation.y();
      output.orientation.z = sample.orientation.z();
    } else {
      output.orientation.w = 1.0;
    }
    packet.samples.push_back(std::move(output));
  }
  packet.payload_hash = flight_packet_message_payload_hash(packet);

  transmit_packet(packet);
}

void FlightPacketizerNode::transmit_packet(const hope_msgs::msg::BallFlightPacket& packet) {
  const auto now = steady_now_ns();
  bool immediate_sent = false;
  std::lock_guard<std::mutex> lock(pending_mutex_);
  for (std::size_t i = 0; i < retransmit_delays_ms_.size(); ++i) {
    if (retransmit_delays_ms_[i] == 0 && !immediate_sent) {
      publish_copy(packet, static_cast<std::uint8_t>(i));
      immediate_sent = true;
    } else {
      PendingTransmit pending;
      pending.message = packet;
      pending.message.transmit_index = static_cast<std::uint8_t>(i);
      pending.due_steady_ns =
          now + retransmit_delays_ms_[i] * 1'000'000LL;
      pending.generation = play_state_.generation();
      pending_transmits_.push_back(std::move(pending));
    }
  }
}

void FlightPacketizerNode::publish_copy(
    const hope_msgs::msg::BallFlightPacket& base,
    std::uint8_t transmit_index) {
  if (!play_state_.may_forward()) return;
  auto output = base;
  output.transmit_index = transmit_index;
  // Transport metadata is per attempt and deliberately excluded from the
  // immutable payload hash.  Keep the freeze timestamp fixed while recording
  // the actual wall-clock send time of every retry.
  output.publish_wall_unix_ns = wall_now_ns();
  packet_publisher_->publish(output);
  write_audit(output, transmit_index == 0 ? "publish" : "retry");
}

void FlightPacketizerNode::retransmit_due() {
  std::deque<PendingTransmit> ready;
  const auto now = steady_now_ns();
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto iterator = pending_transmits_.begin();
    while (iterator != pending_transmits_.end()) {
      if (iterator->due_steady_ns <= now) {
        ready.push_back(std::move(*iterator));
        iterator = pending_transmits_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }
  for (const auto& pending : ready) {
    if (pending.generation == play_state_.generation())
      publish_copy(pending.message, pending.message.transmit_index);
  }
}

bool FlightPacketizerNode::sync_play_state() {
  if (state_generation_ == play_state_.generation()) return false;
  state_generation_ = play_state_.generation();
  trajectory_.reset_phase();
  state_estimator_->reset(); state_spin_.reset();
  fit_bounce_time_ = -1e9;
  forecast_time_ = 0;
  last_forwarded_pose_ns_ = 0;
  { std::lock_guard<std::mutex> lock(pending_mutex_); pending_transmits_.clear(); }
  return true;
}

void FlightPacketizerNode::publish_play_state() {
  // Compare lightweight scalar/string-reference fields before serialization.
  // An unchanged timer tick does no JSON construction or string allocation;
  // the 10 Hz heartbeat reuses the cached text. Changes still publish at once.
  const auto key = std::make_tuple(std::cref(session_id_), play_state_.state(),
      std::cref(play_state_.reason()), std::cref(play_state_.forecast_advisory()),
      play_state_.generation(), play_state_.may_forward(), play_state_.history_known(),
      play_state_.return_confirmed(), play_state_.serve_armed(), play_state_.receive_armed(),
      play_state_.striker_bounces(), play_state_.receiver_bounces(),
      play_state_.local_stopped(), local_intent_revision_, play_state_.observation_missing(),
      std::string_view(play_state_.substate_name()));
  const auto now = steady_now_ns();
  const bool changed = !last_state_key_ || key != *last_state_key_;
  if (!changed && now - state_publish_ns_ < 100'000'000LL) return;
  if (changed) {
    // Strings are bounded local enum/reason/launch identifiers. std::quoted
    // escapes session identity supplied by launch; no raw mocap text is used.
    std::ostringstream out;
    out << "{\"schema\":2,\"session_id\":" << std::quoted(session_id_)
        << ",\"producer_instance_id\":" << std::quoted(producer_instance_id_)
        << ",\"intent_revision\":" << std::quoted(std::to_string(local_intent_revision_))
        << ",\"local_stopped\":" << (play_state_.local_stopped() ? "true" : "false")
        << ",\"state\":" << std::quoted(play_state_name(play_state_.state()))
        << ",\"substate\":" << std::quoted(play_state_.substate_name())
        << ",\"reason\":" << std::quoted(play_state_.reason())
        << ",\"forecast_advisory\":" << std::quoted(play_state_.forecast_advisory())
        << ",\"generation\":" << std::quoted(std::to_string(play_state_.generation()))
        << ",\"forwarding\":" << (play_state_.may_forward() ? "true" : "false")
        << ",\"observation_missing\":" << (play_state_.observation_missing() ? "true" : "false")
        << ",\"history_known\":" << (play_state_.history_known() ? "true" : "false")
        << ",\"return_confirmed\":" << (play_state_.return_confirmed() ? "true" : "false")
        << ",\"serve_armed\":" << (play_state_.serve_armed() ? "true" : "false")
        << ",\"receive_armed\":" << (play_state_.receive_armed() ? "true" : "false")
        << ",\"striker_bounces\":" << play_state_.striker_bounces()
        << ",\"receiver_bounces\":" << play_state_.receiver_bounces() << '}';
    last_state_text_ = out.str();
    last_state_key_ = key;
    RCLCPP_INFO(get_logger(), "ball state %s", last_state_text_.c_str());
  }
  if (local_status_) local_status_->update(last_state_text_);
  state_publish_ns_ = now;
}

void FlightPacketizerNode::write_audit(
    const hope_msgs::msg::BallFlightPacket& message,
    const char* event) {
  if (!audit_stream_) return;
  std::lock_guard<std::mutex> lock(audit_mutex_);
  const std::int64_t first = message.samples.empty()
      ? 0 : message.samples.front().exposure_unix_stamp_ns;
  const std::int64_t last = message.samples.empty()
      ? 0 : message.samples.back().exposure_unix_stamp_ns;
  audit_stream_ << message.session_id << ',' << message.producer_instance_id
                << ',' << message.trajectory_epoch << ','
                << message.flight_sequence << ',' << message.payload_hash << ','
                << event << ',' << static_cast<int>(message.transmit_index) << ','
                << static_cast<int>(message.transmit_count) << ','
                << message.samples.size() << ',' << first << ',' << last << ','
                << message.net_cross_exposure_unix_ns << ','
                << message.commit_exposure_unix_ns << ','
                << message.freeze_wall_unix_ns << ','
                << message.publish_wall_unix_ns << '\n';
  audit_stream_.flush();
}

}  // namespace hope_planner_cpp
