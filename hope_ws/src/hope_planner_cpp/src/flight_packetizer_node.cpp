#include "hope_planner_cpp/flight_packetizer_node.hpp"

#include "hope_planner_cpp/flight_packet.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
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
  config.commit_delay_s =
      node.declare_parameter<double>("post_net_commit_delay_s", 0.05);
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
      trajectory_(trajectory_config_) {
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
  pose_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pose_topic, input_qos,
      std::bind(&FlightPacketizerNode::pose_callback, this,
                std::placeholders::_1));
  retransmit_timer_ = create_wall_timer(1ms, [this] { retransmit_due(); });

  RCLCPP_INFO(
      get_logger(),
      "Laptop Flight Packet producer started: input=%s output=%s session=%s "
      "producer=%s window=%.3fs net+%.3fs retries=%zu QoS=best_effort/KeepLast(1)",
      pose_topic.c_str(), packet_topic_.c_str(), session_id_.c_str(),
      producer_instance_id_.c_str(), trajectory_config_.estimator_window_s,
      trajectory_config_.commit_delay_s, retransmit_delays_ms_.size());
}

void FlightPacketizerNode::pose_callback(
    const geometry_msgs::msg::PoseArray::SharedPtr message) {
  if (ball_pose_index_ < 0 ||
      static_cast<std::size_t>(ball_pose_index_) >= message->poses.size()) {
    return;
  }
  const std::int64_t source_ns =
      static_cast<std::int64_t>(message->header.stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(message->header.stamp.nanosec);
  if (source_ns <= 0 || source_ns <= last_source_stamp_ns_.load()) return;
  const auto& pose = message->poses[static_cast<std::size_t>(ball_pose_index_)];
  BallSample sample;
  sample.source_time_ns = source_ns;
  sample.source_time_s = static_cast<double>(source_ns) * 1.0e-9;
  sample.position = Vec3(pose.position.x, pose.position.y, pose.position.z);
  if (!sample.position.allFinite()) return;
  Eigen::Quaterniond orientation(
      pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
  const double norm = orientation.norm();
  sample.orientation_valid = orientation.coeffs().allFinite() &&
      std::isfinite(norm) && norm >= 0.5 && norm <= 1.5;
  if (sample.orientation_valid) sample.orientation = orientation.normalized();
  sample.receipt_steady_ns = steady_now_ns();
  sample.receipt_wall_ns = wall_now_ns();
  sample.sequence = ++sample_sequence_;
  last_source_stamp_ns_.store(source_ns);

  IncomingTrajectoryUpdate update = trajectory_.observe(sample);
  if (update.snapshot_ready) {
    publish_snapshot(
        update.snapshot,
        message->header.frame_id.empty() ? "world" : message->header.frame_id);
  }
}

void FlightPacketizerNode::publish_snapshot(
    const TrajectorySnapshot& snapshot,
    const std::string& frame_id) {
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
  packet.post_net_delay_s = trajectory_config_.commit_delay_s;
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
      pending_transmits_.push_back(std::move(pending));
    }
  }
}

void FlightPacketizerNode::publish_copy(
    const hope_msgs::msg::BallFlightPacket& base,
    std::uint8_t transmit_index) {
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
    publish_copy(pending.message, pending.message.transmit_index);
  }
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
