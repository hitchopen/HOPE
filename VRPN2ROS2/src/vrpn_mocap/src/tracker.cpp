// MIT License
//
// Copyright (c) 2022 Alvin Sun
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "vrpn_mocap/tracker.hpp"
#include "vrpn_mocap/vrpn_timestamp.h"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>

namespace vrpn_mocap
{

  using geometry_msgs::msg::AccelStamped;
  using geometry_msgs::msg::PoseStamped;
  using geometry_msgs::msg::TwistStamped;
  using namespace std::chrono_literals;

  std::string Tracker::ValidNodeName(const std::string &tracker_name)
  {
    // replace non alphanum characters with _
    const std::string alnum_name = std::regex_replace(tracker_name, std::regex("[^a-zA-Z0-9_]"), "_");
    // strip consecutive underscores
    const std::string node_name = std::regex_replace(alnum_name, std::regex("_+"), "_");

    return node_name;
  }

  Tracker::Tracker(const std::string &tracker_name)
      : Node(ValidNodeName(tracker_name)),
        name_(tracker_name),
        multi_sensor_(declare_parameter("multi_sensor", false)),
        frame_id_(declare_parameter("frame_id", "world")),
        sensor_data_qos_(declare_parameter("sensor_data_qos", true)),
        use_vrpn_timestamps_(declare_parameter("use_vrpn_timestamps", true)),
        validate_vrpn_timestamps_(declare_parameter("validate_vrpn_timestamps", true)),
        max_vrpn_timestamp_age_ms_(declare_parameter("max_vrpn_timestamp_age_ms", 100.0)),
        max_vrpn_future_skew_ms_(declare_parameter("max_vrpn_future_skew_ms", 5.0)),
        min_age_monitor_window_ms_(declare_parameter("min_age_monitor_window_ms", 5000.0)),
        min_age_monitor_warmup_samples_(
            declare_parameter<int64_t>("min_age_monitor_warmup_samples", 100)),
        max_vrpn_min_age_shift_ms_(
            declare_parameter("max_vrpn_min_age_shift_ms", 5.0)),
        validate_expected_vrpn_min_age_(
            declare_parameter("validate_expected_vrpn_min_age", false)),
        expected_vrpn_min_age_ms_(
            declare_parameter("expected_vrpn_min_age_ms", 0.0)),
        max_expected_vrpn_min_age_error_ms_(
            declare_parameter("max_expected_vrpn_min_age_error_ms", 5.0)),
        vrpn_tracker_(name_.c_str())
  {
    Init();

    // start main loop when instantiated as a standalone node
    const double update_freq = this->declare_parameter("update_freq", 500.);
    timer_ = this->create_wall_timer(1s / update_freq, std::bind(&Tracker::MainLoop, this));
  }

  Tracker::Tracker(
      const rclcpp::Node &base_node, const std::string &tracker_name,
      const std::shared_ptr<vrpn_Connection> &connection)
      : Node(base_node, ValidNodeName(tracker_name)),
        name_(tracker_name),
        multi_sensor_(base_node.get_parameter("multi_sensor").as_bool()),
        frame_id_(base_node.get_parameter("frame_id").as_string()),
        sensor_data_qos_(base_node.get_parameter("sensor_data_qos").as_bool()),
        use_vrpn_timestamps_(base_node.get_parameter("use_vrpn_timestamps").as_bool()),
        validate_vrpn_timestamps_(
            base_node.get_parameter("validate_vrpn_timestamps").as_bool()),
        max_vrpn_timestamp_age_ms_(
            base_node.get_parameter("max_vrpn_timestamp_age_ms").as_double()),
        max_vrpn_future_skew_ms_(
            base_node.get_parameter("max_vrpn_future_skew_ms").as_double()),
        min_age_monitor_window_ms_(
            base_node.get_parameter("min_age_monitor_window_ms").as_double()),
        min_age_monitor_warmup_samples_(
            base_node.get_parameter("min_age_monitor_warmup_samples").as_int()),
        max_vrpn_min_age_shift_ms_(
            base_node.get_parameter("max_vrpn_min_age_shift_ms").as_double()),
        validate_expected_vrpn_min_age_(
            base_node.get_parameter("validate_expected_vrpn_min_age").as_bool()),
        expected_vrpn_min_age_ms_(
            base_node.get_parameter("expected_vrpn_min_age_ms").as_double()),
        max_expected_vrpn_min_age_error_ms_(
            base_node.get_parameter("max_expected_vrpn_min_age_error_ms").as_double()),
        vrpn_tracker_(name_.c_str(), connection.get())
  {
    Init();
  }

  Tracker::~Tracker()
  {
    vrpn_tracker_.unregister_change_handler(this, &Tracker::HandlePose);
    vrpn_tracker_.unregister_change_handler(this, &Tracker::HandleTwist);
    vrpn_tracker_.unregister_change_handler(this, &Tracker::HandleAccel);

    RCLCPP_INFO_STREAM(this->get_logger(), "Destroyed new tracker " << name_);
  }

  void Tracker::Init()
  {
    constexpr double kLargestConvertibleMilliseconds =
        static_cast<double>(std::numeric_limits<int64_t>::max()) / 1000000.0;
    if (!std::isfinite(max_vrpn_timestamp_age_ms_) ||
        !std::isfinite(max_vrpn_future_skew_ms_) ||
        !std::isfinite(min_age_monitor_window_ms_) ||
        !std::isfinite(max_vrpn_min_age_shift_ms_) ||
        !std::isfinite(expected_vrpn_min_age_ms_) ||
        !std::isfinite(max_expected_vrpn_min_age_error_ms_) ||
        max_vrpn_timestamp_age_ms_ < 0.0 ||
        max_vrpn_future_skew_ms_ < 0.0 ||
        min_age_monitor_window_ms_ <= 0.0 ||
        min_age_monitor_warmup_samples_ <= 0 ||
        max_vrpn_min_age_shift_ms_ < 0.0 ||
        max_expected_vrpn_min_age_error_ms_ < 0.0 ||
        max_vrpn_timestamp_age_ms_ > kLargestConvertibleMilliseconds ||
        max_vrpn_future_skew_ms_ > kLargestConvertibleMilliseconds ||
        min_age_monitor_window_ms_ > kLargestConvertibleMilliseconds ||
        max_vrpn_min_age_shift_ms_ > kLargestConvertibleMilliseconds ||
        std::abs(expected_vrpn_min_age_ms_) > kLargestConvertibleMilliseconds ||
        max_expected_vrpn_min_age_error_ms_ > kLargestConvertibleMilliseconds)
    {
      throw std::invalid_argument(
          "VRPN timestamp and minimum-age monitor parameters are invalid");
    }

    min_age_monitor_.reset(new detail::VrpnMinAgeMonitor(
        static_cast<int64_t>(min_age_monitor_window_ms_ * 1000000.0),
        static_cast<std::size_t>(min_age_monitor_warmup_samples_),
        static_cast<int64_t>(max_vrpn_min_age_shift_ms_ * 1000000.0),
        validate_expected_vrpn_min_age_,
        static_cast<int64_t>(expected_vrpn_min_age_ms_ * 1000000.0),
        static_cast<int64_t>(max_expected_vrpn_min_age_error_ms_ * 1000000.0)));

    vrpn_tracker_.register_change_handler(this, &Tracker::HandlePose);
    vrpn_tracker_.register_change_handler(this, &Tracker::HandleTwist);
    vrpn_tracker_.register_change_handler(this, &Tracker::HandleAccel);
    vrpn_tracker_.shutup = true;

    RCLCPP_INFO_STREAM(this->get_logger(), "Created new tracker " << name_);
    if (use_vrpn_timestamps_)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Using VRPN server timeval as ROS absolute time (strict validation=%s, "
          "max_age=%.3f ms, max_future=%.3f ms, min-age window=%.0f ms, "
          "max min-age shift=%.3f ms, expected-min validation=%s). "
          "Camera-exposure provenance "
          "must be verified against the mocap vendor/SDK.",
          validate_vrpn_timestamps_ ? "true" : "false",
          max_vrpn_timestamp_age_ms_, max_vrpn_future_skew_ms_,
          min_age_monitor_window_ms_, max_vrpn_min_age_shift_ms_,
          validate_expected_vrpn_min_age_ ? "true" : "false");
      if (!validate_expected_vrpn_min_age_)
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Expected VRPN minimum-age validation is disabled. A static server "
            "clock offset present at adapter startup cannot be detected; this "
            "configuration is for bring-up, not competition acceptance.");
      }
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Using ROS receipt timestamps; these include VRPN network/polling latency.");
    }
  }

  void Tracker::MainLoop() { vrpn_tracker_.mainloop(); }

  bool Tracker::ResolveStamp(const timeval &source_time, rclcpp::Time *stamp)
  {
    if (!use_vrpn_timestamps_)
    {
      // Receipt time must remain wall-clock Unix time even if /use_sim_time is
      // enabled on this node.
      *stamp = system_clock_.now();
      return true;
    }

    const int64_t seconds = static_cast<int64_t>(source_time.tv_sec);
    const int64_t microseconds = static_cast<int64_t>(source_time.tv_usec);
    const rclcpp::Time system_now = system_clock_.now();
    const auto validation = detail::ValidateVrpnTimestamp(
        seconds, microseconds, system_now.nanoseconds(),
        static_cast<int64_t>(max_vrpn_timestamp_age_ms_ * 1000000.0),
        static_cast<int64_t>(max_vrpn_future_skew_ms_ * 1000000.0));

    const bool structurally_invalid =
        validation.status != detail::VrpnTimestampStatus::kOk &&
        validation.status != detail::VrpnTimestampStatus::kTooOld &&
        validation.status != detail::VrpnTimestampStatus::kTooFarInFuture;
    if (structurally_invalid || (validate_vrpn_timestamps_ && !validation.ok()))
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), system_clock_, 2000,
          "Dropping VRPN sample with invalid absolute timestamp: %s "
          "(server=%lld.%06lld, local_system_now_ns=%lld, age_ms=%.3f). "
          "Verify the VRPN server and adapter clocks use the same NTP epoch.",
          detail::VrpnTimestampStatusName(validation.status),
          static_cast<long long>(seconds),
          static_cast<long long>(microseconds),
          static_cast<long long>(system_now.nanoseconds()),
          static_cast<double>(validation.age_ns) / 1000000.0);
      return false;
    }

    // Use steady time only to age the sliding window. The age itself compares
    // the source Unix stamp with RCL_SYSTEM_TIME above.
    const int64_t steady_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto min_age = min_age_monitor_->Observe(steady_now_ns, validation.age_ns);
    if (min_age.ready && !min_age_reference_logged_)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "VRPN minimum-age monitor established runtime reference %.3f ms "
          "(window %.0f ms, samples >= %lld). This detects changes only; "
          "enable expected-min validation to detect a startup-static offset.",
          static_cast<double>(min_age.reference_min_age_ns) / 1000000.0,
          min_age_monitor_window_ms_,
          static_cast<long long>(min_age_monitor_warmup_samples_));
      min_age_reference_logged_ = true;
    }
    if (!min_age.acceptable())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), system_clock_, 2000,
          "%s VRPN sample: minimum-age proxy is outside its trusted regime "
          "(current_min=%.3f ms, runtime_reference=%.3f ms, shift=%.3f ms, "
          "expected_check=%s, expected=%.3f ms, expected_error=%.3f ms). "
          "This proxy combines server clock offset and one-way delay; verify "
          "both hosts' NTP health and the wired network.",
          validate_vrpn_timestamps_ ? "Dropping" : "Publishing",
          static_cast<double>(min_age.current_min_age_ns) / 1000000.0,
          static_cast<double>(min_age.reference_min_age_ns) / 1000000.0,
          static_cast<double>(min_age.shift_ns) / 1000000.0,
          validate_expected_vrpn_min_age_ ? "enabled" : "disabled",
          expected_vrpn_min_age_ms_,
          static_cast<double>(min_age.expected_error_ns) / 1000000.0);
      if (validate_vrpn_timestamps_)
      {
        return false;
      }
    }

    *stamp = rclcpp::Time(validation.stamp_ns, RCL_SYSTEM_TIME);
    return true;
  }

  void VRPN_CALLBACK Tracker::HandlePose(void *data, const vrpn_TRACKERCB tracker_pose)
  {
    Tracker *tracker = static_cast<Tracker *>(data);

    rclcpp::Time stamp;
    if (!tracker->ResolveStamp(tracker_pose.msg_time, &stamp))
    {
      return;
    }

    // Create the topic only after this tracker has produced an accepted frame.
    auto pub = tracker->GetOrCreatePublisher<PoseStamped>(
        static_cast<size_t>(tracker_pose.sensor), "pose", &tracker->pose_pubs_);

    PoseStamped msg;
    msg.header.frame_id = tracker->frame_id_;
    msg.header.stamp = stamp;

    msg.pose.position.x = tracker_pose.pos[0];
    msg.pose.position.y = tracker_pose.pos[1];
    msg.pose.position.z = tracker_pose.pos[2];

    msg.pose.orientation.x = tracker_pose.quat[0];
    msg.pose.orientation.y = tracker_pose.quat[1];
    msg.pose.orientation.z = tracker_pose.quat[2];
    msg.pose.orientation.w = tracker_pose.quat[3];

    pub->publish(msg);
  }

  void VRPN_CALLBACK Tracker::HandleTwist(void *data, const vrpn_TRACKERVELCB tracker_twist)
  {
    Tracker *tracker = static_cast<Tracker *>(data);

    if (!std::isfinite(tracker_twist.vel_quat_dt) || tracker_twist.vel_quat_dt <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
          tracker->get_logger(), tracker->system_clock_, 2000,
          "Dropping VRPN velocity sample with invalid vel_quat_dt=%g",
          tracker_twist.vel_quat_dt);
      return;
    }

    rclcpp::Time stamp;
    if (!tracker->ResolveStamp(tracker_twist.msg_time, &stamp))
    {
      return;
    }

    auto pub = tracker->GetOrCreatePublisher<TwistStamped>(
        static_cast<size_t>(tracker_twist.sensor), "velocity", &tracker->twist_pubs_);

    TwistStamped msg;
    msg.header.frame_id = tracker->frame_id_;
    msg.header.stamp = stamp;

    msg.twist.linear.x = tracker_twist.vel[0];
    msg.twist.linear.y = tracker_twist.vel[1];
    msg.twist.linear.z = tracker_twist.vel[2];

    const Eigen::Quaterniond quat(
        tracker_twist.vel_quat[3], tracker_twist.vel_quat[0], tracker_twist.vel_quat[1],
        tracker_twist.vel_quat[2]);
    const Eigen::AngleAxisd axis_ang(quat);
    const Eigen::Vector3d rot_vel = axis_ang.axis() * axis_ang.angle() / tracker_twist.vel_quat_dt;
    msg.twist.angular.x = rot_vel.x();
    msg.twist.angular.y = rot_vel.y();
    msg.twist.angular.z = rot_vel.z();

    pub->publish(msg);
  }

  void VRPN_CALLBACK Tracker::HandleAccel(void *data, const vrpn_TRACKERACCCB tracker_accel)
  {
    Tracker *tracker = static_cast<Tracker *>(data);

    if (!std::isfinite(tracker_accel.acc_quat_dt) || tracker_accel.acc_quat_dt <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
          tracker->get_logger(), tracker->system_clock_, 2000,
          "Dropping VRPN acceleration sample with invalid acc_quat_dt=%g",
          tracker_accel.acc_quat_dt);
      return;
    }

    rclcpp::Time stamp;
    if (!tracker->ResolveStamp(tracker_accel.msg_time, &stamp))
    {
      return;
    }

    auto pub = tracker->GetOrCreatePublisher<AccelStamped>(
        static_cast<size_t>(tracker_accel.sensor), "accel", &tracker->accel_pubs_);

    AccelStamped msg;
    msg.header.frame_id = tracker->frame_id_;
    msg.header.stamp = stamp;

    msg.accel.linear.x = tracker_accel.acc[0];
    msg.accel.linear.y = tracker_accel.acc[1];
    msg.accel.linear.z = tracker_accel.acc[2];

    const Eigen::Quaterniond quat(
        tracker_accel.acc_quat[3], tracker_accel.acc_quat[0], tracker_accel.acc_quat[1],
        tracker_accel.acc_quat[2]);
    const Eigen::AngleAxisd axis_ang(quat);
    const Eigen::Vector3d rot_acc = axis_ang.axis() * axis_ang.angle() / tracker_accel.acc_quat_dt;
    msg.accel.angular.x = rot_acc.x();
    msg.accel.angular.y = rot_acc.y();
    msg.accel.angular.z = rot_acc.z();

    pub->publish(msg);
  }

} // namespace vrpn_mocap
