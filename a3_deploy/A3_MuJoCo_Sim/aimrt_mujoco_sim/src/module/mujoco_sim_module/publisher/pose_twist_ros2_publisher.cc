// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/pose_twist_ros2_publisher.h"

#include <chrono>
#include <cstring>

#include "mujoco_sim_module/common/xmodel_reader.h"

namespace {
void FillStamp(auto& stamp) {
  const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
  stamp.sec = static_cast<int32_t>(timestamp / 1000000000);
  stamp.nanosec = static_cast<uint32_t>(timestamp % 1000000000);
}
}  // namespace

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher::PoseSensorRos2Publisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher::PoseSensorRos2Publisher::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["bind_framepos"] = rhs.bind_framepos;
    node["bind_framequat"] = rhs.bind_framequat;
    node["frame_id"] = rhs.frame_id;
    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;
    rhs.bind_framepos = node["bind_framepos"].as<std::string>();
    rhs.bind_framequat = node["bind_framequat"].as<std::string>();
    if (node["frame_id"]) rhs.frame_id = node["frame_id"].as<std::string>();
    return true;
  }
};

template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher::TwistSensorRos2Publisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher::TwistSensorRos2Publisher::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["bind_linear_vel"] = rhs.bind_linear_vel;
    node["bind_angular_vel"] = rhs.bind_angular_vel;
    node["frame_id"] = rhs.frame_id;
    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;
    rhs.bind_linear_vel = node["bind_linear_vel"].as<std::string>();
    rhs.bind_angular_vel = node["bind_angular_vel"].as<std::string>();
    if (node["frame_id"]) rhs.frame_id = node["frame_id"].as<std::string>();
    return true;
  }
};

template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher::OdometryRos2Publisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher::OdometryRos2Publisher::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["bind_framepos"] = rhs.bind_framepos;
    node["bind_framequat"] = rhs.bind_framequat;
    node["frame_id"] = rhs.frame_id;
    node["child_frame_id"] = rhs.child_frame_id;
    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;
    rhs.bind_framepos = node["bind_framepos"].as<std::string>();
    rhs.bind_framequat = node["bind_framequat"].as<std::string>();
    if (node["frame_id"]) rhs.frame_id = node["frame_id"].as<std::string>();
    if (node["child_frame_id"]) rhs.child_frame_id = node["child_frame_id"].as<std::string>();
    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

void PoseSensorRos2Publisher::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void PoseSensorRos2Publisher::CopySensorData(int addr, double* dest, size_t n) const {
  if (addr >= 0) std::memcpy(dest, &d_->sensordata[addr], n * sizeof(double));
}

void PoseSensorRos2Publisher::RegisterSensorAddr() {
  sensor_addr_group_.framepos_addr = common::GetSensorIdBySensorName(m_, options_.bind_framepos).value_or(-1);
  sensor_addr_group_.framequat_addr = common::GetSensorIdBySensorName(m_, options_.bind_framequat).value_or(-1);
}

void PoseSensorRos2Publisher::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull()) options_ = options_node.as<Options>();
  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);
  RegisterSensorAddr();
}

void PoseSensorRos2Publisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<geometry_msgs::msg::PoseStamped>(publisher_),
                          "Register publish type failed.");
}

void PoseSensorRos2Publisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;
  if (counter_++ < avg_interval_) return;

  auto state = std::make_unique<SensorStateGroup>();
  CopySensorData(sensor_addr_group_.framepos_addr, state->position, 3);
  CopySensorData(sensor_addr_group_.framequat_addr, state->orientation, 4);

  executor_.Execute([this, state = std::move(state)]() {
    geometry_msgs::msg::PoseStamped msg;
    FillStamp(msg.header.stamp);
    msg.header.frame_id = options_.frame_id;
    msg.pose.position.x = state->position[0];
    msg.pose.position.y = state->position[1];
    msg.pose.position.z = state->position[2];
    msg.pose.orientation.w = state->orientation[0];
    msg.pose.orientation.x = state->orientation[1];
    msg.pose.orientation.y = state->orientation[2];
    msg.pose.orientation.z = state->orientation[3];
    aimrt::channel::Publish(publisher_, msg);
  });

  avg_interval_ += avg_interval_base_;
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

void TwistSensorRos2Publisher::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void TwistSensorRos2Publisher::CopySensorData(int addr, double* dest, size_t n) const {
  if (addr >= 0) std::memcpy(dest, &d_->sensordata[addr], n * sizeof(double));
}

void TwistSensorRos2Publisher::RegisterSensorAddr() {
  sensor_addr_group_.linear_vel_addr = common::GetSensorIdBySensorName(m_, options_.bind_linear_vel).value_or(-1);
  sensor_addr_group_.angular_vel_addr = common::GetSensorIdBySensorName(m_, options_.bind_angular_vel).value_or(-1);
}

void TwistSensorRos2Publisher::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull()) options_ = options_node.as<Options>();
  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);
  RegisterSensorAddr();
}

void TwistSensorRos2Publisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<geometry_msgs::msg::TwistStamped>(publisher_),
                          "Register publish type failed.");
}

void TwistSensorRos2Publisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;
  if (counter_++ < avg_interval_) return;

  auto state = std::make_unique<SensorStateGroup>();
  CopySensorData(sensor_addr_group_.linear_vel_addr, state->linear_velocity, 3);
  CopySensorData(sensor_addr_group_.angular_vel_addr, state->angular_velocity, 3);

  executor_.Execute([this, state = std::move(state)]() {
    geometry_msgs::msg::TwistStamped msg;
    FillStamp(msg.header.stamp);
    msg.header.frame_id = options_.frame_id;
    msg.twist.linear.x = state->linear_velocity[0];
    msg.twist.linear.y = state->linear_velocity[1];
    msg.twist.linear.z = state->linear_velocity[2];
    msg.twist.angular.x = state->angular_velocity[0];
    msg.twist.angular.y = state->angular_velocity[1];
    msg.twist.angular.z = state->angular_velocity[2];
    aimrt::channel::Publish(publisher_, msg);
  });

  avg_interval_ += avg_interval_base_;
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

void OdometryRos2Publisher::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void OdometryRos2Publisher::CopySensorData(int addr, double* dest, size_t n) const {
  if (addr >= 0) std::memcpy(dest, &d_->sensordata[addr], n * sizeof(double));
}

void OdometryRos2Publisher::RegisterSensorAddr() {
  sensor_addr_group_.framepos_addr = common::GetSensorIdBySensorName(m_, options_.bind_framepos).value_or(-1);
  sensor_addr_group_.framequat_addr = common::GetSensorIdBySensorName(m_, options_.bind_framequat).value_or(-1);
}

void OdometryRos2Publisher::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull()) options_ = options_node.as<Options>();
  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);
  RegisterSensorAddr();
}

void OdometryRos2Publisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<tf2_msgs::msg::TFMessage>(publisher_),
                          "Register publish type failed.");
}

void OdometryRos2Publisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;
  if (counter_++ < avg_interval_) return;

  auto state = std::make_unique<SensorStateGroup>();
  CopySensorData(sensor_addr_group_.framepos_addr, state->position, 3);
  CopySensorData(sensor_addr_group_.framequat_addr, state->orientation, 4);

  executor_.Execute([this, state = std::move(state)]() {
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.resize(1);
    FillStamp(msg.transforms[0].header.stamp);
    msg.transforms[0].header.frame_id = options_.frame_id;
    msg.transforms[0].child_frame_id = options_.child_frame_id;
    msg.transforms[0].transform.translation.x = state->position[0];
    msg.transforms[0].transform.translation.y = state->position[1];
    msg.transforms[0].transform.translation.z = state->position[2];
    msg.transforms[0].transform.rotation.w = state->orientation[0];
    msg.transforms[0].transform.rotation.x = state->orientation[1];
    msg.transforms[0].transform.rotation.y = state->orientation[2];
    msg.transforms[0].transform.rotation.z = state->orientation[3];
    aimrt::channel::Publish(publisher_, msg);
  });

  avg_interval_ += avg_interval_base_;
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
