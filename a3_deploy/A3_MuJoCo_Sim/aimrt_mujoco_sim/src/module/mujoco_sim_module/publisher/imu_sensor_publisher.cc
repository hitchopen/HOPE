// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/imu_sensor_publisher.h"
#include "mujoco_sim_module/common/xmodel_reader.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher::ImuSensorPublisherBase::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher::ImuSensorPublisherBase::Options;
  static Node encode(const Options& rhs) {
    Node node;

    node["bind_site"] = rhs.bind_site;
    node["bind_framequat"] = rhs.bind_framequat;
    node["bind_gyro"] = rhs.bind_gyro;
    node["bind_accelerometer"] = rhs.bind_accelerometer;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.bind_site = node["bind_site"].as<std::string>();

    if (node["bind_framequat"]) {
      rhs.bind_framequat = node["bind_framequat"].as<std::string>();
    }
    if (node["bind_gyro"]) {
      rhs.bind_gyro = node["bind_gyro"].as<std::string>();
    }
    if (node["bind_accelerometer"]) {
      rhs.bind_accelerometer = node["bind_accelerometer"].as<std::string>();
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {
void ImuSensorPublisherBase::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void ImuSensorPublisherBase::RegisterSensorAddr() {
  sensor_addr_group_.framequat_addr = common::GetSensorIdBySensorName(m_, options_.bind_framequat).value_or(-1),
  sensor_addr_group_.gyro_addr = common::GetSensorIdBySensorName(m_, options_.bind_gyro).value_or(-1),
  sensor_addr_group_.accelerometer_addr = common::GetSensorIdBySensorName(m_, options_.bind_accelerometer).value_or(-1);
}

void ImuSensorPublisherBase::CopySensorData(int addr, auto& dest, size_t n) {
  if (addr >= 0) {
    std::memcpy(&dest, &d_->sensordata[addr], n * sizeof(double));
  }
}

void ImuSensorPublisherBase::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();
  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);
  RegisterSensorAddr();
  options_node = options_;
}

void ImuSensorPublisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::ImuState>(publisher_),
                          "Register publish type failed.");
}

void ImuSensorPublisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;

  if (counter_++ < avg_interval_) return;

  auto state_array = std::make_unique<SensorStateGroup>();

  CopySensorData(sensor_addr_group_.framequat_addr, state_array->orientation, 4);
  CopySensorData(sensor_addr_group_.gyro_addr, state_array->angular_velocity, 3);
  CopySensorData(sensor_addr_group_.accelerometer_addr, state_array->linear_acceleration, 3);

  executor_.Execute([this, state_array = std::move(state_array)]() {
    aimrt::protocols::sensor::ImuState state;

    state.mutable_header()->set_time_stamp(std::chrono::system_clock::now().time_since_epoch().count());

    auto* orientation = state.mutable_orientation();
    orientation->set_x(state_array->orientation.x);
    orientation->set_y(state_array->orientation.y);
    orientation->set_z(state_array->orientation.z);
    orientation->set_w(state_array->orientation.w);

    state.mutable_orientation_covariance()->Resize(9, 0.0);

    auto* angular_velocity = state.mutable_angular_velocity();
    angular_velocity->set_x(state_array->angular_velocity.x);
    angular_velocity->set_y(state_array->angular_velocity.y);
    angular_velocity->set_z(state_array->angular_velocity.z);

    state.mutable_angular_velocity_covariance()->Resize(9, 0.0);

    auto* linear_acceleration = state.mutable_linear_acceleration();
    linear_acceleration->set_x(state_array->linear_acceleration.x);
    linear_acceleration->set_y(state_array->linear_acceleration.y);
    linear_acceleration->set_z(state_array->linear_acceleration.z);

    state.mutable_linear_acceleration_covariance()->Resize(9, 0.0);

    aimrt::channel::Publish(publisher_, state);
  });

  avg_interval_ += avg_interval_base_;

  // avoid overflow
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
void ImuSensorRos2Publisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<sensor_msgs::msg::Imu>(publisher_),
                          "Register publish type failed.");
}

void ImuSensorRos2Publisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;
  if (counter_++ < avg_interval_) return;

  const uint64_t timestamp =
      publish_context_.timestamp_ns != 0
          ? publish_context_.timestamp_ns
          : std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
  auto state_array = std::make_unique<SensorStateGroup>();

  CopySensorData(sensor_addr_group_.framequat_addr, state_array->orientation, 4);
  CopySensorData(sensor_addr_group_.gyro_addr, state_array->angular_velocity, 3);
  CopySensorData(sensor_addr_group_.accelerometer_addr, state_array->linear_acceleration, 3);

  executor_.Execute([this, state_array = std::move(state_array), timestamp]() {
    sensor_msgs::msg::Imu state;
    state.header.stamp.sec = static_cast<int32_t>(timestamp / 1000000000);
    state.header.stamp.nanosec = static_cast<uint32_t>(timestamp % static_cast<uint64_t>(1000000000));
    state.header.frame_id = options_.bind_site;
    state.orientation.x = state_array->orientation.x;
    state.orientation.y = state_array->orientation.y;
    state.orientation.z = state_array->orientation.z;
    state.orientation.w = state_array->orientation.w;
    state.angular_velocity.x = state_array->angular_velocity.x;
    state.angular_velocity.y = state_array->angular_velocity.y;
    state.angular_velocity.z = state_array->angular_velocity.z;
    state.linear_acceleration.x = state_array->linear_acceleration.x;
    state.linear_acceleration.y = state_array->linear_acceleration.y;
    state.linear_acceleration.z = state_array->linear_acceleration.z;
    aimrt::channel::Publish(publisher_, state);
  });
  avg_interval_ += avg_interval_base_;
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
