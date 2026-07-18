// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/publisher/utils.h"

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "imu.pb.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include <sensor_msgs/msg/imu.hpp>
  #include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#endif

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

class ImuSensorPublisherBase : public PublisherBase {
 public:
  struct Options {
    std::string bind_site;
    std::string bind_framequat;
    std::string bind_gyro;
    std::string bind_accelerometer;
  };

 public:
  ImuSensorPublisherBase() = default;
  virtual ~ImuSensorPublisherBase() override = default;

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual std::string_view Type() const noexcept override = 0;
  virtual void PublishSensorData() override = 0;

  void Start() override {}
  void Shutdown() override {}

  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override { publisher_ = publisher_handle; }
  void SetExecutor(aimrt::executor::ExecutorRef executor) override { executor_ = executor; };
  void SetFreq(uint32_t freq) override { channel_frq_ = freq; };
  void SetPublishContext(const PublishContext& context) override { publish_context_ = context; }
  void SetMj(mjModel* m, mjData* d) override;

 protected:
  void InitializeBase(YAML::Node options_node);
  void RegisterSensorAddr();
  void CopySensorData(int addr, auto& dest, size_t n);

 protected:
  struct SensorAddrGroup {
    int32_t framequat_addr;
    int32_t gyro_addr;
    int32_t accelerometer_addr;
  };

  struct SensorStateGroup {
    struct {
      double w, x, y, z;
    } orientation;
    struct {
      double x, y, z;
    } angular_velocity;
    struct {
      double x, y, z;
    } linear_acceleration;
  };

  Options options_;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;

  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;
  size_t imu_num_ = 0;
  uint32_t counter_ = 0;
  PublishContext publish_context_;

  SensorAddrGroup sensor_addr_group_;
};

class ImuSensorPublisher : public ImuSensorPublisherBase {
 public:
  ImuSensorPublisher() = default;
  ~ImuSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "imu_sensor"; }
  void PublishSensorData() override;
};

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
class ImuSensorRos2Publisher : public ImuSensorPublisherBase {
 public:
  ImuSensorRos2Publisher() = default;
  ~ImuSensorRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "imu_sensor_ros2"; }
  void PublishSensorData() override;
};
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
