// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/publisher/utils.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

class PoseSensorRos2Publisher : public PublisherBase {
 public:
  struct Options {
    std::string bind_framepos;
    std::string bind_framequat;
    std::string frame_id = "odom";
  };

  PoseSensorRos2Publisher() = default;
  ~PoseSensorRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "pose_sensor_ros2"; }
  void PublishSensorData() override;

  void Start() override {}
  void Shutdown() override {}

  void SetMj(mjModel* m, mjData* d) override;
  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override { publisher_ = publisher_handle; }
  void SetExecutor(aimrt::executor::ExecutorRef executor) override { executor_ = executor; }
  void SetFreq(uint32_t freq) override { channel_frq_ = freq; }

 private:
  struct SensorAddrGroup {
    int32_t framepos_addr = -1;
    int32_t framequat_addr = -1;
  };

  struct SensorStateGroup {
    double position[3] = {0.0, 0.0, 0.0};
    double orientation[4] = {1.0, 0.0, 0.0, 0.0};
  };

  void InitializeBase(YAML::Node options_node);
  void RegisterSensorAddr();
  void CopySensorData(int addr, double* dest, size_t n) const;

  Options options_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;
  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;
  uint32_t counter_ = 0;
  SensorAddrGroup sensor_addr_group_;
};

class TwistSensorRos2Publisher : public PublisherBase {
 public:
  struct Options {
    std::string bind_linear_vel;
    std::string bind_angular_vel;
    std::string frame_id = "odom";
  };

  TwistSensorRos2Publisher() = default;
  ~TwistSensorRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "twist_sensor_ros2"; }
  void PublishSensorData() override;

  void Start() override {}
  void Shutdown() override {}

  void SetMj(mjModel* m, mjData* d) override;
  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override { publisher_ = publisher_handle; }
  void SetExecutor(aimrt::executor::ExecutorRef executor) override { executor_ = executor; }
  void SetFreq(uint32_t freq) override { channel_frq_ = freq; }

 private:
  struct SensorAddrGroup {
    int32_t linear_vel_addr = -1;
    int32_t angular_vel_addr = -1;
  };

  struct SensorStateGroup {
    double linear_velocity[3] = {0.0, 0.0, 0.0};
    double angular_velocity[3] = {0.0, 0.0, 0.0};
  };

  void InitializeBase(YAML::Node options_node);
  void RegisterSensorAddr();
  void CopySensorData(int addr, double* dest, size_t n) const;

  Options options_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;
  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;
  uint32_t counter_ = 0;
  SensorAddrGroup sensor_addr_group_;
};

class OdometryRos2Publisher : public PublisherBase {
 public:
  struct Options {
    std::string bind_framepos;
    std::string bind_framequat;
    std::string frame_id = "odom";
    std::string child_frame_id = "pelvis_link";
  };

  OdometryRos2Publisher() = default;
  ~OdometryRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "odometry_ros2"; }
  void PublishSensorData() override;

  void Start() override {}
  void Shutdown() override {}

  void SetMj(mjModel* m, mjData* d) override;
  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override { publisher_ = publisher_handle; }
  void SetExecutor(aimrt::executor::ExecutorRef executor) override { executor_ = executor; }
  void SetFreq(uint32_t freq) override { channel_frq_ = freq; }

 private:
  struct SensorAddrGroup {
    int32_t framepos_addr = -1;
    int32_t framequat_addr = -1;
  };

  struct SensorStateGroup {
    double position[3] = {0.0, 0.0, 0.0};
    double orientation[4] = {1.0, 0.0, 0.0, 0.0};
  };

  void InitializeBase(YAML::Node options_node);
  void RegisterSensorAddr();
  void CopySensorData(int addr, double* dest, size_t n) const;

  Options options_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;
  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;
  uint32_t counter_ = 0;
  SensorAddrGroup sensor_addr_group_;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
