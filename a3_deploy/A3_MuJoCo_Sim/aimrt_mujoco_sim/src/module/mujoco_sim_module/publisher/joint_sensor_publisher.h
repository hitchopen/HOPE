// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/publisher/utils.h"

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "joint_state.pb.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include "aimrt_module_ros2_interface/channel/ros2_channel.h"
  #include "aimrt_msgs/msg/joint_state_array.hpp"
  #include "joint_msgs/msg/joint_state.hpp"
#endif

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {
class JointSensorPublisherBase : public PublisherBase {
 public:
  struct Options {
    struct Joint {
      std::string name;
      std::string bind_joint;
    };
    std::vector<Joint> joints;
  };

 public:
  JointSensorPublisherBase() = default;
  virtual ~JointSensorPublisherBase() override = default;

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual std::string_view Type() const noexcept override = 0;
  virtual void PublishSensorData() override = 0;

  void Start() override {}
  void Shutdown() override {}

  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override {
    publisher_ = publisher_handle;
  }

  void SetMj(mjModel* m, mjData* d) override;

  void SetExecutor(aimrt::executor::ExecutorRef executor) override {
    executor_ = executor;
  };

  void SetFreq(uint32_t freq) override {
    channel_frq_ = freq;
  };
  void SetPublishContext(const PublishContext& context) override {
    publish_context_ = context;
  }

 protected:
  void RegisterSensorAddr();
  void InitializeBase(YAML::Node options_node);

 protected:
  struct SensorAddrGroup {
    int32_t jointpos_addr = -1;
    int32_t jointvel_addr = -1;
    int32_t jointactuatorfrc_addr = -1;
  };

  struct SensorStateGroup {
    double jointpos_state = 0.0;
    double jointvel_state = 0.0;
    double jointactuatorfrc_state = 0.0;
  };

  Options options_;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;

  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;

  uint32_t joint_num_ = 0;
  uint32_t counter_ = 0;
  PublishContext publish_context_;

  std::vector<SensorAddrGroup> sensor_addr_vec_;
  std::vector<std::string> name_vec_;
};

class JointSensorPublisher : public JointSensorPublisherBase {
 public:
  JointSensorPublisher() = default;
  ~JointSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  void PublishSensorData() override;

  std::string_view Type() const noexcept override { return "joint_sensor"; }
};

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
class JointSensorRos2Publisher : public JointSensorPublisherBase {
 public:
  JointSensorRos2Publisher() = default;
  ~JointSensorRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  void PublishSensorData() override;

  std::string_view Type() const noexcept override { return "joint_sensor_ros2"; }
};

class BodyDriveJointSensorPublisher : public JointSensorPublisherBase {
 public:
  BodyDriveJointSensorPublisher() = default;
  ~BodyDriveJointSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  void PublishSensorData() override;

  std::string_view Type() const noexcept override { return "body_drive_joint_sensor"; }
};
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
