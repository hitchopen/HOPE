// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/publisher/utils.h"
#include "touch_sensor_state.pb.h"

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "mujoco_sim_module/global.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include "aimrt_module_ros2_interface/channel/ros2_channel.h"
  #include "aimrt_msgs/msg/touch_sensor_state_array.hpp"
#endif

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {
class TouchSensorPublisherBase : public PublisherBase {
 public:
  struct Options {
    std::vector<std::string> names;

    struct State {
      std::string bind_site;
      std::string bind_touch_sensor;
    };
    std::vector<std::vector<State>> states;
  };

 public:
  TouchSensorPublisherBase() = default;
  virtual ~TouchSensorPublisherBase() override = default;

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual std::string_view Type() const noexcept override = 0;
  virtual void PublishSensorData() override = 0;

  void Start() override {}
  void Shutdown() override {}

  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override { publisher_ = publisher_handle; }

  void SetExecutor(aimrt::executor::ExecutorRef executor) override { executor_ = executor; };

  void SetFreq(uint32_t freq) override { channel_frq_ = freq; };

  void SetMj(mjModel* m, mjData* d) override;

 protected:
  void RegisterSensorAddr();
  void InitializeBase(YAML::Node options_node);

 protected:
  struct SensorAddrGroup {
    std::vector<int32_t> addr_vec;
  };

  struct SensorStateGroup {
    std::vector<double> state_vec;
  };

  Options options_;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;

  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;

  uint32_t touch_sensor_group_num_ = 0;
  uint32_t counter_ = 0;

  std::vector<SensorAddrGroup> sensor_addr_group_vec_;
  std::vector<std::string> name_vec_;
  std::vector<uint32_t> touch_sensor_num_vec_;
};

class TouchSensorPublisher : public TouchSensorPublisherBase {
 public:
  TouchSensorPublisher() = default;
  ~TouchSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "touch_sensor"; }
  void PublishSensorData() override;
};

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
class TouchSensorRos2Publisher : public TouchSensorPublisherBase {
 public:
  TouchSensorRos2Publisher() = default;
  ~TouchSensorRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "touch_sensor_ros2"; }
  void PublishSensorData() override;
};
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher