// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>

#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_cpp_interface/executor/executor.h"

#include "yaml-cpp/yaml.h"

#include "mujoco/mujoco.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

struct PublishContext {
  uint32_t sequence = 0;
  uint64_t timestamp_ns = 0;
};

class PublisherBase {
 public:
  PublisherBase() = default;
  virtual ~PublisherBase() = default;

  PublisherBase(const PublisherBase&) = delete;
  PublisherBase& operator=(const PublisherBase&) = delete;

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual void Start() = 0;
  virtual void Shutdown() = 0;

  virtual std::string_view Type() const noexcept = 0;

  virtual void SetMj(mjModel* m, mjData* d) = 0;
  virtual void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) = 0;
  virtual void SetExecutor(aimrt::executor::ExecutorRef executor) = 0;
  virtual void SetFreq(uint32_t freq) = 0;
  virtual void SetPublishContext(const PublishContext& context) { (void)context; }

  virtual void PublishSensorData() = 0;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
