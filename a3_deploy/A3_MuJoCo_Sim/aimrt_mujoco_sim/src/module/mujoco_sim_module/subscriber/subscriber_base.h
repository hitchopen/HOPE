// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/channel/channel_handle.h"

#include "yaml-cpp/yaml.h"

#include "mujoco/mujoco.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {

class SubscriberBase {
 public:
  SubscriberBase() = default;
  virtual ~SubscriberBase() = default;

  SubscriberBase(const SubscriberBase&) = delete;
  SubscriberBase& operator=(const SubscriberBase&) = delete;

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual void Start() = 0;
  virtual void Shutdown() = 0;

  virtual std::string_view Type() const noexcept = 0;

  virtual void SetMj(mjModel* m, mjData* d) = 0;
  virtual void SetSubscriberHandle(aimrt::channel::SubscriberRef subscriber_handle) = 0;

  virtual void ApplyCtrlData() = 0;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber