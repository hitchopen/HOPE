// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/subscriber/subscriber_base.h"

#include "glfw_adapter.h"
#include "mujoco/mujoco.h"
#include "simulate.h"
#include "yaml-cpp/yaml.h"

namespace aimrt_mujoco_sim::mujoco_sim_module {

class MujocoSimModule : public aimrt::ModuleBase {
 public:
  struct Options {
    std::string simulation_model_path;
    std::string sim_executor;
    std::string gui_executor;
    std::string default_free_camera_focus_body;
    std::string default_tracking_camera_body;
    double default_camera_distance = 0.0;

    struct SubscriberOption {
      std::string topic;
      std::string type;
      YAML::Node options;
    };
    std::vector<SubscriberOption> subscriber_options;

    struct PublisherOption {
      std::string topic;
      uint32_t frequency;
      std::string executor;
      std::string type;
      YAML::Node options;
    };
    std::vector<PublisherOption> publisher_options;
  };

 public:
  MujocoSimModule() = default;
  ~MujocoSimModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "MujocoSimModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  void RegisterSubscriberGenFunc();
  void RegisterPublisherGenFunc();
  void ApplyDefaultCameraFocus();
  void UpdateDefaultCameraFollowLocked();

  aimrt::co::Task<void> GuiLoop();
  aimrt::co::Task<void> SimLoop();

 private:
  aimrt::CoreRef core_;

  Options options_;

  aimrt::executor::ExecutorRef gui_executor_;
  aimrt::executor::ExecutorRef sim_executor_;

  std::shared_ptr<mujoco::Simulate> sim_;
  std::mutex sim_lifecycle_mutex_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  aimrt::co::AsyncScope scope_;
  std::atomic_bool run_flag_ = true;
  std::atomic_bool sim_loop_exited_ = true;

  // key:type
  using SubscriberGenFunc = std::function<std::unique_ptr<subscriber::SubscriberBase>()>;
  std::unordered_map<std::string, SubscriberGenFunc> subscriber_gen_func_map_;

  using PublisherGenFunc = std::function<std::unique_ptr<publisher::PublisherBase>()>;
  std::unordered_map<std::string, PublisherGenFunc> publisher_gen_func_map_;

  // key:topic
  std::unordered_map<std::string, std::unique_ptr<subscriber::SubscriberBase>> subscriber_map_;
  std::unordered_map<std::string, std::unique_ptr<publisher::PublisherBase>> publisher_map_;
  uint32_t publish_sequence_ = 0;
  int default_camera_follow_body_id_ = -1;
  mjtNum default_camera_follow_last_pos_[3] = {0.0, 0.0, 0.0};
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module
