// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <array>
#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
  void InitializeDebugCsv();
  void WriteDebugCsv(std::uint64_t wall_time_ns);
  void InitializeGate3Ball();
  void ApplyGate3BallDrag();
  void UpdateGate3BallContacts();

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

  // Optional plant-side trace, enabled only when A3_MUJOCO_DEBUG_CSV is set.
  // The simulator loop is its sole writer, so this adds no synchronization to
  // the physics path. A3_MUJOCO_DEBUG_STRIDE controls the 1 kHz down-sampling.
  struct DebugActuator {
    std::string name;
    int qpos_addr = -1;
    int dof_addr = -1;
  };
  std::ofstream debug_csv_;
  std::vector<DebugActuator> debug_actuators_;
  std::uint64_t debug_sim_step_ = 0;
  std::uint64_t debug_rows_ = 0;
  std::uint64_t debug_reset_seq_ = 0;
  int debug_stride_ = 5;
  double debug_last_sim_time_ = -1.0;
  int debug_pelvis_body_id_ = -1;
  int debug_torso_body_id_ = -1;
  int debug_left_foot_body_id_ = -1;
  int debug_right_foot_body_id_ = -1;
  int debug_racket_site_id_ = -1;
  std::string debug_pd_mode_ = "explicit";

  int gate3_ball_body_id_ = -1;
  int gate3_ball_joint_id_ = -1;
  int gate3_ball_qpos_addr_ = -1;
  int gate3_ball_dof_addr_ = -1;
  int gate3_ball_geom_id_ = -1;
  int gate3_racket_geom_id_ = -1;
  int gate3_racket_site_id_ = -1;
  int gate3_table_geom_id_ = -1;
  int gate3_net_geom_id_ = -1;
  double gate3_ball_drag_k_ = 0.1261;
  double gate3_ball_restitution_h_ = 0.64;
  double gate3_ball_restitution_v_ = 0.9215;
  std::array<double, 6> gate3_ball_pre_step_velocity_{};
  std::uint64_t gate3_last_shot_id_ = 0;
  bool gate3_racket_contact_prev_ = false;
  bool gate3_table_contact_prev_ = false;
  bool gate3_net_contact_prev_ = false;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module
