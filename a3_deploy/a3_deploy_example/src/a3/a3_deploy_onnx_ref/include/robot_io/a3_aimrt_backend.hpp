// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 7 / PR 7a
//
// A3AimrtBackend — RobotIOBackend implementation for the AgiBot A3 robot,
// wiring together A3SyncLoop (PR 3), A3SubscriberManager (PR 5) and
// A3PublisherManager (PR 6). AimRT runtime linkage is fully gated behind the
// ENABLE_A3_AIMRT_BACKEND CMake option (default OFF). With the option OFF
// (or unavailable), the backend still compiles — only the real AimRT
// Initialize / AsyncStart / Subscribe / Publish code is excised. Tests drive
// the pipeline via Inject*_ForTest + SetTestCaptureFn_ForTest hooks.
#pragma once

#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/robot_io_backend.hpp"

#include "a3_deploy/a3_teleop_reference.hpp"
#include "a3_io/publisher_manager.hpp"
#include "a3_io/subscriber_manager.hpp"
#include "a3_sync/a3_ring_buffer.hpp"
#include "a3_sync/a3_sync_loop.hpp"
#include "a3_sync/a3_sync_types.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifdef ENABLE_A3_AIMRT_BACKEND
#include <future>

#include "aimrt_module_cpp_interface/core.h"
#include "core/aimrt_core.h"
#endif

namespace robot_io {

class A3AimrtBackend : public RobotIOBackend {
 public:
  A3AimrtBackend();
  ~A3AimrtBackend() override;

  A3AimrtBackend(const A3AimrtBackend&)            = delete;
  A3AimrtBackend& operator=(const A3AimrtBackend&) = delete;

  // config format (all keys optional except cfg_file_path; comma-separated).
  // The default timing path is fixed 100Hz min_skew_pair sync for a 50Hz policy:
  // pick the newest low-skew joint/IMU pair, auto-calibrate the pick phase from
  // receive timing, then run policy shortly after it.
  //   cfg_file_path=/path/to/a3_deploy_config.yaml
  //
  // Advanced overrides, normally omitted:
  //   phase_ms=<double>           (disables auto phase)
  //   auto_phase=true|false
  //   sync_mode=latest_frame|header_interp|min_skew_pair
  //   sync_hz=<double>
  //   align_delay_ms=<double>
  //   sync_release_margin_ms=<double>  (auto-phase target wait after newest input)
  //   max_skew_ms=<double>
  //   sync_ready_after_input_ms=<double>
  //   max_group_internal_skew_ms=<double>
  //   max_group_pair_skew_ms=<double>
  //   group_pair_search_depth=<int>
  //   max_backtrack=<int>
  //   max_sample_age_ms=<double>  (<=0 disables per-channel freshness)
  //
  // cfg_file_path is REQUIRED when ENABLE_A3_AIMRT_BACKEND=ON; otherwise it is
  // parsed but not enforced (so unit tests can Init without a real file).
  // publish_enabled=false keeps backend-only/probe runs from registering
  // command publishers while still subscribing to the six state streams.
  bool Init(const std::string& config) override;

  bool Start() override;
  void Stop() override;

  const JointLayout& GetLayout() const override;
  void RegisterStateCallback(StateCallback cb) override;
  bool SendCommand(const RobotCommand& cmd) override;
  std::string Name() const override { return "a3"; }
  double StateRateHz() const override;
  std::int64_t SyncPhaseNs() const noexcept { return actual_phase_ns_; }
  a3_sync::A3SyncStatistics::Snapshot SyncStatistics() const;
  a3_sync::A3SyncStatistics::LatencySnapshot ConsumeLatencyStatistics();

  using TeleopFrameCallback =
      std::function<void(const a3_deploy::A3TeleopFrame&)>;
  void SetTeleopFrameCallback(TeleopFrameCallback cb);
  void SetTeleopTopic(std::string topic);

  // LIVE PLANNER inputs (Path B): std_msgs/Float64MultiArray carried over the ros2 backend.
  // A subscriber is registered (in RegisterPubSub_) only when the callback is set — set
  // these before Start(). The callback receives the raw msg.data (decoded by the caller).
  using FlatArrayCallback = std::function<void(const std::vector<double>&)>;
  void SetRacketTargetCallback(FlatArrayCallback cb);
  void SetRacketTopic(std::string topic);
  void SetBasePoseCallback(FlatArrayCallback cb);
  void SetBasePoseTopic(std::string topic);
  void SetBallStateCallback(FlatArrayCallback cb);
  void SetBallStateTopic(std::string topic);

  // Narrow local-Runner operator channel.  The request callback receives only
  // the frozen fixed-action wire; the Runner validates and queues it.  State
  // publishing is independent of body command publishing so --dry-run can
  // still be observed without enabling motor output.
  void SetRunnerControlCallback(FlatArrayCallback cb);
  void SetRunnerControlTopic(std::string topic);
  void SetRunnerStateTopic(std::string topic);
  bool PublishRunnerState(const std::vector<double>& values);

  // ---------- Test hooks (no AimRT required) ----------
  void InjectWaistSample_ForTest(const a3_sync::WaistSample& s);
  void InjectLegSample_ForTest(const a3_sync::LegSample& s);
  void InjectArmSample_ForTest(const a3_sync::ArmSample& s);
  void InjectNeckSample_ForTest(const a3_sync::NeckSample& s);
  void InjectPelvisImu_ForTest(const a3_sync::ImuSample& s);
  void InjectTorsoImu_ForTest(const a3_sync::ImuSample& s);

  using TestCaptureFn = std::function<void(const std::string& topic_suffix,
                                           std::int64_t stamp_ns,
                                           std::uint32_t seq,
                                           const RobotCommand& cmd_31)>;
  void SetTestCaptureFn_ForTest(TestCaptureFn fn);

 private:
  bool ParseConfig_(const std::string& config);
  void CalibrateSyncPhase_();
  void OnSyncState_(const RobotState& state);

#ifdef ENABLE_A3_AIMRT_BACKEND
  bool InitAimrt_();
  bool RegisterPubSub_();
#endif

  // ---------- Config ----------
  std::string        cfg_file_path_;
  a3_sync::SyncConfig sync_cfg_{};
  bool               publish_enabled_{true};
  bool               auto_phase_{true};
  std::int64_t       sync_release_margin_ns_{500'000};
  std::int64_t       actual_phase_ns_{sync_cfg_.phase_ns};

  // ---------- Ring buffers (owned) ----------
  std::unique_ptr<a3_sync::RingBuffer<a3_sync::WaistSample>> waist_ring_;
  std::unique_ptr<a3_sync::RingBuffer<a3_sync::LegSample>>   leg_ring_;
  std::unique_ptr<a3_sync::RingBuffer<a3_sync::ArmSample>>   arm_ring_;
  std::unique_ptr<a3_sync::RingBuffer<a3_sync::NeckSample>>  neck_ring_;
  std::unique_ptr<a3_sync::RingBuffer<a3_sync::ImuSample>>   pelvis_imu_ring_;
  std::unique_ptr<a3_sync::RingBuffer<a3_sync::ImuSample>>   torso_imu_ring_;

  // ---------- Orchestration ----------
  std::unique_ptr<a3_io::A3SubscriberManager> sub_mgr_;
#if defined(ENABLE_A3_AIMRT_BACKEND) && defined(HAS_A3_ROS_MSGS)
  std::unique_ptr<a3_io::A3PublisherManager> pub_mgr_;
#endif
  std::unique_ptr<a3_sync::A3SyncLoop> sync_loop_;

  StateCallback user_cb_{};
  TestCaptureFn test_capture_fn_{};
  TeleopFrameCallback teleop_frame_cb_{};
  std::string teleop_topic_{"/ta/whole_body_command"};
  FlatArrayCallback racket_target_cb_{};
  std::string racket_topic_{"/racket/command_flat"};
  FlatArrayCallback base_pose_cb_{};
  std::string base_pose_topic_{"/a3/base_pose_flat"};
  FlatArrayCallback ball_state_cb_{};
  std::string ball_state_topic_{"/serve/ball_state_flat"};
  FlatArrayCallback runner_control_cb_{};
  std::string runner_control_topic_{"/hope/v17/runner/control_request_flat"};
  std::string runner_state_topic_{"/hope/v17/runner/state_flat"};
  std::mutex runner_state_publish_mutex_;
  std::function<void(const std::vector<double>&)> runner_state_publish_fn_{};

#ifdef ENABLE_A3_AIMRT_BACKEND
  aimrt::runtime::core::AimRTCore          aimrt_core_;
  aimrt::runtime::core::AimRTCore::Options aimrt_options_{};
  aimrt::CoreRef                           module_handle_{};
  std::future<void>                        shutdown_future_{};
#endif

  std::atomic<std::uint32_t> seq_counter_{0};
  std::atomic<bool>          inited_{false};
  std::atomic<bool>          started_{false};
};

}  // namespace robot_io
