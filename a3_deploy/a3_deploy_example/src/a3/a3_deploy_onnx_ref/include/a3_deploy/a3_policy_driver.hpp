// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 8 Task 8.1
//
// A3PolicyDriver: a 50Hz RT loop (inheriting a3_rt::A3BasedTask from PR 4) that
// feeds a 29-DOF policy from the latest RobotState cached via a backend state
// callback. Output goes through ExpandToBackend() to a 31-DOF RobotCommand and
// is sent through RobotIOBackend::SendCommand. A3Watchdog decides each tick
// whether to run the policy or issue a safe-halt command.
#pragma once

#include "a3_deploy/a3_watchdog.hpp"
#include "a3_rt/a3_based_task.hpp"
#include "a3_rt/a3_rt.hpp"
#include "robot_io/robot_io_backend.hpp"

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace a3_deploy {

// 29-DOF policy signature (PR 9). The RT thread calls this at `policy_hz`
// with:
//   - tick_idx: monotonic counter starting at 0, incremented AFTER every
//     successful call (i.e. only on the Watchdog::Ok branch — safe-halt
//     ticks do NOT advance the counter). This indexes the offline-baked
//     reference-motion stream via A3TokenizerReplay::At(tick_idx).
//   - state: latest synced RobotState (timestamp_ns already matches the
//     clock domain the watchdog uses).
//   - q_des_29_out: 29-float MuJoCo-ordered joint target (body only; neck
//     is added by ExpandToBackend).
using PolicyFn = std::function<void(
    std::uint64_t                   tick_idx,
    const robot_io::RobotState&     state,
    std::array<double, 29>&         q_des_29_out)>;

// Full-command signature for bring-up modes that must bypass the policy
// scatter path, e.g. PASSIVE with kp/kd/tau_ff all zero. Return true when
// command_out is valid and should be published; return false to skip this
// tick without sending any command.
using CommandFn = std::function<bool(
    std::uint64_t                   tick_idx,
    const robot_io::RobotState&     state,
    robot_io::RobotCommand&         command_out)>;

struct A3PolicyDriverOptions {
  double           policy_hz = 50.0;
  WatchdogConfig   watchdog{};
  a3_rt::RtSched   sched{};
  std::int64_t     first_wake_monotonic_ns = 0;
  bool             trigger_on_state = false;
  std::int64_t     trigger_offset_ns = 0;
  std::int64_t     trigger_min_period_ns = 0;
  bool             send_safe_halt_before_first_command = true;
};

class A3PolicyDriver : public a3_rt::A3BasedTask {
 public:
  A3PolicyDriver(robot_io::RobotIOBackend& backend,
                 PolicyFn policy,
                 A3PolicyDriverOptions opt);

  A3PolicyDriver(robot_io::RobotIOBackend& backend,
                 CommandFn command,
                 A3PolicyDriverOptions opt);

  ~A3PolicyDriver() override;

  // Must be called AFTER backend.Init(). Wires state callback and starts the
  // RT thread. Returns false on failure (already running, etc.).
  bool StartDriver();

  // Graceful stop. Joins the RT thread.
  void StopDriver();

  const A3Watchdog& Watchdog() const noexcept { return watchdog_; }

  std::uint64_t PolicyTickCount() const noexcept { return policy_tick_count_.load(std::memory_order_relaxed); }
  std::uint64_t SafeHaltCount()   const noexcept { return safe_halt_count_.load(std::memory_order_relaxed); }
  bool CommandFaultLatched() const noexcept {
    return command_fault_latched_.load(std::memory_order_acquire);
  }

  // Current value of the PR-9 PolicyFn tick counter (== PolicyTickCount).
  // Exposed as a distinct accessor so the test / diagnostic surface is
  // explicit about the contract (monotonic, advances only on Ok ticks).
  std::uint64_t TickIndex() const noexcept { return policy_tick_count_.load(std::memory_order_relaxed); }

 protected:
  void RunOnce() noexcept override;

 private:
  static a3_rt::A3BasedTask::Options BuildBaseOptions_(const A3PolicyDriverOptions& opt);

  void OnBackendState_(const robot_io::RobotState& state) noexcept;
  void EventThreadMain_() noexcept;
  void RunOnceWithState_(std::shared_ptr<const robot_io::RobotState> state) noexcept;
  // Returns false only when the pre-first-command silent-start contract
  // suppresses publication. Backend SendCommand failure handling is R05 and is
  // intentionally unchanged here.
  bool EmitSafeHalt_(const robot_io::RobotState& state) noexcept;

  robot_io::RobotIOBackend& backend_;
  PolicyFn                  policy_;
  CommandFn                 command_;
  A3PolicyDriverOptions     opt_;
  A3Watchdog                watchdog_;

  // Latest RobotState cached from the backend's state callback. Writer: the
  // backend's sync-loop thread (OnBackendState_); reader: the RT thread
  // (RunOnce). We use free-function std::atomic_load / atomic_store on
  // shared_ptr (GCC 11.4 libstdc++ workaround — the std::atomic<shared_ptr>
  // specialisation is not yet available there).
  std::shared_ptr<const robot_io::RobotState> latest_state_;

  std::mutex event_mtx_;
  std::condition_variable event_cv_;
  std::shared_ptr<const robot_io::RobotState> event_state_;
  std::thread event_thread_;
  std::uint64_t event_state_seq_{0};
  std::atomic<bool> event_running_{false};

  std::atomic<std::uint64_t> policy_tick_count_{0};
  std::atomic<std::uint64_t> safe_halt_count_{0};
  // A thrown policy/inference exception or non-finite command is not treated as
  // a one-tick dropout. It latches safe halt until this driver is destroyed and
  // the runner is deliberately restarted. CommandFn false remains the existing
  // skip-this-tick contract and is intentionally not part of this R02/R03 latch.
  std::atomic<bool> command_fault_latched_{false};
  std::atomic<bool> has_sent_command_{false};

  // Reusable buffers — avoid alloc in the RT hot path.
  std::array<double, 29> q_out_buf_{};
  robot_io::RobotCommand cmd_out_{};
};

}  // namespace a3_deploy
