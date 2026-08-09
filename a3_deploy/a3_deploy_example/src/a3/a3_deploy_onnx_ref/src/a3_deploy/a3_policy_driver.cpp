// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 8 Task 8.1
#include "a3_deploy/a3_policy_driver.hpp"

#include "a3_deploy/expand_to_backend.hpp"
#include "a3_deploy/numeric_safety.hpp"
#include "a3_deploy/safe_halt.hpp"
#include "a3_policy_parameters.hpp"
#include "robot_io/a3_layout_extra.hpp"

#include <sys/prctl.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <utility>

namespace a3_deploy {

namespace {

// NOTE: a3_sync::A3SyncLoop stamps RobotState.timestamp_ns using
// std::chrono::system_clock (see a3_sync_loop.cpp:27), so the watchdog
// must compare ages against the *same* clock. Using steady_clock here
// would mix epochs (wall-clock ~1.7e18 vs monotonic ~1e13) and the age
// check would be meaningless.
std::int64_t NowSystemNs() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

void PreSizeCommandBuffer(robot_io::RobotCommand& cmd) {
  constexpr int N = robot_io::kA3Dof;
  cmd.q_des  = Eigen::VectorXd::Zero(N);
  cmd.dq_des = Eigen::VectorXd::Zero(N);
  cmd.tau_ff = Eigen::VectorXd::Zero(N);
  cmd.kp     = Eigen::VectorXd::Zero(N);
  cmd.kd     = Eigen::VectorXd::Zero(N);
}

bool IsFiniteCommand(const robot_io::RobotCommand& cmd) noexcept {
  constexpr int N = robot_io::kA3Dof;
  const auto finite = [](const Eigen::VectorXd& values) noexcept {
    for (Eigen::Index i = 0; i < values.size(); ++i) {
      if (!numeric_safety::IsFinite(values[i])) return false;
    }
    return true;
  };
  return cmd.q_des.size() == N && finite(cmd.q_des) &&
         cmd.dq_des.size() == N && finite(cmd.dq_des) &&
         cmd.tau_ff.size() == N && finite(cmd.tau_ff) &&
         cmd.kp.size() == N && finite(cmd.kp) &&
         cmd.kd.size() == N && finite(cmd.kd);
}

}  // namespace

// ---------------------------------------------------------------------------
a3_rt::A3BasedTask::Options A3PolicyDriver::BuildBaseOptions_(
    const A3PolicyDriverOptions& opt) {
  a3_rt::A3BasedTask::Options b;
  b.name = "a3_policy";
  const double hz = (opt.policy_hz > 0.0) ? opt.policy_hz : 50.0;
  b.period_ns = static_cast<std::int64_t>(1.0e9 / hz);
  b.first_wake_monotonic_ns = opt.first_wake_monotonic_ns;
  b.sched = opt.sched;
  return b;
}

// ---------------------------------------------------------------------------
A3PolicyDriver::A3PolicyDriver(robot_io::RobotIOBackend& backend,
                               PolicyFn policy,
                               A3PolicyDriverOptions opt)
    : a3_rt::A3BasedTask(BuildBaseOptions_(opt)),
      backend_(backend),
      policy_(std::move(policy)),
      opt_(opt),
      watchdog_(opt.watchdog) {
  // Pre-size the reusable command buffer so RunOnce is alloc-free from tick 1.
  PreSizeCommandBuffer(cmd_out_);
  command_ = [this](std::uint64_t tick_idx,
                    const robot_io::RobotState& state,
                    robot_io::RobotCommand& command_out) {
    if (policy_) {
      policy_(tick_idx, state, q_out_buf_);
    } else {
      q_out_buf_.fill(0.0);
    }
    ExpandToBackend(q_out_buf_, a3_kps, a3_kds, command_out);
    return true;
  };
}

A3PolicyDriver::A3PolicyDriver(robot_io::RobotIOBackend& backend,
                               CommandFn command,
                               A3PolicyDriverOptions opt)
    : a3_rt::A3BasedTask(BuildBaseOptions_(opt)),
      backend_(backend),
      command_(std::move(command)),
      opt_(opt),
      watchdog_(opt.watchdog) {
  // Pre-size the reusable command buffer so RunOnce is alloc-free from tick 1.
  PreSizeCommandBuffer(cmd_out_);
}

A3PolicyDriver::~A3PolicyDriver() { StopDriver(); }

// ---------------------------------------------------------------------------
bool A3PolicyDriver::StartDriver() {
  backend_.RegisterStateCallback(
      [this](const robot_io::RobotState& s) { OnBackendState_(s); });
  if (opt_.trigger_on_state) {
    if (event_running_.exchange(true, std::memory_order_acq_rel)) {
      return false;
    }
    try {
      event_thread_ = std::thread(&A3PolicyDriver::EventThreadMain_, this);
    } catch (...) {
      event_running_.store(false, std::memory_order_release);
      return false;
    }
    return true;
  }
  return Start();
}

void A3PolicyDriver::StopDriver() {
  if (opt_.trigger_on_state) {
    event_running_.store(false, std::memory_order_release);
    event_cv_.notify_all();
    if (event_thread_.joinable()) event_thread_.join();
    backend_.RegisterStateCallback({});
    return;
  }
  Stop();
}

// ---------------------------------------------------------------------------
void A3PolicyDriver::OnBackendState_(const robot_io::RobotState& state) noexcept {
  // Copy into a heap-allocated snapshot so the RT thread can read it without
  // racing with the sync-loop's writes. shared_ptr refcount is thread-safe,
  // but the std::atomic<shared_ptr> specialisation is missing on GCC 11.4
  // libstdc++; use the free-function std::atomic_load / atomic_store instead
  // (same pattern as a3_aimrt_backend.cpp).
  try {
    auto snap = std::make_shared<const robot_io::RobotState>(state);
    std::atomic_store(&latest_state_, snap);
    if (opt_.trigger_on_state) {
      {
        std::lock_guard<std::mutex> lk(event_mtx_);
        event_state_ = std::move(snap);
        ++event_state_seq_;
      }
      event_cv_.notify_one();
    }
  } catch (...) {
    // noexcept contract — swallow any bad_alloc.
  }
}

void A3PolicyDriver::EventThreadMain_() noexcept {
  char name[16]{};
  std::strncpy(name, "a3_policy_evt", sizeof(name) - 1);
  prctl(PR_SET_NAME, name, 0, 0, 0);
  a3_rt::SetRtSchedFifo(opt_.sched.priority);
  a3_rt::PinCurrentThreadToCpu(opt_.sched.cpu);

  std::uint64_t seen_seq = 0;
  std::int64_t last_policy_state_stamp_ns = 0;
  const std::int64_t min_period_ns =
      opt_.trigger_min_period_ns > 0
          ? opt_.trigger_min_period_ns
          : static_cast<std::int64_t>(1.0e9 /
                                      (opt_.policy_hz > 0.0 ? opt_.policy_hz
                                                            : 50.0));

  while (event_running_.load(std::memory_order_acquire)) {
    std::shared_ptr<const robot_io::RobotState> state;
    {
      std::unique_lock<std::mutex> lk(event_mtx_);
      event_cv_.wait(lk, [&] {
        return !event_running_.load(std::memory_order_acquire) ||
               event_state_seq_ != seen_seq;
      });
      if (!event_running_.load(std::memory_order_acquire)) break;
      seen_seq = event_state_seq_;
      state = event_state_;
    }

    if (!state || !state->sync_complete || state->state_sync_ready_ns <= 0) {
      continue;
    }
    if (last_policy_state_stamp_ns > 0 &&
        state->timestamp_ns < last_policy_state_stamp_ns + min_period_ns) {
      continue;
    }
    last_policy_state_stamp_ns = state->timestamp_ns;

    const auto target_ns = state->state_sync_ready_ns +
                           std::max<std::int64_t>(0, opt_.trigger_offset_ns);
    const auto now_ns = NowSystemNs();
    if (target_ns > now_ns) {
      std::this_thread::sleep_until(
          std::chrono::system_clock::time_point(
              std::chrono::nanoseconds(target_ns)));
      if (!event_running_.load(std::memory_order_acquire)) break;
    }

    RunOnceWithState_(state);
  }
}

// ---------------------------------------------------------------------------
void A3PolicyDriver::RunOnce() noexcept {
  // Non-RT-safe things to avoid here: new/malloc, std::cout, throws. All
  // buffers are pre-allocated members.
  auto s = std::atomic_load(&latest_state_);
  if (!s) return;  // No state yet; skip this tick.
  RunOnceWithState_(std::move(s));
}

bool A3PolicyDriver::EmitSafeHalt_(const robot_io::RobotState& state) noexcept {
  if (!opt_.send_safe_halt_before_first_command &&
      !has_sent_command_.load(std::memory_order_acquire)) {
    return false;
  }
  BuildSafeHaltCommand(state.q, cmd_out_);
  backend_.SendCommand(cmd_out_);
  has_sent_command_.store(true, std::memory_order_release);
  safe_halt_count_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void A3PolicyDriver::RunOnceWithState_(
    std::shared_ptr<const robot_io::RobotState> s) noexcept {
  const std::int64_t now = NowSystemNs();

  const bool complete = s->sync_complete;
  const bool aligned = complete && s->sync_aligned;
  const auto verdict = watchdog_.Check(now, s->timestamp_ns, aligned);

  if (command_fault_latched_.load(std::memory_order_acquire)) {
    EmitSafeHalt_(*s);
    return;
  }

  if (verdict == WatchdogVerdict::Ok && complete) {
    // Hand the RT thread the full state + its monotonic tick index. The
    // callback is responsible for gathering the 29-DOF MuJoCo policy view
    // out of state.q via robot_io::ExtractPolicyView, building the
    // obs_dict, running inference, and writing q_des_29_out (29-DOF
    // MuJoCo policy view — waist + arms + legs, neck excluded; see
    // notes/a3_dof_orderings.md). tick_counter_ is the current Ok-tick
    // count (0 on the very first Ok tick); it is post-incremented below
    // so the NEXT Ok tick sees one higher. The tokenizer replay uses
    // tick_idx to index its offline-baked stream. Short complete-but-
    // unaligned bursts are still allowed here; A3Watchdog escalates only
    // after the configured consecutive threshold.
    if (command_) {
      const std::uint64_t tick_idx =
          policy_tick_count_.load(std::memory_order_relaxed);
      try {
        if (!command_(tick_idx, *s, cmd_out_)) {
          return;
        }
      } catch (const std::exception& e) {
        command_fault_latched_.store(true, std::memory_order_release);
        const bool emitted = EmitSafeHalt_(*s);
        std::fprintf(stderr,
                     "[a3_policy_driver] SAFETY LATCH: policy command threw: %s; "
                     "%s until runner restart\n",
                     e.what(), emitted ? "safe halt emitted" :
                                         "startup-silent; safe halt suppressed");
        return;
      } catch (...) {
        command_fault_latched_.store(true, std::memory_order_release);
        const bool emitted = EmitSafeHalt_(*s);
        std::fprintf(stderr,
                     "[a3_policy_driver] SAFETY LATCH: policy command threw an unknown "
                     "exception; %s until runner restart\n",
                     emitted ? "safe halt emitted" :
                               "startup-silent; safe halt suppressed");
        return;
      }
    } else {
      q_out_buf_.fill(0.0);
      ExpandToBackend(q_out_buf_, a3_kps, a3_kds, cmd_out_);
    }
    if (!IsFiniteCommand(cmd_out_)) {
      command_fault_latched_.store(true, std::memory_order_release);
      const bool emitted = EmitSafeHalt_(*s);
      std::fprintf(stderr,
                   "[a3_policy_driver] SAFETY LATCH: non-finite or wrong-sized policy "
                   "command; %s until runner restart\n",
                   emitted ? "safe halt emitted" :
                             "startup-silent; safe halt suppressed");
      return;
    }
    backend_.SendCommand(cmd_out_);
    has_sent_command_.store(true, std::memory_order_release);
    policy_tick_count_.fetch_add(1, std::memory_order_relaxed);
  } else {
    EmitSafeHalt_(*s);
  }
}

}  // namespace a3_deploy
