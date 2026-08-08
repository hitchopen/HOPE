// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// A3SyncLoop — 6-channel (waist / leg / arm / neck / pelvis_imu / torso_imu)
// time-synchronised sampler that emits a robot_io::RobotState (31 DOF) at a
// fixed rate. Algorithm mirrors aimrl_sdk::Core::sync_loop_ (MIT) extended
// from 3 rings to 6 rings. Fixed clock only; IMU-driven clock is deferred.
//
// Pure C++ — no AimRT dependency (AimRT glue lives in PR 5).
// See notes/a3_backend_plan.md §8 / PR 3.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "a3_sync/a3_ring_buffer.hpp"
#include "a3_sync/a3_statistics.hpp"
#include "a3_sync/a3_sync_types.hpp"
#include "robot_io/robot_io_backend.hpp"

namespace a3_sync {

// --- Pure interpolation helpers (unit-tested in isolation) ------------------
double Lerp(double a, double b, double t) noexcept;

template <std::size_t N>
void LerpArrays(const std::array<double, N>& a,
                const std::array<double, N>& b,
                double t,
                std::array<double, N>& out) noexcept {
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = a[i] + (b[i] - a[i]) * t;
  }
}

// Spherical linear interpolation of quaternions in WXYZ order. Inputs are
// renormalised, shortest-arc selected, and output is a unit quaternion.
std::array<double, 4> SlerpQuatWxyz(const std::array<double, 4>& q0_wxyz,
                                    const std::array<double, 4>& q1_wxyz,
                                    double t);

// --- Assembly into the robot_io 31-DOF state -------------------------------
//
// `*_ok` is true iff the corresponding channel contributed a valid
// interpolated sample for this tick. If a channel is not ok its slot in the
// output is still filled from the (stale / zero) sample — downstream should
// consume `aligned` / `complete` flags via the Snapshot.
robot_io::RobotState AssembleState(
    std::int64_t tick_ns,
    std::int64_t tick_seq,
    const WaistSample& waist, bool waist_ok,
    const LegSample&   leg,   bool leg_ok,
    const ArmSample&   arm,   bool arm_ok,
    const NeckSample&  neck,  bool neck_ok,
    const ImuSample&   pelvis, bool pelvis_ok,
    const ImuSample&   torso,  bool torso_ok,
    bool complete = true,
    bool aligned = true,
    std::int64_t skew_ns = 0);

// --- The sync loop itself ---------------------------------------------------
class A3SyncLoop {
 public:
  struct Options {
    SyncConfig sync{};
    std::uint32_t waist_ring      = 2048;
    std::uint32_t leg_ring        = 2048;
    std::uint32_t arm_ring        = 2048;
    std::uint32_t neck_ring       = 2048;
    std::uint32_t pelvis_imu_ring = 512;
    std::uint32_t torso_imu_ring  = 512;
  };

  using StateCallback = std::function<void(const robot_io::RobotState&)>;

  explicit A3SyncLoop(Options opt);
  ~A3SyncLoop();

  A3SyncLoop(const A3SyncLoop&)            = delete;
  A3SyncLoop& operator=(const A3SyncLoop&) = delete;

  void Start();
  void Stop();
  bool Running() const noexcept { return running_.load(std::memory_order_relaxed); }
  bool SetPhaseNsForStartup(std::int64_t phase_ns);
  bool EstimateAutoPhaseNs(std::int64_t release_margin_ns,
                           std::int64_t& phase_ns,
                           std::int64_t& latest_recv_ns,
                           std::int64_t& pair_skew_ns) const;

  // Subscriber-thread-safe sample intake.
  void OnWaistState(const WaistSample& s);
  void OnLegState(const LegSample& s);
  void OnArmState(const ArmSample& s);
  void OnNeckState(const NeckSample& s);
  void OnPelvisImu(const ImuSample& s);
  void OnTorsoImu(const ImuSample& s);

  // Register the downstream RobotState consumer. Must be called before Start()
  // (or at latest before the first tick fires).
  void RegisterStateCallback(StateCallback cb);

  A3SyncStatistics::Snapshot Statistics() const { return stats_.snapshot(); }
  A3SyncStatistics::LatencySnapshot ConsumeLatencyStatistics() {
    return stats_.ConsumeLatencySnapshot();
  }
  void ResetStatistics() { stats_.Reset(); }

 private:
  void SyncThread();
  void NotifySampleAvailable();
  bool TryBuildLatestFrameLocked(robot_io::RobotState& state);

  Options opt_;

  RingBuffer<WaistSample> waist_ring_;
  RingBuffer<LegSample>   leg_ring_;
  RingBuffer<ArmSample>   arm_ring_;
  RingBuffer<NeckSample>  neck_ring_;
  RingBuffer<ImuSample>   pelvis_ring_;
  RingBuffer<ImuSample>   torso_ring_;

  StateCallback state_cb_;
  mutable std::mutex cb_mtx_;
  std::mutex sample_mtx_;
  std::condition_variable sample_cv_;
  std::atomic<std::uint64_t> sample_seq_{0};

  std::atomic<bool> running_{false};
  std::thread       thread_;
  std::atomic<std::int64_t> tick_seq_{0};

  A3SyncStatistics stats_;
};

}  // namespace a3_sync
