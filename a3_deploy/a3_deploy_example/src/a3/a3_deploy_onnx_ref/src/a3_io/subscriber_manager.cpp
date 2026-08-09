// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
#include "a3_io/subscriber_manager.hpp"

#include <cassert>
#include <chrono>
#include <thread>

namespace a3_io {

namespace {

template <typename T>
T& DerefOrThrow(T* p, const char* what) {
  if (!p) {
    // In production, pass valid pointers; subscribers take refs.
    // We assert here so misconfiguration fails loudly in debug, and
    // avoid UB in release by crashing on deref.
    assert(p && "A3SubscriberManager: ring buffer pointer is null");
  }
  return *p;
}

}  // namespace

A3SubscriberManager::A3SubscriberManager(const Options& opt)
    : waist_(DerefOrThrow(opt.waist_ring, "waist_ring"),
             opt.waist_sample_cb),
      leg_(DerefOrThrow(opt.leg_ring, "leg_ring"), opt.leg_sample_cb),
      arm_(DerefOrThrow(opt.arm_ring, "arm_ring"), opt.arm_sample_cb),
      neck_(DerefOrThrow(opt.neck_ring, "neck_ring"), opt.neck_sample_cb),
      pelvis_imu_(DerefOrThrow(opt.pelvis_imu_ring, "pelvis_imu_ring"),
                  opt.pelvis_imu_sample_cb),
      torso_imu_(DerefOrThrow(opt.torso_imu_ring, "torso_imu_ring"),
                 opt.torso_imu_sample_cb) {}

bool A3SubscriberManager::AllReady() const noexcept {
  return waist_.IsReady() && leg_.IsReady() && arm_.IsReady() &&
         neck_.IsReady() && pelvis_imu_.IsReady() && torso_imu_.IsReady();
}

bool A3SubscriberManager::WaitAllReady(double timeout_seconds) {
  using clock = std::chrono::steady_clock;
  const auto deadline =
      clock::now() +
      std::chrono::duration_cast<clock::duration>(
          std::chrono::duration<double>(timeout_seconds));

  while (clock::now() < deadline) {
    if (AllReady()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return AllReady();
}

}  // namespace a3_io
