// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
//
// Abstract base for all A3 subscribers. Pure C++ — no AimRT / ROS2
// dependency on this header (so it compiles whether or not
// ENABLE_A3_ROS_MSGS is ON).
//
// Each concrete subscriber owns a reference to an external ring buffer
// (the ring is owned by A3SyncLoop / A3AimrtBackend). Once at least one
// valid message has been received and a sample committed, IsReady()
// returns true — A3SubscriberManager::WaitAllReady() polls this flag
// across all six subs to form the launch barrier.

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <utility>

namespace a3_io {

template <typename SampleT>
class SubscriberBase {
 public:
  using Sample = SampleT;
  using SampleCallback = std::function<void(const Sample&)>;

  SubscriberBase() = default;
  explicit SubscriberBase(SampleCallback cb) : sample_cb_(std::move(cb)) {}
  virtual ~SubscriberBase() = default;

  SubscriberBase(const SubscriberBase&) = delete;
  SubscriberBase& operator=(const SubscriberBase&) = delete;

  // Returns true once at least one message has been successfully
  // converted and written to the ring buffer.
  bool IsReady() const noexcept {
    return ready_.load(std::memory_order_acquire);
  }

  std::uint64_t SampleCount() const noexcept {
    return sample_count_.load(std::memory_order_relaxed);
  }

 protected:
  void MarkReady() noexcept {
    sample_count_.fetch_add(1, std::memory_order_relaxed);
    ready_.store(true, std::memory_order_release);
  }

  void NotifySample(const Sample& sample) {
    if (sample_cb_) sample_cb_(sample);
  }

 private:
  std::atomic<bool> ready_{false};
  std::atomic<std::uint64_t> sample_count_{0};
  SampleCallback sample_cb_{};
};

}  // namespace a3_io
