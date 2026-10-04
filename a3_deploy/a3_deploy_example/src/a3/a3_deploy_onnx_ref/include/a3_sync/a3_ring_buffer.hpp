// SPDX-License-Identifier: MIT
//
// Adapted from aimrl_sdk::RingBuffer (see
// /home/agiuser/code/rl_deploy/aimrl_sdk/src/aimrl_sdk/cpp/ring_buffer.hpp).
// Upstream is MIT-licensed; copyright belongs to the original authors. This
// version protects payload copies against a concurrent slot overwrite.
//
// Single-writer / multi-reader ring buffer. Per-slot exclusion is necessary:
// checking an atomic commit counter around a non-atomic payload copy neither
// prevents a C++ data race nor detects an overwrite that has not committed yet.
// Readers try-lock and skip a busy slot, so synchronization never waits on a
// writer. Payload access is short and bounded in production (sample copies).
//
// See notes/a3_backend_plan.md §8 / PR 3.
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace a3_sync {

template <class T>
class RingBuffer {
 public:
  explicit RingBuffer(std::uint32_t capacity)
      : capacity_(capacity), slots_(capacity), committed_(capacity), slot_mutexes_(capacity) {
    for (auto& c : committed_) {
      c.store(0, std::memory_order_relaxed);
    }
  }

  std::uint32_t capacity() const noexcept { return capacity_; }

  template <class F>
  std::uint64_t write(F&& fill) {
    // Index starts at 1 so that `0` can be used as an empty-slot sentinel.
    const auto idx = write_idx_.fetch_add(1, std::memory_order_relaxed) + 1;
    const auto s   = static_cast<std::uint32_t>(idx % capacity_);
    std::lock_guard<std::mutex> lock(slot_mutexes_[s]);
    committed_[s].store(0, std::memory_order_relaxed);
    fill(slots_[s]);
    committed_[s].store(idx, std::memory_order_release);
    return idx;
  }

  bool read_at(std::uint64_t idx, T& out) const {
    if (idx == 0) return false;
    const auto s = static_cast<std::uint32_t>(idx % capacity_);
    std::unique_lock<std::mutex> lock(slot_mutexes_[s], std::try_to_lock);
    if (!lock.owns_lock()) return false;
    const auto c = committed_[s].load(std::memory_order_acquire);
    if (c != idx) return false;
    out = slots_[s];
    return true;
  }

  std::uint64_t latest_index() const noexcept {
    return write_idx_.load(std::memory_order_relaxed);
  }

 private:
  std::uint32_t capacity_;
  std::vector<T> slots_;
  std::vector<std::atomic<std::uint64_t>> committed_;
  mutable std::vector<std::mutex> slot_mutexes_;
  std::atomic<std::uint64_t> write_idx_{0};
};

}  // namespace a3_sync
