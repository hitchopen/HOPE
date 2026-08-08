// SPDX-License-Identifier: MIT
//
// Near-verbatim port of aimrl_sdk::RingBuffer (see
// /home/agiuser/code/rl_deploy/aimrl_sdk/src/aimrl_sdk/cpp/ring_buffer.hpp).
// Upstream is MIT-licensed; copyright belongs to the original authors. This
// file only renames the namespace to a3_sync and adds a small header comment.
//
// Lock-free single-writer / multi-reader ring buffer. The writer fetch-adds
// a monotonic index and commits by storing that index into the slot's
// `committed` cell with release semantics; readers acquire-load the cell and
// verify the index still matches before copying the payload out (so a
// writer-stomping-reader race is detected rather than returning torn data).
//
// See notes/a3_backend_plan.md §8 / PR 3.
#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

namespace a3_sync {

template <class T>
class RingBuffer {
 public:
  explicit RingBuffer(std::uint32_t capacity)
      : capacity_(capacity), slots_(capacity), committed_(capacity) {
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
    fill(slots_[s]);
    committed_[s].store(idx, std::memory_order_release);
    return idx;
  }

  bool read_at(std::uint64_t idx, T& out) const {
    if (idx == 0) return false;
    const auto s = static_cast<std::uint32_t>(idx % capacity_);
    const auto c = committed_[s].load(std::memory_order_acquire);
    if (c != idx) return false;
    out = slots_[s];
    // Re-check the commit index after the copy to detect a stomping writer.
    const auto c2 = committed_[s].load(std::memory_order_acquire);
    return c2 == idx;
  }

  std::uint64_t latest_index() const noexcept {
    return write_idx_.load(std::memory_order_relaxed);
  }

 private:
  std::uint32_t capacity_;
  std::vector<T> slots_;
  std::vector<std::atomic<std::uint64_t>> committed_;
  std::atomic<std::uint64_t> write_idx_{0};
};

}  // namespace a3_sync
