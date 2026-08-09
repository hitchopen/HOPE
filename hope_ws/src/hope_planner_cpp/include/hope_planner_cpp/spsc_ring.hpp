#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace hope_planner_cpp {

template <typename T, std::size_t Capacity>
class SpscRing {
  static_assert(Capacity >= 2, "SPSC capacity must be at least two");
  static_assert(std::is_copy_assignable<T>::value, "SPSC payload must be copy assignable");

 public:
  bool try_push(const T& value) noexcept {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto next = increment(head);
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    storage_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool try_pop(T& value) noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;
    }
    value = storage_[tail];
    tail_.store(increment(tail), std::memory_order_release);
    return true;
  }

  std::size_t size_approx() const noexcept {
    const auto head = head_.load(std::memory_order_acquire);
    const auto tail = tail_.load(std::memory_order_acquire);
    return head >= tail ? head - tail : Capacity - (tail - head);
  }

 private:
  static constexpr std::size_t increment(std::size_t value) noexcept {
    return (value + 1U) % Capacity;
  }

  std::array<T, Capacity> storage_{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace hope_planner_cpp
