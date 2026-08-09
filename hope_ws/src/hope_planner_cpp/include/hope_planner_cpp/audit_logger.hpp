#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace hope_planner_cpp {

class AuditLogger {
 public:
  AuditLogger() = default;
  AuditLogger(const std::string& path, const std::string& header);
  ~AuditLogger();

  AuditLogger(const AuditLogger&) = delete;
  AuditLogger& operator=(const AuditLogger&) = delete;

  bool enabled() const noexcept { return enabled_; }
  void enqueue(std::string row) noexcept;
  void stop() noexcept;
  std::uint64_t dropped_rows() const noexcept { return dropped_rows_.load(); }
  std::size_t queue_depth() const noexcept;

 private:
  void run() noexcept;

  static constexpr std::size_t kQueueCapacity = 4096;
  bool enabled_ = false;
  std::FILE* stream_ = nullptr;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::string> queue_;
  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> dropped_rows_{0};
};

}  // namespace hope_planner_cpp
