#include "hope_planner_cpp/audit_logger.hpp"
#include "a3_deploy/recording_file.hpp"
#include <cstdlib>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <unistd.h>

namespace hope_planner_cpp {

AuditLogger::AuditLogger(const std::string& path, const std::string& header) {
  if (path.empty()) {
    return;
  }
  const char* control = std::getenv("HOPE_RECORD_CONTROL_FILE");
  if (control && *control) {
    recording_ = std::make_unique<hope_recording::RecordingFile>(path, control, header + "\n");
    enabled_ = true;
    thread_ = std::thread(&AuditLogger::run, this);
    return;
  }
  const std::filesystem::path output(path);
  if (output.has_parent_path()) {
    std::filesystem::create_directories(output.parent_path());
  }
  const int file_descriptor = ::open(
      output.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
  if (file_descriptor < 0) {
    throw std::runtime_error(
        "refusing to overwrite Planner audit CSV " + output.string() + ": " +
        std::strerror(errno));
  }
  stream_ = ::fdopen(file_descriptor, "w");
  if (stream_ == nullptr) {
    const std::string error = std::strerror(errno);
    ::close(file_descriptor);
    std::error_code remove_error;
    std::filesystem::remove(output, remove_error);
    throw std::runtime_error(
        "cannot open Planner audit CSV stream " + output.string() + ": " + error);
  }
  if (std::fwrite(header.data(), 1, header.size(), stream_) != header.size() ||
      std::fputc('\n', stream_) == EOF || std::fflush(stream_) != 0) {
    const std::string error = std::strerror(errno);
    std::fclose(stream_);
    stream_ = nullptr;
    throw std::runtime_error(
        "cannot initialize Planner audit CSV " + output.string() + ": " + error);
  }
  enabled_ = true;
  thread_ = std::thread(&AuditLogger::run, this);
}

AuditLogger::~AuditLogger() {
  stop();
}

void AuditLogger::enqueue(std::string row) noexcept {
  if (!enabled_) {
    return;
  }
  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= kQueueCapacity) {
        ++dropped_rows_;
        return;
      }
      queue_.emplace_back(std::move(row), recording_generation_.load(std::memory_order_acquire));
    }
    condition_.notify_one();
  } catch (...) {
    ++dropped_rows_;
  }
}

std::size_t AuditLogger::queue_depth() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

void AuditLogger::stop() noexcept {
  if (!enabled_) {
    return;
  }
  stop_requested_.store(true, std::memory_order_release);
  condition_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  if (stream_ != nullptr) {
    std::fflush(stream_);
    std::fclose(stream_);
    stream_ = nullptr;
  }
  enabled_ = false;
}

void AuditLogger::run() noexcept {
  try {
    for (;;) {
      if (recording_) {
        recording_->Dropped(dropped_rows_.load());
        recording_generation_.store(recording_->Poll(), std::memory_order_release);
      }
      std::deque<std::pair<std::string, std::uint64_t>> batch;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(20), [this] {
          return stop_requested_.load(std::memory_order_acquire) || !queue_.empty();
        });
        batch.swap(queue_);
        if (batch.empty() && stop_requested_.load(std::memory_order_acquire)) {
          break;
        }
      }
      for (const auto& entry : batch) {
        const auto& row = entry.first;
        if (recording_) { recording_->Write(row + "\n", entry.second); continue; }
        if (std::fwrite(row.data(), 1, row.size(), stream_) != row.size() ||
            std::fputc('\n', stream_) == EOF) {
          throw std::runtime_error("Planner audit CSV write failed");
        }
      }
      if (stream_ && std::fflush(stream_) != 0) {
        throw std::runtime_error("Planner audit CSV flush failed");
      }
      if (stop_requested_.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
          break;
        }
      }
    }
    if (recording_) recording_->Close();
  } catch (...) {
    std::size_t remaining = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      remaining = queue_.size();
      queue_.clear();
    }
    dropped_rows_.fetch_add(remaining, std::memory_order_relaxed);
  }
}

}  // namespace hope_planner_cpp
