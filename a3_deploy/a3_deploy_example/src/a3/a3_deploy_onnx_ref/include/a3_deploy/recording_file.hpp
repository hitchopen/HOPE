#pragma once
// Shared disk-worker utility. Never call from a robot control callback.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <sys/statvfs.h>

namespace hope_recording {
class RecordingFile {
 public:
  RecordingFile(std::string base, std::string control, std::string header = {})
      : base_(std::move(base)), control_(std::move(control)), header_(std::move(header)) {}
  ~RecordingFile() { Close(); }
  void Header(std::string header) { header_ = std::move(header); next_poll_ = {}; }
  std::uint64_t Poll() {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_poll_) return file_ ? generation_ : 0;
    next_poll_ = now + std::chrono::milliseconds(50);
    std::ifstream input(control_); std::string requested; std::getline(input, requested);
    if (requested.size() > 96 || requested.rfind("record_", 0) != 0 ||
        requested.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos)
      requested.clear();
    if (requested != segment_) {
      Close(); segment_ = requested; ++generation_; failed_ = false;
      if (!segment_.empty()) {
        // The producer counter lives for the whole process. A new recording
        // owns only losses after this boundary, including after a failed take.
        dropped_baseline_ = producer_dropped_;
        dropped_ = io_dropped_ = written_ = rows_ = 0;
        io_error_ = low_disk_ = false;
      }
    }
    if (!segment_.empty() && !file_ && !failed_ && !header_.empty()) {
      try {
        const auto base = std::filesystem::path(base_);
        const auto directory = base.parent_path() / "recordings" / segment_;
        std::filesystem::create_directories(directory);
        path_ = (directory / base.filename()).string();
        written_ = rows_ = 0; io_error_ = low_disk_ = false;
        file_ = std::fopen(path_.c_str(), "wx");
        if (!file_) { failed_ = true; io_error_ = true; }
        else WriteBytes(header_);
      } catch (...) { failed_ = true; io_error_ = true; }
    }
    if (file_) {
      struct statvfs fs{};
      if (::statvfs(path_.c_str(), &fs) == 0 &&
          static_cast<unsigned long long>(fs.f_bavail) * fs.f_frsize < 512ULL * 1024 * 1024) {
        low_disk_ = failed_ = true; Close();
      } else if (io_error_ || std::fflush(file_) != 0) { io_error_ = failed_ = true; Close(); }
      else Health(false);
    }
    Status();
    return file_ ? generation_ : 0;
  }
  void Write(const std::string& line, std::uint64_t generation) {
    if (file_ && generation && generation == generation_) { WriteBytes(line); ++rows_; }
  }
  void Close() {
    if (file_) {
      if (std::fclose(file_) != 0) io_error_ = true;
      file_ = nullptr; Health(true);
    }
    Status();
  }
  void Dropped(unsigned long long count) {
    producer_dropped_ = count;
    // Preserve the closed take's outcome while OFF. Never charge OFF traffic
    // or a previous take to the next recording, and never erase I/O losses.
    if (!segment_.empty())
      dropped_ = (count >= dropped_baseline_ ? count - dropped_baseline_ : 0) + io_dropped_;
  }
 private:
  void WriteBytes(const std::string& value) {
    const auto n = std::fwrite(value.data(), 1, value.size(), file_); written_ += n;
    if (n != value.size()) {
      io_error_ = failed_ = true;
      io_dropped_ += value.size() - n;
      dropped_ += value.size() - n;
    }
  }
  void Health(bool closed) {
    const auto temp = path_ + ".health.json.tmp";
    if (auto* f = std::fopen(temp.c_str(), "w")) {
      std::fprintf(f, "{\"schema\":1,\"closed\":%s,\"written_bytes\":%llu,\"rows\":%llu,"
                      "\"dropped_bytes\":%llu,\"io_error\":%s,\"disk_reserve_hit\":%s}\n",
                   closed ? "true" : "false", written_, rows_, dropped_,
                   io_error_ ? "true" : "false", low_disk_ ? "true" : "false");
      std::fclose(f); std::rename(temp.c_str(), (path_ + ".health.json").c_str());
    }
  }
  void Status() {
    const auto temp = base_ + ".recording.json.tmp";
    if (auto* f = std::fopen(temp.c_str(), "w")) {
      std::fprintf(f, "{\"state\":\"%s\",\"segment\":\"%s\",\"rows\":%llu,\"dropped_bytes\":%llu}\n",
                   failed_ || io_error_ || dropped_ ? "ERROR" : file_ ? "ON" : "OFF", segment_.c_str(), rows_, dropped_);
      std::fclose(f); std::rename(temp.c_str(), (base_ + ".recording.json").c_str());
    }
  }
  std::string base_, control_, header_, segment_, path_;
  std::FILE* file_ = nullptr;
  unsigned long long written_ = 0, rows_ = 0, dropped_ = 0;
  unsigned long long producer_dropped_ = 0, dropped_baseline_ = 0, io_dropped_ = 0;
  std::uint64_t generation_ = 0;
  bool failed_ = false, io_error_ = false, low_disk_ = false;
  std::chrono::steady_clock::time_point next_poll_{};
};
}  // namespace hope_recording
