#pragma once

#include <array>
#include <algorithm>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <ostream>
#include <streambuf>
#include <string>
#include <thread>
#include <sys/statvfs.h>
#include "a3_deploy/recording_file.hpp"

namespace a3_deploy {

// One producer (control callback), one disk writer. After open(), no disk I/O,
// locks, or allocation in streambuf writes. Overflow drops bytes, never waits
// for storage. A nonzero loss/error counter invalidates the CSV for replay.
// Formatting still runs on the caller; this is not a hard-real-time guarantee.
class AsyncCsvFile : public std::ostream {
  class Buffer : public std::streambuf {
   public:
    static constexpr std::size_t kChunk = 16384, kSlots = 256;
    struct Slot { std::array<char, kChunk> data; std::size_t size = 0; std::uint64_t generation = 0; };
    bool Open(const std::string& path) {
      if (opened_) return false;
      const char* control = std::getenv("HOPE_RECORD_CONTROL_FILE");
      if (control && *control) gated_ = std::make_unique<hope_recording::RecordingFile>(path, control);
      // Never truncate an existing capture.
      if (!gated_) {
        file_ = std::fopen(path.c_str(), "wx");
        if (!file_) return false;
      }
      opened_ = true;
      path_ = path;
      slots_ = std::make_unique<Slot[]>(kSlots);
      setp(pending_.data(), pending_.data() + kChunk);
      worker_ = std::thread([this] { Run(); });
      return true;
    }
    bool IsOpen() const { return opened_; }
    // Query before formatting a data row. Header emission still happens once
    // after open(), even when the recording switch starts OFF.
    bool IsRecording() const {
      return opened_ && (!gated_ || generation_.load(std::memory_order_acquire) != 0);
    }
    void Close() {
      if (!opened_) return;
      sync();
      stop_.store(true, std::memory_order_release);
      worker_.join();
      if (file_ && std::fclose(file_) != 0) io_error_.store(true);
      file_ = nullptr;
      if (!gated_) Health(true);
      opened_ = false;
    }
    ~Buffer() override { Close(); }
   protected:
    std::streamsize xsputn(const char* data, std::streamsize count) override {
      if (!gated_) return std::streambuf::xsputn(data, count);
      const auto original = count;
      while (count > 0) {
        auto n = std::min<std::streamsize>(count, epptr() - pptr());
        if (n == 0) { sync(); continue; }
        const auto* newline = static_cast<const char*>(std::memchr(data, '\n', n));
        if (newline) n = newline - data + 1;
        std::memcpy(pptr(), data, n); pbump(static_cast<int>(n)); data += n; count -= n;
        if (newline) { sync(); row_generation_ = generation_.load(std::memory_order_acquire); }
      }
      return original;
    }
    int sync() override {
      const auto size = static_cast<std::size_t>(pptr() - pbase());
      if (!size) return 0;
      const bool row_end = pending_[size - 1] == '\n';
      // Keep the header even when starting OFF. Subsequent OFF rows are
      // intentionally unrecorded, so they must not occupy the disk queue or
      // contribute to its overflow counter. This path stays nonblocking.
      if (gated_ && header_complete_ && row_generation_ == 0) {
        if (row_end) row_generation_ = generation_.load(std::memory_order_acquire);
        setp(pending_.data(), pending_.data() + kChunk);
        return 0;
      }
      const auto head = head_.load(std::memory_order_relaxed);
      if (head - tail_.load(std::memory_order_acquire) == kSlots) {
        dropped_.fetch_add(size, std::memory_order_relaxed);
      } else {
        auto& slot = slots_[head % kSlots];
        std::memcpy(slot.data.data(), pbase(), size);
        slot.size = size;
        slot.generation = row_generation_;
        head_.store(head + 1, std::memory_order_release);
      }
      if (row_end) {
        header_complete_ = true;
        row_generation_ = generation_.load(std::memory_order_acquire);
      }
      setp(pending_.data(), pending_.data() + kChunk);
      return 0;
    }
    int_type overflow(int_type ch) override {
      sync();
      if (!traits_type::eq_int_type(ch, traits_type::eof())) {
        *pptr() = traits_type::to_char_type(ch);
        pbump(1);
        if (traits_type::to_char_type(ch) == '\n') {
          sync(); row_generation_ = generation_.load(std::memory_order_acquire);
        }
      }
      return traits_type::not_eof(ch);
    }
   private:
    void Health(bool closed) {
      const auto temp = path_ + ".health.json.tmp";
      if (auto* f = std::fopen(temp.c_str(), "w")) {
        std::fprintf(f, "{\"schema\":1,\"closed\":%s,\"written_bytes\":%llu,"
                        "\"dropped_bytes\":%llu,\"io_error\":%s,\"disk_reserve_hit\":%s}\n",
                     closed ? "true" : "false", static_cast<unsigned long long>(written_),
                     static_cast<unsigned long long>(dropped_.load()),
                     io_error_.load() ? "true" : "false", low_disk_ ? "true" : "false");
        std::fclose(f);
        std::rename(temp.c_str(), (path_ + ".health.json").c_str());
      }
    }
    void Run() {
      auto next_health = std::chrono::steady_clock::now();
      std::string pending_line; bool have_header = false;
      for (;;) {
        if (gated_) {
          gated_->Dropped(dropped_.load());
          generation_.store(gated_->Poll(), std::memory_order_release);
        }
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail != head_.load(std::memory_order_acquire)) {
          auto& slot = slots_[tail % kSlots];
          if (gated_) {
            pending_line.append(slot.data.data(), slot.size);
            std::size_t newline;
            while ((newline = pending_line.find('\n')) != std::string::npos) {
              auto line = pending_line.substr(0, newline + 1);
              pending_line.erase(0, newline + 1);
              if (!have_header) { gated_->Header(std::move(line)); have_header = true; }
              else gated_->Write(line, slot.generation);
            }
          } else {
          const auto n = (io_error_.load() || low_disk_) ? 0 :
              std::fwrite(slot.data.data(), 1, slot.size, file_);
          written_ += n;
          if (n != slot.size) {
            dropped_.fetch_add(slot.size - n, std::memory_order_relaxed);
            if (!low_disk_) io_error_.store(true);
          }
          }
          tail_.store(tail + 1, std::memory_order_release);
        } else if (stop_.load(std::memory_order_acquire)) {
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (!gated_ && std::chrono::steady_clock::now() >= next_health) {
          if (std::fflush(file_) != 0) io_error_.store(true);
          struct statvfs fs{};
          if (::statvfs(path_.c_str(), &fs) == 0 &&
              static_cast<unsigned long long>(fs.f_bavail) * fs.f_frsize < 512ULL * 1024 * 1024)
            low_disk_ = true;
          Health(false);
          next_health = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        }
      }
      if (gated_) { gated_->Dropped(dropped_.load()); gated_->Close(); }
    }
    std::FILE* file_ = nullptr;
    bool opened_ = false;
    std::unique_ptr<hope_recording::RecordingFile> gated_;
    std::atomic<std::uint64_t> generation_{0};
    std::uint64_t row_generation_ = 0;
    bool header_complete_ = false;
    std::string path_;
    std::array<char, kChunk> pending_{};
    std::unique_ptr<Slot[]> slots_;
    std::atomic<std::size_t> head_{0}, tail_{0};
    std::atomic<unsigned long long> dropped_{0};
    std::atomic<bool> stop_{false}, io_error_{false};
    unsigned long long written_ = 0;
    bool low_disk_ = false;
    std::thread worker_;
  } buffer_;
 public:
  AsyncCsvFile() : std::ostream(nullptr) { rdbuf(&buffer_); }
  void open(const std::string& path) { if (!buffer_.Open(path)) setstate(std::ios::failbit); }
  bool is_open() const { return buffer_.IsOpen(); }
  bool is_recording() const { return buffer_.IsRecording(); }
  void close() { buffer_.Close(); }
};

}  // namespace a3_deploy
