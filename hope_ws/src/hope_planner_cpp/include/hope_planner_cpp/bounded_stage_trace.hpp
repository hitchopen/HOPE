#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace hope_planner_cpp {

// Optional local accounting. No DDS traffic or file IO on the callback/worker
// path, no allocation after configuration, and a fixed latest-record bound.
class BoundedStageTrace {
 public:
  void configure(std::string path, std::size_t capacity = 65536) {
    if (path.empty()) return;
    if (!capacity || capacity > 65536) throw std::invalid_argument("invalid trace capacity");
    std::ofstream probe(path);
    if (!probe) throw std::invalid_argument("stage trace path is not writable");
    path_ = std::move(path);
    rows_.resize(capacity);
  }
  bool enabled() const noexcept { return !path_.empty(); }
  void record(std::string_view stage, std::int64_t source_ns, std::uint64_t sequence,
              std::uint64_t generation, std::int64_t wall_ns, std::int64_t steady_ns,
              std::int64_t duration_ns = 0, std::string_view reason = {}) {
    if (!enabled()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    auto &row = rows_[total_++ % rows_.size()];
    row.source_ns = source_ns; row.sequence = sequence; row.generation = generation;
    row.wall_ns = wall_ns; row.steady_ns = steady_ns; row.duration_ns = duration_ns;
    copy(row.stage, stage); copy(row.reason, reason);
  }
  void flush() {
    if (!enabled()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream out(path_);
    if (!out) throw std::runtime_error("cannot flush stage accounting");
    const auto count = std::min(total_, rows_.size());
    out << "# capacity=" << rows_.size() << " total=" << total_
        << " overwritten=" << total_ - count << '\n';
    out << "stage\tsource_ns\tsequence\tgeneration\twall_ns\tsteady_ns\tduration_ns\treason\n";
    for (std::size_t i = total_ - count; i < total_; ++i) {
      const auto &row = rows_[i % rows_.size()];
      out << row.stage.data() << '\t' << row.source_ns << '\t' << row.sequence << '\t'
          << row.generation << '\t' << row.wall_ns << '\t' << row.steady_ns << '\t'
          << row.duration_ns << '\t' << row.reason.data() << '\n';
    }
    if (!out) throw std::runtime_error("incomplete stage accounting write");
  }
 private:
  template <std::size_t N> static void copy(std::array<char, N> &to, std::string_view from) {
    const auto count = std::min(N - 1, from.size());
    for (std::size_t i = 0; i < count; ++i)
      to[i] = static_cast<unsigned char>(from[i]) < 32 ? '_' : from[i];
    to[count] = '\0';
  }
  struct Row {
    std::int64_t source_ns{}, wall_ns{}, steady_ns{}, duration_ns{};
    std::uint64_t sequence{}, generation{};
    std::array<char, 32> stage{};
    std::array<char, 96> reason{};
  };
  std::string path_;
  std::vector<Row> rows_;
  std::size_t total_ = 0;
  std::mutex mutex_;
};
}  // namespace hope_planner_cpp
