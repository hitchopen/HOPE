#pragma once

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace hope_planner_cpp {

struct WireIdentityRange { std::uint64_t begin, end; };

// Reserve a disjoint U52 range at startup, before any publication. The file
// stores the exclusive upper bound, not the last command: crashes and clock
// rollback cannot reuse a previous process's IDs. No per-shot disk I/O.
inline WireIdentityRange reserve_wire_identity_range(
    const std::string& path, std::uint64_t clock_seed) {
  constexpr std::uint64_t limit = (1ULL << 52);
  constexpr std::uint64_t block = (1ULL << 32);
  struct Fd {
    int value;
    ~Fd() { if (value >= 0) ::close(value); }
  };
  const Fd lock{::open((path + ".lock").c_str(), O_RDWR | O_CREAT | O_NOFOLLOW, 0600)};
  if (lock.value < 0 || ::flock(lock.value, LOCK_EX) != 0)
    throw std::runtime_error("cannot lock Planner wire identity state");
  std::uint64_t previous = 0;
  if (std::filesystem::exists(path)) {
    std::ifstream input(path);
    std::string value, extra;
    if (!std::getline(input, value) || std::getline(input, extra))
      throw std::runtime_error("invalid Planner wire identity state");
    const auto result = std::from_chars(value.data(), value.data() + value.size(), previous);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || previous == 0)
      throw std::runtime_error("invalid Planner wire identity counter");
  }
  const auto begin = std::max(clock_seed, previous);
  if (begin == 0 || begin >= limit - block)
    throw std::runtime_error("Planner wire identity range exhausted");
  const WireIdentityRange range{begin, begin + block};
  std::string pattern = path + ".tmp.XXXXXX";
  std::vector<char> temporary(pattern.begin(), pattern.end());
  temporary.push_back('\0');
  const Fd output{::mkstemp(temporary.data())};
  if (output.value < 0) throw std::runtime_error("cannot stage Planner wire identity state");
  const auto bytes = std::to_string(range.end) + "\n";
  if (::write(output.value, bytes.data(), bytes.size()) != static_cast<ssize_t>(bytes.size()) ||
      ::fsync(output.value) != 0 || ::rename(temporary.data(), path.c_str()) != 0) {
    ::unlink(temporary.data());
    throw std::runtime_error("cannot persist Planner wire identity reservation");
  }
  const auto parent = std::filesystem::path(path).parent_path();
  const Fd directory{::open((parent.empty() ? "." : parent.string()).c_str(), O_RDONLY | O_DIRECTORY)};
  if (directory.value < 0 || ::fsync(directory.value) != 0)
    throw std::runtime_error("cannot sync Planner wire identity directory");
  return range;
}
}  // namespace hope_planner_cpp
