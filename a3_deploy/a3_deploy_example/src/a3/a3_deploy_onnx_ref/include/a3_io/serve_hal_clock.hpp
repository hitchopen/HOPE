#pragma once
#include <cstdint>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace a3_io {
// Single HAL writer, read-only Runner. CLOCK_MONOTONIC in both processes.
// The edge is command selection, NOT feedback that the ball has detached.
struct alignas(64) ServeHalClock {
  static constexpr std::uint64_t kMagic = 0x48414c5345525602ULL;
  std::uint64_t sequence, magic, pid, selected_ns, release_edge_ns, left, node, heartbeat_ns;
};
static_assert(sizeof(ServeHalClock) == 64);
static_assert(__atomic_always_lock_free(8, nullptr));
struct ServeHalSnapshot {
  std::uint64_t pid{}, selected_ns{}, release_edge_ns{}, left{}, heartbeat_ns{};
  bool Ready(std::uint64_t now_ns) const {
    return pid && heartbeat_ns && heartbeat_ns <= now_ns && now_ns - heartbeat_ns <= 100'000'000;
  }
};
inline bool ReadServeHalClock(const ServeHalClock* p, ServeHalSnapshot& out) {
  if (!p) return false;
  // Bounded: a preempted writer must never block the body control thread.
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto seq = __atomic_load_n(&p->sequence, __ATOMIC_SEQ_CST);
    if (seq & 1) continue;
    const auto magic = __atomic_load_n(&p->magic, __ATOMIC_SEQ_CST);
    ServeHalSnapshot value{
      __atomic_load_n(&p->pid, __ATOMIC_SEQ_CST),
      __atomic_load_n(&p->selected_ns, __ATOMIC_SEQ_CST),
      __atomic_load_n(&p->release_edge_ns, __ATOMIC_SEQ_CST),
      __atomic_load_n(&p->left, __ATOMIC_SEQ_CST),
      __atomic_load_n(&p->heartbeat_ns, __ATOMIC_SEQ_CST)};
    if (seq == __atomic_load_n(&p->sequence, __ATOMIC_SEQ_CST) &&
        magic == ServeHalClock::kMagic) { out = value; return true; }
  }
  return false;
}
class ServeHalClockReader {
 public:
  ~ServeHalClockReader() { if (auto* p = clock_.load()) munmap(p, sizeof(*p)); }
  bool IsOpen() const { return clock_.load(std::memory_order_acquire) != nullptr; }
  bool Open(const char* path) {
    if (IsOpen()) return true;
    const int fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) return false;
    struct stat st{};
    const bool valid = fstat(fd, &st) == 0 && S_ISREG(st.st_mode) &&
        st.st_uid == getuid() && st.st_size == sizeof(ServeHalClock);
    void* mapped = valid ? mmap(nullptr, sizeof(ServeHalClock), PROT_READ, MAP_SHARED, fd, 0) : MAP_FAILED;
    close(fd);
    if (mapped == MAP_FAILED) return false;
    clock_.store(static_cast<ServeHalClock*>(mapped), std::memory_order_release);
    return true;
  }
  bool Read(ServeHalSnapshot& value) const { return ReadServeHalClock(clock_.load(std::memory_order_acquire), value); }
 private:
  std::atomic<ServeHalClock*> clock_{nullptr};
};
}  // namespace a3_io
