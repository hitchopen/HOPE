// ============================================================================
//  SIMULATION-ONLY oracle pelvis-pose reader.
// ============================================================================
//  Reads the MuJoCo ground-truth pelvis pose that the standalone bridge process
//  (scripts/oracle_pose_bridge.py, an rclpy node subscribing /sim/a3/pelvis_pose)
//  writes into a small shared-memory file (default /dev/shm/pp_oracle_pelvis).
//
//  WHY a shared-memory bridge instead of subscribing here:
//    * The ping-pong runner uses a PURE-iceoryx backend cfg and deliberately
//      does NOT load the (ABI-mismatched, prebuilt) ROS 2 plugin. Linking
//      rclcpp into the runner would reintroduce that risk. The bridge keeps all
//      ROS 2 code in a separate process; the runner only mmaps a 72-byte file.
//
//  >>> HARD SAFETY RULE: ORACLE LOCALIZATION IS FOR SIMULATION ONLY. <<<
//  On the real robot there is no ground-truth pelvis pose, the shm file will not
//  exist, and Open() returns false -> the policy MUST fall back to a non-oracle
//  localization mode. Never run oracle_pose_bridge.py on hardware.
//
//  Wire layout (little-endian, packed), written atomically-ish with a seqlock:
//    [0]  uint64  seq            (even = stable; bridge writes odd, then even)
//    [8]  double  stamp_wall_ns  (writer wall clock, for staleness)
//    [16] double  pos[3]         (odom-frame pelvis position, m)
//    [40] double  quat_wxyz[4]   (odom-frame pelvis orientation, w,x,y,z)
//  total 72 bytes. The Python bridge writes the identical struct.
#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>

#include "a3_pingpong/pp_frame_math.hpp"

namespace a3_pingpong {

struct PpOracleSample {
  Vec3 pos;
  Vec4 quat;  // w,x,y,z
  double age_s = 1e9;
  std::uint64_t seq = 0;
};

class PpOraclePose {
 public:
  static constexpr std::size_t kBytes = 72;

  // Returns false if the shm file does not exist (e.g. on hardware) -> caller
  // must NOT use oracle mode. Never creates the file.
  bool Open(const std::string& path) {
    path_ = path;
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0) return false;
    struct stat st {};
    if (::fstat(fd_, &st) != 0 || static_cast<std::size_t>(st.st_size) < kBytes) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    map_ = static_cast<std::uint8_t*>(
        ::mmap(nullptr, kBytes, PROT_READ, MAP_SHARED, fd_, 0));
    if (map_ == MAP_FAILED) {
      map_ = nullptr;
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  bool is_open() const { return map_ != nullptr; }
  const std::string& path() const { return path_; }

  // Tear-free seqlock read. Returns true only if a stable, fresh-enough sample
  // was read. `max_age_s` rejects stale samples (bridge stopped / sim paused).
  bool Latest(PpOracleSample& out, double max_age_s = 0.1) const {
    if (!map_) return false;
    std::uint64_t s0, s1;
    double stamp_ns, pos[3], quat[4];
    for (int tries = 0; tries < 8; ++tries) {
      std::memcpy(&s0, map_ + 0, 8);
      std::atomic_thread_fence(std::memory_order_acquire);
      std::memcpy(&stamp_ns, map_ + 8, 8);
      std::memcpy(pos, map_ + 16, 24);
      std::memcpy(quat, map_ + 40, 32);
      std::atomic_thread_fence(std::memory_order_acquire);
      std::memcpy(&s1, map_ + 0, 8);
      if (s0 == s1 && (s0 & 1ULL) == 0ULL) {  // stable snapshot
        const double now_ns = NowWallNs();
        out.age_s = (now_ns - stamp_ns) * 1e-9;
        out.seq = s0;
        out.pos = Vec3(pos[0], pos[1], pos[2]);
        out.quat = Vec4(quat[0], quat[1], quat[2], quat[3]);
        return out.age_s >= 0.0 && out.age_s <= max_age_s;
      }
    }
    return false;
  }

  ~PpOraclePose() {
    if (map_) ::munmap(map_, kBytes);
    if (fd_ >= 0) ::close(fd_);
  }

 private:
  static double NowWallNs() {
    struct timespec ts {};
    ::clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<double>(ts.tv_sec) * 1e9 + static_cast<double>(ts.tv_nsec);
  }

  std::string path_;
  int fd_ = -1;
  std::uint8_t* map_ = nullptr;
};

}  // namespace a3_pingpong
