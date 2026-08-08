// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// A3SyncLoop implementation. Algorithm derived from aimrl_sdk::Core::sync_loop_
// (MIT, upstream authors). Extended from 3 to 6 channels and adapted to emit a
// robot_io::RobotState instead of the 88-float frame used by aimrl_sdk.
//
// See notes/a3_backend_plan.md §8 / PR 3.

#include "a3_sync/a3_sync_loop.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <limits>
#include <utility>

#include "robot_io/a3_layout_extra.hpp"

namespace a3_sync {

namespace {

// --- Local helpers (equivalent to aimrl_sdk::core.cpp anonymous namespace) --

std::int64_t NowSystemNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

constexpr int kMaxGroupPairSearchDepth = 16;

std::int64_t AbsDiffNs(std::int64_t a, std::int64_t b) noexcept {
  return (a >= b) ? (a - b) : (b - a);
}

std::int64_t ModNs(std::int64_t value, std::int64_t period) noexcept {
  if (period <= 0) return value;
  auto out = value % period;
  if (out < 0) out += period;
  return out;
}

template <class Sample>
struct Bracket {
  bool has_before{false};
  bool has_after{false};
  Sample before{};
  Sample after{};
};

// Scan the ring newest → oldest looking for the closest sample <= tick.
// Along the way keep track of the closest sample strictly > tick so we can
// interpolate once the pair is found.
template <class Ring, class Sample>
Bracket<Sample> FindBracket(const Ring& ring, TimestampNs tick,
                            std::uint64_t start_idx, int max_backtrack) {
  Bracket<Sample> out;
  Sample tmp{};
  auto idx = start_idx;
  std::int64_t best_after = std::numeric_limits<std::int64_t>::max();
  for (int i = 0; i < max_backtrack && idx > 0; ++i, --idx) {
    if (!ring.read_at(idx, tmp)) continue;
    if (tmp.stamp.value <= 0) continue;
    if (tmp.stamp.value <= tick.value) {
      out.before     = tmp;
      out.has_before = true;
      break;
    }
    if (tmp.stamp.value < best_after) {
      best_after    = tmp.stamp.value;
      out.after     = tmp;
      out.has_after = true;
    }
  }
  return out;
}

template <class Sample>
std::int64_t SampleAgeNs(const Sample& sample, TimestampNs tick) noexcept {
  const auto freshness_stamp =
      (sample.recv_stamp.value > 0) ? sample.recv_stamp : sample.stamp;
  return tick.value - freshness_stamp.value;
}

template <class Sample>
void RecordTransportLatency(A3SyncStatistics& stats,
                            const Sample& sample) noexcept {
  if (!sample.source_stamp_valid) return;
  if (sample.stamp.value <= 0 || sample.recv_stamp.value <= 0) return;
  stats.RecordStateTransportApparentNs(sample.recv_stamp.value -
                                       sample.stamp.value);
}

template <class Sample>
bool SampleHasUsableSourceStamp(const Sample& sample) noexcept {
  return sample.source_stamp_valid && sample.stamp.value > 0;
}

template <class Sample>
bool SampleFreshAtReceiveTime(const Sample& sample, std::int64_t ready_ns,
                              std::int64_t max_sample_age_ns) noexcept {
  if (max_sample_age_ns <= 0) return true;
  const auto freshness_stamp =
      (sample.recv_stamp.value > 0) ? sample.recv_stamp.value
                                    : sample.stamp.value;
  if (freshness_stamp <= 0) return false;
  return ready_ns - freshness_stamp <= max_sample_age_ns;
}

template <class Ring, class Sample>
bool ReadLatestSample(const Ring& ring, Sample& out) {
  const auto idx = ring.latest_index();
  return idx > 0 && ring.read_at(idx, out);
}

template <class Sample>
A3SyncStatistics::ChannelHealth SelectedSampleHealth(
    const Sample& sample, std::int64_t ready_ns,
    std::int64_t max_sample_age_ns) noexcept {
  A3SyncStatistics::ChannelHealth health{};
  health.source_stamp_valid = sample.source_stamp_valid;
  health.source_stamp_ns = sample.stamp.value;
  const auto freshness_stamp =
      (sample.recv_stamp.value > 0) ? sample.recv_stamp.value
                                    : sample.stamp.value;
  health.age_ns = (freshness_stamp > 0) ? (ready_ns - freshness_stamp) : -1;
  if (max_sample_age_ns > 0 &&
      (freshness_stamp <= 0 || health.age_ns > max_sample_age_ns)) {
    health.stale = true;
    return health;
  }
  health.ok = SampleHasUsableSourceStamp(sample);
  return health;
}

template <class Ring, class Sample>
bool FindClosestByStamp(const Ring& ring, std::int64_t target_stamp_ns,
                        int max_backtrack, std::int64_t max_spread_ns,
                        std::int64_t ready_ns,
                        std::int64_t max_sample_age_ns, Sample& out) {
  Sample tmp{};
  auto idx = ring.latest_index();
  bool found = false;
  auto best_diff = std::numeric_limits<std::int64_t>::max();
  std::int64_t best_stamp = std::numeric_limits<std::int64_t>::min();
  for (int i = 0; i < max_backtrack && idx > 0; ++i, --idx) {
    if (!ring.read_at(idx, tmp)) continue;
    if (!SampleHasUsableSourceStamp(tmp)) continue;
    if (!SampleFreshAtReceiveTime(tmp, ready_ns, max_sample_age_ns)) continue;
    const auto diff = AbsDiffNs(tmp.stamp.value, target_stamp_ns);
    if (diff > max_spread_ns) continue;
    if (!found || diff < best_diff ||
        (diff == best_diff && tmp.stamp.value > best_stamp)) {
      out = tmp;
      found = true;
      best_diff = diff;
      best_stamp = tmp.stamp.value;
    }
  }
  return found;
}

struct JointGroupCandidate {
  WaistSample waist{};
  LegSample leg{};
  ArmSample arm{};
  NeckSample neck{};
  std::int64_t stamp_ns{0};
  std::int64_t spread_ns{0};
  std::int64_t latest_recv_ns{0};
};

struct ImuGroupCandidate {
  ImuSample pelvis{};
  ImuSample torso{};
  std::int64_t stamp_ns{0};
  std::int64_t spread_ns{0};
  std::int64_t latest_recv_ns{0};
};

int BuildJointGroupCandidates(
    const RingBuffer<WaistSample>& waist_ring,
    const RingBuffer<LegSample>& leg_ring,
    const RingBuffer<ArmSample>& arm_ring,
    const RingBuffer<NeckSample>& neck_ring,
    const SyncConfig& cfg,
    std::int64_t ready_ns,
    std::array<JointGroupCandidate, kMaxGroupPairSearchDepth>& out) {
  int count = 0;
  WaistSample waist{};
  auto idx = waist_ring.latest_index();
  const int target_count =
      std::min(cfg.group_pair_search_depth, kMaxGroupPairSearchDepth);
  for (int i = 0; i < cfg.max_backtrack && idx > 0 && count < target_count;
       ++i, --idx) {
    if (!waist_ring.read_at(idx, waist)) continue;
    if (!SampleHasUsableSourceStamp(waist)) continue;
    if (!SampleFreshAtReceiveTime(waist, ready_ns, cfg.max_sample_age_ns)) {
      continue;
    }

    LegSample leg{};
    ArmSample arm{};
    NeckSample neck{};
    if (!FindClosestByStamp(leg_ring, waist.stamp.value, cfg.max_backtrack,
                            cfg.max_group_internal_skew_ns, ready_ns,
                            cfg.max_sample_age_ns, leg)) {
      continue;
    }
    if (!FindClosestByStamp(arm_ring, waist.stamp.value, cfg.max_backtrack,
                            cfg.max_group_internal_skew_ns, ready_ns,
                            cfg.max_sample_age_ns, arm)) {
      continue;
    }
    if (!FindClosestByStamp(neck_ring, waist.stamp.value, cfg.max_backtrack,
                            cfg.max_group_internal_skew_ns, ready_ns,
                            cfg.max_sample_age_ns, neck)) {
      continue;
    }

    const auto lo = std::min({waist.stamp.value, leg.stamp.value,
                              arm.stamp.value, neck.stamp.value});
    const auto hi = std::max({waist.stamp.value, leg.stamp.value,
                              arm.stamp.value, neck.stamp.value});
    if (hi - lo > cfg.max_group_internal_skew_ns) continue;

    bool duplicate = false;
    for (int j = 0; j < count; ++j) {
      if (out[j].stamp_ns == hi) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;

    auto& candidate = out[count++];
    candidate.waist = waist;
    candidate.leg = leg;
    candidate.arm = arm;
    candidate.neck = neck;
    candidate.stamp_ns = hi;
    candidate.spread_ns = hi - lo;
    candidate.latest_recv_ns =
        std::max({waist.recv_stamp.value, leg.recv_stamp.value,
                  arm.recv_stamp.value, neck.recv_stamp.value});
  }
  return count;
}

int BuildImuGroupCandidates(
    const RingBuffer<ImuSample>& pelvis_ring,
    const RingBuffer<ImuSample>& torso_ring,
    const SyncConfig& cfg,
    std::int64_t ready_ns,
    std::array<ImuGroupCandidate, kMaxGroupPairSearchDepth>& out) {
  int count = 0;
  ImuSample pelvis{};
  auto idx = pelvis_ring.latest_index();
  const int target_count =
      std::min(cfg.group_pair_search_depth, kMaxGroupPairSearchDepth);
  for (int i = 0; i < cfg.max_backtrack && idx > 0 && count < target_count;
       ++i, --idx) {
    if (!pelvis_ring.read_at(idx, pelvis)) continue;
    if (!SampleHasUsableSourceStamp(pelvis)) continue;
    if (!SampleFreshAtReceiveTime(pelvis, ready_ns, cfg.max_sample_age_ns)) {
      continue;
    }

    ImuSample torso{};
    if (!FindClosestByStamp(torso_ring, pelvis.stamp.value, cfg.max_backtrack,
                            cfg.max_group_internal_skew_ns, ready_ns,
                            cfg.max_sample_age_ns, torso)) {
      continue;
    }

    const auto lo = std::min(pelvis.stamp.value, torso.stamp.value);
    const auto hi = std::max(pelvis.stamp.value, torso.stamp.value);
    if (hi - lo > cfg.max_group_internal_skew_ns) continue;

    bool duplicate = false;
    for (int j = 0; j < count; ++j) {
      if (out[j].stamp_ns == hi) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;

    auto& candidate = out[count++];
    candidate.pelvis = pelvis;
    candidate.torso = torso;
    candidate.stamp_ns = hi;
    candidate.spread_ns = hi - lo;
    candidate.latest_recv_ns =
        std::max(pelvis.recv_stamp.value, torso.recv_stamp.value);
  }
  return count;
}

struct SelectedGroupPair {
  bool valid{false};
  JointGroupCandidate joint{};
  ImuGroupCandidate imu{};
  std::int64_t pair_skew_ns{0};
  std::int64_t sync_stamp_ns{0};
  std::int64_t latest_recv_ns{0};
};

SelectedGroupPair SelectMinSkewPair(
    const std::array<JointGroupCandidate, kMaxGroupPairSearchDepth>& joints,
    int joint_count,
    const std::array<ImuGroupCandidate, kMaxGroupPairSearchDepth>& imus,
    int imu_count,
    std::int64_t min_sync_stamp_ns = 0,
    std::int64_t max_aligned_pair_skew_ns = 0) {
  SelectedGroupPair best{};
  bool best_aligned = false;
  for (int j = 0; j < joint_count; ++j) {
    for (int i = 0; i < imu_count; ++i) {
      const auto pair_skew = AbsDiffNs(joints[j].stamp_ns, imus[i].stamp_ns);
      const auto sync_stamp = std::max(joints[j].stamp_ns, imus[i].stamp_ns);
      if (min_sync_stamp_ns > 0 && sync_stamp < min_sync_stamp_ns) {
        continue;
      }
      const auto latest_recv =
          std::max(joints[j].latest_recv_ns, imus[i].latest_recv_ns);
      const bool aligned_candidate =
          max_aligned_pair_skew_ns <= 0 ||
          pair_skew <= max_aligned_pair_skew_ns;

      bool take = !best.valid;
      if (best.valid && aligned_candidate != best_aligned) {
        take = aligned_candidate;
      } else if (best.valid && aligned_candidate) {
        take = latest_recv > best.latest_recv_ns ||
               (latest_recv == best.latest_recv_ns &&
                (sync_stamp > best.sync_stamp_ns ||
                 (sync_stamp == best.sync_stamp_ns &&
                  pair_skew < best.pair_skew_ns)));
      } else if (best.valid) {
        take = pair_skew < best.pair_skew_ns ||
               (pair_skew == best.pair_skew_ns &&
                (sync_stamp > best.sync_stamp_ns ||
                 (sync_stamp == best.sync_stamp_ns &&
                  latest_recv > best.latest_recv_ns)));
      }

      if (take) {
        best.valid = true;
        best_aligned = aligned_candidate;
        best.joint = joints[j];
        best.imu = imus[i];
        best.pair_skew_ns = pair_skew;
        best.sync_stamp_ns = sync_stamp;
        best.latest_recv_ns = latest_recv;
      }
    }
  }
  return best;
}

template <class JointS>
A3SyncStatistics::ChannelHealth ResampleJointAt(
    const RingBuffer<JointS>& ring, TimestampNs tick,
    int max_backtrack, std::int64_t max_sample_age_ns, JointS& out) {
  A3SyncStatistics::ChannelHealth health{};
  const auto br = FindBracket<RingBuffer<JointS>, JointS>(
      ring, tick, ring.latest_index(), max_backtrack);
  if (!br.has_before) return health;
  health.source_stamp_valid = br.before.source_stamp_valid;
  health.source_stamp_ns = br.before.stamp.value;
  health.age_ns = SampleAgeNs(br.before, tick);
  if (max_sample_age_ns > 0 && health.age_ns > max_sample_age_ns) {
    health.stale = true;
    return health;
  }
  if (br.has_after &&
      br.after.stamp.value > br.before.stamp.value &&
      br.before.stamp.value <= tick.value &&
      tick.value <= br.after.stamp.value) {
    const double denom =
        static_cast<double>(br.after.stamp.value - br.before.stamp.value);
    const double num =
        static_cast<double>(tick.value - br.before.stamp.value);
    const double alpha = std::clamp(denom > 0.0 ? num / denom : 0.0, 0.0, 1.0);
    out = br.before;
    out.stamp = tick;
    out.recv_stamp.value = std::max(br.before.recv_stamp.value,
                                    br.after.recv_stamp.value);
    LerpArrays(br.before.pos, br.after.pos, alpha, out.pos);
    LerpArrays(br.before.vel, br.after.vel, alpha, out.vel);
    LerpArrays(br.before.eff, br.after.eff, alpha, out.eff);
    health.ok = true;
    health.interpolated = true;
    return health;
  }
  // Tail case: only a sample <= tick exists (ZOH / hold).
  out = br.before;
  health.ok = true;
  health.held = true;
  return health;
}

A3SyncStatistics::ChannelHealth ResampleImuAt(
    const RingBuffer<ImuSample>& ring, TimestampNs tick,
    int max_backtrack, std::int64_t max_sample_age_ns, ImuSample& out) {
  A3SyncStatistics::ChannelHealth health{};
  const auto br = FindBracket<RingBuffer<ImuSample>, ImuSample>(
      ring, tick, ring.latest_index(), max_backtrack);
  if (!br.has_before) return health;
  health.source_stamp_valid = br.before.source_stamp_valid;
  health.source_stamp_ns = br.before.stamp.value;
  health.age_ns = SampleAgeNs(br.before, tick);
  if (max_sample_age_ns > 0 && health.age_ns > max_sample_age_ns) {
    health.stale = true;
    return health;
  }
  if (br.has_after &&
      br.after.stamp.value > br.before.stamp.value &&
      br.before.stamp.value <= tick.value &&
      tick.value <= br.after.stamp.value) {
    const double denom =
        static_cast<double>(br.after.stamp.value - br.before.stamp.value);
    const double num =
        static_cast<double>(tick.value - br.before.stamp.value);
    const double alpha = std::clamp(denom > 0.0 ? num / denom : 0.0, 0.0, 1.0);
    out = br.before;
    out.stamp = tick;
    out.recv_stamp.value = std::max(br.before.recv_stamp.value,
                                    br.after.recv_stamp.value);
    out.quat_wxyz = SlerpQuatWxyz(br.before.quat_wxyz, br.after.quat_wxyz, alpha);
    LerpArrays(br.before.gyro, br.after.gyro, alpha, out.gyro);
    LerpArrays(br.before.acc,  br.after.acc,  alpha, out.acc);
    health.ok = true;
    health.interpolated = true;
    return health;
  }
  out = br.before;
  health.ok = true;
  health.held = true;
  return health;
}

}  // namespace

// --- Public interpolation helpers ------------------------------------------

double Lerp(double a, double b, double t) noexcept { return a + (b - a) * t; }

std::array<double, 4> SlerpQuatWxyz(const std::array<double, 4>& q0_wxyz,
                                    const std::array<double, 4>& q1_wxyz,
                                    double t) {
  auto normalise = [](std::array<double, 4> q) {
    const double n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 <= 0.0) return std::array<double, 4>{1.0, 0.0, 0.0, 0.0};
    const double inv = 1.0 / std::sqrt(n2);
    for (auto& v : q) v *= inv;
    return q;
  };

  auto qa = normalise(q0_wxyz);
  auto qb = normalise(q1_wxyz);

  double dot = qa[0]*qb[0] + qa[1]*qb[1] + qa[2]*qb[2] + qa[3]*qb[3];
  if (dot < 0.0) {
    dot = -dot;
    for (auto& v : qb) v = -v;
  }

  constexpr double kDotThreshold = 0.9995;
  std::array<double, 4> out{};
  if (dot > kDotThreshold) {
    for (int i = 0; i < 4; ++i) out[i] = qa[i] + (qb[i] - qa[i]) * t;
    return normalise(out);
  }
  dot = std::clamp(dot, -1.0, 1.0);
  const double theta0     = std::acos(dot);
  const double theta      = theta0 * t;
  const double sin_theta0 = std::sin(theta0);
  const double sin_theta  = std::sin(theta);
  const double s0 = std::cos(theta) - dot * (sin_theta / sin_theta0);
  const double s1 = sin_theta / sin_theta0;
  for (int i = 0; i < 4; ++i) out[i] = qa[i] * s0 + qb[i] * s1;
  return normalise(out);
}

// --- AssembleState ----------------------------------------------------------
//
// Populates a 31-DOF RobotState in MuJoCo real DOF order (matches
// MakeA3Layout31 post 2026-04 realignment — see notes/a3_dof_orderings.md):
//   [0..2]   waist  (← waist_sample[0..2])
//   [3..4]   neck   (← neck_sample[0..1])
//   [5..11]  L_arm  (← arm_sample[0..6])
//   [12..18] R_arm  (← arm_sample[7..13])
//   [19..24] L_leg  (← leg_sample[0..5])
//   [25..30] R_leg  (← leg_sample[6..11])
// The sample↔slice correspondence is 1:1 because kA3{Waist,Leg,Arm,Neck}JointNames
// (layouts.cpp / a3_layout_extra.cpp) list joints in the same order MuJoCo uses
// for each slice. Verified joint-by-joint 2026-04-28.
//
// pelvis → imu_*, torso → sec_imu_*.
robot_io::RobotState AssembleState(
    std::int64_t tick_ns,
    std::int64_t tick_seq,
    const WaistSample& waist, bool /*waist_ok*/,
    const LegSample&   leg,   bool /*leg_ok*/,
    const ArmSample&   arm,   bool /*arm_ok*/,
    const NeckSample&  neck,  bool /*neck_ok*/,
    const ImuSample&   pelvis, bool /*pelvis_ok*/,
    const ImuSample&   torso,  bool torso_ok,
    bool complete,
    bool aligned,
    std::int64_t skew_ns) {
  robot_io::RobotState state;
  state.timestamp_ns = tick_ns;
  state.tick         = tick_seq;
  state.sync_complete = complete;
  state.sync_aligned  = aligned;
  state.sync_skew_ns  = skew_ns;

  constexpr int kDof = robot_io::kA3Dof;  // 31
  state.q.resize(kDof);
  state.dq.resize(kDof);
  state.tau_est.resize(kDof);

  // waist [0..2]
  for (int i = 0; i < 3; ++i) {
    state.q(i)       = waist.pos[i];
    state.dq(i)      = waist.vel[i];
    state.tau_est(i) = waist.eff[i];
  }
  // neck [3..4]
  for (int i = 0; i < 2; ++i) {
    state.q(3 + i)       = neck.pos[i];
    state.dq(3 + i)      = neck.vel[i];
    state.tau_est(3 + i) = neck.eff[i];
  }
  // left arm  [5..11]  ← arm_sample[0..6]
  for (int i = 0; i < 7; ++i) {
    state.q(5 + i)       = arm.pos[i];
    state.dq(5 + i)      = arm.vel[i];
    state.tau_est(5 + i) = arm.eff[i];
  }
  // right arm [12..18] ← arm_sample[7..13]
  for (int i = 0; i < 7; ++i) {
    state.q(12 + i)       = arm.pos[7 + i];
    state.dq(12 + i)      = arm.vel[7 + i];
    state.tau_est(12 + i) = arm.eff[7 + i];
  }
  // left leg  [19..24] ← leg_sample[0..5]
  for (int i = 0; i < 6; ++i) {
    state.q(19 + i)       = leg.pos[i];
    state.dq(19 + i)      = leg.vel[i];
    state.tau_est(19 + i) = leg.eff[i];
  }
  // right leg [25..30] ← leg_sample[6..11]
  for (int i = 0; i < 6; ++i) {
    state.q(25 + i)       = leg.pos[6 + i];
    state.dq(25 + i)      = leg.vel[6 + i];
    state.tau_est(25 + i) = leg.eff[6 + i];
  }

  state.imu_quat_wxyz = Eigen::Vector4d(
      pelvis.quat_wxyz[0], pelvis.quat_wxyz[1],
      pelvis.quat_wxyz[2], pelvis.quat_wxyz[3]);
  state.imu_gyro  = Eigen::Vector3d(pelvis.gyro[0], pelvis.gyro[1], pelvis.gyro[2]);
  state.imu_accel = Eigen::Vector3d(pelvis.acc[0],  pelvis.acc[1],  pelvis.acc[2]);

  state.has_secondary_imu = torso_ok;
  state.sec_imu_quat_wxyz = Eigen::Vector4d(
      torso.quat_wxyz[0], torso.quat_wxyz[1],
      torso.quat_wxyz[2], torso.quat_wxyz[3]);
  state.sec_imu_gyro  = Eigen::Vector3d(torso.gyro[0], torso.gyro[1], torso.gyro[2]);
  state.sec_imu_accel = Eigen::Vector3d(torso.acc[0],  torso.acc[1],  torso.acc[2]);

  return state;
}

// --- A3SyncLoop -------------------------------------------------------------

A3SyncLoop::A3SyncLoop(Options opt)
    : opt_(std::move(opt)),
      waist_ring_(opt_.waist_ring),
      leg_ring_(opt_.leg_ring),
      arm_ring_(opt_.arm_ring),
      neck_ring_(opt_.neck_ring),
      pelvis_ring_(opt_.pelvis_imu_ring),
      torso_ring_(opt_.torso_imu_ring) {}

A3SyncLoop::~A3SyncLoop() { Stop(); }

void A3SyncLoop::Start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) return;
  if (opt_.sync.sync_mode == SyncMode::LatestFrame) return;
  thread_ = std::thread([this] { SyncThread(); });
}

void A3SyncLoop::Stop() {
  running_.store(false, std::memory_order_relaxed);
  sample_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

bool A3SyncLoop::SetPhaseNsForStartup(std::int64_t phase_ns) {
  if (running_.load(std::memory_order_relaxed)) return false;
  opt_.sync.phase_ns = phase_ns;
  return true;
}

bool A3SyncLoop::EstimateAutoPhaseNs(std::int64_t release_margin_ns,
                                     std::int64_t& phase_ns,
                                     std::int64_t& latest_recv_ns,
                                     std::int64_t& pair_skew_ns) const {
  if (opt_.sync.sync_mode != SyncMode::MinSkewPair) return false;
  const auto period_ns =
      static_cast<std::int64_t>(1e9 / opt_.sync.sync_hz);
  if (period_ns <= 0) return false;

  const auto ready_ns = NowSystemNs();
  std::array<JointGroupCandidate, kMaxGroupPairSearchDepth> joint_groups{};
  std::array<ImuGroupCandidate, kMaxGroupPairSearchDepth> imu_groups{};
  const int joint_count = BuildJointGroupCandidates(
      waist_ring_, leg_ring_, arm_ring_, neck_ring_, opt_.sync, ready_ns,
      joint_groups);
  const int imu_count = BuildImuGroupCandidates(
      pelvis_ring_, torso_ring_, opt_.sync, ready_ns, imu_groups);
  const auto pair = SelectMinSkewPair(joint_groups, joint_count,
                                      imu_groups, imu_count, 0,
                                      opt_.sync.max_group_pair_skew_ns);
  if (!pair.valid || pair.latest_recv_ns <= 0 ||
      pair.pair_skew_ns > opt_.sync.max_group_pair_skew_ns) {
    return false;
  }

  auto align_delay_ns =
      (opt_.sync.sync_mode == SyncMode::MinSkewPair)
          ? opt_.sync.sync_ready_after_input_ns
          : opt_.sync.align_delay_ns;
  if (align_delay_ns < 0) align_delay_ns = 0;
  if (release_margin_ns < 0) release_margin_ns = 0;

  latest_recv_ns = pair.latest_recv_ns;
  pair_skew_ns = pair.pair_skew_ns;
  phase_ns = ModNs(pair.latest_recv_ns + release_margin_ns - align_delay_ns,
                   period_ns);
  return true;
}

void A3SyncLoop::NotifySampleAvailable() {
  sample_seq_.fetch_add(1, std::memory_order_release);
  sample_cv_.notify_one();
}

void A3SyncLoop::OnWaistState(const WaistSample& s) {
  stats_.RecordLatestWaistRaw(s.source_stamp_valid, s.stamp.value,
                              s.source_sequence_valid, s.source_sequence);
  RecordTransportLatency(stats_, s);
  if (opt_.sync.sync_mode == SyncMode::LatestFrame) {
    robot_io::RobotState state;
    StateCallback cb_copy;
    bool emit = false;
    {
      std::lock_guard<std::mutex> lk(sample_mtx_);
      waist_ring_.write([&](WaistSample& slot) { slot = s; });
      if (running_.load(std::memory_order_relaxed)) {
        emit = TryBuildLatestFrameLocked(state);
        if (emit) {
          std::lock_guard<std::mutex> cb_lk(cb_mtx_);
          cb_copy = state_cb_;
        }
      }
    }
    if (emit && cb_copy) cb_copy(state);
    return;
  }
  waist_ring_.write([&](WaistSample& slot) { slot = s; });
  NotifySampleAvailable();
}
void A3SyncLoop::OnLegState(const LegSample& s) {
  stats_.RecordLatestLegRaw(s.source_stamp_valid, s.stamp.value,
                            s.source_sequence_valid, s.source_sequence);
  RecordTransportLatency(stats_, s);
  if (opt_.sync.sync_mode == SyncMode::LatestFrame) {
    robot_io::RobotState state;
    StateCallback cb_copy;
    bool emit = false;
    {
      std::lock_guard<std::mutex> lk(sample_mtx_);
      leg_ring_.write([&](LegSample& slot) { slot = s; });
      if (running_.load(std::memory_order_relaxed)) {
        emit = TryBuildLatestFrameLocked(state);
        if (emit) {
          std::lock_guard<std::mutex> cb_lk(cb_mtx_);
          cb_copy = state_cb_;
        }
      }
    }
    if (emit && cb_copy) cb_copy(state);
    return;
  }
  leg_ring_.write([&](LegSample& slot) { slot = s; });
  NotifySampleAvailable();
}
void A3SyncLoop::OnArmState(const ArmSample& s) {
  stats_.RecordLatestArmRaw(s.source_stamp_valid, s.stamp.value,
                            s.source_sequence_valid, s.source_sequence);
  RecordTransportLatency(stats_, s);
  if (opt_.sync.sync_mode == SyncMode::LatestFrame) {
    robot_io::RobotState state;
    StateCallback cb_copy;
    bool emit = false;
    {
      std::lock_guard<std::mutex> lk(sample_mtx_);
      arm_ring_.write([&](ArmSample& slot) { slot = s; });
      if (running_.load(std::memory_order_relaxed)) {
        emit = TryBuildLatestFrameLocked(state);
        if (emit) {
          std::lock_guard<std::mutex> cb_lk(cb_mtx_);
          cb_copy = state_cb_;
        }
      }
    }
    if (emit && cb_copy) cb_copy(state);
    return;
  }
  arm_ring_.write([&](ArmSample& slot) { slot = s; });
  NotifySampleAvailable();
}
void A3SyncLoop::OnNeckState(const NeckSample& s) {
  stats_.RecordLatestNeckRaw(s.source_stamp_valid, s.stamp.value,
                             s.source_sequence_valid, s.source_sequence);
  RecordTransportLatency(stats_, s);
  if (opt_.sync.sync_mode == SyncMode::LatestFrame) {
    robot_io::RobotState state;
    StateCallback cb_copy;
    bool emit = false;
    {
      std::lock_guard<std::mutex> lk(sample_mtx_);
      neck_ring_.write([&](NeckSample& slot) { slot = s; });
      if (running_.load(std::memory_order_relaxed)) {
        emit = TryBuildLatestFrameLocked(state);
        if (emit) {
          std::lock_guard<std::mutex> cb_lk(cb_mtx_);
          cb_copy = state_cb_;
        }
      }
    }
    if (emit && cb_copy) cb_copy(state);
    return;
  }
  neck_ring_.write([&](NeckSample& slot) { slot = s; });
  NotifySampleAvailable();
}
void A3SyncLoop::OnPelvisImu(const ImuSample& s) {
  stats_.RecordLatestPelvisImuRaw(s.source_stamp_valid, s.stamp.value);
  RecordTransportLatency(stats_, s);
  if (opt_.sync.sync_mode == SyncMode::LatestFrame) {
    robot_io::RobotState state;
    StateCallback cb_copy;
    bool emit = false;
    {
      std::lock_guard<std::mutex> lk(sample_mtx_);
      pelvis_ring_.write([&](ImuSample& slot) { slot = s; });
      if (running_.load(std::memory_order_relaxed)) {
        emit = TryBuildLatestFrameLocked(state);
        if (emit) {
          std::lock_guard<std::mutex> cb_lk(cb_mtx_);
          cb_copy = state_cb_;
        }
      }
    }
    if (emit && cb_copy) cb_copy(state);
    return;
  }
  pelvis_ring_.write([&](ImuSample& slot) { slot = s; });
  NotifySampleAvailable();
}
void A3SyncLoop::OnTorsoImu(const ImuSample& s) {
  stats_.RecordLatestTorsoImuRaw(s.source_stamp_valid, s.stamp.value);
  RecordTransportLatency(stats_, s);
  if (opt_.sync.sync_mode == SyncMode::LatestFrame) {
    robot_io::RobotState state;
    StateCallback cb_copy;
    bool emit = false;
    {
      std::lock_guard<std::mutex> lk(sample_mtx_);
      torso_ring_.write([&](ImuSample& slot) { slot = s; });
      if (running_.load(std::memory_order_relaxed)) {
        emit = TryBuildLatestFrameLocked(state);
        if (emit) {
          std::lock_guard<std::mutex> cb_lk(cb_mtx_);
          cb_copy = state_cb_;
        }
      }
    }
    if (emit && cb_copy) cb_copy(state);
    return;
  }
  torso_ring_.write([&](ImuSample& slot) { slot = s; });
  NotifySampleAvailable();
}

void A3SyncLoop::RegisterStateCallback(StateCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mtx_);
  state_cb_ = std::move(cb);
}

bool A3SyncLoop::TryBuildLatestFrameLocked(robot_io::RobotState& state) {
  WaistSample waist{};
  LegSample leg{};
  ArmSample arm{};
  NeckSample neck{};
  ImuSample pelvis{};
  ImuSample torso{};
  if (!ReadLatestSample(waist_ring_, waist) ||
      !ReadLatestSample(leg_ring_, leg) ||
      !ReadLatestSample(arm_ring_, arm) ||
      !ReadLatestSample(neck_ring_, neck) ||
      !ReadLatestSample(pelvis_ring_, pelvis) ||
      !ReadLatestSample(torso_ring_, torso)) {
    return false;
  }

  const auto ready_ns = NowSystemNs();
  auto waist_health =
      SelectedSampleHealth(waist, ready_ns, opt_.sync.max_sample_age_ns);
  auto leg_health =
      SelectedSampleHealth(leg, ready_ns, opt_.sync.max_sample_age_ns);
  auto arm_health =
      SelectedSampleHealth(arm, ready_ns, opt_.sync.max_sample_age_ns);
  auto neck_health =
      SelectedSampleHealth(neck, ready_ns, opt_.sync.max_sample_age_ns);
  auto pelvis_health =
      SelectedSampleHealth(pelvis, ready_ns, opt_.sync.max_sample_age_ns);
  auto torso_health =
      SelectedSampleHealth(torso, ready_ns, opt_.sync.max_sample_age_ns);

  const bool complete =
      waist_health.ok && leg_health.ok && arm_health.ok &&
      neck_health.ok && pelvis_health.ok && torso_health.ok;
  if (!complete) {
    return false;
  }

  const auto source_hi =
      std::max({waist_health.source_stamp_ns, leg_health.source_stamp_ns,
                arm_health.source_stamp_ns, neck_health.source_stamp_ns,
                pelvis_health.source_stamp_ns, torso_health.source_stamp_ns});
  const auto source_lo =
      std::min({waist_health.source_stamp_ns, leg_health.source_stamp_ns,
                arm_health.source_stamp_ns, neck_health.source_stamp_ns,
                pelvis_health.source_stamp_ns, torso_health.source_stamp_ns});
  const auto skew_ns = source_hi - source_lo;
  const bool aligned = skew_ns <= opt_.sync.max_skew_ns;
  const auto latest_recv_ns =
      std::max({waist.recv_stamp.value, leg.recv_stamp.value,
                arm.recv_stamp.value, neck.recv_stamp.value,
                pelvis.recv_stamp.value, torso.recv_stamp.value});

  if (latest_recv_ns > 0) {
    stats_.RecordStateReadyNs(ready_ns - latest_recv_ns);
  }
  stats_.RecordStateHeaderSkewNs(skew_ns);

  const auto seq = tick_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
  stats_.OnTick(source_lo, complete, aligned, skew_ns,
                waist_health, leg_health, arm_health, neck_health,
                pelvis_health, torso_health);

  state = AssembleState(source_lo, seq,
                        waist, waist_health.ok,
                        leg, leg_health.ok,
                        arm, arm_health.ok,
                        neck, neck_health.ok,
                        pelvis, pelvis_health.ok,
                        torso, torso_health.ok,
                        complete, aligned, skew_ns);
  state.state_data_ready_ns = latest_recv_ns;
  state.state_sync_ready_ns = ready_ns;
  return true;
}

void A3SyncLoop::SyncThread() {
  const auto period_ns =
      static_cast<std::int64_t>(1e9 / opt_.sync.sync_hz);
  if (period_ns <= 0) return;

  auto phase_ns = opt_.sync.phase_ns;
  if (phase_ns < 0) phase_ns = 0;
  if (phase_ns >= period_ns) phase_ns = phase_ns % period_ns;
  auto align_delay_ns =
      (opt_.sync.sync_mode == SyncMode::MinSkewPair)
          ? opt_.sync.sync_ready_after_input_ns
          : opt_.sync.align_delay_ns;
  if (align_delay_ns < 0) align_delay_ns = 0;

  std::int64_t last_min_skew_emit_stamp_ns = 0;

  // Initialise the tick grid: first tick strictly in the future.
  const auto start_ns = NowSystemNs();
  auto next_tick_ns = (start_ns / period_ns) * period_ns + phase_ns;
  if (next_tick_ns <= start_ns) next_tick_ns += period_ns;

  while (running_.load(std::memory_order_relaxed)) {
    TimestampNs tick{next_tick_ns};
    next_tick_ns += period_ns;

    // Fast-forward if we're behind schedule (drop missed ticks).
    const auto now0 = NowSystemNs();
    const auto align_ref = (now0 > align_delay_ns) ? (now0 - align_delay_ns) : 0;
    const auto newest_base = (align_ref / period_ns) * period_ns;
    const auto newest_tick = newest_base + phase_ns;
    if (newest_tick > tick.value) {
      tick.value = newest_tick;
      next_tick_ns = tick.value + period_ns;
    }

    // Wait until tick + align_delay_ns so that post-tick samples are available.
    const auto release_ns = tick.value + align_delay_ns;
    const auto now_ns = NowSystemNs();
    if (release_ns > now_ns) {
      std::this_thread::sleep_until(
          std::chrono::system_clock::time_point(std::chrono::nanoseconds(release_ns)));
    }
    if (!running_.load(std::memory_order_relaxed)) break;

    if (opt_.sync.sync_mode == SyncMode::MinSkewPair) {
      const auto ready_ns = NowSystemNs();
      std::array<JointGroupCandidate, kMaxGroupPairSearchDepth> joint_groups{};
      std::array<ImuGroupCandidate, kMaxGroupPairSearchDepth> imu_groups{};
      const int joint_count = BuildJointGroupCandidates(
          waist_ring_, leg_ring_, arm_ring_, neck_ring_, opt_.sync, ready_ns,
          joint_groups);
      const int imu_count = BuildImuGroupCandidates(
          pelvis_ring_, torso_ring_, opt_.sync, ready_ns, imu_groups);
      const auto min_sync_stamp_ns =
          (last_min_skew_emit_stamp_ns > 0)
              ? (last_min_skew_emit_stamp_ns + 1)
              : 0;
      const auto pair = SelectMinSkewPair(joint_groups, joint_count,
                                          imu_groups, imu_count,
                                          min_sync_stamp_ns,
                                          opt_.sync.max_group_pair_skew_ns);
      if (!pair.valid) continue;

      WaistSample waist = pair.joint.waist;
      LegSample leg = pair.joint.leg;
      ArmSample arm = pair.joint.arm;
      NeckSample neck = pair.joint.neck;
      ImuSample pelvis = pair.imu.pelvis;
      ImuSample torso = pair.imu.torso;

      auto waist_health = SelectedSampleHealth(
          waist, ready_ns, opt_.sync.max_sample_age_ns);
      auto leg_health = SelectedSampleHealth(
          leg, ready_ns, opt_.sync.max_sample_age_ns);
      auto arm_health = SelectedSampleHealth(
          arm, ready_ns, opt_.sync.max_sample_age_ns);
      auto neck_health = SelectedSampleHealth(
          neck, ready_ns, opt_.sync.max_sample_age_ns);
      auto pelvis_health = SelectedSampleHealth(
          pelvis, ready_ns, opt_.sync.max_sample_age_ns);
      auto torso_health = SelectedSampleHealth(
          torso, ready_ns, opt_.sync.max_sample_age_ns);

      const bool complete =
          waist_health.ok && leg_health.ok && arm_health.ok &&
          neck_health.ok && pelvis_health.ok && torso_health.ok;
      const auto skew_ns = pair.pair_skew_ns;
      const bool aligned =
          complete && skew_ns <= opt_.sync.max_group_pair_skew_ns;
      const auto state_stamp_ns = pair.sync_stamp_ns;
      const auto latest_recv_ns = pair.latest_recv_ns;
      if (latest_recv_ns > 0) {
        stats_.RecordStateReadyNs(ready_ns - latest_recv_ns);
      }
      stats_.RecordGroupPairSkewNs(skew_ns);

      const auto seq =
          tick_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
      stats_.OnTick(state_stamp_ns, complete, aligned, skew_ns,
                    waist_health, leg_health, arm_health, neck_health,
                    pelvis_health, torso_health);

      auto state = AssembleState(state_stamp_ns, seq,
                                 waist, waist_health.ok,
                                 leg, leg_health.ok,
                                 arm, arm_health.ok,
                                 neck, neck_health.ok,
                                 pelvis, pelvis_health.ok,
                                 torso, torso_health.ok,
                                 complete, aligned, skew_ns);
      if (complete) {
        state.state_data_ready_ns = latest_recv_ns;
        state.state_sync_ready_ns = ready_ns;
      }

      StateCallback cb_copy;
      {
        std::lock_guard<std::mutex> lk(cb_mtx_);
        cb_copy = state_cb_;
      }
      if (cb_copy) cb_copy(state);

      last_min_skew_emit_stamp_ns = pair.sync_stamp_ns;
      continue;
    }

    // Resample every channel at `tick`.
    WaistSample waist{};
    LegSample   leg{};
    ArmSample   arm{};
    NeckSample  neck{};
    ImuSample   pelvis{};
    ImuSample   torso{};

    const auto waist_health  = ResampleJointAt(waist_ring_, tick, opt_.sync.max_backtrack,
                                               opt_.sync.max_sample_age_ns, waist);
    const auto leg_health    = ResampleJointAt(leg_ring_,   tick, opt_.sync.max_backtrack,
                                               opt_.sync.max_sample_age_ns, leg);
    const auto arm_health    = ResampleJointAt(arm_ring_,   tick, opt_.sync.max_backtrack,
                                               opt_.sync.max_sample_age_ns, arm);
    const auto neck_health   = ResampleJointAt(neck_ring_,  tick, opt_.sync.max_backtrack,
                                               opt_.sync.max_sample_age_ns, neck);
    const auto pelvis_health = ResampleImuAt  (pelvis_ring_, tick, opt_.sync.max_backtrack,
                                               opt_.sync.max_sample_age_ns, pelvis);
    const auto torso_health  = ResampleImuAt  (torso_ring_,  tick, opt_.sync.max_backtrack,
                                               opt_.sync.max_sample_age_ns, torso);

    const bool waist_ok  = waist_health.ok;
    const bool leg_ok    = leg_health.ok;
    const bool arm_ok    = arm_health.ok;
    const bool neck_ok   = neck_health.ok;
    const bool pelvis_ok = pelvis_health.ok;
    const bool torso_ok  = torso_health.ok;

    const bool complete =
        waist_ok && leg_ok && arm_ok && neck_ok && pelvis_ok && torso_ok;

    const auto ready_ns = NowSystemNs();
    std::int64_t latest_recv_ns = 0;
    if (complete) {
      latest_recv_ns = std::max(
          {waist.recv_stamp.value, leg.recv_stamp.value, arm.recv_stamp.value,
           neck.recv_stamp.value, pelvis.recv_stamp.value,
           torso.recv_stamp.value});
      if (latest_recv_ns > 0) {
        stats_.RecordStateReadyNs(ready_ns - latest_recv_ns);
      }
      const bool all_source_stamped =
          waist_health.source_stamp_valid && leg_health.source_stamp_valid &&
          arm_health.source_stamp_valid && neck_health.source_stamp_valid &&
          pelvis_health.source_stamp_valid && torso_health.source_stamp_valid;
      if (all_source_stamped) {
        const auto source_hi =
            std::max({waist_health.source_stamp_ns, leg_health.source_stamp_ns,
                      arm_health.source_stamp_ns, neck_health.source_stamp_ns,
                      pelvis_health.source_stamp_ns,
                      torso_health.source_stamp_ns});
        const auto source_lo =
            std::min({waist_health.source_stamp_ns, leg_health.source_stamp_ns,
                      arm_health.source_stamp_ns, neck_health.source_stamp_ns,
                      pelvis_health.source_stamp_ns,
                      torso_health.source_stamp_ns});
        stats_.RecordStateHeaderSkewNs(source_hi - source_lo);
      }
      const auto interp_count =
          static_cast<std::uint64_t>(waist_health.interpolated) +
          static_cast<std::uint64_t>(leg_health.interpolated) +
          static_cast<std::uint64_t>(arm_health.interpolated) +
          static_cast<std::uint64_t>(neck_health.interpolated) +
          static_cast<std::uint64_t>(pelvis_health.interpolated) +
          static_cast<std::uint64_t>(torso_health.interpolated);
      const auto hold_count =
          static_cast<std::uint64_t>(waist_health.held) +
          static_cast<std::uint64_t>(leg_health.held) +
          static_cast<std::uint64_t>(arm_health.held) +
          static_cast<std::uint64_t>(neck_health.held) +
          static_cast<std::uint64_t>(pelvis_health.held) +
          static_cast<std::uint64_t>(torso_health.held);
      stats_.RecordResampleSamples(interp_count, hold_count);
    }

    std::int64_t skew_ns = 0;
    bool aligned = false;
    if (complete) {
      const auto hi = std::max({waist.stamp.value, leg.stamp.value,
                                arm.stamp.value,   neck.stamp.value,
                                pelvis.stamp.value, torso.stamp.value});
      const auto lo = std::min({waist.stamp.value, leg.stamp.value,
                                arm.stamp.value,   neck.stamp.value,
                                pelvis.stamp.value, torso.stamp.value});
      skew_ns = hi - lo;
      aligned = (skew_ns <= opt_.sync.max_skew_ns);
    }

    const auto seq = tick_seq_.fetch_add(1, std::memory_order_relaxed) + 1;

    stats_.OnTick(tick.value, complete, aligned, skew_ns,
                  waist_health, leg_health, arm_health, neck_health,
                  pelvis_health, torso_health);

    auto state = AssembleState(tick.value, seq,
                               waist, waist_ok,
                               leg,   leg_ok,
                               arm,   arm_ok,
                               neck,  neck_ok,
                               pelvis, pelvis_ok,
                               torso,  torso_ok,
                               complete, aligned, skew_ns);
    if (complete) {
      state.state_data_ready_ns = latest_recv_ns;
      state.state_sync_ready_ns = ready_ns;
    }

    StateCallback cb_copy;
    {
      std::lock_guard<std::mutex> lk(cb_mtx_);
      cb_copy = state_cb_;
    }
    if (cb_copy) cb_copy(state);
  }
}

}  // namespace a3_sync
