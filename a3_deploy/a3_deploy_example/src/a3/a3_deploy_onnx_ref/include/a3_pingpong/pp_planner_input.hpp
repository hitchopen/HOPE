// ============================================================================
//  Live planner inputs for the ping-pong runner
//  (racket target + base pose + pre-serve ball state).
// ============================================================================
//  These are the OFFICIAL-path replacement for ScriptedTarget: instead of a
//  fixed front-right TEST target baked into PpPolicyConfig, a real planner feeds
//  (pos/vel/side/time-to-strike) here and a mocap localizer feeds the base pose.
//
//  DELIVERY: the A3AimrtBackend subscribes std_msgs/Float64MultiArray over the
//  AimRT *ros2* backend (the pure-iceoryx body-drive path is untouched) and
//  calls SetFromFlat(...) on these holders from the subscriber thread. PpPolicy
//  reads the latest value from the 50 Hz driver thread. Both are lock-guarded.
//
//  WHY std_msgs/Float64MultiArray and not hope_msgs/RacketCommand: subscribing
//  the custom hope_msgs type would require its rosidl typesupport vendored+built
//  for aarch64 (the exact G1 pain we are avoiding). std_msgs is core ROS, already
//  in the runner's link closure, so the aarch64 cross-build needs no new msg pkg.
//
//  FRESHNESS: local receipt age is measured with CLOCK_MONOTONIC. For schema 2,
//  the absolute strike deadline is converted once at receipt into the MDU's
//  monotonic domain; later countdown never follows CLOCK_REALTIME corrections.
//  Cross-host wall-clock deltas remain audit-only and never gate a command.
//
//  Flat wire layouts (indices are fixed; element [0] is a schema tag):
//    RACKET legacy /racket/command_flat   (>=11 doubles)
//      [0]=schema(1)  [1]=valid(0/1)  [2]=swing_sign(+1 fore/-1 back)
//      [3..5]=pos_w(x,y,z)  [6..8]=vel_w(x,y,z)
//      [9]=time_to_strike(s)  [10]=strike_time(s, informational)
//      [11]=frame_code(0=world/table, 1=base_link)   (optional; default 0)
//    RACKET revisioned /racket/command_flat (19 doubles)
//      [0]=schema(2) [1]=valid [2]=swing_sign [3..5]=pos [6..8]=vel
//      [9]=time_to_strike [10]=absolute strike wall time [11]=frame_code
//      [12]=producer_sec [13]=producer_nsec [14]=command_seq
//      [15]=flight_id [16]=revision_id [17]=estimator_sample_count
//      [18]=estimator_span_s
//    BASE legacy /a3/base_pose_flat (>=9 doubles; forbidden for V17 MOTION)
//      [0]=schema(1)  [1]=valid(0/1)  [2..4]=pos(x,y,z)
//      [5..8]=quat(w,x,y,z)
//    BASE authoritative /a3/base_pose_flat (16 doubles)
//      [0]=schema(2) [1]=valid [2]=sequence [3]=source_sec [4]=source_nsec
//      [5..7]=base position xyz [8..11]=base quaternion wxyz
//      [12]=tracking quality [13]=flags
//      [14]=marker/base calibration receipt id_u52 [15]=world-frame receipt id_u52
//      flags: bit0 tracking-valid, bit1 quaternion-valid, bit2 extrinsic-calibrated,
//      bit3 source stamp is HDU ROS receipt, bit4 policy z offset applied,
//      bit5 world/table frame calibrated.
//    SERVE BALL /serve/ball_state_flat (>=11 doubles)
//      [0]=schema(1)  [1]=valid(0/1)  [2..4]=pos_w(x,y,z)
//      [5..7]=vel_w(x,y,z)  [8]=source_stamp_s
//      [9]=frame_code(0=world/table, 1=base_link)
//      [10]=position-fit sample count
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <vector>

#include "a3_deploy/numeric_safety.hpp"
#include "a3_pingpong/pp_frame_math.hpp"

namespace a3_pingpong {

inline bool PpAllFinite(const Eigen::VectorXd& values) noexcept {
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (!a3_deploy::numeric_safety::IsFinite(values[i])) return false;
  }
  return true;
}

inline double PpNowWallSec() {
  struct timespec ts {};
  ::clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Freshness is a local elapsed-time property.  CLOCK_REALTIME can jump when
// phc2sys/chrony corrects the robot clock and it is not comparable across the
// HDU/MDU boundary.  Keep wall stamps for audit/shot identity, but age every
// locally received packet with CLOCK_MONOTONIC.
inline double PpNowSteadySec() {
  struct timespec ts {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// ------------------------------- racket target ------------------------------
struct PpRacketMsg {
  bool valid = false;
  double swing_sign = 0.0;     // +1 forehand / -1 backhand; 0 = unspecified/runner heuristic
  Vec3 pos_w = Vec3::Zero();   // racket intercept point (frame per frame_code)
  Vec3 vel_w = Vec3::Zero();   // desired racket velocity at strike
  double time_to_strike = 0.0; // producer-side TTS carried on the wire
  double strike_time = 0.0;    // absolute Unix strike deadline for schema 2
  double deadline_steady_s = 0.0; // receipt-time conversion into MDU monotonic
  int frame_code = 0;          // 0 = world/table, 1 = base_link
  int schema = 1;
  double producer_wall_s = 0.0;
  std::uint64_t command_seq = 0;
  std::uint64_t flight_id = 0;
  std::uint64_t revision_id = 0;
  int estimator_samples = 0;
  double estimator_span_s = 0.0;
  int stable_revision_count = 0;
  // local MDU wall receipt minus HDU producer wall time.  Audit only: it must
  // never decide command freshness or Planner release.
  double producer_clock_delta_s = 0.0;
};

// A latest-value mailbox with per-validity timestamps. The engage logic needs
// both "age of the newest VALID command" and "was an invalid seen AFTER it"
// (the planner_invalid_grace_s flutter tolerance from the Python runner).
class PpRacketTargetInput {
 public:
  // Fed from the AimRT subscriber thread. `a` is the decoded Float64MultiArray.
  void SetFromFlat(const std::vector<double>& a) {
    const double now_wall = PpNowWallSec();
    const double now_steady = PpNowSteadySec();
    if (a.size() < 2 ||
        !a3_deploy::numeric_safety::IsFinite(a[0]) ||
        !a3_deploy::numeric_safety::IsFinite(a[1])) {
      Reject_(now_wall, "size<2 or non-finite schema/valid");
      return;
    }
    if (a[1] != 0.0 && a[1] != 1.0) {
      Reject_(now_wall, "valid must be 0 or 1");
      return;
    }
    if (a[0] == 1.0) {
      SetSchema1_(a, now_wall, now_steady);
      return;
    }
    if (a[0] == 2.0) {
      SetSchema2_(a, now_wall, now_steady);
      return;
    }
    Reject_(now_wall, "unsupported schema");
  }

  struct Snapshot {
    bool has_valid = false;      // a valid command was ever received
    PpRacketMsg cmd;             // the newest VALID command (invalids never overwrite)
    double valid_age_s = 1e9;    // local monotonic receipt age (control clock)
    double producer_age_s = 1e9; // cross-host wall age (audit only)
    double control_time_to_strike_s = -1e9; // monotonic absolute-deadline countdown
    bool invalid_after = false;  // an invalid arrived AFTER the newest valid
    std::uint64_t seq = 0;       // accepted packet or schema-2 command sequence
  };

  Snapshot Latest() const {
    Snapshot s;
    const double now_wall = PpNowWallSec();
    const double now_steady = PpNowSteadySec();
    std::lock_guard<std::mutex> lk(mu_);
    s.seq = seq_;
    if (last_valid_receipt_steady_ < 0.0) return s;  // no valid yet
    s.has_valid = true;
    s.cmd = last_valid_;
    s.valid_age_s = now_steady - last_valid_receipt_steady_;
    s.producer_age_s = last_valid_.schema == 2
        ? now_wall - last_valid_.producer_wall_s : s.valid_age_s;
    s.control_time_to_strike_s =
        last_valid_.schema == 2 && last_valid_.deadline_steady_s > 0.0
        ? last_valid_.deadline_steady_s - now_steady
        : last_valid_.time_to_strike - s.valid_age_s;
    s.invalid_after = last_invalid_event_seq_ > last_valid_event_seq_;
    return s;
  }

  bool has_any() const { return any_.load(); }

 private:
  static bool ExactU52_(double value, std::uint64_t& out,
                        bool allow_zero = true) {
    constexpr double kMaxExact = 4503599627370496.0;  // 2^52
    if (!a3_deploy::numeric_safety::IsFinite(value) || value < 0.0 ||
        value > kMaxExact || std::floor(value) != value ||
        (!allow_zero && value == 0.0))
      return false;
    out = static_cast<std::uint64_t>(value);
    return true;
  }

  void SetSchema1_(const std::vector<double>& a, double now_wall,
                   double now_steady) {
    if (a.size() < 11) {
      Reject_(now_wall, "schema1 size<11");
      return;
    }
    for (std::size_t i = 2; i < 11; ++i) {
      if (!a3_deploy::numeric_safety::IsFinite(a[i])) {
        Reject_(now_wall, "schema1 non-finite required field");
        return;
      }
    }
    int frame_code = 0;
    if (a.size() >= 12) {
      // Keep the explicit bitwise finite predicate: comparisons against NaN
      // are not a finite-value guard under finite-math-only compilation.
      if (!a3_deploy::numeric_safety::IsFinite(a[11]) ||
          (a[11] != 0.0 && a[11] != 1.0)) {
        Reject_(now_wall, "frame_code must be 0 or 1");
        return;
      }
      frame_code = static_cast<int>(a[11]);
    }
    PpRacketMsg m;
    m.valid = a[1] == 1.0;
    m.swing_sign = std::fabs(a[2]) < 0.5 ? 0.0 : (a[2] > 0.0 ? 1.0 : -1.0);
    m.pos_w = Vec3(a[3], a[4], a[5]);
    m.vel_w = Vec3(a[6], a[7], a[8]);
    m.time_to_strike = a[9];
    m.strike_time = a[10];
    m.frame_code = frame_code;
    m.schema = 1;
    std::lock_guard<std::mutex> lk(mu_);
    ++event_seq_;
    ++seq_;
    if (m.valid) {
      last_valid_ = m;
      last_valid_wall_ = now_wall;
      last_valid_receipt_steady_ = now_steady;
      stable_revision_count_ = 0;
      last_valid_event_seq_ = event_seq_;
    } else {
      last_invalid_wall_ = now_wall;
      stable_revision_count_ = 0;
      last_invalid_event_seq_ = event_seq_;
    }
    any_ = true;
  }

  void SetSchema2_(const std::vector<double>& a, double now_wall,
                   double now_steady) {
    if (a.size() != 19) {
      Reject_(now_wall, "schema2 size!=19");
      return;
    }
    for (std::size_t i = 2; i < 19; ++i) {
      if (!a3_deploy::numeric_safety::IsFinite(a[i])) {
        Reject_(now_wall, "schema2 non-finite required field");
        return;
      }
    }
    std::uint64_t producer_sec = 0;
    std::uint64_t producer_nsec = 0;
    std::uint64_t command_seq = 0;
    std::uint64_t flight_id = 0;
    std::uint64_t revision_id = 0;
    std::uint64_t estimator_samples = 0;
    const bool wire_valid = a[1] == 1.0;
    if (!ExactU52_(a[12], producer_sec, false) ||
        !ExactU52_(a[13], producer_nsec) ||
        producer_nsec >= 1000000000ULL ||
        !ExactU52_(a[14], command_seq, false) ||
        !ExactU52_(a[15], flight_id, !wire_valid) ||
        !ExactU52_(a[16], revision_id, !wire_valid) ||
        !ExactU52_(a[17], estimator_samples) || estimator_samples > 10000ULL) {
      Reject_(now_wall, "schema2 integer/timestamp field invalid");
      return;
    }
    if (a[11] != 0.0 && a[11] != 1.0) {
      Reject_(now_wall, "schema2 frame_code must be 0 or 1");
      return;
    }
    if (wire_valid && a[2] != 1.0 && a[2] != -1.0) {
      Reject_(now_wall, "schema2 swing_sign must be +1 or -1");
      return;
    }
    if ((wire_valid && (a[9] <= 0.0 || a[10] <= 0.0)) ||
        (!wire_valid && (a[9] < 0.0 || a[10] < 0.0)) || a[18] < 0.0) {
      Reject_(now_wall, "schema2 timing/sample span invalid");
      return;
    }
    const double producer_wall = static_cast<double>(producer_sec) +
        static_cast<double>(producer_nsec) * 1.0e-9;
    if (wire_valid &&
        std::fabs(a[10] - (producer_wall + a[9])) > 0.010) {
      Reject_(now_wall, "schema2 absolute strike time disagrees with producer+tts");
      return;
    }

    PpRacketMsg m;
    m.valid = wire_valid;
    m.swing_sign = std::fabs(a[2]) < 0.5 ? 0.0 : (a[2] > 0.0 ? 1.0 : -1.0);
    m.pos_w = Vec3(a[3], a[4], a[5]);
    m.vel_w = Vec3(a[6], a[7], a[8]);
    m.time_to_strike = a[9];
    m.strike_time = a[10];
    // Convert synchronized wall deadline into a local monotonic deadline once.
    // No clock-quality threshold is consulted here: skew remains log/audit
    // evidence, matching the operator-owned safety decision for this runner.
    m.deadline_steady_s = now_steady + (m.strike_time - now_wall);
    m.frame_code = static_cast<int>(a[11]);
    m.schema = 2;
    m.producer_wall_s = producer_wall;
    m.command_seq = command_seq;
    m.flight_id = flight_id;
    m.revision_id = revision_id;
    m.estimator_samples = static_cast<int>(estimator_samples);
    m.estimator_span_s = a[18];
    m.producer_clock_delta_s = now_wall - producer_wall;

    bool reordered = false;
    bool producer_restarted = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      ++event_seq_;
      producer_restarted = have_wire_seq_ && command_seq < last_wire_seq_ &&
          last_wire_receipt_steady_ >= 0.0 &&
          now_steady - last_wire_receipt_steady_ > 0.050;
      if (producer_restarted) {
        // A restarted HDU Planner starts its command/revision counters again.
        // A real transport reorder has no 50 ms publisher gap, so preserve
        // the last command for a reorder and open a new wire epoch for a
        // producer restart.  Neither case becomes a planner-invalid event.
        have_wire_seq_ = false;
        have_schema2_valid_ = false;
        stable_revision_count_ = 0;
      }
      if (!producer_restarted &&
          ((have_wire_seq_ && command_seq <= last_wire_seq_) ||
          (m.valid && have_schema2_valid_ &&
           flight_id == last_schema2_flight_id_ &&
           revision_id <= last_schema2_revision_id_))) {
        // Ignore a duplicate/reordered transport sample without poisoning the
        // retained latest valid command.  Its local receipt timestamp is not
        // refreshed, so ordinary command_timeout still expires it naturally.
        any_ = true;
        reordered = true;
      } else {
        have_wire_seq_ = true;
        last_wire_seq_ = command_seq;
        last_wire_receipt_steady_ = now_steady;
        seq_ = command_seq;
        if (m.valid) {
          const bool same_track = have_schema2_valid_ &&
              flight_id == last_schema2_flight_id_ &&
              m.swing_sign == last_schema2_valid_.swing_sign;
          const bool stable = same_track &&
              (m.pos_w - last_schema2_valid_.pos_w).norm() <= 0.030 + 1.0e-12 &&
              (m.vel_w - last_schema2_valid_.vel_w).norm() <= 0.250 + 1.0e-12 &&
              std::fabs(m.strike_time - last_schema2_valid_.strike_time) <=
                  0.030 + 1.0e-12;
          stable_revision_count_ = stable ? stable_revision_count_ + 1 : 1;
          m.stable_revision_count = stable_revision_count_;
          last_schema2_valid_ = m;
          last_schema2_flight_id_ = flight_id;
          last_schema2_revision_id_ = revision_id;
          have_schema2_valid_ = true;
          last_valid_ = m;
          last_valid_wall_ = now_wall;
          last_valid_receipt_steady_ = now_steady;
          last_valid_event_seq_ = event_seq_;
        } else {
          last_invalid_wall_ = now_wall;
          stable_revision_count_ = 0;
          have_schema2_valid_ = false;
          last_invalid_event_seq_ = event_seq_;
        }
        any_ = true;
      }
    }
    if (reordered)
      WarnIgnored_("schema2 sequence/revision duplicate or reordered");
    if (producer_restarted) WarnProducerRestart_(command_seq);
    if (std::fabs(m.producer_clock_delta_s) > 0.010)
      WarnClockSkew_(m.producer_clock_delta_s);
  }

  void WarnClockSkew_(double delta_s) {
    const std::uint64_t n =
        clock_skew_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1 || n % 250 == 0) {
      std::fprintf(
          stderr,
          "[pp clock audit] racket local_wall-producer_wall=%+.6f s; "
          "packet accepted; absolute deadline is mapped once to local "
          "CLOCK_MONOTONIC and skew remains audit-only "
          "(count=%llu)\n",
          delta_s, static_cast<unsigned long long>(n));
    }
  }

  void WarnIgnored_(const char* reason) {
    const std::uint64_t n =
        ignored_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1 || n % 50 == 0) {
      std::fprintf(stderr,
                   "[pp input audit] IGNORE racket flat: %s; retained command "
                   "age was not refreshed (count=%llu)\n",
                   reason, static_cast<unsigned long long>(n));
    }
  }

  void WarnProducerRestart_(std::uint64_t command_seq) {
    const std::uint64_t n =
        producer_restart_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    std::fprintf(stderr,
                 "[pp input audit] racket producer counter restart -> new wire "
                 "epoch at command_seq=%llu (count=%llu)\n",
                 static_cast<unsigned long long>(command_seq),
                 static_cast<unsigned long long>(n));
  }

  void WarnReject_(const char* reason) {
    const std::uint64_t n =
        reject_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1 || n % 50 == 0) {
      std::fprintf(stderr, "[pp input] REJECT racket flat: %s (count=%llu)\n",
                   reason, static_cast<unsigned long long>(n));
    }
  }

  void Reject_(double now, const char* reason) {
    {
      // A malformed packet is an explicit planner-invalid event. Record the
      // fail-closed state before potentially blocking on stderr; repeated bad
      // packets therefore prevent a new engage after the configured grace.
      std::lock_guard<std::mutex> lk(mu_);
      ++event_seq_;
      last_invalid_wall_ = now;
      stable_revision_count_ = 0;
      have_schema2_valid_ = false;
      last_invalid_event_seq_ = event_seq_;
      any_ = true;
    }
    WarnReject_(reason);
  }

  mutable std::mutex mu_;
  PpRacketMsg last_valid_{};
  double last_valid_wall_ = -1.0;
  double last_valid_receipt_steady_ = -1.0;
  double last_invalid_wall_ = -1.0;
  std::atomic<bool> any_{false};
  std::atomic<std::uint64_t> reject_count_{0};
  std::atomic<std::uint64_t> clock_skew_count_{0};
  std::atomic<std::uint64_t> ignored_count_{0};
  std::atomic<std::uint64_t> producer_restart_count_{0};
  std::uint64_t seq_ = 0;
  std::uint64_t last_wire_seq_ = 0;
  double last_wire_receipt_steady_ = -1.0;
  std::uint64_t last_schema2_flight_id_ = 0;
  std::uint64_t last_schema2_revision_id_ = 0;
  PpRacketMsg last_schema2_valid_{};
  int stable_revision_count_ = 0;
  bool have_wire_seq_ = false;
  bool have_schema2_valid_ = false;
  std::uint64_t event_seq_ = 0;
  std::uint64_t last_valid_event_seq_ = 0;
  std::uint64_t last_invalid_event_seq_ = 0;
};

// ---------------------------- pre-serve ball state -------------------------
struct PpBallSample {
  Vec3 pos_w = Vec3::Zero();
  Vec3 vel_w = Vec3::Zero();
  double source_stamp_s = 0.0;
  int frame_code = 0;
  double age_s = 1e9;
  double receipt_wall_s = -1.0;
  double receipt_steady_s = -1.0;
  std::uint64_t seq = 0;
  int estimator_samples = 0;
};

// Position-only fitting stays in the planner process.  This mailbox carries
// the resulting finite position/velocity estimate to the 50 Hz controller and
// age-gates it by LOCAL receipt time, avoiding any assumption that clocks on
// the planner and robot are synchronized.
class PpBallStateInput {
 public:
  void SetFromFlat(const std::vector<double>& a) {
    const double now_wall = PpNowWallSec();
    const double now_steady = PpNowSteadySec();
    if (a.size() < 11) {
      Reject_(now_wall, "size<11");
      return;
    }
    for (std::size_t i = 0; i < 11; ++i) {
      if (!a3_deploy::numeric_safety::IsFinite(a[i])) {
        Reject_(now_wall, "non-finite required field");
        return;
      }
    }
    if (a[0] != 1.0) {
      Reject_(now_wall, "unsupported schema");
      return;
    }
    if (a[1] != 0.0 && a[1] != 1.0) {
      Reject_(now_wall, "valid must be 0 or 1");
      return;
    }
    if (a[9] != 0.0 && a[9] != 1.0) {
      Reject_(now_wall, "frame_code must be 0 or 1");
      return;
    }
    if (a[10] < 0.0 || a[10] > 1000.0 ||
        std::floor(a[10]) != a[10]) {
      Reject_(now_wall, "estimator sample count must be an integer in [0,1000]");
      return;
    }

    std::lock_guard<std::mutex> lk(mu_);
    ++seq_;
    any_ = true;
    if (a[1] == 0.0) {
      valid_ = false;
      receipt_wall_s_ = now_wall;
      receipt_steady_s_ = now_steady;
      return;
    }
    pos_w_ = Vec3(a[2], a[3], a[4]);
    vel_w_ = Vec3(a[5], a[6], a[7]);
    source_stamp_s_ = a[8];
    frame_code_ = static_cast<int>(a[9]);
    estimator_samples_ = static_cast<int>(a[10]);
    receipt_wall_s_ = now_wall;
    receipt_steady_s_ = now_steady;
    valid_ = true;
  }

  bool Latest(PpBallSample& out, double max_age_s) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (!valid_ || receipt_steady_s_ < 0.0 ||
        !a3_deploy::numeric_safety::IsFinite(max_age_s) ||
        max_age_s <= 0.0) {
      return false;
    }
    out.pos_w = pos_w_;
    out.vel_w = vel_w_;
    out.source_stamp_s = source_stamp_s_;
    out.frame_code = frame_code_;
    out.receipt_wall_s = receipt_wall_s_;
    out.receipt_steady_s = receipt_steady_s_;
    out.seq = seq_;
    out.estimator_samples = estimator_samples_;
    out.age_s = PpNowSteadySec() - receipt_steady_s_;
    return a3_deploy::numeric_safety::IsFinite(out.age_s) &&
           out.age_s >= 0.0 && out.age_s <= max_age_s;
  }

  bool has_any() const { return any_.load(); }

 private:
  void Reject_(double now, const char* reason) {
    const std::uint64_t n =
        reject_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    {
      std::lock_guard<std::mutex> lk(mu_);
      valid_ = false;
      receipt_wall_s_ = now;
      any_ = true;
    }
    if (n == 1 || n % 50 == 0) {
      std::fprintf(stderr,
                   "[pp input] REJECT serve-ball flat: %s (count=%llu)\n",
                   reason, static_cast<unsigned long long>(n));
    }
  }

  mutable std::mutex mu_;
  Vec3 pos_w_ = Vec3::Zero();
  Vec3 vel_w_ = Vec3::Zero();
  double source_stamp_s_ = 0.0;
  int frame_code_ = 0;
  int estimator_samples_ = 0;
  double receipt_wall_s_ = -1.0;
  double receipt_steady_s_ = -1.0;
  bool valid_ = false;
  std::uint64_t seq_ = 0;
  std::atomic<bool> any_{false};
  std::atomic<std::uint64_t> reject_count_{0};
};

// -------------------------------- base pose ---------------------------------
struct PpBaseSample {
  Vec3 pos = Vec3::Zero();
  Vec4 quat = Vec4(1, 0, 0, 0);  // w,x,y,z
  double age_s = 1e9;
  double receipt_age_s = 1e9;
  double source_age_s = 1e9;
  double receipt_wall_s = -1.0;
  double receipt_steady_s = -1.0;
  // Backward-compatible name used by the isolated deterministic-serve
  // velocity gate. It is always identical to receipt_wall_s.
  double stamp_wall_s = -1.0;
  double source_wall_s = -1.0;
  // MDU local wall receipt minus the HDU relay stamp.  This is diagnostic
  // clock/transport evidence only and is not a freshness input.
  double source_clock_delta_s = 0.0;
  std::uint64_t seq = 0;
  double tracking_quality = 0.0;
  std::uint64_t flags = 0;
  std::uint64_t calibration_id = 0;
  std::uint64_t world_frame_id = 0;
  int schema = 0;
  bool authoritative = false;
};

inline constexpr std::uint64_t kBaseFlagTrackingValid = 1ULL << 0;
inline constexpr std::uint64_t kBaseFlagQuaternionValid = 1ULL << 1;
inline constexpr std::uint64_t kBaseFlagExtrinsicCalibrated = 1ULL << 2;
inline constexpr std::uint64_t kBaseFlagSourceStampHduRos = 1ULL << 3;
inline constexpr std::uint64_t kBaseFlagPolicyZOffsetApplied = 1ULL << 4;
inline constexpr std::uint64_t kBaseFlagWorldFrameCalibrated = 1ULL << 5;
inline constexpr std::uint64_t kV17RequiredBaseFlags =
    kBaseFlagTrackingValid | kBaseFlagQuaternionValid |
    kBaseFlagExtrinsicCalibrated | kBaseFlagSourceStampHduRos |
    kBaseFlagWorldFrameCalibrated;

// Mocap/localizer base pose in the SAME world frame as the racket target.
// Schema 1 remains readable by retired tasks and the isolated serve harness.
// V17 calls Latest(..., require_authoritative=true), which requires schema 2,
// full-pose validity and both calibration receipts. Freshness is the local MDU
// monotonic receipt age; the HDU source wall time remains audit evidence only.
class PpBasePoseInput {
 public:
  void SetFromFlat(const std::vector<double>& a) {
    const double now_wall = PpNowWallSec();
    const double now_steady = PpNowSteadySec();
    if (a.size() < 2) {
      Reject_(now_wall, "size<2");
      return;
    }
    if (!a3_deploy::numeric_safety::IsFinite(a[0]) ||
        !a3_deploy::numeric_safety::IsFinite(a[1])) {
      Reject_(now_wall, "non-finite schema/valid");
      return;
    }
    if (a[1] != 0.0 && a[1] != 1.0) {
      Reject_(now_wall, "valid must be 0 or 1");
      return;
    }
    if (a[1] == 0.0) {
      // Explicit valid=0 immediately invalidates the mailbox. Dropout without
      // a packet instead expires through the receipt/source age gates.
      std::lock_guard<std::mutex> lk(mu_);
      valid_ = false;
      any_ = true;
      return;
    }

    if (a[0] == 1.0) {
      SetSchema1_(a, now_wall, now_steady);
      return;
    }
    if (a[0] == 2.0) {
      SetSchema2_(a, now_wall, now_steady);
      return;
    }
    Reject_(now_wall, "unsupported schema");
  }

  bool Latest(PpBaseSample& out, double max_age_s,
              bool require_authoritative = false) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (!valid_ || receipt_steady_ < 0.0 || max_age_s < 0.0) return false;
    if (require_authoritative && !authoritative_) return false;
    const double receipt_age = PpNowSteadySec() - receipt_steady_;
    const double source_age =
        schema_ == 2 ? PpNowWallSec() - source_wall_ : receipt_age;
    if (!a3_deploy::numeric_safety::IsFinite(receipt_age) ||
        receipt_age < 0.0 || receipt_age > max_age_s) {
      return false;
    }
    out.pos = pos_;
    out.quat = quat_;
    out.age_s = receipt_age;
    out.receipt_age_s = receipt_age;
    out.source_age_s = source_age;
    out.receipt_wall_s = receipt_wall_;
    out.receipt_steady_s = receipt_steady_;
    out.stamp_wall_s = receipt_wall_;
    out.source_wall_s = source_wall_;
    out.source_clock_delta_s = source_clock_delta_s_;
    out.seq = seq_;
    out.tracking_quality = tracking_quality_;
    out.flags = flags_;
    out.calibration_id = calibration_id_;
    out.world_frame_id = world_frame_id_;
    out.schema = schema_;
    out.authoritative = authoritative_;
    return true;
  }

  bool has_any() const { return any_.load(); }

 private:
  static bool ExactU52_(double value, std::uint64_t& out,
                        bool allow_zero = true) {
    constexpr double kMaxExact = 4503599627370496.0;  // 2^52
    if (!a3_deploy::numeric_safety::IsFinite(value) || value < 0.0 ||
        value > kMaxExact || std::floor(value) != value ||
        (!allow_zero && value == 0.0)) {
      return false;
    }
    out = static_cast<std::uint64_t>(value);
    return true;
  }

  void SetSchema1_(const std::vector<double>& a, double now_wall,
                   double now_steady) {
    if (a.size() < 9) {
      Reject_(now_wall, "schema1 size<9");
      return;
    }
    for (std::size_t i = 2; i < 9; ++i) {
      if (!a3_deploy::numeric_safety::IsFinite(a[i])) {
        Reject_(now_wall, "schema1 non-finite pose");
        return;
      }
    }
    Vec4 quat(a[5], a[6], a[7], a[8]);
    const double quat_norm = quat.norm();
    const bool quat_valid =
        a3_deploy::numeric_safety::IsFinite(quat_norm) &&
        quat_norm > 1.0e-9;
    {
      std::lock_guard<std::mutex> lk(mu_);
      pos_ = Vec3(a[2], a[3], a[4]);
      quat_ = quat_valid ? quat / quat_norm : Vec4(1, 0, 0, 0);
      receipt_wall_ = now_wall;
      receipt_steady_ = now_steady;
      source_wall_ = now_wall;
      source_clock_delta_s_ = 0.0;
      valid_ = true;
      authoritative_ = false;
      schema_ = 1;
      tracking_quality_ = 0.0;
      flags_ = 0;
      calibration_id_ = 0;
      world_frame_id_ = 0;
      ++seq_;
      any_ = true;
    }
    if (!quat_valid) WarnQuatFallback_();
  }

  void SetSchema2_(const std::vector<double>& a, double now_wall,
                   double now_steady) {
    if (a.size() < 16) {
      Reject_(now_wall, "schema2 size<16");
      return;
    }
    for (std::size_t i = 2; i < 16; ++i) {
      if (!a3_deploy::numeric_safety::IsFinite(a[i])) {
        Reject_(now_wall, "schema2 non-finite required field");
        return;
      }
    }
    std::uint64_t wire_seq = 0;
    std::uint64_t source_sec = 0;
    std::uint64_t source_nsec = 0;
    std::uint64_t flags = 0;
    std::uint64_t calibration_id = 0;
    std::uint64_t world_frame_id = 0;
    if (!ExactU52_(a[2], wire_seq) ||
        !ExactU52_(a[3], source_sec, false) ||
        !ExactU52_(a[4], source_nsec) || source_nsec >= 1000000000ULL ||
        !ExactU52_(a[13], flags) ||
        !ExactU52_(a[14], calibration_id, false) ||
        !ExactU52_(a[15], world_frame_id, false)) {
      Reject_(now_wall, "schema2 integer/timestamp/receipt field invalid");
      return;
    }
    if ((flags & (kBaseFlagTrackingValid | kBaseFlagQuaternionValid)) !=
        (kBaseFlagTrackingValid | kBaseFlagQuaternionValid)) {
      Reject_(now_wall, "schema2 tracking/quaternion validity flags missing");
      return;
    }
    if (a[12] <= 0.0 || a[12] > 1.0) {
      Reject_(now_wall, "schema2 tracking quality outside (0,1]");
      return;
    }
    Vec4 quat(a[8], a[9], a[10], a[11]);
    const double quat_norm = quat.norm();
    if (!a3_deploy::numeric_safety::IsFinite(quat_norm) ||
        quat_norm < 0.5 || quat_norm > 1.5) {
      Reject_(now_wall, "schema2 quaternion norm outside [0.5,1.5]");
      return;
    }
    const double source_wall =
        static_cast<double>(source_sec) +
        static_cast<double>(source_nsec) * 1.0e-9;
    if (!a3_deploy::numeric_safety::IsFinite(source_wall)) {
      Reject_(now_wall, "schema2 source wall time is non-finite");
      return;
    }
    const double source_clock_delta = now_wall - source_wall;
    const bool authoritative =
        (flags & kV17RequiredBaseFlags) == kV17RequiredBaseFlags;
    bool reordered = false;
    bool producer_restarted = false;
    bool source_clock_regressed = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      producer_restarted = have_schema2_seq_ && wire_seq < last_wire_seq_ &&
          receipt_steady_ >= 0.0 && now_steady - receipt_steady_ > 0.050;
      source_clock_regressed =
          have_source_wall_ && source_wall <= last_source_wall_;
      if (!producer_restarted && have_schema2_seq_ &&
          wire_seq <= last_wire_seq_) {
        // Do not invalidate a good pose because DDS delivered a duplicate or
        // an older sample.  Keeping the old monotonic receipt time means the
        // retained pose still expires normally if the stream really stops.
        any_ = true;
        reordered = true;
      } else {
        pos_ = Vec3(a[5], a[6], a[7]);
        quat_ = quat / quat_norm;
        if (have_schema2_quat_ && quat_.dot(last_schema2_quat_) < 0.0)
          quat_ = -quat_;
        last_schema2_quat_ = quat_;
        have_schema2_quat_ = true;
        receipt_wall_ = now_wall;
        receipt_steady_ = now_steady;
        source_wall_ = source_wall;
        source_clock_delta_s_ = source_clock_delta;
        valid_ = true;
        authoritative_ = authoritative;
        schema_ = 2;
        tracking_quality_ = a[12];
        flags_ = flags;
        calibration_id_ = calibration_id;
        world_frame_id_ = world_frame_id;
        seq_ = wire_seq;
        last_wire_seq_ = wire_seq;
        last_source_wall_ = source_wall;
        have_schema2_seq_ = true;
        have_source_wall_ = true;
        any_ = true;
      }
    }
    if (reordered) WarnIgnoredSequence_();
    if (producer_restarted) WarnProducerRestart_(wire_seq);
    if (source_clock_regressed) WarnSourceClockRegression_();
    if (std::fabs(source_clock_delta) > 0.010)
      WarnClockSkew_(source_clock_delta);
  }

  void WarnClockSkew_(double delta_s) {
    const std::uint64_t n =
        clock_skew_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1 || n % 250 == 0) {
      std::fprintf(
          stderr,
          "[pp clock audit] base local_wall-source_wall=%+.6f s; "
          "packet accepted, freshness uses local CLOCK_MONOTONIC receipt age "
          "(count=%llu)\n",
          delta_s, static_cast<unsigned long long>(n));
    }
  }

  void WarnIgnoredSequence_() {
    const std::uint64_t n =
        ignored_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1 || n % 50 == 0) {
      std::fprintf(stderr,
                   "[pp input audit] IGNORE base flat duplicate/reordered "
                   "sequence; retained pose age was not refreshed (count=%llu)\n",
                   static_cast<unsigned long long>(n));
    }
  }

  void WarnProducerRestart_(std::uint64_t wire_seq) {
    const std::uint64_t n =
        producer_restart_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    std::fprintf(stderr,
                 "[pp input audit] base relay counter restart -> new wire epoch "
                 "at sequence=%llu (count=%llu)\n",
                 static_cast<unsigned long long>(wire_seq),
                 static_cast<unsigned long long>(n));
  }

  void WarnSourceClockRegression_() {
    const std::uint64_t n =
        source_clock_regression_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1 || n % 250 == 0) {
      std::fprintf(stderr,
                   "[pp clock audit] base source wall time did not increase; "
                   "packet control remains on sequence + local CLOCK_MONOTONIC "
                   "receipt age (count=%llu)\n",
                   static_cast<unsigned long long>(n));
    }
  }

  void Reject_(double now, const char* reason) {
    const std::uint64_t n = reject_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    {
      std::lock_guard<std::mutex> lk(mu_);
      valid_ = false;
      receipt_wall_ = now;
      any_ = true;
    }
    if (n == 1 || n % 50 == 0) {
      std::fprintf(stderr, "[pp input] REJECT base flat: %s (count=%llu)\n", reason,
                   static_cast<unsigned long long>(n));
    }
  }

  void WarnQuatFallback_() {
    const std::uint64_t n =
        quat_fallback_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1 || n % 250 == 0) {
      std::fprintf(stderr,
                   "[pp input] WARN base quaternion invalid; accepted position with "
                   "identity quaternion (count=%llu)\n",
                   static_cast<unsigned long long>(n));
    }
  }

  mutable std::mutex mu_;
  Vec3 pos_ = Vec3::Zero();
  Vec4 quat_ = Vec4(1, 0, 0, 0);
  Vec4 last_schema2_quat_ = Vec4(1, 0, 0, 0);
  double receipt_wall_ = -1.0;
  double receipt_steady_ = -1.0;
  double source_wall_ = -1.0;
  double source_clock_delta_s_ = 0.0;
  double last_source_wall_ = -1.0;
  double tracking_quality_ = 0.0;
  std::uint64_t flags_ = 0;
  std::uint64_t calibration_id_ = 0;
  std::uint64_t world_frame_id_ = 0;
  std::uint64_t last_wire_seq_ = 0;
  int schema_ = 0;
  bool valid_ = false;
  bool authoritative_ = false;
  bool have_schema2_seq_ = false;
  bool have_source_wall_ = false;
  bool have_schema2_quat_ = false;
  std::uint64_t seq_ = 0;
  std::atomic<bool> any_{false};
  std::atomic<std::uint64_t> reject_count_{0};
  std::atomic<std::uint64_t> quat_fallback_count_{0};
  std::atomic<std::uint64_t> clock_skew_count_{0};
  std::atomic<std::uint64_t> ignored_count_{0};
  std::atomic<std::uint64_t> producer_restart_count_{0};
  std::atomic<std::uint64_t> source_clock_regression_count_{0};
};

}  // namespace a3_pingpong
