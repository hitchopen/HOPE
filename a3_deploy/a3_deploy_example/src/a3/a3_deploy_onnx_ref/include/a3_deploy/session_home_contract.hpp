#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace a3_deploy {
// Kinematic waiting predicate only: no foot-force/contact sensor is inferred.
class PlanePostureGate {
 public:
  void reset() { since_ = last_ = -1.0; }
  bool update(double now, bool waiting, bool fresh_velocity, double speed,
              double angular_speed, double gravity_z) {
    const bool ok = std::isfinite(now) && waiting && fresh_velocity &&
        std::isfinite(speed) && speed >= 0.0 && speed <= .03 &&
        std::isfinite(angular_speed) && angular_speed >= 0.0 && angular_speed <= .20 &&
        std::isfinite(gravity_z) && gravity_z >= -1.000001 && gravity_z <= -std::cos(5.0 * 3.141592653589793 / 180.0);
    if (!ok || (last_ >= 0.0 && (now <= last_ || now-last_ > .05))) since_ = -1.0;
    if (ok && since_ < 0.0) since_ = now;
    last_ = now;
    return ok && now-since_ >= .20;
  }
 private:
  double since_ = -1.0, last_ = -1.0;
};


// Runner owns this immutable playing-session receipt. Positions use the same
// calibrated world as /a3/base_pose_flat; plane_x comes from the loaded ONNX.
// Wire: [version=1, epoch_u52, home_x, home_y, plane_x, calibration_id, world_id].
// Schema-4 racket commands are schema-2's 19 fields plus this epoch at [19].
struct SessionHomeReceipt {
  std::uint64_t epoch = 0;
  double x = 0.0, y = 0.0, plane_x = 0.0;
  std::uint64_t calibration_id = 0, world_id = 0;
  bool runtime_planes = false; // v2 advertises schema-5 per-flight references

  static bool integer(double value, std::uint64_t& result) {
    if (!std::isfinite(value) || value <= 0.0 || value > 4503599627370496.0 ||
        std::floor(value) != value) return false;
    result = static_cast<std::uint64_t>(value);
    return true;
  }
  static bool parse(const std::vector<double>& a, SessionHomeReceipt& out) {
    const bool v2 = a.size() == 10 && a[0] == 2.0;
    if (!v2 && (a.size() != 7 || a[0] != 1.0)) return false;
    std::uint64_t sequence = 0;
    if (v2 && (a[7] != 5.0 || !integer(a[8], sequence) ||
               (a[9] != 0.0 && a[9] != 1.0))) return false;
    SessionHomeReceipt next;
    if (!integer(a[1], next.epoch) || !integer(a[5], next.calibration_id) ||
        !integer(a[6], next.world_id) || !std::isfinite(a[2]) ||
        !std::isfinite(a[3]) || !std::isfinite(a[4]) || a[4] <= 0.0)
      return false;
    next.x = a[2]; next.y = a[3]; next.plane_x = a[4];
    next.runtime_planes = v2;
    out = next;
    return true;
  }
  std::vector<double> pack(std::uint64_t sequence = 1, bool posture_stable = false) const {
    std::vector<double> wire{runtime_planes ? 2.0 : 1.0, static_cast<double>(epoch), x, y, plane_x,
            static_cast<double>(calibration_id), static_cast<double>(world_id)};
    // v2 tail: supported wire schema, monotonically increasing heartbeat, posture gate.
    // The gate is kinematic, not a foot-support/contact measurement.
    if (runtime_planes) wire.insert(wire.end(), {5.0, static_cast<double>(sequence), posture_stable ? 1.0 : 0.0});
    return wire;
  }
  bool same_geometry(const SessionHomeReceipt& other) const {
    return x == other.x && y == other.y && plane_x == other.plane_x &&
        calibration_id == other.calibration_id && world_id == other.world_id &&
        runtime_planes == other.runtime_planes;
  }
  bool matches_command(int schema, std::uint64_t command_epoch,
                       int frame, double target_x, double reference_x = 0.0, int mode = 0) const {
    if (epoch == 0 || command_epoch != epoch || frame != 0 || !std::isfinite(target_x)) return false;
    if (schema == 4) return std::abs(target_x - (x + plane_x)) < 1e-6;
    return schema == 5 && runtime_planes && (mode == 1 || mode == 2) &&
        std::isfinite(reference_x) && std::abs(target_x - (reference_x + plane_x)) < 1e-6;
  }
};

// Caller serializes access. Reordered heartbeats cannot revive an old HOME;
// changing a receipt without changing its epoch is invalid, not a recapture.
struct SessionHomeMailbox {
  SessionHomeReceipt value;
  std::int64_t receipt_ns = 0;
  std::uint64_t sequence = 0;
  bool posture_stable = false;
  bool accept(const std::vector<double>& wire, std::int64_t now_ns) {
    SessionHomeReceipt next;
    if (!SessionHomeReceipt::parse(wire, next) || next.epoch < value.epoch ||
        (next.epoch == value.epoch && !next.same_geometry(value))) return false;
    const auto next_sequence = next.runtime_planes ? static_cast<std::uint64_t>(wire[8]) : 0;
    if (next.runtime_planes && next.epoch == value.epoch && next_sequence <= sequence) return false;
    sequence = next_sequence;
    posture_stable = next.runtime_planes && wire[9] == 1.0;
    value = next;
    receipt_ns = now_ns;
    return true;
  }
  bool fresh(std::int64_t now_ns, std::int64_t max_age_ns,
             std::uint64_t calibration_id, std::uint64_t world_id) const {
    return value.epoch != 0 && receipt_ns > 0 && now_ns >= receipt_ns &&
        now_ns - receipt_ns <= max_age_ns &&
        value.calibration_id == calibration_id && value.world_id == world_id;
  }
};
}  // namespace a3_deploy
