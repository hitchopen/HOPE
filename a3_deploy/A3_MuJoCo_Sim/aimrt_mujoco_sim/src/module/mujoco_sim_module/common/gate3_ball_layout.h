// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstddef>
#include <cmath>
#include <string>
#include <algorithm>
#include <stdexcept>
#include "mujoco/mujoco.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball {

inline constexpr char kBodyName[] = "gate3_ball";
inline constexpr char kJointName[] = "gate3_ball_free_joint";
inline constexpr char kGeomName[] = "gate3_ball_collision";
inline constexpr char kRacketGeomName[] = "right_racket_collision";
inline constexpr char kTableGeomName[] = "gate3_table_collision";
inline constexpr char kNetGeomName[] = "gate3_net_collision";

inline constexpr double kTableSurfaceZ = 0.760;
inline constexpr double kBallRadius = 0.020;

inline constexpr std::size_t kShotId = 0;
inline constexpr std::size_t kActive = 1;
inline constexpr std::size_t kContactBits = 2;
inline constexpr std::size_t kRacketContactCount = 3;
inline constexpr std::size_t kTableContactCount = 4;
inline constexpr std::size_t kNetContactCount = 5;
inline constexpr std::size_t kRacketNormalForce = 6;
inline constexpr std::size_t kRequiredUserData = 7;

inline constexpr unsigned char kContactRacket = 1;
inline constexpr unsigned char kContactTable = 2;
inline constexpr unsigned char kContactNet = 4;


// Slot 0 is current incoming ball; the others retain completed launcher windows.
inline constexpr int kFlightSlots = 9;
// A ball can finish on the table. Bound its motion for a continuous dwell;
// never use elapsed flight time to retire an airborne ball.
struct TableRestTracker {
  double shot = -1, since = -1, last_time = -1;
  bool Update(double id, double time, const mjtNum* p, const mjtNum* v,
              double table_contacts) {
    if (id != shot || time < last_time) since = -1;
    shot = id;
    last_time = time;
    const bool rest = table_contacts > 0 && p[0] >= 0 &&
        p[0] <= 2.74 && p[1] >= -1.525 && p[1] <= 0 &&
        std::abs(p[2]-(kTableSurfaceZ+kBallRadius)) <= .001 &&
        std::hypot(v[0],v[1]) <= .002 && std::abs(v[2]) <= .12;
    if (!rest) { since = -1; return false; }
    if (since < 0) since = time;
    return time-since >= 2.0;
  }
};
inline std::string SlotName(const char* name, int slot) {
  return slot == 0 ? std::string(name) : std::string(name) + "_return" + std::to_string(slot);
}
inline int SlotJoint(const mjModel* m, int slot) {
  return mj_name2id(m, mjOBJ_JOINT, SlotName(kJointName, slot).c_str());
}
inline bool IsFlightGeom(const mjModel* m, int geom) {
  const char* raw = mj_id2name(m, mjOBJ_GEOM, geom);
  if (!raw) return false;
  const std::string name(raw);
  return name == kGeomName || name.rfind(std::string(kGeomName) + "_return", 0) == 0;
}
inline void PreserveFlight(const mjModel* m, mjData* d, int q, int v) {
  if (d->userdata[kActive] <= 0.5) return;
  for (int slot = 1; slot < kFlightSlots; ++slot) {
    auto* telemetry = d->userdata + slot * kRequiredUserData;
    // 0 is a terminal sample still awaiting publication; -1 acknowledges it.
    // Never reuse a flight before its final physical sample has been queued.
    if (telemetry[kShotId] > 0.0 && telemetry[kActive] >= -0.5) continue;
    const int joint = SlotJoint(m, slot);
    if (joint < 0) throw std::runtime_error("continuous Gate3 scene missing flight slot");
    std::copy_n(d->qpos + q, 7, d->qpos + m->jnt_qposadr[joint]);
    std::copy_n(d->qvel + v, 6, d->qvel + m->jnt_dofadr[joint]);
    std::copy_n(d->userdata, kRequiredUserData, telemetry);
    return;
  }
  throw std::runtime_error("Gate3 flight pool exhausted; refusing to truncate an airborne ball");
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball
