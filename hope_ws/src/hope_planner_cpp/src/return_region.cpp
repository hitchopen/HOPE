#include "hope_planner_cpp/return_region.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
namespace hope_planner_cpp {
const char *name(ContactSide s) {
  return s == ContactSide::kOpponent   ? "opponent"
         : s == ContactSide::kReceiver ? "receiver"
                                       : "ambiguous";
}
const char *name(HistoryContinuity s) {
  return s == HistoryContinuity::kComplete     ? "complete"
         : s == HistoryContinuity::kIncomplete ? "incomplete"
                                               : "unknown";
}
const char *name(ContactProvenance s) {
  switch (s) {
  case ContactProvenance::kObservedConfirmed:
    return "observed_confirmed";
  case ContactProvenance::kExternalSideVerified:
    return "external_side_verified";
  case ContactProvenance::kInferredUnconfirmed:
    return "inferred_unconfirmed";
  default:
    return "unknown";
  }
}
void ReturnRegionPolicy::validate() const {
  for (double x : {post_hit_distance_m, side_tolerance_m, stall_speed_mps,
                   stall_duration_s, tie_tolerance_s})
    if (!std::isfinite(x) || x <= 0)
      throw std::invalid_argument("invalid return-region policy");
  if (post_hit_distance_m > 2. || !std::isfinite(ground_z_m) ||
      !std::isfinite(publication_period_s) || publication_period_s < 0. ||
      !std::isfinite(max_source_age_s) || max_source_age_s <= 0.)
    throw std::invalid_argument("invalid return-region bounds");
}
ContactSide contact_side(double x, const ProbabilisticConfig &c,
                         const ReturnRegionPolicy &p) {
  return x > c.table.net_x + p.side_tolerance_m   ? ContactSide::kOpponent
         : x < c.table.net_x - p.side_tolerance_m ? ContactSide::kReceiver
                                                  : ContactSide::kAmbiguous;
}
// Standard SHA-256; no runtime subprocess, platform hash or crypto dependency.
std::string sha256(const std::string &input) {
  constexpr std::uint32_t k[]{
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
      0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
      0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
      0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
      0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
      0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
      0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
      0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
      0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
      0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  auto r = [](std::uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
  std::vector<unsigned char> bytes(input.begin(), input.end());
  const std::uint64_t bits = bytes.size() * 8;
  bytes.push_back(0x80);
  while (bytes.size() % 64 != 56)
    bytes.push_back(0);
  for (int j = 7; j >= 0; --j)
    bytes.push_back(static_cast<unsigned char>(bits >> (j * 8)));
  std::array<std::uint32_t, 8> hash{0x6a09e667, 0xbb67ae85, 0x3c6ef372,
                                    0xa54ff53a, 0x510e527f, 0x9b05688c,
                                    0x1f83d9ab, 0x5be0cd19};
  for (std::size_t off = 0; off < bytes.size(); off += 64) {
    std::uint32_t w[64];
    for (int j = 0; j < 16; ++j)
      w[j] = (std::uint32_t(bytes[off + j * 4]) << 24) |
             (std::uint32_t(bytes[off + j * 4 + 1]) << 16) |
             (std::uint32_t(bytes[off + j * 4 + 2]) << 8) |
             bytes[off + j * 4 + 3];
    for (int j = 16; j < 64; ++j)
      w[j] = w[j - 16] +
             (r(w[j - 15], 7) ^ r(w[j - 15], 18) ^ (w[j - 15] >> 3)) +
             w[j - 7] + (r(w[j - 2], 17) ^ r(w[j - 2], 19) ^ (w[j - 2] >> 10));
    auto a = hash[0], b = hash[1], c = hash[2], d = hash[3], e = hash[4],
         f = hash[5], g = hash[6], h = hash[7];
    for (int j = 0; j < 64; ++j) {
      auto t1 = h + (r(e, 6) ^ r(e, 11) ^ r(e, 25)) + ((e & f) ^ (~e & g)) +
                k[j] + w[j];
      auto t2 = (r(a, 2) ^ r(a, 13) ^ r(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    hash[0] += a;
    hash[1] += b;
    hash[2] += c;
    hash[3] += d;
    hash[4] += e;
    hash[5] += f;
    hash[6] += g;
    hash[7] += h;
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (auto x : hash)
    out << std::setw(8) << x;
  return out.str();
}
std::string return_region_config_identity(const ProbabilisticConfig &c,
                                          const ReturnRegionPolicy &p,
                                          const std::string &frame) {
  std::ostringstream o;
  o.imbue(std::locale::classic());
  // Versioned typed fixed-order canonical serialization: ASCII labels; floats
  // are C++ classic-locale hexfloat; frame is length-prefixed raw UTF-8.
  o << "return-region-v1;time=int64-ns/local-origin;units=m,m/s,s;frame="
    << frame.size() << ':' << frame << ';' << std::hexfloat;
  auto v = [&](const char *key, double x) { o << key << "=f64:" << x << ';'; };
  v("drag", c.physics.drag_k);
  v("restitution_h", c.physics.restitution_h);
  v("restitution_v", c.physics.restitution_v);
  v("grip", c.physics.table_tangential_gain);
  v("friction", c.physics.table_friction_cap_mu);
  v("radius", c.physics.radius);
  for (int i = 0; i < 3; ++i) {
    v(("gravity" + std::to_string(i)).c_str(), c.physics.gravity[i]);
    v(("measurement" + std::to_string(i)).c_str(), c.measurement_std_m[i]);
    v(("bias" + std::to_string(i)).c_str(), c.measurement_bias_std_m[i]);
    v(("velocity_prior_mean" + std::to_string(i)).c_str(),
      c.velocity_prior_mean[i]);
    v(("velocity_prior_std" + std::to_string(i)).c_str(),
      c.velocity_prior_std[i]);
    v(("bounce_std" + std::to_string(i)).c_str(), c.bounce_velocity_std[i]);
  }
  v("length", c.table.length);
  v("width", c.table.width);
  v("y_max", c.table.y_max);
  v("net_x", c.table.net_x);
  v("net_height", c.table.net_height);
  v("net_overhang", c.table.net_overhang);
  v("x_hit", c.x_hit);
  v("post_hit", p.post_hit_distance_m);
  v("integration_dt", c.integration_dt_s);
  v("output_dt", c.trajectory_dt_s);
  v("horizon", c.horizon_s);
  v("side_tolerance", p.side_tolerance_m);
  v("ground", p.ground_z_m);
  v("stall_speed", p.stall_speed_mps);
  v("stall_duration", p.stall_duration_s);
  v("tie", p.tie_tolerance_s);
  v("publication_period", p.publication_period_s);
  v("max_source_age", p.max_source_age_s);
  v("bias_tau", c.measurement_bias_tau_s);
  v("q", c.acceleration_spectral_density);
  v("max_gap", c.max_tracking_gap_s);
  v("gap_warning", c.gap_warning_s);
  v("clock_reset", c.clock_reset_backstep_s);
  v("innovation_gate", c.innovation_gate_squared);
  v("min_tracking_z", c.min_tracking_z_m);
  v("direction_speed", c.direction_speed_mps);
  v("direction_sigma", c.direction_sigma);
  v("direction_window", c.direction_window_s);
  o << "confirmations=u64:" << c.direction_confirmations
    << ";reacquire=u64:" << c.reacquire_rejections
    << ";history=side-aware-v1;tracker=conditional-cubature-cross-repair-v1;"
       "unknown=bounded-fail-closed;";
  return "return-region-v1:sha256:" + sha256(o.str());
}
std::string physics_fingerprint(const ProbabilisticConfig &c) {
  std::ostringstream o;
  o.imbue(std::locale::classic());
  o << "physical_step-v1:" << std::hexfloat;
  for (double v : {c.physics.gravity[0], c.physics.gravity[1], c.physics.gravity[2],
       c.physics.drag_k, c.physics.radius, c.physics.restitution_v,
       c.physics.table_tangential_gain, c.physics.table_friction_cap_mu,
       c.table.length, c.table.width, c.table.y_max, c.integration_dt_s,
       c.trajectory_dt_s, c.horizon_s, c.x_hit, c.acceleration_spectral_density,
       c.bounce_velocity_std[0], c.bounce_velocity_std[1], c.bounce_velocity_std[2]})
    o << v << ':';
  o << c.max_rollout_bounces << ':' << c.ensemble_size;
  return o.str();
}
} // namespace hope_planner_cpp
