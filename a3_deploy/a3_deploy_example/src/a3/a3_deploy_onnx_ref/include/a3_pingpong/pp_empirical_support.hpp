#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace a3_pingpong {
// A measured deployment admission domain, separate from the immutable training
// metadata and the actor's station input. It never bypasses localization or
// physical fault handling. Planner must use the same two lateral intervals.
struct EmpiricalLateralSupport {
  bool enabled = false;
  std::array<double, 2> forehand{}, backhand{};
  std::string evidence_sha256;
};
inline EmpiricalLateralSupport ParseEmpiricalLateralSupport(const YAML::Node& n) {
  EmpiricalLateralSupport out;
  if (!n || n.IsNull()) return out;
  if (!n.IsMap() || !n["contract"] ||
      n["contract"].as<std::string>() != "super_gate3_empirical_lateral_v1")
    throw std::invalid_argument("empirical lateral support requires its versioned contract");
  auto read = [&](const char* key) {
    auto v=n[key];
    if (!v || !v.IsSequence() || v.size()!=2)
      throw std::invalid_argument("empirical lateral support requires two complete intervals");
    std::array<double,2> a{v[0].as<double>(),v[1].as<double>()};
    if (!std::isfinite(a[0]) || !std::isfinite(a[1]) || a[0]>=a[1])
      throw std::invalid_argument("empirical lateral support interval must be finite and ordered");
    return a;
  };
  out.forehand=read("forehand_y_range");out.backhand=read("backhand_y_range");
  if (!n["evidence_sha256"])
    throw std::invalid_argument("empirical lateral support requires a frozen evidence digest");
  out.evidence_sha256=n["evidence_sha256"].as<std::string>();
  if (out.evidence_sha256.size()!=64 || out.evidence_sha256.find_first_not_of("0123456789abcdef")!=std::string::npos)
    throw std::invalid_argument("empirical lateral support evidence digest is not SHA256");
  out.enabled=true;return out;
}
inline std::array<double,2> EffectiveLateralSupport(
    const EmpiricalLateralSupport& support,int clip,const std::array<double,2>& trained) {
  if (clip!=0 && clip!=1) throw std::invalid_argument("invalid support clip");
  return !support.enabled?trained:clip==0?support.forehand:support.backhand;
}
}  // namespace a3_pingpong
