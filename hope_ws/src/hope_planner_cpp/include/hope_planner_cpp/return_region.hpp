#pragma once
#include "hope_planner_cpp/probabilistic_types.hpp"
#include <ostream>
namespace hope_planner_cpp {
enum class ContactSide { kOpponent, kReceiver, kAmbiguous };
enum class HistoryContinuity { kComplete, kIncomplete, kUnknown };
enum class ContactProvenance {
  kObservedConfirmed,
  kExternalSideVerified,
  kInferredUnconfirmed,
  kUnknown
};
const char *name(ContactSide);
const char *name(HistoryContinuity);
const char *name(ContactProvenance);
struct ContactHistoryEvent {
  std::int64_t source_time_ns = 0;
  double source_time_s = 0.;
  std::string source_id;
  double confidence = 1.;
  Vec3 position = Vec3::Zero();
  ContactSide side = ContactSide::kAmbiguous;
  ContactProvenance provenance = ContactProvenance::kUnknown;
  std::uint64_t epoch = 0;
};
struct BounceContext {
  HistoryContinuity continuity = HistoryContinuity::kUnknown;
  std::vector<ContactHistoryEvent> events;
  bool conflict = false;
  int count_only_hint = -1;
};
struct ReturnRegionPolicy {
  double post_hit_distance_m = .50;
  double side_tolerance_m = .020;
  double ground_z_m = 0.;
  double stall_speed_mps = .05;
  double stall_duration_s = .020;
  double tie_tolerance_s = 1e-9;
  double publication_period_s = 0., max_source_age_s = .030;
  void validate() const;
};
ContactSide contact_side(double x, const ProbabilisticConfig &,
                         const ReturnRegionPolicy &);
std::string return_region_config_identity(const ProbabilisticConfig &,
                                          const ReturnRegionPolicy &,
                                          const std::string &frame);
std::string sha256(const std::string &);
std::string physics_fingerprint(const ProbabilisticConfig &);
} // namespace hope_planner_cpp
