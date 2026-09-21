#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace motion_capture_tracking::detail
{

struct MarkerAssociationDefinition
{
  std::uint32_t member_id;
  std::array<double, 3> expected_world_m;
};

struct MarkerAssociationSample
{
  std::uint32_t model_id;
  std::uint32_t member_id;
  std::array<double, 3> world_position_m;
  std::uint16_t params;
};

struct MarkerAssociationResult
{
  static constexpr std::size_t no_sample =
    std::numeric_limits<std::size_t>::max();

  std::vector<std::size_t> sample_indices;
  std::optional<std::int64_t> confirmed_member_id_offset;
  std::size_t id_verified_count{0};
  std::size_t selected_count{0};
  std::size_t same_model_physical_count{0};
  bool used_geometric_fallback{false};
  bool used_pose_invariant_geometry_confirmation{false};
  double pose_invariant_max_pairwise_error_m{0.0};
};

inline bool finite_position(const std::array<double, 3> &position)
{
  return std::all_of(
    position.begin(), position.end(),
    [](double value) {return std::isfinite(value);});
}

inline bool physical_sample(const MarkerAssociationSample &sample)
{
  // NatNet bit 0: occluded. Bit 1: reconstructed by the point-cloud solve.
  // Calibration must not count a model-filled position as a physical marker.
  return (sample.params & 0x01U) == 0U &&
         (sample.params & 0x02U) != 0U &&
         finite_position(sample.world_position_m);
}

inline double position_distance_m(
  const std::array<double, 3> &left,
  const std::array<double, 3> &right)
{
  const double dx = left[0] - right[0];
  const double dy = left[1] - right[1];
  const double dz = left[2] - right[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline std::optional<std::uint32_t> apply_member_id_offset(
  std::uint32_t definition_member_id, std::int64_t raw_minus_definition)
{
  const std::int64_t raw_id =
    static_cast<std::int64_t>(definition_member_id) + raw_minus_definition;
  if (raw_id < 0 ||
    raw_id > static_cast<std::int64_t>(
      std::numeric_limits<std::uint32_t>::max()))
  {
    return std::nullopt;
  }
  return static_cast<std::uint32_t>(raw_id);
}

// Confirm a complete exact-MemberID convention without trusting the current
// rigid-body pose. A rigid transform changes every world position, but it does
// not change the pairwise distances among the markers. This is the safe
// recovery path when NatNet publishes a temporarily flipped/stale rigid-body
// pose while all physical labeled samples and their IDs are present.
inline bool confirm_member_id_offset_by_pairwise_geometry(
  const std::vector<MarkerAssociationDefinition> &definitions,
  const std::vector<MarkerAssociationSample> &samples,
  std::uint32_t model_id, std::int64_t raw_minus_definition,
  double geometric_gate_m, std::vector<std::size_t> &sample_indices,
  double &max_pairwise_error_m)
{
  sample_indices.assign(
    definitions.size(), MarkerAssociationResult::no_sample);
  max_pairwise_error_m = 0.0;
  if (definitions.size() < 3 || !(geometric_gate_m > 0.0) ||
    !std::isfinite(geometric_gate_m))
  {
    return false;
  }

  for (std::size_t definition_index = 0;
    definition_index < definitions.size(); ++definition_index)
  {
    const auto expected_raw_id = apply_member_id_offset(
      definitions[definition_index].member_id, raw_minus_definition);
    if (!expected_raw_id.has_value()) {
      return false;
    }
    std::size_t match_count = 0;
    for (std::size_t sample_index = 0;
      sample_index < samples.size(); ++sample_index)
    {
      const auto &sample = samples[sample_index];
      if (sample.model_id == model_id &&
        sample.member_id == *expected_raw_id && physical_sample(sample))
      {
        sample_indices[definition_index] = sample_index;
        ++match_count;
      }
    }
    // Duplicate IDs are ambiguous even if one happens to fit geometrically.
    if (match_count != 1) {
      return false;
    }
  }

  double maximum_definition_span_m = 0.0;
  for (std::size_t left = 0; left < definitions.size(); ++left) {
    for (std::size_t right = left + 1; right < definitions.size(); ++right) {
      const double definition_distance = position_distance_m(
        definitions[left].expected_world_m,
        definitions[right].expected_world_m);
      const double sample_distance = position_distance_m(
        samples[sample_indices[left]].world_position_m,
        samples[sample_indices[right]].world_position_m);
      maximum_definition_span_m = std::max(
        maximum_definition_span_m, definition_distance);
      max_pairwise_error_m = std::max(
        max_pairwise_error_m,
        std::abs(definition_distance - sample_distance));
      if (max_pairwise_error_m > geometric_gate_m) {
        return false;
      }
    }
  }

  // Reject a coincident/tiny cluster: it cannot provide meaningful geometric
  // evidence for an ID convention at this gate scale.
  return maximum_definition_span_m > 2.0 * geometric_gate_m;
}

// NatNet ModelDef transmits marker arrays but no explicit MemberID field.
// Motive/NatNet combinations have been observed to label the corresponding
// frame samples with either the zero-based ModelDef index or index + 1. Infer
// the relationship only from a complete, unique per-model ID set. This keeps
// an incomplete frame from silently changing the association convention.
inline std::optional<std::int64_t> infer_complete_member_id_offset(
  const std::vector<MarkerAssociationDefinition> &definitions,
  const std::vector<MarkerAssociationSample> &samples,
  std::uint32_t model_id)
{
  if (definitions.empty()) {
    return std::nullopt;
  }

  std::set<std::uint32_t> definition_ids;
  for (const auto &definition : definitions) {
    definition_ids.insert(definition.member_id);
  }
  if (definition_ids.size() != definitions.size()) {
    return std::nullopt;
  }

  std::set<std::uint32_t> raw_ids;
  for (const auto &sample : samples) {
    if (sample.model_id == model_id && physical_sample(sample)) {
      raw_ids.insert(sample.member_id);
    }
  }
  if (raw_ids.size() != definition_ids.size()) {
    return std::nullopt;
  }

  auto definition_iter = definition_ids.begin();
  auto raw_iter = raw_ids.begin();
  const std::int64_t offset =
    static_cast<std::int64_t>(*raw_iter) -
    static_cast<std::int64_t>(*definition_iter);
  for (; definition_iter != definition_ids.end();
    ++definition_iter, ++raw_iter)
  {
    if (static_cast<std::int64_t>(*raw_iter) -
      static_cast<std::int64_t>(*definition_iter) != offset ||
      !apply_member_id_offset(*definition_iter, offset).has_value())
    {
      return std::nullopt;
    }
  }
  return offset;
}

inline MarkerAssociationResult associate_rigid_body_markers(
  const std::vector<MarkerAssociationDefinition> &definitions,
  const std::vector<MarkerAssociationSample> &samples,
  std::uint32_t model_id, double geometric_gate_m,
  std::optional<std::int64_t> cached_member_id_offset = std::nullopt)
{
  MarkerAssociationResult result;
  result.sample_indices.assign(
    definitions.size(), MarkerAssociationResult::no_sample);
  if (definitions.empty() || !(geometric_gate_m > 0.0) ||
    !std::isfinite(geometric_gate_m))
  {
    return result;
  }

  std::set<std::uint32_t> same_model_ids;
  for (const auto &sample : samples) {
    if (sample.model_id == model_id && physical_sample(sample)) {
      same_model_ids.insert(sample.member_id);
    }
  }
  result.same_model_physical_count = same_model_ids.size();

  const auto inferred_offset = infer_complete_member_id_offset(
    definitions, samples, model_id);
  // A cached convention has already been proven by a complete frame and
  // geometry. From then on MemberID is the identity contract: marker motion,
  // cloth deformation, or momentary rigid-body solve error must not make a
  // real physical sample disappear merely because it moved beyond the initial
  // discovery gate.
  const bool using_cached_offset = cached_member_id_offset.has_value();
  const auto candidate_offset = using_cached_offset
    ? cached_member_id_offset : inferred_offset;
  std::set<std::size_t> used_samples;

  if (candidate_offset.has_value()) {
    for (std::size_t definition_index = 0;
      definition_index < definitions.size(); ++definition_index)
    {
      const auto expected_raw_id = apply_member_id_offset(
        definitions[definition_index].member_id, *candidate_offset);
      if (!expected_raw_id.has_value()) {
        continue;
      }
      std::size_t nearest_index = MarkerAssociationResult::no_sample;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (std::size_t sample_index = 0;
        sample_index < samples.size(); ++sample_index)
      {
        const auto &sample = samples[sample_index];
        if (sample.model_id != model_id ||
          sample.member_id != *expected_raw_id ||
          !physical_sample(sample))
        {
          continue;
        }
        const double distance = position_distance_m(
          definitions[definition_index].expected_world_m,
          sample.world_position_m);
        if ((using_cached_offset || distance <= geometric_gate_m) &&
          distance < nearest_distance)
        {
          nearest_distance = distance;
          nearest_index = sample_index;
        }
      }
      if (nearest_index != MarkerAssociationResult::no_sample) {
        result.sample_indices[definition_index] = nearest_index;
        used_samples.insert(nearest_index);
        ++result.id_verified_count;
      }
    }
  }

  // The pointwise discovery check above depends on the rigid-body pose. If a
  // complete exact-ID frame is present but that pose is temporarily flipped,
  // prove the same convention from the full pairwise distance matrix instead.
  // This retains the one-time geometry requirement without making calibration
  // availability depend on a single NatNet pose solve.
  if (!using_cached_offset && inferred_offset.has_value() &&
    result.id_verified_count != definitions.size())
  {
    std::vector<std::size_t> pose_invariant_indices;
    double max_pairwise_error_m = 0.0;
    if (confirm_member_id_offset_by_pairwise_geometry(
        definitions, samples, model_id, *inferred_offset,
        geometric_gate_m, pose_invariant_indices, max_pairwise_error_m))
    {
      result.sample_indices = std::move(pose_invariant_indices);
      result.id_verified_count = definitions.size();
      result.used_pose_invariant_geometry_confirmation = true;
      result.pose_invariant_max_pairwise_error_m = max_pairwise_error_m;
      used_samples.clear();
      used_samples.insert(
        result.sample_indices.begin(), result.sample_indices.end());
    }
  }

  // A complete set is accepted as a new convention only after every ID
  // association passes either pointwise pose geometry or the pose-invariant
  // pairwise geometry contract. Thus equal ID sets alone cannot recreate a
  // silent shift.
  if (!using_cached_offset && inferred_offset.has_value() &&
    result.id_verified_count == definitions.size())
  {
    result.confirmed_member_id_offset = inferred_offset;
  }

  struct GeometricCandidate
  {
    double distance;
    std::size_t definition_index;
    std::size_t sample_index;
  };
  std::vector<GeometricCandidate> candidates;
  // Once an ID convention is cached, never substitute a differently numbered
  // marker by proximity. A missing exact MemberID remains visibly missing.
  if (using_cached_offset) {
    result.selected_count = result.id_verified_count;
    return result;
  }
  for (std::size_t definition_index = 0;
    definition_index < definitions.size(); ++definition_index)
  {
    if (result.sample_indices[definition_index] !=
      MarkerAssociationResult::no_sample)
    {
      continue;
    }
    for (std::size_t sample_index = 0;
      sample_index < samples.size(); ++sample_index)
    {
      if (used_samples.count(sample_index) != 0U ||
        samples[sample_index].model_id != model_id ||
        !physical_sample(samples[sample_index]))
      {
        continue;
      }
      const double distance = position_distance_m(
        definitions[definition_index].expected_world_m,
        samples[sample_index].world_position_m);
      if (distance <= geometric_gate_m) {
        candidates.push_back({distance, definition_index, sample_index});
      }
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const GeometricCandidate &left, const GeometricCandidate &right) {
      if (left.distance != right.distance) {
        return left.distance < right.distance;
      }
      if (left.definition_index != right.definition_index) {
        return left.definition_index < right.definition_index;
      }
      return left.sample_index < right.sample_index;
    });
  for (const auto &candidate : candidates) {
    if (result.sample_indices[candidate.definition_index] !=
      MarkerAssociationResult::no_sample ||
      used_samples.count(candidate.sample_index) != 0U)
    {
      continue;
    }
    result.sample_indices[candidate.definition_index] =
      candidate.sample_index;
    used_samples.insert(candidate.sample_index);
    result.used_geometric_fallback = true;
  }

  result.selected_count = static_cast<std::size_t>(std::count_if(
      result.sample_indices.begin(), result.sample_indices.end(),
      [](std::size_t index) {
        return index != MarkerAssociationResult::no_sample;
      }));
  return result;
}

}  // namespace motion_capture_tracking::detail
