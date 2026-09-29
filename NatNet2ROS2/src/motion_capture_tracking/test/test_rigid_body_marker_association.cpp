#include "motion_capture_tracking/rigid_body_marker_association.hpp"

#include <cassert>
#include <cstdint>
#include <optional>
#include <vector>

namespace detail = motion_capture_tracking::detail;

namespace
{

detail::MarkerAssociationDefinition definition(
  std::uint32_t member_id, double x)
{
  return {member_id, {x, 0.0, 0.0}};
}

detail::MarkerAssociationSample sample(
  std::uint32_t model_id, std::uint32_t member_id, double x,
  std::uint16_t params = 0x02U)
{
  return {model_id, member_id, {x, 0.0, 0.0}, params};
}

void assert_mapping_positions(
  const detail::MarkerAssociationResult &result,
  const std::vector<detail::MarkerAssociationDefinition> &definitions,
  const std::vector<detail::MarkerAssociationSample> &samples)
{
  assert(result.selected_count == definitions.size());
  for (std::size_t index = 0; index < definitions.size(); ++index) {
    assert(result.sample_indices[index] !=
           detail::MarkerAssociationResult::no_sample);
    assert(samples[result.sample_indices[index]].world_position_m ==
           definitions[index].expected_world_m);
  }
}

}  // namespace

int main()
{
  const std::vector<detail::MarkerAssociationDefinition> definitions{
    definition(0, 0.0), definition(1, 0.1), definition(2, 0.2)};

  // Current UCB_P2 failure: ModelDef is 0..N-1 while raw NatNet is 1..N.
  // Same-ID matching would shift two markers and lose Marker001. The complete
  // set plus geometry must instead prove raw_member = definition_member + 1.
  const std::vector<detail::MarkerAssociationSample> one_based{
    sample(99, 1, 5.0),  // unrelated model must not influence ID inference
    sample(15, 1, 0.0), sample(15, 2, 0.1), sample(15, 3, 0.2)};
  auto result = detail::associate_rigid_body_markers(
    definitions, one_based, 15, 0.005);
  assert(result.confirmed_member_id_offset == std::optional<std::int64_t>(1));
  assert(result.id_verified_count == 3);
  assert(!result.used_geometric_fallback);
  assert(!result.used_pose_invariant_geometry_confirmation);
  assert_mapping_positions(result, definitions, one_based);

  // A transiently flipped/stale rigid-body pose moves every expected point,
  // but the complete exact-ID sample set retains the same pairwise geometry.
  // UCB_P2 calibration must establish and cache the +1 convention immediately
  // instead of reporting 0/10 until the pose solve happens to recover.
  const std::vector<detail::MarkerAssociationDefinition> pose_flipped_definitions{
    {0, {0.0, 0.0, 0.0}},
    {1, {0.10, 0.0, 0.0}},
    {2, {0.02, 0.18, 0.03}}};
  const std::vector<detail::MarkerAssociationSample> rigidly_transformed{
    sample(15, 1, 2.0),
    {15, 2, {2.0, -0.10, 0.0}, 0x02U},
    {15, 3, {2.18, -0.02, 0.03}, 0x02U}};
  result = detail::associate_rigid_body_markers(
    pose_flipped_definitions, rigidly_transformed, 15, 0.005);
  assert(result.confirmed_member_id_offset == std::optional<std::int64_t>(1));
  assert(result.id_verified_count == 3);
  assert(result.selected_count == 3);
  assert(result.used_pose_invariant_geometry_confirmation);
  assert(result.pose_invariant_max_pairwise_error_m < 1.0e-12);
  assert(!result.used_geometric_fallback);
  assert(result.sample_indices[0] == 0);
  assert(result.sample_indices[1] == 1);
  assert(result.sample_indices[2] == 2);

  // A native zero-based stream remains supported.
  const std::vector<detail::MarkerAssociationSample> zero_based{
    sample(15, 0, 0.0), sample(15, 1, 0.1), sample(15, 2, 0.2)};
  result = detail::associate_rigid_body_markers(
    definitions, zero_based, 15, 0.005);
  assert(result.confirmed_member_id_offset == std::optional<std::int64_t>(0));
  assert_mapping_positions(result, definitions, zero_based);

  // Non-contiguous ModelDef IDs are valid when the complete raw set has one
  // consistent offset; the logic does not hard-code 0/1-based numbering.
  const std::vector<detail::MarkerAssociationDefinition> non_contiguous{
    definition(4, 0.0), definition(9, 0.1), definition(20, 0.2)};
  const std::vector<detail::MarkerAssociationSample> shifted_non_contiguous{
    sample(15, 11, 0.0), sample(15, 16, 0.1), sample(15, 27, 0.2)};
  result = detail::associate_rigid_body_markers(
    non_contiguous, shifted_non_contiguous, 15, 0.005);
  assert(result.confirmed_member_id_offset == std::optional<std::int64_t>(7));
  assert_mapping_positions(
    result, non_contiguous, shifted_non_contiguous);

  // A transient incomplete frame cannot infer a new convention, but a
  // previously geometry-confirmed convention remains usable.
  const std::vector<detail::MarkerAssociationSample> partial{
    sample(15, 1, 0.0), sample(15, 3, 0.2)};
  result = detail::associate_rigid_body_markers(
    definitions, partial, 15, 0.005, 1);
  assert(!result.confirmed_member_id_offset.has_value());
  assert(result.id_verified_count == 2);
  assert(result.selected_count == 2);

  // After a complete frame has proved +1, exact MemberID remains authoritative
  // even when a physical cloth marker moves outside the 5 mm discovery gate.
  const std::vector<detail::MarkerAssociationSample> deformed_after_cache{
    sample(15, 1, 0.0), sample(15, 2, 0.14), sample(15, 3, 0.2)};
  result = detail::associate_rigid_body_markers(
    definitions, deformed_after_cache, 15, 0.005, 1);
  assert(result.id_verified_count == 3);
  assert(result.selected_count == 3);
  assert(!result.used_geometric_fallback);
  assert(result.sample_indices[0] == 0);
  assert(result.sample_indices[1] == 1);
  assert(result.sample_indices[2] == 2);

  // A cached convention never borrows a nearby marker with another MemberID.
  const std::vector<detail::MarkerAssociationSample> wrong_id_after_cache{
    sample(15, 1, 0.0), sample(15, 99, 0.1), sample(15, 3, 0.2)};
  result = detail::associate_rigid_body_markers(
    definitions, wrong_id_after_cache, 15, 0.005, 1);
  assert(result.id_verified_count == 2);
  assert(result.selected_count == 2);
  assert(!result.used_geometric_fallback);

  // Equal ID sets alone are not authoritative. If the same-ID geometry is
  // wrong, reject it and recover the correct permutation geometrically.
  const std::vector<detail::MarkerAssociationSample> permuted{
    sample(15, 0, 0.2), sample(15, 1, 0.0), sample(15, 2, 0.1)};
  result = detail::associate_rigid_body_markers(
    definitions, permuted, 15, 0.005);
  assert(!result.confirmed_member_id_offset.has_value());
  assert(result.id_verified_count == 0);
  assert(!result.used_pose_invariant_geometry_confirmation);
  assert(result.used_geometric_fallback);
  assert_mapping_positions(result, definitions, permuted);

  // A complete but scaled point set has matching ID cardinality only. Its
  // pairwise distances are wrong, so it must not establish a cached offset.
  const std::vector<detail::MarkerAssociationSample> scaled_wrong_geometry{
    sample(15, 1, 1.0), sample(15, 2, 1.2), sample(15, 3, 1.4)};
  result = detail::associate_rigid_body_markers(
    definitions, scaled_wrong_geometry, 15, 0.005);
  assert(!result.confirmed_member_id_offset.has_value());
  assert(result.id_verified_count == 0);
  assert(result.selected_count == 0);
  assert(!result.used_pose_invariant_geometry_confirmation);

  // Geometric fallback is rigid-body local. A coincident marker from another
  // model must never fill a missing UCB_P1/UCB_P2 definition.
  const std::vector<detail::MarkerAssociationSample> other_model_only{
    sample(99, 7, 0.0), sample(99, 8, 0.1), sample(99, 9, 0.2)};
  result = detail::associate_rigid_body_markers(
    definitions, other_model_only, 15, 0.005);
  assert(result.same_model_physical_count == 0);
  assert(result.selected_count == 0);
  assert(!result.used_geometric_fallback);

  // Occluded or model-only samples are never counted as physical evidence.
  const std::vector<detail::MarkerAssociationSample> non_physical{
    sample(15, 0, 0.0, 0x01U), sample(15, 1, 0.1, 0x04U),
    sample(15, 2, 0.2, 0x02U)};
  result = detail::associate_rigid_body_markers(
    definitions, non_physical, 15, 0.005);
  assert(result.same_model_physical_count == 1);
  assert(result.selected_count == 1);

  return 0;
}
