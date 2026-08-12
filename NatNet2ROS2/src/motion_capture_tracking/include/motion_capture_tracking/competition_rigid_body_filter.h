#pragma once

#include <array>
#include <string_view>

namespace motion_capture_tracking
{
namespace detail
{

inline constexpr std::array<std::string_view, 3> kCompetitionRigidBodyNames = {
  "Ball",
  "P1",
  "P2",
};

constexpr const std::array<std::string_view, 3> & competitionRigidBodyNames()
{
  return kCompetitionRigidBodyNames;
}

constexpr bool isCompetitionRigidBody(std::string_view name)
{
  for (const auto allowed_name : kCompetitionRigidBodyNames) {
    if (name == allowed_name) {
      return true;
    }
  }
  return false;
}

}  // namespace detail
}  // namespace motion_capture_tracking

