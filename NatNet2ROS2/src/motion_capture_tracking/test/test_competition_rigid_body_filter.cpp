#include "motion_capture_tracking/competition_rigid_body_filter.h"

#include <cassert>
#include <string_view>

using motion_capture_tracking::detail::competitionRigidBodyNames;
using motion_capture_tracking::detail::isCompetitionRigidBody;

int main()
{
  const auto &names = competitionRigidBodyNames();
  assert(names.size() == 3);
  assert(names[0] == "Ball");
  assert(names[1] == "P1");
  assert(names[2] == "P2");

  assert(isCompetitionRigidBody("Ball"));
  assert(isCompetitionRigidBody("P1"));
  assert(isCompetitionRigidBody("P2"));

  // The allowlist is exact and case-sensitive. Setup assets, arbitrary
  // Motive bodies, and marker/skeleton labels must never reach ROS output.
  assert(!isCompetitionRigidBody("ball"));
  assert(!isCompetitionRigidBody("Table"));
  assert(!isCompetitionRigidBody("PPT"));
  assert(!isCompetitionRigidBody("P3"));
  assert(!isCompetitionRigidBody("Marker001"));
  assert(!isCompetitionRigidBody(""));
  return 0;
}

