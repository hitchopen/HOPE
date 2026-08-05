#include "stdio.h"

#include "librigidbodytracker/rigid_body_tracker.h"
#include "librigidbodytracker/cloudlog.hpp"

using namespace librigidbodytracker;

int main()
{
  RigidBodyTracker ot({}, {}, {});
  // PointCloudLogger logger;
  PointCloudPlayer player;
  player.play(ot);
  puts("test OK");
}

