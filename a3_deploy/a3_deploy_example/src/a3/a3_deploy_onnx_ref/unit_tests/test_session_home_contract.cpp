#include <gtest/gtest.h>
#include "a3_deploy/session_home_contract.hpp"

TEST(SessionHomeContract, ReentryReplacesHomeButMotionWithinSessionDoesNot) {
  a3_deploy::SessionHomeMailbox mailbox;
  const a3_deploy::SessionHomeReceipt first{100, -.50, -.76, .58, 7, 8};
  ASSERT_TRUE(mailbox.accept(first.pack(), 1000));
  auto changed = first;
  changed.x = -.40;
  EXPECT_FALSE(mailbox.accept(changed.pack(), 1100));
  EXPECT_DOUBLE_EQ(mailbox.value.x, -.50);
  changed.epoch = 101;
  ASSERT_TRUE(mailbox.accept(changed.pack(), 1200));
  EXPECT_FALSE(mailbox.accept(first.pack(), 1300));
  EXPECT_DOUBLE_EQ(mailbox.value.x + mailbox.value.plane_x, .18);
  EXPECT_EQ(mailbox.receipt_ns, 1200);
}

TEST(SessionHomeContract, StaleOrDifferentCalibrationCannotAuthorizePlanning) {
  a3_deploy::SessionHomeMailbox mailbox;
  EXPECT_FALSE(mailbox.fresh(1000, 500, 7, 8));
  ASSERT_TRUE(mailbox.accept({1, 100, -.50, -.76, .58, 7, 8}, 1000));
  EXPECT_TRUE(mailbox.fresh(1400, 500, 7, 8));
  EXPECT_FALSE(mailbox.fresh(1600, 500, 7, 8));
  EXPECT_FALSE(mailbox.fresh(1400, 500, 9, 8));
  EXPECT_FALSE(mailbox.fresh(1400, 500, 7, 9));
}

TEST(SessionHomeContract, OldHomeAndOldFixedPlaneCannotExecuteInNewSession) {
  const a3_deploy::SessionHomeReceipt home{101, -.50, -.76, .58, 7, 8};
  EXPECT_TRUE(home.matches_command(4, 101, 0, .08));
  EXPECT_FALSE(home.matches_command(4, 100, 0, .08));
  EXPECT_FALSE(home.matches_command(4, 101, 0, .15));
  EXPECT_FALSE(home.matches_command(4, 101, 1, .08));
  EXPECT_FALSE(home.matches_command(2, 101, 0, .08));
}

TEST(SessionHomeContract, RuntimeCapabilityAndPostureHeartbeatDoNotRecaptureHome) {
  a3_deploy::SessionHomeMailbox mailbox;
  a3_deploy::SessionHomeReceipt home{100, -.5, -.76, .58, 7, 8, true};
  ASSERT_TRUE(mailbox.accept(home.pack(1, false), 1000));
  ASSERT_TRUE(mailbox.accept(home.pack(2, true), 1100));
  EXPECT_TRUE(mailbox.posture_stable);
  EXPECT_FALSE(mailbox.accept(home.pack(2, false), 1200));
  EXPECT_EQ(mailbox.receipt_ns, 1100);
  EXPECT_TRUE(home.matches_command(4, 100, 0, .08));
  EXPECT_TRUE(home.matches_command(5, 100, 0, .18, -.40, 1));
  EXPECT_TRUE(home.matches_command(5, 100, 0, .18, -.40, 2));
  EXPECT_FALSE(home.matches_command(5, 100, 0, .18, -.50, 1));
  EXPECT_FALSE(home.matches_command(5, 99, 0, .18, -.40, 1));
  EXPECT_FALSE(home.matches_command(5, 100, 1, .18, -.40, 1));
  EXPECT_FALSE(home.matches_command(5, 100, 0, .18, -.40, 0));
  home.runtime_planes = false;
  EXPECT_FALSE(home.matches_command(5, 100, 0, .18, -.40, 1));
}

TEST(SessionHomeContract, PostureGateRequiresContinuousFreshWaitingWindow) {
  a3_deploy::PlanePostureGate gate;
  for(int i=0;i<10;++i) EXPECT_FALSE(gate.update(10+i*.02,true,true,.01,.1,-1));
  EXPECT_TRUE(gate.update(10.21,true,true,.01,.1,-1));
  EXPECT_FALSE(gate.update(10.23,false,true,.01,.1,-1));
  for(int i=0;i<11;++i) gate.update(10.25+i*.02,true,true,.01,.1,-1);
  EXPECT_TRUE(gate.update(10.47,true,true,.01,.1,-1));
  EXPECT_FALSE(gate.update(10.60,true,true,.01,.1,-1)); // stream gap resets window
  EXPECT_FALSE(gate.update(10.62,true,false,.01,.1,-1));
  EXPECT_FALSE(gate.update(10.64,true,true,.04,.1,-1));
  EXPECT_FALSE(gate.update(10.66,true,true,.01,.3,-1));
  EXPECT_FALSE(gate.update(10.68,true,true,.01,.1,-.9));
}
