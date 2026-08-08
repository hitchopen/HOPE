#include "a3_pingpong/pp_racket_fk.hpp"

#include <gtest/gtest.h>

namespace a3_pingpong {

TEST(PpRacketFkPose, PositionWrapperAndRotationStayConsistent) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(31);
  q[2] = 0.13;
  q[13] = -0.45;
  q[18] = 0.21;
  q[24] = 0.72;
  q[28] = -0.18;
  q[30] = 0.31;

  const RacketPosePelvis pose = racket_pose_pelvis(q);
  EXPECT_TRUE(pose.position.allFinite());
  EXPECT_TRUE(pose.rotation.allFinite());
  EXPECT_LT((pose.position - racket_pos_pelvis(q)).norm(), 1.0e-12);
  EXPECT_LT((pose.rotation.transpose() * pose.rotation - Mat3::Identity()).norm(), 1.0e-12);
  EXPECT_NEAR(pose.rotation.determinant(), 1.0, 1.0e-12);
  EXPECT_NEAR(pose.rotation.col(1).norm(), 1.0, 1.0e-12);
}

TEST(PpRacketFkPose, WristYawRotatesFaceNormal) {
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(31);
  Eigen::VectorXd q1 = q0;
  q1[30] = 0.5;

  const Vec3 normal0 = racket_pose_pelvis(q0).rotation.col(1);
  const Vec3 normal1 = racket_pose_pelvis(q1).rotation.col(1);
  EXPECT_GT((normal1 - normal0).norm(), 0.1);
  EXPECT_NEAR(normal1.norm(), 1.0, 1.0e-12);
}

}  // namespace a3_pingpong
