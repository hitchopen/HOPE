#include <gtest/gtest.h>
#include "a3_pingpong/pp_kernel_odometry.hpp"
using namespace a3_pingpong;
TEST(KernelStanceOdometry, StationaryDoesNotInventDrift) {
  KernelStanceOdometry odom;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(31), dq = q;
  const Vec4 orientation(1,0,0,0);
  auto p = odom.Step(q,dq,orientation,Vec3::Zero(),.02);
  for(int i=0;i<3000;++i) {
    const auto next=odom.Step(q,dq,orientation,Vec3::Zero(),.02);
    EXPECT_LT((next-p).norm(),1e-12);
  }
}
TEST(KernelStanceOdometry, PoseChangeIsYawEquivariantAndResetReanchors) {
  KernelStanceOdometry a,b;
  Eigen::VectorXd q=Eigen::VectorXd::Zero(31), dq=q;
  const Vec4 identity(1,0,0,0), rotated(std::cos(.85),0,0,std::sin(.85));
  a.Step(q,dq,identity,Vec3::Zero(),.02);
  b.Step(q,dq,rotated,Vec3::Zero(),.02);
  q[9]=.1; q[10]=.1;
  const auto pa=a.Step(q,dq,identity,Vec3::Zero(),.02);
  const auto pb=b.Step(q,dq,rotated,Vec3::Zero(),.02);
  EXPECT_GT(pa.head<2>().norm(),.001);
  EXPECT_LT((mat_from_quat(rotated)*pa-pb).norm(),1e-10);
  a.Reset();
  EXPECT_LT(a.Step(q,dq,identity,Vec3::Zero(),.02).head<2>().norm(),1e-12);
}
