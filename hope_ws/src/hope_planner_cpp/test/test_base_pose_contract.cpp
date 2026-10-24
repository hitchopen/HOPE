#include "a3_pingpong/pp_obs_builder.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include "hope_planner_cpp/base_pose_orientation.hpp"
#include "hope_planner_cpp/hit_plane_manager.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/target_turn_guard.hpp"
#include <gtest/gtest.h>
using namespace hope_planner_cpp;
namespace {
std::vector<double> wire(const Eigen::Quaterniond &q) {
  return {2,     1,     1,     1800000000, 0, -.43, -.7, .76,
          q.w(), q.x(), q.y(), q.z(),      1, 63,   1,   1};
}
Eigen::Quaterniond yaw(double deg) {
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(deg * 3.141592653589793 / 180, Vec3::UnitZ()));
}
TEST(BaseContract, BP01_WxyzRoundTripAndHeading) {
  for (double angle : {0., 30., -30., 50.}) {
    auto expected = yaw(angle);
    auto q = base_orientation_wxyz(wire(expected));
    ASSERT_TRUE(q);
    EXPECT_TRUE(
        q->toRotationMatrix().isApprox(expected.toRotationMatrix(), 1e-12));
  }
}
TEST(BaseContract, BP02_AnalyticTurnGateAtRotatedYaw) {
  for (double angle : {0., 30., -30., 50.}) {
    auto q = base_orientation_wxyz(wire(yaw(angle)));
    ASSERT_TRUE(q);
    auto d =
        check_target_turn(Vec3(.15, -.7, .3), Vec3(-.43, -.7, 0), *q, true);
    EXPECT_NEAR(d.base_yaw_deg, angle, 1e-12);
    EXPECT_EQ(d.allowed, std::abs(angle) <= 40);
    const Vec3 aligned = Vec3(-.43, -.7, 0) + .58 * (*q * Vec3::UnitX());
    EXPECT_TRUE(
        check_target_turn(aligned, Vec3(-.43, -.7, 0), *q, true).allowed);
  }
}
TEST(BaseContract, BP03_PitchRollAndSignAreEquivalent) {
  Eigen::Quaterniond original = Eigen::AngleAxisd(.5, Vec3::UnitZ()) *
                                Eigen::AngleAxisd(.2, Vec3::UnitY()) *
                                Eigen::AngleAxisd(-.3, Vec3::UnitX());
  for (double sign : {-1., 1.}) {
    auto q = original;
    q.coeffs() *= sign;
    auto decoded = base_orientation_wxyz(wire(q));
    ASSERT_TRUE(decoded);
    EXPECT_TRUE(decoded->toRotationMatrix().isApprox(
        original.toRotationMatrix(), 1e-12));
  }
}
TEST(BaseContract, BP04_InvalidStaleWorldAndReorderedInputsFailClosed) {
  HitPlaneManager p({}, "table", "survey", 1);
  auto w = wire(yaw(30));
  ASSERT_TRUE(p.observe(w, 1, 1800000000000000000LL, true));
  EXPECT_FALSE(p.observe(w, 2, 1800000000000000000LL, true));
  w[2] = 2;
  w[4] = 1000000;
  w[15] = 2;
  EXPECT_FALSE(p.observe(w, 3, 1800000000001000000LL, true));
  w[15] = 1;
  EXPECT_FALSE(p.observe(w, 4, 1800000000400000000LL, true));
  w[8] = w[9] = w[10] = w[11] = 0;
  EXPECT_FALSE(base_orientation_wxyz(w));
  EXPECT_FALSE(p.observe(w, 5, 1800000000001000000LL, true));
  w = wire(yaw(30));
  w[2] = 3;
  w[4] = 2000000;
  EXPECT_TRUE(p.observe(w, 6, 1800000000002000000LL, true));
}
TEST(BaseContract, BP05_ActualPackerDecoderAnd110DHeightOnceAtRotatedBase) {
  RacketCommand c;
  c.valid = true;
  c.position = Vec3(.15, -.7, .35);
  c.velocity = Vec3(3, 0, .2);
  auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                 std::chrono::system_clock::now().time_since_epoch())
                 .count();
  auto packet = Schema2Packer::pack(&c, 1, (now + 400000000) * 1e-9, .76, now,
                                    {1, 1, 1}, 20, .1);
  a3_pingpong::PpRacketTargetInput receiver(2);
  receiver.SetFromFlat(
      std::vector<double>(packet.values.begin(), packet.values.end()));
  auto d = receiver.Latest();
  ASSERT_TRUE(d.has_valid);
  EXPECT_NEAR(d.cmd.pos_w.z(), 1.11, 1e-12);
  a3_pingpong::PpRobotState robot;
  robot.base_pos_w = Vec3(-.43, -.7, .76);
  auto q = yaw(30);
  robot.base_quat_w << q.w(), q.x(), q.y(), q.z();
  robot.base_ang_vel_b.setZero();
  robot.q = Eigen::VectorXd::Zero(31);
  robot.qd = robot.q;
  a3_pingpong::PpRacketTarget target;
  target.pos_w = d.cmd.pos_w;
  target.vel_w = d.cmd.vel_w;
  target.time_to_strike = d.control_time_to_strike_s;
  target.swing_sign = 1;
  target.base_target_xy = robot.base_pos_w.head<2>();
  auto obs = a3_pingpong::build_obs_110(robot, target, robot.q, robot.q);
  ASSERT_EQ(obs.size(), 110);
  EXPECT_TRUE(obs.segment<3>(103).isApprox(Vec3(.58, 0, .35), 1e-12));
  EXPECT_TRUE(obs.segment<3>(106).isApprox(c.velocity, 1e-12));
  EXPECT_NEAR(
      (c.position - (robot.base_pos_w - Vec3(0, 0, .76))).head<2>().norm(), .58,
      1e-12);
}
} // namespace
