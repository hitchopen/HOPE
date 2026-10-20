#include "a3_pingpong/pp_serve_controller.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace {
using namespace a3_pingpong;
std::filesystem::path SegmentPath() {
  auto root = std::filesystem::path(__FILE__);
  for (int i=0; i<5; ++i) root = root.parent_path();
  return root / "assets/a3_runtime/serve/archive/a3p_op3_serve025_v21_segments_model14600.csv";
}
PpServe025FullbodyTimeline Segments() {
  PpServe025FullbodyTimeline t; std::string error;
  EXPECT_TRUE(t.LoadCsv(SegmentPath().string(), error)) << error;
  return t;
}
std::string ReadBundle() {
  std::ifstream f(SegmentPath()); std::stringstream s; s << f.rdbuf(); return s.str();
}
void ExpectRejected(std::string s) {
  auto path = std::filesystem::temp_directory_path() / ("serve_segments_" + std::to_string(getpid()) + ".csv");
  { std::ofstream f(path); f << s; }
  auto t = Segments(); std::string error;
  EXPECT_FALSE(t.LoadCsv(path.string(), error)); EXPECT_FALSE(error.empty());
  EXPECT_TRUE(t.empty()); EXPECT_FALSE(t.segments().present);
  std::filesystem::remove(path);
}
robot_io::RobotState State(const robot_io::RobotCommand& c) {
  robot_io::RobotState s; s.q=c.q_des; s.dq=Eigen::VectorXd::Zero(31);
  s.imu_quat_wxyz=Eigen::Vector4d(1,0,0,0); s.imu_gyro=Eigen::Vector3d::Zero(); return s;
}
TEST(ServeSegments, AnnotatedBundleAndPolicyBinding) {
  auto t=Segments(); const auto& b=t.segments();
  ASSERT_TRUE(b.present); EXPECT_EQ(b.branch_frame,146U); EXPECT_EQ(b.policy_ready_frame,70U);
  EXPECT_EQ(b.commands[0].size(),501U); EXPECT_EQ(b.commands[1].size(),322U); EXPECT_EQ(b.commands[2].size(),81U);
  EXPECT_NO_THROW(b.RequirePolicy(b.policy_sha256,b.deploy_sha256));
  EXPECT_THROW(b.RequirePolicy(std::string(64,'0'),b.deploy_sha256),std::invalid_argument);
  EXPECT_THROW(b.RequirePolicy(b.policy_sha256,std::string(64,'0')),std::invalid_argument);
}
TEST(ServeSegments, V23SelectsFrame110InBothModesAndRetainsLegacyServe) {
  PpServe025FullbodyTimeline t; std::string error;
  ASSERT_TRUE(t.LoadCsv((SegmentPath().parent_path().parent_path() / "motions" /
      "a3p_op3_serve025_v23_branch110_model14600.csv").string(), error)) << error;
  const auto b=t.segments();
  ASSERT_TRUE(b.present);
  EXPECT_EQ(b.branch_frame,110U);
  EXPECT_EQ(t.At(110).event,"SERVE_TO_POLICY");
  EXPECT_EQ(b.commands[0].size(),501U);
  EXPECT_EQ(b.commands[1].size(),358U);
  EXPECT_GT(b.commands[2].size(),b.policy_ready_frame+1);
  for (bool normal : {false,true}) {
    auto stand=b.commands[0].front();
    PpServeController c(t,stand.q_des,stand.kp,stand.kd,nullptr);
    c.SelectSegmentDestination(normal,b.policy_sha256,b.deploy_sha256);
    EXPECT_EQ(c.handoff_frame(),110U);
    EXPECT_NO_THROW(c.SetPolicyHandoffFrame(110));
    EXPECT_THROW(c.SetPolicyHandoffFrame(90),std::invalid_argument);
  }
  std::ifstream file(SegmentPath().parent_path().parent_path() / "motions" /
      "a3p_op3_serve025_v23_branch110_model14600.csv");
  std::stringstream stream; stream << file.rdbuf(); const auto original=stream.str();
  auto changed=original;
  auto tag=changed.find(",FRAME,SERVE_TO_POLICY,");
  ASSERT_NE(tag,std::string::npos);
  changed.replace(tag,23,",FRAME,NONE,"); ExpectRejected(changed);
  changed=original;
  auto duplicate=changed.find(",FRAME,NONE,109,109,");
  ASSERT_NE(duplicate,std::string::npos);
  changed.replace(duplicate,12,",FRAME,SERVE_TO_POLICY,"); ExpectRejected(changed);
  changed=original;
  changed.erase(changed.find("#HOPE_SERVE_SEGMENTS_V1")); ExpectRejected(changed);
}
TEST(ServeSegments, RejectsMissingAndRelabelledSegments) {
  auto s=ReadBundle(); s.erase(s.find("SERVE_TO_POLICY,0,")); ExpectRejected(s);
  s=ReadBundle(); s.replace(s.find("SERVE_TO_POLICY,0,"),15,"SERVE_TO_STAND,0"); ExpectRejected(s);
}
TEST(ServeSegments, RejectsMissingReadyAnnotationAndIncompleteEnd) {
  auto s=ReadBundle(); auto pos=s.find(",POLICY_READY,"); s.replace(pos,14,",SAMPLE,"); ExpectRejected(s);
  s=ReadBundle(); s.erase(s.rfind("SERVE_TO_POLICY,")); ExpectRejected(s);
}
TEST(ServeSegments, RejectsInvalidIdentityTimeAndNumericCommands) {
  auto s=ReadBundle(); auto pos=s.find("#HOPE_SERVE_SEGMENTS_V1,146,0.15,")+32;
  s[pos]='z'; ExpectRejected(s);
  s=ReadBundle(); pos=s.find("SERVE_TO_POLICY,1,0.01,"); ASSERT_NE(pos,std::string::npos);
  s.replace(pos,22,"SERVE_TO_POLICY,1,0.02,"); ExpectRejected(s);
  s=ReadBundle(); pos=s.find("SERVE_TO_POLICY,0,0.00,BEGIN,")+27;
  s.replace(pos,s.find(',',pos)-pos,"nan"); ExpectRejected(s);
}
TEST(ServeSegments, LegacyCsvStillLoadsWithoutSegments) {
  auto s=ReadBundle(); s.erase(s.find("#HOPE_SERVE_SEGMENTS_V1"));
  auto path=std::filesystem::temp_directory_path()/("serve_legacy_"+std::to_string(getpid())+".csv");
  {std::ofstream f(path);f<<s;}
  PpServe025FullbodyTimeline t;std::string error; EXPECT_TRUE(t.LoadCsv(path.string(),error))<<error;
  EXPECT_FALSE(t.segments().present); EXPECT_EQ(t.size(),468U); std::filesystem::remove(path);
}
TEST(ServeSegments, ModesPlaySelectedTailAndNormalRequiresMeasuredReady) {
  for (bool normal : {false,true}) {
    auto t=Segments(); const auto b=t.segments(); auto stand=b.commands[0].front();
    PpServeController c(std::move(t),stand.q_des,stand.kp,stand.kd,nullptr);
    EXPECT_THROW(c.SetPolicyHandoffFrame(110),std::invalid_argument);
    if(normal) EXPECT_THROW(c.SelectSegmentDestination(true,"bad","bad"),std::invalid_argument);
    c.SelectSegmentDestination(normal,b.policy_sha256,b.deploy_sha256);
    auto cmd=stand; auto state=State(cmd); c.Start();
    for(int tick=0;tick<501;++tick) { ASSERT_TRUE(c.ComputeCommand(tick,state,cmd)); state=State(cmd); }
    ASSERT_EQ(c.state(),ServeControllerState::kWaitReadyToServe);
    c.TriggerReadyToServe();
    for(int tick=0;tick<101+147;++tick) {ASSERT_TRUE(c.ComputeCommand(tick,state,cmd));state=State(cmd);}
    ASSERT_EQ(c.state(),ServeControllerState::kHandoffReady);
    const auto& clip=b.commands[normal?2:1];
    const auto source=cmd;
    for(std::size_t idx=normal?0:1;idx<clip.size();++idx) {
      ASSERT_TRUE(c.ComputeCommand(idx,state,cmd));
      EXPECT_TRUE(cmd.q_des.isApprox(clip[idx].q_des,1e-7));
      EXPECT_TRUE(cmd.dq_des.isApprox(clip[idx].dq_des,1e-7));
      EXPECT_TRUE(cmd.tau_ff.isApprox(clip[idx].tau_ff,1e-10));
      EXPECT_TRUE(cmd.kp.isApprox(clip[idx].kp,1e-10));
      EXPECT_TRUE(cmd.kd.isApprox(clip[idx].kd,1e-10));
      if (normal) { state=State(cmd); state.dq.setConstant(.5); } else state=State(cmd);
      if(normal && idx==0) EXPECT_TRUE(cmd.q_des.isApprox(source.q_des,1e-12));
    }
    if(normal) {
      EXPECT_EQ(c.state(),ServeControllerState::kHandoffReady);
      for(int tick=0;tick<5;++tick) {state=State(cmd); ASSERT_TRUE(c.ComputeCommand(tick,state,cmd));}
      EXPECT_GT((cmd.q_des-stand.q_des).cwiseAbs().maxCoeff(),.2);
    }
    EXPECT_EQ(c.state(),ServeControllerState::kComplete);
  }
}
TEST(ServeSegments, NeverSettledPolicyTargetFaultsInsteadOfEngagingActor) {
  auto t=Segments(); const auto b=t.segments(); auto stand=b.commands[0].front();
  PpServeController c(std::move(t),stand.q_des,stand.kp,stand.kd,nullptr);
  c.SelectSegmentDestination(true,b.policy_sha256,b.deploy_sha256);
  auto cmd=stand; auto state=State(cmd); c.Start();
  for(int i=0;i<501;++i){ASSERT_TRUE(c.ComputeCommand(i,state,cmd));state=State(cmd);}
  c.TriggerReadyToServe();
  for(int i=0;i<101+147;++i){ASSERT_TRUE(c.ComputeCommand(i,state,cmd));state=State(cmd);}
  for(int i=0;i<183;++i){state=State(cmd);state.dq.setConstant(.5);ASSERT_TRUE(c.ComputeCommand(i,state,cmd));}
  EXPECT_EQ(c.state(),ServeControllerState::kFault);
}
} // namespace
