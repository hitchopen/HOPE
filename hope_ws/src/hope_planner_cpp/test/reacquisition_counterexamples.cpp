#include "hope_planner_cpp/flight_state_v2.hpp"
#include <gtest/gtest.h>
using namespace hope_planner_cpp;
namespace {
ShotSettings enabled_settings() {
  ShotSettings s;
#ifdef HOPE_REARM_CONTINUATION
  s.automatic_rearm = true;
#endif
  return s;
}
constexpr std::int64_t epoch=1800000000000000000LL;
BallSample sample(double t,const ModelState &s) { BallSample b;b.source_time_ns=epoch+std::llround(t*1e9);b.source_time_s=t;b.position=s.position;b.frame_id="table";b.calibration_id="survey";b.ball_id="ball";b.clock_domain="simulation";b.clock_mapping_id="mapped";return b; }
TEST(ContinuationBefore,E01_RejectThenGapAndOrdinaryBallB) {
 for(int hz:{150,300}) { FlightPipeline p({},enabled_settings());ModelState a;a.position=Vec3(2.5,-.7,.4);a.velocity=Vec3(-5,0,-.5);
 for(int i=0;i<hz/12;++i) {p.observe(sample(i*1./hz,a));integrate_ball(a,1./hz,{});} ASSERT_TRUE(p.shot().active());auto old=p.shot().generation();
 a.position.z()=-.2;p.observe(sample(.1,a));ASSERT_FALSE(p.shot().active());ASSERT_EQ(p.shot().reason(),"observed_under_table");
 ModelState b;b.position=Vec3(2.5,-.7,.4);b.velocity=Vec3(-5,0,-.5);
 for(int i=0;i<hz/10;++i) {p.observe(sample(1+i*1./hz,b));integrate_ball(b,1./hz,{});} EXPECT_TRUE(p.shot().active())<<p.shot().reason();EXPECT_GT(p.shot().generation(),old);
 }
}
TEST(ContinuationBefore,E02_QuaternionOnlyMagnusAcquisition) {
 for(int hz:{150,300})for(bool producer:{false,true}) { FlightPipeline p({},enabled_settings());ModelState a;a.position=Vec3(2.6,-.7,1.5);a.velocity=Vec3(-30,0,0);a.spin.omega=Vec3(0,0,100);a.spin.mode=EffectiveSpinMode::MEASURED;
 for(int i=0;i<=int(.085*hz);++i) {double t=i*1./hz;auto b=sample(t,a);b.orientation_valid=b.orientation_authoritative=true;b.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(100*t,Vec3::UnitZ()));b.producer_omega_valid=producer;b.producer_omega_rad_s=Vec3(0,0,100);p.observe(b);integrate_ball(a,1./hz,{});}
 EXPECT_TRUE(p.shot().active())<<hz<<":"<<producer<<":"<<p.shot().reason();if(p.shot().active()) {EXPECT_TRUE(p.shot().spin().usable());}
 }
}
TEST(ContinuationBefore,NegativeControl_HeldNeverAcquires) {FlightPipeline p({},enabled_settings());ModelState a;a.position=Vec3(2,-.7,.4);for(int i=0;i<100;++i)p.observe(sample(i*.006,a));EXPECT_FALSE(p.shot().active());}
TEST(ContinuationBefore,Control_QuaternionStreamMeetsUnchangedQualityThresholds) {for(int hz:{150,300}) {TrustedSpin spin;ModelState a;for(int i=0;i<=int(.085*hz);++i) {auto b=sample(i*1./hz,a);b.orientation_valid=b.orientation_authoritative=true;b.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(100*i*1./hz,Vec3::UnitZ()));spin.observe(b);}ASSERT_TRUE(spin.value().usable());EXPECT_NEAR(spin.value().omega->z(),100,1e-8);}}

}
