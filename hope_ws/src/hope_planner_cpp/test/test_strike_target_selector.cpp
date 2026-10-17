#include "hope_planner_cpp/strike_target_selector.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"
#include <gtest/gtest.h>
using namespace hope_planner_cpp;
namespace {
constexpr std::int64_t t0=1780000000000000000LL;
StrikeTargetSelectorConfig cfg() {
  StrikeTargetSelectorConfig c;c.registration="fixture";c.synthetic=true;c.context_authority="fixture";return c;
}
hope_msgs::msg::BallForecast event(std::int64_t ns,int revision) {
  hope_msgs::msg::BallForecast m;m.schema="ball-forecast-v1";m.kind=m.FORECAST;
  m.publisher_instance_id="fixture";m.event_sequence=m.identity.revision_id=revision;
  m.identity.trajectory_epoch=m.identity.rally_id=1;m.identity.source_time_ns=ns;
  m.identity.frame_id="table";m.identity.return_region_config_id="fixture";
  m.hit_plane_generation=1;m.x_hit_calibrated=m.known_test_plane=true;
  m.frame_registration_id="fixture";m.source_clock_domain="ros_system";m.source_clock_registered=true;
  m.last_measurement_time_ns=ns;m.valid_until_source_ns=ns+30000000;
  m.forecast_available=m.crossing_exists=m.event_future_at_publish=m.physically_usable=m.height_above_zero=true;
  m.hit_plane.physically_usable=m.hit_plane.absolute_time_valid=true;
  m.hit_plane.dt_ns=400000000;m.hit_plane.absolute_time_ns=ns+m.hit_plane.dt_ns;
  m.hit_plane.pre_state={0,-.7,.4,-4,.1,.5};m.rally_context="normal_return";
  m.physics_event_sequence={"table_bounce","hit_plane"};m.physics_event_dt_ns={100000000,400000000};return m;
}
void feed(StrikeTargetSelector& s,int end=29,bool forecasts=true) {
  for(int i=0;i<=end;++i) {
    BallSample b;b.source_time_ns=t0+i*5000000LL;b.position=Vec3(1.8-.025*i,-.7,.4);
    s.observe(b,"table",b.source_time_ns,b.source_time_ns);
    if(forecasts)s.forecast(event(b.source_time_ns,i+1),b.source_time_ns);
  }
}
}
TEST(StrikeTargetSelector, NetDropoutStillCommitsFromTimer) {
  StrikeTargetSelector s(cfg());feed(s,18);
  auto out=s.poll(t0+160000000,t0+160000000);
  ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);EXPECT_FALSE(s.poll(t0+170000000,t0+170000000));
}
TEST(StrikeTargetSelector, ExpiryRetainsUsefulAbsoluteCrossing) {
  StrikeTargetSelector s(cfg());feed(s);
  auto m=event(t0+145000000,100);m.kind=m.INVALIDATION;m.reason="source_age_expired";s.forecast(m,m.identity.source_time_ns);
  auto out=s.poll(t0+180000000,t0+180000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
}
TEST(StrikeTargetSelector, MissingHistoryDoesNotSuppressGeometry) {
  StrikeTargetSelector s(cfg());feed(s);auto out=s.poll(t0+160000000,t0+160000000);
  ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
}
TEST(StrikeTargetSelector, MissingForecastUsesOriginalBatchCalculation) {
  StrikeTargetSelector s(cfg());auto metadata=event(t0,1);metadata.crossing_exists=false;
  s.forecast(metadata,t0);feed(s,29,false);
  auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
}
TEST(StrikeTargetSelector, FixedNanosecondCutoffAndGrace) {
  StrikeTargetSelector s(cfg());feed(s);
  EXPECT_FALSE(s.poll(t0+145999999,t0+145999999));
  auto out=s.poll(t0+146000000,t0+146000000);ASSERT_TRUE(out);
  EXPECT_EQ(out->net_ns,t0+86000000);EXPECT_EQ(out->commit_ns,t0+136000000);
  EXPECT_EQ(out->source_ns,t0+135000000);EXPECT_TRUE(out->strike.valid);
}
TEST(StrikeTargetSelector, PostCutoffRevisionsCannotEvictDurableCrossing) {
  StrikeTargetSelector s(cfg());feed(s,27);
  for(int i=0;i<100;++i) s.forecast(event(t0+136000001+i,100+i),t0+145000000);
  auto out=s.poll(t0+146000000,t0+146000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
  EXPECT_EQ(out->source_ns,t0+135000000);EXPECT_LE(s.cached(),16u);
}
TEST(StrikeTargetSelector, UnknownAndIllegalAreAdvisory) {
  for(bool illegal:{false,true}) {
    StrikeTargetSelector s(cfg());
    hope_msgs::msg::HybridTrajectoryContext c;c.kind=c.BEGIN;c.frame_id="table";c.evidence_source_id="fixture";
    c.rally_id=1;c.source_time_ns=t0;c.history_complete=illegal;s.context(c,t0);
    if(illegal) {c.kind=c.VERIFIED_CONTACT;c.source_time_ns++;c.contact_side="receiver";c.contact_position.x=9;s.context(c,c.source_time_ns);}
    feed(s);auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
    EXPECT_EQ(out->eligibility_status,illegal?"known_illegal":"unknown");
  }
}
TEST(StrikeTargetSelector, LateAcquisitionHasIndependentIdentity) {
  StrikeTargetSelector s(cfg());
  for(int i=0;i<6;++i) {auto ns=t0+i*5000000LL;BallSample b;b.source_time_ns=ns;b.position=Vec3(1.25-.025*i,-.7,.4);
    s.observe(b,"table",ns,ns);s.forecast(event(ns,i+1),ns);}
  auto out=s.poll(t0+50000000,t0+50000000);ASSERT_TRUE(out);EXPECT_TRUE(out->late_acquisition);EXPECT_TRUE(out->strike.valid);
  EXPECT_LT(out->net_ns,t0);EXPECT_EQ(out->physical_flight_id,1u);
}
TEST(StrikeTargetSelector, HardFaultStillProducesOneInvalidDecision) {
  StrikeTargetSelector s(cfg());feed(s);BallSample b;b.source_time_ns=t0+146000000;b.position=Vec3(1,-.7,.4);
  s.observe(b,"wrong",b.source_time_ns,b.source_time_ns);auto out=s.poll(t0+160000000,t0+160000000);
  ASSERT_TRUE(out);EXPECT_FALSE(out->strike.valid);EXPECT_EQ(out->reason,"invalid_pose");
  EXPECT_FALSE(s.poll(t0+170000000,t0+170000000));
}
TEST(StrikeTargetSelector, BatchFallbackMatchesOriginalOnIdenticalCausalWindow) {
  auto c=cfg();c.frozen_plane=SelectorFrozenPlane{0,1,0,0,true};StrikeTargetSelector s(c);feed(s,29,false);
  auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);ASSERT_TRUE(out->strike.valid);EXPECT_EQ(out->origin,"batch_fit");
  BatchPhysicsEstimator legacy(c.physics,c.estimator);legacy.begin_flight();
  for(int i=0;i<=27;++i) {BallSample b;b.source_time_s=static_cast<double>(i*5000000LL)*1e-9;b.position=Vec3(1.8-.025*i,-.7,.4);legacy.push(b);}
  auto expected=TrajectoryPredictor(c.physics,c.planner,TableParams{}).predict_with_spin(legacy.estimate(),0,Vec3::Zero(),SpinPhysicsMode::kVenueGripBounce);
  ASSERT_TRUE(expected.valid);EXPECT_TRUE(out->strike.ball_position.isApprox(expected.ball_position,1e-13));
  EXPECT_TRUE(out->strike.ball_velocity.isApprox(expected.ball_velocity,1e-13));EXPECT_EQ(out->strike.predicted_bounces,expected.predicted_bounces);
  EXPECT_EQ(out->absolute_hit_ns,t0+std::llround(expected.strike_source_time_s*1e9));
}
TEST(StrikeTargetSelector, FullStateUsesSeparateCounterAndLegacyPhysics) {
  StrikeTargetSelector s(cfg());auto compact=event(t0,100);compact.crossing_exists=false;s.forecast(compact,t0);
  feed(s,18,false);auto full=event(t0+90000000,1);full.crossing_exists=false;
  hope_msgs::msg::BallForecastWaypoint w;w.absolute_time_valid=w.physical_continuation=true;w.absolute_time_ns=full.identity.source_time_ns;
  w.state={1.35,-.7,.4,-5,0,1};full.waypoints={w};ASSERT_TRUE(s.full_forecast(full,full.identity.source_time_ns));
  auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);EXPECT_EQ(out->origin,"ckf_state_rollout");
  EXPECT_EQ(out->source_ns,full.identity.source_time_ns);
}
TEST(StrikeTargetSelector, SparseDropoutAndExpiredPastDeadlineAreExplicit) {
  auto c=cfg();c.frozen_plane=SelectorFrozenPlane{0,1,0,0,true};StrikeTargetSelector s(c);
  // Short observed segment straddles the net, before batch readiness.
  for(int i=0;i<7;++i) {auto ns=t0+i*5000000LL;BallSample b;b.source_time_ns=ns;b.position=Vec3(1.5-.025*i,-.7,.4);s.observe(b,"table",ns,ns);}
  auto out=s.poll(t0+100000000,t0+100000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);EXPECT_EQ(out->origin,"sparse_fit");
  EXPECT_EQ(out->eligibility_status,"unknown");EXPECT_FALSE(s.poll(t0+110000000,t0+110000000));
  StrikeTargetSelector late(c);feed(late,29,false);auto invalid=late.poll(t0+3000000000LL,t0+3000000000LL);
  ASSERT_TRUE(invalid);EXPECT_FALSE(invalid->strike.valid);EXPECT_EQ(invalid->reason,"legacy_past_or_below_ground_crossing");
}
TEST(StrikeTargetSelector, NewestBadCrossingRetainsPriorAndShortLeadIsAllowed) {
  StrikeTargetSelector s(cfg());feed(s,27);auto bad=event(t0+136000000,100);bad.hit_plane.pre_state[2]=NAN;
  EXPECT_FALSE(s.forecast(bad,t0+140000000));
  auto out=s.poll(t0+500000000,t0+500000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
  EXPECT_EQ(out->source_ns,t0+135000000);EXPECT_EQ(out->absolute_hit_ns,t0+535000000);
}
TEST(StrikeTargetSelector, ExactCutoffAndAdvisoryThirtyMillisecondBoundary) {
  for(int offset:{0,1}) {
    StrikeTargetSelector s(cfg());feed(s,27);auto f=event(t0+136000000+offset,100);
    ASSERT_TRUE(s.forecast(f,t0+140000000));auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);
    EXPECT_EQ(out->source_ns,t0+(offset?135000000:136000000));EXPECT_TRUE(out->strike.valid);
  }
  for(int offset:{0,1}) {
    StrikeTargetSelector s(cfg());feed(s,27);auto out=s.poll(t0+165000000+offset,t0+165000000+offset);ASSERT_TRUE(out);
    EXPECT_TRUE(out->strike.valid);EXPECT_EQ(out->origin,offset?"retained_ckf":"nominal_ckf");
  }
}
TEST(StrikeTargetSelector, TwoSparseSamplesAndZeroOrOneCausalSample) {
  for(int count:{0,1,2}) {
    auto c=cfg();c.frozen_plane=SelectorFrozenPlane{0,1,0,0,true};StrikeTargetSelector s(c);
    const double x=count==0?1.095:count==1?1.12:1.145;
    for(int i=0;i<6;++i) {BallSample b;b.source_time_ns=t0+i*5000000LL;b.position=Vec3(x-.025*i,-.7,.4);s.observe(b,"table",b.source_time_ns,b.source_time_ns);}
    auto out=s.poll(t0+60000000,t0+60000000);ASSERT_TRUE(out);EXPECT_EQ(out->strike.valid,count==2)<<out->reason;
    if(count==2) {EXPECT_EQ(out->origin,"sparse_fit");EXPECT_EQ(out->sample_count,2u);}
    else EXPECT_NE(out->reason,"no_usable_crossing");
  }
}
TEST(StrikeTargetSelector, PositiveNanosecondDeadlineAndOldPublishFlags) {
  auto f=event(t0,1);f.event_future_at_publish=false;f.forecast_available=false;
  f.hit_plane.linearization_quality="uncalibrated";
  EXPECT_TRUE(StrikeTargetSelector::extract(f,cfg(),f.hit_plane.absolute_time_ns-1).strike.valid);
  EXPECT_FALSE(StrikeTargetSelector::extract(f,cfg(),f.hit_plane.absolute_time_ns).strike.valid);
  EXPECT_FALSE(StrikeTargetSelector::extract(f,cfg(),f.hit_plane.absolute_time_ns+1).strike.valid);
  f.hit_plane.absolute_time_ns++;EXPECT_FALSE(StrikeTargetSelector::extract(f,cfg(),t0).strike.valid);
}
TEST(StrikeTargetSelector, WrongPlaneAndSessionCannotUseFallbackHistory) {
  StrikeTargetSelector s(cfg());feed(s);auto f=event(t0+146000000,100);f.hit_plane_generation=2;
  EXPECT_FALSE(s.forecast(f,f.identity.source_time_ns));auto out=s.poll(t0+160000000,t0+160000000);
  ASSERT_TRUE(out);EXPECT_FALSE(out->strike.valid);EXPECT_TRUE(s.halted());
}
TEST(StrikeTargetSelector, FullStateAfterCutoffCannotEvictCausalState) {
  StrikeTargetSelector s(cfg());feed(s,18,false);
  auto f=event(t0+90000000,1);f.crossing_exists=false;hope_msgs::msg::BallForecastWaypoint w;
  w.absolute_time_valid=w.physical_continuation=true;w.absolute_time_ns=f.identity.source_time_ns;w.state={1.35,-.7,.4,-5,0,1};f.waypoints={w};ASSERT_TRUE(s.full_forecast(f,f.identity.source_time_ns));
  for(int i=0;i<100;++i) {f.event_sequence=f.identity.revision_id=2+i;f.identity.source_time_ns=t0+136000001+i;f.waypoints[0].absolute_time_ns=f.identity.source_time_ns;s.full_forecast(f,t0+145000000);}
  auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);EXPECT_EQ(out->origin,"ckf_state_rollout");EXPECT_EQ(out->source_ns,t0+90000000);
}
TEST(StrikeTargetSelector, LegacyLowHeightWarningDoesNotHidePositiveCrossing) {
  StrikeTargetSelector s(cfg());feed(s,27,false);auto f=event(t0+135000000,1);f.crossing_exists=false;
  hope_msgs::msg::BallForecastWaypoint w;w.absolute_time_valid=w.physical_continuation=true;
  w.absolute_time_ns=f.identity.source_time_ns;w.state={.2,-.7,.1,-4,0,-1};f.waypoints={w};ASSERT_TRUE(s.full_forecast(f,f.identity.source_time_ns));
  auto out=s.poll(t0+146000000,t0+146000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid)<<out->reason;
  EXPECT_GT(out->strike.ball_position.z(),0);EXPECT_LT(out->strike.ball_position.z(),.05);
  EXPECT_GT(out->absolute_hit_ns,t0+146000000);EXPECT_NE(out->quality.find("legacy_low_height"),std::string::npos);
}
TEST(StrikeTargetSelector, LateNetReceiptDoesNotRestartGrace) {
  StrikeTargetSelector s(cfg());
  for(int i=0;i<=18;++i) {const auto source=t0+i*5000000LL;const auto receipt=t0+200000000+i*1000;
    BallSample b;b.source_time_ns=source;b.position=Vec3(1.8-.025*i,-.7,.4);s.observe(b,"table",receipt,receipt);s.forecast(event(source,i+1),receipt);}
  auto out=s.poll(t0+200018000,t0+200018000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
  EXPECT_EQ(out->commit_ns,t0+136000000);
}
TEST(StrikeTargetSelector, MultipleAuthorizedResetAcknowledgmentsAreIdempotent) {
  StrikeTargetSelector s(cfg());
  for(auto ns:{t0-20000000,t0-10000000}) {hope_msgs::msg::HybridTrajectoryContext c;c.kind=c.RESET;c.frame_id="table";c.evidence_source_id="fixture";c.source_time_ns=ns;s.context(c,ns);}
  auto ack=event(t0-20000000,1);ack.kind=ack.INVALIDATION;ack.reason="explicit_reset";
  s.forecast(ack,t0);feed(s,27);ack.identity.source_time_ns=t0-10000000;ack.event_sequence=100;
  s.forecast(ack,t0+140000000);s.forecast(ack,t0+140000001);s.full_forecast(ack,t0+140000002);
  EXPECT_FALSE(s.halted());auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
  EXPECT_FALSE(s.poll(t0+170000000,t0+170000000));
}
TEST(StrikeTargetSelector, UnrequestedForecasterResetStillFencesFlight) {
  StrikeTargetSelector s(cfg());feed(s);auto ack=event(t0+145000000,100);ack.kind=ack.INVALIDATION;ack.reason="explicit_reset";
  s.forecast(ack,t0+145000000);auto out=s.poll(t0+160000000,t0+160000000);
  ASSERT_TRUE(out);EXPECT_TRUE(s.halted());EXPECT_FALSE(out->strike.valid);
}
TEST(StrikeTargetSelector, PausedRegisteredSimulationClockCannotCommitEarly) {
  auto c=cfg();c.clock="ros_sim";StrikeTargetSelector s(c);
  for(int i=0;i<=18;++i) {const auto ns=t0+i*5000000LL;BallSample b;b.source_time_ns=ns;b.position=Vec3(1.8-.025*i,-.7,.4);
    s.observe(b,"table",ns,ns);auto f=event(ns,i+1);f.source_clock_domain="ros_sim";s.forecast(f,ns);}
  EXPECT_FALSE(s.poll(t0+90000000,t0+500000000));
  auto out=s.poll(t0+146000000,t0+600000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
}
TEST(StrikeTargetSelector, AbsentPastContactProofIsUnknownNotKnownIllegal) {
  StrikeTargetSelector s(cfg());hope_msgs::msg::HybridTrajectoryContext c;c.kind=c.BEGIN;c.frame_id="table";c.evidence_source_id="fixture";
  c.source_time_ns=t0;c.rally_id=1;c.history_complete=true;s.context(c,t0);
  for(int i=0;i<=29;++i) {const auto ns=t0+i*5000000LL;BallSample b;b.source_time_ns=ns;b.position=Vec3(1.8-.025*i,-.7,.4);
    s.observe(b,"table",ns,ns);auto f=event(ns,i+1);f.physics_event_sequence={"hit_plane"};f.physics_event_dt_ns={400000000};s.forecast(f,ns);}
  auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);EXPECT_TRUE(out->strike.valid);
  EXPECT_EQ(out->eligibility_status,"unknown");EXPECT_EQ(out->strike.predicted_bounces,0);
}

TEST(StrikeTargetSelector, ClockFaultProducesExactlyOneInvalidDecision) {
  StrikeTargetSelector s(cfg());feed(s);auto f=event(t0+146000000,100);
  f.source_clock_domain="ros_sim";EXPECT_FALSE(s.forecast(f,t0+146000000));
  auto out=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(out);EXPECT_FALSE(out->strike.valid);
  EXPECT_FALSE(s.poll(t0+170000000,t0+170000000));
}
TEST(StrikeTargetSelector, HitDeadlineOverflowCannotBecomeTarget) {
  auto f=event(t0,1);f.identity.source_time_ns=INT64_MAX-10;
  f.hit_plane.dt_ns=20;f.hit_plane.absolute_time_ns=10;
  EXPECT_FALSE(StrikeTargetSelector::extract(f,cfg(),INT64_MAX-1).strike.valid);
}

TEST(StrikeTargetSelector, OutgoingOpponentReturnRearmsPhysicalIdentity) {
  auto c=cfg();c.frozen_plane=SelectorFrozenPlane{0,1,0,0,true};StrikeTargetSelector s(c);feed(s,29,false);
  auto first=s.poll(t0+160000000,t0+160000000);ASSERT_TRUE(first);EXPECT_EQ(first->physical_flight_id,1u);
  for(int i=0;i<35;++i) {auto ns=t0+170000000+i*5000000LL;BallSample b;b.source_time_ns=ns;
    b.position=Vec3(1.1+.03*i,-.7,.5);s.observe(b,"table",ns,ns);}
  for(int i=0;i<40;++i) {auto ns=t0+345000000+i*5000000LL;BallSample b;b.source_time_ns=ns;
    b.position=Vec3(2.12-.025*i,-.7,.5);s.observe(b,"table",ns,ns);}
  auto second=s.poll(t0+600000000,t0+600000000);ASSERT_TRUE(second);EXPECT_EQ(second->physical_flight_id,2u);
  EXPECT_FALSE(s.poll(t0+610000000,t0+610000000));
}

#include "hope_planner_cpp/hit_plane_manager.hpp"
namespace {
std::vector<double> registered_base(std::int64_t ns,unsigned sequence,double x) {
  // Synthetic schema-2 fixture: wxyz identity, measured/registered flags, IDs.
  return {2,1,double(sequence),double(ns/1000000000),double(ns%1000000000),
    x,-.7,.76,1,0,0,0,1,47,11,1};
}
void stable_base(HitPlaneManager& manager,std::int64_t start,unsigned first,double x) {
  for(unsigned i=0;i<12;++i) {
    const auto ns=start+i*5000000;
    ASSERT_TRUE(manager.observe(registered_base(ns,first+i,x+(i%2?.0002:-.0002)),ns,ns,true))<<manager.reason();
  }
}
}
TEST(StrikeTargetSelector, RegisteredSessionPlaneOffsetFreezeAndIdleRefresh) {
  for(double base_x:{-.58,-.50,-.68}) {
    HitPlaneManager manager({},"table","fixture",1);
    stable_base(manager,t0,1,base_x);
    ASSERT_TRUE(manager.freeze(t0+60000000,t0+60000000,1,true))<<manager.reason();
    const auto initial=manager.snapshot();
    EXPECT_NEAR(initial.x_hit_m,base_x+.58,1e-12);
    EXPECT_EQ(initial.generation,1u);EXPECT_EQ(initial.world_frame_id,1u);EXPECT_EQ(initial.calibration_id,11u);
    EXPECT_TRUE(initial.calibrated);EXPECT_FALSE(initial.test_plane);
    stable_base(manager,t0+65000000,13,base_x+.04);
    EXPECT_EQ(manager.snapshot().x_hit_m,initial.x_hit_m);
    EXPECT_FALSE(manager.freeze(t0+125000000,t0+125000000,1,false));
    EXPECT_EQ(manager.reason(),"refresh_forbidden_active_rally");
    EXPECT_EQ(manager.snapshot().generation,initial.generation);
    EXPECT_EQ(manager.snapshot().x_hit_m,initial.x_hit_m);
    // New settled session: old samples have left the bounded 0.5-s window.
    stable_base(manager,t0+650000000,25,base_x+.04);
    ASSERT_TRUE(manager.freeze(t0+710000000,t0+710000000,2,true))<<manager.reason();
    EXPECT_NEAR(manager.snapshot().x_hit_m,base_x+.04+.58,1e-12);
    EXPECT_EQ(manager.snapshot().generation,initial.generation+1);
    EXPECT_EQ(manager.snapshot().rally_id,2u);
  }
}
TEST(StrikeTargetSelector, PlaneCalibrationRejectsUnregisteredAndOutsideGeometry) {
  HitPlaneManager manager({},"table","fixture",1);
  auto wire=registered_base(t0,1,-.58);
  EXPECT_FALSE(manager.observe(wire,t0,t0,false));
  wire[15]=2;EXPECT_FALSE(manager.observe(wire,t0,t0,true));
  wire[15]=1;wire[13]=0;EXPECT_FALSE(manager.observe(wire,t0,t0,true));
  EXPECT_FALSE(manager.snapshot().calibrated);
  stable_base(manager,t0+10000000,2,-.27); // x_hit=+0.31: never clamp to range.
  EXPECT_FALSE(manager.freeze(t0+70000000,t0+70000000,1,true));
  EXPECT_EQ(manager.reason(),"plane_out_of_range");EXPECT_FALSE(manager.snapshot().calibrated);
}
