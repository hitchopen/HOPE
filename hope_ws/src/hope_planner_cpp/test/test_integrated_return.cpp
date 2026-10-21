#include "hope_planner_cpp/integrated_command_lifecycle.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include "a3_pingpong/pp_planner_lifecycle.hpp"
#include "a3_pingpong/pp_obs_builder.hpp"
#include <gtest/gtest.h>

using namespace hope_planner_cpp;
namespace {
constexpr std::int64_t epoch_ns=1800000000000000000LL;
ContactPlanningInput contact() {
  ContactPlanningInput i;
  i.token.generation=1;i.token.context_revision=1;i.token.solve_sequence=1;
  i.token.source_ns=epoch_ns;i.source.source_time_ns=epoch_ns;
  i.arrival_ns=epoch_ns+400000000;i.expires_ns=epoch_ns+30000000;
  i.contact.pre.position=Vec3(.15,-.7,.35);i.contact.pre.velocity=Vec3(-3,0,1);
  i.observation_count=20;i.observation_span_s=.1;i.model_config_id="test";
  return i;
}
ForecastPhysicsEvent event(const char *kind,double time,double x,double z=.02) {
  ForecastPhysicsEvent e;e.kind=kind;e.dt_s=time;e.pre<<x,-.7,z,-3,0,1;
  e.post=e.pre;e.covariance.setZero();e.quality=LinearizationQuality::Valid;
  e.time_std_s=.0001;return e;
}
struct Incoming {
  FlightPrediction f;
  ShotSnapshot s;
  Incoming() {
    f.physical_available=true;f.belief.initialized=true;f.belief.mean<<1,-.7,.4,-3,0,-1;
    f.belief.observation_count=20;f.belief.last_measurement_time_s=.1;
    f.token.generation=s.generation=1;s.direction=-1;s.boundary_ns=epoch_ns-100000000;
    f.reference_ns=f.token.source_ns=epoch_ns;f.expires_ns=epoch_ns+30000000;
    f.plane.calibrated=true;f.plane.frame_id="world";f.plane.registration_id="test";
    BallSample source;source.source_time_ns=epoch_ns;source.frame_id="world";
    source.calibration_id="test";source.clock_mapping_id="test";source.ball_id="Ball";
    s.observations.push_back(source);
    f.nominal.events={event("table_bounce",.1,.6),event("receiving_plane",.3,.15,.3)};
  }
  ReturnEligibility select() {return select_physical_return(f,s,{},epoch_ns);}
};
TEST(ReturnEligibility,I17_I18_I19_UnknownIncompleteAndEarlyPrediction) {
  Incoming i;
  EXPECT_TRUE(i.select().eligible);
  EXPECT_FALSE(i.f.target.valid); // Deliberately not the public game-route target.
  i.s.continuity=HistoryContinuity::kIncomplete;
  ASSERT_TRUE(i.select().eligible);
  EXPECT_EQ(i.select().input->history,HistoryContinuity::kIncomplete);
  EXPECT_FALSE(i.select().input->bounce_observed);
  i.f.nominal.events.erase(i.f.nominal.events.begin());
  ShotEvent bounce;bounce.generation=1;bounce.status=EventStatus::CONFIRMED;
  bounce.kind=PhysicalEventKind::TABLE;bounce.event_time_ns=epoch_ns-10000000;
  bounce.confirmed_at_ns=epoch_ns;bounce.pre.position=Vec3(.6,-.7,.02);
  bounce.pre.velocity=Vec3(-3,0,-1);i.s.events.push_back(bounce);
  ASSERT_TRUE(i.select().eligible);EXPECT_TRUE(i.select().input->bounce_observed);
  bounce.status=EventStatus::PREDICTED;i.s.events.push_back(bounce);
  EXPECT_TRUE(i.select().eligible); // Ledger prediction cannot double-count.
}
TEST(ReturnEligibility,I20_PhysicalOrderingAndDirectionNegatives) {
  Incoming before;std::swap(before.f.nominal.events[0],before.f.nominal.events[1]);
  before.f.nominal.events[0].dt_s=.05;EXPECT_FALSE(before.select().eligible);
  Incoming second;second.f.nominal.events.insert(second.f.nominal.events.begin()+1,event("table_bounce",.2,.3));
  EXPECT_EQ(second.select().reason,"SECOND_RECEIVER_BOUNCE");
  Incoming outgoing;outgoing.s.direction=1;EXPECT_FALSE(outgoing.select().eligible);
  Incoming stale;stale.f.expires_ns=epoch_ns-1;EXPECT_FALSE(stale.select().eligible);
  Incoming historical;historical.f.nominal.events.back().dt_s=0;EXPECT_FALSE(historical.select().eligible);
  Incoming edge;edge.f.nominal.events.insert(edge.f.nominal.events.begin(),event("edge_ambiguous",.01,.9));
  EXPECT_FALSE(edge.select().eligible);
}
TEST(ReturnEligibility,I21_I22_AdvisoryGameContextAndPhysicalSupport) {
  Incoming i;
  for (auto m : {MatchContext::UNKNOWN,MatchContext::SERVE,MatchContext::NORMAL_RALLY}) {
    i.s.match=m;EXPECT_TRUE(i.select().eligible);
  }
  i.f.nominal.events.back().quality=LinearizationQuality::Unavailable;
  EXPECT_FALSE(i.select().eligible);
  i.f.nominal.events.back().quality=LinearizationQuality::Valid;
  i.s.observations.back().clock_mapping_valid=false;EXPECT_FALSE(i.select().eligible);
  i.s.observations.back().clock_mapping_valid=true;
  i.f.nominal.events[0].time_std_s=.1;EXPECT_FALSE(i.select().eligible);
}
TEST(ReturnEligibility,SignedSimulationEpochAndRegistrationMismatch) {
  Incoming i;i.f.reference_ns=i.f.token.source_ns=-1000000000LL;
  i.f.expires_ns=-970000000LL;i.s.boundary_ns=-1100000000LL;
  i.s.observations.back().source_time_ns=-1000000000LL;
  auto result=select_physical_return(i.f,i.s,{},-1000000000LL);
  ASSERT_TRUE(result.eligible);EXPECT_EQ(result.input->arrival_ns,-700000000LL);
  i.s.observations.back().calibration_id="another_registration";
  EXPECT_FALSE(select_physical_return(i.f,i.s,{},-1000000000LL).eligible);
}
TEST(PaddleImpact,P03_IndependentAnalyticClosureAndApproach) {
  auto p=reconstruct_paddle_impact(Vec3(-3,2,1),Vec3(1,0,0),Vec3::UnitX(),.5);
  ASSERT_TRUE(p.valid);EXPECT_NEAR(p.outgoing.x(),3,1e-12);
  EXPECT_EQ(p.outgoing.y(),2);EXPECT_EQ(p.outgoing.z(),1);
  EXPECT_FALSE(reconstruct_paddle_impact(Vec3(3,0,0),Vec3::Zero(),Vec3::UnitX(),.5).valid);
  EXPECT_FALSE(reconstruct_paddle_impact(Vec3(-3,0,0),Vec3::Zero(),2*Vec3::UnitX(),.5).valid);
}
TEST(ValidatedReturn,P01_P03_P05_P08_P09_ActualActionFirstLandingAndDuration) {
  auto i=contact();ValidatedReturnPlanner planner({});auto r=planner.solve(i);
  ASSERT_TRUE(r.command.valid)<<r.reason;
  EXPECT_TRUE(r.solve_ok && r.impact_ok && r.return_ok);
  EXPECT_LT(r.solver_residual_m,1e-5);EXPECT_LT(r.impact_residual_mps,1e-8);
  EXPECT_NEAR(r.first_landing.z(),.02,1e-12);
  EXPECT_NEAR(r.first_landing.x(),2.055,.001);
  EXPECT_NEAR(r.first_landing.y(),-.7625,.001);
  EXPECT_NEAR(r.landing_time_s,r.selected_duration_s,.0005);
  EXPECT_FALSE(r.reconstructed.spin.omega.has_value());
  EXPECT_GT(r.net_clearance_m,0);
  r.command.velocity.x()+=.1;EXPECT_FALSE(planner.validate_action(i,r));EXPECT_FALSE(r.command.valid);
}
TEST(ValidatedReturn,P02_FiniteUnconvergedIsRejected) {
  ValidatedReturnSettings settings;settings.max_iterations=1;
  settings.endpoint_tolerance_m=1e-12;settings.durations_s={.5};
  ValidatedReturnPlanner p({},settings);auto r=p.solve(contact());
  EXPECT_FALSE(r.command.valid);EXPECT_EQ(r.reason,"INVERSE_NONCONVERGENCE");
}
TEST(ValidatedReturn,P04_AnalyticNoDragSurfaceRadiusAndHeightOnce) {
  ProbabilisticConfig config;config.physics.drag_k=0;config.physics.magnus_k=0;
  ValidatedReturnPlanner planner(config);auto i=contact();auto r=planner.solve(i);
  ASSERT_TRUE(r.command.valid)<<r.reason;
  const double t=r.selected_duration_s;
  Vec3 analytic=i.contact.pre.position+r.reconstructed.velocity*t+.5*config.physics.gravity*t*t;
  EXPECT_NEAR(analytic.x(),2.055,1e-8);EXPECT_NEAR(analytic.y(),-.7625,1e-8);
  EXPECT_NEAR(analytic.z(),.02,1e-8);
  auto wire=Schema2Packer::pack(&r.command,1,(epoch_ns+400000000)*1e-9,.76,epoch_ns,{1,1,1},20,.1);
  EXPECT_NEAR(wire.values[5],i.contact.pre.position.z()+.76,1e-12);
  EXPECT_NEAR(wire.values[3],i.contact.pre.position.x(),1e-12);
}
TEST(ValidatedReturn,P06_P07_NetAndGeometryFaultsReject) {
  auto i=contact();i.contact.pre.position.z()=.021;
  ValidatedReturnSettings s;s.durations_s={.1};
  auto r=ValidatedReturnPlanner({},s).solve(i);EXPECT_FALSE(r.command.valid);
  i.contact.pre.position.z()=-.1;EXPECT_FALSE(ValidatedReturnPlanner({}).solve(i).command.valid);
  s.landing_surface.z()=.02;EXPECT_THROW(ValidatedReturnPlanner({},s),std::invalid_argument);
}
TEST(ValidatedReturn,P08_RetryRecordsActualSelectedDuration) {
  ValidatedReturnSettings settings;settings.durations_s={.1,.5};
  auto i=contact();i.contact.pre.position.z()=.03;
  auto result=ValidatedReturnPlanner({},settings).solve(i);
  ASSERT_TRUE(result.command.valid)<<result.reason;
  EXPECT_EQ(result.attempts,2);EXPECT_EQ(result.selected_duration_s,.5);
  EXPECT_NEAR(result.landing_time_s,.5,.0005);
}
TEST(IntegratedLifecycle,L01_L02_L03_SemanticIdentityAndFreshness) {
  auto i=contact();auto r=ValidatedReturnPlanner({}).solve(i);ASSERT_TRUE(r.command.valid);
  IntegratedCommandLifecycle life;
  auto first=life.admit(i,r,1,epoch_ns);ASSERT_TRUE(first);EXPECT_EQ(first->flight_id,1u);
  EXPECT_FALSE(life.admit(i,r,1,epoch_ns));
  i.source.source_time_ns+=1000000;i.expires_ns+=1000000;
  auto revised=life.admit(i,r,1,epoch_ns+1000000);ASSERT_TRUE(revised);
  EXPECT_EQ(revised->flight_id,1u);EXPECT_EQ(revised->revision_id,2u);
  i.token.generation=2;i.source.source_time_ns+=1000000;
  auto next=life.admit(i,r,2,epoch_ns+2000000);ASSERT_TRUE(next);EXPECT_EQ(next->flight_id,2u);
  EXPECT_FALSE(life.admit(i,r,3,epoch_ns+2000000));
  EXPECT_FALSE(life.admit(i,r,2,epoch_ns+50000000));
}
TEST(IntegratedLifecycle,L05_L06_CommitAndExactIntegerOverflow) {
  auto i=contact();auto r=ValidatedReturnPlanner({}).solve(i);ASSERT_TRUE(r.command.valid);
  i.arrival_ns=epoch_ns+100000000;
  IntegratedCommandLifecycle life;ASSERT_TRUE(life.admit(i,r,1,epoch_ns));EXPECT_TRUE(life.committed());
  EXPECT_FALSE(life.refresh_idle(epoch_ns+100000000));
  EXPECT_TRUE(life.refresh_idle(epoch_ns+1100000001));
  ++i.source.source_time_ns;EXPECT_FALSE(life.admit(i,r,1,epoch_ns));
  IntegratedCommandLifecycle overflow(IntegratedCommandLifecycle::max_wire_integer);
  EXPECT_THROW(overflow.admit(i,r,1,epoch_ns),std::overflow_error);
}
TEST(RunnerRoundTrip,W01_W02_W03_W05_ActualSchema2Decoder) {
  auto i=contact();auto r=ValidatedReturnPlanner({}).solve(i);ASSERT_TRUE(r.command.valid);
  // Real Unix epoch, decoded by the unchanged receiver (no fixture decoder).
  const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  auto p=Schema2Packer::pack(&r.command,-1,(now+400000000)*1e-9,.76,now,{1,1,1},20,.1);
  a3_pingpong::PpRacketTargetInput receiver(2);
  std::vector<double> wire(p.values.begin(),p.values.end());receiver.SetFromFlat(wire);
  auto decoded=receiver.Latest();ASSERT_TRUE(decoded.has_valid);
  EXPECT_NEAR(decoded.cmd.pos_w.z(),r.command.position.z()+.76,1e-12);
  EXPECT_NEAR((decoded.cmd.vel_w-r.command.velocity).norm(),0,1e-12);
  EXPECT_NEAR((decoded.cmd.vel_w.normalized()-r.command.normal).norm(),0,1e-12);
  EXPECT_EQ(decoded.cmd.swing_sign,-1);EXPECT_EQ(decoded.cmd.flight_id,1u);
  a3_pingpong::PpRobotState robot;
  robot.base_pos_w=Vec3(-.43,-.7,.76);robot.base_quat_w<<1,0,0,0;
  robot.base_ang_vel_b.setZero();robot.q=Eigen::VectorXd::Zero(31);robot.qd=robot.q;
  a3_pingpong::PpRacketTarget target;
  target.pos_w=decoded.cmd.pos_w;target.vel_w=decoded.cmd.vel_w;
  target.time_to_strike=decoded.control_time_to_strike_s;target.swing_sign=-1;
  target.base_target_xy=robot.base_pos_w.head<2>();
  const auto obs=a3_pingpong::build_obs_110(robot,target,robot.q,robot.q);
  ASSERT_EQ(obs.size(),110);EXPECT_NEAR(obs[103],.58,1e-12);
  EXPECT_NEAR(obs[105],i.contact.pre.position.z(),1e-12);
  EXPECT_NEAR((obs.segment<3>(106)-r.command.velocity).norm(),0,1e-12);
  EXPECT_NEAR(obs[109],decoded.control_time_to_strike_s,1e-12);
  receiver.SetFromFlat(wire);EXPECT_EQ(receiver.Latest().seq,decoded.seq);
  auto invalid=Schema2Packer::pack(nullptr,1,0,.76,now+1,{2,1,1},0,0);
  receiver.SetFromFlat(std::vector<double>(invalid.values.begin(),invalid.values.end()));
  EXPECT_TRUE(receiver.Latest().invalid_after);
  wire[14]=3;wire[6]=std::numeric_limits<double>::quiet_NaN();receiver.SetFromFlat(wire);
  EXPECT_EQ(receiver.Latest().cmd.flight_id,1u);
  // Counter rollback in a fresh producer is ignored by an existing receiver.
  auto restart=Schema2Packer::pack(&r.command,1,(now+400000000)*1e-9,.76,now,{1,1,1},20,.1);
  receiver.SetFromFlat(std::vector<double>(restart.values.begin(),restart.values.end()));
  EXPECT_TRUE(receiver.Latest().invalid_after);
  a3_pingpong::PpRacketTargetInput restarted_receiver(2);
  restarted_receiver.SetFromFlat(std::vector<double>(restart.values.begin(),restart.values.end()));
  EXPECT_TRUE(restarted_receiver.Latest().has_valid);
  using namespace a3_pingpong;
  EXPECT_EQ(planner_schema22_cold_receipt_decision(false,.001,.05,true),
            PlannerSchema22ColdReceiptDecision::kRejectInvalidInitialReceipt);
  EXPECT_EQ(planner_schema22_cold_receipt_decision(true,.001,.05,true),
            PlannerSchema22ColdReceiptDecision::kAcceptRetainedFlight);
}
TEST(IntegratedFlight,I12_I17_I19_ActualCkfWithoutContextPublisher) {
  ShotSettings settings;settings.game_legality_advisory=true;
  ProbabilisticConfig config;FlightPipeline pipeline(config,settings);
  HitPlaneSnapshot plane;plane.calibrated=true;plane.x_hit_m=.15;plane.frame_id="world";plane.registration_id="test";
  ModelState ball;ball.position=Vec3(1.8,-.7,.4);ball.velocity=Vec3(-5,0,-.5);
  bool early=false,late=false;
  EventGeometry geometry;geometry.table=config.table;
  for (int k=0;k<120;++k) {
    const auto ns=epoch_ns+std::llround(k/300.*1e9);
    BallSample sample;sample.position=ball.position;sample.source_time_ns=ns;sample.source_time_s=ns*1e-9;
    sample.frame_id="world";sample.calibration_id="test";sample.clock_domain="ros_system";
    sample.clock_mapping_id="test";sample.ball_id="Ball";
    pipeline.observe(sample);
    auto forecast=pipeline.forecast(plane,ns);
    auto e=select_physical_return(forecast,pipeline.shot().export_snapshot(),config,ns);
    if (e.eligible) {
      EXPECT_NE(e.input->history,HistoryContinuity::kComplete);
      if (e.input->bounce_observed) late=true;else early=true;
    }
    advance_ball(ball,1/300.,config.physics,geometry);
  }
  EXPECT_TRUE(early);EXPECT_TRUE(late);
}
TEST(IntegratedFlight,I21_GameIllegalHistoryDoesNotRetirePhysicalTrack) {
  ShotSettings settings;settings.game_legality_advisory=true;
  ShotContext shot({},settings);
  shot.start(0,-1,BoundaryEvidence::ACQUISITION,HistoryContinuity::kUnknown);
  ModelState pre;pre.position=Vec3(2,-.7,.02);pre.velocity=Vec3(-3,0,-1);
  ASSERT_TRUE(shot.confirm(PhysicalEventKind::TABLE,10000000,20000000,pre,"sensor"));
  ASSERT_TRUE(shot.confirm(PhysicalEventKind::TABLE,200000000,210000000,pre,"sensor"));
  EXPECT_TRUE(shot.active());
  EXPECT_EQ(shot.continuity(),HistoryContinuity::kUnknown);
  Incoming i;i.s.events=shot.events();
  EXPECT_TRUE(i.select().eligible); // Two old opponent-side contacts are advisory.
}
} // namespace
