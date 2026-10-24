#include "hope_planner_cpp/flight_state_v2.hpp"
#include "hope_planner_cpp/return_eligibility.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"
#include "hope_planner_cpp/validated_return_planner.hpp"
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <set>

using namespace hope_planner_cpp;
namespace {
// New independently specified equivalents, not unavailable original reviewer traces.
BallSample observation(double t, const ModelState &state) {
  BallSample s;
  s.source_time_ns = 1800000000000000000LL + std::llround(t * 1e9);
  s.source_time_s = t;
  s.position = state.position;
  s.frame_id = "table";
  s.calibration_id = "synthetic-review";
  s.clock_domain = "simulation";
  s.clock_mapping_id = "synthetic-clock";
  s.ball_id = "ball";
  return s;
}
HitPlaneSnapshot plane() {
  HitPlaneSnapshot p;
  p.calibrated = true;
  p.x_hit_m = .15;
  p.frame_id = "table";
  p.registration_id = "synthetic-review";
  return p;
}
TEST(PR64Review, RV01_DragAcquisitionBothRatesDirectionsAndSpin) {
  for (int rate : {150, 300})
    for (double speed : {10., 12.})
      for (int direction : {-1, 1})
        for (bool known : {false, true}) for (bool noisy : {false,true}) {
          SCOPED_TRACE(::testing::Message() << rate << ":" << speed << ":" << direction << ":" << known);
          ProbabilisticConfig c;
          ShotSettings settings;
          settings.game_legality_advisory = true;
          FlightPipeline pipeline(c, settings);
          ModelState state;
          const double incoming_x=speed==10?2.5:2.6;
          state.position = Vec3(direction < 0 ? incoming_x : c.table.length-incoming_x,
                                -.7, speed==10?.32:.3);
          state.velocity = Vec3(direction * speed, 0, -.5);
          if (known) {
            state.spin.omega = Vec3(0, 20, 0);
            state.spin.mode = EffectiveSpinMode::MEASURED;
          }
          EventGeometry geometry;
          geometry.receiving_x = .15;
          double t = 0;
          bool acquired = false, physical = false, future_plane = false;
          std::uint64_t generation = 0;
          int true_bounces=0;
          for (int i = 0; i < rate / 2; ++i) {
            auto s = observation(t, state);
            if(noisy)s.position+=Vec3(.0001*std::sin(i*1.7),.0001*std::cos(i*.7),.0001*std::sin(i*.3));
            s.producer_omega_valid = known;
            if (known) s.producer_omega_rad_s = *state.spin.omega;
            pipeline.observe(s);
            if(acquired) { EXPECT_TRUE(pipeline.shot().active())<<pipeline.shot().reason(); }
            if (pipeline.shot().active()) {
              acquired = true;
              if (!generation) generation = pipeline.shot().generation();
              EXPECT_EQ(pipeline.shot().generation(), generation);
              auto f = pipeline.forecast(plane(), s.source_time_ns);
              physical |= f.physical_available;
              future_plane |= f.nominal.hit_plane.has_value();
            }
            // Deliberately vary source intervals without hiding a contact.
            const double dt = (i % 2 ? .95 : 1.05) / rate;
            auto step = advance_ball(state, dt, c.physics, geometry);
            for(const auto &e:step.events)if(e.kind==PhysicalEventKind::TABLE)++true_bounces;
            t += dt;
            if (step.stopped || (direction < 0 && state.position.x() < .15) ||
                (direction>0 && state.position.x()>c.table.length)) break;
          }
          EXPECT_TRUE(acquired) << pipeline.shot().reason();
          EXPECT_TRUE(physical);
          if (direction < 0) { EXPECT_TRUE(future_plane); }
          EXPECT_EQ(true_bounces,1);
        }
}
TEST(PR64Review, RV02_HeldImpossibleAccelerationAndProvenanceFaults) {
  for (int rate : {150, 300}) {
    for (bool held : {false, true}) {
      FlightPipeline p({});
      ModelState state;
      for (int i = 0; i < rate / 10; ++i) {
        const double t = double(i) / rate;
        state.position = Vec3(1 + (held ? 0 : 5*t), -.7, .5 + (held ? 0 : 50*t*t));
        p.observe(observation(t, state));
      }
      EXPECT_FALSE(p.shot().active());
    }
    FlightPipeline p({});
    p.start(1800000000000000000LL, -1, BoundaryEvidence::RESTART, HistoryContinuity::kUnknown);
    ModelState state; state.position = Vec3(2, -.7, .4);
    auto s = observation(0, state); p.observe(s);
    EXPECT_FALSE(p.observe(s)); // Repeated input cannot create evidence.
    s = observation(.01, state); s.frame_id = "wrong"; p.observe(s);
    EXPECT_FALSE(p.shot().active());
  }
}
TEST(PR64Review, RV01b_RecentLaunchAfterHeldCandidatePrefixAcquiresPromptly) {
  for(int rate:{150,300}) {
    FlightPipeline p({});ModelState state;state.position=Vec3(2.5,-.7,.32);
    for(int i=0;i<rate/2;++i)p.observe(observation(i*1./rate,state));
    EXPECT_FALSE(p.shot().active());
    state.velocity=Vec3(-10,0,-.5);
    for(int i=0;i<=std::ceil(.06*rate);++i) {
      p.observe(observation(.5+i*1./rate,state));
      integrate_ball(state,1./rate,{});
    }
    ASSERT_TRUE(p.shot().active())<<p.shot().reason();
    EXPECT_EQ(p.shot().continuity(),HistoryContinuity::kUnknown);
    EXPECT_GT(p.shot().samples().front().source_time_ns,observation(.49,{}).source_time_ns);
  }
}
TEST(PR64Review, RV03_OneFastBounceHasOneStableIdentity) {
  for (int rate : {150, 300})
    for (double speed : {10., 12., 20.}) {
      SCOPED_TRACE(::testing::Message() << rate << ":" << speed);
      ProbabilisticConfig c;
      ShotSettings settings; settings.game_legality_advisory = true;
      FlightPipeline p(c, settings);
      ModelState state;
      state.position = Vec3(2.5, -.7, .65);
      state.velocity = Vec3(-speed, 0, -5);
      auto probe = state;
      probe.position.x() = 0;
      while (probe.position.z() > c.physics.radius) integrate_ball(probe, .00001, c.physics);
      state.position.x() = .6 - probe.position.x();
      auto first = observation(0, state);
      // Isolate R02 from R01; automatic acquisition is tested separately.
      p.start(first.source_time_ns, -1, BoundaryEvidence::RESTART, HistoryContinuity::kUnknown);
      EventGeometry geometry; geometry.receiving_x = .15;
      std::set<std::uint64_t> ids;
      int true_bounces = 0, max_contacts = 0;
      for (int i = 0; i < rate / 3; ++i) {
        auto s = observation(double(i)/rate, state);
        p.observe(s);
        if (!p.shot().active()) break;
        auto f = p.forecast(plane(), s.source_time_ns);
        max_contacts = std::max(max_contacts, p.shot().observed_contacts());
        for (const auto &e : p.shot().events())
          if (e.kind == PhysicalEventKind::TABLE && e.status == EventStatus::CONFIRMED) ids.insert(e.id);
        const auto step = advance_ball(state, 1./rate, c.physics, geometry);
        for (const auto &e : step.events) if (e.kind == PhysicalEventKind::TABLE) ++true_bounces;
        if (state.position.x() < .15 || step.stopped) break;
      }
      ASSERT_EQ(true_bounces, 1);
      EXPECT_EQ(max_contacts, 1);
      EXPECT_EQ(ids.size(), 1u);
      EXPECT_TRUE(p.shot().active()) << p.shot().reason();
    }
}
BallSample angular(double t, double angle) {
  ModelState state;
  auto s = observation(t, state);
  s.orientation_valid = s.orientation_authoritative = true;
  s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitY()));
  return s;
}
TEST(PR64Review, RV03b_ReproducedOverlappingTwentyMetrePerSecondBounce) {
  for(int rate:{150,300}) {
    ProbabilisticConfig c;ShotSettings settings;settings.game_legality_advisory=true;
    FlightPipeline p(c,settings);ModelState state;
    state.position=Vec3(1.5,-.7,.2525);state.velocity=Vec3(-20,0,-5);
    p.start(observation(0,state).source_time_ns,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
    EventGeometry g;g.receiving_x=.15;
    std::set<std::uint64_t> ids;int physical=0;
    std::optional<std::uint64_t> confirmed_revision;
    for(int i=0;i<rate/5;++i) {
      auto s=observation(i*1./rate,state);p.observe(s);
      auto forecast=p.forecast(plane(),s.source_time_ns);
      EXPECT_TRUE(p.shot().active())<<p.shot().reason();
      EXPECT_LE(p.shot().observed_contacts(),1);
      if(p.shot().observed_contacts()==1) {
        if(!confirmed_revision)confirmed_revision=p.shot().revision();
        EXPECT_EQ(p.shot().revision(),*confirmed_revision);
        auto eligibility=select_physical_return(forecast,p.shot().export_snapshot(),c,s.source_time_ns);
        EXPECT_NE(eligibility.reason,"SECOND_RECEIVER_BOUNCE");
      }
      for(const auto &e:p.shot().events())
        if(e.kind==PhysicalEventKind::TABLE&&e.status==EventStatus::CONFIRMED)ids.insert(e.id);
      auto step=advance_ball(state,1./rate,c.physics,g);
      for(const auto &e:step.events)if(e.kind==PhysicalEventKind::TABLE)++physical;
      if(state.position.x()<.15||step.stopped)break;
    }
    EXPECT_EQ(physical,1);EXPECT_EQ(ids.size(),1u);
    RecordProperty("confirmed_ids_"+std::to_string(rate),std::to_string(ids.size()));
  }
}
TEST(PR64Review, RV03c_WireStyleRestorePreservesReversalIdentityOnNextObservation) {
  for(int rate:{150,300}) {
    ProbabilisticConfig c;ShotSettings settings;settings.game_legality_advisory=true;
    FlightPipeline original(c,settings),restored(c,settings);
    ModelState state;state.position=Vec3(1.5,-.7,.2525);state.velocity=Vec3(-20,0,-5);
    original.start(observation(0,state).source_time_ns,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
    EventGeometry g;bool transferred=false;int continued=0;
    for(int i=0;i<rate/5;++i) {
      auto s=observation(i*1./rate,state);original.observe(s);
      if(transferred) {
        restored.observe(s);++continued;
        EXPECT_EQ(restored.shot().observed_contacts(),1);
        EXPECT_EQ(restored.shot().revision(),original.shot().revision());
      } else if(original.shot().observed_contacts()==1) {
        auto snapshot=original.shot().export_snapshot();
#ifndef HOPE_PR64_BASELINE
        snapshot.spin_checkpoint.reset();
        for(auto &e:snapshot.events) {e.reversal_sample_ns.reset();e.bracket_begin_ns=e.bracket_end_ns=0;}
#endif
        restored.restore(snapshot,original.checkpoint(),original.source_origin_ns());
        transferred=true;
      }
      auto step=advance_ball(state,1./rate,c.physics,g);
      if(state.position.x()<.15||step.stopped)break;
    }
    EXPECT_TRUE(transferred);EXPECT_GE(continued,2);
  }
}
TEST(PR64Review, RV03d_ObservedBracketCorroboratesAuthoritativeContactOnce) {
  ShotSettings settings;settings.game_legality_advisory=true;
  ShotContext s({},settings);const auto epoch=observation(0,{}).source_time_ns;
  s.start(epoch,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
  ModelState pre;pre.position=Vec3(.6,-.7,.02);pre.velocity=Vec3(-20,0,-5);
  auto measured=observation(0,pre);measured.producer_omega_valid=true;
  measured.producer_omega_rad_s=Vec3::Zero();s.observe(measured);
  ASSERT_TRUE(s.confirm(PhysicalEventKind::TABLE,epoch+40000000,epoch+50000000,pre,"authority"));
  const auto revision=s.revision(),id=s.events().front().id;
  const auto omega=s.spin().omega;
  pre.position.x()-=.13;
#ifndef HOPE_PR64_BASELINE
  ASSERT_TRUE(s.confirm(PhysicalEventKind::TABLE,epoch+41000000,epoch+53333333,pre,
      "source_reversal_bracket",6666667,epoch+40000000,epoch+33333333,epoch+46666667));
#else
  s.confirm(PhysicalEventKind::TABLE,epoch+41000000,epoch+53333333,pre,"source_reversal_bracket",6666667);
#endif
  EXPECT_EQ(s.observed_contacts(),1);EXPECT_EQ(s.revision(),revision);
  EXPECT_EQ(s.events().front().id,id);EXPECT_EQ(s.spin().omega,omega);
}
TEST(PR64Review, RV05_DelayedContactSegmentsQuaternionWindow) {
  for (int rate : {150, 300}) {
    TrustedSpin spin;
    const double contact = .12;
    for (int i = 0; i <= int(.14*rate); ++i) {
      double t = double(i)/rate;
      auto s = angular(t, 10*std::min(t, contact)+54.2*std::max(0., t-contact));
      if (i % 2) s.orientation.coeffs() *= -1;
      spin.observe(s);
    }
    ASSERT_TRUE(spin.contact(42, observation(contact, {}).source_time_ns, Vec3(3,0,-2), {}));
    // Not enough pure post-contact span to call the mixed window measured.
    EXPECT_NE(spin.value().mode, EffectiveSpinMode::MEASURED);
    for (int i = int(.14*rate)+1; i < int(.20*rate); ++i) {
      double t = double(i)/rate;
      spin.observe(angular(t, 10*contact+54.2*(t-contact)));
      if (t > .175) {
        EXPECT_EQ(spin.value().mode, EffectiveSpinMode::MEASURED);
        ASSERT_TRUE(spin.value().usable());
        EXPECT_NEAR(spin.value().omega->y(), 54.2, 1e-8);
      }
    }
    auto value = spin.value();
    EXPECT_FALSE(spin.contact(42, observation(contact, {}).source_time_ns, Vec3(3,0,-2), {}));
    EXPECT_EQ(value.omega, spin.value().omega);
  }
}
TEST(PR64Review, RV06_HistoricalPreContactSpinRestoresImpactVelocity) {
  ShotSettings settings; settings.game_legality_advisory = true;
  ShotContext shot({}, settings);
  const auto epoch = observation(0, {}).source_time_ns;
  shot.start(epoch, -1, BoundaryEvidence::RESTART, HistoryContinuity::kUnknown);
  ModelState pre; pre.position = Vec3(.6,-.7,.02); pre.velocity = Vec3(3,0,-2);
  auto s = observation(0, pre); s.producer_omega_valid = true;
  s.producer_omega_rad_s = Vec3(0,150,0); shot.observe(s);
  s = observation(.02, pre); s.producer_omega_valid = true;
  s.producer_omega_rad_s = Vec3(0,54.2,0); shot.observe(s);
  pre.spin = shot.spin(); // Deliberately latest, causally wrong for the earlier bounce.
  ASSERT_TRUE(shot.confirm(PhysicalEventKind::TABLE, epoch+10000000, epoch+20000000, pre, "review"));
  const auto &e = shot.events().front();
  RecordProperty("pre_spin_available",e.pre.spin.usable()?"true":"false");
  RecordProperty("post_vx",std::to_string(e.post.velocity.x()));
  ASSERT_TRUE(e.pre.spin.usable());
  EXPECT_EQ(*e.pre.spin.omega, Vec3(0,150,0));
  EXPECT_NEAR(e.post.velocity.x(), 3., 1e-12); // vx - R*wy = 0: no slip impulse.
  EXPECT_EQ(e.post.spin.mode, EffectiveSpinMode::CONTACT_PROPAGATED);
}
TEST(PR64Review, RV07_AuthoritativeAndQuaternionZeroRemainAvailable) {
  TrustedSpin spin;
  auto s = angular(0, 0); s.orientation_authoritative = false;
  spin.observe(s); EXPECT_FALSE(spin.value().usable());
  s = angular(.01, 0); s.producer_omega_valid = true; s.producer_omega_rad_s.setZero();
  spin.observe(s);
  ASSERT_TRUE(spin.value().usable()); EXPECT_EQ(*spin.value().omega, Vec3::Zero());
  EXPECT_EQ(spin.value().mode, EffectiveSpinMode::MEASURED);
  spin.contact(1, s.source_time_ns, Vec3(0,0,-2), {});
  EXPECT_TRUE(spin.value().usable());
  EXPECT_EQ(spin.value().mode, EffectiveSpinMode::CONTACT_PROPAGATED);
  spin.reset();
  for (int i=0;i<20;++i) spin.observe(angular(i*.005,0));
  ASSERT_TRUE(spin.value().usable());
  EXPECT_EQ(spin.value().mode, EffectiveSpinMode::MEASURED);
  EXPECT_EQ(*spin.value().omega, Vec3::Zero());
}
TEST(PR64Review, RV06b_WireStyleCheckpointRecoversHistoricalSpinSuffix) {
  ShotSettings settings;settings.game_legality_advisory=true;
  ShotContext shot({},settings);
  const auto epoch=observation(0,{}).source_time_ns;
  shot.start(epoch,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
  ModelState state;state.position=Vec3(.6,-.7,.02);state.velocity=Vec3(3,0,-2);
  auto s=observation(0,state);s.producer_omega_valid=true;s.producer_omega_rad_s=Vec3(0,150,0);
  shot.observe(s);s=observation(.02,state);s.producer_omega_valid=true;s.producer_omega_rad_s=Vec3(0,54.2,0);
  shot.observe(s);
  auto snapshot=shot.export_snapshot();
#ifndef HOPE_PR64_BASELINE
  snapshot.spin_checkpoint.reset();
#endif
  ShotContext restored({},settings);restored.restore_snapshot(snapshot);
  ASSERT_TRUE(restored.confirm(PhysicalEventKind::TABLE,epoch+10000000,epoch+20000000,state,"wire_review"));
  ASSERT_TRUE(restored.events().front().pre.spin.usable());
  EXPECT_EQ(*restored.events().front().pre.spin.omega,Vec3(0,150,0));
  EXPECT_NEAR(restored.events().front().post.velocity.x(),3,1e-12);
}
TEST(PR64Review, RV08_FiniteNetCornerClearanceAndTrueContact) {
  BallPhysics physics; physics.gravity.setZero(); physics.drag_k = physics.magnus_k = 0;
  EventGeometry g;
  for (int direction : {-1,1})
    for (double clearance : {.02753, .019, .020}) {
      ModelState s;
      const double offset = clearance/std::sqrt(2.);
      s.position = Vec3(g.table.net_x-direction*.04, g.table.y_max+g.table.net_overhang+offset, g.table.net_height+offset);
      s.velocity = Vec3(direction*20,0,0);
      auto step = advance_ball(s,.004,physics,g);
      bool collision=false;
      for (const auto &e:step.events) collision |= e.kind==PhysicalEventKind::NET_COLLISION;
      EXPECT_EQ(collision,clearance<=.02) << direction << ":" << clearance;
      if(direction==1 && clearance==.02753) RecordProperty("clearance_02753_collision",collision?"true":"false");
    }
}
TEST(PR64Review, RV09_NetFirstContactTimeAcrossStepSizes) {
  BallPhysics p; p.gravity.setZero(); p.drag_k=p.magnus_k=0;
  EventGeometry g;
  for(double dt:{.001,.0005,.00025}) {
    ModelState s; s.position=Vec3(g.table.net_x-.03,-.7,g.table.net_height+.012);
    s.velocity=Vec3(20,0,0);
    double time=0; bool found=false;
    for(int i=0;i<20&&!found;++i) {
      const auto step=advance_ball(s,dt,p,g);
      for(const auto &e:step.events) if(e.kind==PhysicalEventKind::NET_COLLISION) {
        EXPECT_NEAR(time+e.dt_s,(.03-std::sqrt(.02*.02-.012*.012))/20,1e-9);
        found=true;
      }
      time+=step.elapsed_s;
    }
    EXPECT_TRUE(found);
  }
}
TEST(PR64Review, RV09b_TrueCornerCollisionWinsPlaneTie) {
  BallPhysics p;p.gravity.setZero();p.drag_k=p.magnus_k=0;
  EventGeometry g;
  const double offset=std::sqrt(p.radius*p.radius-2*.012*.012);
  g.receiving_x=g.table.net_x+offset;
  ModelState s;s.position=Vec3(g.table.net_x+.03,g.table.y_max+g.table.net_overhang+.012,
                              g.table.net_height+.012);
  s.velocity=Vec3(-20,0,0);
  const auto step=advance_ball(s,.001,p,g);
  ASSERT_FALSE(step.events.empty());
  EXPECT_EQ(step.events.front().kind,PhysicalEventKind::NET_COLLISION);
  EXPECT_NEAR(step.events.front().dt_s,(.03-offset)/20,1e-9);
  EXPECT_TRUE(step.stopped);
  EXPECT_FALSE(std::any_of(step.events.begin(),step.events.end(),
      [](const auto &e){return e.kind==PhysicalEventKind::PLANE;}));
}
TEST(PR64Review, RV10_SameStepBounceNoiseReachesPlane) {
  for (double dt : {.001,.0005,.00025}) {
    ProbabilisticConfig c; c.physics.drag_k=c.physics.magnus_k=0; c.physics.gravity.setZero();
    c.integration_dt_s=dt; c.horizon_s=.002;
    StateBelief b; b.initialized=true; b.covariance=1e-10*Matrix6::Identity();
    const double vx=-3, post_vx=vx*(1-c.physics.table_tangential_gain);
    // Friction cap, not tangential gain, is active for these velocities.
    const double impulse=std::min(c.physics.table_tangential_gain*3,
       c.physics.table_friction_cap_mu*(1+c.physics.restitution_v));
    (void)post_vx;
    b.mean<<c.x_hit+3*.000499+(3-impulse)*.000251,-.7,.02+.000499,vx,0,-1;
    ForecastNoiseConfig n=ForecastNoiseConfig::inherited(c); n.acceleration_spectral_density=0; n.attribute_covariance=true;
    auto f=forecast_nominal(b,c,n);
    ASSERT_TRUE(f.hit_plane);
    const auto &e=*f.hit_plane;
    RecordProperty("vx_variance_dt_"+std::to_string(std::llround(dt*1e6)),std::to_string(e.covariance(3,3)));
    EXPECT_NEAR(e.dt_s,.000750,1e-10);
    EXPECT_NEAR(e.covariance(3,3),.4225,1e-6);
    EXPECT_NEAR(e.covariance_budget.bounce(3,3),.4225,1e-8);
    EXPECT_LT((e.covariance_budget.total()-e.covariance).norm(),1e-9);
    Eigen::SelfAdjointEigenSolver<Matrix7> eigen(e.covariance);
    EXPECT_GE(eigen.eigenvalues().minCoeff(),-1e-10);
  }
}
TEST(PR64Review, RV04_RealCloseBouncesRemainDistinctAndGapDoesNotInventContact) {
  for(int rate:{150,300}) {
    ProbabilisticConfig c; c.physics.drag_k=0; c.physics.gravity.z()=-100;
    c.measurement_std_m=Vec3::Constant(.0001);
    ShotSettings settings;settings.game_legality_advisory=true;
    FlightPipeline p(c,settings);
    ModelState state;state.position=Vec3(.9,-.7,.025);state.velocity=Vec3(-2,0,-1);
    p.start(observation(0,state).source_time_ns,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
    EventGeometry g;g.receiving_x=.15;
    int true_count=0;
    for(int i=0;i<rate/10;++i) {
      p.observe(observation(double(i)/rate,state));
      for(const auto &e:advance_ball(state,1./rate,c.physics,g).events)
        if(e.kind==PhysicalEventKind::TABLE)++true_count;
      if(p.shot().observed_contacts()>=2)break;
    }
    EXPECT_GE(true_count,2); EXPECT_EQ(p.shot().observed_contacts(),2);
    auto f=p.forecast(plane(),p.shot().samples().back().source_time_ns);
    EXPECT_EQ(select_physical_return(f,p.shot().export_snapshot(),c,f.reference_ns).reason,"SECOND_RECEIVER_BOUNCE");
  }
  // A long missing interval spans the sole bounce: no source reversal exists.
  ProbabilisticConfig c;ShotSettings settings;settings.game_legality_advisory=true;
  FlightPipeline p(c,settings);ModelState state;
  state.position=Vec3(1,-.7,.1);state.velocity=Vec3(-3,0,-1);
  p.start(observation(0,state).source_time_ns,-1,BoundaryEvidence::RESTART,HistoryContinuity::kComplete);
  EventGeometry g;
  bool hidden=false;
  for(int i=0;i<60;++i) {
    const double t=i/300.;
    if(t<.025||t>.13)p.observe(observation(t,state));
    for(const auto &e:advance_ball(state,1./300,c.physics,g).events)
      if(e.kind==PhysicalEventKind::TABLE)hidden=true;
  }
  EXPECT_TRUE(hidden); EXPECT_EQ(p.shot().observed_contacts(),0);
  EXPECT_EQ(p.shot().continuity(),HistoryContinuity::kIncomplete);
}
TEST(PR64Review, RV11_IndependentImpulseCovarianceOracleAndFreeFlightControl) {
  // Independent analytic event map for a post-bounce impulse. No runtime
  // Jacobian, covariance helper or stochastic runtime participates in this oracle.
  ProbabilisticConfig c;c.physics.drag_k=c.physics.magnus_k=0;c.physics.gravity.setZero();
  c.horizon_s=.002;
  StateBelief b;b.initialized=true;b.covariance=1e-14*Matrix6::Identity();
  const double impulse=c.physics.table_friction_cap_mu*(1+c.physics.restitution_v);
  const double vx=-3+impulse, vz=c.physics.restitution_v;
  const double dx=-vx*.000251;
  b.mean<<c.x_hit+3*.000499+dx,-.7,.020499,-3,0,-1;
  auto map=[&](const Vec3 &dv) {
    const Vec3 v=Vec3(vx,0,vz)+dv;
    const double duration=-dx/v.x();
    Eigen::Matrix<double,7,1> x;
    x<<c.x_hit,-.7+duration*v.y(),.02+duration*v.z(),v,.000499+duration;
    return x;
  };
  Eigen::Matrix<double,7,3> j;
  for(int k=0;k<3;++k) {
    Vec3 a=Vec3::Zero();a[k]=1e-6;
    j.col(k)=(map(a)-map(-a))/(2e-6);
  }
  const Matrix7 reference=j*c.bounce_velocity_std.array().square().matrix().asDiagonal()*j.transpose();
  auto n=ForecastNoiseConfig::inherited(c);n.acceleration_spectral_density=0;n.attribute_covariance=true;
  for(double dt:{.001,.0005,.00025}) {
    c.integration_dt_s=dt;
    auto f=forecast_nominal(b,c,n);ASSERT_TRUE(f.hit_plane);
    EXPECT_LT((f.hit_plane->covariance_budget.bounce-reference).norm(),1e-8);
    EXPECT_LT((f.waypoints.back().covariance_budget.total()-f.waypoints.back().covariance).norm(),1e-9);
  }
  // Plane first, no contact in the step: noise from a future bounce is absent.
  b.mean<<c.x_hit+.001,-.7,.5,-3,0,-1;
  auto f=forecast_nominal(b,c,n);ASSERT_TRUE(f.hit_plane);
  EXPECT_TRUE(f.hit_plane->covariance_budget.bounce.isZero());
}
TEST(PR64Review, RV11c_BounceNoiseChangesPhysicalReturnAdmission) {
  // A plane four milliseconds after contact isolates an application-level
  // effect: configured bounce noise makes the contact order uncertain. With
  // that noise explicitly disabled the same physical branch is eligible.
  for (double dt : {.005, .00025}) {
    ProbabilisticConfig c;
    c.physics.drag_k = c.physics.magnus_k = 0;
    c.physics.gravity.setZero();
    c.integration_dt_s = dt;
    c.trajectory_dt_s = c.horizon_s = .01;
    ModelState contact;
    contact.velocity = Vec3(-3, 0, -2);
    apply_model_contact(contact, c.physics);
    StateBelief belief;
    belief.initialized = true;
    belief.covariance = 1e-10 * Matrix6::Identity();
    belief.mean << c.x_hit + 3 * .000499 - contact.velocity.x() * .004,
        -.7, c.physics.radius + 2 * .000499, -3, 0, -2;
    ModelState initial;
    initial.position = belief.mean.head<3>();
    const auto source = observation(0, initial);
    ShotSnapshot shot;
    shot.generation = 1;
    shot.direction = -1;
    shot.boundary_ns = source.source_time_ns;
    shot.observations.push_back(source);
    FlightPrediction forecast;
    forecast.physical_available = true;
    forecast.belief = belief;
    forecast.token.generation = shot.generation;
    forecast.reference_ns = source.source_time_ns;
    forecast.expires_ns = forecast.reference_ns + 30000000;
    forecast.plane = plane();
    auto noise = ForecastNoiseConfig::inherited(c);
    noise.acceleration_spectral_density = 0;
    for (bool uncertain : {false, true}) {
      SCOPED_TRACE(::testing::Message() << dt << ":" << uncertain);
      noise.bounce_velocity_std = uncertain ? c.bounce_velocity_std : Vec3::Zero();
      forecast.nominal = forecast_nominal(belief, c, noise);
      ASSERT_TRUE(forecast.nominal.hit_plane);
      const auto result = select_physical_return(forecast, shot, c, forecast.reference_ns);
      EXPECT_EQ(result.eligible, !uncertain);
      EXPECT_EQ(result.reason, uncertain ? "CONTACT_ORDER_UNCERTAIN" : "RETURN_ELIGIBLE");
      EXPECT_NEAR(forecast.nominal.hit_plane->covariance(3, 3),
                  uncertain ? .4225 : 0., 1e-8);
      if (result.input)
        EXPECT_EQ(result.input->covariance, forecast.nominal.hit_plane->covariance);
    }
  }
}
TEST(PR64Review, RV12_NoOpAndRejectedContextsPreserveJobsSemanticChangeFencesThem) {
  ShotContext s({});s.start(0,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
  BallSample sample;sample.source_time_ns=1;sample.source_time_s=1e-9;
  sample.position=Vec3(1,-.7,.4);ASSERT_TRUE(s.observe(sample));
  auto job=s.snapshot();
  EXPECT_FALSE(s.context(MatchContext::SERVE,1,"referee",s.generation()+1));
  EXPECT_TRUE(s.context(MatchContext::UNKNOWN,1,"referee",s.generation()));
  EXPECT_TRUE(s.publishable(job,1));
  auto delayed=s.snapshot();
  EXPECT_FALSE(s.context(MatchContext::SERVE,1,"wrong-authority",s.generation()));
  EXPECT_EQ(s.revision(),delayed.context_revision);
  EXPECT_TRUE(s.context(MatchContext::SERVE,1,"referee",s.generation()));
  EXPECT_FALSE(s.publishable(delayed,1));
  auto current=s.snapshot();
  s.context(MatchContext::SERVE,1,"referee",s.generation());
  EXPECT_TRUE(s.publishable(current,1));
}
TEST(PR64Review, RV11b_EdgeBranchAndTangentPlaneNeverClaimValidLinearization) {
  ProbabilisticConfig c;c.physics.drag_k=c.physics.magnus_k=0;c.horizon_s=.01;
  StateBelief b;b.initialized=true;b.covariance=1e-6*Matrix6::Identity();
  b.mean<<.001,-.7,.021,-.01,0,-1;
  auto f=forecast_nominal(b,c);
  auto bounce=std::find_if(f.events.begin(),f.events.end(),[](const auto &e){return e.kind=="table_bounce";});
  ASSERT_NE(bounce,f.events.end());EXPECT_EQ(bounce->quality,LinearizationQuality::BranchSensitive);
  b.mean<<c.x_hit+.000001,-.7,.5,-.01,0,-1;
  f=forecast_nominal(b,c);ASSERT_TRUE(f.hit_plane);
  EXPECT_EQ(f.hit_plane->quality,LinearizationQuality::IllConditioned);
  EXPECT_NE(f.uncertainty_provenance.find("unmodelled"),std::string::npos);
}
TEST(PR64Review, RV13_OriginalNakashimaFrictionSensitivityAndTrueZero) {
  PlannerConfig config;config.adaptive_predict_horizon=true;config.max_predict_time_cap_s=3;
  BallState s;s.valid=true;s.position=Vec3(1.4,-.55,.35);s.velocity=Vec3(-4.2,.35,-1.2);
  BallPhysics a,b;a.nakashima_friction_mu=0;b.nakashima_friction_mu=.5;
  TrajectoryPredictor p(a,config,{}),q(b,config,{});
  for(auto mode:{SpinPhysicsMode::kNakashimaBounce,SpinPhysicsMode::kNakashimaBounceAndMagnus,
                SpinPhysicsMode::kVenueGripBounce,SpinPhysicsMode::kVenueGripBounceAndMagnus,
                SpinPhysicsMode::kLegacyNoSpin}) {
    auto x=p.predict_with_spin(s,.15,Vec3::Zero(),mode), y=q.predict_with_spin(s,.15,Vec3::Zero(),mode);
    ASSERT_TRUE(x.valid)<<x.reason;ASSERT_TRUE(y.valid)<<y.reason;
    if(mode==SpinPhysicsMode::kNakashimaBounce||mode==SpinPhysicsMode::kNakashimaBounceAndMagnus)
      EXPECT_GT((x.ball_velocity-y.ball_velocity).norm(),.1);
    else EXPECT_EQ(x.ball_velocity,y.ball_velocity);
  }
}
TEST(PR64Review, RV14_FastAcquisitionReachesPhysicalReturnAndValidatedOutgoing) {
  int eligible_count=0, commands=0;
  for(int rate:{150,300})for(double speed:{10.,12.}) {
    ProbabilisticConfig c;ShotSettings settings;settings.game_legality_advisory=true;
    FlightPipeline p(c,settings);ModelState state;
    state.position=Vec3(speed==10?2.5:2.6,-.7,speed==10?.32:.3);state.velocity=Vec3(-speed,0,-.5);
    EventGeometry g;g.receiving_x=.15;
    for(int i=0;i<rate/2;++i) {
      auto s=observation(double(i)/rate,state);p.observe(s);
      if(p.shot().active()) {
        auto f=p.forecast(plane(),s.source_time_ns);
        auto e=select_physical_return(f,p.shot().export_snapshot(),c,s.source_time_ns);
        if(e.eligible) {
          ++eligible_count;
          EXPECT_NE(e.input->history,HistoryContinuity::kComplete);
          auto result=ValidatedReturnPlanner(c).solve(*e.input);
          if(result.command.valid) {++commands;EXPECT_TRUE(result.return_ok);}
        }
      }
      auto step=advance_ball(state,1./rate,c.physics,g);
      if(state.position.x()<.15||step.stopped)break;
    }
  }
  EXPECT_GT(eligible_count,20); EXPECT_GT(commands,10);
}
TEST(PR64Review, RV15_ClearingTopEdgeReturnUsesCorrectedSharedKernel) {
  ProbabilisticConfig c;c.physics.drag_k=c.physics.magnus_k=0;
  const double net_time=(c.table.net_x-.15)/4;
  const double vz=-1.6+9.81*net_time;
  const double z=c.table.net_height+.024-vz*net_time+4.905*net_time*net_time;
  const double duration=(vz+std::sqrt(vz*vz+19.62*(z-.02)))/9.81;
  ContactPlanningInput i;i.contact.pre.position=Vec3(.15,-.7,z);
  i.contact.pre.velocity=Vec3(-3,0,vz);
  ValidatedReturnSettings settings;settings.durations_s={duration};
  settings.landing_surface=Vec3(.15+4*duration,-.7,0);
  auto result=ValidatedReturnPlanner(c,settings).solve(i);
  ASSERT_TRUE(result.command.valid)<<result.reason;
  EXPECT_GT(result.net_clearance_m,0);
  EXPECT_LT((result.reconstructed.velocity-Vec3(4,0,vz)).norm(),1e-4);
  // A physically lower trajectory really hits the net and must remain invalid.
  i.contact.pre.position.z()-=.015;
  result=ValidatedReturnPlanner(c,settings).solve(i);
  EXPECT_FALSE(result.command.valid);
}
TEST(PR64Review, RV17_CausalCheckpointAndObservationCutoffParity) {
  ProbabilisticConfig c;ShotSettings settings;settings.game_legality_advisory=true;
  FlightPipeline p(c,settings);ModelState state;
  state.position=Vec3(1,-.7,.12);state.velocity=Vec3(-3,0,-1);
  p.start(observation(0,state).source_time_ns,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
  EventGeometry g;
  for(int i=0;i<45;++i) {
    auto s=observation(i/300.,state);
    s.producer_omega_valid=true;s.producer_omega_rad_s=Vec3(0,i<20?30:54.2,0);
    p.observe(s);
    if(i>6 && p.shot().active()) {
      FlightPipeline restored(c,settings);
      restored.restore(p.shot().export_snapshot(),p.checkpoint(),p.source_origin_ns());
      auto a=p.forecast(plane(),s.source_time_ns),b=restored.forecast(plane(),s.source_time_ns);
      EXPECT_EQ(a.belief.mean,b.belief.mean);EXPECT_EQ(a.belief.covariance,b.belief.covariance);
      ASSERT_EQ(a.nominal.events.size(),b.nominal.events.size());
      for(std::size_t j=0;j<a.nominal.events.size();++j) {
        EXPECT_EQ(a.nominal.events[j].pre,b.nominal.events[j].pre);
        EXPECT_EQ(a.nominal.events[j].pre_spin.omega,b.nominal.events[j].pre_spin.omega);
      }
    }
    advance_ball(state,1./300,c.physics,g);
  }
}
TEST(PR64Review, RV17b_LongGapStartsFreshQualifiedShotWithoutOldBounceLedger) {
  ProbabilisticConfig c;ShotSettings settings;settings.game_legality_advisory=true;
  FlightPipeline p(c,settings);std::uint64_t previous=0;
  for(int flight=0;flight<2;++flight) {
    ModelState state;state.position=Vec3(2.5,-.7,.32);state.velocity=Vec3(-10,0,-.5);
    EventGeometry g;
    for(int i=0;i<35;++i) {
      auto s=observation(flight+i/150.,state);p.observe(s);
      if(i==0&&flight) { EXPECT_FALSE(p.shot().active()); }
      if(i==10) {
        EXPECT_TRUE(p.shot().active());
        EXPECT_GT(p.shot().generation(),previous);previous=p.shot().generation();
        EXPECT_EQ(p.shot().observed_contacts(),0);
      }
      advance_ball(state,1./150,c.physics,g);
    }
  }
}
TEST(PR64Review, RV08b_NetTopFlyoverRemainsAnEligibleIncomingReturn) {
  ProbabilisticConfig c;c.physics.drag_k=c.physics.magnus_k=0;
  StateBelief b;b.initialized=true;b.covariance=Matrix6::Identity()*1e-10;
  const double net_time=(2.6-c.table.net_x)/10;
  const double vz=-4.+9.81*net_time;
  const double z=c.table.net_height+.024-vz*net_time+4.905*net_time*net_time;
  b.mean<<2.6,-.7,z,-10,0,vz;
  FlightPrediction f;f.belief=b;f.nominal=forecast_nominal(b,c);
  ASSERT_TRUE(f.nominal.hit_plane);
  EXPECT_FALSE(std::any_of(f.nominal.events.begin(),f.nominal.events.end(),
      [](const auto &e){return e.kind=="net_collision";}));
  f.physical_available=true;f.plane=plane();f.plane.x_hit_m=c.x_hit;
  f.reference_ns=observation(0,{}).source_time_ns;
  f.expires_ns=f.reference_ns+30000000;f.token.generation=1;
  ShotSnapshot snapshot;snapshot.generation=1;snapshot.direction=-1;snapshot.boundary_ns=f.reference_ns;
  snapshot.observations.push_back(observation(0,{}));
  auto result=select_physical_return(f,snapshot,c,f.reference_ns);
  EXPECT_TRUE(result.eligible)<<result.reason;
}
} // namespace
