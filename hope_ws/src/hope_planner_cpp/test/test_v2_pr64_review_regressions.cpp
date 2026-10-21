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
        for (bool known : {false, true}) {
          SCOPED_TRACE(::testing::Message() << rate << ":" << speed << ":" << direction << ":" << known);
          ProbabilisticConfig c;
          ShotSettings settings;
          settings.game_legality_advisory = true;
          FlightPipeline pipeline(c, settings);
          ModelState state;
          state.position = Vec3(direction < 0 ? 2.6 : .14, -.7, .42);
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
          for (int i = 0; i < rate / 3; ++i) {
            auto s = observation(t, state);
            s.producer_omega_valid = known;
            if (known) s.producer_omega_rad_s = *state.spin.omega;
            pipeline.observe(s);
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
            t += dt;
            if (step.stopped || (direction < 0 && state.position.x() < .15)) break;
          }
          EXPECT_TRUE(acquired) << pipeline.shot().reason();
          EXPECT_TRUE(physical);
          if (direction < 0) EXPECT_TRUE(future_plane);
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
  s = observation(.02, pre); s.producer_omega_rad_s = Vec3(0,54.2,0); shot.observe(s);
  pre.spin = shot.spin(); // Deliberately latest, causally wrong for the earlier bounce.
  ASSERT_TRUE(shot.confirm(PhysicalEventKind::TABLE, epoch+10000000, epoch+20000000, pre, "review"));
  const auto &e = shot.events().front();
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
    EXPECT_NEAR(e.dt_s,.000750,1e-10);
    EXPECT_NEAR(e.covariance(3,3),.4225,1e-6);
    EXPECT_NEAR(e.covariance_budget.bounce(3,3),.4225,1e-8);
    EXPECT_LT((e.covariance_budget.total()-e.covariance).norm(),1e-9);
    Eigen::SelfAdjointEigenSolver<Matrix7> eigen(e.covariance);
    EXPECT_GE(eigen.eigenvalues().minCoeff(),-1e-10);
  }
}
} // namespace
