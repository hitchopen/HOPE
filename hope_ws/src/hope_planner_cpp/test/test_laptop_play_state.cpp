#include "hope_planner_cpp/laptop_play_state.hpp"
#include "hope_planner_cpp/ball_dynamics.hpp"
#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {
BallSample sample(double t, const Vec3& p) {
  BallSample s; s.source_time_s = 10 + t; s.source_time_ns = std::llround(s.source_time_s * 1e9);
  s.receipt_steady_ns = s.source_time_ns; s.position = p;
  return s;
}

// Synthetic source exposures, not wall-clock sleeps. The identical canonical
// arc can be encoded at either physical end and transformed before estimation.
void arc(LaptopPlayState& c, int hz, Vec3 p, Vec3 v, double duration,
         double start = 0, bool p2 = false, double first_retention = 1, double later_retention = 1) {
  int bounces = 0;
  for (int i = 0; i <= std::lround(duration * hz); ++i) {
    Vec3 raw = p2 ? Vec3(2.74 - p.x(), -1.525 - p.y(), p.z()) : p;
    Vec3 canonical = p2 ? Vec3(2.74 - raw.x(), -1.525 - raw.y(), raw.z()) : raw;
    c.observe(sample(start + double(i) / hz, canonical));
    const double dt = 1.0 / hz;
    double remaining = dt;
    const double endz = p.z() + v.z() * dt - 4.905 * dt * dt;
    if (endz < .02 && v.z() < 0 && p.x() >= 0 && p.x() <= 2.74) {
      const double contact = (v.z() + std::sqrt(v.z()*v.z() + 19.62*(p.z()-.02))) / 9.81;
      p += v * contact + Vec3(0,0,-4.905*contact*contact);
      v.z() -= 9.81 * contact; v.z() = -v.z() * .9;
      v.x() *= bounces++ == 0 ? first_retention : later_retention;
      p.z() = .02; remaining -= contact;
    }
    p += v * remaining + Vec3(0,0,-4.905*remaining*remaining);
    v.z() -= 9.81 * remaining;
  }
}

TEST(LaptopPlayState, StartupHeldAndMovingHeldRemainInvalid) {
  LaptopPlayState c;
  EXPECT_EQ(c.state(), PlayState::INVALID);
  for (int i=0; i<120; ++i) c.observe(sample(i/300., Vec3(.5+i/300., -.7, .6)));
  EXPECT_EQ(c.state(), PlayState::INVALID);
  EXPECT_FALSE(c.may_forward());
}

TEST(LaptopPlayState, BothRatesAndBothPlacementsHaveSameFourFlightLabels) {
  for (int hz : {150,300}) for (bool p2 : {false,true}) {
    LaptopPlayState out, in, serve, opponent_serve;
    arc(out,hz,{.3,-.7,.6},{5,0,.5},.16,0,p2);
    arc(in,hz,{2.44,-.7,.6},{-5,0,.5},.16,0,p2);
    EXPECT_EQ(out.state(),PlayState::RALLY); EXPECT_FALSE(out.may_forward());
    EXPECT_EQ(in.state(),PlayState::OPPONENT_RALLY); EXPECT_TRUE(in.may_forward());
    arc(serve,hz,{.04,-.7,.4},{3.5,0,-2},.19,0,p2);
    arc(opponent_serve,hz,{2.70,-.7,.4},{-3.5,0,-2},.19,0,p2);
    EXPECT_EQ(serve.state(),PlayState::SERVE) << hz << ' ' << p2 << ' ' << serve.reason();
    EXPECT_EQ(opponent_serve.state(),PlayState::OPPONENT_SERVE) << opponent_serve.reason();
    EXPECT_FALSE(serve.may_forward()); EXPECT_TRUE(opponent_serve.may_forward());
    EXPECT_EQ(serve.striker_bounces(),1); EXPECT_EQ(opponent_serve.striker_bounces(),1);
  }
}

TEST(LaptopPlayState, SameDirectionServeRetainsGenerationHistoryAndContactOnlyOnce) {
  LaptopPlayState c;
  arc(c,300,{2.70,-.7,.4},{-3.5,0,-2},.08);
  ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY);
  const auto generation=c.generation(); const auto boundary=c.boundary_time();
  const double t=.083333333333;
  arc(c,300,{2.70-3.5*t,-.7,.4-2*t-4.905*t*t},{-3.5,0,-2-9.81*t},.12,t);
  EXPECT_EQ(c.state(),PlayState::OPPONENT_SERVE) << c.reason();
  EXPECT_EQ(c.generation(),generation); EXPECT_EQ(c.boundary_time(),boundary);
  EXPECT_EQ(c.striker_bounces(),1); EXPECT_GT(c.retained_samples(),25U);
}

TEST(LaptopPlayState, StaleClearsShotAndSameFlightCannotRearm) {
  LaptopPlayState c;
  arc(c,300,{2.6,-.7,.8},{-3,0,0},.1);
  ASSERT_TRUE(c.may_forward());
  c.poll(10'201'000'000LL);
  EXPECT_EQ(c.state(),PlayState::INVALID); EXPECT_EQ(c.retained_samples(),0U);
  arc(c,300,{2.27,-.7,.74065},{-3,0,-1.0791},.12,.11);
  EXPECT_EQ(c.state(),PlayState::INVALID);
}

TEST(LaptopPlayState, MissingPrehistoryAndDuplicateCannotOpenGate) {
  LaptopPlayState c;
  arc(c,150,{1,-.7,.8},{-3,0,0},.1);
  ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY);
  EXPECT_FALSE(c.history_known()); EXPECT_FALSE(c.may_forward());
  c.observe(sample(.1,{1,-.7,.8}));
  c.poll(10'201'000'000LL); EXPECT_EQ(c.state(),PlayState::INVALID);
}

TEST(LaptopPlayState, LocalServeWaitsForBallAndRepeatedServePreservesIntent) {
  LaptopPlayState c;c.local_intent("SERVE");
  EXPECT_EQ(c.state(),PlayState::SERVE);EXPECT_TRUE(c.serve_armed());EXPECT_FALSE(c.may_forward());
  const auto generation=c.generation();
  c.local_intent("SERVE");c.poll(60'000'000'000LL);
  EXPECT_EQ(c.generation(),generation);EXPECT_EQ(c.state(),PlayState::SERVE);
  EXPECT_FALSE(c.local_intent("FAULT"));EXPECT_FALSE(c.local_intent("MOTION"));
  EXPECT_EQ(c.generation(),generation);EXPECT_TRUE(c.serve_armed());
}

TEST(LaptopPlayState, ApexIsNotHeldAndInvalidOrientationDoesNotMatter) {
  LaptopPlayState c;
  arc(c,300,{2.5,-.7,.5},{-3,0,.8},.18);
  EXPECT_EQ(c.state(),PlayState::OPPONENT_RALLY); EXPECT_TRUE(c.may_forward());
  c.observe(sample(.19,{1.93,-.7,-.05}));
  EXPECT_EQ(c.state(),PlayState::INVALID);
}

TEST(LaptopPlayState, ForecastAdvisoryNeverChangesValidityHistoryOrObservedContacts) {
  LaptopPlayState c;
  arc(c,300,{2.7,-.7,.7},{-3,0,0},.1);
  c.set_hit_plane(-.1);
  BallState fit; fit.valid=true; fit.source_time_s=c.latest_time();
  fit.position={2.4,-.7,.12}; fit.velocity={-.3,0,-.5}; fit.residual_rms_m=.001;
  const auto generation=c.generation();const auto history=c.retained_samples();
  const auto reason=c.reason();
  c.forecast(fit,BallPhysics{},SpinEstimate{});
  EXPECT_FALSE(c.forecast_advisory().empty());
  EXPECT_EQ(c.state(),PlayState::OPPONENT_RALLY);EXPECT_TRUE(c.may_forward());
  EXPECT_EQ(c.generation(),generation);EXPECT_EQ(c.retained_samples(),history);
  EXPECT_EQ(c.reason(),reason);
  EXPECT_EQ(c.striker_bounces(),0); EXPECT_EQ(c.receiver_bounces(),0);
  fit.valid=false;c.forecast(fit,BallPhysics{},SpinEstimate{});
  EXPECT_TRUE(c.forecast_advisory().empty()); // A stale/unusable fit is not a current warning.
  fit.valid=true;c.forecast(fit,BallPhysics{},SpinEstimate{});
  c.reset("operator_stop");EXPECT_TRUE(c.forecast_advisory().empty());
}

TEST(LaptopPlayState, RepeatedForecastViolationsAreAdvisoryWithOrWithoutSpin) {
  struct Case { Vec3 p,v; double plane; const char* advisory; };
  const Case cases[] = {
    {{2.4,-.7,.7},{-12,0,0},.15,"predicted_target_without_required_bounce"},
    {{2.4,-.7,.7},{-12,0,0},-5,"predicted_floor_before_target"},
    {{1.5,-.7,.18},{-1,0,-.5},.15,"predicted_net_collision"},
    {{2.4,-.01,.7},{-4,3,0},.15,"predicted_side_boundary_exit"},
    {{2.4,-.7,.12},{-.3,0,-.5},-.1,"predicted_second_striker_bounce"},
  };
  for(const auto& item:cases)for(bool spin_known:{false,true}) {
    SCOPED_TRACE(std::string(item.advisory)+(spin_known?" known spin":" unknown spin"));
    LaptopPlayState c;arc(c,300,{2.7,-.7,.7},{-3,0,0},.1);
    c.set_hit_plane(item.plane);
    const auto generation=c.generation();const auto history=c.samples();
    BallState fit;fit.valid=true;fit.source_time_s=c.latest_time();
    fit.position=item.p;fit.velocity=item.v;fit.residual_rms_m=.001;
    BallPhysics physics;physics.drag_k=0;
    SpinEstimate spin;spin.valid=spin_known;spin.omega_rad_s={0,.1,0};
    for(int i=0;i<10;++i) {
      c.forecast(fit,physics,spin);
      EXPECT_EQ(c.forecast_advisory(),item.advisory);
      EXPECT_EQ(c.state(),PlayState::OPPONENT_RALLY);EXPECT_TRUE(c.may_forward());
      EXPECT_EQ(c.generation(),generation);ASSERT_EQ(c.samples().size(),history.size());
      EXPECT_EQ(c.striker_bounces(),0);EXPECT_EQ(c.receiver_bounces(),0);
      for(std::size_t j=0;j<history.size();++j) {
        EXPECT_EQ(c.samples()[j].source_time_ns,history[j].source_time_ns);
        EXPECT_EQ(c.samples()[j].position,history[j].position);
      }
    }
  }
}

TEST(LaptopPlayState, UnknownTopspinLongForecastCannotDiscardObservedLegalIncomingBall) {
  for(int hz:{150,300}) {
    LaptopPlayState c;c.set_hit_plane(.5);
    BallPhysics physics;physics.drag_k=0;physics.table_tangential_gain=0;
    Vec3 p(3.,-.7,1.),v(-8,0,0),true_spin(0,-400,0);
    std::uint64_t generation=0;bool warned=false;int physical_bounces=0;
    // Truth uses the shared Magnus and contact kernels. Mocap supplies only
    // position; no quaternion/spin estimate is passed to the classifier.
    for(int i=0;i<=std::lround(.38*hz);++i) {
      double t=i/double(hz);c.observe(sample(t,p));
      if(i==std::lround(.09*hz)) {
        ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY);ASSERT_TRUE(c.may_forward());
        generation=c.generation();BallState fit;fit.valid=true;
        fit.source_time_s=c.latest_time();fit.position=p;fit.velocity=v;fit.residual_rms_m=.001;
        c.forecast(fit,physics,SpinEstimate{});
        EXPECT_EQ(c.forecast_advisory(),"predicted_target_without_required_bounce");
        warned=true;
      }
      if(warned) {
        ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY) << c.reason();
        ASSERT_TRUE(c.may_forward());EXPECT_EQ(c.generation(),generation);
      }
      // Integrate truth at <=1 ms, resolving contact within each step.
      for(double remaining=1./hz;remaining>1e-10;) {
        const double dt=std::min(.001,remaining);remaining-=dt;
        const Vec3 a=ball_acceleration(physics,v,true_spin);
        const Vec3 next=p+v*dt+.5*a*dt*dt;
        if(p.z()>.02 && next.z()<=.02 && v.z()<0) {
          double first=dt*(p.z()-.02)/(p.z()-next.z());
          p+=v*first+.5*a*first*first;v+=a*first;p.z()=.02;
          ASSERT_GE(p.x(),0);ASSERT_LT(p.x(),1.37);++physical_bounces;
          table_contact(physics,v,true_spin);
          const Vec3 after=ball_acceleration(physics,v,true_spin);double rest=dt-first;
          p+=v*rest+.5*after*rest*rest;v+=after*rest;
        } else {p=next;v+=a*dt;}
      }
    }
    EXPECT_TRUE(warned);EXPECT_EQ(physical_bounces,1);EXPECT_EQ(c.receiver_bounces(),1);
    EXPECT_TRUE(c.forecast_advisory().empty()); // Observed contact supersedes the old forecast.
  }
}

TEST(LaptopPlayState, SecondStrikerContactInvalidatesOnlyIncomingFlight) {
  for (int hz : {150,300}) for (int d : {-1,1}) {
    LaptopPlayState c;
    arc(c,hz,{d>0?.04:2.70,-.7,.4},{d*3.0,0,-2},.80,0,false,.2);
    EXPECT_EQ(c.state(),d<0?PlayState::INVALID:PlayState::SERVE) << hz << ' ' << d << ' ' << c.reason();
    if(d<0) EXPECT_EQ(c.retained_samples(),0U);
  }
}

TEST(LaptopPlayState, SecondReceiverBounceInvalidatesIncomingButRetainsVisibleEgo) {
  for (int hz : {150,300}) {
    LaptopPlayState out, in;
    out.set_hit_plane(.15); in.set_hit_plane(.15);
    arc(out,hz,{.3,-.7,.6},{4,0,.5},1.15,0,false,.1);
    arc(in,hz,{2.44,-.7,.6},{-4,0,.5},1.15,0,false,.1);
    EXPECT_EQ(out.state(),PlayState::RALLY) << out.reason();
    EXPECT_FALSE(out.may_forward());
    EXPECT_EQ(in.state(),PlayState::INVALID) << in.reason();
  }
}

TEST(LaptopPlayState, OnlyOpponentServeRejectsThirdBounceBeforeEndpoint) {
  for (int hz : {150,300}) for (int d : {-1,1}) {
    LaptopPlayState c; c.set_hit_plane(0.0);
    arc(c,hz,{d>0?.04:2.70,-.7,.4},{d*3.0,0,-2},.78,0,false,1,.1);
    EXPECT_EQ(c.state(),d>0?PlayState::SERVE:PlayState::OPPONENT_SERVE) << c.reason();
    EXPECT_EQ(c.striker_bounces(),1); EXPECT_EQ(c.receiver_bounces(),1);
    LaptopPlayState later; later.set_hit_plane(0.0);
    arc(later,hz,{d>0?.04:2.70,-.7,.4},{d*3.0,0,-2},1.4,0,false,1,.1);
    EXPECT_EQ(later.state(),d<0?PlayState::INVALID:PlayState::SERVE) << later.reason();
  }
}

TEST(LaptopPlayState, EgoServeRetainsLabelWithoutFirstBounceWhileVisible) {
  LaptopPlayState c;
  c.local_intent("SERVE");
  arc(c,300,{.3,-.7,.6},{5,0,.5},.32);
  EXPECT_EQ(c.state(),PlayState::SERVE) << c.reason();
  EXPECT_FALSE(c.may_forward());
}

TEST(LaptopPlayState, EarlyContactBeforeLaunchConfirmationIsNotLost) {
  for (int hz : {150,300}) {
    LaptopPlayState c;
    arc(c,hz,{2.7,-.7,.08},{-3,0,-2},.18);
    EXPECT_EQ(c.state(),PlayState::OPPONENT_SERVE) << hz << ' ' << c.reason();
    EXPECT_EQ(c.striker_bounces(),1);
  }
}

TEST(LaptopPlayState, HeldSamplesDoNotEnterTheNextShotQueue) {
  LaptopPlayState c;
  for (int i=0;i<60;++i) c.observe(sample(i/300., {2.7,-.7,.6}));
  arc(c,300,{2.7,-.7,.6},{-3,0,.2},.14,.2);
  ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY);
  EXPECT_GE(c.boundary_time(),10.2);
  for (const auto& s : c.samples()) EXPECT_GE(s.source_time_s,10.2);
}

TEST(LaptopPlayState, RejectedShotSamplesDoNotEnterAnOppositeReturn) {
  LaptopPlayState c;
  arc(c,300,{2.0,-.7,.6},{-6,0,.5},.2);
  ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY);
  c.invalidate("test_rejected_shot");
  arc(c,300,{.8,-.7,.5038},{3,0,.3},.16,.203);
  ASSERT_EQ(c.state(),PlayState::RALLY) << c.reason();
  EXPECT_GE(c.boundary_time(),10.203);
  for (const auto& s : c.samples()) EXPECT_GE(s.source_time_s,10.203);
}


// Piecewise physical source trace with actual serve bounces, then a racket
// velocity change. End coordinates may be outside the table at playable height.
struct TraceBall { Vec3 p, v; double t=0; int contacts=0; };
void trace_until(LaptopPlayState& c, TraceBall& b, int hz, double until,
                 double after_first_bounce_vx=1.0, bool p2=false) {
  const double dt=1./hz;
  while(b.t<until-1e-8) {
    const Vec3 raw=p2?Vec3(2.74-b.p.x(),-1.525-b.p.y(),b.p.z()):b.p;
    const Vec3 canonical=p2?Vec3(2.74-raw.x(),-1.525-raw.y(),raw.z()):raw;
    c.observe(sample(b.t,canonical));
    double remaining=dt;
    if(b.p.z()+b.v.z()*dt-4.905*dt*dt<.02 && b.v.z()<0 &&
        b.p.x()>=0 && b.p.x()<=2.74) {
      const double contact=(b.v.z()+std::sqrt(b.v.z()*b.v.z()+19.62*(b.p.z()-.02)))/9.81;
      b.p+=b.v*contact+Vec3(0,0,-4.905*contact*contact);
      b.v.z()=(b.v.z()-9.81*contact)*-.9;
      if(++b.contacts==1)b.v.x()*=after_first_bounce_vx;
      b.p.z()=.02;remaining-=contact;
    }
    b.p+=b.v*remaining+Vec3(0,0,-4.905*remaining*remaining);
    b.v.z()-=9.81*remaining;b.t+=dt;
  }
}

TEST(LaptopPlayState, VisibleEgoFlightsSurviveOutsideAndBadSamplesUntilDisappearance) {
  for (int hz : {150,300}) for (bool serve : {false,true}) {
    LaptopPlayState c;
    if(serve)c.local_intent("SERVE");
    arc(c,hz,{.3,-.7,.6},{5,0,.5},.16);
    const auto label = serve?PlayState::SERVE:PlayState::RALLY;
    ASSERT_EQ(c.state(),label);
    const auto generation=c.generation();
    // SERVE may remain held. A moving RALLY remains valid outside the table
    // and below tabletop; its separately measured low-speed rule still applies.
    for(int i=1;i<=hz;++i) {
      double t=.16+i/double(hz);
      Vec3 p(1.1,-.7,.554432);
      if(!serve)p.x()+=.2*i/double(hz);
      if(i>hz/2) {const double a=(i-hz/2)/double(hz);p.y()-=2*a;p.z()-=2*a;}
      c.observe(sample(t,p));c.poll(std::llround((10+t)*1e9));
      ASSERT_EQ(c.state(),label) << c.reason(); ASSERT_FALSE(c.may_forward());
    }
    EXPECT_GE(c.generation(),generation);
    const auto last=std::llround((10+1.16)*1e9);
    for(int ms:{20,40,60,80}) {
      auto bad=sample(1.16+ms/1000.,{0,0,0});bad.position.x()=NAN;
      c.observe(bad);c.input_unavailable("ball_pose_missing",last+ms*1'000'000LL);
      EXPECT_EQ(c.state(),label);
    }
    c.poll(last+101'000'000LL);
    EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_EQ(c.retained_samples(),0U);
  }
}

TEST(LaptopPlayState, EgoReturnNeedsIncomingInBoundsButNotOutgoingBounceProof) {
  for(int hz:{150,300}) for(bool p2:{false,true}) for(bool serve:{false,true}) {
    LaptopPlayState c;
    if(serve)c.local_intent("SERVE");
    TraceBall b{{2.40,-.7,.6},{3.,0,.5}};
    trace_until(c,b,hz,.20,1,p2);
    ASSERT_EQ(c.state(),serve?PlayState::SERVE:PlayState::RALLY);
    ASSERT_EQ(c.receiver_bounces(),0); // No outgoing bounce proof required.
    auto generation=c.generation();const double turn_time=10+b.t;
    b.v={-4,0,1.};
    trace_until(c,b,hz,.25,1,p2); // x still beyond opponent's end.
    EXPECT_TRUE(c.ego_flight());EXPECT_FALSE(c.may_forward());
    trace_until(c,b,hz,.31,1,p2); // Confirmed -X motion is now in table bounds.
    EXPECT_EQ(c.state(),PlayState::OPPONENT_RALLY) << c.reason();
    EXPECT_TRUE(c.return_confirmed());EXPECT_TRUE(c.may_forward());
    EXPECT_GT(c.generation(),generation);
    EXPECT_GE(c.boundary_time(),turn_time-1./hz);
    EXPECT_EQ(c.striker_bounces(),0);EXPECT_EQ(c.receiver_bounces(),0);
    for(const auto& s:c.samples())EXPECT_GE(s.source_time_s,turn_time-1./hz);
  }
}

TEST(LaptopPlayState, InTableIncomingHandoverDoesNotFabricateMissingPreNetHistory) {
  for(int hz:{150,300}) for(bool serve:{false,true}) {
    LaptopPlayState c;
    if(serve)c.local_intent("SERVE");
    TraceBall b{{.3,-.7,.6},{5.,0,.5}};
    trace_until(c,b,hz,.10);b.v={-4,0,.5};
    trace_until(c,b,hz,.20);
    EXPECT_EQ(c.state(),PlayState::OPPONENT_RALLY) << c.reason();
    EXPECT_FALSE(c.history_known());EXPECT_TRUE(c.return_confirmed());
    EXPECT_TRUE(c.may_forward());EXPECT_EQ(c.receiver_bounces(),0);
  }
}

TEST(LaptopPlayState, HeldEgoBallMovedByHandIsNotAnIncomingFlight) {
  LaptopPlayState c;arc(c,300,{.3,-.7,.6},{5,0,.5},.16);
  for(int i=1;i<=90;++i)c.observe(sample(.16+i/300.,{1.1,-.7,.554432}));
  for(int i=1;i<=60;++i)c.observe(sample(.46+i/300.,{1.1-i/300.,-.7,.554432}));
  EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_FALSE(c.may_forward());
  EXPECT_EQ(c.retained_samples(),0U);
}

TEST(LaptopPlayState, HeldServeCannotAutomaticallyBecomeOpponentServeButManualReadyCan) {
  for(int hz:{150,300}) for(bool p2:{false,true}) for(bool ui_serve:{false,true}) {
    LaptopPlayState c;
    if(ui_serve)c.local_intent("SERVE");
    TraceBall b{{.04,-.7,.4},{3.,0,-2.}};
    trace_until(c,b,hz,.90,1,p2);
    ASSERT_EQ(c.state(),PlayState::SERVE);ASSERT_FALSE(c.serve_armed());
    for(int i=0;i<hz;++i) {c.observe(sample(b.t,b.p));b.t+=1./hz;}
    EXPECT_EQ(c.reason(),"serve_held_waiting_for_local_intent");
    const auto generation=c.generation();const auto held=b;
    b.v={-3.,0,-2.};b.contacts=0;const double launch=b.t;
    while(b.t<launch+.82-1e-8) {
      trace_until(c,b,hz,b.t+1./hz,1,p2);
      EXPECT_EQ(c.state(),PlayState::SERVE);EXPECT_FALSE(c.may_forward());
      EXPECT_EQ(c.generation(),generation);EXPECT_EQ(c.retained_samples(),0U);
    }
    // A local Ready explicitly authorizes the next opponent serve, without
    // requiring Stand first. It starts a clean generation and clean history.
    const double next=b.t+.01;c.local_intent("READY");
    b=held;b.t=next;b.v={-3.,0,-2.};b.contacts=0;
    trace_until(c,b,hz,next+.82,1,p2);
    EXPECT_EQ(c.state(),PlayState::OPPONENT_SERVE)<<c.reason();
    EXPECT_TRUE(c.may_forward());EXPECT_FALSE(c.return_confirmed());
    EXPECT_EQ(c.striker_bounces(),1);EXPECT_EQ(c.receiver_bounces(),1);
    for(const auto& s:c.samples())EXPECT_GE(s.source_time_s,10+next-1e-6);
  }
}

TEST(LaptopPlayState, HeldServeInputLossDoesNotAuthorizeAutomaticNewOpponentServe) {
  LaptopPlayState c;c.local_intent("SERVE");TraceBall b{{.04,-.7,.4},{3.,0,-2.}};
  trace_until(c,b,300,.90);
  for(int i=0;i<300;++i){c.observe(sample(b.t,b.p));b.t+=1./300.;}
  c.poll(std::llround((10+b.t+.101)*1e9));ASSERT_EQ(c.state(),PlayState::INVALID);
  b.t+=.2;b.v={-3.,0,-2.};b.contacts=0;trace_until(c,b,300,b.t+.82);
  EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_FALSE(c.may_forward());
  c.local_intent("STOP");EXPECT_EQ(c.state(),PlayState::INVALID);
  c.local_intent("READY");EXPECT_EQ(c.state(),PlayState::OPPONENT_SERVE);
}

TEST(LaptopPlayState, HeldEgoServeMovedByHandDoesNotManufactureNewLaunch) {
  for(int hz:{150,300}) {
    LaptopPlayState c;TraceBall b{{.04,-.7,.4},{3.,0,-2.}};
    trace_until(c,b,hz,.90);
    for(int i=0;i<hz;++i) {c.observe(sample(b.t,b.p));b.t+=1./hz;}
    const auto generation=c.generation();
    for(int i=0;i<hz/3;++i) {
      c.observe(sample(b.t,b.p));b.p.x()-=1./hz;b.t+=1./hz;
      EXPECT_EQ(c.state(),PlayState::SERVE);EXPECT_FALSE(c.may_forward());
      EXPECT_EQ(c.generation(),generation);EXPECT_FALSE(c.return_confirmed());
    }
  }
}

TEST(LaptopPlayState, HeldBallBeforeAcceptedEgoServeLaunchPreservesServeIntent) {
  for(int hz:{150,300}) {
    LaptopPlayState c;c.local_intent("SERVE");
    const auto generation=c.generation();
    for(int i=0;i<hz;++i)c.observe(sample(i/double(hz),{.04,-.7,.4}));
    ASSERT_EQ(c.state(),PlayState::SERVE);ASSERT_TRUE(c.serve_armed());
    ASSERT_EQ(c.generation(),generation);
    arc(c,hz,{.04,-.7,.4},{3.,0,-2.},.19,1.0);
    EXPECT_EQ(c.state(),PlayState::SERVE);EXPECT_FALSE(c.serve_armed());
    EXPECT_FALSE(c.may_forward());EXPECT_FALSE(c.return_confirmed());
    EXPECT_EQ(c.striker_bounces(),1);
  }
}

TEST(LaptopPlayState, SustainedMeasuredSlowRalliesInvalidateAtBothRatesAndPlacements) {
  for(int hz:{150,300}) for(bool p2:{false,true}) for(int direction:{-1,1})
      for(double speed:{0.,.08,.099}) {
    SCOPED_TRACE(::testing::Message()<<hz<<" P2="<<p2<<" direction="<<direction<<" speed="<<speed);
    LaptopPlayState c;
    arc(c,hz,{direction>0?.3:2.44,-.7,1.},{direction*3.,0,.4},.12,0,p2);
    const auto expected=direction>0?PlayState::RALLY:PlayState::OPPONENT_RALLY;
    ASSERT_EQ(c.state(),expected);
    const auto generation=c.generation();const Vec3 p=c.samples().back().position;
    double invalid_at=0;
    for(int i=1;i<=std::lround(.24*hz);++i) {
      const double t=i/double(hz);const Vec3 position=p+Vec3(direction*speed*t,0,0);
      const Vec3 raw=p2?Vec3(2.74-position.x(),-1.525-position.y(),position.z()):position;
      const Vec3 canonical=p2?Vec3(2.74-raw.x(),-1.525-raw.y(),raw.z()):raw;
      c.observe(sample(.12+t,canonical));
      if(c.state()==PlayState::INVALID && invalid_at==0)invalid_at=t;
      if(t<.13)EXPECT_EQ(c.state(),expected); // Never a one-frame rejection.
    }
    EXPECT_GE(invalid_at,.14-1e-8);EXPECT_LE(invalid_at,.14+2./hz);
    EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_FALSE(c.may_forward());
    EXPECT_FALSE(c.sample_usable() && c.may_forward());EXPECT_EQ(c.retained_samples(),0U);
    EXPECT_EQ(c.generation(),generation+1);EXPECT_EQ(c.reason(),"rally_speed_below_threshold");
    // A held-to-launch edge can start a fresh shot, without the invalid history.
    const double restart=.36+1./hz;
    arc(c,hz,p+Vec3(direction*speed*.24,0,0),{direction*3.,0,.4},.12,restart,p2);
    EXPECT_EQ(c.state(),expected);EXPECT_GT(c.generation(),generation+1);
    EXPECT_GT(c.boundary_time(),10+.24);
  }
}

TEST(LaptopPlayState, SlowRuleUsesFullThreeDimensionalSpeedAndStrictThreshold) {
  for(int hz:{150,300}) for(int direction:{-1,1}) for(Vec3 v:{
      Vec3(.10,0,0),Vec3(.101,0,0),Vec3(.03,.2,0),Vec3(.03,0,.2),Vec3(.07,.07,.07)}) {
    LaptopPlayState c;
    arc(c,hz,{direction>0?.3:2.44,-.7,1.},{direction*3.,0,.4},.12);
    const auto state=c.state();const auto generation=c.generation();const Vec3 p=c.samples().back().position;
    v.x()*=direction;
    for(int i=1;i<=std::lround(.30*hz);++i)c.observe(sample(.12+i/double(hz),p+v*(i/double(hz))));
    EXPECT_EQ(c.state(),state)<<hz<<" v="<<v.transpose()<<' '<<c.reason();
    EXPECT_EQ(c.generation(),generation);
  }
}

TEST(LaptopPlayState, NearVerticalApexAndShortSlowIntervalDoNotInvalidate) {
  for(int hz:{150,300}) for(int direction:{-1,1}) {
    LaptopPlayState c;
    arc(c,hz,{direction>0?.3:2.44,-.7,1.},{direction*3.,0,.4},.12);
    const auto state=c.state();const auto generation=c.generation();const Vec3 p=c.samples().back().position;
    // Total speed really falls below 0.10 m/s at the apex, for about 20 ms.
    arc(c,hz,p,{direction*.005,0,.6},.18,.12);
    ASSERT_EQ(c.state(),state)<<c.reason();EXPECT_EQ(c.generation(),generation);
    const Vec3 held=c.samples().back().position;
    for(int i=1;i<=std::lround(.11*hz);++i)c.observe(sample(.30+i/double(hz),held));
    ASSERT_EQ(c.state(),state); // Short stationary interruption, then motion resumes.
    for(int i=1;i<=std::lround(.12*hz);++i)
      c.observe(sample(.41+i/double(hz),held+Vec3(direction*.2*i/hz,0,0)));
    EXPECT_EQ(c.state(),state)<<c.reason();EXPECT_EQ(c.generation(),generation);
  }
}

TEST(LaptopPlayState, SlowConfirmationRestartsAfterGapsAndUnusableObservations) {
  for(int hz:{150,300}) for(bool bad_sample:{false,true}) {
    LaptopPlayState c;arc(c,hz,{.3,-.7,1.},{3.,0,.4},.12);
    Vec3 p=c.samples().back().position;
    // 40 ms warm-up + 70 ms below threshold, insufficient to reject.
    for(int i=1;i<=std::lround(.11*hz);++i)c.observe(sample(.12+i/double(hz),p));
    ASSERT_EQ(c.state(),PlayState::RALLY);
    if(bad_sample){auto bad=sample(.235,{NAN,0,0});c.observe(bad);}
    const double resume=bad_sample?.24:.27; // The other case has a 40 ms gap.
    for(int i=0;i<=std::lround(.10*hz);++i)c.observe(sample(resume+i/double(hz),p));
    EXPECT_EQ(c.state(),PlayState::RALLY)<<c.reason();
    // Duplicates, reordered packets and timer polls cannot create observations.
    const double last=c.latest_time()-10;
    for(int i=0;i<60;++i) {
      c.observe(sample(last,p));c.observe(sample(last-.01,p));
      c.poll(std::llround((10+last+.01)*1e9));
    }
    EXPECT_EQ(c.state(),PlayState::RALLY);
    for(int i=1;i<=std::lround(.08*hz);++i)c.observe(sample(last+i/double(hz),p));
    EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_EQ(c.reason(),"rally_speed_below_threshold");
  }
}

TEST(LaptopPlayState, SlowRuleDoesNotApplyToEitherServeState) {
  for(int hz:{150,300}) for(int direction:{-1,1}) {
    LaptopPlayState c;
    arc(c,hz,{direction>0?.04:2.70,-.7,.4},{direction*3.5,0,-2},.19);
    const auto state=direction>0?PlayState::SERVE:PlayState::OPPONENT_SERVE;
    ASSERT_EQ(c.state(),state);const Vec3 p=c.samples().back().position;
    // 0.08 m/s avoids the distinct 8 mm / 180 ms incoming held-ball detector.
    for(int i=1;i<=std::lround(.3*hz);++i)
      c.observe(sample(.19+i/double(hz),p+Vec3(direction*.08*i/hz,0,0)));
    EXPECT_EQ(c.state(),state)<<c.reason();
  }
}

TEST(LaptopPlayState, SlowSpeedUsesRealEpochTimestampsAndToleratesSamplingJitter) {
  for(int hz:{150,300}) for(bool jitter:{false,true}) {
    LaptopPlayState c;
    auto feed=[&](double t,const Vec3& p) {
      auto s=sample(t,p);s.source_time_ns+=1'790'000'000'000'000'000LL;
      s.source_time_s=s.source_time_ns*1e-9;s.receipt_steady_ns=s.source_time_ns;
      c.observe(s);
    };
    for(int i=0;i<=std::lround(.12*hz);++i) {
      const double t=i/double(hz);
      feed(t,{.3+3*t,-.7,1.+.4*t-4.905*t*t});
    }
    ASSERT_EQ(c.state(),PlayState::RALLY);
    const Vec3 p=c.samples().back().position;
    for(int i=1;i<=std::lround(.21*hz);++i) {
      const double t=i/double(hz)+(jitter?(i%2?.0005:-.0005):0);
      feed(.12+t,p+Vec3(.08*t,0,0));
    }
    EXPECT_EQ(c.state(),PlayState::INVALID)<<hz<<" jitter="<<jitter<<' '<<c.reason();
    EXPECT_EQ(c.reason(),"rally_speed_below_threshold");
  }
}

TEST(LaptopPlayState, LegalServeReturnStartsANewRallyAndCountsOnlyReturnContacts) {
  for(int hz:{150,300}) for(bool p2:{false,true}) for(int d:{-1,1}) {
    LaptopPlayState c; c.set_hit_plane(.35);
    TraceBall b{{d<0?2.70:.04,-.7,.4},{d*3.,0,-2.}};
    trace_until(c,b,hz,.85,1,p2);
    ASSERT_EQ(c.state(),d<0?PlayState::OPPONENT_SERVE:PlayState::SERVE) << c.reason();
    ASSERT_EQ(c.striker_bounces(),1);ASSERT_EQ(c.receiver_bounces(),1);
    const auto generation=c.generation(); const double hit_time=b.t;
    b.v={-d*5.,0,1.6};b.contacts=0;
    trace_until(c,b,hz,hit_time+.50,1,p2);
    EXPECT_EQ(c.state(),d<0?PlayState::RALLY:PlayState::OPPONENT_RALLY) << c.reason();
    EXPECT_GT(c.generation(),generation);
    EXPECT_GE(c.boundary_time(),10+hit_time-1./hz);
    EXPECT_EQ(c.striker_bounces(),0);EXPECT_EQ(c.receiver_bounces(),1);
    EXPECT_EQ(c.may_forward(),d>0);
  }
}

TEST(LaptopPlayState, ServeVolleyCannotBecomeRallyOrRearmOnTheLateReturn) {
  for(int hz:{150,300}) {
    LaptopPlayState c;
    TraceBall b{{2.70,-.7,.4},{-3.,0,-2.}};
    trace_until(c,b,hz,.58);
    ASSERT_EQ(c.state(),PlayState::OPPONENT_SERVE);
    ASSERT_EQ(c.receiver_bounces(),0);
    b.v={5.,0,1.0};b.contacts=0;
    trace_until(c,b,hz,.76);
    EXPECT_EQ(c.state(),PlayState::INVALID) << c.reason();
    EXPECT_FALSE(c.may_forward());EXPECT_EQ(c.retained_samples(),0U);
    trace_until(c,b,hz,.88);
    EXPECT_EQ(c.state(),PlayState::INVALID); // No same-point resurrection.
  }
}

TEST(LaptopPlayState, EgoReturnOnOwnTableRetainsRallyInsteadOfInvalidOrServe) {
  for(int hz:{150,300}) {
    LaptopPlayState c;TraceBall b{{2.70,-.7,.4},{-3.,0,-2.}};
    trace_until(c,b,hz,.85);
    b.v={1.,0,-.4};b.contacts=0;
    trace_until(c,b,hz,1.18);
    EXPECT_EQ(c.state(),PlayState::RALLY) << c.reason();
    EXPECT_GE(c.striker_bounces(),1); // Real evidence is retained, not rejected.
    EXPECT_FALSE(c.may_forward());
  }
}

TEST(LaptopPlayState, SecondEgoReturnBounceOnOpponentTableRetainsVisibleRally) {
  for(int hz:{150,300}) {
    LaptopPlayState c;TraceBall b{{2.70,-.7,.4},{-3.,0,-2.}};
    trace_until(c,b,hz,.85); b.v={5.,0,1.6};b.contacts=0;
    trace_until(c,b,hz,1.9,.05);
    EXPECT_EQ(c.state(),PlayState::RALLY) << c.reason();
    EXPECT_FALSE(c.may_forward());
  }
}

TEST(LaptopPlayState, ServeMissAfterHitPlaneStillInvalidates) {
  for(int hz:{150,300}) {
    LaptopPlayState c;c.set_hit_plane(.35);
    arc(c,hz,{2.70,-.7,.4},{-3.,0,-2},1.4,0,false,1,.1);
    EXPECT_EQ(c.state(),PlayState::INVALID) << c.reason();
    EXPECT_EQ(c.retained_samples(),0U);EXPECT_FALSE(c.may_forward());
    // Missing return with no second table bounce: ball passes below tabletop
    // outside the end line. No assumption that crossing the hit plane is a hit.
    LaptopPlayState miss;miss.set_hit_plane(.35);
    arc(miss,hz,{2.70,-.7,.4},{-3.,0,-2},1.35);
    EXPECT_EQ(miss.state(),PlayState::INVALID) << miss.reason();
  }
}

TEST(LaptopPlayState, SideExitInvalidatesEvenWithoutAFitOrHitPlane) {
  LaptopPlayState c;arc(c,300,{2.7,-.01,.4},{-3.,0,-2.},.19);
  ASSERT_EQ(c.state(),PlayState::OPPONENT_SERVE);
  c.observe(sample(.2,{2.1,.01,.25}));
  EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_FALSE(c.may_forward());
}

TEST(LaptopPlayState, EgoForecastDoesNotInvalidateOrReclassifyObservedRally) {
  LaptopPlayState c;TraceBall b{{2.70,-.7,.4},{-3.,0,-2.}};
  trace_until(c,b,300,.85);b.v={5.,0,1.6};b.contacts=0;
  trace_until(c,b,300,.97);
  ASSERT_EQ(c.state(),PlayState::RALLY) << c.reason();
  BallState fit;fit.valid=true;fit.source_time_s=c.latest_time();
  fit.position=c.samples().back().position;fit.velocity={1,0,-3};fit.residual_rms_m=.001;
  c.forecast(fit,BallPhysics{},SpinEstimate{});
  EXPECT_EQ(c.state(),PlayState::RALLY) << c.reason();
}


TEST(LaptopPlayState, PredictedSecondEgoBounceCannotInvalidateVisibleRally) {
  LaptopPlayState c;TraceBall b{{2.70,-.7,.4},{-3.,0,-2.}};
  trace_until(c,b,300,.85);b.v={5.,0,1.6};b.contacts=0;
  trace_until(c,b,300,1.35,.05);
  ASSERT_EQ(c.state(),PlayState::RALLY) << c.reason();
  ASSERT_EQ(c.receiver_bounces(),1);
  BallState fit;fit.valid=true;fit.source_time_s=c.latest_time();
  fit.position=c.samples().back().position;fit.velocity=b.v;fit.residual_rms_m=.001;
  c.forecast(fit,BallPhysics{},SpinEstimate{});
  EXPECT_EQ(c.state(),PlayState::RALLY);
  EXPECT_FALSE(c.may_forward());
}

TEST(LaptopPlayState, TableSpinReversalIsNotARacketReturn) {
  for(int hz:{150,300}) {
    LaptopPlayState c;TraceBall b{{2.70,-.7,.4},{-3.,0,-2.}};
    // Reach the receive-side bounce, then simulate a table-induced X reversal.
    while(b.contacts<2)trace_until(c,b,hz,b.t+1./hz);
    b.v.x()=-b.v.x();
    trace_until(c,b,hz,b.t+.14);
    EXPECT_EQ(c.state(),PlayState::INVALID) << c.reason();
  }
}

TEST(LaptopPlayState, LocalReadyWaitsWithoutRobotFeedbackAndDoesNotInventHistory) {
  LaptopPlayState c;c.local_intent("READY");
  ASSERT_EQ(c.state(),PlayState::OPPONENT_SERVE);EXPECT_TRUE(c.receive_armed());
  EXPECT_TRUE(c.may_forward());EXPECT_FALSE(c.history_known());
  c.poll(60'000'000'000LL); // No Runner heartbeat required while waiting for a human.
  EXPECT_TRUE(c.receive_armed());EXPECT_TRUE(c.may_forward());
  c.observe(sample(.001,{2.7,-.7,.4}));EXPECT_TRUE(c.sample_usable());
  EXPECT_EQ(c.striker_bounces(),0);
  c.local_intent("STOP");EXPECT_FALSE(c.may_forward());EXPECT_TRUE(c.local_stopped());
  c.local_intent("READY");EXPECT_TRUE(c.may_forward());EXPECT_FALSE(c.local_stopped());
}

TEST(LaptopPlayState, LocalReadyChecksSamplesAndReturnsToNormalFlightValidation) {
  for(int hz:{150,300}) {
    LaptopPlayState c;c.local_intent("READY");
    c.observe(sample(0,{2.7,-.7,-.2}));
    EXPECT_FALSE(c.sample_usable());EXPECT_TRUE(c.receive_armed());
    arc(c,hz,{2.70,-.7,.4},{-3.5,0,-2},.19,.01);
    EXPECT_EQ(c.state(),PlayState::OPPONENT_SERVE);EXPECT_FALSE(c.receive_armed());
    EXPECT_TRUE(c.history_known());EXPECT_EQ(c.striker_bounces(),1);
    c.poll(10'400'000'000LL);EXPECT_EQ(c.state(),PlayState::INVALID);
  }
}

TEST(LaptopPlayState, LocalStopOverridesEveryFlightUntilExplicitLocalReadyOrServe) {
  for(int variant=0;variant<4;++variant) {
    LaptopPlayState c;
    arc(c,300,{variant%2?2.7:.04,-.7,variant<2?.6:.4},
        {variant%2?-3.5:3.5,0,variant<2?.5:-2},.19);
    EXPECT_NE(c.state(),PlayState::INVALID);
    c.local_intent("STOP");EXPECT_EQ(c.state(),PlayState::INVALID);
    EXPECT_FALSE(c.may_forward());EXPECT_EQ(c.retained_samples(),0U);
    arc(c,300,{2.44,-.7,.6},{-5,0,.5},.16,.22);
    EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_TRUE(c.local_stopped());
    c.local_intent(variant%2?"READY":"SERVE");EXPECT_FALSE(c.local_stopped());
    EXPECT_EQ(c.state(),variant%2?PlayState::OPPONENT_SERVE:PlayState::SERVE);
  }
}

TEST(LaptopPlayState, LocalServeTracksReturnWithoutWaitingForRobotMotionMode) {
  for(int hz:{150,300}) for(bool p2:{false,true}) {
    LaptopPlayState c;c.local_intent("SERVE");TraceBall b{{.04,-.7,.4},{3.,0,-2.}};
    trace_until(c,b,hz,.85,1,p2);ASSERT_EQ(c.state(),PlayState::SERVE);
    b.v={-5.,0,1.6};trace_until(c,b,hz,.97,1,p2);
    EXPECT_EQ(c.state(),PlayState::OPPONENT_RALLY);EXPECT_TRUE(c.may_forward());
    EXPECT_TRUE(c.return_confirmed());
    c.poll(std::llround((10+b.t+.101)*1e9));EXPECT_EQ(c.state(),PlayState::INVALID);
  }
}

TEST(LaptopPlayState, BackwardClockRecoversWithNewHistoryAfterOldInputExpires) {
  for(int hz:{150,300}) {
    LaptopPlayState c;arc(c,hz,{2.5,-.7,1.},{-3,0,.4},.12);
    ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY);
    const auto old_generation=c.generation();const double old_source=c.latest_time();
    bool rebased=false;
    for(int i=1;i<=std::lround(.30*hz);++i) {
      const double t=i/double(hz);
      auto s=sample(-2+t,{2.6-5*t,-.7,1.+.4*t-4.905*t*t});
      s.receipt_steady_ns=10'120'000'000LL+std::llround(t*1e9);
      const bool accepted=c.observe(s);
      if(t<=.1) {EXPECT_FALSE(accepted);EXPECT_DOUBLE_EQ(c.latest_time(),old_source);}
      if(c.reason()=="source_clock_rebased") {
        rebased=true;EXPECT_TRUE(accepted);EXPECT_FALSE(c.may_forward());
        EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_EQ(c.retained_samples(),0U);
      }
    }
    EXPECT_TRUE(rebased);EXPECT_EQ(c.state(),PlayState::OPPONENT_RALLY)<<c.reason();
    EXPECT_TRUE(c.may_forward());EXPECT_GT(c.generation(),old_generation);
    EXPECT_GE(c.boundary_time(),8.1);EXPECT_LT(c.latest_time(),old_source);
    for(const auto& s:c.samples())EXPECT_LT(s.source_time_s,old_source);
  }
}

TEST(LaptopPlayState, DuplicateReorderAndBurstDoNotRebaseOrRenewFreshness) {
  for(int hz:{150,300}) for(int kind:{0,1,2}) {
    LaptopPlayState c;arc(c,hz,{2.5,-.7,1.},{-3,0,.4},.12);
    const double old_source=c.latest_time();
    for(int i=1;i<=40;++i) {
      const double t=i/double(hz);
      const double source=kind==0?.12:kind==1?.1-i*.0001:-2+t;
      auto s=sample(source,{2.1,-.7,.9});
      // A source-time burst delivered in <1 ms cannot prove a running clock.
      s.receipt_steady_ns=10'230'000'000LL+(kind==2?i*10'000LL:std::llround(t*1e9));
      EXPECT_FALSE(c.observe(s));c.poll(s.receipt_steady_ns);
    }
    EXPECT_DOUBLE_EQ(c.latest_time(),old_source);EXPECT_EQ(c.state(),PlayState::INVALID);
    EXPECT_FALSE(c.may_forward());EXPECT_EQ(c.retained_samples(),0U);
  }
}

TEST(LaptopPlayState, ClockRecoveryDoesNotReleaseLocalStopOrInventReady) {
  LaptopPlayState c;c.local_intent("READY");arc(c,300,{2.5,-.7,1.},{-3,0,.4},.12);
  for(int i=1;i<=90;++i) {
    const double t=i/300.;auto s=sample(-2+t,{2.6-5*t,-.7,1.+.4*t-4.905*t*t});
    s.receipt_steady_ns=10'120'000'000LL+std::llround(t*1e9);c.observe(s);
  }
  ASSERT_EQ(c.state(),PlayState::OPPONENT_RALLY);EXPECT_FALSE(c.receive_armed());
  c.local_intent("STOP");
  for(int i=1;i<=90;++i) {
    auto s=sample(-4+i/300.,{2.6-5*i/300.,-.7,1.});
    s.receipt_steady_ns=10'440'000'000LL+i*3'333'333LL;EXPECT_FALSE(c.observe(s));
  }
  EXPECT_EQ(c.state(),PlayState::INVALID);EXPECT_EQ(c.reason(),"local_stop");
  EXPECT_FALSE(c.may_forward());EXPECT_TRUE(c.local_stopped());
}
}  // namespace
}  // namespace hope_planner_cpp
