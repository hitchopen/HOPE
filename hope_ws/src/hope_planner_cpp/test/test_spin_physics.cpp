#include "hope_planner_cpp/ball_dynamics.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/future_contact_predictor.hpp"
#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {
// Independent small-step RK4 reference, including analytically integrated
// constant world angular velocity. No estimator-generated labels.
void reference_step(Vec3& x, Vec3& v, const Vec3& w, const BallPhysics& p, double h) {
  auto a = [&](const Vec3& u) -> Vec3 {
    return Vec3(0., 0., -9.81) - p.drag_k * u.norm() * u + p.magnus_k * w.cross(u);
  };
  const Vec3 a1 = a(v), v2 = v + .5*h*a1, a2 = a(v2);
  const Vec3 v3 = v + .5*h*a2, a3 = a(v3), v4 = v + h*a3, a4 = a(v4);
  x += h/6. * (v + 2.*v2 + 2.*v3 + v4);
  v += h/6. * (a1 + 2.*a2 + 2.*a3 + a4);
}

TEST(SpinPhysics, ObservedMagnusAndFuturePredictionUseOneSpinState) {
  BallPhysics p;
  EstimatorConfig c; c.spin_enabled = true;
  SpinEstimatorConfig sc; sc.window_s = c.window_s;
  BatchPhysicsEstimator estimator(p, c, sc);
  Vec3 x(1.8, -.7, 1.0), v(-4., .3, .7), w(20., -35., 10.);
  for (int i = 0; i <= 65; ++i) {
    const double t = i/360.;
    BallSample s; s.source_time_s = 20.+t; s.position = x;
    s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(w.norm()*t, w.normalized()));
    s.orientation_valid = true; estimator.push(s);
    if (i < 65) for (int j=0;j<10;++j) reference_step(x,v,w,p,1./3600.);
  }
  const auto state = estimator.estimate();
  ASSERT_TRUE(state.valid); ASSERT_TRUE(state.physics_spin.valid);
  EXPECT_LT((state.physics_spin.omega_rad_s-w).norm(),1e-8);
  EXPECT_LT((state.position-x).norm(),.002);
  EXPECT_LT((state.velocity-v).norm(),.02);
  FutureContactConfig fc; fc.spin_enabled=true; fc.magnus_enabled=true;
  fc.tangential_gain=p.table_tangential_gain; fc.omega_scale=1.;
  FutureContactPredictor predictor(p,{}, {},fc);
  SpinEstimate unrelated; unrelated.valid=true; unrelated.coherence=1.;
  unrelated.omega_rad_s=Vec3(70.,70.,70.);
  auto predicted=predictor.predict(state,.6,unrelated);
  ASSERT_TRUE(predicted.valid);
  Vec3 prev=x; double t=state.source_time_s;
  while(x.x()>.6){prev=x;reference_step(x,v,w,p,.0001);t+=.0001;}
  const double f=(prev.x()-.6)/(prev.x()-x.x());
  Vec3 expected=prev+f*(x-prev);
  EXPECT_LT((predicted.ball_position-expected).norm(),.005);
  EXPECT_NEAR(predicted.strike_source_time_s,t-.0001+f*.0001,.002);
}

TEST(SpinPhysics, MissingOrientationKeepsPositionFitAndResetsSpinBetweenFlights) {
  BallPhysics p; EstimatorConfig c; c.spin_enabled=true;
  BatchPhysicsEstimator estimator(p,c);
  Vec3 x(1.8,-.7,1.),v(-3.,0.,.3);
  for(int i=0;i<50;++i){BallSample s;s.source_time_s=10.+i/360.;s.position=x;
    estimator.push(s);reference_step(x,v,Vec3::Zero(),p,1./360.);}
  auto state=estimator.estimate(); ASSERT_TRUE(state.valid);
  EXPECT_FALSE(state.physics_spin.valid); EXPECT_TRUE(state.physics_spin.omega_rad_s.isZero());
  estimator.reset();EXPECT_FALSE(estimator.estimate().valid);
}

TEST(SpinPhysics, TableImpulseRespectsFrictionCapAndAngularMomentum) {
  BallPhysics p; Vec3 v(-5.,.3,-.4),w(8.,45.,3.);const Vec3 before=v,spin=w;
  table_contact(p,v,w);const Vec3 dv=v-before;
  EXPECT_NEAR(dv.head<2>().norm(),p.table_friction_cap_mu*(1+p.restitution_v)*.4,1e-12);
  EXPECT_NEAR(v.z(),.4*p.restitution_v,1e-12);
  EXPECT_LT((w-spin-Vec3(dv.y(),-dv.x(),0.)/((2./3.)*p.radius)).norm(),1e-12);
  EXPECT_DOUBLE_EQ(w.z(),spin.z());
}
}  // namespace
}  // namespace hope_planner_cpp
