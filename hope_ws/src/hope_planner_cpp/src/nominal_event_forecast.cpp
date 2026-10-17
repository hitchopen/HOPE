#include "hope_planner_cpp/nominal_event_forecast.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
namespace hope_planner_cpp {
namespace {
Eigen::Matrix3d acceleration_jacobian(const Vec3 &v, const BallPhysics &p) {
  const double n=v.norm();
  if(n<1e-15) return Eigen::Matrix3d::Zero();
  return -p.drag_k*(n*Eigen::Matrix3d::Identity()+v*v.transpose()/n);
}
State6 flow(const State6 &s,const BallPhysics &p) {
  State6 f; f.head<3>()=s.tail<3>();
  f.tail<3>()=p.gravity-p.drag_k*s.tail<3>().norm()*s.tail<3>(); return f;
}
bool covariance_ok(const Matrix6 &p) {
  if(!p.allFinite() || (p-p.transpose()).norm()>1e-9*std::max(1.,p.norm())) return false;
  Eigen::SelfAdjointEigenSolver<Matrix6> e(p);
  return e.info()==Eigen::Success && e.eigenvalues().minCoeff()>=-1e-10*std::max(1.,p.norm());
}
bool footprint(const State6 &s,const TableParams &t) {
  return s[0]>=0 && s[0]<=t.length && s[1]<=t.y_max && s[1]>=t.y_max-t.width;
}
}
const char *quality_name(LinearizationQuality q) {
  switch(q) {
    case LinearizationQuality::Valid:return "valid_linearized";
    case LinearizationQuality::BranchSensitive:return "branch_sensitive";
    case LinearizationQuality::IllConditioned:return "ill_conditioned";
    default:return "unavailable";
  }
}
Matrix6 free_flight_jacobian(const State6 &s,double h,const BallPhysics &p) {
  const Vec3 v=s.tail<3>();
  const Vec3 vm=v+.5*h*(p.gravity-p.drag_k*v.norm()*v);
  const Eigen::Matrix3d m=Eigen::Matrix3d::Identity()+.5*h*acceleration_jacobian(v,p);
  Matrix6 j=Matrix6::Identity(); j.topRightCorner<3,3>()=h*m;
  j.bottomRightCorner<3,3>()=Eigen::Matrix3d::Identity()+h*acceleration_jacobian(vm,p)*m;
  return j;
}
Matrix6 physical_step_jacobian(const State6 &s,double h,const ProbabilisticConfig &c,bool &branch) {
  State6 nominal=s; const auto event=physical_step(nominal,h,c); branch=false;
  if(!event.bounced) return free_flight_jacobian(s,h,c.physics);
  // Differentiate the complete implemented contact step. This includes the
  // contact fraction derivative, reset, and remaining flight exactly once.
  Matrix6 j;
  for(int k=0;k<6;++k) {
    const double eps=1e-6*std::max(1.,std::abs(s[k]));
    State6 a=s,b=s; a[k]+=eps;b[k]-=eps;
    const auto ea=physical_step(a,h,c),eb=physical_step(b,h,c);
    branch=branch || ea.bounced!=event.bounced || eb.bounced!=event.bounced;
    j.col(k)=(a-b)/(2*eps);
  }
  return j;
}
Matrix6 bounce_saltation(const State6 &pre,const ProbabilisticConfig &c) {
  auto reset=[&](State6 s) {
    const double speed=s.segment<2>(3).norm();
    const double impulse=std::min(c.physics.table_tangential_gain*speed,
      c.physics.table_friction_cap_mu*(1+c.physics.restitution_v)*std::abs(s[5]));
    if(speed>1e-12) s.segment<2>(3)*=1-impulse/speed;
    s[5]*=-c.physics.restitution_v; return s;
  };
  if(std::abs(pre[5])<1e-8) return Matrix6::Constant(NAN);
  Matrix6 dr;
  for(int k=0;k<6;++k) {
    State6 a=pre,b=pre;const double eps=1e-6*std::max(1.,std::abs(pre[k]));
    a[k]+=eps;b[k]-=eps;dr.col(k)=(reset(a)-reset(b))/(2*eps);
  }
  Matrix6 salt=dr; salt.col(2)+=(flow(reset(pre),c.physics)-dr*flow(pre,c.physics))/pre[5];
  return salt;
}
Matrix7 spatial_event_covariance(const State6 &s,const Matrix6 &p,int axis,const BallPhysics &physics) {
  Eigen::Matrix<double,7,6> b=Eigen::Matrix<double,7,6>::Zero();
  b.topRows<6>()=Matrix6::Identity();
  b.row(6)[axis]=-1./s[axis+3];
  b.topRows<6>()+=flow(s,physics)*b.row(6);
  Matrix7 result=b*p*b.transpose();return .5*(result+result.transpose()).eval();
}
NominalBallForecast forecast_nominal(const StateBelief &b,const ProbabilisticConfig &c) {
  return forecast_nominal(b,c,ForecastNoiseConfig::inherited(c));
}
NominalBallForecast forecast_nominal(const StateBelief &b,const ProbabilisticConfig &c,const ForecastNoiseConfig &noise_config) {
  c.validate();noise_config.validate();
  NominalBallForecast out;out.source=b;
  if(noise_config.acceleration_spectral_density!=c.acceleration_spectral_density || noise_config.bounce_velocity_std!=c.bounce_velocity_std)
    out.uncertainty_provenance="local_linearization_v1;forecast_noise_v1_experimental;uncalibrated;clock_and_plane_unavailable";
  if(!b.initialized || !b.mean.allFinite()) return out;
  out.available=true;out.termination_reason="horizon";
  State6 state=b.mean;Matrix6 p=b.covariance;
  auto quality=covariance_ok(p)?LinearizationQuality::Valid:LinearizationQuality::Unavailable;
  ForecastCovarianceBudget<6> budget;
  budget.available=noise_config.attribute_covariance && quality!=LinearizationQuality::Unavailable;
  if(budget.available)budget.posterior=p;
  auto transform=[](const ForecastCovarianceBudget<6> &a,const Matrix6 &j) {
    ForecastCovarianceBudget<6> z;z.available=a.available;
    if(a.available) {z.posterior=j*a.posterior*j.transpose();z.free_flight=j*a.free_flight*j.transpose();z.bounce=j*a.bounce*j.transpose();}
    return z;
  };
  bool ground=state[2]<=0;bool table_surface_seen=false;bool net_seen=false;unsigned bounces=0;
  auto waypoint=[&](double t) {
    ForecastWaypoint w;w.dt_s=t;w.state=state;w.quality=quality;w.physical_continuation=!ground;
    if(quality!=LinearizationQuality::Unavailable) w.covariance=p;
    w.covariance_budget=budget;w.covariance_budget.available=budget.available && quality!=LinearizationQuality::Unavailable;
    out.waypoints.push_back(w);
  };
  if(ground) {ForecastPhysicsEvent e;e.kind="ground";e.pre=e.post=state;out.events.push_back(e);}
  waypoint(0);double next_waypoint=c.trajectory_dt_s;
  auto event=[&](const char *kind,double t,const State6 &pre,const State6 &post,
                 const Matrix6 &cov,int axis,LinearizationQuality q,const ForecastCovarianceBudget<6> &terms) {
    ForecastPhysicsEvent e;e.kind=kind;e.dt_s=t;e.pre=pre;e.post=post;e.quality=q;
    if(std::abs(pre[axis+3])<.05 && q!=LinearizationQuality::Unavailable)
      e.quality=LinearizationQuality::IllConditioned;
    if(q!=LinearizationQuality::Unavailable && std::abs(pre[axis+3])>1e-8) {
      e.covariance=spatial_event_covariance(pre,cov,axis,c.physics);
      if(terms.available) {
        e.covariance_budget.available=true;
        e.covariance_budget.posterior=spatial_event_covariance(pre,terms.posterior,axis,c.physics);
        e.covariance_budget.free_flight=spatial_event_covariance(pre,terms.free_flight,axis,c.physics);
        e.covariance_budget.bounce=spatial_event_covariance(pre,terms.bounce,axis,c.physics);
      }
      if(e.covariance.allFinite()) e.time_std_s=std::sqrt(std::max(0.,e.covariance(6,6)));
      else {e.quality=LinearizationQuality::Unavailable;e.covariance_budget.available=false;}
    }
    return e;
  };
  for(double t=0;t<c.horizon_s-1e-12;) {
    const double h=std::min({c.integration_dt_s,c.horizon_s-t,next_waypoint-t});
    const State6 before=state;const Matrix6 prior=p;const auto prior_budget=budget;
    const auto contact=physical_step(state,h,c);
    bool changed=false;const Matrix6 j=physical_step_jacobian(before,h,c,changed);
    if(changed && quality!=LinearizationQuality::Unavailable) quality=LinearizationQuality::BranchSensitive;
    if(quality!=LinearizationQuality::Unavailable) {
      p=j*prior*j.transpose();budget=transform(prior_budget,j);
      if(contact.bounced) {
        const double hc=contact.fraction*h;
        State6 pre=free_flight(before,hc,c.physics);pre[2]=c.physics.radius;
        const Matrix6 salt=bounce_saltation(pre,c);
        Matrix6 noise=Matrix6::Zero();
        if(hc>0 && noise_config.acceleration_spectral_density>0)
          noise=salt*process_covariance(hc,noise_config.acceleration_spectral_density)*salt.transpose();
        noise.bottomRightCorner<3,3>()+=noise_config.bounce_velocity_std.array().square().matrix().asDiagonal();
        const Matrix6 remaining=free_flight_jacobian(contact.contact,h-hc,c.physics);
        p+=remaining*noise*remaining.transpose()+process_covariance(h-hc,noise_config.acceleration_spectral_density);
        if(budget.available) {
          if(hc>0 && noise_config.acceleration_spectral_density>0) budget.free_flight+=remaining*salt*process_covariance(hc,noise_config.acceleration_spectral_density)*salt.transpose()*remaining.transpose();
          budget.free_flight+=process_covariance(h-hc,noise_config.acceleration_spectral_density);
          Matrix6 qb=Matrix6::Zero();qb.bottomRightCorner<3,3>()=noise_config.bounce_velocity_std.array().square().matrix().asDiagonal();
          budget.bounce+=remaining*qb*remaining.transpose();
        }
      } else {
        p+=process_covariance(h,noise_config.acceleration_spectral_density);
        if(budget.available)budget.free_flight+=process_covariance(h,noise_config.acceleration_spectral_density);
      }
      p=.5*(p+p.transpose()).eval();
      if(!p.allFinite()) quality=LinearizationQuality::Unavailable;
    }
    auto segment=[&](const State6 &a,const State6 &z,double start,double duration,const Matrix6 &start_cov,const ForecastCovarianceBudget<6> &start_terms) {
      if(duration<=0) return;
      if(!net_seen && (a[0]-c.table.net_x)*(z[0]-c.table.net_x)<=0 && a[0]!=z[0]) {
        const double frac=(a[0]-c.table.net_x)/(a[0]-z[0]);
        if(frac>0 && frac<=1) {
          State6 hit=a+frac*(z-a);hit[0]=c.table.net_x;
          const auto jac=free_flight_jacobian(a,frac*duration,c.physics);
          auto terms=transform(start_terms,jac);if(terms.available)terms.free_flight+=process_covariance(frac*duration,noise_config.acceleration_spectral_density);
          auto e=event("net",start+frac*duration,hit,hit,
            jac*start_cov*jac.transpose()+process_covariance(frac*duration,noise_config.acceleration_spectral_density),0,quality,terms);
          if(out.events.size()<32)out.events.push_back(e);
          net_seen=true;
        }
      }
      for(int axis:{0,2}) {
        const double plane=axis==0?c.x_hit:0.;
        if(!(a[axis]>plane && z[axis]<=plane)) continue;
        const double fraction=(a[axis]-plane)/(a[axis]-z[axis]);
        const double dt=fraction*duration;
        State6 hit=a+fraction*(z-a);hit[axis]=plane;
        const auto fj=free_flight_jacobian(a,dt,c.physics);
        const Matrix6 cp=fj*start_cov*fj.transpose()+process_covariance(dt,noise_config.acceleration_spectral_density);
        auto terms=transform(start_terms,fj);if(terms.available)terms.free_flight+=process_covariance(dt,noise_config.acceleration_spectral_density);
        auto e=event(axis==0?"hit_plane":"ground",start+dt,hit,hit,cp,axis,quality,terms);
        if(axis==0 && !out.hit_plane && e.dt_s>0 && hit[3]<0) {
          e.physically_usable=hit[2]>0 && !ground;out.hit_plane=e;
        }
        if(axis==2 && !ground) { ground=true;if(out.events.size()<32)out.events.push_back(e); }
      }
    };
    if(contact.bounced) {
      const double hc=contact.fraction*h;
      if(quality==LinearizationQuality::Valid && (hc<=1e-12 || std::abs(contact.contact[5])<.05))
        quality=LinearizationQuality::IllConditioned;
      State6 pre=free_flight(before,hc,c.physics);pre[2]=c.physics.radius;
      const Matrix6 jf=free_flight_jacobian(before,hc,c.physics);
      Matrix6 pc=jf*prior*jf.transpose()+process_covariance(hc,noise_config.acceleration_spectral_density);
      if(quality!=LinearizationQuality::Unavailable) {
        const auto ep=spatial_event_covariance(pre,pc,2,c.physics);
        const double sx=3*std::sqrt(std::max(0.,ep(0,0))),sy=3*std::sqrt(std::max(0.,ep(1,1)));
        if(pre[0]-sx<=0 || pre[0]+sx>=c.table.length || pre[1]+sy>=c.table.y_max || pre[1]-sy<=c.table.y_max-c.table.width)
          quality=LinearizationQuality::BranchSensitive;
      }
      auto contact_terms=transform(prior_budget,jf);if(contact_terms.available)contact_terms.free_flight+=process_covariance(hc,noise_config.acceleration_spectral_density);
      segment(before,pre,t,hc,prior,prior_budget);
      auto e=event("table_bounce",t+hc,pre,contact.contact,pc,2,quality,contact_terms);
      if(out.events.size()<32)out.events.push_back(e);
      // For events after impact use reset/saltation covariance at the contact.
      const Matrix6 salt=bounce_saltation(pre,c);Matrix6 post_cov=Matrix6::Zero();
      if(pc.norm()>0)post_cov=salt*pc*salt.transpose();
      post_cov.bottomRightCorner<3,3>()+=noise_config.bounce_velocity_std.array().square().matrix().asDiagonal();
      auto post_terms=transform(contact_terms,salt);
      if(post_terms.available)post_terms.bounce.bottomRightCorner<3,3>()+=noise_config.bounce_velocity_std.array().square().matrix().asDiagonal();
      segment(contact.contact,state,t+hc,h-hc,post_cov,post_terms);
      table_surface_seen=false;
      if(++bounces>=c.max_rollout_bounces) {out.termination_reason="bounce_limit";break;}
    } else {
      if(!table_surface_seen && before[5]<0 && before[2]>=c.physics.radius && state[2]<=c.physics.radius) {
        const double fraction=(before[2]-c.physics.radius)/(before[2]-state[2]);
        State6 miss=free_flight(before,fraction*h,c.physics);miss[2]=c.physics.radius;
        if(!footprint(miss,c.table)) {
          const auto jm=free_flight_jacobian(before,fraction*h,c.physics);
          Matrix6 pm=jm*prior*jm.transpose()+process_covariance(fraction*h,noise_config.acceleration_spectral_density);
          if(quality!=LinearizationQuality::Unavailable && std::abs(miss[5])>1e-8) {
            const auto ep=spatial_event_covariance(miss,pm,2,c.physics);
            const double sx=3*std::sqrt(std::max(0.,ep(0,0))),sy=3*std::sqrt(std::max(0.,ep(1,1)));
            if(miss[0]+sx>=0 && miss[0]-sx<=c.table.length && miss[1]-sy<=c.table.y_max && miss[1]+sy>=c.table.y_max-c.table.width)
              quality=LinearizationQuality::BranchSensitive;
          }
          auto miss_terms=transform(prior_budget,jm);if(miss_terms.available)miss_terms.free_flight+=process_covariance(fraction*h,noise_config.acceleration_spectral_density);
          auto e=event("table_miss",t+fraction*h,miss,miss,pm,2,quality,miss_terms);
          if(out.events.size()<32)out.events.push_back(e);
        }
        table_surface_seen=true;
      }
      segment(before,state,t,h,prior,prior_budget);
    }
    t+=h;
    if(!state.allFinite()) {out.termination_reason="nonfinite_physics";break;}
    if(t+1e-10>=next_waypoint || t+1e-10>=c.horizon_s) {
      waypoint(t);next_waypoint+=c.trajectory_dt_s;
      if(out.waypoints.size()>=384) {out.termination_reason="waypoint_limit";break;}
    }
  }
  // Ground may precede plane within a single step; use times, not loop order.
  if(out.hit_plane) for(const auto &e:out.events)
    if(e.kind=="ground" && e.dt_s<=out.hit_plane->dt_s)out.hit_plane->physically_usable=false;
  if(out.hit_plane) {
    for(const auto &e:out.events)if(e.kind=="table_bounce") {
      const bool exact_tie=std::abs(e.dt_s-out.hit_plane->dt_s)<=1e-9;
      const bool uncertain_order=std::isfinite(e.covariance(0,0)) &&
        std::abs(e.pre[0]-c.x_hit)<=3*std::sqrt(std::max(0.,e.covariance(0,0)));
      if((exact_tie || uncertain_order) && out.hit_plane->quality!=LinearizationQuality::Unavailable)
        out.hit_plane->quality=LinearizationQuality::BranchSensitive;
    }
    if(out.events.size()<32)out.events.push_back(*out.hit_plane);
  }
  std::stable_sort(out.events.begin(),out.events.end(),[](const auto&a,const auto&z){return a.dt_s<z.dt_s;});
  return out;
}
} // namespace hope_planner_cpp
