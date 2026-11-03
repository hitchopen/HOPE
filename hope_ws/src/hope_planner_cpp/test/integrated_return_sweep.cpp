#include "hope_planner_cpp/validated_return_planner.hpp"
#include <iomanip>
#include <iostream>
using namespace hope_planner_cpp;
int main() {
  ValidatedReturnPlanner planner({});
  std::cout<<std::setprecision(17);
  std::cout<<"z,y,vx,vz,valid,reason,duration,rx,ry,rz,nx,ny,nz,voutx,vouty,voutz,lx,ly,lz,lt\n";
  for (double z:{.15,.35,.6,.9}) for(double y:{-1.1,-.7,-.3})
    for(double vx:{-2.,-3.,-5.,-7.}) for(double vz:{-.5,1.,2.}) {
      ContactPlanningInput in;in.contact.pre.position=Vec3(.15,y,z);in.contact.pre.velocity=Vec3(vx,.2,vz);
      auto r=planner.solve(in);
      std::cout<<z<<','<<y<<','<<vx<<','<<vz<<','<<r.command.valid<<','<<r.reason<<','<<r.selected_duration_s;
      for (const auto &v:{r.command.velocity,r.command.normal,r.reconstructed.velocity,r.first_landing})
        std::cout<<','<<v.x()<<','<<v.y()<<','<<v.z();
      std::cout<<','<<r.landing_time_s<<'\n';
    }
}
