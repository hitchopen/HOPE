#include "vrpn_mocap/pose_timing.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
void check(bool ok, const char *why) { if (!ok) { std::cerr << why << "\n"; std::exit(1); } }
bool near(double a, double b, double tol=1e-6) { return std::abs(a-b) < tol; }
int main() {
  vrpn_mocap::PoseTiming timing;
  const std::int64_t epoch = 1793736000000000000LL;
  for (int i=0;i<61;++i) {
    const std::int64_t ns = i * 3333333LL;
    const std::int64_t server_us = (epoch+ns)/1000+440000;
    timing.Receive(server_us/1000000,server_us%1000000,epoch+ns,1000000000+ns);
    timing.Published(1000000000+ns+200000);
  }
  auto a=timing.Snapshot(14);
  check(a[0]==1 && a[1]==14 && a[2]==61 && a[3]==61,"sample identity/count");
  check(near(a[5],300,0.001) && near(a[19],300,0.01),"actual rate uses all intervals");
  check(near(a[11],-440) && near(a[12],-440) && near(a[13],-440),"future raw stamps remain signed/unqualified");
  check(near(a[14],3.333333) && near(a[20],0.2),"gap/publish duration");
  check(a[21]==0,"no inferred clock qualification");
  timing.Receive(1793736000,740000,epoch+300000000,1300000000);
  a=timing.Snapshot(14);
  check(a[3]==1 && a[2]==62 && near(a[14],100.00002),"gap across summary boundaries survives");
  check(near(a[20],0),"window publish max resets");
  timing.Receive(1793736000,740000,epoch+310000000,1310000000);
  timing.Receive(1793736000,730000,epoch+320000000,1320000000);
  timing.Receive(0,0,epoch+330000000,1330000000);
  a=timing.Snapshot(14);
  check(a[16]==2 && a[17]==1 && std::isnan(a[11]),"duplicate/backward/invalid source events");
  check(a[18]==0 && a[19]==0,"nonincreasing stamps cannot inflate source rate");
  timing.Receive(0,-1,epoch+340000000,1340000000);
  a=timing.Snapshot(14);
  check(std::isnan(a[12]) && std::isnan(a[13]),"invalid-only window has no fabricated delta");
  std::cout << "pose timing checks passed\n";
}
