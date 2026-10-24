// SI units. Constant world spin, RK4 free-flight shooting for calibration only.
#include <cmath>
static void acc(const double* v,const double* w,double kd,double km,double* a){
 double s=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
 a[0]=-kd*s*v[0]+km*(w[1]*v[2]-w[2]*v[1]);
 a[1]=-kd*s*v[1]+km*(w[2]*v[0]-w[0]*v[2]);
 a[2]=-9.81-kd*s*v[2]+km*(w[0]*v[1]-w[1]*v[0]);
}
extern "C" void shoot(int n,const int* lens,const double* states,const double* ws,double kd,double km,double dt,double* out){
 int k=0;
 for(int j=0;j<n;j++){
  double p[3],v[3];for(int c=0;c<3;c++){p[c]=states[6*j+c];v[c]=states[6*j+3+c];}
  const double* w=ws+3*j;
  for(int i=0;i<lens[j];i++){
   for(int c=0;c<3;c++)out[3*k+c]=p[c];k++;
   double a[3],b[3],c[3],d[3],v2[3],v3[3],v4[3];acc(v,w,kd,km,a);
   for(int h=0;h<3;h++)v2[h]=v[h]+dt*.5*a[h];acc(v2,w,kd,km,b);
   for(int h=0;h<3;h++)v3[h]=v[h]+dt*.5*b[h];acc(v3,w,kd,km,c);
   for(int h=0;h<3;h++)v4[h]=v[h]+dt*c[h];acc(v4,w,kd,km,d);
   for(int h=0;h<3;h++){p[h]+=dt/6*(v[h]+2*v2[h]+2*v3[h]+v4[h]);v[h]+=dt/6*(a[h]+2*b[h]+2*c[h]+d[h]);}
  }
 }
}
