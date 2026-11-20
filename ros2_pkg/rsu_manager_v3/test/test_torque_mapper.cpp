#include <gtest/gtest.h>
#include <limits>
#include "rsu_manager_v3/torque_mapper.hpp"
#include "rsu_manager_v2/rsu_lut.hpp"
using namespace rsu_manager_v3;
TEST(Torque, InverseTransposeAndPowerAcrossWorkspace) {
  rsu_manager_v2::RsuLut lut; lut.load(RSU_TEST_LUT);
  TorqueConfig c; c.motor_limit=1000;c.virtual_limit=1000;c.slew_rate=1e9;
  for(int r=0;r<=70;++r) for(int p=0;p<=150;++p) {
    const double q0=lut.roll_min()+(lut.roll_max()-lut.roll_min())*r/70;
    const double q1=lut.pitch_min()+(lut.pitch_max()-lut.pitch_min())*p/150;
    const auto a=lut.query(q0,q1).jacobian;
    for (double sign : {-1.,1.}) {
      const std::array<double,2> q{sign*q0,sign*q1},v{.12,-.08};
      const auto result=map_torque(a,q,v,{q[0]+.02,q[1]-.03},{0,0},.005,c);
      ASSERT_TRUE(result.valid);
      const auto& m=result.applied; const auto& t=result.virtual_torque;
      EXPECT_NEAR(a[0]*m[0]+a[2]*m[1],t[0],1e-11);
      EXPECT_NEAR(a[1]*m[0]+a[3]*m[1],t[1],1e-11);
      const std::array<double,2> av{a[0]*v[0]+a[1]*v[1],a[2]*v[0]+a[3]*v[1]};
      EXPECT_NEAR(m[0]*av[0]+m[1]*av[1],t[0]*v[0]+t[1]*v[1],1e-11);
    }
  }
}
TEST(Torque, StaticDemandDoesNotDecay) {
 TorqueConfig c; std::array<double,2> prev{};
 for(int n=0;n<2000;++n) {
   auto o=map_torque({2,1,-1,3},{0,0},{0,0},{.1,-.1},prev,.005,c);
   ASSERT_TRUE(o.valid); prev=o.applied;
   if(n>5) {EXPECT_NEAR(prev[0],1.1107142857142858,1e-10);EXPECT_NEAR(prev[1],-1.2035714285714285,1e-10);}
 }
}
TEST(Torque, LimitsSlewAndInvalid) {
 TorqueConfig c;c.virtual_limit=1.;c.motor_limit=.5;c.slew_rate=10.;
 auto o=map_torque({1,0,0,1},{0,0},{0,0},{1,-1},{0,0},.005,c);
 ASSERT_TRUE(o.valid);EXPECT_TRUE(o.saturated);EXPECT_DOUBLE_EQ(o.applied[0],.05);
 EXPECT_DOUBLE_EQ(o.applied[1],-.05);
 EXPECT_FALSE(map_torque({1,1,1,1},{0,0},{0,0},{0,0},{0,0},.005,c).valid);
 EXPECT_FALSE(map_torque({1,0,0,1},{NAN,0},{0,0},{0,0},{0,0},.005,c).valid);
 EXPECT_FALSE(map_torque({1,0,0,1},{0,0},{0,0},{0,0},{0,0},.2,c).valid);
 c.motor_limit=NAN;EXPECT_THROW(c.validate(),std::invalid_argument);
}
TEST(Torque, LutJacobianMatchesPositionFiniteDifference) {
 rsu_manager_v2::RsuLut lut;lut.load(RSU_TEST_LUT);
 for(int r=1;r<20;++r)for(int p=1;p<30;++p) {
  double x=lut.roll_min()+(lut.roll_max()-lut.roll_min())*r/20;
  double y=lut.pitch_min()+(lut.pitch_max()-lut.pitch_min())*p/30;
  auto a=lut.query(x,y).jacobian;
  const double eps=1e-5;
  auto xp=lut.query(x+eps,y),xm=lut.query(x-eps,y);
  auto yp=lut.query(x,y+eps),ym=lut.query(x,y-eps);
  for(int i=0;i<2;++i) {
   EXPECT_NEAR((xp.alpha[i]-xm.alpha[i])/(2*eps),a[2*i],.025);
   EXPECT_NEAR((yp.alpha[i]-ym.alpha[i])/(2*eps),a[2*i+1],.025);
  }
 }
}
TEST(Torque, ClosedLoopConstantRampSineStepAndLoad) {
 rsu_manager_v2::RsuLut lut;lut.load(RSU_TEST_LUT);TorqueConfig c;
 for(int profile=0;profile<4;++profile) {
  std::array<double,2> q{0,-.4},v{},prev{};double error2=0;int count=0;
  for(int n=0;n<4000;++n) {
   double t=n*.005;
   std::array<double,2> target{.02,-.42};
   if(profile==1)target[0]=-.03+.003*t;
   if(profile==2)target[0]=.04*std::sin(.6*t);
   if(profile==3)target[0]=t>2?.05:-.02;
   const auto a=lut.query(q[0],q[1]);ASSERT_TRUE(a.valid);
   auto o=map_torque(a.jacobian,q,v,target,prev,.005,c);ASSERT_TRUE(o.valid);prev=o.applied;
   const std::array<double,2> virtual_applied{
    a.jacobian[0]*prev[0]+a.jacobian[2]*prev[1],
    a.jacobian[1]*prev[0]+a.jacobian[3]*prev[1]};
   for(int i=0;i<2;++i){v[i]+=(virtual_applied[i]-.05*v[i])/.12*.005;q[i]+=v[i]*.005;}
   if(t>4){error2+=(target[0]-q[0])*(target[0]-q[0]);++count;}
  }
  EXPECT_LT(std::sqrt(error2/count),.003);
 }
 // A constant external load requires sustained nonzero torque even at rest.
 std::array<double,2> q{0,-.4},v{},prev{};double early=0,late=0;
 for(int n=0;n<4000;++n){
  auto a=lut.query(q[0],q[1]);ASSERT_TRUE(a.valid);
  auto o=map_torque(a.jacobian,q,v,{0,-.4},prev,.005,c);ASSERT_TRUE(o.valid);prev=o.applied;
  std::array<double,2> t{a.jacobian[0]*prev[0]+a.jacobian[2]*prev[1],a.jacobian[1]*prev[0]+a.jacobian[3]*prev[1]};
  for(int i=0;i<2;++i){double load=i==0?.3:-.7;v[i]+=(t[i]+load-.05*v[i])/.12*.005;q[i]+=v[i]*.005;}
  if(n==2000)early=std::hypot(prev[0],prev[1]);
  if(n==3999){late=std::hypot(prev[0],prev[1]);EXPECT_NEAR(t[0],-.3,1e-7);EXPECT_NEAR(t[1],.7,1e-7);}
 }
 EXPECT_GT(late,.1);EXPECT_NEAR(early,late,1e-7);
}
