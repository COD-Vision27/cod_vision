#include <tasks/auto_aim/planner/planner.hpp>
#include <tools/math_tools.hpp>
#include <iostream>

using Clock=std::chrono::steady_clock;
static auto_aim::Target fixture(Clock::time_point at) {
  auto_aim::Armor a(0,3,.99f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
  a.name=auto_aim::three; a.type=auto_aim::small; a.priority=auto_aim::fifth;
  a.xyz_in_world={4,.2,-.3};a.ypr_in_world={3.,0,0};
  auto_aim::Target t(a,at,.25,4,Eigen::VectorXd::Ones(11));
  auto &x=const_cast<tools::ExtendedKalmanFilter &>(t.ekf()).x;
  x[1]=.3;x[3]=.7;x[7]=4;
  return t;
}
int main(int argc,char **argv) try {
  if(argc!=2) throw std::runtime_error("config required");
  auto_aim::Planner a(argv[1]),b(argv[1]);
  const auto zero=Clock::time_point(std::chrono::seconds(100));
  const auto shift=std::chrono::hours(24*100);
  const auto target=fixture(zero), shifted=fixture(zero+shift);
  for(int ms:{0,10,75,150,200}) {
    const auto p=a.plan_at(target,23.,zero+std::chrono::milliseconds(ms));
    const auto q=b.plan_at(shifted,23.,zero+shift+std::chrono::milliseconds(ms));
    if(!p.control || !q.control || p.fire!=q.fire ||
       std::abs(tools::limit_rad(p.yaw-q.yaw))>1e-7 || std::abs(p.pitch-q.pitch)>1e-7 ||
       std::abs(p.yaw_vel-q.yaw_vel)>1e-7 || std::abs(p.pitch_vel-q.pitch_vel)>1e-7 ||
       std::abs(p.yaw_acc-q.yaw_acc)>1e-7 || std::abs(p.pitch_acc-q.pitch_acc)>1e-7)
      throw std::runtime_error("planner output depends on absolute clock epoch");
  }
  if(a.plan_at(std::nullopt,23,zero).control) throw std::runtime_error("empty target enabled control");
  std::cout<<"PASS: planner clock translation and empty-target stop\n";
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
