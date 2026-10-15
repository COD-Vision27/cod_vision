#include <sim_geometry.hpp>
#include <tasks/auto_aim/solver.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>

static void check(bool good,const char *message) {if(!good) throw std::runtime_error(message);}
int main(int argc,char **argv) try {
  check(argc==2,"config required");
  auto yaml=YAML::LoadFile(argv[1]);
  auto r=sp_sim::optical_to_gimbal();
  check((r*r.transpose()-Eigen::Matrix3d::Identity()).norm()<1e-12,"orthogonal");
  check(std::abs(r.determinant()-1)<1e-12,"proper rotation");
  check((r*Eigen::Vector3d(0,0,1)-Eigen::Vector3d(1,0,0)).norm()<1e-12,"forward optical axis");
  Eigen::Quaterniond shot(0.8433914184570312,-0.537299633026123,0,0);
  auto t=sp_sim::camera_offset(shot,{-0.0020168116316199303,0.1963421255350113,-0.0600251629948616},
                                   {-0.0018309999722987413,0.11028800159692764,-0.010703999549150467});
  auto tv=yaml["t_camera2gimbal"].as<std::vector<double>>();
  check((t-Eigen::Vector3d(tv.data())).norm()<1e-9,"model offset vs YAML");
  check(std::abs(sp_sim::muzzle_pitch(-0.1)-(0.1-M_PI/2))<1e-12,"pitch sign");
  auto_aim::Solver solver(argv[1]);
  int cases=0;
  for(double yaw:{-0.4,0.0,0.4}) for(double distance:{2.0,5.0,8.0}) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d xyz=q*Eigen::Vector3d(distance,0.15,0.3);
    auto points=solver.reproject_armor(xyz,yaw+M_PI,auto_aim::small,auto_aim::three);
    auto_aim::Armor armor(1,3,0.99,cv::boundingRect(points),points);
    solver.solve(armor);
    check((armor.xyz_in_world-xyz).norm()<0.002,"PnP/reprojection round trip"); ++cases;
  }
  auto_aim::Planner planner(argv[1]);
  check(!planner.plan(std::optional<auto_aim::Target>{},25).control,"no target disables control");
  for(double speed:{-3.0,22.0,25.0,50.0}) {
    auto_aim::Target target(5,0.5,0.2,0);
    auto plan=planner.plan(target,speed);
    check(plan.control && sp_sim::finite_plan(plan),"valid finite plan");
    check(std::abs(plan.yaw_acc)<=50.01 && std::abs(plan.pitch_acc)<=100.01,"acceleration bounds");
  }
  std::cout<<"PASS: geometry, "<<cases<<" PnP cases, pitch convention, no-target, bullet-speed fallback, MPC bounds\n";
  return 0;
} catch(const std::exception &e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
