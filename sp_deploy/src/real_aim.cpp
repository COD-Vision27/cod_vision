// Hardware adapter reuses original SP perception/planning and original Gimbal transport.
#include <io/camera.hpp>
#include <io/gimbal/gimbal.hpp>
#include <tasks/auto_aim/yolo.hpp>
#include <tasks/auto_aim/solver.hpp>
#include <tasks/auto_aim/tracker.hpp>
#include <tasks/auto_aim/planner/planner.hpp>
#include <tools/math_tools.hpp>
#include <sim_geometry.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>
using Clock=std::chrono::steady_clock;
volatile std::sig_atomic_t stopping=0;
static void stop(int){stopping=1;}
static void diagnostic(const nlohmann::json &report) {
  auto line=std::string("SP_REAL_DIAG ")+report.dump()+"\n";
  std::fwrite(line.data(),1,line.size(),stdout);std::fflush(stdout);
}
struct Permission {bool control,fire;};
static Permission policy(bool armed,bool shooting,bool valid,double age_ms,bool planned_fire,double error) {
  bool control=armed && valid && age_ms>=-20 && age_ms<=250;
  return {control,control && shooting && planned_fire && error<.01};
}
int main(int argc,char **argv) try {
  if(argc<2 || std::string(argv[1])=="--help") {
    std::cout<<"sp_real_aim CONFIG [--control] [--fire]\nDefault observes only. --fire requires --control. Armor auto-aim only.\n";return argc<2?2:0;
  }
  if(std::string(argv[1])=="--self-test") {
    auto idle=policy(false,true,true,0,true,0);auto stale=policy(true,true,true,251,true,0);
    auto observe=policy(true,false,true,10,true,0);auto lag=policy(true,true,true,10,true,.02);
    auto enabled=policy(true,true,true,10,true,0);auto lost_pose=policy(true,true,false,10,true,0);
    if(idle.control || idle.fire || stale.control || stale.fire || !observe.control || observe.fire || lag.fire || !enabled.fire || lost_pose.control || lost_pose.fire)return 1;
    std::cout<<"PASS: disarmed, stale, tracking without fire, feedback error and explicit fire policy\n";return 0;
  }
  if(std::string(argv[1])=="--model-check") {
    if(argc!=3)throw std::runtime_error("--model-check CONFIG");
    auto cfg=YAML::LoadFile(argv[2]);auto_aim::YOLO detector(argv[2],false);
    cv::Mat blank(cfg["deploy_image_height"].as<int>(),cfg["deploy_image_width"].as<int>(),CV_8UC3,cv::Scalar());
    auto found=detector.detect(blank);
    std::cout<<"PASS: packaged model inference, blank armor count="<<found.size()<<std::endl;return found.empty()?0:1;
  }
  bool armed=false,shooting=false;
  for(int i=2;i<argc;++i){std::string arg=argv[i];if(arg=="--control")armed=true;else if(arg=="--fire")shooting=true;else throw std::runtime_error("unknown option: "+arg);}
  if(shooting && !armed)throw std::runtime_error("--fire requires --control");
  auto cfg=YAML::LoadFile(argv[1]);
  auto_aim::YOLO yolo(argv[1],false);auto_aim::Solver solver(argv[1]);
  auto_aim::Tracker tracker(argv[1],solver);auto_aim::Planner planner(argv[1]);
  io::Gimbal gimbal(argv[1]);io::Camera camera(argv[1]);
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  std::mutex mutex;std::optional<auto_aim::Target> latest;Clock::time_point last_frame{};
  Eigen::Matrix<double,3,3,Eigen::RowMajor> imu_basis(cfg["R_gimbal2imubody"].as<std::vector<double>>().data());
  std::atomic<bool> failed=false;
  std::thread plan_thread([&]{
    try {
      unsigned ticks=0;
      while(!stopping) {
        auto next=Clock::now()+std::chrono::milliseconds(10);
        std::optional<auto_aim::Target> target;Eigen::Vector2d feedback;double age;
        {std::lock_guard<std::mutex> lock(mutex);target=latest;age=std::chrono::duration<double,std::milli>(Clock::now()-last_frame).count();}
        if(age>250 || age<-20)target.reset();
        auto [live_q,live_stamp]=gimbal.latest_pose();
        double pose_age=std::chrono::duration<double,std::milli>(Clock::now()-live_stamp).count();
        bool live_valid=live_q.coeffs().allFinite() && std::abs(live_q.norm()-1)<.01 && pose_age>=-20 && pose_age<=200;
        if(live_valid) {
          Eigen::Matrix3d R=imu_basis.transpose()*live_q.normalized().toRotationMatrix()*imu_basis;
          auto live_ypr=tools::eulers(R,2,1,0);feedback={live_ypr[0],live_ypr[1]};
        } else {feedback={0,0};target.reset();}
        auto p=planner.plan(target,gimbal.state().bullet_speed);
        double error=std::hypot(tools::limit_rad(p.yaw-feedback[0]),p.pitch-feedback[1]);
        auto permit=policy(armed,shooting,live_valid && p.control && sp_sim::finite_plan(p) && gimbal.mode()==io::GimbalMode::AUTO_AIM,age,p.fire,error);
        gimbal.send(permit.control,permit.fire,permit.control?p.yaw:0,permit.control?p.yaw_vel:0,permit.control?p.yaw_acc:0,
          permit.control?p.pitch:0,permit.control?p.pitch_vel:0,permit.control?p.pitch_acc:0);
        if(++ticks%10==0)diagnostic(nlohmann::json{{"event","plan"},{"control",permit.control},{"fire",permit.fire},
          {"age_ms",age},{"pose_age_ms",pose_age},{"feedback_error_rad",error},{"yaw",p.yaw},{"pitch",p.pitch}});
        std::this_thread::sleep_until(next);
      }
    } catch(const std::exception &e){std::cerr<<"planner/transport: "<<e.what()<<std::endl;failed=true;stopping=1;}
    try {gimbal.send(false,false,0,0,0,0,0,0);}catch(...){}
  });
  try {
    while(!stopping) {
      cv::Mat image;Clock::time_point stamp;camera.read(image,stamp);
      if(image.empty())continue;
      if(image.cols!=cfg["deploy_image_width"].as<int>() || image.rows!=cfg["deploy_image_height"].as<int>()) {
        {std::lock_guard<std::mutex> lock(mutex);latest.reset();}
        std::cerr<<"capture resolution differs from measured calibration"<<std::endl;continue;
      }
      auto begin=Clock::now();double age=std::chrono::duration<double,std::milli>(begin-stamp).count();
      if(age>250 || age<-20){std::lock_guard<std::mutex> lock(mutex);latest.reset();continue;}
      try {
        solver.set_R_gimbal2world(gimbal.q(stamp));
        auto armors=yolo.detect(image);auto targets=tracker.track(armors,stamp);
        {std::lock_guard<std::mutex> lock(mutex);latest=targets.empty()?std::nullopt:std::optional<auto_aim::Target>(targets.front());last_frame=stamp;}
        diagnostic(nlohmann::json{{"event","vision"},{"armors",armors.size()},{"tracker",tracker.state()},
          {"image_age_ms",age},{"vision_ms",std::chrono::duration<double,std::milli>(Clock::now()-begin).count()}});
      } catch(const std::exception &e){std::lock_guard<std::mutex> lock(mutex);latest.reset();std::cerr<<"rejected frame: "<<e.what()<<std::endl;}
    }
  } catch(...) {stopping=1;plan_thread.join();throw;}
  stopping=1;plan_thread.join();return failed?1:0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
