#include <io/gimbal/gimbal.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <pty.h>
#include <thread>
#include <unistd.h>
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
int main() try {
  int master,slave;char device[128];
  if(openpty(&master,&slave,device,nullptr,nullptr)!=0)throw std::runtime_error("openpty failed");
  char config[]="/tmp/sp_live_pose_XXXXXX";int file=mkstemp(config);close(file);
  {std::ofstream f(config);f<<"use_can: false\nuart_mode: auto_aim\nuart_bullet_speed: 22\nimu_timeout_ms: 1000\nserial_imu:\n  port: "<<device<<"\n  baudrate: 921600\n  protocol: uart44\n  frame_size: 44\n  frame_stride: 44\n  rpy_offsets: [23,19,27]\n  angle_unit: degrees\n  velocity_unit: degrees\n  tx:\n    protocol: sp44\n    angle_unit: degrees\n";}
  std::atomic<bool> stop=false;std::atomic<float> yaw=10;
  std::thread producer([&]{while(!stop){std::array<uint8_t,44> frame{};frame[0]='S';frame[1]='P';frame[43]=13;
    float pitch=20,roll=30,y=yaw;std::memcpy(frame.data()+19,&pitch,4);std::memcpy(frame.data()+23,&roll,4);std::memcpy(frame.data()+27,&y,4);
    if(write(master,frame.data(),frame.size())<0)break;std::this_thread::sleep_for(10ms);}});
  bool passed=false;
  try {
    io::Gimbal gimbal(config);
    auto wait=[&](float expected_yaw){auto end=Clock::now()+1s;
      while(Clock::now()<end){auto [q,t]=gimbal.latest_pose();
        Eigen::Quaterniond expected=Eigen::AngleAxisd(expected_yaw*M_PI/180,Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(20*M_PI/180,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(30*M_PI/180,Eigen::Vector3d::UnitX());
        if(t!=Clock::time_point{} && q.angularDistance(expected)<1e-5 && Clock::now()-t<50ms)return t;
        std::this_thread::sleep_for(2ms);
      }throw std::runtime_error("fresh full-roll UART pose not received");};
    wait(10);std::this_thread::sleep_for(60ms);
    auto [old_q,old_stamp]=gimbal.latest_pose();
    yaw=45;wait(45);
    auto historical=gimbal.q(old_stamp-1ns);
    if(historical.angularDistance(old_q)>1e-5)throw std::runtime_error("fresh getter consumed camera pose history");
    stop=true;producer.join();std::this_thread::sleep_for(250ms);
    auto [q,t]=gimbal.latest_pose();
    if(Clock::now()-t<200ms)throw std::runtime_error("getter restamped stale pose");
    passed=true;
  } catch(...) {stop=true;if(producer.joinable())producer.join();close(master);close(slave);unlink(config);throw;}
  close(master);close(slave);unlink(config);
  std::cout<<"PASS: latest full quaternion updates without consuming camera history; stale receipt stays stale\n";return passed?0:1;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
