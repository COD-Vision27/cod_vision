#include <io/camera.hpp>
#include <io/gimbal/gimbal.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
using Clock=std::chrono::steady_clock;
int main(int argc,char **argv) try {
  if(argc<4 || std::string(argv[1])=="--help") {
    std::cout<<"sp_capture CONFIG OUTPUT SECONDS [PERIOD_MS=500] [imu]\nRead-only capture; images and timestamp-matched wxyz poses.\n";return argc<2?2:0;
  }
  std::filesystem::path out=argv[2];
  if(std::filesystem::exists(out) && !std::filesystem::is_empty(out))throw std::runtime_error("output must be a new or empty directory");
  double seconds=std::stod(argv[3]);int period=argc>4?std::stoi(argv[4]):500;
  if(seconds<=0 || period<50)throw std::runtime_error("invalid capture duration/period");
  std::filesystem::create_directories(out);
  auto cfg=YAML::LoadFile(argv[1]);
  std::unique_ptr<io::Gimbal> gimbal;
  if(argc>5 && std::string(argv[5])=="imu")gimbal=std::make_unique<io::Gimbal>(argv[1]);
  io::Camera camera(argv[1]);
  nlohmann::json manifest={{"schema","sp_capture_v1"},{"kind","hardware"},
    {"camera_name",cfg["camera_name"].as<std::string>()},{"R_gimbal2imubody",cfg["R_gimbal2imubody"].as<std::vector<double>>()},
    {"pose_sync","original Gimbal::q(camera_timestamp); steady_clock; wxyz"},{"samples",nlohmann::json::array()}};
  auto start=Clock::now(),next=start;
  while(std::chrono::duration<double>(Clock::now()-start).count()<seconds) {
    cv::Mat im;Clock::time_point stamp;camera.read(im,stamp);
    if(im.empty() || Clock::now()<next)continue;
    Eigen::Quaterniond q;
    if(gimbal) {try {q=gimbal->q(stamp);}catch(const std::exception &e){std::cerr<<"Rejected pose: "<<e.what()<<'\n';continue;}}
    auto name=std::to_string(manifest["samples"].size()+1)+".png";
    if(!cv::imwrite((out/name).string(),im))throw std::runtime_error("image write failed");
    nlohmann::json sample={{"image",name},{"stamp_steady_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count()}};
    if(gimbal)sample["quaternion_wxyz"]={q.w(),q.x(),q.y(),q.z()};
    manifest["image_size"]={im.cols,im.rows};manifest["samples"].push_back(sample);
    {std::ofstream f(out/"dataset.json.tmp");f<<manifest.dump(2)<<'\n';if(!f)throw std::runtime_error("manifest write failed");}
    std::filesystem::rename(out/"dataset.json.tmp",out/"dataset.json");
    std::cout<<"saved "<<name<<std::endl;next=Clock::now()+std::chrono::milliseconds(period);
  }
  return manifest["samples"].empty()?1:0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
