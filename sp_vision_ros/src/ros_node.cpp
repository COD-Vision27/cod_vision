#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <rm_interfaces/msg/sp_gimbal_plan.hpp>
#include <tasks/auto_aim/yolo.hpp>
#include <tasks/auto_aim/solver.hpp>
#include <tasks/auto_aim/tracker.hpp>
#include <tools/logger.hpp>
#include <tools/math_tools.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <sim_geometry.hpp>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <map>
#include <thread>
#include <unistd.h>

using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using TF = tf2_msgs::msg::TFMessage;
using Clock = std::chrono::steady_clock;
static int64_t stamp_ns(const builtin_interfaces::msg::Time &t) {
  return int64_t(t.sec)*1000000000LL + t.nanosec;
}
static Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion &q) {
  Eigen::Quaterniond result(q.w,q.x,q.y,q.z);
  if (!result.coeffs().allFinite() || result.norm()<0.99 || result.norm()>1.01)
    throw std::runtime_error("Invalid frame quaternion");
  return result.normalized();
}

class SimAim : public rclcpp::Node {
  struct Frame { Image::ConstSharedPtr image; Info::ConstSharedPtr info; TF::ConstSharedPtr tf; };
  YAML::Node cfg_;
  std::filesystem::path runtime_config_;
  std::unique_ptr<auto_aim::YOLO> detector_;
  std::unique_ptr<auto_aim::Solver> solver_;
  std::unique_ptr<auto_aim::Tracker> tracker_;
  std::unique_ptr<auto_aim::Planner> planner_;
  rclcpp::Subscription<Image>::SharedPtr images_;
  rclcpp::Subscription<Info>::SharedPtr infos_;
  rclcpp::Subscription<TF>::SharedPtr poses_;
  rclcpp::Publisher<rm_interfaces::msg::SpGimbalPlan>::SharedPtr plans_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex frame_mutex_, target_mutex_;
  std::condition_variable wake_;
  std::map<int64_t,Frame> pending_;
  std::optional<Frame> ready_;
  std::optional<auto_aim::Target> target_;
  Clock::time_point last_frame_{};
  Clock::time_point last_update_{};
  Eigen::Vector2d actual_yaw_pitch_{0,0};
  bool quit_=false;
  std::thread vision_;
  int64_t last_processed_stamp_=0;
  int frames_=0, dropped_=0;

  void collect(int64_t stamp, const std::function<void(Frame&)> &set) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (stamp<=last_processed_stamp_) return;
    set(pending_[stamp]);
    auto it=pending_.find(stamp);
    if (it->second.image && it->second.info && it->second.tf) {
      ready_=it->second;
      last_processed_stamp_=stamp;
      pending_.erase(pending_.begin(),pending_.upper_bound(stamp));
      wake_.notify_one();
    }
    while(pending_.size()>16) { pending_.erase(pending_.begin()); ++dropped_; }
  }
  Eigen::Quaterniond validate_frame(const Frame &f) {
    const auto &im=*f.image; const auto &ci=*f.info;
    if (im.width!=cfg_["sim_width"].as<unsigned>() || im.height!=cfg_["sim_height"].as<unsigned>() ||
        ci.width!=im.width || ci.height!=im.height)
      throw std::runtime_error("Capture resolution changed: regenerate simulator.yaml");
    auto k=cfg_["camera_matrix"].as<std::vector<double>>();
    for(size_t i=0;i<9;++i) if(std::abs(ci.k[i]-k[i])>0.02)
      throw std::runtime_error("CameraInfo intrinsics changed: regenerate simulator.yaml");
    for(auto d:ci.d) if(std::abs(d)>1e-8) throw std::runtime_error("Unexpected simulated lens distortion");
    const geometry_msgs::msg::TransformStamped *gun=nullptr,*cam=nullptr;
    for(const auto &t:f.tf->transforms) {
      if(stamp_ns(t.header.stamp)!=stamp_ns(im.header.stamp)) throw std::runtime_error("Frame stamp mismatch");
      if(t.child_frame_id=="sp_gimbal" && t.header.frame_id=="map") gun=&t;
      if(t.child_frame_id=="sp_camera_optical" && t.header.frame_id=="sp_gimbal") cam=&t;
    }
    if(!gun || !cam) throw std::runtime_error("Missing synchronized camera/gimbal transform");
    auto expected=cfg_["t_camera2gimbal"].as<std::vector<double>>();
    auto &t=cam->transform.translation;
    if((Eigen::Vector3d(t.x,t.y,t.z)-Eigen::Vector3d(expected.data())).norm()>0.001 ||
       (quaternion(cam->transform.rotation).toRotationMatrix()-sp_sim::optical_to_gimbal()).norm()>0.001)
      throw std::runtime_error("Camera is not robot-mounted: use first-person mode (F3), or regenerate geometry");
    return quaternion(gun->transform.rotation);
  }
  void vision_loop() {
    while(true) {
      Frame f;
      { std::unique_lock<std::mutex> lock(frame_mutex_);
        wake_.wait(lock,[&]{return quit_||ready_.has_value();});
        if(quit_) return;
        f=*ready_; ready_.reset(); }
      auto t=sp_sim::steady_from_ros(stamp_ns(f.image->header.stamp));
      double age=std::chrono::duration<double,std::milli>(Clock::now()-t).count();
      try {
        if(age < -20 || age>cfg_["max_frame_age_ms"].as<double>()) throw std::runtime_error("Stale/future image timestamp");
        auto q=validate_frame(f);
        const auto &im=*f.image;
        if((im.encoding!="rgb8" && im.encoding!="bgr8") || im.step<im.width*3 ||
           im.data.size()<size_t(im.step)*im.height) throw std::runtime_error("Invalid image encoding/stride");
        cv::Mat raw(im.height,im.width,CV_8UC3,const_cast<uint8_t*>(im.data.data()),im.step),bgr;
        if(im.encoding=="rgb8") cv::cvtColor(raw,bgr,cv::COLOR_RGB2BGR); else bgr=raw.clone();
        solver_->set_R_gimbal2world(q);
        auto armors=detector_->detect(bgr);
        auto targets=tracker_->track(armors,t);
        auto ypr=tools::eulers(q,2,1,0);
        { std::lock_guard<std::mutex> lock(target_mutex_);
          target_=targets.empty()?std::nullopt:std::optional<auto_aim::Target>(targets.front());
          last_frame_=t; last_update_=Clock::now(); actual_yaw_pitch_={ypr[0],ypr[1]}; }
        std_msgs::msg::String report;
        report.data=nlohmann::json{{"event","vision"},{"stamp_ns",stamp_ns(im.header.stamp)},
          {"frames",++frames_},{"armors",armors.size()},{"tracker",tracker_->state()},
          {"age_ms",age},{"has_target",!targets.empty()}}.dump();
        diagnostics_->publish(report);
      } catch(const std::exception &e) {
        {std::lock_guard<std::mutex> lock(target_mutex_); target_.reset();}
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Rejected frame: %s",e.what());
      }
    }
  }
  void plan_once() {
    std::optional<auto_aim::Target> target; Eigen::Vector2d actual;
    { std::lock_guard<std::mutex> lock(target_mutex_);
      if(std::chrono::duration<double,std::milli>(Clock::now()-last_update_).count()<cfg_["target_timeout_ms"].as<double>() &&
         std::chrono::duration<double,std::milli>(Clock::now()-last_frame_).count()<cfg_["max_frame_age_ms"].as<double>()) target=target_;
      actual=actual_yaw_pitch_; }
    auto_aim::Plan plan{};
    try {plan=planner_->plan(target,cfg_["sim_bullet_speed"].as<double>());}
    catch(const std::exception &e) {RCLCPP_WARN(get_logger(),"Planner failed: %s",e.what());}
    rm_interfaces::msg::SpGimbalPlan out;
    out.header.stamp=now(); out.header.frame_id="map";
    out.control=plan.control && sp_sim::finite_plan(plan);
    // This pose belongs to the delayed camera frame and is diagnostic only.
    // The simulator checks the actual firing axis at command execution time.
    double feedback_error=std::hypot(tools::limit_rad(plan.yaw-actual[0]),plan.pitch-actual[1]);
    out.fire=out.control && plan.fire && cfg_["allow_fire"].as<bool>();
    if(out.control) {out.yaw=plan.yaw; out.pitch=plan.pitch; out.yaw_vel=plan.yaw_vel;
      out.pitch_vel=plan.pitch_vel; out.yaw_acc=plan.yaw_acc; out.pitch_acc=plan.pitch_acc;}
    out.max_yaw_acc=cfg_["max_yaw_acc"].as<double>(); out.max_pitch_acc=cfg_["max_pitch_acc"].as<double>();
    out.valid_for=cfg_["command_timeout_s"].as<double>(); plans_->publish(out);
    std_msgs::msg::String report;
    report.data=nlohmann::json{{"event","plan"},{"control",out.control},{"fire",out.fire},
      {"yaw",out.yaw},{"pitch",out.pitch},{"yaw_vel",out.yaw_vel},{"pitch_vel",out.pitch_vel},
      {"yaw_acc",out.yaw_acc},{"pitch_acc",out.pitch_acc},{"feedback_error",feedback_error}}.dump();
    diagnostics_->publish(report);
  }
public:
  SimAim():Node("sp_sim_auto_aim") {
    auto config=std::filesystem::absolute(declare_parameter<std::string>("config","configs/standard3.yaml"));
    cfg_=YAML::LoadFile(config.string());
    auto root=std::filesystem::path(declare_parameter<std::string>("sp_vision_root",SP_SOURCE_ROOT));
    for(const char *key:{"classify_model","yolov5_model_path","yolov8_model_path","yolo11_model_path"})
      cfg_[key]=(root/cfg_[key].as<std::string>()).string();
    std::filesystem::create_directories("artifacts"); std::filesystem::create_directories("logs");
    runtime_config_=std::filesystem::absolute("artifacts/runtime-"+std::to_string(getpid())+".yaml");
    {std::ofstream file(runtime_config_); file<<cfg_;}
    tools::logger();
    detector_=std::make_unique<auto_aim::YOLO>(runtime_config_.string(),false);
    solver_=std::make_unique<auto_aim::Solver>(runtime_config_.string());
    tracker_=std::make_unique<auto_aim::Tracker>(runtime_config_.string(),*solver_);
    planner_=std::make_unique<auto_aim::Planner>(runtime_config_.string());
    auto qos=rclcpp::SensorDataQoS().keep_last(4);
    plans_=create_publisher<rm_interfaces::msg::SpGimbalPlan>(cfg_["plan_topic"].as<std::string>(),rclcpp::QoS(1));
    diagnostics_=create_publisher<std_msgs::msg::String>(cfg_["diagnostics_topic"].as<std::string>(),qos);
    images_=create_subscription<Image>(cfg_["image_topic"].as<std::string>(),qos,[this](Image::ConstSharedPtr msg){collect(stamp_ns(msg->header.stamp),[&](Frame &f){f.image=msg;});});
    infos_=create_subscription<Info>(cfg_["camera_info_topic"].as<std::string>(),qos,[this](Info::ConstSharedPtr msg){collect(stamp_ns(msg->header.stamp),[&](Frame &f){f.info=msg;});});
    poses_=create_subscription<TF>(cfg_["frame_tf_topic"].as<std::string>(),qos,[this](TF::ConstSharedPtr msg){if(!msg->transforms.empty()) collect(stamp_ns(msg->transforms[0].header.stamp),[&](Frame &f){f.tf=msg;});});
    timer_=create_wall_timer(std::chrono::milliseconds(cfg_["plan_period_ms"].as<int>()),[this]{plan_once();});
    vision_=std::thread([this]{vision_loop();});
    RCLCPP_INFO(get_logger(),"SP YOLO -> Solver -> Tracker/EKF -> Planner/TinyMPC ready; no hardware IO");
  }
  ~SimAim() override {
    timer_->cancel();
    {std::lock_guard<std::mutex> lock(frame_mutex_);quit_=true;} wake_.notify_all();
    if(vision_.joinable()) vision_.join();
    std::error_code ec; std::filesystem::remove(runtime_config_,ec);
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<SimAim>());}
  catch(const std::exception &e) {fprintf(stderr,"sp_sim_auto_aim: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
