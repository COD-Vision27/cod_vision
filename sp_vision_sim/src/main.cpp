#include "local_sim_client.hpp"
#include <tasks/auto_aim/yolo.hpp>
#include <tasks/auto_aim/solver.hpp>
#include <tasks/auto_aim/tracker.hpp>
#include <sim_geometry.hpp>
#include <tools/math_tools.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include <condition_variable>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>

// All observation and prediction timestamps share the simulator clock. Wall time
// is used only for watchdogs; pausing/restarting never advances an old target.
static Clock::time_point sim_time(double seconds) {
  return Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds)));
}

int main(int argc, char **argv) try {
  if (argc != 5) throw std::runtime_error("Usage: sp_sim_ipc config.yaml channel seconds output-prefix");
  std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
  const auto config = std::filesystem::absolute(argv[1]);
  const auto prefix = std::filesystem::absolute(argv[4]);
  const double duration = std::stod(argv[3]);
  if (!std::isfinite(duration) || duration<0) throw std::runtime_error("seconds must be finite and nonnegative (0 = unlimited)");
  auto cfg = YAML::LoadFile(config.string());
  // Simulator default leaves CPU capacity for rendering/physics; explicit 0 restores OpenVINO auto.
  if (!cfg["inference_num_threads"]) cfg["inference_num_threads"]=4;
  // Avoid nested OpenCV pools competing with OpenVINO and the renderer.
  cv::setNumThreads(cfg["opencv_threads"].as<int>(1));
  for (const char *key : {"classify_model","yolov5_model_path","yolov8_model_path","yolo11_model_path"})
    cfg[key] = (std::filesystem::path(SP_SOURCE_ROOT)/cfg[key].as<std::string>()).string();
  const auto runtime = prefix.string()+"-runtime.yaml";
  struct Cleanup { std::string p; ~Cleanup(){std::filesystem::remove(p);} } cleanup{runtime};
  {std::ofstream out(runtime); out << cfg;}
  auto_aim::Planner planner(runtime);
  LocalSimClient client(argv[2]);
  std::ofstream diagnostics(prefix.string()+"-vision.jsonl"), states(prefix.string()+"-states.jsonl");
  std::mutex mutex;
  std::condition_variable wake;
  std::optional<SimImage> ready;
  std::optional<auto_aim::Target> target;
  double observed = 0;
  Clock::time_point updated{};
  Json active_context;
  uint64_t generation = 0;
  bool quit = false;
  int frames = 0, dropped = 0, errors = 0;
  std::thread vision([&] {
    std::unique_ptr<auto_aim::YOLO> detector;
    std::unique_ptr<auto_aim::Solver> solver;
    std::unique_ptr<auto_aim::Tracker> tracker;
    uint64_t tracker_generation = UINT64_MAX;
    while (true) {
      SimImage frame; uint64_t gen;
      {std::unique_lock<std::mutex> lock(mutex);
       wake.wait(lock,[&]{return quit || ready.has_value();});
       if (quit) return;
       frame = std::move(*ready); ready.reset(); gen = generation;}
      const auto start = Clock::now();
      try {
        const auto &meta = frame.metadata;
        if (tracker_generation != gen) {
          const auto &camera = meta.at("camera");
          auto local = YAML::Clone(cfg);
          for (const char *key : {"K", "distortion", "R_camera2gimbal", "t_camera2gimbal"}) {
            const char *dest = std::string(key)=="K" ? "camera_matrix" :
                               std::string(key)=="distortion" ? "distort_coeffs" : key;
            local[dest] = camera.at(key).get<std::vector<double>>();
          }
          local["enemy_color"] = meta.at("appearance").at("team").get<std::string>();
          {std::ofstream out(runtime); out << local;}
          if (!detector) detector = std::make_unique<auto_aim::YOLO>(runtime,false);
          solver = std::make_unique<auto_aim::Solver>(runtime);
          tracker = std::make_unique<auto_aim::Tracker>(runtime,*solver);
          tracker_generation = gen;
        }
        const auto qv = meta.at("gimbal").at("quaternion_wxyz").get<std::vector<double>>();
        if (qv.size()!=4) throw std::runtime_error("Invalid frame orientation");
        Eigen::Quaterniond q(qv[0],qv[1],qv[2],qv[3]);
        if (!q.coeffs().allFinite() || std::abs(q.norm()-1)>0.01) throw std::runtime_error("Invalid quaternion");
        solver->set_R_gimbal2world(q.normalized());
        // Online detection consumes only raw camera pixels, never simulator truth.
        if (frames==0) cv::imwrite(prefix.string()+"-first.png",frame.bgr);
        auto armors = detector->detect(frame.bgr);
        const auto detected_at=Clock::now();
        const double stamp = meta.at("sim_time").get<double>();
        auto targets = tracker->track(armors,sim_time(stamp));
        {std::lock_guard<std::mutex> lock(mutex);
         if (gen == generation) {
           target = targets.empty() ? std::nullopt : std::optional<auto_aim::Target>(targets.front());
           observed = stamp; updated = Clock::now();
         }}
        Json estimate=nullptr;
        if(!targets.empty()) {
          const auto x=targets.front().ekf_x();
          estimate=std::vector<double>(x.data(),x.data()+x.size());
        }
        Json observations=Json::array();
        for (const auto &a:armors) {
          Json corners=Json::array();for(const auto &p:a.points) corners.push_back({p.x,p.y});
          observations.push_back({{"corners",corners},{"confidence",a.confidence}});
        }
        diagnostics << Json{{"event","vision"},{"estimate",estimate},{"observations",observations},
          {"detect_ms",std::chrono::duration<double,std::milli>(detected_at-start).count()},
          {"track_ms",std::chrono::duration<double,std::milli>(Clock::now()-detected_at).count()},{"sim_time",stamp},{"armors",armors.size()},
          {"tracker",tracker->state()},{"has_target",!targets.empty()},
          {"processing_ms",std::chrono::duration<double,std::milli>(Clock::now()-start).count()},
          {"queue_ms",std::chrono::duration<double,std::milli>(start-frame.received).count()},
          {"capture",meta.value("capture_timing",Json::object())}}.dump() << '\n';
        ++frames;
      } catch (const std::exception &e) {
        {std::lock_guard<std::mutex> lock(mutex); if(gen==generation) target.reset();}
        diagnostics << Json{{"event","error"},{"message",e.what()}}.dump() << '\n'; ++errors;
      }
    }
  });
  // Joining on all paths avoids leaving the trigger active after an IPC error.
  struct Join { std::thread &t; std::mutex &m; std::condition_variable &cv; bool &q;
    ~Join(){ {std::lock_guard<std::mutex> l(m);q=true;} cv.notify_all();if(t.joinable())t.join();}
  } join{vision,mutex,wake,quit};
  const auto start = Clock::now();
  auto next = start;
  Json final_state;
  while (!exiting && (duration==0 || std::chrono::duration<double>(Clock::now()-start).count() < duration)) {
    if (client.poll_state()) {
      const auto &state=client.state();
      const auto context=LocalSimClient::frame_context(state);
      {std::lock_guard<std::mutex> lock(mutex);
       if(context!=active_context) {active_context=context;++generation;target.reset();ready.reset();}}
      states << state.dump() << '\n'; final_state=state;
    }
    if (!client.state().empty()) {
      SimImage frame;
      if (client.try_frame(frame)) {
        std::lock_guard<std::mutex> lock(mutex);
        if (ready) ++dropped;
        ready=std::move(frame); wake.notify_one();
      }
      client.request_frame();
      const auto &state=client.state();
      const double now = state.at("sim_time").get<double>();
      std::optional<auto_aim::Target> snapshot;
      {std::lock_guard<std::mutex> lock(mutex);
       if(client.fresh() && Clock::now()-updated < 250ms && now-observed >= 0 && now-observed < .25)
         snapshot=target;}
      const auto before=Clock::now();
      auto plan=planner.plan_at(snapshot,state.at("weapon").at("speed").get<double>(),sim_time(now));
      const auto &feedback=state.at("gimbal");
      const double error=std::hypot(tools::limit_rad(plan.yaw-feedback.at("yaw").get<double>()),
        plan.pitch-feedback.at("pitch").get<double>());
      const bool control=plan.control && sp_sim::finite_plan(plan);
      const bool fire=control && plan.fire && error < cfg["alignment_threshold"].as<double>(.01) &&
                      cfg["allow_fire"].as<bool>(true);
      client.send(control,plan.yaw,plan.pitch,plan.yaw_vel,plan.pitch_vel,plan.yaw_acc,plan.pitch_acc,fire);
      // Decimate plan telemetry; expensive detection never blocks mailbox/control updates.
      static unsigned count=0;
      if (++count%10==0) states << Json{{"event","plan"},{"sim_time",now},{"control",control},
        {"fire",fire},{"yaw",plan.yaw},{"pitch",plan.pitch},{"yaw_vel",plan.yaw_vel},
        {"pitch_vel",plan.pitch_vel},{"feedback_error",error},{"observation_age_s",now-observed},
        {"plan_ms",std::chrono::duration<double,std::milli>(Clock::now()-before).count()}}.dump() << '\n';
    }
    next+=10ms;
    if (next<Clock::now()) next=Clock::now();
    std::this_thread::sleep_until(next);
  }
  client.send(false,0,0);
  // Do not read worker counters before joining.
  {std::lock_guard<std::mutex> lock(mutex); quit=true;} wake.notify_all(); vision.join();
  std::ofstream report(prefix.string()+"-summary.json");
  report << Json{{"inference_num_threads",cfg["inference_num_threads"].as<int>()},
    {"opencv_threads",cv::getNumThreads()},{"frames",frames},{"dropped",dropped},{"errors",errors},
    {"wall_seconds",std::chrono::duration<double>(Clock::now()-start).count()},
    {"final",final_state}}.dump(2) << '\n';
  return frames>0 && errors==0 ? 0 : 1;
} catch (const std::exception &e) {std::cerr << e.what() << '\n';return 1;}
