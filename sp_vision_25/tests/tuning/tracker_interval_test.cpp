#include <tasks/auto_aim/tracker.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

static void check(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

int main(int argc, char **argv) try {
  check(argc == 2, "base config required");
  const auto original = YAML::LoadFile(argv[1]);
  const auto temp = std::filesystem::temp_directory_path() /
    ("sp-tracker-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  struct Cleanup { std::filesystem::path p; ~Cleanup() { std::filesystem::remove(p); } } cleanup{temp};
  auto write = [&](YAML::Node cfg) { std::ofstream f(temp); f << cfg; };
  for (bool omni : {false, true}) for (bool configured : {false, true}) {
    auto cfg = YAML::Clone(original);
    cfg["enemy_color"] = "blue";
    cfg.remove("tracker_max_dt");
    if (configured) cfg["tracker_max_dt"] = .2;
    write(cfg);
    auto_aim::Solver solver(temp.string());
    solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    auto_aim::Tracker tracker(temp.string(), solver);
    auto t = std::chrono::steady_clock::now();
    auto track = [&](bool visible, double dt) {
      t += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
      std::list<auto_aim::Armor> armors;
      if (visible) {
        auto points = solver.reproject_armor({4., 0., .1}, 0., auto_aim::small, auto_aim::three);
        armors.emplace_back(0, 3, .99, cv::boundingRect(points), points);
        armors.back().priority = auto_aim::fifth;
      }
      if (omni) tracker.track(std::vector<omniperception::DetectionResult>{}, armors, t);
      else tracker.track(armors, t);
    };
    for (int i = 0; i < 6; ++i) track(true, .02);
    check(tracker.state() == "tracking", "synthetic observations acquire tracking");
    track(false, .12);
    check(tracker.state() == (configured ? "temp_lost" : "lost"), "configured gap keeps state; legacy gap resets");
    if (configured) {
      track(true, .02);
      check(tracker.state() == "tracking", "tracking resumes after missed observation");
    }
    track(false, .25);
    check(tracker.state() == "lost", "long outage clears target");
    track(true, .02);
    track(false, -.01);
    check(tracker.state() == "lost", "out of order observation clears target");
  }
  for (double value : {0., -.1, .501, std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN()}) {
    auto cfg = YAML::Clone(original); cfg["tracker_max_dt"] = value; write(cfg);
    auto_aim::Solver solver(temp.string());
    bool rejected = false;
    try { auto_aim::Tracker tracker(temp.string(), solver); }
    catch (const std::invalid_argument &) { rejected = true; }
    check(rejected, "invalid interval rejected");
  }
  for (const char *key : {"initial_radius_variance", "initial_dr_variance", "initial_dz_variance"}) {
    for (double value : {0., -1., 1.001, std::numeric_limits<double>::quiet_NaN()}) {
      auto cfg = YAML::Clone(original); cfg[key] = value; write(cfg);
      auto_aim::Solver solver(temp.string());
      bool rejected = false;
      try { auto_aim::Tracker tracker(temp.string(), solver); }
      catch (const std::invalid_argument &) { rejected = true; }
      check(rejected, "invalid geometry variance rejected");
    }
  }
  {
    auto cfg = YAML::Clone(original); cfg["enemy_color"] = "blue";
    cfg["initial_radius_variance"] = .01; cfg["initial_dr_variance"] = .01;
    cfg["initial_dz_variance"] = .0025; write(cfg);
    auto_aim::Solver solver(temp.string()); solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    auto_aim::Tracker tracker(temp.string(), solver);
    auto points = solver.reproject_armor({4., 0., .1}, 0., auto_aim::small, auto_aim::three);
    std::list<auto_aim::Armor> armors;
    armors.emplace_back(0, 3, .99, cv::boundingRect(points), points);
    armors.back().priority = auto_aim::fifth;
    auto targets = tracker.track(armors, std::chrono::steady_clock::now());
    check(targets.size() == 1, "geometry-prior target initialized");
    const auto &cov = targets.front().ekf().P;
    check(std::abs(cov(8,8)-.01)<1e-12 && std::abs(cov(9,9)-.01)<1e-12 &&
          std::abs(cov(10,10)-.0025)<1e-12, "geometry covariance uses configured physical prior");
    check(std::abs(cov(1,1)-64)<1e-12, "velocity uncertainty preserved");
  }
  {
    auto cfg = YAML::Clone(original); cfg["enemy_color"] = "blue"; write(cfg);
    auto_aim::Solver solver(temp.string()); solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    auto_aim::Tracker tracker(temp.string(), solver);
    auto t = std::chrono::steady_clock::now();
    for (int i=0;i<8;++i) {
      auto points = solver.reproject_armor({4.,0.,.1},0.,auto_aim::big,auto_aim::three);
      std::list<auto_aim::Armor> armors;
      armors.emplace_back(0,3,.99,cv::boundingRect(points),points); armors.back().type=auto_aim::big;
      t += std::chrono::milliseconds(20);
      auto targets=tracker.track(armors,t);
      check(targets.size()==1 && targets.front().armor_xyza_list().size()==2 &&
            targets.front().ekf_x().allFinite(), "two-armor target updates remain finite");
    }
    check(tracker.state()=="tracking", "two-armor target acquires tracking");
  }
  for (const char *key : {"acceleration_variance", "angular_acceleration_variance"}) {
    for (double value : {0., -1., std::numeric_limits<double>::infinity()}) {
      auto cfg = YAML::Clone(original); cfg[key] = value; write(cfg);
      auto_aim::Solver solver(temp.string()); bool rejected = false;
      try { auto_aim::Tracker tracker(temp.string(), solver); }
      catch (const std::invalid_argument &) { rejected = true; }
      check(rejected, "invalid process variance rejected");
    }
  }
  {
    auto_aim::Armor armor(0, 3, .99, cv::Rect(0,0,20,10),
      {{0,0}, {20,0}, {20,10}, {0,10}}); armor.name = auto_aim::three; armor.type = auto_aim::small;
    check(armor.priority == auto_aim::fifth, "detected armor has deterministic default priority");
    armor.priority = auto_aim::fifth; armor.xyz_in_world = Eigen::Vector3d(4,0,.1);
    armor.ypr_in_world = Eigen::Vector3d::Zero();
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(11);
    auto_aim::Target target(armor, std::chrono::steady_clock::now(), .2, 4, zero, 25, 100);
    target.predict(.1);
    check(std::abs(target.ekf().P(1,1)-.25)<1e-12 &&
          std::abs(target.ekf().P(7,7)-1.)<1e-12, "configured continuous model variances propagate with dt");
    auto_aim::Target legacy(armor, std::chrono::steady_clock::now(), .2, 4, zero);
    legacy.predict(.1);
    check(std::abs(legacy.ekf().P(1,1)-1.)<1e-12 &&
          std::abs(legacy.ekf().P(7,7)-4.)<1e-12, "legacy process model preserved");
  }
  std::cout << "PASS: both tracker APIs, legacy/configured gaps, recovery, outage, reversed time, invalid config\n";
  return 0;
} catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
