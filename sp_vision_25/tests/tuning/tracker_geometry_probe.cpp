// Independent optical-observation probe. Truth is used only to project observations
// and score the output; Tracker receives image points and timestamps through its public API.
#include <tasks/auto_aim/tracker.hpp>
#include <tools/logger.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>

int main(int argc, char **argv) try {
  if (argc != 2) throw std::runtime_error("base config required");
  tools::logger()->set_level(spdlog::level::err);
  auto original = YAML::LoadFile(argv[1]);
  const auto temp = std::filesystem::temp_directory_path() /
    ("sp-geometry-probe-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  struct Cleanup { std::filesystem::path p; ~Cleanup() { std::filesystem::remove(p); } } cleanup{temp};
  nlohmann::json rows = nlohmann::json::array();
  for (std::string profile : {"legacy", "geometry_prior", "noise25_100", "noise25_25"}) {
    bool prior = profile == "geometry_prior";
    auto cfg = YAML::Clone(original); cfg["enemy_color"] = "blue"; cfg["tracker_max_dt"] = .2;
    if (prior) {
      cfg["initial_radius_variance"] = .01; cfg["initial_dr_variance"] = .01;
      cfg["initial_dz_variance"] = .0025;
    }
    if (profile == "noise25_100" || profile == "noise25_25") {
      cfg["acceleration_variance"] = 25;
      cfg["angular_acceleration_variance"] = profile == "noise25_100" ? 100 : 25;
    }
    { std::ofstream f(temp); f << cfg; }
    for (double distance : {2., 5., 8.}) for (double radius : {.12, .25, .4}) for (double dt : {.03, .12}) for (bool maneuver : {false, true}) {
      auto_aim::Solver solver(temp.string()); solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
      auto_aim::Tracker tracker(temp.string(), solver);
      auto t = std::chrono::steady_clock::now();
      std::mt19937 rng(2510); std::normal_distribution<float> noise(0., .2);
      double squared_error = 0, radius_error = 0, future_squared_error = 0;
      int estimates = 0, tracking = 0;
      for (int frame = 0; frame < 120; ++frame) {
        double seconds = frame * dt;
        double phase = 2. * seconds + (maneuver ? 2.*std::sin(3.*seconds) : 0.);
        Eigen::Vector3d center(distance, .3 * std::sin(seconds*(maneuver ? 3. : .7)), .1);
        double view = std::atan2(center.y(), center.x());
        double angle = phase + std::round((view-phase)/(M_PI/2))*(M_PI/2);
        Eigen::Vector3d xyz = center - Eigen::Vector3d(radius*std::cos(angle), radius*std::sin(angle), 0);
        auto points = solver.reproject_armor(xyz, angle, auto_aim::small, auto_aim::three);
        for (auto &point : points) { point.x += noise(rng); point.y += noise(rng); }
        std::list<auto_aim::Armor> armors;
        armors.emplace_back(0, 3, .99, cv::boundingRect(points), points); armors.back().priority = auto_aim::fifth;
        t += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
        auto targets = tracker.track(armors, t);
        if (tracker.state() == "tracking") ++tracking;
        if (targets.empty() || frame < 20) continue;
        const auto x = targets.front().ekf_x();
        squared_error += (Eigen::Vector3d(x[0],x[2],x[4])-center).squaredNorm();
        radius_error += std::abs(x[8]-radius);
        auto future = targets.front();
        const double lookahead = distance/22.;
        future.predict(lookahead);
        const auto predicted = future.ekf_x();
        const Eigen::Vector3d future_center(distance, .3*std::sin((seconds+lookahead)*(maneuver ? 3. : .7)), .1);
        future_squared_error += (Eigen::Vector3d(predicted[0],predicted[2],predicted[4])-future_center).squaredNorm();
        ++estimates;
      }
      rows.push_back({{"profile", profile}, {"maneuver", maneuver}, {"prior", prior}, {"distance_m", distance}, {"true_radius_m", radius}, {"frame_dt_s", dt},
        {"tracking_frames", tracking}, {"scored_estimates", estimates},
        {"center_rmse_m", estimates ? std::sqrt(squared_error/estimates) : -1},
        {"radius_mae_m", estimates ? radius_error/estimates : -1},
        {"future_center_rmse_m", estimates ? std::sqrt(future_squared_error/estimates) : -1}});
    }
  }
  std::cout << nlohmann::json{{"schema", "synthetic_optical_probe_v1"}, {"pixel_noise_std", .2}, {"cases", rows}}.dump(2) << '\n';
  return 0;
} catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
