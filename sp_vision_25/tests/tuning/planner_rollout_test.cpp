#include <tasks/auto_aim/target.hpp>
#include <tools/math_tools.hpp>
#include <tools/trajectory.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

using auto_aim::Target;
using Clock = std::chrono::steady_clock;

static Target fixture(int count, double omega, double phase) {
  auto_aim::Armor armor(0, 3, .99f, cv::Rect(0, 0, 100, 40),
    {{0, 0}, {100, 0}, {100, 40}, {0, 40}});
  armor.name = count == 3 ? auto_aim::outpost : auto_aim::three;
  armor.type = auto_aim::small;
  armor.priority = auto_aim::fifth;
  armor.xyz_in_world = Eigen::Vector3d(4, 0, .1);
  armor.ypr_in_world = Eigen::Vector3d(phase, 0, 0);
  Target target(armor, Clock::now(), .25, count, Eigen::VectorXd::Ones(11));
  // Deterministic estimator states, never supplied to the online aiming node.
  auto &x = const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x;
  x[1] = .7; x[3] = -.4; x[5] = .2; x[7] = omega; x[9] = .08; x[10] = .05;
  return target;
}

static Eigen::Vector2d aim(const std::vector<Eigen::Vector4d> &armors) {
  auto closest = armors.front();
  for (const auto &armor : armors)
    if (armor.head<2>().norm() < closest.head<2>().norm()) closest = armor;
  tools::Trajectory trajectory(25, closest.head<2>().norm(), closest[2]);
  return {std::atan2(closest[1], closest[0]), -trajectory.pitch};
}

int main(int argc, char **argv) try {
  double max_geometry_error = 0, max_aim_error = 0;
  int samples = 0;
  for (int count : {2, 3, 4}) for (double omega : {-7.42, 0., 2.51, 7.42})
    for (double phase : {-3.13, 0., 3.13}) {
      const Target original = fixture(count, omega, phase);
      auto reference = original;
      const auto original_x = original.ekf_x();
      const auto original_p = original.ekf().P;
      reference.predict(-.51);
      for (int i = 0; i < 102; ++i) {
        const auto expected = reference.armor_xyza_list();
        const auto actual = original.armor_xyza_list(-.51 + .01*i);
        if (actual.size() != expected.size()) throw std::runtime_error("armor count changed");
        for (size_t j = 0; j < actual.size(); ++j) {
          max_geometry_error = std::max(max_geometry_error,
            (actual[j].head<3>() - expected[j].head<3>()).norm());
          max_geometry_error = std::max(max_geometry_error,
            std::abs(tools::limit_rad(actual[j][3] - expected[j][3])));
        }
        const auto old_aim = aim(expected), new_aim = aim(actual);
        max_aim_error = std::max(max_aim_error,
          std::hypot(tools::limit_rad(new_aim[0]-old_aim[0]), new_aim[1]-old_aim[1]));
        reference.predict(.01);
        ++samples;
      }
      if ((original.ekf_x()-original_x).norm() != 0 ||
          (original.ekf().P-original_p).norm() != 0)
        throw std::runtime_error("read-only rollout modified filter");
    }
  if (max_geometry_error > 1e-10 || max_aim_error > 1e-10)
    throw std::runtime_error("kinematic rollout differs from EKF state prediction");

  const auto target = fixture(4, 7.42, 3.13);
  double checksum = 0;
  auto benchmark = [&](bool fast) {
    const auto start = Clock::now();
    for (int repeat = 0; repeat < 300; ++repeat) {
      auto copy = target;
      if (!fast) copy.predict(-.51);
      for (int i = 0; i < 102; ++i) {
        const auto armors = fast ? target.armor_xyza_list(-.51 + .01*i) : copy.armor_xyza_list();
        checksum += aim(armors).squaredNorm();
        if (!fast) copy.predict(.01);
      }
    }
    return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
  };
  // Alternate order to reduce warmup/order bias; timing is evidence, not a gate.
  const double old_first = benchmark(false);
  const double fast_first = benchmark(true);
  const double fast_second = benchmark(true);
  const double old_second = benchmark(false);
  const double old_ms = (old_first + old_second)/2;
  const double fast_ms = (fast_first + fast_second)/2;
  nlohmann::json report = {{"samples",samples},{"max_geometry_error",max_geometry_error},
    {"max_aim_error_rad",max_aim_error},{"legacy_rollout_ms",old_ms},
    {"readonly_rollout_ms",fast_ms},{"speedup",old_ms/fast_ms},{"checksum",checksum}};
  if (argc > 1) { std::ofstream file(argv[1]); file << report.dump(2) << '\n'; }
  std::cout << report.dump(2) << '\n';
  return 0;
} catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
