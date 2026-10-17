#include <tasks/auto_aim/planner/fire_window.hpp>
#include <tasks/auto_aim/planner/planner.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

static void check(bool good, const char *message) {
  if (!good) throw std::runtime_error(message);
}
int main(int argc, char **argv) try {
  using auto_aim::within_fire_window;
  for (double yaw : {-.01, -.003, 0., .003, .01}) for (double pitch : {-.004, 0., .004})
    check(within_fire_window(yaw,pitch,.0035,1,0,0,0,false) ==
      (std::hypot(yaw,pitch)<.0035), "legacy gate unchanged");
  check(within_fire_window(.006,0,.0035,2,.5,4,1,false), "horizontal central armor window");
  check(!within_fire_window(.006,0,.0035,2,.5,8,1,false), "distance shrinks window");
  check(!within_fire_window(.006,0,.0035,2,.5,4,.25,false), "oblique armor shrinks window");
  check(!within_fire_window(0,.004,.0035,2,.5,4,1,false), "vertical tolerance does not grow");
  check(!within_fire_window(.006,.0025,.0035,2,.5,4,1,false), "ellipse rejects rectangle corners");
  check(within_fire_window(.006,0,.0035,2,.5,8,1,true), "large armor uses its own width");
  for (double cosine : {0., -1.})
    check(!within_fire_window(0,0,.0035,2,.5,4,cosine,false), "edge/back facing cannot fire");
  check(!within_fire_window(NAN,0,.0035,2,.5,4,1,false), "nonfinite error rejected");
  check(!within_fire_window(0,0,.0035,2,.5,NAN,1,false), "nonfinite geometry rejected");
  check(argc==2, "config required");
  auto cfg = YAML::LoadFile(argv[1]); cfg["fire_yaw_scale"] = 2.; cfg["fire_armor_fraction"] = .5;
  const auto temp = std::filesystem::temp_directory_path() /
    ("sp-fire-window-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  struct Cleanup { std::filesystem::path p; ~Cleanup() { std::filesystem::remove(p); } } cleanup{temp};
  {std::ofstream f(temp); f << cfg;}
  auto_aim::Planner planner(temp.string());
  for (double distance : {2.,4.,8.}) for (double speed : {0.,2.,6.}) {
    auto_aim::Target target(distance,speed,.25,.03);
    auto plan = planner.plan(target,22.);
    check(plan.control && std::isfinite(plan.yaw) && std::isfinite(plan.pitch), "finite geometry-aware planning");
  }
  for (double invalid : {-1., 1.01, double(INFINITY)}) {
    cfg["fire_armor_fraction"] = invalid;
    {std::ofstream f(temp); f << cfg;}
    bool rejected = false;
    try {auto_aim::Planner bad(temp.string());} catch(const std::invalid_argument &) {rejected=true;}
    check(rejected, "invalid window config rejected");
  }
  std::cout << "PASS: legacy gate, range, facing, dimensions, vertical bound, ellipse, finite plans, invalid input\n";
  return 0;
} catch(const std::exception &e) {std::cerr << "FAIL: " << e.what() << '\n';return 1;}
