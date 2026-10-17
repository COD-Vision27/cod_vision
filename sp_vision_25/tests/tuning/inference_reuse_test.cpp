#include <tasks/auto_aim/yolo.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <chrono>
#include <fstream>
#include <iostream>

int main(int argc, char **argv) try {
  if (argc != 3) throw std::runtime_error("config and image required");
  auto original = YAML::LoadFile(argv[1]);
  const std::filesystem::path root = SP_SOURCE_ROOT;
  for (const char *key : {"classify_model", "yolov5_model_path", "yolov8_model_path", "yolo11_model_path"})
    original[key] = (root/original[key].as<std::string>()).string();
  auto image = cv::imread(argv[2]);
  if (image.empty()) throw std::runtime_error("image required");
  cv::Mat blank = cv::Mat::zeros(image.size(), image.type()), flipped;
  cv::flip(image, flipped, 1);
  auto temp = std::filesystem::temp_directory_path() /
    ("sp-inference-reuse-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  struct Cleanup { std::filesystem::path p; ~Cleanup() { std::filesystem::remove(p); } } cleanup{temp};
  nlohmann::json variants = nlohmann::json::array();
  for (bool reuse : {false, true}) {
    auto cfg = YAML::Clone(original); cfg["reuse_infer_request"] = reuse;
    {std::ofstream f(temp); f << cfg;}
    auto_aim::YOLO detector(temp.string(),false);
    nlohmann::json sequence = nlohmann::json::array();
    for (const auto &frame : {image, blank, flipped, blank, image}) {
      auto armors = detector.detect(frame);
      nlohmann::json detections = nlohmann::json::array();
      for (const auto &a : armors) {
        nlohmann::json points = nlohmann::json::array();
        for (const auto &p : a.points) points.push_back({p.x,p.y});
        detections.push_back({{"color",a.color},{"name",a.name},{"confidence",a.confidence},{"points",points}});
      }
      sequence.push_back(detections);
    }
    // Time the complete existing detector, not just infer(), with fresh inputs.
    for (int i=0;i<5;++i) detector.detect(image);
    auto start = std::chrono::steady_clock::now();
    for (int i=0;i<40;++i) detector.detect(i%2 ? image : flipped);
    double elapsed = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/40.;
    variants.push_back({{"reuse",reuse},{"mean_detect_ms",elapsed},{"sequence",sequence}});
  }
  if (variants[0]["sequence"] != variants[1]["sequence"])
    throw std::runtime_error("request reuse changed detections");
  if (variants[0]["sequence"][0].empty() || !variants[0]["sequence"][1].empty() ||
      !variants[0]["sequence"][3].empty())
    throw std::runtime_error("test requires a detected image and empty blank frames");
  std::cout << nlohmann::json{{"status","PASS"},{"variants",variants}}.dump(2) << '\n';
  return 0;
} catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
