#include <tasks/auto_aim/yolo.hpp>
#include <openvino/openvino.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>
int main(int argc,char **argv) try {
  if(argc<3) {std::cerr<<"usage: sp_sim_image_probe CONFIG IMAGE [CONFIDENCE]\n";return 2;}
  auto cfg=YAML::LoadFile(argv[1]);
  for(const char *key:{"classify_model","yolov5_model_path","yolov8_model_path","yolo11_model_path"})
    cfg[key]=(std::filesystem::path(SP_SOURCE_ROOT)/cfg[key].as<std::string>()).string();
  if(argc>3) cfg["min_confidence"]=std::stod(argv[3]);
  std::filesystem::create_directories("artifacts");std::filesystem::create_directories("logs");
  auto path="artifacts/probe-"+std::to_string(getpid())+".yaml";
  {std::ofstream f(path);f<<cfg;}
  auto_aim::YOLO detector(path,false);
  auto image=cv::imread(argv[2]); if(image.empty()) throw std::runtime_error("image unavailable");
  auto armors=detector.detect(image);
  ov::Core core;
  auto compiled=core.compile_model(cfg["yolov5_model_path"].as<std::string>(),"CPU");
  double scale=std::min(640.0/image.cols,640.0/image.rows);
  cv::Mat input(640,640,CV_8UC3,cv::Scalar(0,0,0));
  cv::resize(image,input(cv::Rect(0,0,int(image.cols*scale),int(image.rows*scale))),
             {int(image.cols*scale),int(image.rows*scale)});
  auto blob=cv::dnn::blobFromImage(input,1.0/255.0,{},cv::Scalar(),true);
  auto request=compiled.create_infer_request();
  request.set_input_tensor(ov::Tensor(ov::element::f32,{1,3,640,640},blob.ptr<float>()));request.infer();
  auto output=request.get_output_tensor();auto shape=output.get_shape();float *data=output.data<float>();
  size_t best=0;for(size_t i=1;i<shape[1];++i) if(data[i*shape[2]+8]>data[best*shape[2]+8]) best=i;
  nlohmann::json row=nlohmann::json::array();for(size_t i=0;i<shape[2];++i)row.push_back(data[best*shape[2]+i]);
  std::cerr<<nlohmann::json{{"shape",shape},{"best_row",row}}.dump()<<'\n';
  nlohmann::json out=nlohmann::json::array();
  for(const auto &a:armors) out.push_back({{"name",auto_aim::ARMOR_NAMES[a.name]},
    {"color",auto_aim::COLORS[a.color]},{"confidence",a.confidence},
    {"center",{a.center.x,a.center.y}}});
  std::cout<<out.dump(2)<<'\n';std::filesystem::remove(path);return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
