# `zimiao/sp_vision_25` 函数库参考（首批）

本文根据项目根目录 `CMakeLists.txt`、各子目录构建文件和源码中的 `#include`/调用整理。版本以构建环境为准；项目没有在仓库内固定所有第三方库版本。

## 1. OpenCV

- **定位**：视觉系统的图像、视频、相机标定和几何计算基础库。
- **构建**：根 `CMakeLists.txt` 使用 `find_package(OpenCV REQUIRED)`，所有主要目标链接 `${OpenCV_LIBS}`。
- **项目用途**：
  - `cv::Mat` 承载相机帧和模型输入输出；`cv::VideoCapture`/`cv::VideoWriter` 处理 USB 视频与录制文件。
  - `cv::findContours`、颜色/灰度变换、缩放和绘制用于装甲板检测与调试显示。
  - `cv::findCirclesGrid`、`cv::calibrateCamera`、`cv::solvePnP`、`cv::calibrateHandEye` 用于相机和手眼标定。
  - `cv::dnn::readNetFromONNX`、`blobFromImage` 为分类器提供 ONNX 推理路径。
- **常见接口**：`cv::imread`、`cv::resize`、`cv::projectPoints`、`cv::Rodrigues`、`cv::imshow`、`cv::waitKey`。
- **源码落点**：`tasks/auto_aim/*`、`tasks/auto_buff/*`、`calibration/*`、`io/usbcamera/*`。

## 2. Eigen3

- **定位**：线性代数、三维旋转和坐标变换库。
- **构建**：`find_package(Eigen3 REQUIRED)`，并将 `${EIGEN3_INCLUDE_DIR}` 加入头文件路径。
- **项目用途**：使用 `Vector2d/3d/4d`、`MatrixXd/Matrix3d`、`Quaterniond` 表示位置、姿态、协方差和雅可比矩阵；扩展卡尔曼滤波、PnP 坐标变换、云台姿态插值和手眼标定都依赖它。
- **常见接口**：`q.toRotationMatrix()`、`Quaterniond::slerp`、`asDiagonal()`、矩阵乘法和转置。
- **互操作**：`opencv2/core/eigen.hpp` 提供 `cv::eigen2cv`/`cv::cv2eigen`，项目在求解器和标定代码中混用 Eigen 与 OpenCV 类型。
- **源码落点**：`tasks/auto_aim/solver.*`、`tools/extended_kalman_filter.*`、`io/gimbal/*`、`calibration/*`。

## 3. OpenVINO Runtime

- **定位**：Intel 推理运行时，用于加载 ONNX/IR 模型并在 CPU、GPU 或 AUTO 设备执行检测与分类。
- **构建**：`tasks/auto_aim`、`tasks/auto_buff`、`tasks/omniperception` 查找 OpenVINO，并链接 `openvino::runtime`；根配置将路径设为 `/opt/intel/openvino_2024.6.0/runtime/cmake/`。
- **项目用途**：YOLOv5/v8/v11、Buff 检测和全向感知；代码读取 `.onnx` 或 `.xml/.bin` 模型，配置输入色彩/数据类型，创建推理请求并读取输出张量。
- **常见接口**：`ov::Core::read_model`、`compile_model`、`ov::CompiledModel::create_infer_request`、`ov::Tensor`、`ov::preprocess::PrePostProcessor`、`ov::hint::performance_mode`。
- **性能模式**：实时检测使用 `LATENCY`，多线程检测器使用 `THROUGHPUT`。
- **源码落点**：`tasks/auto_aim/yolos/*`、`tasks/auto_aim/classifier.*`、`tasks/auto_aim/multithread/*`、`tasks/auto_buff/yolo11_buff.*`。

## 4. yaml-cpp

- **定位**：YAML 配置解析和序列化库。
- **构建**：根目录和 `io/CMakeLists.txt` 使用 `find_package(yaml-cpp REQUIRED)`；可执行目标链接 `yaml-cpp`。
- **项目用途**：读取相机、检测器、云台、标定和任务参数；标定程序通过 `YAML::Emitter` 输出相机矩阵、畸变系数和手眼结果。
- **常见接口**：`YAML::LoadFile`、`node["key"].as<T>()`、`YAML::Emitter`、`YAML::BeginMap`/`EndMap`。
- **源码落点**：`configs/*.yaml`、`tools/yaml.hpp`、`tasks/*`、`calibration/*`、`io/gimbal/*`。

## 5. fmt

- **定位**：类型安全的格式化和终端输出库。
- **构建**：`find_package(fmt REQUIRED)`，目标链接 `fmt::fmt`。
- **项目用途**：生成模型/日志文件名、格式化调试信息、输出标定结果和状态文本；`fmt/chrono.h` 用于时间戳格式化。
- **常见接口**：`fmt::format`、`fmt::print`。
- **源码落点**：`tools/logger.cpp`、`calibration/*`、`tasks/*`、各 `src/*` 和测试程序。

## 6. spdlog

- **定位**：日志框架，支持多 sink、日志级别和线程安全输出。
- **构建**：根目录使用 `find_package(spdlog REQUIRED)`；封装见 `tools/logger.hpp/.cpp`。
- **项目用途**：同时写入 `logs/*.log` 和带颜色的标准输出；默认记录 `debug`，在 `info` 级别刷新文件。
- **常见接口**：`spdlog::logger`、`spdlog::sinks::basic_file_sink_mt`、`stdout_color_sink_mt`、`set_level`、`flush_on`。
- **源码落点**：`tools/logger.*`，并被相机、云台、IMU、检测器等模块调用。

## 7. nlohmann/json

- **定位**：单头文件 JSON 库，用于调试数据和绘图数据交换。
- **构建**：根目录使用 `find_package(nlohmann_json REQUIRED)`；源码通过 `<nlohmann/json.hpp>` 使用。
- **项目用途**：记录检测、跟踪、云台响应和预测器数据；`tools::Plotter` 将 JSON 序列化后交给绘图管线。
- **常见接口**：`nlohmann::json`、`operator[]`、`dump()`。
- **源码落点**：`tools/plotter.*`、`tasks/auto_buff/buff_predict.hpp`、`src/*`、`tests/*`。

## 8. Ceres Solver

- **定位**：非线性最小二乘优化库。
- **构建**：`tasks/auto_buff/CMakeLists.txt` 使用 `find_package(Ceres REQUIRED)`，并链接 `${CERES_LIBRARIES}`；根目录的 Buff 调试目标也链接该变量。
- **项目现状**：当前源码检索未发现 `ceres::Problem`、`ceres::Solve` 等直接 API 调用；它由 Buff 模块预留并参与构建链接，后续优化模型可能使用它。
- **源码落点**：`tasks/auto_buff/CMakeLists.txt`。

## 9. ROS 2（可选）

- **定位**：机器人消息、发布/订阅和节点运行时。
- **构建**：仅当 `ament_cmake`、`rclcpp`、`std_msgs`、`rosidl_typesupport_cpp`、`sp_msgs` 全部找到时，才编译 ROS 相关源文件和 `sentry`/通信测试目标。
- **项目用途**：发布导航/自动瞄准消息，订阅敌方状态和目标消息；通过独立线程运行 `rclcpp::spin`。
- **常见接口**：`rclcpp::init`、`rclcpp::Node`、`create_publisher`、`create_subscription`、`rclcpp::spin`、`rclcpp::shutdown`。
- **源码落点**：`io/ros2/*`、`src/sentry*`、`tests/*publish*`、`tests/*subscribe*`。

## 10. serial（项目内置库）

- **定位**：跨平台串口封装，项目在 `io/serial` 内直接编译其源文件。
- **构建**：`io/serial/CMakeLists.txt` 生成 `serial` 静态库；Linux 额外链接 `rt` 和 `pthread`。
- **项目用途**：读取 DM-IMU 数据、向云台发送控制帧；设置波特率、校验位、停止位、数据位和超时。
- **常见接口**：`serial::Serial`、`serial::Timeout::simpleTimeout`、`setParity`、`setStopbits`、`setBytesize`、`read`/`write`。
- **源码落点**：`io/dm_imu/*`、`io/gimbal/*`、`io/serial/*`。

## 11. HikRobot / MindVision 相机 SDK

- **定位**：两类工业相机的厂商采集 SDK，不由系统包管理器提供；对应动态库随项目放在 `io/hikrobot/lib` 和 `io/mindvision/lib`，并按 `x86_64`/`aarch64` 选择。
- **项目用途**：枚举设备、打开/关闭采集、设置曝光和图像格式、获取帧缓冲，并转换为 OpenCV 图像。
- **主要接口**：HikRobot 的 `MV_CC_CreateHandle`、`MV_CC_OpenDevice`、`MV_CC_StartGrabbing`、`MV_CC_GetImageBuffer`；MindVision 的 `CameraSdkInit`、`CameraEnumerateDevice`、`CameraInit`、`CameraGetImageBuffer`。
- **源码落点**：`io/hikrobot/*`、`io/mindvision/*`、`io/CMakeLists.txt`。

## 12. libusb-1.0 与 Linux SocketCAN

- **libusb-1.0**：`io` 链接 `usb-1.0`，用于相机设备的 USB 初始化、按 VID/PID 查找和复位；主要调用在 `io/hikrobot/hikrobot.cpp`、`io/mindvision/mindvision.cpp`。
- **SocketCAN**：Linux 内核 CAN 接口，`io/socketcan.hpp` 使用 `linux/can.h`、`net/if.h`、`sys/epoll.h`、`sys/ioctl.h` 和 `unistd.h`；`io::CBoard` 通过 `socket(PF_CAN, SOCK_RAW, CAN_RAW)`、绑定 `can0`、`epoll_wait` 和 `read/write` 与下位机通信。

## 13. TinyMPC（项目内置模块）

- **定位**：轻量模型预测控制实现，不是系统安装的第三方包。
- **构建**：`tasks/auto_aim/planner/tinympc` 编译为 `tinympcstatic`，由 `auto_aim` 链接。
- **项目用途**：自动瞄准规划器的约束控制和轨迹优化。
- **源码落点**：`tasks/auto_aim/planner/tinympc/*`、`tasks/auto_aim/planner/planner.*`。

## 构建依赖速查

| 依赖 | CMake 发现方式 | 主要功能 |
| --- | --- | --- |
| OpenCV | `find_package(OpenCV REQUIRED)` | 图像、视频、标定、几何 |
| Eigen3 | `find_package(Eigen3 REQUIRED)` | 矩阵、姿态、滤波 |
| OpenVINO | `find_package(OpenVINO REQUIRED)` | YOLO/分类/感知推理 |
| yaml-cpp | `find_package(yaml-cpp REQUIRED)` | 配置读写 |
| fmt | `find_package(fmt REQUIRED)` | 格式化输出 |
| spdlog | `find_package(spdlog REQUIRED)` | 日志 |
| nlohmann_json | `find_package(nlohmann_json REQUIRED)` | JSON 数据 |
| Ceres | `find_package(Ceres REQUIRED)` | Buff 非线性优化 |
| ROS 2 | 条件查找 | 节点与消息通信 |
| serial | 项目内 `add_subdirectory` | 串口通信 |
| TinyMPC | 项目内 `add_library` | MPC 规划 |
