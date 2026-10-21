# sp_vision_25 开发与使用指南

> 本文面向第一次运行、调试或扩展本项目的开发者，内容基于提交 `bd9f5e7` 的实际源码和命令行参数。算法原理见 [PROJECT_INTRODUCTION.md](PROJECT_INTRODUCTION.md)，新版 `standard_mpc` 自瞄和 Planner 的逐步源码说明见 [PLANNER_DEVELOPMENT_GUIDE.md](PLANNER_DEVELOPMENT_GUIDE.md)，USB/海康/迈德威视相机的逐项参数和排障见 [CAMERA_GUIDE.md](CAMERA_GUIDE.md)，相机内参与手眼标定的完整操作见 [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md)。

> 本地 UART 更新（2026-10-10）：本次 new_uart1/new_uart2 协议在 CBoard/SocketCAN 内通过 `use_can: false` 选择；使用 `cboard_test` 检查，单相机视觉入口为 `standard`。本文原有“新串口”仍指独立的 Gimbal/SP/CRC 协议，`gimbal_test` 和 `standard_mpc` 不适用于本次设备。完整差异与操作见 [VERSION_COMPARISON_AND_USAGE.md](VERSION_COMPARISON_AND_USAGE.md)，字节协议及实测结果见 [SERIAL_IMU_GUIDE.md](SERIAL_IMU_GUIDE.md)。

## 目录

1. [先选择测试目标](#1-先选择测试目标)
2. [环境与依赖](#2-环境与依赖)
3. [编译项目](#3-编译项目)
4. [不使用海康相机进行测试](#4-不使用海康相机进行测试)
5. [离线数据格式](#5-离线数据格式)
6. [相机后端与测试入口](#6-相机后端与测试入口)
7. [配置文件的选择与修改](#7-配置文件的选择与修改)
8. [测试程序索引](#8-测试程序索引)
9. [从离线测试走到实车运行](#9-从离线测试走到实车运行)
10. [日志、曲线和录像](#10-日志曲线和录像)
11. [新增相机或输入源](#11-新增相机或输入源)
12. [推荐开发流程](#12-推荐开发流程)
13. [常见问题](#13-常见问题)
14. [安全注意事项](#14-安全注意事项)

## 1. 先选择测试目标

不要一开始就运行完整机器人入口。项目把检测、解算、跟踪、规划、通信和硬件驱动放在同一仓库，但不同测试需要的设备完全不同。

| 目标 | 推荐程序 | 是否需要工业相机 | 是否需要 IMU/CAN/串口 | 能验证什么 |
|---|---|---:|---:|---|
| 跑通仓库自带示例 | `auto_aim_test` | 否 | 否 | 检测、PnP、跟踪、传统 Aimer |
| 只测试任意视频中的装甲检测 | `detector_video_test` | 否 | 否 | YOLO 或传统检测器 |
| 测试 MPC 规划器 | `planner_test_offline` | 否 | 否 | 合成旋转目标、TinyMPC 输出 |
| 测试自定义完整录像 | `auto_aim_test` | 否 | 否，但需要姿态文本 | 检测到瞄准的完整离线链路 |
| 测试普通 UVC/V4L2 摄像头 | `usbcamera_test` | 否 | 否 | USB 取流、帧率和画面 |
| 用 USB 摄像头实时检测 | `usbcamera_detect_test` | 否 | 否 | USB 取流加 YOLO |
| 测试工业相机 | `camera_test` | 是，海康或迈德威视 | 否 | 工业相机驱动和帧率 |
| 测试 CBoard CAN / 本地 UART | `cboard_test` | 否 | 需要匹配协议的下位机 | CAN 姿态/弹速；UART 欧拉角转换后的姿态 |
| 测试原 SP/CRC 串口协议 | `gimbal_test` | 否 | 需要 SP 固件的下位机 | 模式、姿态、云台状态；不适用 new_uart 图片 |
| 运行机器人完整流程 | `standard` 或 `standard_mpc` | 是 | 是 | 实车闭环 |

最小上手路径是：

```text
auto_aim_test
  -> detector_video_test
  -> planner_test_offline
  -> 自己的录像和配置
  -> 单独验证相机
  -> 单独验证通信
  -> 完整机器人入口
```

## 2. 环境与依赖

### 2.1 基准环境

仓库原始开发环境为 Ubuntu 22.04、C++17、CMake 3.16 以上、OpenCV、Eigen、OpenVINO 2024.6、yaml-cpp、fmt、spdlog、nlohmann-json、Ceres 和 libusb。

常规系统依赖可安装为：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  libopencv-dev \
  libeigen3-dev \
  libyaml-cpp-dev \
  libfmt-dev \
  libspdlog-dev \
  nlohmann-json3-dev \
  libceres-dev \
  libusb-1.0-0-dev \
  can-utils
```

OpenVINO 需要另外安装。当前多个 `CMakeLists.txt` 将路径固定为：

```text
/opt/intel/openvino_2024.6.0/runtime/cmake
```

如果 OpenVINO 安装在其他位置，仅设置环境变量不一定有效，因为项目会在 CMake 文件中再次设置 `OpenVINO_DIR`。应统一修改顶层和 `tasks/auto_aim`、`tasks/auto_buff`、`tasks/omniperception` 下的路径，或后续将它改成可覆盖的 CMake cache 变量。

### 2.2 没有海康相机是否能编译

“没有海康相机”有三种不同情况：

| 情况 | 当前仓库能否处理 |
|---|---|
| 没有物理海康相机，只做离线测试 | 可以 |
| 没有安装系统级海康 MVS 软件，但保留仓库内 SDK 文件 | x86_64/aarch64 通常可以编译，仓库自带头文件和动态库 |
| 希望删除所有海康 SDK 文件并完全不编译海康代码 | 当前 CMake 不支持，需要把相机后端改成可选组件 |

`io/CMakeLists.txt` 当前会无条件编译海康和迈德威视源文件，并将二者链接到 `io`。但 `io` 是静态库，链接器只会把被使用的对象拉进最终程序。因此当前构建出的以下程序没有海康或迈德威视运行时依赖：

- `auto_aim_test`
- `detector_video_test`
- `planner_test_offline`
- `calibrate_camera`
- `calibrate_handeye`
- `usbcamera_test`

而以下程序会真正加载工业相机 SDK：

- `capture`
- `camera_test`
- 使用 `io::Camera` 的机器人入口

可用下面的命令检查某个二进制是否仍依赖工业相机库：

```bash
ldd build/auto_aim_test | grep -E 'MvCameraControl|MVSDK|not found'
ldd build/camera_test | grep -E 'MvCameraControl|MVSDK|not found'
```

第一条在当前构建中没有输出；第二条应列出 `libMvCameraControl.so` 和 `libMVSDK.so`。

### 2.3 架构限制

工业相机 CMake 只识别：

- `x86_64`
- `aarch64`

其他处理器架构会在配置阶段触发 `Unsupported architecture`。纯离线逻辑理论上并不依赖这两个架构，但当前 CMake 尚未把硬件后端隔离出来。

## 3. 编译项目

所有命令建议在仓库根目录执行，因为 YAML 中的模型路径和程序默认输入都是相对路径：

```bash
cd /path/to/project/sp_vision_25
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

只编译常用离线目标可使用：

```bash
cmake --build build \
  --target auto_aim_test detector_video_test planner_test_offline calibrate_camera \
  -j"$(nproc)"
```

不过顶层 CMake 在“配置阶段”仍会进入全部子目录，所以即使只构建 `auto_aim_test`，OpenVINO 和 Ceres 仍必须能被 CMake 找到。

编译完成后先检查帮助信息：

```bash
./build/auto_aim_test --help
./build/detector_video_test --help
./build/calibrate_camera --help
```

## 4. 不使用海康相机进行测试

### 4.1 方案一：运行仓库自带完整录像

这是最推荐的首次测试。它不创建 `io::Camera`，直接读取：

```text
assets/demo/demo.avi
assets/demo/demo.txt
```

运行：

```bash
./build/auto_aim_test
```

等价的完整命令为：

```bash
./build/auto_aim_test \
  --config-path=configs/demo.yaml \
  --start-index=0 \
  --end-index=0 \
  assets/demo/demo
```

注意输入参数不带 `.avi` 或 `.txt` 后缀，程序会自动拼接这两个后缀。`end-index=0` 表示一直读到文件结束。

运行时会执行：

```text
视频帧 + 对应四元数
  -> YOLOv5 检测
  -> PnP 与坐标变换
  -> Tracker/EKF
  -> Aimer 与弹道预测
  -> 重投影显示和 UDP 调试数据
```

窗口中按 `q` 退出。程序使用 `cv::imshow`，所以需要图形桌面；SSH 无显示环境时可使用 X11 转发、远程桌面或 `xvfb-run`，否则会出现 Qt/GTK display 错误。

只检查一小段录像：

```bash
./build/auto_aim_test --start-index=100 --end-index=180 assets/demo/demo
```

### 4.2 方案二：只对任意视频做检测

如果只有手机录像、USB 摄像头录像或比赛视频，没有 IMU 四元数，先运行检测器测试：

```bash
./build/detector_video_test \
  --config-path=configs/demo.yaml \
  /absolute/path/to/input.avi
```

使用传统灯条检测：

```bash
./build/detector_video_test \
  --config-path=configs/demo.yaml \
  --tradition=true \
  /absolute/path/to/input.avi
```

这个程序不做 PnP、跟踪或瞄准，因此视频可以来自任意相机。它仍要求画面中装甲外观与模型训练域相近，并且配置中的 `enemy_color`、阈值和 ROI 合理。

使用任意视频时建议先：

1. 将 `device` 设为 `CPU`，排除 GPU 插件问题。
2. 将 `use_roi` 设为 `false`，避免目标落在裁剪区域外。
3. 确认 `yolo_name` 对应的模型文件存在。
4. 根据画面选择正确的 `enemy_color`。
5. 先检查整段视频，再逐步收紧 `min_confidence`。

### 4.3 方案三：离线测试 TinyMPC

规划器测试不需要图像、相机、IMU 或下位机：

```bash
./build/planner_test_offline \
  --d=3.0 \
  --w=5.0 \
  configs/standard4.yaml
```

参数含义：

- `d`：合成目标与云台的距离，单位 m；
- `w`：合成目标绕中心旋转的角速度，单位 rad/s；
- 配置文件：提供弹道偏置、MPC 的 `Q/R`、角加速度上限和开火阈值。

程序持续以约 100 Hz 发送调试数据到 UDP 9870 端口，按 `Ctrl+C` 退出。它没有图像窗口，PlotJuggler 未启动时程序也可运行，只是看不到曲线。

### 4.4 方案四：把普通视频变成完整离线输入

`auto_aim_test` 需要同名的 `.avi` 和 `.txt`。对于固定不动的相机，可以为每帧使用单位四元数：

```text
1 0 0 0
```

它表示“相机/云台姿态没有旋转”。每行仍需要时间，所以完整格式为：

```text
t w x y z
```

例如 30 FPS 的前三帧：

```text
0.000000000 1 0 0 0
0.033333333 1 0 0 0
0.066666667 1 0 0 0
```

这种方式可以检查检测、PnP、跟踪和预测代码是否连通，但不能验证 IMU 姿态补偿。若视频来自手持或运动相机，伪造单位四元数会把相机运动误认为目标运动。

自定义视频还必须使用对应相机、镜头和分辨率的 `camera_matrix` 与 `distort_coeffs`。直接套用 `configs/demo.yaml` 可能仍能显示检测框，但 PnP 距离和重投影没有可信的物理意义。

## 5. 离线数据格式

### 5.1 文件命名

给定输入基名：

```text
records/run_001
```

程序实际读取：

```text
records/run_001.avi
records/run_001.txt
```

### 5.2 姿态文本格式

每一帧对应一行：

```text
timestamp_seconds q_w q_x q_y q_z
```

各字段含义：

| 字段 | 含义 | 要求 |
|---|---|---|
| `timestamp_seconds` | 相对时间，单位 s | 应单调不减，并反映真实帧间隔 |
| `q_w` | 四元数实部 | 与后面三项组成单位四元数 |
| `q_x,q_y,q_z` | 四元数虚部 | 顺序必须是 `w,x,y,z`，不是 Eigen 内部的 `x,y,z,w` |

单位四元数应满足：

```math
q_w^2+q_x^2+q_y^2+q_z^2\approx 1
```

这里 `q_w,q_x,q_y,q_z` 是四元数的四个分量；左侧是四元数模长的平方。明显偏离 1 的数据应先归一化或检查通信解析。

### 5.3 配对规则

- 第 `i` 帧严格对应文本第 `i` 行。
- 文本行数应不少于视频帧数。
- 使用 `start-index` 时，程序会同时跳过视频帧和相同数量的文本行。
- 当前测试程序没有完善检查文本读取失败；行数不足可能导致未定义的姿态输入，而不是友好的报错。
- 时间戳不应按播放速度重新生成，应保留采集时的真实时间间隔。

### 5.4 Recorder 生成的格式

`tools::Recorder` 会在 `records/` 下生成同名 AVI 和 TXT：

- 视频编码：MJPG；
- 文本顺序：`time w x y z`；
- 默认记录频率：30 FPS；
- 内部队列容量为 1；保存线程尚未取走上一帧时，新送入的帧会被丢弃，因而不会持续积压内存。

当前 `standard_mpc` 和 `sentry_multithread` 实际调用了 `recorder.record(...)`；多个旧入口虽然创建了 Recorder，但录制调用被注释。开始实车测试前应检查当前入口是否真的启用了录制，避免以为已经留下数据。

## 6. 相机后端与测试入口

相机的逐项参数、白屏实测、海康 MVS 设置和故障码已独立整理到 [CAMERA_GUIDE.md](CAMERA_GUIDE.md)。本节只保留开发流程中需要的入口，避免把开发文档写成单一 USB 相机手册。

### 6.1 USB/UVC 相机

USB 相机使用 Linux V4L2，先查看设备及能力：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video4 --list-formats-ext
v4l2-ctl -d /dev/video4 --list-ctrls-menus
```

只测试取流：

```bash
./build/usbcamera_test \
  --name=video4 \
  --display \
  configs/sentry.yaml
```

实时检测：

```bash
./build/usbcamera_detect_test \
  --name=video4 \
  --display \
  configs/sentry.yaml
```

当前环境曾复现：`usb_exposure: 500` 在一台 UVC 设备上约等于 50 ms，采样帧平均亮度达到 `251.8/255`，会显示为几乎全白。重新插拔后的当前设备还暴露出更深层的问题：`/dev/video5` 是 Metadata Capture，图像节点变为 `/dev/video4`；项目固定写入 `CAP_PROP_AUTO_EXPOSURE=1` 后，摄像头进入手动模式，即使 `usb_exposure` 降到 10，画面仍接近全白，而自动模式 `auto_exposure=3` 正常。详细复现、临时恢复命令和源码修复方向见 [CAMERA_GUIDE.md](CAMERA_GUIDE.md)。不同设备范围不同，必须以 `v4l2-ctl` 读回值为准。

当前 USB 后端还通过 sharpness 值 2/3 识别左右相机，只有满足该条件才设置分辨率和曝光。它不能只靠 `camera_name` 配置替代 `standard` 使用的主工业相机。另一个重要限制是源码把 `CAP_PROP_AUTO_EXPOSURE` 固定写成 1；这在当前设备上是手动模式，不是自动模式，因此不能仅通过 YAML 恢复自动曝光。

### 6.2 海康 HikRobot 相机

海康后端使用 MVS SDK，不通过 `/dev/video*`。先在官方客户端确认设备和连续取流：

```bash
/opt/MVS/bin/MVS.sh
```

配置示例：

```yaml
camera_name: hikrobot
exposure_ms: 3
gain: 10.0
vid_pid: "2bdf:0001"
```

测试：

```bash
./build/camera_test \
  --config-path=configs/camera.yaml \
  --display
```

源码会关闭自动曝光和自动增益，将 `exposure_ms` 转为微秒写入相机，开启连续自动白平衡，并固定请求 150 FPS。`vid_pid` 只用于掉线后的 USB reset，不用于选择相机；连接多台海康时仍会打开 SDK 枚举到的第一台。

MVS 中应确认 `TriggerMode=Off`，像素格式为四种 Bayer8 之一。当前代码未主动设置这两个节点，并且不能处理 Mono8、RGB8、YUV 或 10/12 位 Bayer。

### 6.3 迈德威视相机

配置示例：

```yaml
camera_name: mindvision
exposure_ms: 2
gamma: 0.5
vid_pid: "f622:d13a"
```

仍使用 `camera_test`。源码关闭自动曝光，把曝光转换为微秒、把 gamma 乘 100 写入 SDK，设置 BGR8 输出、连续采集和低帧率档。该后端不读取 YAML 的 gain。

### 6.4 相机调试原则

三种后端都应遵循：

```text
厂商工具或系统工具验证设备
  -> 单独取流
  -> 参数回读
  -> 实时检测
  -> 相机标定
  -> 完整机器人入口
```

曝光、分辨率或焦距变化与标定的关系见 [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md)。

## 7. 配置文件的选择与修改

### 7.1 配置文件用途

| 文件 | 主要用途 | 注意事项 |
|---|---|---|
| `configs/demo.yaml` | 自带自瞄录像 | CPU 推理，和 demo 的相机标定匹配 |
| `configs/calibration.yaml` | 标定采集与计算 | 只含标定、相机和旧 CAN 所需字段 |
| `configs/camera.yaml` | 工业相机单测 | 当前默认迈德威视 |
| `configs/standard3.yaml` | 3 号步兵 | 包含新串口、MPC 和打符参数 |
| `configs/standard4.yaml` | 4 号步兵 | 包含新串口、MPC 和打符参数 |
| `configs/sentry.yaml` | 哨兵和 USB 全向相机 | 不包含完整打符/MPC字段 |
| `configs/uav.yaml` | 无人机 | 参数仅适用于对应硬件 |
| `configs/example.yaml` | 字段示例 | 不是当前所有入口都能直接运行的完整配置 |

不要把另一台机器人的外参、内参或发射偏置直接复制到新机器人。至少以下字段是设备实例相关的：

- `camera_matrix`
- `distort_coeffs`
- `R_camera2gimbal`
- `t_camera2gimbal`
- `R_gimbal2imubody`
- `yaw_offset`、`pitch_offset`
- 相机 `vid_pid`、曝光和增益
- CAN ID 或串口设备名

### 7.2 相对路径

模型路径如 `assets/yolov5.xml` 是相对当前工作目录解析，而不是相对 YAML 文件解析。因此应从仓库根目录运行：

```bash
./build/auto_aim_test
```

在 `build/` 目录内直接执行 `./auto_aim_test` 会让 `assets/...` 和 `configs/...` 指向错误位置，除非同时传入绝对路径并修改 YAML 中的模型路径。

### 7.3 CPU 与 GPU

首次运行建议：

```yaml
device: CPU
```

只有确认 OpenVINO 能枚举 Intel GPU 后再改成：

```yaml
device: GPU
```

自瞄模型读取 `device`；能量机关 `YOLO11_BUFF` 当前在源码中固定使用 `CPU`，不会跟随 YAML 的 `device`。

### 7.4 配置完整性

YAML 缺少字段时，代码通常直接调用 `.as<T>()` 并抛出异常。常见组合要求如下：

| 模块 | 必需字段组 |
|---|---|
| YOLO | 模型路径、`device`、阈值、ROI、传统检测参数 |
| Solver | 内参、畸变、相机到云台外参、云台到 IMU 轴变换 |
| Tracker/Aimer | 颜色、丢失帧数、延迟、进出装甲角、偏置 |
| Planner | `fire_thresh`、最大角加速度、`Q_yaw/R_yaw/Q_pitch/R_pitch` |
| Buff | `model`、`fire_gap_time`、`predict_time` 及 Solver 字段 |
| Camera | `camera_name`、曝光、设备 ID，以及对应后端的 gain/gamma |
| Gimbal | `com_port` |

当前 `auto_buff_test` 默认写的是 `configs/sentry.yaml`，但该文件没有 `model`、`fire_gap_time` 和 `predict_time`，直接使用默认值会在构造模块时失败。应传入包含打符字段的配置，例如 `configs/standard4.yaml`，同时确保录像和其中的相机标定匹配。

## 8. 测试程序索引

### 8.1 无硬件或低风险测试

| 程序 | 输入 | 退出方式 | 说明 |
|---|---|---|---|
| `auto_aim_test` | AVI + 姿态 TXT | 图像窗口按 `q` | 完整自瞄离线链路 |
| `auto_buff_test` | AVI + 姿态 TXT | `q` | 打符离线链路，需要完整打符配置 |
| `detector_video_test` | 单个视频 | `q` | YOLO/传统检测 |
| `planner_test_offline` | 合成参数 | `Ctrl+C` | MPC，不需要图像 |
| `calibrate_camera` | 编号 JPG | 每图任意键 | 内参标定 |
| `calibrate_handeye` | JPG + 四元数 TXT | 每图任意键 | 手眼外参计算 |
| `calibrate_robotworld_handeye` | JPG + 四元数 TXT | 每图任意键 | 同时估计外参与标定板世界位姿 |
| `split_video` | AVI + TXT | `q` | 同步裁剪录像和姿态 |

### 8.2 相机和感知硬件测试

| 程序 | 需要设备 | 说明 |
|---|---|---|
| `camera_test` | 海康或迈德威视 | 工业相机取流/FPS |
| `camera_detect_test` | 工业相机 | 实时检测 |
| `camera_thread_test` | 工业相机 | 多线程检测压力测试 |
| `usbcamera_test` | V4L2 USB 相机 | USB 取流 |
| `usbcamera_detect_test` | V4L2 USB 相机 | USB 实时 YOLO |
| `multi_usbcamera_test` | 两路 USB + 一路工业相机 | 固定使用 `video0`、`video2`，不是纯 USB 测试 |
| `dm_test` | 达妙 IMU | 姿态读取 |
| `minimum_vision_system` | 工业相机 + 达妙 IMU | 不依赖 CBoard 的最小系统 |

### 8.3 通信和执行机构测试

| 程序 | 协议 | 风险 |
|---|---|---|
| `cboard_test` | CBoard CAN / 本地 UART | 只读取；UART 仅姿态，通信配置用 `configs/serial_imu.yaml` |
| `gimbal_test` | 原 SP/CRC 串口 | 会持续发送云台控制；带 `-f` 时请求开火；不适用 new_uart 图片 |
| `gimbal_response_test` | 旧 CAN | 主动发送阶跃、三角波或复合运动命令 |
| `planner_test` | 新串口 | 让真实云台跟随合成目标，并可能满足开火条件 |
| `fire_test` | 新串口 | 周期发送开火模式，属于危险测试 |
| `handeye_test` | 工业相机 + CAN | 将世界网格重投影到画面以检查外参 |

任何主动控制测试都应先断开摩擦轮、取出弹丸、架空底盘、限制云台活动范围，并安排急停人员。

## 9. 从离线测试走到实车运行

推荐按以下顺序逐层增加硬件：

1. `auto_aim_test` 跑通自带 demo。
2. `detector_video_test` 验证自己的录像。
3. 完成相机内参标定。
4. `camera_test` 验证曝光、增益、帧率和掉线恢复。
5. `cboard_test` 或 `gimbal_test` 单独验证姿态和弹速。
6. 完成手眼标定并验证坐标轴方向。
7. 录制自己的 AVI/TXT，再次运行 `auto_aim_test`。
8. 架空机器人运行 debug 入口，关闭自动开火。
9. 低速、近距离验证云台方向和弹道。
10. 最后才启用自动开火和高速旋转目标。

旧协议入口：

```bash
./build/standard configs/standard3.yaml
```

新版串口和 MPC 入口：

```bash
./build/standard_mpc configs/standard3.yaml
```

二者不能仅凭名称互换。`standard` 使用 `CBoard` 和旧 `Command`；`standard_mpc` 使用 `Gimbal`，发送角度、角速度和角加速度。下位机协议必须与入口一致。

## 10. 日志、曲线和录像

### 10.1 PlotJuggler

`tools::Plotter` 将一行 JSON 通过 UDP 发送到：

```text
127.0.0.1:9870
```

PlotJuggler 需要配置 UDP Server/JSON 数据源监听该端口。仓库中的 `mpc_layout.xml` 和 `buff_layout.xml` 可用作已有曲线布局，但数据源仍需先连接。

UDP 没有接收端时 `sendto` 通常不会阻塞主流程，所以不打开 PlotJuggler 也能运行测试。

### 10.2 日志

日志可用于确认：

- 模型推理时间；
- 相机实际 FPS；
- 串口/CAN 是否收到数据；
- Tracker 状态变化；
- 相机或串口是否触发重连；
- 弹道是否无解。

性能分析时不要只看平均 FPS，还要记录图像时间戳到命令生成的延迟。高吞吐但队列积压仍会导致瞄准落后。

### 10.3 裁剪录像

`split_video` 会同步裁剪 AVI 和姿态 TXT：

```bash
./build/split_video \
  --start-index=100 \
  --end-index=300 \
  --output-path=records/clip \
  records/original
```

输入和输出都使用“不带扩展名的基名”。输出目录必须预先存在；当前代码不会为 `output-path` 自动创建父目录。

## 11. 新增相机或输入源

### 11.1 接入新的工业相机

项目统一接口为：

```cpp
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(
    cv::Mat & img,
    std::chrono::steady_clock::time_point & timestamp) = 0;
};
```

接入步骤：

1. 新建后端类并继承 `io::CameraBase`。
2. 在构造函数中打开设备、设置像素格式、曝光、增益和触发模式。
3. 在 `read` 中返回 BGR `cv::Mat` 和对应时间戳。
4. 在 `io/camera.cpp` 的工厂中增加新的 `camera_name` 分支。
5. 在 `io/CMakeLists.txt` 增加源文件、头文件目录和 SDK 库。
6. 增加独立相机测试，确认退出和断线重连不会死锁。
7. 为该相机重新做内参和手眼标定。

时间戳应尽量接近曝光时刻。若 SDK 能提供硬件时间戳，应建立它和 `steady_clock` 的映射；仅在取图函数返回后调用 `steady_clock::now()` 会把传输和缓存延迟也算入图像时间。

### 11.2 把 USB 相机接入完整主链路

可以让 `USBCamera` 实现或适配 `CameraBase`，再给工厂增加 `camera_name: usbcamera`。同时需要处理：

- 稳定设备路径，而不是易变化的 `video0`；
- 通用曝光/分辨率设置；
- 断线时不能让消费者永久阻塞；
- 图像格式和颜色顺序；
- 与主相机分辨率一致的内参；
- USB 缓冲导致的额外延迟；
- 帧时间戳和 IMU 时间基准。

只让编译通过不等于可以用于射击，延迟和标定误差会直接转化为瞄准误差。

### 11.3 让硬件后端真正可选

若目标是在没有任何厂商 SDK 的开发机上构建，建议引入 CMake 选项：

```text
ENABLE_HIKROBOT
ENABLE_MINDVISION
ENABLE_USB_CAMERA
ENABLE_ROS2
```

每个选项分别控制源文件、头文件、链接库和工厂分支。离线目标只链接算法和通用工具，不应链接 `io::Camera`。这是当前仓库尚未完成的构建层改进，不要把这些选项当成已经存在的命令使用。

## 12. 推荐开发流程

### 12.1 修改检测器

1. 使用固定短视频作为回归输入。
2. 运行 `detector_video_test` 检查候选数量和关键点。
3. 检查模型输入尺寸、颜色顺序、缩放和 padding。
4. 检查关键点顺序是否与 Solver 的物点顺序一致。
5. 再运行 `auto_aim_test` 检查 PnP 和 Tracker 是否退化。

### 12.2 修改 Solver 或坐标变换

1. 保留原始像素点和 PnP 输出。
2. 检查重投影误差。
3. 用静止目标转动云台，世界坐标中的目标应基本不动。
4. 分别检查相机坐标、云台坐标和世界坐标，避免一次跨越三层排错。
5. 更改轴定义后重新核对 `R_gimbal2imubody`。

### 12.3 修改 Tracker/EKF

1. 先使用可重复的 AVI/TXT。
2. 输出观测和估计，不要只看最终命令。
3. 检查 `dt`、残差、半径、角速度和装甲 ID。
4. 对丢帧、装甲切换、跨越 `-pi/pi` 和视频时间跳变单独测试。
5. 不要只用当前 NIS/NEES 阈值判断统计一致性，详见项目介绍中的实现注意事项。

### 12.4 修改 Planner 或开火逻辑

1. 先运行 `planner_test_offline`。
2. 扫描距离 `d` 和角速度 `w`。
3. 检查角度、角速度、角加速度是否连续。
4. 再连接架空云台运行 `planner_test`。
5. 最后在无弹、低速条件下检查开火信号时序。

## 13. 常见问题

### 13.1 找不到 OpenVINO

典型错误：`Could not find OpenVINOConfig.cmake`。

检查：

```bash
ls /opt/intel/openvino_2024.6.0/runtime/cmake
```

如果版本或路径不同，需要修改项目中硬编码的 `OpenVINO_DIR`。仅执行其他版本的 `setupvars.sh` 可能仍会被硬编码路径覆盖。

### 13.2 找不到模型

典型错误发生在 `core.read_model(...)`。确认：

```bash
pwd
ls assets/yolov5.xml assets/yolov5.bin
```

OpenVINO IR 的 `.xml` 与 `.bin` 必须成对存在。建议从仓库根目录启动程序。

### 13.3 GPU 编译或运行失败

先把 YAML 的 `device` 改为 `CPU`。CPU 能运行说明模型和前后处理基本正常，之后再处理 Intel GPU runtime、权限或驱动问题。

### 13.4 YAML 报 `bad conversion` 或 `invalid node`

说明当前入口读取了配置中不存在或类型不匹配的字段。不要只参照 `example.yaml`；用 `rg` 搜索构造函数实际读取的键，并从对应机器人完整配置复制字段结构，再填写本机数据。

### 13.5 视频有画面但 PnP 距离错误

常见原因：

- 使用了别的相机内参；
- 视频分辨率和标定分辨率不同；
- 图像被裁剪或缩放但内参未变换；
- 关键点顺序错误；
- 大小装甲分类错误；
- 镜头重新对焦或更换。

### 13.6 `auto_aim_test` 很快结束或姿态异常

检查 AVI 是否能打开、TXT 是否存在、文本行数是否足够、每行是否为五个数，以及四元数顺序是否为 `w,x,y,z`。

### 13.7 无图形界面时报错

`auto_aim_test`、`detector_video_test` 和标定程序都会调用 `imshow`。纯 SSH 终端需要 X11 转发、NoMachine 等远程桌面，或使用：

```bash
xvfb-run -a ./build/auto_aim_test
```

虚拟显示可以让程序运行，但看不到标定角点时不适合交互式筛选标定图片。

### 13.8 USB 摄像头卡住

`USBCamera::read` 会等待内部队列。如果设备没有成功产生帧，主线程可能一直等待。先用系统工具确认设备：

```bash
v4l2-ctl --device=/dev/video0 --all
```

再检查是否被其他进程占用、是否支持 MJPG、分辨率和帧率组合是否有效。

### 13.9 工业相机动态库找不到

检查：

```bash
ldd build/camera_test | grep -E 'MvCameraControl|MVSDK|not found'
```

构建文件会把仓库内对应架构的库目录写入 RUNPATH；如果复制单个可执行文件到别处，原绝对 RUNPATH 可能失效。部署时应一起安装 SDK，或设置稳定的系统库搜索路径。

### 13.10 配置了 ROI 后崩溃

ROI 必须完全落在输入图像内。分辨率变化后，原配置的 `x/y/width/height` 可能越界。排错时先设：

```yaml
use_roi: false
```

## 14. 安全注意事项

- `gimbal_test -f`、`fire_test`、`planner_test` 和完整机器人入口可能发送真实运动或开火命令。
- 首次通信测试必须取出弹丸并关闭摩擦轮供电。
- 云台响应测试会主动改变目标角，必须架空机器人并清理机械活动范围。
- 配置文件中的 `auto_fire: false` 只影响使用 `Shooter` 的旧链路，不能假设它会屏蔽所有新协议测试程序。
- 调试串口/CAN 时应准备独立急停手段，不能只依赖视觉进程正常退出。
- 手眼标定采集需要转动云台，应避免标定板、线缆和枪管发生碰撞。

完成上述离线、单设备和标定验证后，再进入实弹调参与命中率测试。这样可以把模型、几何、时间同步、通信和机械问题分层定位。
