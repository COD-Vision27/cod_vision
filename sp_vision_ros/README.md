# SP Vision 新版自瞄 ROS2 仿真接入

> **2026-10-15 已完成远端验收。** 已先回滚并验证原版，再接入新版 ROS 自瞄；标准 3 派生配置的静态、旋转、横移闭环均通过，标准 4 派生配置的三个长时闭环也通过。原 SP 的 230 个文件、原仿真配置与资产未改动。详见 [测试说明](docs/测试说明.md)与[远端基线记录](../docs/simulator-baseline.md)。

本目录独立编译 `sp_vision_25` 中已有的 YOLO、Solver、Tracker、Target/EKF、Planner 和 TinyMPC 源文件，通过 ROS2 接入 `bevy_robomaster_simulator`。不修改实机项目的源码、CMakeLists.txt、standard3.yaml 或 standard4.yaml，不构造 Camera/CBoard/Gimbal，不打开实机相机、串口或 CAN。

详细说明见 [使用报告](docs/使用报告.md) 和 [测试说明](docs/测试说明.md)。

运行配置统一位于 **[configs/](configs/README.md)**：`standard3.yaml`、`standard4.yaml`、调优候选 `standard3_high_rate.yaml` 和 Bevy 的 `simulator.toml`。默认启动 standard3；测试调优参数可直接执行：

```bash
python3 tools/sim_view.py --profile standard3_high_rate --scenario spin --seconds 180
```

构建和测试不会重新生成或覆盖配置。自定义 YAML / TOML、参数分工和几何刷新见 [configs 说明](configs/README.md)。

本电脑性能有限，构建和测试优先使用工作区根目录的 SSH 调度入口，将任务发送到 `wuxian@192.168.1.102`：

```bash
python3 tools/remote_run.py doctor
python3 tools/remote_run.py check
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true test
```

两端一起远程运行并显示窗口（避免分别启动长期作业占用同一个调度锁）：

```bash
# 目标机运行，本地浏览器看，默认静态演示 120 秒
python3 tools/sim_view.py

# 在目标机已登录的桌面显示；自动选择显示号及认证
python3 tools/sim_view.py --view remote

# standard4 派生配置 + 横移目标
python3 tools/sim_view.py --base standard4 --scenario move --seconds 180
```

详细操作、停止方法与验证记录见[可视化远程仿真测试](../docs/可视化远程仿真测试.md)。该入口复用已构建程序；修改源码后先远程构建。全部文档见[目录](../docs/文档目录.md)。无可见窗口的自动实验仍可通过 `remote_run.py` 与 `run_pair.sh` 调度。

连接配置、结果回传和目标依赖见 [远程测试说明](../docs/remote-testing.md)。下方直接调用构建与运行脚本的命令在当前机器执行。

## 当前机器启动

在 `/home/kendas/桌面/zimiao` 下执行：

```bash
# 更新仿真内外参（仅在模型、分辨率或 FOV 变化后执行；保留算法调参）
python3 sp_vision_ros/scripts/derive_config.py --geometry-only --output sp_vision_ros/configs/standard3.yaml

# 构建 ROS 消息、SP 适配器、仿真器
bash sp_vision_ros/scripts/build.sh

# 终端 1：新版自瞄
bash sp_vision_ros/scripts/run_aim.sh

# 终端 2：仿真（默认自动接收自瞄指令，第一人称相机）
bash sp_vision_ros/scripts/run_sim.sh
```

目标机使用 Ubuntu 24.04 / ROS2 Jazzy。脚本自动选择系统 ROS2（Humble 或 Jazzy），也可用 `SP_ROS_DISTRO` 指定。如果没有系统 ROS，使用本目录 `.runtime` 中的 RoboStack 环境。两端必须使用相同的 `ROS_DOMAIN_ID`（默认 25）和 RMW。不要同时向 `/rm_gimbal/cmd` 发送另一套算法指令。

`run_sim.sh` 使用本目录生成的 `configs/simulator.toml`，关闭无关的 Livox 深度捕获；保留原阴影、照明和物理子步设置；原仿真 `config.toml` 不变。运行目录在 `artifacts/sim-runtime`，资产通过符号链接引用原模型。

默认 `DAEDALUS_AUTO_AIM=1` 强制接收自瞄，方便自动实验。要用 F5 手动切换，运行 `DAEDALUS_AUTO_AIM=0 bash sp_vision_ros/scripts/run_sim.sh`。

## 可重复闭环实验

```bash
source sp_vision_ros/scripts/env.sh
python3 sp_vision_ros/tests/closed_loop.py --scenario static --seconds 25
python3 sp_vision_ros/tests/closed_loop.py --scenario spin --seconds 25
python3 sp_vision_ros/tests/closed_loop.py --scenario move --seconds 25
```

测试模式将蓝方步兵置于前方 4 米，并初始化己方枪口向下 9°，避开自身枪管遮挡；`spin` 旋转敌方底盘，`move` 让敌方通过原物理驱动跟随 ±0.8 米的横移参考（4 秒后开始）。正常运行不设置 `DAEDALUS_SP_SCENARIO`，保留原场景和交互行为。每次实验写出结果 JSON、诊断 JSONL、仿真命中遥测和最后一帧图像。

## ROS 接口

| 话题 | 消息 | 方向 | 内容 |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | 仿真 → SP | RGB8 图像 |
| `/camera_info` | `sensor_msgs/CameraInfo` | 仿真 → SP | 当前相机 K、D |
| `/sp_vision/sim/frame_tf` | `tf2_msgs/TFMessage` | 仿真 → SP | 与图像完全相同时间戳的枪口姿态、相机外参 |
| `/sp_vision/sim/plan` | `rm_interfaces/SpGimbalPlan` | SP → 仿真 | 控制、开火、两轴角度/速度/加速度、有效期和加速度限制 |
| `/sp_vision/sim/diagnostics` | `std_msgs/String` | SP → 调试 | JSON：检测、跟踪、规划、实际反馈误差 |

消息定义在仿真项目 `rm_interfaces/msg/SpGimbalPlan.msg`。旧 `/rm_gimbal/cmd` 接口仍保留。

## 不依赖 ROS 的检查

```bash
cmake -S sp_vision_ros -B sp_vision_ros/build-offline \
  -DSP_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build sp_vision_ros/build-offline -j2
ctest --test-dir sp_vision_ros/build-offline --output-on-failure
python3 sp_vision_ros/tests/test_geometry.py
python3 sp_vision_ros/scripts/check_sp_unchanged.py
```

几何生成器和 Python 几何测试需要 NumPy。ROS 联调使用 `rclpy`；闭环实验截图需要 Python OpenCV。仿真构建需要 Rust、Vulkan 驱动和 ALSA/udev 开发文件。

## 其他机器环境

目标机已配置 Jazzy；Ubuntu 22.04 使用 Humble，Ubuntu 24.04 使用 Jazzy。另需 `ros-humble-rosidl-default-generators`、`ros-humble-tf2-msgs`、`ros-humble-visualization-msgs`、`libasound2-dev`、`libudev-dev`，`libclang-dev`、`nlohmann-json3-dev`，以及原 SP 项目已有的 OpenCV/Eigen/yaml-cpp/fmt/spdlog/OpenVINO。

不具备 sudo 权限时，可创建独立环境：

```bash
conda create -y -p sp_vision_ros/.runtime --solver libmamba \
  --override-channels -c robostack-staging -c conda-forge \
  ros-humble-ros-base ros-humble-tf2-msgs ros-humble-visualization-msgs \
  ros-humble-rosidl-default-generators ros-humble-rmw-cyclonedds-cpp \
  cmake ninja colcon-common-extensions alsa-lib libudev pkg-config
```

脚本针对当前 SP 的系统依赖路径编写；其他 OpenVINO 安装版本需调整 `scripts/build.sh` 和 `scripts/env.sh`。为避免 ROSIDL 对中文路径的解析问题，构建脚本使用 `$HOME/.cache/sp-vision-sim/workspace` 的 ASCII 符号链接，原文件位置不变。


实机标定与一键部署工具见 [sp_deploy](../sp_deploy/README.md)；实际相机/电控采集与实机命中尚未执行，不能把仿真资格当作实机成绩。
