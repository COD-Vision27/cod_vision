# 本机使用仿真

在本机桌面终端执行。下面的命令不使用 SSH；首次只开 Godot 窗口，不需要编译 SP，也不需要 ROS 或 OpenVINO。

## 首次准备与开窗口

```bash
cd "/home/kendas/桌面/zimiao"
python3 rm_aim_sim/tools/setup_godot.py
bash rm_aim_sim/run.sh --headless --editor --import --quit
bash rm_aim_sim/run.sh --max-fps 60 --print-fps
```

第一条 Python 命令下载校验过的 Godot 4.7.2 到 `rm_aim_sim/.tools/godot`，只需首次执行；导入资源也只需首次执行。后续直接运行最后一条命令。当前项目默认使用 OpenGL Compatibility，本机已有的 Intel Arc/OpenGL 硬件加速已做过只读查询，实际帧率仍要运行后测量。

终端的 `Project FPS` 是 Godot 画面帧率；`--max-fps 60` 是上限，不代表一定达到 60 FPS。

- 方向键：转动云台。
- 按住 F：内置辅助跟踪。
- 按住 Space：发射。
- P：暂停/继续；R：重置；V：切换视角。
- 左侧开关和滑条：设置平移、自转等参数。

F 使用仿真内部真值辅助，不是 SP 图像检测，不能用它的命中率代替 SP 自瞄成绩。首次手动操作保持“本机自瞄控制”关闭，以免外部控制模式接管方向键和发射。

## 同时在本机运行 SP 图像自瞄

需要先准备 C++ 编译环境及 OpenCV、Eigen、fmt、spdlog、yaml-cpp、nlohmann-json、OpenVINO 的开发依赖。源代码和模型已在本项目中。OpenVINO 默认查找路径为 `/opt/intel/openvino_2024.6.0/runtime/cmake`，不同位置通过 `OpenVINO_DIR` 指定。

先关闭不需要的程序，释放足够内存，再在本机首次构建；使用单编译任务降低内存并发占用：

```bash
cd "/home/kendas/桌面/zimiao"
# 前面的 Godot 下载和资源导入也需要先完成
SP_BUILD_JOBS=1 bash sp_vision_sim/build.sh

# 静态目标，运行 30 秒，同时打开本机窗口和 SP
python3 tools/aim_lab.py --local --scenario static --seconds 30
```

如果 OpenVINO 安装位置不同：

```bash
OpenVINO_DIR=/actual/openvino/runtime/cmake SP_BUILD_JOBS=1 bash sp_vision_sim/build.sh
```

切换场景：

```bash
python3 tools/aim_lab.py --local --scenario spin --seconds 30
python3 tools/aim_lab.py --local --scenario move --seconds 30
```

`--local` 是显式的本机运行开关；不加时仍要求远端调度工作区，防止误在本机启动。这个入口会自动同时启动 Godot 和 SP，不必再手动打开一个仿真窗口或按 F。到时自动结束，Ctrl+C 可提前停止。当前完整接入使用 Vulkan Mobile，需要可用的 Vulkan 驱动；OpenGL 查询成功本身不等于已经验证 Vulkan 和完整 SP 链路。

结果保存到 `artifacts/local-aim/`：`*-result.json` 含画面 FPS、视觉处理 FPS、观测年龄和命中统计；逐帧证据默认 gzip 压缩。100 Hz 控制循环不能当作画面/检测 FPS。

2026-10-21 已核对启动参数，并验证本机 opt-in 和默认远程限制的轻量回归。本轮没有下载安装引擎、进行本机大型构建或启动仿真，尚无本机 FPS 实测。
