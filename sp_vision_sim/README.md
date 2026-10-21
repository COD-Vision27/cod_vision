# SP Vision / Godot 本机接入

Godot 仿真和 SP Vision 均在当前电脑运行，通过 `/dev/shm` 交换实际 RGB 图像、同帧相机标定与云台姿态，以及控制命令。仿真源码来自 [ForgetWW520/rm_aim_sim](https://github.com/ForgetWW520/rm_aim_sim)，固定提交见 `../rm_aim_sim/UPSTREAM.md`，保留 MIT 许可。

```bash
python3 rm_aim_sim/tools/setup_godot.py
bash rm_aim_sim/run.sh --headless --editor --import --quit
python3 tools/local_run.py build
python3 tools/aim_lab.py --scenario static
python3 tools/local_run.py test
```

完整依赖与窗口操作见 [本机说明](../docs/local-simulator.md)。默认单编译任务；需要 C++17、CMake、OpenCV/Eigen/fmt/spdlog/yaml-cpp/nlohmann-json/OpenVINO 和可用 Vulkan 驱动。当前仿真不需要 ROS、Cargo 或 Bevy。

在线视觉输入不读取真值装甲位置，走 YOLO → Solver/PnP → Tracker/EKF → Planner/TinyMPC。100 Hz 控制循环与视觉线程分开，待处理图像最多一帧；目标过期、暂停、重启或标定变更时清空旧目标。每个实例使用独立 IPC channel，规划与图像采用同一模拟时钟，墙钟用于连接和目标看门狗。

OpenVINO 推理与 OpenCV 线程数在 YAML 中配置，按本机能力调整。当前比赛预设为 350 HP、单发 20 伤害；`--health` 是实验覆盖项，不得混用不同血量工况比较 TTK。

`--seconds` 设置有限时长，默认持续运行；`--max-fps` 设置渲染上限，0 表示不限。结果保存到 `artifacts/local-aim/`，包括首帧、处理耗时、逐发事件、输入摘要和命中统计；渲染 FPS、视觉 FPS 与控制频率分别记录。实际命中并不等于实机验收。

相机标定按同步帧元数据生成临时 YAML，不覆盖实机标定。引擎二进制、导入缓存、日志和机器信息均不得提交。
