# RoboMaster 自瞄与仿真工作区

本项目在当前电脑运行 Godot 仿真和 SP Vision 图像自瞄，通过同机共享内存通信。公开版本不包含远程调度、服务器连接或私人机器配置。

## 本地准备

需要 Linux x86_64、Python 3.10+、图形桌面和可用的 Vulkan 驱动。完整 SP 接入还需要 CMake、C++17 编译器、OpenCV、Eigen、fmt、spdlog、yaml-cpp、nlohmann-json 和 OpenVINO 开发依赖。

```bash
# 在项目根目录执行，首次准备 Godot 并导入资源
python3 rm_aim_sim/tools/setup_godot.py
bash rm_aim_sim/run.sh --headless --editor --import --quit

# 仅启动本机仿真窗口
python3 tools/local_run.py sim

# 完整自瞄：先在本机构建，再启动仿真和 SP
python3 tools/local_run.py build
python3 tools/aim_lab.py --scenario static
```

构建默认单任务，按本机能力设置 `SP_BUILD_JOBS`。默认持续运行、关闭窗口或 Ctrl+C 停止；使用 `--seconds 30` 定时，`--max-fps 60` 设置渲染帧率上限。

```bash
python3 tools/local_run.py doctor
python3 tools/local_run.py check
python3 tools/local_run.py test
python3 tools/aim_lab.py --scenario spin --seconds 30 --max-fps 60
```

`test` 会在本机构建并执行 C++ 测试、Godot 测试和真实图像闭环，需要先准备引擎与全部依赖。检查通过不等于实机验收；渲染、图像处理和控制频率分别计量。

- [本机使用说明](docs/local-simulator.md)
- [SP / Godot 接入与时序](sp_vision_sim/README.md)
- [全部文档目录](docs/文档目录.md)
- [实机标定与部署](sp_deploy/README.md)
- [训练流水线](sp_training/README.md)
- [源码与版本管理](docs/version-control.md)

运行结果保存在已忽略的 `artifacts/` 中。私人配置放在 `.local/` 或 `*.local.json` 等忽略文件中；不要将真实地址、账号或机器路径写入公共示例。
