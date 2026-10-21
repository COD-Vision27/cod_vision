# 本机使用仿真

所有命令均在克隆后的项目根目录执行，不需要 SSH、服务器账号或端口转发。

## 引擎与窗口

当前下载助手支持 Linux x86_64。需要图形桌面及兼容的图形驱动。先安装 Python 依赖（建议使用本地虚拟环境）：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pyyaml
python3 rm_aim_sim/tools/setup_godot.py
bash rm_aim_sim/run.sh --headless --editor --import --quit
python3 tools/local_run.py sim
```

Godot 下载助手校验固定版本 ZIP 的 SHA256，二进制与导入缓存不会入库。仅仿真窗口不要求 SP、ROS 或 OpenVINO。内置 F 辅助使用模拟器内部真值，不能用其成绩代替 SP 图像检测成绩。

## 本机 SP 图像闭环

安装 CMake、C++17、OpenCV/Eigen/fmt/spdlog/yaml-cpp/nlohmann-json/OpenVINO 开发依赖。默认 OpenVINO 路径见 `sp_vision_sim/build.sh`；如不同，用 `OpenVINO_DIR` 指定。

```bash
# 默认单编译任务；可按当前电脑能力设置 SP_BUILD_JOBS
python3 tools/local_run.py doctor
python3 tools/local_run.py build
python3 tools/aim_lab.py --scenario static
python3 tools/aim_lab.py --scenario spin --seconds 30 --max-fps 60
python3 tools/aim_lab.py --scenario move
```

Godot 和 C++ 客户端均在当前电脑运行，通过 `/dev/shm` 交换图像和控制数据。完整闭环使用 Vulkan Mobile，需要可用的 Vulkan 驱动；OpenGL 可用不代表 Vulkan 已验证。`--local` 仅保留为兼容参数，不改变执行位置。

默认持续运行、不限渲染帧率；关闭窗口或 Ctrl+C 停止。`--seconds N` 限制运行时长；`--max-fps N` 限制渲染 FPS，0 表示不限。实际渲染和视觉帧率取决于本机负载，控制循环标称频率不能代替两者。

## 本机测试

```bash
python3 tools/local_run.py check
python3 tools/local_run.py test
# 复用构建，只验证一个场景
python3 tools/local_run.py closed-loop --skip-build --scenario static --seconds 25
# 仅打印计划
python3 tools/local_run.py test --dry-run
```

`check` 校验审核过的源码清单。`test` 包括本机构建和 CTest、Godot 单元/通信测试及三个真实图像闭环，需准备全部依赖和图形会话。无桌面的环境可以自行使用 Xvfb；仍需要实际图形驱动。

结果、日志、首帧截图和逐帧 gzip 数据保存在已忽略的 `artifacts/local-aim/`。本地环境检查结果保存到 `artifacts/local-preflight.json`。发布前不要强制加入这些可能记录机器路径和硬件信息的文件。

公开版本未在任意读者机器上预先验收；实际帧率与闭环结果须在其本机复测。
