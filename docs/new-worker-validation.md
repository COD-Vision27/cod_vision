# 更换仿真主机与快速验证

适用当前 Godot + SP 共享内存接入，主机为 Linux x86_64。测试不要求 NVIDIA 品牌显卡，但需要可用的 Vulkan 图形驱动。Godot、视觉算法和共享内存必须在同一台新主机运行。Windows/macOS 不属于当前自动准备脚本支持范围。

## 新机器一条验证命令

先配置能免密连接的新 SSH 别名，例如 `sim-new`，并完成正常的首次连接/主机指纹确认。`--user` 请替换为新电脑的真实用户名；只改 host 会继续使用默认的 `wuxian`。

```bash
python3 tools/remote_run.py --host sim-new --user YOUR_USER --seconds 15 validate
```

此命令先同步源码，然后依次执行：

1. 检查 Linux/架构、编译工具、显示或 Xvfb，以及 OpenCV/Eigen/fmt/spdlog/yaml-cpp/OpenVINO/nlohmann-json 的 CMake 依赖发现。
2. 校验固定 Godot 引擎，缺失时下载并校验 SHA256；初始化导入缓存。
3. 在新主机首次构建 SP，执行 C++ 回归、源码审计、Godot 物理与通信测试。
4. 各运行 15 秒静态、旋转、横移真实图像闭环，回传结果及压缩后的逐帧证据。
5. 每个 `*-result.json` 的 `performance` 分别报告 **渲染 FPS、视觉处理 FPS、处理耗时、控制使用的观测年龄、模拟时间/墙钟时间比例和实际图形设备**。

这里的 15 秒是每个闭环采样时间，**不是保证整套准备在 15 秒内完成**。首次下载、导入和构建另计；空白系统若缺少依赖，会列出缺失项并停止，不会擅自安装系统包或回退到本机构建。无需安装 ROS/Cargo。

默认 OpenVINO 查找路径为 `/opt/intel/openvino_2024.6.0/runtime/cmake`。新机器安装在其他位置时显式指定：

```bash
python3 tools/remote_run.py --host sim-new --user YOUR_USER \
  --env OpenVINO_DIR=/actual/openvino/runtime/cmake --seconds 15 validate
```

直接下载 GitHub 不通时，将同一官方 ZIP 放到新主机，使用校验过的离线安装方式：

```bash
python3 tools/remote_run.py --host sim-new --user YOUR_USER run -- \
  python3 rm_aim_sim/tools/setup_godot.py --archive /absolute/path/godot.zip
```

再重跑 `validate`。不能直接复制另一台电脑的 SP 构建目录来代替首次构建；项目同步本来就排除 build、引擎、驱动、虚拟环境和凭据。

## 已准备好的机器快速复测

```bash
# 一个静态场景，复用已构建程序；仍检查环境、引擎和回归
python3 tools/remote_run.py --host sim-new --user YOUR_USER \
  --skip-build --scenario static --seconds 10 validate

# 只测真实图像闭环，省去准备与回归步骤
python3 tools/remote_run.py --host sim-new --user YOUR_USER \
  --skip-build --scenario static --seconds 15 closed-loop

# 本地浏览器查看新主机窗口
python3 tools/sim_view.py --host sim-new --user YOUR_USER --scenario static
```

若希望后续命令默认使用新机器，在当前终端设置：

```bash
export ZIMIAO_REMOTE_HOST=sim-new
export ZIMIAO_REMOTE_USER=YOUR_USER
```

结果位置为本机 `artifacts/remote/<作业ID>/project-artifacts/`，不会把远端编译缓存拉回本机。`host-preflight.json` 只说明依赖检查；真实驱动和图像链路仍要看闭环结果。若 `software_renderer` 为 true，说明落到了软件渲染；为 null 表示没拿到设备信息，不能视作确认硬件加速。

## 如何理解帧率

- **渲染 FPS**：Godot 日志中实际绘图帧率的样本中位数。
- **视觉处理 FPS**：C++ 完成处理的图像数 / 本次运行墙钟秒数，包含启动及击毁后的帧。
- **100 Hz**：控制循环请求周期，不能当成渲染或检测 FPS。
- **模拟时间 / 墙钟时间**：远小于 1 时，物理时间跑得比现实慢；不能只看击杀的模拟秒数认为达到实时。

过去 RTX 4060 主机完整闭环实测约 17–23 个处理帧/秒；这不是本机核显成绩，也不是渲染 FPS。新机器测出的配置、显卡设备、耗时和原始事件会随报告保存，便于比较。

2026-10-21 只读查看本机：Ultra 5 125H、Intel Arc 核显、16 GB 内存，OpenGL 报告硬件加速正常。查看时可用内存约 323 MB、2 GB swap 已满。本轮没有在本机启动仿真或编译，所以没有可引用的本机 FPS；仅凭型号不能给可靠数字。应先释放工作负载，再进行本机短测，避免把换页卡顿误判为引擎性能。

本入口的 4 项指标/编排/缺失依赖回归与 7 项调度回归已通过。2026-10-21 对既有 `wuxian-remote` 的实际试跑在 SSH 转发阶段返回 `Connection refused`，尚未完成新增 validate 流程的远端端到端验证；没有在本机启动替代仿真。
