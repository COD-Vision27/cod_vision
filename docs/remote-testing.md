# SSH 远程运行与测试

## 当前连接：服务器中转（2026-10-18）

调度与可视化默认使用本机已有的 SSH 别名 `wuxian-remote`：本机 → `ubuntu@111.229.26.21:22` → 服务器 loopback 反向隧道端口 `2222` → 原 `wuxian` RTX 4060 目标机。服务器只转发 SSH，项目构建、ROS 和 Vulkan 渲染仍在目标机执行。`ssh wuxian-remote` 已确认原主机名与 RTX 4060。

```bash
ssh wuxian-remote
python3 tools/remote_run.py doctor
python3 tools/sim_view.py --profile standard3_high_rate
```

脚本未显式指定 `--port` 时遵从 `~/.ssh/config`，因此不会用 22 覆盖别名中的目标端口 2222。`--host` 接受 SSH 别名或地址；回到原局域网时可显式使用 `--host 192.168.1.102 --port 22`。SSH 配置和私钥不随项目同步。其他电脑需自行配置同等 SSH 别名及认证，不能仅将 `--host` 改成中转服务器公网 IP 就在服务器上运行项目。

本机只负责编辑、同步和调度；项目命令在 `wuxian@192.168.1.102` 执行。默认远程目录为 `/home/wuxian/.cache/zimiao-remote/workspace`，无需两个电脑具有相同路径。调度程序不自动安装依赖，也不在连接失败后自动启动本机编译或仿真。

要直接看到仿真窗口，推荐同时启动仿真和自瞄的可视化入口：

```bash
python3 tools/sim_view.py               # 目标机计算，本地浏览器看
python3 tools/sim_view.py --view remote # 目标机计算，目标机桌面看
```

默认静态场景运行 120 秒，Ctrl+C 提前停止。详细参数和实际验证见[可视化远程仿真测试](可视化远程仿真测试.md)，全部资料见[文档目录](文档目录.md)。普通 Xvfb 自动测试不显示可见窗口。

## 快速使用

从工作区根目录执行，所有调度参数放在动作名称之前：

```bash
# 连接、同步并报告远端依赖
python3 tools/remote_run.py doctor

# 使用系统 Python 执行几何和实机项目完整性检查
python3 tools/remote_run.py check

# 构建 ROS 消息、SP 适配器和仿真器
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true build

# 完整测试：构建、几何、完整性、ROS smoke、Rust、static/spin/move 闭环
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true test

# 不依赖 ROS 的 C++ 构建、CTest 与轻量检查
python3 tools/remote_run.py offline

# 复用上次远端构建，只跑一个闭环场景
python3 tools/remote_run.py --skip-build --scenario static --seconds 25 closed-loop

# 默认先远端构建，再启动仿真；无 DISPLAY 时使用隐藏的 Xvfb
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true sim -- --duration-secs 10

# 单独启动自瞄节点
python3 tools/remote_run.py --skip-build aim

# 自定义命令，参数逐个传递，不经过隐式 shell 展开
python3 tools/remote_run.py run -- python3 bevy_robomaster_simulator/tools/test_metrics_summary.py

# 只查看计划，不连接、不执行
python3 tools/remote_run.py --dry-run test
```

VS Code 的默认构建/测试任务也调用该调度程序。直接调用原 `build.sh`、`cargo` 等命令仍在当前电脑运行，请使用上述入口卸载计算负载。工作区 `AGENTS.md` 指定后续助手优先使用远程入口。

## SSH 与配置

使用已有 SSH 密钥、ssh-agent 和 `~/.ssh/config`，不会同步私钥。首次连接请在本机终端核对目标主机指纹并完成免密配置：

```bash
ssh wuxian@192.168.1.102
# 若目标尚未授权本机公钥：在终端交互输入远端密码
ssh-copy-id wuxian@192.168.1.102
```

调度使用 `BatchMode=yes`、严格主机密钥验证和连接超时，认证失败会退出。可通过 `--host`、`--user`、`--port`、`--identity`、`--remote-dir` 修改连接，或者设置 `ZIMIAO_REMOTE_HOST` / `ZIMIAO_REMOTE_USER`。

`--python` 指定远程测试 Python；它不负责激活 ROS 环境。`--env KEY=VALUE` 可重复使用，为远程进程设置环境变量。`--jobs` 默认为 8，传给 CMake/Rust 构建。原构建脚本直接使用时仍默认为 2。

运行参数统一位于 [sp_vision_ros/configs](../sp_vision_ros/configs/README.md)。构建、测试均不再隐式执行配置生成器，手动调整的 YAML / TOML 会原样同步。`check` / `test` 检测到调优 agent 的 `tuning-source-manifest.json` 时调用其独立的 `check_tuned_sources.py` 审计，验证已授权调优版本；否则使用原 230 文件基线。原接入基线和历史结果不改写。

## 同步、排队与结果

- 使用 rsync 增量同步源码、模型和资产；包括未提交的编辑。跳过 Git、密钥/常见凭据文件、本机构建产物、Conda/虚拟环境和运行日志，不复制机器相关的本地二进制或 CMake 缓存。
- 实机项目完整性基线 `sp_vision_ros/artifacts/sp_vision_baseline.json` 是测试输入，单独保留并同步。
- 同步使用 `--delete` 删除远端已从本地移除的源码；被排除的远端构建缓存和环境会保留。只允许使用调度程序创建并标记的专用目录，拒绝覆盖已有的非调度目录。
- 同一个远程目录通过文件锁串行执行，锁覆盖同步、构建与测试，避免两个作业混用源码。默认等候 600 秒，可用 `--lock-timeout` 调整。交互式长期运行会占用锁；有需要时为独立实例指定另一 `--remote-dir`，并使用不同 `ROS_DOMAIN_ID`。
- 实时输出保存到本机 `artifacts/remote/<时间戳-作业ID>/scheduler.log`。远端 `job.log`、`result.json` 和本次写出的项目报告、JSONL、截图回传到同一目录，即使测试失败也尝试回传。旧报告不会作为新测试结果返回。
- SSH 中断时尝试终止该作业启动的进程组并释放锁，不终止机器上的其他项目进程。退出码反映远端任务失败；结果回传失败也会使成功任务的调度退出非零。

远端保留结果在 `<remote-dir>/artifacts/remote-runs/<作业ID>/`。本机 `dispatch.json` 记录目标、失败阶段和调度退出码。`doctor` 用于报告环境，不代表完整测试已通过。

调度程序自身的轻量回归检查：`python3 -m unittest discover -s tools -p test_remote_scheduler.py -v`。覆盖串行锁与释放、失败退出码、仅回传本次产物、拒绝覆盖已有目录、断连停止作业，以及同步时保留缓存和排除凭据。

## 目标机器的环境要求

2026-10-15 更新：目标是 Ubuntu 24.04，16 逻辑核心、32 GB 内存、RTX 4060 8 GB。经用户授权，已经安装 CMake、Rust 1.99.0、ALSA/udev 开发依赖、Vulkan 工具、Xvfb，并修复内核与 NVIDIA 模块不匹配问题。`nvidia-smi` 返回驱动 595.99.02，Vulkan/vkcube 确认使用 RTX 4060。系统 Python 已安装 NumPy/OpenCV；原有 `/home/wuxian/miniconda3/envs/YOLO/bin/python` 也可用于几何检查。已安装 ROS2 Jazzy（与 Ubuntu 24.04 匹配）和 OpenVINO 2024.6.0，并验证 NumPy/OpenCV/rclpy 导入。ROS 适配脚本可选择系统 Humble/Jazzy；`SP_ROS_DISTRO` 可显式指定。

原仿真改动已先回滚，原版远端构建、69 项测试、RTX 渲染和 ROS 图像发布已完成验证。已根据实际错误修复 ROS 导入和 CENTER 查询，再恢复必要的自瞄通信与测试场景；详细结果见 [仿真回滚与基线测试](simulator-baseline.md)。

完整测试需要兼容现有代码的 ROS2 Jazzy/Humble、CMake、Rust、OpenCV/Eigen/yaml-cpp/fmt/spdlog/OpenVINO/nlohmann-json、Python NumPy/rclpy/OpenCV，以及 Vulkan/ALSA/udev 开发依赖。原构建脚本使用的系统库路径和 OpenVINO 2024.6 路径见 `sp_vision_ros/scripts/build.sh`，目标部署需与其匹配。Ubuntu 24.04 不应直接套用 Ubuntu 22.04 的 Humble 系统包；目标机已使用原生 Jazzy 并完成 C++ 适配器验证。

仿真需要远端图形会话和可用的 Vulkan 驱动。无 `DISPLAY` 时调度自动尝试 `xvfb-run -a`；没有虚拟显示时会明确失败。可视化入口自动处理桌面显示号和认证；当前 `.102` 已登录桌面是 `:1`。不要假定其他机器也相同。虚拟显示不能修复 GPU 驱动。本地浏览器查看另外使用已安装的 x11vnc、noVNC/websockify，仅监听 loopback 并通过 SSH 隧道访问。

## 早期调度接入记录（2026-10-15）

以下记录保留初次接入时的状态。随后已补齐 ROS2 Jazzy/OpenVINO，完成构建、长时资格和可视化测试；当前结论分别见 [ROS 测试说明](../sp_vision_ros/docs/测试说明.md)和[可视化验证记录](可视化远程仿真测试.md)。

- SSH 免密连接、源码增量同步、任务执行、日志与产物回传：已验证。
- 目标电脑的 Python 几何测试：3 项通过；230 个原 SP 文件完整性检查：通过。作业 `20261015-013853-a33b9cc1`，报告位于本机 `artifacts/remote/20261015-013853-a33b9cc1/`。
- 完整测试已实际调度：配置生成成功，随后因 `ROS2 unavailable` 在构建阶段停止，未运行后续 ROS/Rust/闭环测试。作业 `20261015-013950-e5f7da2e`，失败日志和退出码已回传。
- 调度程序自身的 6 项轻量回归测试：全部通过。
