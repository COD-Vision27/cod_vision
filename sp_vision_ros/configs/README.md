# 仿真配置与调参入口

这是 ROS 自瞄仿真实际读取的配置目录，形式与 `sp_vision_25/configs` 一致。修改这里的参数后重新启动即可，不需要重新编译。修改 C++ / Rust 算法或 ROS 消息后才需远程构建。

## 文件对应关系

| 文件 | 用途 | 选择方法 |
|---|---|---|
| [standard3.yaml](standard3.yaml) | standard3 派生的仿真基准；保留之前验收配置原文 | 默认，或 `--profile standard3` |
| [standard4.yaml](standard4.yaml) | standard4 派生的仿真基准 | `--profile standard4` |
| [standard3_high_rate.yaml](standard3_high_rate.yaml) | 调优 agent 的仿真候选；从 `sp_vision_25/configs/simulator_high_rate.yaml` 原样复制 | `--profile standard3_high_rate` |
| [simulator.toml](simulator.toml) | Bevy 场景、渲染、物理、云台执行器 PID、弹丸和相机参数 | 默认；也可 `--sim-config` 指定另一个 TOML |
| [geometry.json](geometry.json) | 模型导出的相机/枪口几何来源与计算结果 | 计算/检查辅助，算法实际读取 YAML 中的矩阵 |

`--profile` 与原 `--base` 等价；`--aim-config` 显式指定时优先于 profile。两个 YAML 基准未因整理而重新生成，候选参数也没有重新调优。调优候选依赖调优 agent 修改后的 Tracker/Planner 源码；旧二进制不能据此验证新算法。

当前候选相对 standard3 的算法调整为：

```yaml
tracker_max_dt: 0.2
fire_yaw_scale: 3.0
fire_armor_fraction: 0.5
```

其他参数以实际文件为准。该候选是本次收录的独立快照；今后继续在本目录调整，原 SP 下的同名来源不会自动覆盖它。候选说明见[调优 agent 的参数报告](../../sp_vision_25/docs/parameter-tuning.md)。

## 直接复测

从工作区根目录执行：

```bash
# 若调优 agent 改了算法源码，先在目标机编译；不会覆盖配置
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true build

# 用候选参数跑旋转目标，目标机计算、本地浏览器看
python3 tools/sim_view.py --profile standard3_high_rate --scenario spin --seconds 180

# 换为目标机桌面显示
python3 tools/sim_view.py --view remote --profile standard3_high_rate --scenario move --seconds 180

# 回到 standard3 基准作对照
python3 tools/sim_view.py --profile standard3 --scenario spin --seconds 180
```

每次只改少量参数；相同场景、时长、TOML 下对照。画面 HUD 是直观观察，正式效果以逐发命中遥测为准。

## 自建调参方案

```bash
cp sp_vision_ros/configs/standard3_high_rate.yaml sp_vision_ros/configs/my_tuning.yaml
# 用编辑器修改 my_tuning.yaml
python3 tools/sim_view.py --aim-config sp_vision_ros/configs/my_tuning.yaml --scenario spin --seconds 180

# 也可复制一份 Bevy 配置，调整云台 PID 等仿真侧参数
cp sp_vision_ros/configs/simulator.toml sp_vision_ros/configs/my_simulator.toml
python3 tools/sim_view.py --aim-config sp_vision_ros/configs/my_tuning.yaml --sim-config sp_vision_ros/configs/my_simulator.toml --scenario move
```

这两个参数使用**相对于工作区根目录**的路径，也接受工作区内的绝对路径；文件须在会同步的源码目录，不能放在 `artifacts`、`build`、`target`、`install` 等被排除的目录。SSH 调度自动同步修改后的配置。

`run_aim.sh` / `run_sim.sh` 仍分别支持 `SP_AIM_CONFIG` / `SP_SIM_CONFIG`。直接用环境变量时建议传绝对路径；自瞄脚本的相对路径基准是 `sp_vision_ros`。可视化入口优先使用上述命令行参数。

## 参数分工

| 调哪里 | 对应参数 |
|---|---|
| YAML：检测 | `min_confidence`、`use_traditional`、`threshold`、灯条/装甲筛选参数 |
| YAML：跟踪 | `min_detect_count`、`max_temp_lost_count`、候选中的 `tracker_max_dt` |
| YAML：规划与射击 | `fire_thresh`、`Q_yaw` / `R_yaw`、`Q_pitch` / `R_pitch`、`max_yaw_acc` / `max_pitch_acc`、候选中的开火窗口参数 |
| YAML：ROS 时序 | `plan_period_ms`、`max_frame_age_ms`、`target_timeout_ms`、`command_timeout_s`、`allow_fire` |
| TOML：云台执行器 | `[vehicle.gimbal_pid.yaw]` / `[vehicle.gimbal_pid.pitch]` 的 `kp`、`ki`、`kd`、`max_rate` 等 |
| TOML：仿真与渲染 | `[physics]`、`[render]`、`[preview]`、`[projectile]`、`[camera]`、`[capture.color]` |

原生新版 Planner 的开火判断、ROS 的 `allow_fire` 和仿真执行器的实际枪口对齐检查共同决定是否发弹。旧链路的 `first_tolerance` 等字段虽从基准保留，不能据其数值推断当前 MPC 开火窗口。参数读取与解释以当前源码为准。

## 相机、外参与弹速

仅调 PID、检测、跟踪或开火窗口时，保留 YAML 中的相机内外参。修改 TOML 的相机 FOV、RGB 分辨率、弹速，或更换机器人模型后，需要刷新对应 YAML 的几何字段：

```bash
# 只刷新几何和弹速，保留其余算法参数；不会改写 TOML
python3 sp_vision_ros/scripts/derive_config.py --geometry-only --output sp_vision_ros/configs/my_tuning.yaml --sim-config sp_vision_ros/configs/my_simulator.toml
```

这个轻量计算可在本机执行。修改公共 TOML 后，用同一方式刷新你将实际测试的每个 YAML。仿真中没有镜头畸变，`distort_coeffs` 为零。`geometry.json` 是最后一次导出的元数据，实际生效值看选中的 YAML；自瞄节点会检查图像尺寸与内参是否匹配。

`derive_config.py` 默认从原 SP 基准生成一个完整 YAML；已有目标文件会拒绝覆盖。`--force` 会明确重置该 YAML 的算法参数，日常调参请使用复制配置或 `--geometry-only`。构建、自动测试、可视化启动均不隐式执行生成器。

## 确认到底加载了哪份参数

可视化启动终端会打印两份配置的路径与 SHA-256，正常结束后回传到 `artifacts/remote/<作业编号>/project-artifacts/`：

- `active-configs.json`：实际配置路径与哈希。
- `active-aim.yaml`：本次自瞄参数快照。
- `active-simulator.toml`：本次 Bevy 参数快照。启动脚本将它复制为运行目录的 `config.toml`，因此请编辑本目录源码配置并重新启动。

自动闭环结果的 `configs` 字段也记录配置路径和哈希。原 300 秒资格结果属于当时的源码/参数，不能沿用为手调后的通过证明。

本次整理验证（2026-10-15）：目标机重新构建与 CTest 通过，ROS smoke 通过；候选 `spin` 30 秒实际发射 36 发、记录敌方命中 36 发，停止后解除控制正常。作业 `20261015-152427-377ec453` 的结果包含实际配置哈希；可视化启动作业为 `20261015-152301-7fca7fe3`。这仅验证迁移后的短时加载和链路。配置工作流/调度 10 项轻量回归及 3 项几何测试通过，当前调优源码 234 文件审计通过。原基准两份 YAML 与历史验收的字节一致，候选与调优 agent 来源文件的字节一致，本次整理前的 247 个 SP 文件未变。

## 兼容与复测命令

旧 `sp_vision_ros/config` 是指向本目录的兼容链接；`simulator.yaml` / `simulator4.yaml` 分别链接到 `standard3.yaml` / `standard4.yaml`。调优 agent 现有脚本仍能读取旧路径；它们与新文件是同一份内容，日常统一编辑上表中的文件。

```bash
# 短时自动闭环：候选配置，旋转目标
python3 tools/remote_run.py --env SP_AIM_CONFIG=configs/standard3_high_rate.yaml --skip-build --scenario spin --seconds 60 closed-loop

# 长时逐发统计：候选配置，三个场景
python3 tools/remote_run.py run -- bash -c 'source sp_vision_ros/scripts/env.sh && xvfb-run -a python3 tools/sim_accuracy.py --bases standard3_high_rate --seconds 300'
```

返回[文档目录](../../docs/文档目录.md)或[可视化操作说明](../../docs/可视化远程仿真测试.md)。
