> 2026-10-20：默认仿真已迁移至 Godot + 同机共享内存。当前命令与配置方法见 [新接入说明](../../sp_vision_sim/README.md)。下文 Bevy / ROS / TOML 内容保留为历史记录；新的相机标定由同步帧元数据提供。

# 仿真配置与调参入口

这是 ROS 自瞄仿真实际读取的配置目录，形式与 `sp_vision_25/configs` 一致。修改这里的参数后重新启动即可，不需要重新编译。修改 C++ / Rust 算法或 ROS 消息后才需远程构建。

## 配置敌方转速、横移与距离

编辑 [simulator.toml](simulator.toml) 中已有的 `[scenario]` 段，例如将该段的相应字段改为：

```toml
[scenario]
mode = "spin"                  # static 静止 / spin 小陀螺 / move 横移
spin_speed_rad_s = 3.0          # 敌方底盘转速 rad/s；负数反转，0 停转
target_distance_m = 4.0         # 初始前方距离 m
target_height_m = 1.0           # 生成高度 m，随后落地
target_lateral_m = 0.0          # 初始横向位置/横移中心 m
initial_pitch_deg = -9.0        # 己方初始枪口俯仰，向下为负
move_amplitude_m = 0.8         # 横移半幅 m
move_peak_speed_mps = 0.64     # 正弦参考速度峰值 m/s
move_start_delay_s = 4.0       # 横移开始前等待 s
```

不要再追加第二个 `[scenario]` 段。未写出的字段使用默认值。省略 `spin_speed_rad_s` 时，敌方小陀螺继承 `[vehicle].rotation_speed`；填写后仅覆盖测试敌方的转速。1 圈/s = 2π rad/s ≈ 6.283 rad/s。转速变化仍受 `[vehicle].yaw_acceleration` 限制，不会瞬间达到目标转速。

```bash
# 按 TOML 的 mode 和运动参数运行（不再默认用命令行 static 覆盖）
python3 tools/sim_view.py --profile standard3_high_rate --seconds 180

# 临时指定横移模式，仍读取 TOML 内的速度、幅度等参数
python3 tools/sim_view.py --scenario move --seconds 180
```

优先级为命令行 `--scenario`（或直接脚本的 `DAEDALUS_SP_SCENARIO`）高于 TOML 的 `mode`。自动闭环测试也会显式指定场景，所以不会被 `mode` 改成其他测试。

横移周期为 `2π × move_amplitude_m / move_peak_speed_mps`；幅度或参考速度为零表示不横移。实际速度由原物理驱动和反馈控制决定，受 `[vehicle].max_speed`、加速度、阻尼等影响，参考峰值不是对实际速度的硬保证。高级反馈参数 `move_position_gain`、`move_velocity_gain`、`move_deadband` 同样可在 TOML 调整。转速日志打印 `target_rate` 与 `actual_rate`，横移日志打印实际位置/速度与参考位置，便于确认参数生效。

场景模式、生成位置和初始俯仰需重新启动才完整生效。当前运行入口会把所选 TOML 复制到目标机运行目录，因此修改本地源配置后请重启调度；不承诺本地编辑会实时推送到正在运行的窗口。只调整这些运动参数不需要改 YAML 相机外参。原仿真 `bevy_robomaster_simulator/config.toml` 没有 `[scenario]` 段时仍为原交互场景。

2026-10-18：上述场景参数已接入代码，增加默认兼容、反向转速、零幅度/速度与非法参数的 Rust 测试。Python 配置/调度测试、3 项几何测试和 Rust 格式/语法解析检查通过。经 `wuxian-remote` 中转，作业 `20261018-033748-dd51549c` 已在原 RTX 4060 目标机重新构建通过，包括 CTest；随后继续运行 Rust 测试与五种参数场景时，服务器反向隧道 `127.0.0.1:2222` 已停止监听，测试在连接阶段中断。因此本次运行验证仍待隧道恢复，下方 2026-10-15 记录不代表这次改动已验证。

恢复连接后，可运行以下检查；后续只改 TOML 不需重新编译：

```bash
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true build
python3 tools/remote_run.py run -- bash -c 'source sp_vision_ros/scripts/env.sh && cd bevy_robomaster_simulator && cargo test --no-default-features --features ros2 --bin daedalus -j8'
python3 tools/remote_run.py run -- bash -c 'source sp_vision_ros/scripts/env.sh && xvfb-run -a python3 tools/verify_sim_motion.py'
```

`verify_sim_motion.py` 在目标机分别验证正转 3 rad/s、反转 −2 rad/s、停转、继承车辆转速 1.5 rad/s 和自定义横移；每项 12 秒，检查 RTX 4060、实际转速/运动日志、退出码，回传 `motion-config-validation.json` 及每项 TOML / 日志。它不启动实机硬件或自瞄算法，专门验证仿真运动参数。

## 文件对应关系

| 文件 | 用途 | 选择方法 |
|---|---|---|
| [standard3.yaml](standard3.yaml) | standard3 派生的仿真配置；2026-10-20 将短间断容忍设为 0.2 秒，历史参数快照见对应测试产物 | 默认，或 `--profile standard3` |
| [standard4.yaml](standard4.yaml) | standard4 派生的仿真基准 | `--profile standard4` |
| [standard3_high_rate.yaml](standard3_high_rate.yaml) | 调优 agent 的仿真候选；从 `sp_vision_25/configs/simulator_high_rate.yaml` 原样复制 | `--profile standard3_high_rate` |
| [simulator.toml](simulator.toml) | Bevy 场景、渲染、物理、云台执行器 PID、弹丸和相机参数 | 默认；也可 `--sim-config` 指定另一个 TOML |
| [geometry.json](geometry.json) | 模型导出的相机/枪口几何来源与计算结果 | 计算/检查辅助，算法实际读取 YAML 中的矩阵 |

`--profile` 与原 `--base` 等价；`--aim-config` 显式指定时优先于 profile。两个 YAML 基准未因整理而重新生成，候选参数也没有重新调优。调优候选依赖调优 agent 修改后的 Tracker/Planner 源码；旧二进制不能据此验证新算法。

2026-10-15 收录时，候选相对 standard3 的算法调整为：

```yaml
tracker_max_dt: 0.2
fire_yaw_scale: 3.0
fire_armor_fraction: 0.5
```

2026-10-20：默认 standard3 仿真配置也采用 `tracker_max_dt: 0.2`，用于容忍实际 100–200 ms 的图像间断；SP 的缺省值和实机配置仍保留原值，执行端超时不变。Planner 同期增加只读轨迹采样以减少计算开销，数学等价性、在线耗时和闭环结果见 [延迟优化记录](../../sp_vision_25/docs/planner-latency-2026-10-20.md)。历史验收与本次结果按源码/配置摘要分别使用。

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
