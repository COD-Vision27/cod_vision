# 参数调优记录（2026-10-15）

本次只修改 SP 算法及独立调参配置，Bevy 仿真器、ROS 适配器、目标运动、渲染、弹道、云台执行和开火门限保持原样。所有 C++ 编译、回归和图像闭环都在 `wuxian@192.168.1.102` 执行。

## 指标与选择方式

主指标是每秒实际命中的敌方弹丸数，同时检查实际射速、保守命中率及 Wilson 95% 下限。命中按 shot_id 去重并检查敌方队伍和装甲编号；全部实际发射进入分母，包括尚无命中记录的在飞弹丸。每秒指标统一除以完整场景时长，包含启动等待，不把 Planner 开火建议当成已发射。

45/60 秒试验用于筛选，最终配置另做长时复测。最终选择最大化三个场景中最低的有效命中次数/秒，并要求每场至少 100 发、保守命中率 ≥95%、Wilson 95% 下限 ≥90%；同时完整列出静态与动态的吞吐取舍。测试场景为现有 static/spin/move，运动、距离和遮挡覆盖有限；本次结果不能当作实机成绩或全局最优证明。

## 通用调整依据

- `tracker_max_dt`：连续图像的最大可接受时间间隔（秒），缺省 0.1。超过该间隔会重新初始化跟踪。可根据真实相机有效帧率和掉帧情况配置，限于 (0, 0.5]；时间戳倒退也会重置。这个参数不增加 ROS 数据过期和指令看门狗时限。
- `initial_radius_variance`、`initial_dr_variance`、`initial_dz_variance`：普通机器人滤波初始的半径、长短半径差和装甲高度差方差（平方米）。缺省仍为原值 1。候选值 0.01/0.01/0.0025 对应 10/10/5 cm 标准差，保留非零尺寸不确定度；不向算法提供目标真实位置、速度或尺寸。前哨站/基地已有尺寸先验不变。
- `fire_thresh` 是 MPC 参考角与规划角的二维误差阈值（rad），不是发射器机械射速。放宽它必须重新比较真实命中，不能根据开火建议占比单独采纳。
- `acceleration_variance` / `angular_acceleration_variance`：普通目标的运动模型过程噪声，缺省 100 / 400；降低它们会平滑匀速轨迹，也可能导致急变运动滞后。本次 25/100 与 25/25 候选被独立机动测试否决，不进入最终配置。
- `fire_yaw_scale` / `fire_armor_fraction`：几何开火窗口。缺省 1 / 0 完全保留旧圆形误差判据；启用时按估计距离、装甲宽高和面向角计算中央区域的内接椭圆，垂直阈值不超过原 `fire_thresh`，水平阈值也受投影宽度限制。使用原 Solver 的 135/230 mm 宽、56 mm 灯条高度模型；远距或斜向目标自动收紧，背面不允许开火。这里仍是规划误差检查，执行器必须保留实际轴误差检查。
- 保留原检测置信度、传统角点修正、最大加速度、外参、弹速和安装偏差；这些参数需要由对应真实硬件的测量决定。

## 可复现入口

先通过根目录 `tools/remote_run.py` 同步和远端构建，然后在 ROS 环境运行 `sp_vision_25/tools/tune_closed_loop.py`。脚本只在调度器的远端工作区启动，用现有校准过的仿真 YAML 构造候选配置，修改的只有显式列出的 SP 算法参数。所有候选、原始日志和逐次结果写入 `sp_vision_ros/artifacts`，由调度器回传。

```bash
python3 tools/remote_run.py run -- bash -c 'source sp_vision_ros/scripts/env.sh && xvfb-run -a python3 sp_vision_25/tools/tune_closed_loop.py --profiles baseline prior prior_gap150 prior_gap200 --seconds 60'
```

跟踪回归单独位于 `sp_vision_25/tests/tuning`，复用远端构建出的 SP core，覆盖两种跟踪接口、旧参数兼容、短间断恢复、长断流和逆序时间，以及非法配置和初始协方差。原有 PnP/MPC CTest 也要通过。

原 `check_sp_unchanged.py` 比较的是本次获准修改之前的冻结快照；本次不修改它或替换旧快照，改用逐文件摘要记录预期的 Tracker 变更，避免把授权修改误称为“原 SP 完全未改动”。

## 已否决的参数

尺寸先验 0.01/0.01/0.0025 虽可消除部分仿真中的半径发散，却偏向默认初始半径。独立投影测试覆盖 2/5/8 m、半径 0.12/0.25/0.40 m 和 30/120 ms 采样，使用同一个 Solver 和公共 Tracker 接口；普通运动的中心 RMSE 中位数由 0.031 m 增至 0.067 m，故未采用。

过程噪声 25/100 和 25/25 在匀速试验中降低了中心误差，但急变运动（旋转角加速度峰值约 18 rad/s²、横向加速度峰值约 2.7 m/s²）的跟踪帧数和未来中心预测更差。例如 25/100 的未来中心 RMSE 中位数由 0.213 m 增至 0.255 m，故停止剩余仿真筛选。该探测使用 0.2 px 合成观测噪声，是泛化筛查，不是实机精度证明。

这些实验旋钮保留缺省兼容能力，以便后续用真实观测统计辨识；最终配置只采用通过筛选的参数，未采用的候选不会自动生效。独立探测源码见 `tests/tuning/tracker_geometry_probe.cpp`，历史结果包含候选配置和源码摘要，不能混用不同版本的成绩。

## 共用代码修复与对照边界

检测装甲的 `priority` 原先没有初始化，但 Tracker 会用它排序；现统一缺省为 `fifth`，已有调用者显式设置的优先级仍生效。两块装甲的平衡车分支原先固定读取前三个候选，现最多读取实际存在的两个，避免越界。独立构造的数学 Target 也明确初始化型号和内部状态，使离线测试不依赖未初始化枚举。

最终 A/B 在同一构建版本上比较旧参数与候选参数，两者都包含上述修复；不能把修复的收益单独记成参数收益。原 60 秒基线另外保留作前期问题定位证据。

推理请求复用通过“目标图像→空白→镜像→空白→目标”的完全一致性测试，但完整检测平均耗时仅由 4.36 ms 变为 4.31 ms，闭环也没有稳定收益，最终配置不启用该项。输入图像是回归夹具，不用于替换在线真实渲染帧，不更换或训练模型。

## 交付文件与复测

- `configs/standard3_high_rate.yaml`：复制当前 standard3 的完整硬件配置，只新增三个算法键。相机、外参、安装偏差、弹速、串口和原有重复键顺序均保留，适用于原 standard3 对应硬件；其他车辆不能复用其标定。
- `configs/simulator_high_rate.yaml`：仅用于复现评测，保留现有 ROS 测试夹具的相机/枪口几何，与硬件配置使用完全相同的三项算法调整。
- `tools/qualify_tuning.py`：校验六场新鲜结果、精确测试 YAML 摘要、硬件参数保持、准确率门槛与最弱场景吞吐。

最终复测命令（先按远程文档构建同一版本）：

```bash
python3 tools/remote_run.py run -- bash -c 'source sp_vision_ros/scripts/env.sh && xvfb-run -a python3 sp_vision_25/tools/tune_closed_loop.py --profiles baseline candidate --candidate-config sp_vision_25/configs/simulator_high_rate.yaml --seconds 120 --report-output sp_vision_ros/artifacts/final_tuning_comparison.json && python3 sp_vision_25/tools/qualify_tuning.py'
```

同一算法 YAML 也可单场使用：

```bash
python3 tools/remote_run.py --env SP_AIM_CONFIG=../sp_vision_25/configs/simulator_high_rate.yaml --skip-build --scenario spin --seconds 120 closed-loop
```

实机使用对应的 standard_mpc 配置路径，仍须由电控按实际姿态、弹速、通信时延和热量状态执行开火约束。本次没有实际硬件采集或实弹验证。

本次保留原冻结基线，原 `remote_run.py check/test` 中“SP 必须完全不变”的旧守卫会报告这 9 项授权修改。调优版新增独立完整性入口，核对 234 个原文件及交付算法/配置文件的摘要，不覆盖旧基线：

```bash
python3 tools/remote_run.py run -- python3 sp_vision_25/tools/check_tuned_sources.py
```

清单为 `docs/tuning-source-manifest.json`；轻量本地审计也已通过，9 项授权变化之外没有额外原文件变化。

## 最终验收（PASS）

远端作业 `20261015-115001-c8844f90`，同一版本共 6 场，每场 120 秒。两组都包含本次通用代码修复，候选只新增三项算法参数。命中计数包含所有实际发射，按目标敌方装甲的 shot_id 去重。

| 场景 | 原射速（发/s） | 新射速（发/s） | 新发射 / 命中 | 保守命中率 | Wilson 95% 下限 | 有效命中/s 变化 |
|---|---:|---:|---:|---:|---:|---:|
| 静态 | 3.50 | 3.04 | 365 / 364 | 99.73% | 98.46% | -13.3% |
| 旋转 | 0.95 | 1.40 | 168 / 166 | 98.81% | 95.76% | +46.9% |
| 横移 | 1.52 | 1.90 | 228 / 224 | 98.25% | 95.58% | +28.0% |

三个等时长场景合计，确认命中由 708 发增至 754 发（+6.5%）；最弱场景由 0.942 增至 1.383 次确认命中/s（+46.9%）。**这是偏动态目标的配置：静态吞吐下降约 13%，并非每个场景都更快，也未证明全局最优。**

新三场均 ≥100 发、命中率 ≥95%、Wilson 下限 ≥90%，结束停止控制，精确 YAML 摘要和硬件原参数保持检查均通过。仿真源码、配置、资产和 ROS 适配器的哈希从本轮首次实验到最终六场均未变。所有场景仍是现有 4 米及既定运动，真实硬件命中尚未验证。

[验收摘要](high-rate-qualification.json)包含配置和源码摘要；[六场完整原始对照](../../artifacts/remote/20261015-115001-c8844f90/project-artifacts/final_tuning_comparison.json)保留逐场诊断和证据前缀。相同目录中的 `final-tuning-build.log`、ROS smoke、几何结果及全部图像/物理遥测可追溯。

MPC 数值求解仍可能略超名义加速度（本轮原/新最大 yaw 指令约 51.18 / 51.97 rad/s²，配置为 50）；现有执行端继续按 max_acc 限幅，未放宽物理约束。真实电控也须按自身能力限制执行；此次没有把仿真端的限幅当作已验证的实机能力。
