# SP Planner 计算延迟优化（2026-10-20）

## 原因与改动

延迟是前期未对齐的已确认因素之一，但不等于全部问题。此前已修复执行端的旧姿态检查与指令时间推进；第一阶段优化sp_vision_25内部的计算开销，A/B中的仿真执行器、门限、场景、弹道、图像分辨率和两份YAML保持一致。随后单独调整默认仿真配置的跟踪间隔，数据以default-gap200分组区分。

`Planner::plan(optional<Target>)`原本已经根据观测时间预测到当前时刻，再加配置中的机械/通信提前量；因此不能把图像帧龄100ms直接再加为固定提前量，否则会重复补偿。本轮未额外增加提前量。

找到的开销：MPC生成100点参考轨迹时，原先对复制的Target反复调用完整EKF预测，计算状态转移矩阵、过程噪声及协方差传播；但规划只读取位置、速度与装甲几何，不使用这些临时协方差。

修改为只读`Target::armor_xyza_list(dt)`，按现有匀速模型投影中心位置和旋转相位，再由同一装甲几何计算器生成位置。Planner通过时间偏移采样未来轨迹；几何开火窗口也用同样的只读投影。Tracker继续使用完整的predict/update，不删除滤波和观测不确定度。Planner的观测时刻预测和弹丸飞行时间预测仍保留原实现，包括前哨站原有的转速处理。

改动：[target.hpp](../tasks/auto_aim/target.hpp)、[target.cpp](../tasks/auto_aim/target.cpp)、[planner.hpp](../tasks/auto_aim/planner/planner.hpp)、[planner.cpp](../tasks/auto_aim/planner/planner.cpp)。ROS诊断额外记录plan_ms和processing_ms，两组都使用相同计时代码。

## 等价性与性能验证

新增[planner_rollout_test.cpp](../tests/tuning/planner_rollout_test.cpp)：覆盖2/3/4块装甲、正反转、零转速、跨±π以及XYZ方向平移。比较同一滤波状态的完整EKF逐步预测和只读投影，并确认原状态与协方差未被修改。3,672个采样中，最大几何误差2.85×10⁻¹⁴，最大瞄准角差4.06×10⁻¹⁵rad。此结论针对采样参考；不声称异步闭环的每一帧或MPC数值解逐位一致。

独立参考轨迹基准：300轮×102采样，交替顺序重复，两次取均值；完整EKF参考78.915ms，只读参考4.320ms，约18.27倍。**这是参考采样部分，不是整套自瞄18倍。**

实际ROS闭环中只统计control=true的plan_ms，单位ms。它包含从Planner调用开始到发布诊断前的处理，因此是在线近似计时，不是仅TinyMPC求解耗时。processing_ms包括图像转换、检测、求解和跟踪，不能称为单独神经网络推理时间。

| 版本 | 配置 | 场景 | 时长(s) | Planner中位/p95(ms) | 视觉处理中位(ms) | 视觉完成FPS | 发射/敌方命中 | 命中率 | 200血换算最短(s) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| before (before45) | standard3_high_rate | move | 45 | 1.487/2.296 | 30.65 | 11.61 | 115/109 | 94.78% | 2.054 |
| before (before45) | standard3_high_rate | spin | 45 | 1.678/2.493 | 28.98 | 11.47 | 83/83 | 100.00% | 1.863 |
| before (before45) | standard3_high_rate | static | 45 | 1.422/2.251 | 29.92 | 11.81 | 166/165 | 99.40% | 1.386 |
| after (after45) | standard3_high_rate | move | 45 | 0.856/1.253 | 29.29 | 11.83 | 120/105 | 87.50% | 1.353 |
| after (after45) | standard3_high_rate | spin | 45 | 1.018/1.402 | 28.60 | 10.77 | 86/86 | 100.00% | 2.385 |
| after (after45) | standard3_high_rate | static | 45 | 0.817/1.233 | 29.25 | 11.49 | 137/131 | 95.62% | 1.861 |
| before (static90) | standard3_high_rate | static | 90 | 1.469/2.301 | 29.65 | 11.34 | 294/282 | 95.92% | 1.738 |
| after (static90) | standard3_high_rate | static | 90 | 0.760/1.213 | 29.17 | 11.41 | 326/321 | 98.47% | 1.413 |
| before (move90) | standard3_high_rate | move | 90 | 1.483/2.317 | 29.53 | 11.14 | 263/254 | 96.58% | 1.583 |
| after (move90) | standard3_high_rate | move | 90 | 0.867/1.262 | 29.25 | 11.53 | 255/246 | 96.47% | 1.476 |
| after (default) | standard3 | move | 45 | 0.831/1.238 | 28.17 | 11.46 | 70/65 | 92.86% | 2.338 |
| after (default) | standard3 | spin | 45 | 0.967/1.363 | 28.61 | 11.51 | 62/59 | 95.16% | 4.594 |
| after (default) | standard3 | static | 45 | 0.627/1.182 | 29.66 | 11.61 | 160/158 | 98.75% | 1.723 |
| after (default-gap200) | standard3 | move | 45 | 0.854/1.281 | 30.10 | 12.15 | 127/118 | 92.91% | 1.692 |
| after (default-gap200) | standard3 | spin | 45 | 1.066/1.402 | 28.95 | 11.47 | 85/85 | 100.00% | 2.252 |
| after (default-gap200) | standard3 | static | 45 | 0.758/1.211 | 28.96 | 11.51 | 181/180 | 99.45% | 1.488 |

短测不能只挑有利结果：45秒旋转83→86次命中，静态165→131，横移109→105；静态和横移短测下降，横移候选命中率只有87.5%。因此另外保存了优化前、后的完整已安装节点，做90秒A/B；只替换节点二进制，仿真与配置不变，且用EXIT trap恢复优化后版本。二进制摘要在作业日志和JSON中明确记录，不能把当时磁盘上的新源码摘要误称为旧二进制的来源摘要。

90秒静态A/B：before 294发/282命中（95.92%），after 326发/321命中（98.47%）。这组没有重现短测的下降，但只有一次固定顺序的A/B，不证明统计显著性或所有机动条件都更好。视觉完成FPS仍约11–12，未证实图像链路整体提速。

所有发射都进入命中率分母，敌方装甲命中按shot_id去重；尚无命中记录的在飞弹丸也计入分母。200血耗时沿用20伤害、10次命中的滚动窗口定义，含飞行时间，排除窗口前已发射的在飞弹丸。没有热量、回血、法向速度判伤和完整裁判模型，不能当作实战击杀时间。

## 默认仿真跟踪间隔修正

默认standard3使用SP缺省的100ms最大观测间隔；实际帧间隔反复达到110–160ms，日志出现大量Large dt重置。在上述只读采样已启用的程序上，先完成三场原配置回归，再仅在`sp_vision_ros/configs/standard3.yaml`加入`tracker_max_dt: 0.2`，独立复测。高射频配置原本就是0.2，不受这一步影响；SP库缺省和实机YAML没有更改，目标/执行端250ms超时参数没有增加。既有tracker_intervals回归验证120ms短间断保持、250ms断流重置、逆序时间及非法参数拒绝。

这一阶段两组都已使用快速Planner，差别是仿真跟踪间隔，不能将收益全部归于计算加速。以下均45秒：

| 场景 | 修正前发射/命中 | 修正后发射/命中 | Large dt重置次数 | 修正后保守命中率 |
|---|---:|---:|---:|---:|
| static | 160/158 | 181/180 | 60→11 | 99.45% |
| spin | 62/59 | 85/85 | 72→18 | 100.00% |
| move | 70/65 | 127/118 | 45→11 | 92.91% |

三场有效命中吞吐均增加。横移保守命中率仍只有92.91%（包含尚未确认的在飞弹丸），已完成弹丸的命中率为95.50%；正式评估仍使用前者，不能因后者更好就宣称达到95%保守命中率资格。该场未对齐/精度问题尚未完全解决。测试使用真实渲染和物理命中，没有替换在线图像或输入目标真值。

90秒横移二进制A/B另外得到before 263发/254命中（96.58%）、after 255发/246命中（96.47%）；有效命中下降约3.1%，并非所有场景的射频都提高。计算耗时降低得到多场一致支持，稳定提升全部场景的命中吞吐尚未证明。

## 构建、回归和审计

- 所有C++构建、基准、16场仿真均在wuxian-remote对应的RTX 4060主机执行。本机只编辑、统计JSON、轻量摘要审计和5项配置工作流回归（均通过）。
- ROS几何/Planner CTest通过；新增rollout、原tracker_intervals、fire_window三项tuning CTest通过。新测试初次编译的夹具误用了不存在的Armor默认构造，修正为实际构造接口后重新构建与三项回归均通过。
- 原始`tuning-source-manifest.json`不改写；新增[planner-performance-manifest.json](planner-performance-manifest.json)作为本轮授权版本，包含原清单SHA和修改/新增文件SHA。现有check_tuned_sources读取叠加清单，237文件审计通过。
- 自动采集使用宽松入口门槛（min-shots=1、hit-rate/Wilson=0.01），日志PASS只代表闭环/采集完成，不能替代命中率或长时资格验收；低命中短测在表内完整保留。
- 旧版与候选的ROS节点源码、仿真plugin/gimbal源码、TOML和高射频YAML的测试摘要一致，未用更换物理/门限制造算法收益。

证据：

- before45：[20261020-140910-971405a8](../../../artifacts/remote/20261020-140910-971405a8/)，每场45秒。

- after45：[20261020-141255-80a6743d](../../../artifacts/remote/20261020-141255-80a6743d/)，每场45秒。

- static90：[20261020-141537-30594954](../../../artifacts/remote/20261020-141537-30594954/)，每场90秒。

- move90：[20261020-141803-02b91445](../../../artifacts/remote/20261020-141803-02b91445/)，每场90秒。

- default：[20261020-142118-01e45b57](../../../artifacts/remote/20261020-142118-01e45b57/)，每场45秒。

- default-gap200：[20261020-142701-016b1101](../../../artifacts/remote/20261020-142701-016b1101/)，每场45秒。

- [最终三项tuning回归、等价性与独立基准](../../../artifacts/remote/20261020-141238-378712cf/)
- [完整对照、窗口、源码/二进制摘要](../../../artifacts/sp-planner-latency-2026-10-20.json)

复测：先通过根目录remote_run.py远端build；再source ROS环境，配置/构建sp_vision_25/tests/tuning，运行CTest。实际闭环仍使用原closed-loop入口和configs，不需要调大提前量或放宽开火门限。

```bash
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true build
python3 tools/remote_run.py run -- bash -c 'source sp_vision_ros/scripts/env.sh && cmake -S sp_vision_25/tests/tuning -B sp_vision_ros/build-planner-tuning -DCMAKE_BUILD_TYPE=Release -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake && cmake --build sp_vision_ros/build-planner-tuning -j8 && ctest --test-dir sp_vision_ros/build-planner-tuning --output-on-failure'
python3 tools/remote_run.py --env SP_AIM_CONFIG=configs/standard3_high_rate.yaml --skip-build --scenario spin --seconds 60 closed-loop
python3 sp_vision_25/tools/check_tuned_sources.py
```
