# 独立图像管线性能修复与复测（2026-10-20）

多 agent 分别审查渲染回读、ROS 图像链路及独立观测工具。所有仿真均在原 RTX 4060 主机执行；未启动自瞄或硬件接口、未发布控制指令，显式使用独立 ROS 域 90 和 DAEDALUS_AUTO_AIM=0。若意外启用自瞄订阅则测试终止；这不是仿真引擎层面的全局输入互锁。没有测试射速或 TTK。

## 改动

1. 仿真 Cargo 开发构建的本体优化等级由 1 改为 3，与依赖库已有等级一致。没有更改场景、分辨率、图像质量、运动、物理或算法参数。
2. 新增 tools/capture_performance.py：只订阅图像，使用实际 canonical TOML，记录配置/源码哈希、接收 FPS、图像时间戳间隔和帧龄；首帧后排除 5 秒，保留原始时间戳。渲染统计排除的是前 10 个日志样本，与图像观察窗口不同。
3. 独立观测入口拒绝非调度器工作区运行；默认强制 ROS 域 90，覆盖 env.sh 设置的普通域。日志读取不完整、清理失败或有效样本不足会失败。6 项本地回归覆盖时钟、统计、域隔离与本机保护。

## 实测

每组 40 秒；相同 TOML、1440×1080 RGB、RTX 4060/Vulkan。表中图像指标为排除首帧后 5 秒的数据。PASS 仅表示观察链路、源码一致性和正常退出通过，不是性能阈值验收。

| 构建 | 接收 FPS | 帧龄 p50 / p95（ms） | 接收间隔 p95（ms） | 状态 |
|---|---:|---:|---:|---|
| 原等级1，第一次 | 26.64 | 102.71 / 114.52 | 52.12 | PASS |
| 等级3，实验 | 29.73 | 70.33 / 98.02 | 50.62 | PASS |
| 原等级1，复测 | 26.87 | 105.24 / 118.78 | 59.00 | PASS |
| 等级3，默认构建复测 | 33.09 | 31.76 / 95.17 | 49.37 | PASS |

两次原构建为 26.64 / 26.87 FPS，两次优化构建为 29.73 / 33.09 FPS。对照支持优化构建改善图像管线吞吐，但样本较少、波动明显；不把单次更低帧龄宣称为稳定端到端延迟。编译优化涵盖本体，不足以归因到某一个函数。原先把依赖优化后视作本体已充分优化，会遗漏这部分开销。

7 项远端 capture::driver::tests 通过；6 项本地工具回归通过。远端编译存在既有 unused/deprecated 警告，没有新增编译失败。

## 已确认与尚未确认的原因

- 上轮 11–12 FPS 是整帧视觉处理成功后的计数，不能称为 ROS 接收 FPS。本次独立观察完全没有检测负载，因此不能把本次数据直接称为“视觉算法提速”。
- 当前 GPU 回读为 map_async + 线程池等待，源码不支持“主线程同步等待 GPU 导致低帧率”的结论。
- ROS spin 的 1ms 等待在单独线程中，不存在已查明的主循环每帧 100ms 固定等待。
- 第一次独立观察期间六次 GPU 利用率采样为 20%、21%、24%、15%、28%、18%，不支持当时 GPU 已满载的判断。
- 原构建复测时某未命名线程有多个 1 秒区间约 80–89% 单核 CPU 占用；线程名不足以定位函数，不能直接归因到图像回读、DDS 或主循环。
- 性能计数器采样被目标主机 perf_event_paranoid=4 限制。本轮未改变系统设置，尚未取得函数级调用栈。
- ROS 发布排队、图像配对/处理覆盖、线程竞争仍是未排除因素。本轮改善了一部分通用仿真开销，未证明低帧率只有这一个原因。

## 复现独立观察

```bash
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true run -- bash -c 'source sp_vision_ros/scripts/env.sh && cd bevy_robomaster_simulator && cargo build --locked --no-default-features --features ros2 --bin daedalus -j8 && cd .. && python3 tools/capture_performance.py --seconds 40'
```

独立域、远端保护和完整性检查等工具加固在最终性能场后做了本地回归；各次远端都已经显式指定域 90，并且记录中确认了域和源码一致。工具加固不改变仿真二进制或图像处理。

## 原始证据

- 原等级1，第一次：[20261020-015127-17865fd0](../artifacts/remote/20261020-015127-17865fd0/project-artifacts/capture-performance.json)，同作业目录有 job.log、capture-performance-sim.log。
- 等级3，实验：[20261020-015348-fb2e3116](../artifacts/remote/20261020-015348-fb2e3116/project-artifacts/capture-performance.json)，同作业目录有 job.log、capture-performance-sim.log。
- 原等级1，复测：[20261020-015543-2fc09c93](../artifacts/remote/20261020-015543-2fc09c93/project-artifacts/capture-performance.json)，同作业目录有 job.log、capture-performance-sim.log。
- 等级3，默认构建复测：[20261020-015711-87e8c23a](../artifacts/remote/20261020-015711-87e8c23a/project-artifacts/capture-performance.json)，同作业目录有 job.log、capture-performance-sim.log。

线程采样位于原等级复测作业的 cpu-thread-samples.json；首次 GPU 采样是运行时独立 nvidia-smi 读取，并非报告开始/结束的空闲快照。
