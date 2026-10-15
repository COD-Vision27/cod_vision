# 仿真回滚与远端原版基线测试

日期：2026-10-15。目标：`wuxian@192.168.1.102`。

按用户要求，先撤销本次对仿真器的修改，再在性能更高的目标机上检查原版加载与渲染。原版测试完成之前不重新改动仿真器。

## 回滚范围

撤销本次 15 个文件的改动：ROS 新轨迹消息、同帧 TF 发布与命令接收、云台轨迹控制、测试场景、材质/网格加载兼容调整、照明参数与弹丸 CCD，以及仿真 README 中的接入介绍。原 `config.toml` 和 GLB 资产始终未改动。恢复了修改前的云台 PID、场景和装甲构造逻辑。

回滚前副本、逐项修改记录和恢复后摘要保存在 `sp_vision_ros/artifacts/rollback-20261015/`。`sp_vision_25` 的 230 个文件完整性校验通过。独立 ROS 适配器、推导出的相机矩阵及旧测试资料保留为草稿；测试期间用 `SIMULATOR_INTEGRATION_PAUSED` 阻止旧二进制继续运行，基线通过后改名为 `BASELINE_TEST_COMPLETED` 并开始必要接入。

## 原版测试入口

```bash
python3 tools/remote_run.py run -- xvfb-run -a \
  --server-args="-screen 0 1920x1080x24" \
  python3 tools/simulator_baseline.py --jobs 8 --seconds 45
```

`XAUTHORITY` 路径随图形登录变化，应读取目标机实际会话。没有图形会话时可省略这些环境参数，程序使用 `xvfb-run`；必须从渲染日志确认使用 RTX 4060，不能把软件渲染作为硬件性能结果。

测试入口只读原仿真配置、源代码和资产，执行 `cargo test/build --locked --no-default-features`，再运行原场景并记录加载完成、实际 Vulkan 适配器、帧率和退出状态。F2 截图使用原仿真已有功能。运行前后逐项校验源码、配置与资产哈希。报告与日志由调度程序回传到本机 `artifacts/remote/`。

这一轮首先验证不启用 ROS 的原版渲染与物理场景。它不代表 ROS、自瞄或命中闭环通过。后续应在同一硬件上单独验证原版 ROS 捕获，再据实决定接入改动。

## 环境诊断与处理

目标机：Ubuntu 24.04、16 线程、32 GB 内存、RTX 4060。初始缺少 Rust/CMake 等构建依赖。显卡驱动初始不能工作：运行内核 `6.14.0-36-generic`，已安装 NVIDIA 550 模块只对应 `6.14.0-27-generic`，`modinfo nvidia` 返回模块不存在，`nvidia-smi` 无法连接驱动。

用户授权配置环境后，安装构建依赖与 `ubuntu-drivers devices` 推荐的 NVIDIA 595 open 驱动，为当前内核构建 DKMS 模块；Rust 工具链使用与项目原机器相同的 1.99.0。驱动是否实际可用仍以 `nvidia-smi`、Vulkan 和仿真运行结果为准。

## 测试结果

原版远端测试已完成：

- 作业 `20261015-021437-3f51556b`：原版 69 项 Rust 测试全部通过，构建成功；RTX 4060 / Vulkan 渲染，场景约 3.03 秒加载完成，连续运行 45 秒正常退出，最后 10 条诊断平均帧率约 35.94 FPS。
- 作业 `20261015-022258-90291fa1`：复用原版二进制补取截图，运行 30 秒正常退出，采集 3 张原仿真 F2 截图。画面中能看到场地、机器人、灯条、数字和棋盘格标定板。
- 两次运行前后源码、配置和资产哈希不变；远端 131 个文件也与本地回滚后的文件完全一致。
- 本机调度程序 6 项轻量测试通过；SP 230 个原文件完整性校验通过。

首次截图通过 `xdotool key --window` 发送的合成事件没有被游戏读取，后改为在专用 Xvfb 显示中聚焦游戏窗口并通过 XTest 发送按键；未修改仿真截图实现。

原版第一人称初始画面主要是灰色天空和自己的枪管；切换视角后能看到完整场景。因此画面灰暗/没有敌方装甲，不能直接判断成“模型尚未加载”。仍需在原 ROS 图像上确认视角、相机姿态和实际采集延迟，再决定自瞄接入改动。

原版 ROS 消息已在 ROS2 Jazzy 编译安装成功，随后实际编译并用 `tools/ros_image_baseline.py` 观察图像和 CameraInfo（不启动或控制自瞄）。这些原版渲染测试不代表自瞄、控制和命中闭环通过。此前本机闭环未检出装甲/未命中的结果仅为旧接入版本的失败记录。


测试报告：[首次原版构建与运行](../artifacts/remote/20261015-021437-3f51556b/project-artifacts/baseline_result.json)、[补取截图运行](../artifacts/remote/20261015-022258-90291fa1/project-artifacts/baseline_result.json)。

![原版场景截图](../artifacts/remote/20261015-022258-90291fa1/project-artifacts/baseline_screenshot-1.png)

ROS 安装依据为 [官方 Jazzy 安装文档](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst)，当前锁定的 r2r 0.9.5 源码也包含 Jazzy 支持。

ROS 图像观察报告中的 `header_to_receipt_ms_*` 只测消息 Header 到接收的时间差。原版时间戳由现有捕获回调生成，不能单凭这个数值认定图像对应的曝光/渲染时刻或完整帧龄。


### 原版 ROS 编译结果与最小修复

作业 `20261015-022651-33a96ea5`：原版消息包生成与安装成功；原版 ROS 仿真编译失败，Rust E0599 指向 `src/ros2/plugin.rs` 的 `TextureFormat::bevy_default()`，原因是 `BevyDefault` trait 未导入。失败日志已回传，证明此问题属于代码编译兼容性，并非 GPU 加载慢。

在完成原版测试后，根据依赖源码的公共重导出新增 `use bevy::image::BevyDefault;`，保留原来的默认纹理格式。材质、照明、网格解析、弹丸 CCD、原 PID 与场景继续保持回滚后的实现。随后再验证 ROS 构建和图像采集。

中间验证作业 `20261015-023419-b6b4b503` 发现编译器建议的 `bevy::bevy_image` 路径不可访问；检查实际 `bevy_internal` 源码后改用公共重导出 `bevy::image`。

ROS 构建验证作业 `20261015-023514-030f88a4`：成功（18.30 秒）。该阶段仅新增 trait 导入，未恢复之前的仿真参数或实现变更。随后启动 ROS 捕获运行作业。

ROS 原版采集运行作业 `20261015-023605-b358f229`：收到 13 帧图像和 20 条 CameraInfo，随后进程异常退出（-6），未通过持续运行测试。Header 到接收延迟中位数约 35.56 ms、p95 约 210.92 ms；需依据错误日志继续定位退出原因。


ROS 运行错误定位：`capture_rune` 在原 `plugin.rs:219` 对 `CENTER` 的 `.any().one()` 结果调用 `unwrap()`，收到首批 RGB 后随场景加载发生空值 panic。基于目标机已复现的错误，查询直接命名的 `CENTER` 定位节点，并在节点或 GlobalTransform 尚未可用时跳过该帧。两处中心点访问均移除空值 unwrap。此变更仅影响 ROS TF/Marker 发布，未修改模型、材质、照明、物理或云台控制。


### ROS 持续采集结果

最小修复后的作业 `20261015-023912-b168066e`：运行 40 秒正常退出，收到 800 帧 RGB 和 848 条 CameraInfo，约 20 帧/秒；Header 到接收延迟中位数 85.73 ms、p95 101.11 ms。运行前后源码、配置和资产哈希未改变。报告的 variant 名仍是上一版导入修复标签（同步时观察脚本尚未更新标签）；实际编译与运行源码包含此次已记录的中心点查询修复。

结果：[ROS 采集报告](../artifacts/remote/20261015-023912-b168066e/project-artifacts/baseline_ros_result.json)、[采集日志](../artifacts/remote/20261015-023912-b168066e/project-artifacts/baseline_ros_simulator.log)。此测试只验证原 ROS 采集稳定性；没有启动自瞄或发出云台/开火命令。


### 基线通过后的重新接入

保留经远端复现确认的 trait 导入、CENTER 定位查询修复，恢复 ROS 新轨迹消息、同帧姿态和云台导数控制，以及显式启用的静态/旋转/横移测试场景。未恢复先前的材质、网格层级、照明、全局物理子步或弹丸 CCD 改动；原 `config.toml` 和模型继续只读。横移测试用敌方位置/速度反馈驱动原底盘，在前方 4 米处围绕 ±0.8 米参考轨迹往返，普通交互模式不受影响。

重新接入的验证记录与真实命中结果见 [测试说明](../sp_vision_ros/docs/测试说明.md)。这与上面的原版渲染/采集基线分开记录。
