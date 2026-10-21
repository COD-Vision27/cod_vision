# SP Vision / Godot 同机仿真接入

引擎源码来自 [ForgetWW520/rm_aim_sim](https://github.com/ForgetWW520/rm_aim_sim)，固定版本见 `../rm_aim_sim/UPSTREAM.md`。保留 MIT 许可。默认不需要 ROS、Cargo 或 Bevy；Godot 二进制与导入缓存只留在远端。

```bash
# 已配置的 RTX 4060 目标机；连接和环境异常不会回退到本机
python3 tools/remote_run.py doctor
python3 tools/remote_run.py build
python3 tools/remote_run.py test
python3 tools/remote_run.py --skip-build --scenario spin --seconds 30 closed-loop
python3 tools/sim_view.py --scenario move --seconds 120
```

首次准备远端引擎（当前机器已完成）：

```bash
python3 tools/remote_run.py run -- python3 rm_aim_sim/tools/setup_godot.py
python3 tools/remote_run.py run -- bash rm_aim_sim/run.sh --headless --editor --import --quit
```

下载助手校验固定版本 ZIP 的 SHA256；网络受限时可把同一官方 ZIP 分段传入远端，再用 `setup_godot.py --archive /absolute/file.zip` 安装，校验不变。不要同步 `.tools`、`.godot` 或 raw captures 到本机。

`tools/aim_lab.py` 在远端同时启动图形版 Godot 和 C++ `sp_sim_ipc`。没有远端桌面时调度器使用 Xvfb。`--headless` 只适合物理单元测试，不能验证视觉闭环。

SP 在线输入是实际 RGB 图像、该帧相机标定与该帧云台姿态，经 YOLO → Solver/PnP → Tracker/EKF → Planner/TinyMPC 输出指令。不读取 truth 或理想装甲坐标。模拟器和视觉算法在同一台主机通过 `/dev/shm` 通信，SSH/浏览器只负责查看与调度。远程桌面画面延迟仍受网络限制。

默认 OpenVINO 推理使用 4 线程、OpenCV 使用 1 线程，减少与渲染/物理争用；YAML 的 `inference_num_threads: 0` 可恢复 OpenVINO 自动策略。这只在仿真客户端中设置；SP 原实机配置缺省仍为自动策略。

100 Hz 控制循环与视觉线程分开，待处理图像最多一帧。推理期间继续发送控制指令；目标过期、暂停、重启或标定变更会清掉旧目标。Publisher/session/相机上下文变更使在途推理失效。每个实例使用独立 IPC channel。帧时间与 `Planner::plan_at()` 使用同一模拟时钟；墙钟只用于连接和目标看门狗。原 `Planner::plan()` 实机接口仍使用 steady_clock。

当前默认相机、弹速、命中与血量参数来自 `rm_aim_sim/configs/default.cfg`。本轮用户确认 RMUL 3v3 普通步兵：350 HP，单发 20 伤害，18 次有效命中。离线计分读取真实弹丸碰撞事件；首发至击毁 TTK 与模拟器启动至击毁的时间分别保存。`--health 10000` 只用于持续命中率工况，不得拿其 TTK 当作比赛血量成绩。

结果在远端 `sp_vision_ros/artifacts/aim-lab-<scenario>-*`，本次修改的文件回传到本地 `artifacts/remote/<job>/project-artifacts`。`*-states.jsonl.gz` 保留实际反馈/逐发事件，`*-vision.jsonl.gz` 保留处理与回读耗时，`*-first.png` 是实际摄像头首帧。通过条件只要求没有处理异常且存在实际命中，并不等同于赛场资格验收。

相机标定按同步 frame 元数据生成临时 YAML，实机标定文件不被覆盖。旧 Bevy 几何只留小型历史回归 fixture。ROS 接入源码与消息定义仍可单独通过 `sp_vision_ros/scripts/build_ros.sh` 构建，但不属于当前仿真数据链路。

Godot 客户端显式选择 RTX 4060 / Vulkan Mobile、关闭 V-Sync、限制最高 120 FPS；实际 FPS 会随负载变化。Xvfb 下默认 OpenGL 可能落到 llvmpipe 软件渲染，不能仅看到窗口启动就认为在使用 GPU。

逐帧原始 JSONL 默认无损 gzip 压缩后回传，可用 `gzip -dc` 或 Python `gzip.open` 读取，避免反复测试占用大量本机空间。`*-provenance.json` 记录输入配置、实际仿真 CFG、引擎与程序 SHA256。

更换主机可执行 `python3 tools/remote_run.py --host sim-new --user YOUR_USER --seconds 15 validate`。首次准备、依赖要求、快速复测和帧率指标见 [新主机验证说明](../docs/new-worker-validation.md)。

[本机开窗口及本机 SP 自瞄使用说明](../docs/local-simulator.md)；完整本机入口：`python3 tools/aim_lab.py --local --scenario static --seconds 30`（需先准备引擎并完成本机构建）。
