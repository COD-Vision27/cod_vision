# RoboMaster 自瞄与仿真工作区

当前仿真使用 [rm_aim_sim / Godot](https://github.com/ForgetWW520/rm_aim_sim)，SP Vision 通过同机共享内存处理真实相机图像。引擎、构建、仿真均在 `wuxian-remote` RTX 4060 主机运行；本机只同步源码、调度和查看。连接通过 `111.229.26.21` 反向 SSH 隧道中转。

```bash
python3 tools/remote_run.py doctor
python3 tools/remote_run.py build
python3 tools/remote_run.py test

# 本地浏览器查看远端闭环；Ctrl+C 停止
python3 tools/sim_view.py --scenario spin --seconds 120
# 在目标机已登录桌面显示
python3 tools/sim_view.py --view remote --scenario move
# 只跑测试并收集结果
python3 tools/remote_run.py --skip-build --scenario static --seconds 25 closed-loop
```

测试包含 SP C++ 几何/规划、源码审计、Godot 物理/通信测试以及 static/spin/move 的真实图像闭环。Godot 采用无 ROS 的 `/dev/shm` 通信。SSH 和浏览器画面的网络延迟不进入视觉控制链路。原 Bevy 后端及其构建缓存已退役，归档与替换验证见下方迁移报告。

- [新仿真使用与时序设计](sp_vision_sim/README.md)
- [替换、空间清理与赛场适用性报告](docs/godot-migration-2026-10-20.md)
- [SSH 远程运行约定](docs/remote-testing.md)
- [全部文档目录](docs/文档目录.md)
- [SP Planner 计算优化](sp_vision_25/docs/planner-latency-2026-10-20.md)
- [上一轮仿真对齐问题](docs/alignment-fix-2026-10-20.md)
- [实机标定与部署](sp_deploy/README.md)
- [实机实测流程](sp_deploy/docs/实机实测流程.md)
- [训练流水线](sp_training/README.md)
- [源码与版本管理](docs/version-control.md)

历史 Bevy 报告只对应各自注明的源码与工况，不能作为新引擎或实机成绩。SP 的通用计算优化可以在实机使用；实际相机、曝光时间戳、电控延迟、弹速和标定仍须实测。

更换主机可执行 `python3 tools/remote_run.py --host sim-new --user YOUR_USER --seconds 15 validate`。首次准备、依赖要求、快速复测和帧率指标见 [新主机验证说明](docs/new-worker-validation.md)。

[本机开窗口及本机 SP 自瞄使用说明](docs/local-simulator.md)；完整本机入口：`python3 tools/aim_lab.py --local --scenario static --seconds 30`（需先准备引擎并完成本机构建）。

交互式本机入口 `python3 tools/aim_lab.py --local`、远程窗口入口 `python3 tools/sim_view.py` 默认持续运行、不限帧率；用 `--seconds 30` 定时，或 `--max-fps 60` 设置渲染上限。关闭窗口或 Ctrl+C 停止。
