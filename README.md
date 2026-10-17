# RoboMaster 自瞄与仿真工作区

- [全部文档目录](docs/文档目录.md)
- [版本管理与源码来源](docs/version-control.md)
- [装甲标注与训练流水线（building，服务器执行）](sp_training/README.md)
- [可视化远程仿真：本地 / 目标机显示窗口](docs/可视化远程仿真测试.md)
- [仿真 configs 与调参](sp_vision_ros/configs/README.md)
- [SP 新版自瞄 ROS2 仿真接入](sp_vision_ros/README.md)
- [使用报告](sp_vision_ros/docs/使用报告.md)
- [测试说明](sp_vision_ros/docs/测试说明.md)
- [SSH 远程运行与测试](docs/remote-testing.md)
- [仿真回滚与远端原版基线测试](docs/simulator-baseline.md)

已按要求先回滚并完成远端原版基线测试，再完成新版 ROS 自瞄接入。standard3/4 派生配置各三个场景的长时闭环均通过；原 SP 的 230 个文件不变。

在工作区根目录执行，一条命令同时启动目标机仿真与新版自瞄：

```bash
# 使用目标机 RTX 4060，在本地浏览器看仿真（默认 120 秒）
python3 tools/sim_view.py

# 使用目标机性能，在目标机已登录桌面看窗口
python3 tools/sim_view.py --view remote

# standard4 配置 + 横移目标，观察 180 秒
python3 tools/sim_view.py --base standard4 --scenario move --seconds 180

# 复测调优 agent 的高射频候选
python3 tools/sim_view.py --profile standard3_high_rate --scenario spin --seconds 180
```

Ctrl+C 提前停止；本地查看地址会自动打开并在终端打印。入口复用远端构建，源码变更后请先执行远程 `build`。详细操作和验证见[可视化测试文档](docs/可视化远程仿真测试.md)。

本机性能有限，构建和测试默认优先调度到 `wuxian@192.168.1.102`：

```bash
python3 tools/remote_run.py doctor
python3 tools/remote_run.py check
python3 tools/remote_run.py --env CARGO_NET_OFFLINE=true test
```

`test` 包含远程构建、几何/完整性检查、ROS smoke、Rust 测试与三个闭环场景。远端依赖必须先具备；失败不会自动在本机运行。日志与结果回传到 `artifacts/remote/`。


实机部署与标定工具见 [sp_deploy](sp_deploy/README.md)，现场步骤见 [实机实测流程](sp_deploy/docs/实机实测流程.md)。300 秒×6 场景的长时仿真资格测试全部通过；实机相机/电控尚未接入，实际标定和现场命中结果不等同于仿真成绩。
