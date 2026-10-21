# 算法配置

当前 Godot 仿真通过 `tools/aim_lab.py` 在本机启动；`--aim-config` 接受项目内的 YAML，`--sim-config` 接受 Godot CFG。

```bash
python3 tools/aim_lab.py --aim-config sp_vision_ros/configs/standard3.yaml --scenario static
python3 tools/aim_lab.py --aim-config sp_vision_ros/configs/standard3_high_rate.yaml --scenario spin
python3 tools/aim_lab.py --sim-config rm_aim_sim/configs/default.cfg --scenario move
```

`standard3.yaml` / `standard4.yaml` 和 `standard3_high_rate.yaml` 是可审核的调参输入；构建不自动重新生成它们。当前相机标定由同帧图像元数据生成，实机标定文件不被覆盖。`simulator.toml` 属于历史 Bevy 几何回归，不控制当前 Godot 场景。

私人实验配置使用 `*.local.yaml` / `*.local.toml`，默认不入库。共享参数须移除个人路径和设备识别信息后再审核提交。详细入口见 [本机说明](../../docs/local-simulator.md)。
