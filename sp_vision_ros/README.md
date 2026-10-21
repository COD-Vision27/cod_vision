# SP 视觉算法与可选 ROS 适配器

当前主仿真为同机 Godot / 共享内存接入，见 [本机运行说明](../docs/local-simulator.md) 和 [SP / Godot 接入](../sp_vision_sim/README.md)。

```bash
python3 tools/local_run.py build
python3 tools/aim_lab.py --scenario static
python3 tools/local_run.py test
```

该目录承载通用 C++ 算法库和 `sp_sim_ipc` 客户端；默认构建不依赖 ROS。可选 ROS 适配器使用 `scripts/build_ros.sh`，需要本机兼容的 ROS 安装和相应接口包。旧 Bevy 后端已退役，历史机器运行记录不作为公开运行依赖。

算法 YAML 位于 `configs/`，不应在构建时覆盖调参。Godot 场景参数使用 `rm_aim_sim/configs/default.cfg`，旧 Bevy TOML 仅用于历史几何测试。
