# 工作区约定

公开版本仅提供本地运行入口。不得提交私人主机地址、账号、SSH 配置、个人绝对路径、硬件识别信息或未经审查的运行日志。

```bash
python3 tools/local_run.py doctor
python3 tools/local_run.py check
python3 tools/local_run.py build
python3 tools/local_run.py test
```

构建与图形仿真可能消耗较多资源；仅在任务需要并具备本地依赖时执行。日常变更优先做适当的轻量测试。构建缓存、运行结果、个人配置和本地工具放在已忽略目录中；不得通过强制暂存将它们加入提交。运行说明见 `docs/local-simulator.md`。
