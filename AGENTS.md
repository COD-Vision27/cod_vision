# 工作区运行约定

本电脑性能有限。构建、测试与仿真优先通过 SSH 别名 `wuxian-remote` 调度到原 RTX 4060 目标机。该别名经 `ubuntu@111.229.26.21` 的反向 SSH 隧道（服务器 loopback 2222）中转，配置见 `~/.ssh/config`；旧局域网地址为 `192.168.1.102`。计算仍在目标机执行，中转服务器只转发连接：

```bash
python3 tools/remote_run.py doctor
python3 tools/remote_run.py check
python3 tools/remote_run.py test
```

详细参数和环境要求见 `docs/remote-testing.md`。若连接、认证或远端依赖失败，报告具体原因并修复可修复的问题；不要自动在本机执行大型 C++/Rust 构建或 Bevy 仿真。轻量脚本检查和调度程序自身的单元测试可在本机运行。用户明确要求本机运行时遵从用户指令。
