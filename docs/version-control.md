# 版本管理

主仓库为 [COD-Vision27/cod_vision](https://github.com/COD-Vision27/cod_vision)，开发分支为 `main`。提交源码、配置、项目所需模型和说明文档；构建缓存、虚拟环境、训练数据集、训练输出与远程运行日志由各目录的 `.gitignore` 排除。

`sp_vision_25` 和 `knowledgeRepository` 作为普通源码目录由主仓库统一管理。此前它们只被记录为 Git 链接，且没有 `.gitmodules`，主仓库无法保存目录内的改动。2026-10-18 将现有源码及本地修改完整纳入主仓库，保留已有许可证、版权说明和来源信息。重新克隆项目无需初始化子模块。

导入前的上游版本：

| 目录 | 上游 | 原始提交 |
|---|---|---|
| `sp_vision_25` | [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) | `bd9f5e798fa3c6dd3b483ae6627796afb41c608d` |
| `knowledgeRepository` | [klbbs/RM_konwlegeRepository](https://github.com/klbbs/RM_konwlegeRepository) | `a2e3884f50b7b09aada3d7c4ea67775e731d430e` |

原嵌套仓库的 `.git` 元数据和导入清单备份在本机 `artifacts/git-backups/`，不上传远端；备份包含原提交历史和暂存状态。后续日常提交、查看差异和推送均从工作区根目录执行。

```bash
git status
git diff
git add <本次修改的文件或目录>
git commit -m "说明本次修改"
git pull --ff-only origin main
git push origin main
```

推送使用普通快进，不覆盖远端历史。测试遵循 `AGENTS.md`，优先发送到 `wuxian@192.168.1.102`；整理版本与推送本身不代表项目已通过全部运行测试。
