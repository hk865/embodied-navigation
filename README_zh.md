# 工程入口（中文简版）

> 盘点日期：2026-11-20 · 分支 `refactor/06-clean-base` · HEAD `5b880b2`

完整说明已合并到根 [README.md](README.md)（中文），本文件只保留导航入口，避免与新文档不一致。

## 从这里开始

| 想了解 | 读这里 |
| --- | --- |
| 项目定位、端到端数据流、顶层目录、快速启动命令 | [README.md](README.md) |
| 当前阶段目标、Phase 1–4 里程碑、分支归属 | [docs/plan/ROADMAP.md](docs/plan/ROADMAP.md) |
| 活体待办、角色分工与验收标准 | [docs/plan/TASKS.md](docs/plan/TASKS.md) |
| 全部活跃子模块的职责与逐文件说明 | [src/embodied_navigation/README.md](src/embodied_navigation/README.md) |
| 入口脚本、配置、测试 | [scripts/README.md](scripts/README.md) · [configs/README.md](configs/README.md) · [tests/README.md](tests/README.md) |
| 文档分类索引（架构、实验历史、审查、验证、依赖） | [docs/README.md](docs/README.md) |
| 协作规则、测试与分支验收、数据卷规则 | [AGENTS.md](AGENTS.md) |

## 三条最常用命令

```sh
# 默认自主探索：FAST-LIVO2 前端 + 实测 PCT 全局 + 原生 SCAN 局部 + RoboGauge 执行
bash scripts/run_dynanav_native_exploration.sh

# 真值位姿调试模式（跳过 SLAM，仅用于隔离规划问题）
bash scripts/run_dynanav_native_exploration.sh --truth-pose

# CPU 单测 / 回归（当前基线 646 + 60 全绿，回归本机 3 条按环境 skip）
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
```

## 边界

- 唯一主线是 D 路径（Isaac Sim / DynaNav）；Gazebo A 路径已整块退役，不要按旧文档恢复 `start.py` / `launch/` / `control/`。
- 运行产物写数据卷 `$PROJECT_DATA_ROOT/embodied-navigation/`，不进仓库；数据卷布局见根 [README.md](README.md) 第 5.3 节。
- 仿真、GPU 推理与实机验收不在 CPU 测试内，未验证项必须在汇报中写明。
