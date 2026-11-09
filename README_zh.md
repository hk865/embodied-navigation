# 工程入口（中文简版）

请从真实源码目录 [src/embodied_navigation](src/embodied_navigation/README.md) 阅读系统实现，从 [README.md](README.md) 查看运行和验证状态。完整结构、耦合评估与重构建议以根 [README.md](README.md) 和各模块 README 为准，本文件只做导航。

## 目录结构

```text
embodied-navigation/
├── README.md                # 完整说明（结构、耦合、可读性、重构建议、验证状态）
├── README_zh.md             # 本文件：中文简版入口
├── start.py                 # 唯一原生启动入口（--check / --prepare-only / 实际启动）
├── pyproject.toml           # 包元数据与 console script
├── requirements-cpu.lock    # CPU 依赖锁
├── compose.yaml             # Docker Compose 服务定义（默认命令是 --check）
├── Dockerfile               # Native 镜像构建
├── src/                     # 自有源码（embodied_navigation 包，11 个一级模块）
├── configs/                 # base 配置与契约数据（读取顺序见 configs/README.md）
├── scripts/                 # 可发现入口与 DynaNav 实验/回放/评测脚本
├── tests/                   # 离线 CPU 测试（unit / regression / native）
├── docs/                    # 架构、术语、实验、依赖许可、审查记录
├── docker/                  # 构建脚本、源码 pin、私有 DDS
├── third_party/             # vendored 算法与驱动（只接入，不是自有实现）
├── install/                 # 本机构建产物（Git 忽略）
├── assets/                  # 外部数据卷软链：场景资产（Git 忽略）
├── models/                  # 外部数据卷软链：策略权重（Git 忽略）
├── runs/                    # 外部数据卷软链：运行产物（Git 忽略）
├── dynanav/                 # 迁移前历史副本（Git 与 Docker 排除）
├── history/                 # 历史副本（同上）
└── perception_history/      # 历史副本（同上）
```

## 代码、日志与配置

- 代码：`src/embodied_navigation/` 是自有实现；`third_party/` 是 vendored 算法与驱动；`docker/`、`start.py` 是构建/启动装配。
- 日志：本仓库不提交日志。"日志"指运行产物——原生运行写 `runs/<UTC>_<label>_<hex>/`（`runtime_context.json`、`processes.json`、`ros_logs/`、`*.log`、`component_exit.jsonl` 等，`runs/` 是外部数据卷软链）；DynaNav 实验写 `$PROJECT_DATA_ROOT/embodied-navigation/experiments/<run>/`。
- 配置：`configs/` 是 base 配置，不是每次运行的有效值；读取顺序、prepare pin 与无读取点配置见 [configs/README.md](configs/README.md)。

## 模块耦合

包内模块依赖不是严格 DAG（按一级模块统计有 9 组 A↔B 双向依赖，2026-11-09 AST 统计）。明确构成层次倒置的有：规划 import 控制器（`planning/reference/global_route_reference.py` → `control/tracking/cascade_core.Controller`）、定位 import 导航数据类（`localization_mapping/relative_odometry.py` → `navigation.task.Observation`）、传感适配层 import 控制内部（`perception/sensors/feedback_mailbox.py`）、策略 ACK 与停车控制双向依赖、安全门禁同时 import 机器人与规划。逐条证据与其余窄环见根 [README.md](README.md) 的"模块耦合"节与各模块 README。

## 可读性评估

模块级文档可读性好，代码级可读性不均：navigation 11,634 行（文本口径，每文件比 `wc -l` 多 1）/ control 9,718 / planning 9,361 / simulation 9,095 / localization_mapping 8,429 是主要体量；15 个 >800 行文件集中在 navigation、control、planning、robot、localization_mapping、perception，函数 docstring 覆盖约 0.25–0.57（AST 粗计）。

## 重构建议

P1：拆 `cascade_core.py`(1851) 与 `transition_gate.py`(1731)；修 3 处层次倒置（`Observation` 下沉、规划不 import `Controller`、`feedback_mailbox` 只依赖消息契约）；拆 `base_node.py`(1618) 与 `pid_*` mixin 家族。P2：补 docstring、统一运行产物口径、标注无读取点配置。理由、风险与验证方式见根 [README.md](README.md)。

## 边界与限制

DynaNav 只负责仿真场景接入/实验路径，不是生产导航整栈；视觉策略训练独立在 RL_for_unitree；生产整栈完整路线、实机、策略训练与完整 Isaac 链路仍未验收。
