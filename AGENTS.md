# AGENTS.md

所有 agent（Claude Code、Codex、DSH 及其子代理）在本仓库工作时必须遵守本文件。规则如果和某个任务书冲突，以本文件为准；需要改规则，先改本文件，并写明原因。详细的设计理由见 [docs/repo-hygiene.md](docs/repo-hygiene.md)；当前的能力分层与重构计划见 [docs/refactor/capability-map.md](docs/refactor/capability-map.md)。

## 1. 项目现状（2026-11-15）

- **主线是 D 路径**：DynaNav / Isaac Sim。入口是 `scripts/run_dynanav_native_exploration.sh`，仿真侧代码在 `src/embodied_navigation/simulation/dynanav/`。
- **A 路径**（Gazebo 主线，`start.py → launch/main.py`）暂时保留，正在逐步退役。不要为 A 新增功能。
- **B（Building）和 C（RoboGauge 的 Gazebo 基准）已删除**，不要恢复。RoboGauge 的 MoE-CTS 运动策略运行时在 `robot/unitree/robogauge/` 中，D 在用，必须保留。
- 本机（Windows + WSL）跑不了 Isaac，也没有 ROS。仿真和实机测试由用户在服务器上进行。

## 2. 分层与目录

代码按层放置。新代码先判断属于哪一层，再决定放进哪个目录：

| 层 | 目录 | 职责 |
|---|---|---|
| 仿真 | `simulation/` | 场景、资产、传感器仿真，以及仿真到 ROS 的桥 |
| 感知 | `perception/` | 传感器消息适配：自车过滤、去畸变配对、健康门 |
| 定位建图 | `localization_mapping/` | SLAM 前端接入；后端（`backend/`）负责关键帧、回环、位姿图、地图保存与加载、先验重定位 |
| 规划 | `planning/` | 全局路线（PCT）、局部规划（SCAN）、路径准入 |
| 控制 | `control/`、`navigation/` | 路线跟踪、停-转-稳、停车、运行期安全门；输出到 cmd_vel 为止 |
| 机器人执行 | `robot/` | 速度命令 → 策略 → 关节目标 → 执行器 |
| agent 接口 | （待建，见 `docs/design/agent-control-api.md`） | 目标下发、运行中干预、状态查询 |
| 基础设施 | `launch/`、`common/` | 启动编排、配置、公共工具 |

- `scripts/` 只放薄的 CLI 入口：解析参数，再调用包内函数。业务逻辑放进 `src/`。不要新增超过 300 行的脚本。
- 地图管理、关键帧、回环属于**定位建图**，不属于感知。

## 3. 硬规则

1. **不用环境变量做功能开关。** 不要新增 `R88_*`、`R92_*` 这一类开关。验证通过的行为就是唯一实现，旧分支直接删除。需要排查问题时，开 `exp/<批次>` 分支，或者新建一份配置 overlay 来调阈值。
2. **标识符和配置键不带版本号、日期或批次名**（不要用 `_r85_`、`r109_speed_tier` 这种写法），一律按职责命名。批次号写在提交信息或实验记录里。
3. **诊断只能观察，不能改变决策。** 诊断代码不得把 `allowed=False` 改成 True，也不得跳过超时或业务门。不要再新增空的探针调用（`_observer_api(...)` 一类）。需要追踪时序时，用一个真正写盘的 Tracer，并用配置打开。
4. **不加没有依据的校验。** 防护外部输入（ROS 消息、文件、socket、CLI）的校验要保留。内部可信值、上游已经校验过的值、当前调用图里不可能出现的情况，都不写防御分支。新增一项检查前，先说清它防的是哪个真实会发生的场景。
5. **不复制已有的实现。** 写新函数前先 `git grep` 查有没有现成的；例如 `common/geometry.py` 里已有 `wrap_angle` 和 `quaternion_matrix`。确实要共用时，删掉旧副本，只保留一份。
6. **不写本机绝对路径**（`/home/<user>`、`/media/<user>`、`D:\...`）。路径统一用 `${PROJECT_ROOT}`、`$PROJECT_DATA_ROOT` 或参数传入。
7. **运行产物不进仓库**，包括日志、bag、点云、视频、逐帧 JSON、审计台账和 source snapshot。它们放在 `$PROJECT_DATA_ROOT/embodied-navigation/runs/<批次>/<run-id>/`。仓库只放实验定义和不超过 300 行的结论报告。单个文件超过 200 KB 默认不允许提交。
8. **`docs/experiments/`、`docs/review/` 不再新增文件。** 过程记录放进 `experiments/<批次>/REPORT.md`，或者留在数据卷。
9. **vendored 代码（`third_party/`）不原地改格式、不改注释。** 逻辑修改走 `third_party/<包>/patches/`。
10. **控制器只从启动器启动**，运行时配置只来自 run 目录里冻结的那一份。

## 4. 测试

- 运行环境：WSL 或 Linux，Python 3.12，依赖只有 numpy 和 scipy。Windows Git Bash 里的 `python3` 是占位符，不能用。
- 命令（在仓库根执行）：

  ```bash
  PYTHONPATH=src python3 -B -m compileall -q src scripts
  PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
  PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
  ```

- 测试用 unittest。共用的 fixture 放在不以 `test_` 开头的模块里（例如 `tests/unit/slam_backend_fixtures.py`）。
- **不得为了让测试通过而删除或放松旧断言、加 skip、调大超时。** 某个旧测试只有改了才能通过时，停下来，写清楚冲突在哪。
- 验收标准是：全量单测和回归的失败名单，相对起点不能新增。当前基线上有 5 条已知失败（`test_dynanav_rtx_deskewed_perception` 4 条、`test_dynanav_source_snapshot` 1 条），它们与重构无关。
- 删除一个包时，只测试被删代码的测试文件可以一起删掉，但必须在提交信息里点名。

## 5. 分支、提交与验收

- 重构按包叠加分支：`refactor/NN-<名字>`，后一个分支建在前一个之上，最顶端的分支就是全部改动的合集。
- 每个包都要验收后才能交付。验收包括两步：在 LF 换行的克隆上独立重跑全量测试；逐行读完 diff。**由用户推送。** agent 不 push，也不 force-push。
- 难包（核心规则、跨模块）采用“一个 agent 施工、另一个 agent 同时按同一份裁定写验收测试，两边互相看不到”的做法。普通的删除包和返工交给 DSH。
- 提交信息说明做了什么和为什么。不提交密钥，也不提交 `.env*`。

## 6. 沟通

- **汇报、进度和结论一律用中文。** 英文只用于代码标识符、路径和命令。
- 汇报时说清楚：改了什么、在本机验证到什么程度、哪些地方没有验证（例如本机跑不了 Isaac）、服务器上应该重点看什么。
- 否定性的结论（“没有调用方”“恒为关”）必须附上 `git grep` 的结果或者配置的取值，不能凭文件名下判断。
