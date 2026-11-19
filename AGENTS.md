# AGENTS.md

所有 agent（Claude Code、Codex、DSH 及其子代理）在本仓库工作时必须遵守本文件。规则如果和某个任务书冲突，以本文件为准；需要改规则，先改本文件，并写明原因。详细的设计理由见 [docs/repo-hygiene.md](docs/repo-hygiene.md)；当前的能力分层与重构计划见 [docs/refactor/capability-map.md](docs/refactor/capability-map.md)。

## 1. 项目现状与推进顺序（2026-11-20）

- **唯一开发主线是 D 路径**：DynaNav / Isaac Sim。入口是 `scripts/run_dynanav_native_exploration.sh`，仿真侧代码在 `src/embodied_navigation/simulation/dynanav/`。
- **A 路径**（Gazebo 旧主线，`start.py → launch/main.py` 与私有 Fast DDS）已于 `refactor/06-clean-base` 整块退役删除（历史保全于 `main`、`archive/pre-review-20261109` 与 `refactor/05-agents-rules`），不要恢复。
- **B（Building）和 C（RoboGauge 的 Gazebo 基准）已删除**，不要恢复。RoboGauge 的 MoE-CTS 运动策略运行时在 `robot/unitree/robogauge/` 中，D 在用，必须保留。
- **动态行人支线**已完整保全于参考分支 `ref/dynanav-dynamic-pedestrian`，已从主底座移出；待主线规划、控制与 SLAM 后端稳定后再按需参考重构。
- **Agent 感知与控制介入层**（`operator_map.py`、`semantic_mapping/`、`planning/exploration/`、`explore_waypoint_client.py`、`dynanav_exploration.py`、`native_waypoints.py`、`pct_scan_waypoints.py`、`.agents/skills/dynanav-explore/`）是具身导航的核心能力，作为独立接口层完整保留并持续清晰化。
- **运行环境**：Linux 主机已具备 Isaac Sim 与 ROS 2 环境，可直接进行仿真与原生验证；若在无 Isaac/ROS 的 Windows + WSL 环境工作，仅运行 CPU 单元测试与回归，仿真验证留待 Linux 主机。
- **当前推进顺序**：
  1. **干净底座已成型**（`refactor/06-clean-base`，63 个 `.py` / 14,485 行，0 失败单测）：已完成 P1+P2 死代码/废弃实验清理、P3 动态行人移出主线、A 路径整块退役；后续继续按“仿真 / 通信 / 前端 / 后端 / 规划导航 / 控制执行 / Agent 接口”七层边界保持清晰；
  2. **第一优先验证分支**（`exp/pct-scan-stairs-truth`）：基于干净底座切出，直接用 **仿真真值位姿 + 200 Hz 真值去畸变 RTX 点云**（跳过 SLAM 定位漂移干扰）打通 **PCT + SCAN 上楼梯与多层导航闭环**；
  3. **第二优先验证分支**：规划导航闭环跑通并合并回底座后，基于干净底座按不同 SLAM 后端分别切出独立分支进行大幅精简、参数/协方差/IMU 方程规范化与横向对照，避免各方向互相耦合。

### 多 Agent 分工与协作（2026-11-20 确立）

本仓库采用多 Agent 协同模式，各角色职责明确，严禁越位或互相内耗：

| 角色 | 调用方式 | 定位与职责 |
|---|---|---|
| **主控 Agent（高配模型）** | 当前主会话 | **乐观、敢于做破坏性重构**。负责架构判断、推进大刀阔斧的旧代码清理与模块解耦、给出明确思路/数学公式/优化方案、拆立干净分支并驱动端到端闭环。 |
| **Codex** | 本机 `codex` CLI（如 `codex exec` / `codex exec review`） | **严谨审核器**。只负责审核工程规范、深挖隐含前提、找出设计与实现中的具体错误并将其细化为可执行的修正项，并防止低价执行引入错误（定位是查错与细化前提，而非评判重构是否“过于乐观”）。 |
| **DSH（DeepSeek Harness）** | 本机 DSH 子代理适配器（工作区必须绑定本仓库根目录） | **低成本执行 Worker**。适合按主控/Codex 给出的明确思路、公式、优化方法与验收标准执行批量修改、清理包与返工；**每次派单的 session 必须落在本项目工作区下方便用户核验**，完成后必须经 Codex 审核防错。 |
| **Flash 子代理** | `invoke_subagent(..., Model="flash")` | **高性价比编码帮手**。成本低且代码编写质量高，适合承接边界清晰的功能开发、模块拆分、定向清理与单元测试编写。 |

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
6. **代码、配置和文档里不写本机绝对路径**（`/home/<user>`、`/media/<user>`、`D:\...`）。路径统一用 `${PROJECT_ROOT}`、`$PROJECT_DATA_ROOT` 或参数传入。这条只约束写进仓库的内容，不限制读取项目外的数据，见第 6 节。
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
- 验收标准是：全量单测和回归的失败名单，相对起点不能新增。当前干净底座（`refactor/06-clean-base`）基线为 **514 条单测 0 失败 0 跳过、60 条回归 0 失败**（旧 5 条历史失败已随死代码清理全部归零）。
- 删除一个包时，只测试被删代码的测试文件可以一起删掉，但必须在提交信息里点名。

## 5. 分支、提交与验收

- 重构按包叠加分支：`refactor/NN-<名字>`，后一个分支建在前一个之上，最顶端的分支就是全部改动的合集。
- 每个包都要验收后才能交付。验收包括两步：在 LF 换行的克隆上独立重跑全量测试；逐行读完 diff。**由用户推送。** agent 不 push，也不 force-push。
- 难包（核心规则、跨模块）采用“主控/Flash 施工 + Codex 独立审核前提与错误（或独立写验收测试）”的做法。低风险删除包与机械返工可交给 DSH 或 Flash 子代理。
- 提交信息说明做了什么和为什么。不提交密钥，也不提交 `.env*`。

### 相关分支为何保留

以下为 2026-11-20 的盘点。开工先核对 `git status --short --branch`、`git branch -avv` 和相应报告。远端可用 `git ls-remote --heads origin` 只读核对。运行报告记录实际提交及工作树差异。

| 分支 | 用途和复核要求 |
| --- | --- |
| `main` / `origin/main` | 集成、发布和独立验收入口。按验收和审阅结果由用户决定合并及推送 |
| `refactor/00-plan-docs` 至 `refactor/05-agents-rules` | `00` 计划、`01` 几何去重、`02` 死代码、`03` 移除旧路径、`04` SLAM 后端、`05` 协作规则。逐包验收 |
| `refactor/06-clean-base`（当前干净底座） | 基于 `refactor/05-agents-rules` 完成 P1+P2 清理、P3 动态行人移出、A 路径整块退役（累计精简 >8.1 万行，0 失败单测）；后续各专项实验分支均从此干净底座切出 |
| `exp/pct-scan-stairs-truth`（第一优先验证分支） | 基于 `refactor/06-clean-base` 切出，直接用仿真真值位姿 + 200 Hz 真值去畸变 RTX 点云驱动 PCT + SCAN 上楼梯导航闭环（跳过 SLAM 干扰） |
| `ref/dynanav-dynamic-pedestrian`（动态行人参考分支） | 保全移除前的全部动态行人/学习分割支线代码与测试，供后续动态场景开发参考 |
| 本地 `exp/two-floor-demo`（`e378b8d`）及 `backup/exp-two-floor-demo-wip-20261120`（`3720814`） | 隔离并完整保全拆分支前的两层联调代码、楼梯补丁、SLAM 评测与采集改动，供按主题提取到干净分支，不在原分支继续叠加混合改动 |
| `refactor/05-loop-evidence-base`（`d582b28`） | 保全回环共同比较的实现、配置和证据 |
| `exp/loop-measured-submaps`（`d44356c`） | 隔离多帧实测 LiDAR 子图对回环配准的影响。核对查询与历史子图、锚点及接受率 |
| `exp/loop-surface-gate`（`f98372f`） | 对照平面和表面覆盖过滤 ICP 回环的作用，分别报告误配和漏配。独立地面高度因子另立实验 |
| `exp/loop-visual-retrieval`（`2b4f01f`） | 图像检索补充候选，几何配准验证相对位姿。记录实际使用的候选与图边，光度因子和视觉 BA 另立实验 |
| `improve/module-docs-readability`、本地 `feat/navigation-review-web` | 模块文档和阅读入口，独立于运行模式与回环性能改动 |
| `archive/pre-review-20261109`、`wip/navigation-local-snapshot-20261115`、`backup/slam-controlled-experiment-20261116` | 分别保全审查前源码、在途快照、受控 SLAM 追踪与评价及反向航点。按需追证或提取明确修复 |

三个回环方案均已核实远端 ref。到项目数据卷的 `runs/20261117-loop-comparison/20261117T134243Z/REPORT.zh-CN.md` 读比较设计，用 `runs/20261119-hba-project-comparison/` 核对同输入、同时间轴和同对齐的逐帧及最终图。报告收益、失败和适用场景。出现回归、缺少可比证据或用户搁置时，保留结论并停止推广。目标场景和回归均有证据后交用户审阅集成。官方 `hku-mars/HBA` 是外部离线基线。

## 6. 数据在项目外

从所选 checkout 的 `.env.local` 读取 `PROJECT_DATA_ROOT`。Linux 上应解析到新加卷的 `6.project_data`，工程数据根是 `$PROJECT_DATA_ROOT/embodied-navigation/`。同时阅读该数据根的 `AGENTS.md`。

- **开跑前**用 `findmnt -T` 核对挂载设备，用 `df` 核对容量。录包、点云、图像、视频和密集诊断写到数据卷对应项目。挂载、容量或权限不满足时暂停新增采集，避免回退到 `/home`、`Documents/Codex` 或另一项目。工具放行在本机单独配置。
- **写入位置**默认新建 `runs/<批次>/<run-id>/`。旧入口确需 `experiments/` 或 `runtime_assets/runs/` 时沿用已配置路径并写入运行记录。缓存和资产按数据根规则放置。历史记录按其报告所列路径读取。
- **最小清单**使用 `manifest.json` 或等价收据。记录分支、commit 和实际工作树差异、冻结配置和外参、源 bag、真值、坐标系与时间基准、完整性或覆盖校验、结果和报告路径、复现入口、结束状态及备份位置。校验在归档或评测阶段完成。密集诊断限定容量、时间窗和所需信号。
- **何时清理**先确认评分和报告完成、复现材料归档、无进程占用或下游依赖，并核对用户明确批准的范围。原始包、真值、外参、时戳、基线及发布或复盘所需的唯一材料优先保留。可重建缓存、临时件和副本先验证重建来源或备份等价性。不可恢复的数据需用户针对具体对象当场确认。清理采用人工触发，记录对象、恢复来源和实际释放量。

## 7. README 维护

负责改动的 agent 在顶层目录及职责、主链路或启动入口、分支用途、数据入口改变时，同步维护根 `README.md` 并核实相关模块入口。布局和留存变化时同时更新项目数据根的 `AGENTS.md`。标明盘点日期并区分本地检出、远端分支、独立实验和归档。核对链接、实际路径及 `git diff --check`。

## 8. 沟通

- **汇报、进度和结论一律用中文。** 英文只用于代码标识符、路径和命令。
- 汇报时说清楚：改了什么、在本机验证到什么程度、哪些地方没有验证（例如本机跑不了 Isaac）、服务器上应该重点看什么。
- 否定性的结论（“没有调用方”“恒为关”）必须附上 `git grep` 的结果或者配置的取值，不能凭文件名下判断。
