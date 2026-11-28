# 工程文档索引

本目录是工程文档与证据的入口：活体开发计划、当前功能说明、历史实验记录、审查与验证回执、依赖与部署专题。文档不参与运行时，也没有代码读取本目录。盘点日期 2026-11-28：集成基线 `main`（`2144e5f`，已推送 `origin/main`），主力记忆/Agent 分支 `feat/static-memory-streaming`（`e159c8a`，948 单测 / 60 回归），D 路径 V4 闭环分支 `exp/live-v4-ba-sliding-window`（`aada1f1`）。

## 开发计划入口（新会话先读）

- [开发路线图 ROADMAP](plan/ROADMAP.md)：唯一主线（D 路径）、七层架构边界、Phase 1–5 里程碑与当前状态、3 主线 + 4D 8 分支推进清单、分支归属、数据卷约定。
- [七层冻结基线 BASELINES](plan/BASELINES.md)：L1–L7 内部冻结基线、单卡 16 GB 与 Colab 1,000 CU 算力红线、标准化数据集与子模块边界。
- [活体任务板 TASKS](plan/TASKS.md)：已完成 / 进行中 / 待办队列与远期触发式架构优化，每条任务带建议角色、对应模块与验收标准。
- [活跃分支测试计划 TEST_PLANS](plan/TEST_PLANS.md)：每个活跃分支的合入验收清单、Phase 4 统一评测数据集（D1/D2/D3）与 `B0–B3` / 三条回环分支消融口径。
- 专题调研与决策蓝图 `plan/research/`：[2026-11 学术创新查重与论文方向总评](plan/research/2026-11-novelty-audit-and-directions.md)、[VLN 评测真相与「感知 + 记忆 + 失败驱动控制」演进总纲](plan/research/2026-11-vln-perception-memory-strategy-synthesis.md)、[3D/4D 空间表征、混合记忆检索、动态视觉-LiDAR 与算力策略 Trade-Off](plan/research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)、[ADR 深度调研蓝图](plan/research/adr-0002-0003-0004-survey.md)、[四大方向入门精讲与总施工图](plan/research/four-directions-roadmap-and-blueprint.md)。
- 模块教程：[Agent 技能/工具/视频流与记忆教程](agent-tutorial/README.md) · [SLAM 前端至 V4 后端教程](slam-tutorial/README.md)。
- 架构决策记录 `plan/adr/`（5 篇，状态与关联分支见各文件首部）：[ADR-0001](plan/adr/0001-livo-yaw-pipeline-overlap.md)（LIVO 独立 `yaw` 延迟决策与边界，已采纳）、[ADR-0002](plan/adr/0002-embodied-agent-async-and-map-outlook.md)（具身 Agent 异步行动、语义/动态地图与热路径演进备忘）、[ADR-0003](plan/adr/0003-streaming-fork-control-and-semantic-scene-graph.md)（单主干流式 Fork 异步控制、关键帧锚定语义图片图数据库与停驻期 ZUPT 约束；已按 ADR-0005 瘦身为事件驱动仲裁）、[ADR-0004](plan/adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)（面向 FAST-LIVO2 的增量 Mesh / 3DGS 演进与三层 SLAM 后端评测口径）、[ADR-0005](plan/adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)（因果关键帧与过程记忆、Mesh+骨架压缩预测底座、树/图检索与 3 主线 + 4D 8 分支规划）。
- 协作、分支与提交规则以 [AGENTS.md](../AGENTS.md) 为准；项目整体入口见根 [README.md](../README.md)。

`docs/plan/` 是 2026-11-20 文档重建时新增的活体计划目录（2026-11-21 起含 `plan/adr/` 架构决策记录），会随阶段推进更新；本文件其余各节是**相对稳定**的说明与历史证据索引。

## 文档体系与职责

- 架构层级：不属于七层运行栈（仿真 / 感知与通信 / 定位建图 / 规划 / 控制 / 机器人执行 / Agent 接口），是**知识层与证据层**。
- 输入：源码、配置、run 目录与外部数据卷里的运行收据；产物是人工整理的说明、报告与机器可读回执。
- 上游 / 下游：被根 `README.md`、`AGENTS.md`、各模块 `README.md` 与源码注释以相对链接引用；本目录不 import 代码、也不被代码 import（`grep -rn 'docs/' src scripts configs tests` 无运行时读取点）。
- 边界：不产生运行配置、不保存运行产物；实验过程记录不再新增到 `docs/experiments/` 与 `docs/review/`（见 [AGENTS.md 第 3 节第 8 条](../AGENTS.md)），新过程写进 `experiments/<批次>/REPORT.md` 或留在 `$PROJECT_DATA_ROOT/embodied-navigation/`。

**时效分层**（引用前先判断属于哪一层）：

| 层 | 文件 | 可信度 |
| --- | --- | --- |
| 活体计划 | `plan/ROADMAP.md`、`plan/TASKS.md` | 当前，随阶段更新 |
| 当前说明 | `dynanav-pct-scan.md`、`dynanav-waypoint-utility.md`、`dynanav-observer-udp.md`、`repo-hygiene.md` 与模块 `README.md` | 对应具体实现，引用前核对源码 |
| 历史快照 | `experiments/`、`review/`、`verification/`、`control_provenance.json` | 写报告时的结论与计数，会过期 |
| 已过时权威说明 | `architecture.md`、`glossary.md` | 2026-11-09 版本，含已退役模块路径，以 `plan/ROADMAP.md` 与模块 README 为准 |

## 当前文件夹结构

```text
docs/
├── plan/                    # 活体开发计划与 ADR：ROADMAP.md、TASKS.md、adr/（ADR-0001–0005）
├── dependencies/            # 依赖闭包、构建、许可与来源核验（9 文件），见 dependencies/README.md
├── experiments/             # 按日期保存的实验记录与机器可读回执（45 文件）
├── refactor/                # 能力盘点、SLAM 后端审计、不必要校验清查（3 文件）
├── research/                # 研究目标与用户原话边界（1 文件）
├── review/                  # 可读性/结构重构/流程审查报告与回执（22 文件，含 full-readability/ 11 文件）
├── robogauge/               # RoboGauge 部署与历史证据（7 文件），见 robogauge/README.md
├── verification/            # 独立复核与启动/容器/原生构建回执（7 文件）
├── README.md                # 本文件：文档分类索引、时效分层与链接
├── architecture.md          # 旧版系统模块与开发入口（2026-11-09，已过时，见下）
├── control_provenance.json  # 控制公式来源与等价性测试回执（7 行）
├── dynanav-exploration.md   # 2026-11-11 native 探索功能架构、节点/话题/时序（历史基线说明）
├── dynanav-observer-udp.md  # observer UDP 接收缓冲的选择、回退与逐行校验方法
├── dynanav-pct-scan.md      # 实测 PCT 全局 + 原生 SCAN 局部导航的数据流与边界
├── dynanav-waypoint-utility.md # DSH 航点候选与连续路线收益（只读建议工具）
├── glossary.md              # 术语与易混概念对照（2026-11-09，部分条目已过时）
└── repo-hygiene.md          # 仓库卫生约束草案（2026-11-15，AGENTS.md 规则依据）
```

子目录 README（`dependencies/README.md`、`robogauge/README.md`、`review/full-readability/README.md`）本身的文件数与测试计数停留在 2026-11-08/09 快照，**以本文件与 `git ls-files` 为准**。

## 权威说明与顶层文件（12 项：9 个跟踪文件 + 2 个计划文件 + ADR 目录；状态列标明是否为当前事实）

| 文件 | 核心职责 | 状态 |
| --- | --- | --- |
| [`README.md`](README.md) | 本文件：文档分类索引、时效分层与链接 | 当前 |
| [`plan/ROADMAP.md`](plan/ROADMAP.md) | 唯一主线、七层边界、Phase 1–4 里程碑、分支归属、数据卷约定 | 活体 |
| [`plan/TASKS.md`](plan/TASKS.md) | 活体任务板：已完成 / 进行中 / 待办，带角色与验收标准；Phase 4 后端评测数据集入口 | 活体 |
| [`plan/adr/`](plan/adr/) | 架构决策记录 ADR-0001–ADR-0005（yaw 延迟边界、具身 Agent 异步与语义地图、流式 Fork + 关键帧语义图与事件驱动瘦身、增量 Mesh/3DGS 与三层后端评测口径、因果关键帧记忆/Mesh+骨架底座/树图检索与 3 主线 + 4D 8 分支规划） | 活体 |
| [`dynanav-pct-scan.md`](dynanav-pct-scan.md) | 默认 `NAVIGATION_BACKEND=pct_scan`：实测支撑格网、原生 PCT A*、worker 归属的 SCAN 子进程、样条跟随、planner 收据与原生限制 | 当前（英文） |
| [`dynanav-exploration.md`](dynanav-exploration.md) | 2026-11-11 native 探索：功能分层、ROS 话题/QoS、线程与缓存、时钟规则；页首声明 `native_route` 只是历史基线，默认链已切 PCT+SCAN | 历史基线说明 |
| [`dynanav-waypoint-utility.md`](dynanav-waypoint-utility.md) | `scripts/dynanav_exploration.py candidates` 只读建议：收益代理量定义、`--prepare-next`、冻结数据重放边界 | 当前 |
| [`dynanav-observer-udp.md`](dynanav-observer-udp.md) | `--observer-transport auto` 的选择/回退、4 MiB 接收缓冲、双时钟 trace 与逐行校验方法 | 当前 |
| [`repo-hygiene.md`](repo-hygiene.md) | 仓库卫生约束草案：代码/基线配置/实验定义/运行产物四层存放、开关生命周期、由脚本强制的规则 | 规则依据（草案） |
| [`control_provenance.json`](control_provenance.json) | 控制公式的原始来源相对路径、SHA256、抽取行为、保留的安全语义与等价性测试 | 证据（历史） |
| [`architecture.md`](architecture.md) | 2026-11-09 版系统模块与开发入口、任务状态机与安全门禁 | **已过时**：描述已退役路径与旧模块名，不再作为当前事实 |
| [`glossary.md`](glossary.md) | 术语与易混概念对照（运行/定位/状态机/观测/时间分组） | **部分过时**：按当前源码符号复核后再引用 |

> 当前系统边界请读 [`plan/ROADMAP.md`](plan/ROADMAP.md) 的七层表，模块职责读各目录 `README.md`；`architecture.md`/`glossary.md` 只作历史对照，其中出现的入口与模块名可能已不在本检出中。

## 实验历史 `docs/experiments/`（45 文件）

规则：`docs/experiments/` 不再新增文件；下列每篇的结论、run id、计数都是**写报告时**的快照，引用前按对应 run 目录与当前源码复核。

### DynaNav 早期阶段（2026-11-09）

| 文件 | 主题 |
| --- | --- |
| [`dynanav-20261109.md`](experiments/dynanav-20261109.md) | 首段实验：楼梯通行、分层地图与 LIO 观察器；后续进展指向 continuation（421 行） |
| [`dynanav-20261109-continuation.md`](experiments/dynanav-20261109-continuation.md) | 完整视觉对照、闭环实跑与离线感知；不把新诊断倒填成旧运行事实 |
| [`dynanav-online-person-20261109.md`](experiments/dynanav-online-person-20261109.md) | 速度四档与在线 RGB 行人实验；感知质量未通过、用户要求暂停后的收尾 |

### RTX LiDAR 与传感管线（2026-11-10）

| 文件 | 主题 |
| --- | --- |
| [`dynanav-rtx-lidar-20261110.md`](experiments/dynanav-rtx-lidar-20261110.md) | RTX LiDAR + 原始 RGB + 物理 IMU 接入 FAST-LIVO2，25 仿真秒站立通路验证 |
| [`dynanav-rtx-lidar-motion-20261110.md`](experiments/dynanav-rtx-lidar-motion-20261110.md) | 30 仿真秒有限运动诊断：传感通路保持，但未通过运动幅度/精度验收 |
| [`dynanav-rtx-pipeline-timing-20261110.md`](experiments/dynanav-rtx-pipeline-timing-20261110.md) | RTX 缓存与阶段时序实测（source age、host queue） |
| [`dynanav-same-pair-handoff-20261110.md`](experiments/dynanav-same-pair-handoff-20261110.md) | FAST-LIVO2 同时间 pair 的阶段化交接改动与证据 |
| [`dynanav-interactive-waypoint-exploration-20261110.md`](experiments/dynanav-interactive-waypoint-exploration-20261110.md) | 交互航点探索操作手册（方法验证，715 行，含控制死区与完成带参数） |

### Native 探索与 PCT/SCAN（2026-11-11）

| 文件 | 主题 |
| --- | --- |
| [`dynanav-native-autonomous-exploration-20261111.md`](experiments/dynanav-native-autonomous-exploration-20261111.md) | 原生控制自主探索：五个模型选择航点 + stop seq + 终态停车 |
| [`dynanav-native-exploration-32-waypoints-20261111.md`](experiments/dynanav-native-exploration-32-waypoints-20261111.md) | 32 航点运行证据整理：accepted/reached、stop、停车与接触样本分开记录 |
| [`dynanav-native-waypoint-failure-recovery-20261111.md`](experiments/dynanav-native-waypoint-failure-recovery-20261111.md) | 单航点失败作为反馈：零命令、清目标、继续仿真与 SLAM |
| [`dynanav-pct-scan-live-20261111.md`](experiments/dynanav-pct-scan-live-20261111.md) | PCT + SCAN 单个约 4 米目标的实时核验（规划、连续跟随、到点、停车） |
| [`dynanav-waypoint-utility-live-20261111.md`](experiments/dynanav-waypoint-utility-live-20261111.md) | 航点候选工具的离线与实时预备核验 |
| [`dynanav-basement-dsh-exploration-20261111.md`](experiments/dynanav-basement-dsh-exploration-20261111.md) | 真实办公室地下层入口 DSH 探索试行（含 TTL 到期未达的边界） |

### 场景、地图与感知审计（2026-11-11）

| 文件 | 主题 |
| --- | --- |
| [`dynanav-rtx-scene-rendering-20261111.md`](experiments/dynanav-rtx-scene-rendering-20261111.md) | RTX/Fabric 办公室几何恢复与短站立全链验证 |
| [`dynanav-floor-collision-repair-20261111.md`](experiments/dynanav-floor-collision-repair-20261111.md) | 静态 Office 地面碰撞覆盖修复（跨过覆盖边缘后的跌倒） |
| [`dynanav-basement-floor-audit-20261111.md`](experiments/dynanav-basement-floor-audit-20261111.md) | 地下室近场传感器审计与 floor 拟合修复（CPU-only 只读云与位姿） |
| [`dynanav-operator-map-evidence-20261111.md`](experiments/dynanav-operator-map-evidence-20261111.md) | operator map 来源纠正与 unknown 证据审计 |
| [`dynanav-stair-map-projection-audit-20261111.md`](experiments/dynanav-stair-map-projection-audit-20261111.md) | 最新 operator map 的楼梯投影审计（旧 domain 运行，非当前地形验收） |
| [`dynanav-perception-consumption-audit-20261111.md`](experiments/dynanav-perception-consumption-audit-20261111.md) | 非实时试行的 perception 消费时相只读审计 |
| [`dynanav-camera-binding-runtime-20261111.md`](experiments/dynanav-camera-binding-runtime-20261111.md) | 相机绑定 v2 的运行时验证与限制（源发布/绑定失败边界） |
| [`dynanav-rtx-production-map-control-20261111.md`](experiments/dynanav-rtx-production-map-control-20261111.md) | 真实 RTX LiDAR 生产感知/局部地图接线与有限航点控制（含 2026-11-11 勘误，撤回“无回波证明开口”） |
| [`dynanav-nearfield-depth-diagnostic-20261111.md`](experiments/dynanav-nearfield-depth-diagnostic-20261111.md) | 新增仿真下视深度：只诊断、默认关闭、不接 SLAM、不授通行 |

### 异步、完整扫与 waitIdle 诊断（2026-11-11）

| 文件 | 主题 |
| --- | --- |
| [`dynanav-async-complete-scan-ab-20261111.md`](experiments/dynanav-async-complete-scan-ab-20261111.md) | 异步 complete-scan A/B 零动作试验（同修复资产与出生点） |
| [`dynanav-async-dsh-camera-stop-20261111.md`](experiments/dynanav-async-dsh-camera-stop-20261111.md) | 异步 DSH 试点：真实请求、有限运动、相机守卫停止 |
| [`dynanav-async-dsh-camera-v2-motion-20261111.md`](experiments/dynanav-async-dsh-camera-v2-motion-20261111.md) | 相机绑定 v2 后的异步 DSH 运动（跑满 180 仿真秒预算） |
| [`dynanav-complete-scan-runtime-contract-20261111.md`](experiments/dynanav-complete-scan-runtime-contract-20261111.md) | 完整扫 producer 的显式运行属性覆盖契约（含被保守拒绝的无效 A/B 样本） |
| [`dynanav-complete-scan-watermark-20261111.md`](experiments/dynanav-complete-scan-watermark-20261111.md) | RTX 完整扫描区间证据替代尾有效 ray 启发式，默认关闭 |
| [`dynanav-derived-async-readiness-attempt-20261111.md`](experiments/dynanav-derived-async-readiness-attempt-20261111.md) | derived Office 异步探索尝试（domain 207） |
| [`dynanav-offline-derived-office-runtime-20261111.md`](experiments/dynanav-offline-derived-office-runtime-20261111.md) | offline-derived Office 单次零动作运行测试 |
| [`dynanav-waitidle-single-diagnostic-20261111.md`](experiments/dynanav-waitidle-single-diagnostic-20261111.md) | 单次进程内 waitIdle 诊断（20 仿真秒、零请求） |
| [`dynanav-waitidle-async-on-crash-20261111.md`](experiments/dynanav-waitidle-async-on-crash-20261111.md) | waitIdle TRUE / async-ON 诊断的启动崩溃记录 |
| [`dynanav-rtx-model-in-loop-live-20261111.md`](experiments/dynanav-rtx-model-in-loop-live-20261111.md) | DSH 模型在环低频控制 run4 受控结果与更正 |

### 多相机视觉分支（2026-11-12）

| 文件 | 主题 |
| --- | --- |
| [`dynanav-multicamera-shadow-design-20261112.md`](experiments/dynanav-multicamera-shadow-design-20261112.md) | 方案 A 的视觉修正分支：源码审查、旁路原型与相关性对照（共享 LIO 几何、独立视觉修正） |
| [`dynanav-multicamera-prototype-20261112.md`](experiments/dynanav-multicamera-prototype-20261112.md) | 六视角观察与共享 LIO 地图原型，真实 RTX 验证 |
| [`dynanav-multicamera-native-shadow-20261112.md`](experiments/dynanav-multicamera-native-shadow-20261112.md) | 方案 A 的真实 native 快照、曝光分离与六路旁路视觉修正 |
| [`dynanav-five-camera-comparison-20261112.md`](experiments/dynanav-five-camera-comparison-20261112.md) | 五台相机实测与 3/4/5/6 台方案比较（保持约 82.4° 视场） |
| [`dynanav-four-camera-120-comparison-20261112.md`](experiments/dynanav-four-camera-120-comparison-20261112.md) | 四台 120° 相机真实采集与五台对照（可作环视候选，未判定融合收益） |
| [`dynanav-multicamera-sol-exploration-20261112.md`](experiments/dynanav-multicamera-sol-exploration-20261112.md) | 四向 SOL 探索、可回放 bag、漂移评分与原生四路更新对照（默认关闭） |

### 工具归档与脚本说明

| 文件 | 主题 |
| --- | --- |
| [`dynanav-exploration-tools-20261111.md`](experiments/dynanav-exploration-tools-20261111.md) | 实际探索工具找回与项目内索引（来源、SHA、无仿真重放方法） |
| [`dynanav-exploration-tools-20261111.manifest.json`](experiments/dynanav-exploration-tools-20261111.manifest.json) | 归档清单：来源根/线程、文件与外部证据索引 |
| [`dynanav-exploration-tools-20261111.execution.json`](experiments/dynanav-exploration-tools-20261111.execution.json) | 工具找回的执行过程与线程记录（979 行机器可读） |
| [`dynanav-exploration-tools-20261111.validation.json`](experiments/dynanav-exploration-tools-20261111.validation.json) | 归档文件 SHA、语法/JSON 解析、单测与安全 CLI 核验 |
| [`dynanav-exploration-tools-20261111.reusable-validation.json`](experiments/dynanav-exploration-tools-20261111.reusable-validation.json) | 可复验项：真实记录点/样本、新旧布局轨迹等价、hash 未变 |
| [`reconstruct_dynanav_slam_map.md`](experiments/reconstruct_dynanav_slam_map.md) | `scripts/reconstruct_dynanav_slam_map.py` 离线 CPU 适配器的输入/输出/边界说明 |

## 审查记录 `docs/review/`（22 文件）

规则：`docs/review/` 不再新增文件。报告中的行号、文件数、测试数是当时快照；结构重构后可能过期。

| 文件 | 核心内容 |
| --- | --- |
| [`dsh-workflow-and-readability-20261109.md`](review/dsh-workflow-and-readability-20261109.md) | 工程流程导读与可读性整理（802 行，含全仓阅读/测试/配置说明） |
| [`dsh-workflow-fixes-20261109.md`](review/dsh-workflow-fixes-20261109.md) | 在导读报告之后实际实施的修复/重构、未处理项与原因 |
| [`dsh-workflow-independent-review-20261109.md`](review/dsh-workflow-independent-review-20261109.md) | 对上述修复的独立核查与退回修正记录 |
| [`dsh-structural-refactor-20261109.md`](review/dsh-structural-refactor-20261109.md) | 大型 controller 与 DynaNav 采集入口的结构重构（拆 mixin/拆 `capture/`、保留兼容入口） |
| [`dsh-structural-independent-review-20261109.md`](review/dsh-structural-independent-review-20261109.md) | 上述结构重构的独立核查（对照重构前工作树） |
| [`module-structure-coupling-20261109.md`](review/module-structure-coupling-20261109.md) | 62 份模块 README 统一结构后的跨模块耦合总评（历史索引） |
| [`dynanav-documentation-20261111.md`](review/dynanav-documentation-20261111.md) | DynaNav 文档与可复用入口整理（只改说明/索引与 skill 归档） |
| [`runtime-repair-validation-20261109.md`](review/runtime-repair-validation-20261109.md) | capture 收据/生命周期、observer 就绪与 supervisor 回收的修复与真实重验（两条路线均未验收通过） |
| [`validation-and-threshold-audit.md`](review/validation-and-threshold-audit.md) | 校验与阈值审计：DSH 初稿 + Codex 独立事实复核，逐条分类 |
| [`readability-verification.md`](review/readability-verification.md) | 模块文档与代码可读性两阶段复核的说明 |
| [`readability-verification.json`](review/readability-verification.json) | 上述复核的机器可读记录（基础提交、执行、源审查、测试快照） |
| [`full-readability/README.md`](review/full-readability/README.md) | 全量代码/配置可读性报告总入口：589 个目标、三套 ownership（project/vendor/addendum）的口径说明 |
| [`full-readability/project/coverage.json`](review/full-readability/project/coverage.json) | project 310 个目标的逐文件审阅记录（5145 行） |
| [`full-readability/project/findings.md`](review/full-readability/project/findings.md) | project 发现与阈值/校验审计 |
| [`full-readability/project/summary.md`](review/full-readability/project/summary.md) | project 复核摘要与统计口径 |
| [`full-readability/vendor/coverage.json`](review/full-readability/vendor/coverage.json) | vendor 265 个目标的逐文件审阅记录（12223 行） |
| [`full-readability/vendor/findings.md`](review/full-readability/vendor/findings.md) | vendor 可读性发现清单 |
| [`full-readability/vendor/summary.md`](review/full-readability/vendor/summary.md) | vendor 改造总结 |
| [`full-readability/addendum/coverage.json`](review/full-readability/addendum/coverage.json) | addendum 14 个补充目标的审阅记录 |
| [`full-readability/addendum/findings.md`](review/full-readability/addendum/findings.md) | addendum 补充发现 |
| [`full-readability/addendum/summary.md`](review/full-readability/addendum/summary.md) | addendum 摘要 |
| [`full-readability/verification.json`](review/full-readability/verification.json) | 全量可读性的最终独立验证记录（6142 行） |

## 验证回执 `docs/verification/`（7 文件）

| 文件 | 证据内容 |
| --- | --- |
| [`independent-review-2026-11-09.md`](verification/independent-review-2026-11-09.md) | 2026-11-09 独立复核与启动入口修复的说明 |
| [`independent-review-2026-11-09.json`](verification/independent-review-2026-11-09.json) | 同轮复核对 DSH 报告的更正、哈希核对、修复与测试结果 |
| [`cpu-container.json`](verification/cpu-container.json) | CPU 容器基线镜像、单测数/失败/跳过、是否联网与是否启动仿真 |
| [`native-docker.json`](verification/native-docker.json) | 公开基础镜像上的原生 ROS 包与自有模块构建、pin 校验、单测/回归 |
| [`software-startup.json`](verification/software-startup.json) | 容器内 15 个受管组件的启动结果与源码 SHA 校验 |
| [`clean-export.json`](verification/clean-export.json) | 干净导出的候选文件、符号链接、检查项与“未复制外部定制代码/数据”的结论 |
| [`parking-layering.json`](verification/parking-layering.json) | 停车/分层相关职责归属与阈值未改动的分项确认 |

## 依赖与部署 `docs/dependencies/`（9 文件）

| 文件 | 内容 |
| --- | --- |
| [`dependencies/README.md`](dependencies/README.md) | 依赖闭包索引：上游仓库/commit、已提供组件、依赖关系与验证边界 |
| [`BUILD.md`](dependencies/BUILD.md) | 原生依赖的构建与运行边界（Docker 原生层，不等于完整应用） |
| [`PYTHON-AND-DEPLOYMENT.md`](dependencies/PYTHON-AND-DEPLOYMENT.md) | 迁移前自有 Python 模块清单，用于核对依赖闭包 |
| [`LICENSE-STATUS.md`](dependencies/LICENSE-STATUS.md) | 逐组件许可、署名与来源边界表（不代替法律审查） |
| [`MODEL-NOTICE.md`](dependencies/MODEL-NOTICE.md) | Student 权重来源、发布依据与许可状态 |
| [`vendored-source-manifest.json`](dependencies/vendored-source-manifest.json) | 逐文件复制清单与 SHA256（3969 行机器可读；哈希只用于完整性/来源追溯） |
| [`verification.json`](dependencies/verification.json) | 2026-11-08 构建/来源核验快照（`not run` 项不等于通过） |
| [`native-build-plan.json`](dependencies/native-build-plan.json) | 原生构建计划：源根、包、pin、补丁与构建/安装路径 |
| [`runtime-extra-source-closure.json`](dependencies/runtime-extra-source-closure.json) | 隔离启动发现的额外运行时来源闭包 |

## RoboGauge `docs/robogauge/`（7 文件）

| 文件 | 内容 |
| --- | --- |
| [`robogauge/README.md`](robogauge/README.md) | MoE-CTS 可移植部署说明：功能归属、权重与数据、历史结果与边界、许可与安全门禁 |
| [`source_manifest.json`](robogauge/source_manifest.json) | 迁移源文件哈希与来源映射（含“训练提交未核实”的记录） |
| [`PORT_VALIDATION.json`](robogauge/PORT_VALIDATION.json) | 迁移验证：可移植源文件、源归档哈希复核、CPU 回归结果 |
| [`cpu_validation.json`](robogauge/cpu_validation.json) | CPU 策略 fixture 实测（模型 SHA、torch 版本、动作误差、延迟） |
| [`historical_execution_recheck.json`](robogauge/historical_execution_recheck.json) | 历史 run 的同刻命令/力矩复算 |
| [`isaac_closure_audit.json`](robogauge/isaac_closure_audit.json) | Isaac 源闭包审计：已移植纯组件与未移植 live runner |
| [`UPSTREAM_GO2_RL_LAB_LICENSE.txt`](robogauge/UPSTREAM_GO2_RL_LAB_LICENSE.txt) | 模型档案随附的 MIT 许可与下游声明（原样保留，不等于再分发授权） |

> `robogauge/README.md` 引用的部分历史测试文件已随干净底座清理，其“CPU fixtures in tests/regression/…”段落不再对应当前测试清单；当前回归清单见 [tests/README.md](../tests/README.md)。

## 研究 `docs/research/`（1 文件）

| 文件 | 内容 |
| --- | --- |
| [`dynamic-semantic-mapping-agreement.md`](research/dynamic-semantic-mapping-agreement.md) | 动静态语义地图与环境编译器的历史原话、研究目标与实现边界（研究输入，不是已完成能力） |

## 重构分析 `docs/refactor/`（3 文件）

| 文件 | 内容 | 时效 |
| --- | --- | --- |
| [`capability-map.md`](refactor/capability-map.md) | 能力地图与精简计划：七层能力清单、按层行数统计、先删后写分阶段计划（干净底座 P1/P2 清理的依据） | 2026-11-15 盘点，路径/行数为当时快照 |
| [`slam-backend-audit.md`](refactor/slam-backend-audit.md) | SLAM 后端现状与先验地图重定位：D 路径无后端的结论、B/C 后端能力、已确认错误与可选项 | 2026-11-15 盘点 |
| [`unneeded-validations-20261115.md`](refactor/unneeded-validations-20261115.md) | 不必要校验清查：判定分类、复核结果、删除包与已知 bug | 2026-11-15 盘点 |

## 验证与维护入口

- 文档不参与测试；**文档改动的最低自检**：`git status --short -- docs tests` 只出现 Markdown、`git diff --check` 无空白错误、本文件所有相对链接能打开。
- 测试与回归的实际入口见 [tests/README.md](../tests/README.md)（`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit` 与 `... tests/regression`）。
- 源码模块职责见 [src/embodied_navigation/README.md](../src/embodied_navigation/README.md)；配置入口见 [configs/README.md](../configs/README.md)；脚本入口见 [scripts/README.md](../scripts/README.md)。
- 引用规则：活体计划与当前说明可以直接引；`experiments/`、`review/`、`verification/`、`dependencies/*.json`、`robogauge/*.json` 是**快照**，行号/计数/哈希会过期，引用前核对磁盘与对应 run 目录。
- 数据与产物：运行日志、bag、点云、视频、逐帧 JSON 与审计台账都在 `$PROJECT_DATA_ROOT/embodied-navigation/`，不进仓库；文档只写占位符路径与结论。
- 维护约定：`docs/experiments/`、`docs/review/` 不再新增文件；新的过程记录写 `experiments/<批次>/REPORT.md`。文档体系或入口变化时同步本文件与 [`plan/ROADMAP.md`](plan/ROADMAP.md)、[`plan/TASKS.md`](plan/TASKS.md)；新增/修订架构决策时同时维护 [`plan/adr/`](plan/adr/)。
