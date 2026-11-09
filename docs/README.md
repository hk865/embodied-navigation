# 工程文档索引

最新修复与真实运行边界见 [运行故障修复与重验](review/runtime-repair-validation-20261109.md)：故障生命周期/收据通过独立验证，不代表在线行人、R85 路线或 5 秒停车已验收。
本目录是工程文档与证据的入口：`architecture.md` 与 `glossary.md` 是权威说明，`experiments/`、`review/`、`verification/` 是历史记录与回执，`dependencies/`、`robogauge/` 是依赖与部署专题。文档不参与运行时，也没有代码读取本目录。

## 目录结构

```text
docs/
├── dependencies/          # 依赖闭包、构建、许可与核验证据（11 文件），见 dependencies/README.md
├── experiments/           # DynaNav 实验记录 4 篇（含续篇与在线行人实验）
├── research/              # 研究目标、用户原话与实现边界（1 篇）
├── review/                # 可读性/结构重构/流程审查报告与回执（20 文件，含本轮新增的模块结构总评）
├── robogauge/             # RoboGauge 部署说明与历史证据（11 文件），见 robogauge/README.md
├── verification/          # 独立复核与启动/容器/原生构建回执（8 文件）
├── README.md              # 本文件：文档索引与职责分类
├── architecture.md        # 权威：系统边界、四条执行路径、默认状态机、安全门禁（51 行）
├── control_provenance.json # 控制公式来源与 1000 例等价性测试记录（7 行）
└── glossary.md            # 权威：术语与易混概念对照（43 行）
```

## 代码、日志与配置

本目录按“文档职责”理解三类内容（该口径对应其它模块的代码/日志/配置三问）：

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 权威说明 | [`architecture.md`](architecture.md)、[`glossary.md`](glossary.md) | 当前系统边界、四条执行路径、默认任务状态机、安全门禁与术语。代码/配置文档引用它们，而不是反过来；两者都声明“行号会漂移，按文件与符号查找”。 |
| 历史报告 | [`experiments/`](experiments/)、[`review/`](review/) | 实验过程与审查结论：可读性改造、结构重构、独立复核、DynaNav 各阶段实验。报告里的行号/计数是当时快照，引用前需按当前磁盘复核。 |
| 证据回执 | [`control_provenance.json`](control_provenance.json)、[`verification/`](verification/)、[`review/readability-verification.json`](review/readability-verification.json) | 机器可读的历史回执：控制公式来源与 1000 例等价性、CPU/容器/原生构建、独立复核、可读性验证。它们是“当时验证过什么”的证据，不是当前状态。 |
| 配置说明（非配置源） | 全目录 | 文档引用 [configs/](../configs/README.md) 的读取顺序与 pin，但本目录**没有配置读取点**，也不写配置；`grep -rn "docs/" src scripts` 只命中一处脚本注释（`scripts/run_dynanav_online_person.py` 指向实验文档），没有代码加载 `docs/`。 |
| 日志/输出 | 无 | 本目录无运行日志、无写入操作；文中出现的 run 目录、截图、bag 等证据都在外部项目数据卷，文档只记录路径与结论。 |

## 模块内文件耦合

- 依赖方向：`README.md → 其余 9 个直接子项` 是唯一索引关系；子目录内部再各自索引（`dependencies/README.md`、`robogauge/README.md`、`review/full-readability/README.md`）。**无代码 import、无环**。
- 文档之间的引用是单向的：`architecture.md`/`glossary.md` 被实验报告与 review 引用；`review/` 里的报告按时间先后引用前一轮结果；`verification/` 回执被 review/README 引用。没有“报告 A 修改权威说明”的情况。
- 隐式契约：README 的链接必须真实存在（本文件所有相对链接由自检脚本校验）；`control_provenance.json` 的 `original_source_sha256`/`test` 字段是来源追溯，不是可执行配置；`review/` 报告中的行号会随结构重构漂移（例如 `pid_navigation.py` 已拆成 `pid_*.py`、`capture_go2_rgbd.py` 已拆出 `capture/`），引用时须以当前文件为准。

| 文件/目录 | 规模 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `README.md` | 本文件 | 其余 9 个直接子项 | 人；根 README 引用 | 清楚：索引职责单一 |
| `architecture.md` | 51 行 | 源码/launch/configs 事实 | 实验报告、模块 README、根 README | 清楚：权威且边界明确 |
| `glossary.md` | 43 行 | architecture 与源码符号 | 实验报告、模块 README | 清楚：按主题分组 |
| `control_provenance.json` | 7 行 | 原始源码快照 | 控制相关文档/回执 | 清楚：字段少、证据明确 |
| `dependencies/` | 11 文件 | 上游源码/构建 | 本 README、源码 README 链接 | 清楚，见其 README |
| `robogauge/` | 11 文件 | 部署代码/历史 run | 本 README、源码 README 链接 | 偏重：一份 README 承载部署+历史，见其 README |
| `experiments/` | 4 篇（82–421 行） | 外部 run 证据 | 实验报告之间、configs README | 可接受：长文历史记录 |
| `review/` | 20 文件（含 802 行可读性报告与本轮新增的模块结构总评） | 前几轮工作树 | 后续修复报告 | 可接受：历史层级多，需按日期读 |
| `verification/` | 8 文件 | 各轮检查 | review/README | 清楚：回执粒度单一 |

结论：耦合**清楚且合适**：索引单向、权威说明与历史记录分离；没有跨层直达或循环引用。唯一维护风险是“历史报告里的行号/计数会过期”，这是时间性风险而非结构缺陷。

## 可读性评估

- `README.md`（原 20 行）：**可改进（索引完整性）**。原文件只是 18 条链接列表，没有区分“权威说明 / 历史报告 / 证据回执”，新人无法判断哪份是当前事实；且遗漏了 `docs/review/validation-and-threshold-audit.md` 与 `docs/verification/` 下除独立复核外的回执（本次重写已补分类并在下方索引补齐链接）。优点是所有链接原本就能打开，没有死链。
- `architecture.md`：**良好**。51 行覆盖代码/数据边界、四条执行路径、状态机与安全门禁，且明确“不要从 `navigation/task.py` 或 `mission/state_machine.py` 推断默认行为”这类易错点；信息密度高。
- `glossary.md`：**良好**。43 行、按运行/定位/状态机/观测/时间分组，每条给出“别把它当成什么”的对照；明确“数值与契约以源码/配置为准”。
- `control_provenance.json`：**可接受**。7 行、4 个字段，能看懂来源与测试，但字段没有 schema/单位说明，需结合引用它的文档读。
- `review/`：**可接受偏低**。报告真实但长（可读性报告 802 行），且混有“当时行号”；作为历史记录可以接受，作为现状依据不可靠。没有发现空话或未标注的推测。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 保持本文件的“权威 / 历史 / 证据”三分类，并在每份历史报告开头加“快照日期 + 可能过期的行号/计数”声明 | 当前 20 行索引不区分时效，读者容易把历史数字当现状 | 无（纯文档；历史报告正文不改） | 抽查 3 份报告是否都有快照日期；自检脚本链接检查通过 |
| P2 | 补齐原索引遗漏的链接（`review/validation-and-threshold-audit.md`、`verification/` 下 CPU/容器/原生/clean-export/parking/production 回执） | 索引不完整会让证据文件“隐形” | 无 | 逐个点开链接；`check_readmes.py` 校验相对链接 |
| P3 | 给 `experiments/` 与 `review/` 的 4 + 19 个文件在各自目录加一个 `README.md` 索引（当前只有 `docs/README.md` 汇总） | 子目录没有本地索引，新增文件只能靠根索引维护 | 低：新增文件，不改原报告 | 目录列表与索引逐项对照 |
| P3 | 统一 `review/` 报告命名（当前既有 `*-review-20261109.md`，也有 `validation-and-threshold-audit.md`） | 按日期/类型排序困难 | 无（仅建议，不重命名历史文件） | 人工浏览目录 |

没有 P0/P1：本目录不参与运行，问题限于“读者可能误把历史当现状”；结构上不需要拆分或合并，已有权威/历史/证据分离的基础。

## 文档索引（原有链接保留）

- [系统模块与开发入口](architecture.md)：源码职责、数据边界、四条执行路径、默认任务状态机、运行与测试入口。
- [术语与易混概念对照](glossary.md)：四条路径、provenance、raw/corrected 位姿、native64/247D/exact45、RoboGauge 45D、时钟域与分层的速度上限。
- [动静态语义地图与环境编译器：历史原话、研究目标及实现边界](research/dynamic-semantic-mapping-agreement.md)：已核实用户原话、消息出处、未定参数与 DynaNav 工作基线。
- [DynaNav 楼梯、分层地图与 LIO 观察器实验](experiments/dynanav-20261109.md)：2026-11-09 首段 live LIO/移动中行人闭环、完整 GT 路线、SDK 200 Hz 对照、多层精度边界及 LIVO smoke 更新保留、单组VIO评分、录包/两次SDK阻碍及前缀对比边界。
- [DynaNav 后续实验](experiments/dynanav-20261109-continuation.md)：traceback 实测定位、无定时 dump 的完整旁路对照、live post-VIO、RViz 与离线 learned 感知；三目标视觉闭环完成与独立停车审计、learned对象历史/阈值局限和真实地图/视频交付。
- [DynaNav速度四档与在线行人实验](experiments/dynanav-online-person-20261109.md)：完整post-VIO速度阶梯、持久RGB worker/tile反例、重复几何ledger及45s真实在线片段的漏过/历史污染、完整路线当前状态。
- [DynaNav配置](../configs/simulation/dynanav/README.md)与[native LIO fixtures](../tests/native/dynanav_lio/README.md)：模式/标定索引和无模拟器的C++构建测试。
- [依赖与构建](dependencies/README.md)：第三方源码、构建、模型和许可证边界。
- [RoboGauge 部署与历史证据](robogauge/README.md)：策略契约、CPU 验证、Gazebo 入口与 Isaac 历史实验限制。
- [全量可读性审查](review/full-readability/README.md)：代码与配置可读性审查记录。
- [2026-11-09 独立复核](verification/independent-review-2026-11-09.md)：本轮开始前的修复与验证结果。
- [2026-11-09 工程流程修复记录](review/dsh-workflow-fixes-20261109.md)：按可读性报告实际实施的修复/重构、未处理项及原因、已执行的离线检查。
- [2026-11-09 现役 controller 与 DynaNav 采集入口结构重构](review/dsh-structural-refactor-20261109.md)：`pid_navigation.py` 按职责拆为 mixins/共享支持/入口模块、`capture_go2_rgbd.py` 改为显式 `run/main` 并拆出 `capture/` 包；行为保持证据、兼容入口与剩余限制。
- [模块结构与耦合总评](review/module-structure-coupling-20261109.md)：62 个 README 统一结构后的跨模块结论索引（新增报告，补充索引）。
- [DSH 流程修复独立核查](review/dsh-workflow-independent-review-20261109.md)：修复前工作树基线、独立发现与退回修正、最终 CPU 检查及剩余边界。
- [原有代码结构重构：独立核查](review/dsh-structural-independent-review-20261109.md)：导航与采集大文件的职责拆分、兼容/接线修正、最终行为对照与验证边界。
- [测试入口](../tests/README.md)与[配置入口](../configs/README.md)：测试命令/边界与配置读取顺序、pin 与无读取点数据。
- [验证阈值与判据审计](review/validation-and-threshold-audit.md)：阈值/判据清单与审计结论（补充索引）。
- [可读性验证回执](review/readability-verification.json)：可读性改造的机器可读验证记录（补充索引）。
- [独立复核回执](verification/independent-review-2026-11-09.json)：独立复核的 JSON 版本（补充索引）。
- [CPU 容器回执](verification/cpu-container.json)、[原生 Docker 回执](verification/native-docker.json)、[软件启动回执](verification/software-startup.json)、[clean-export 回执](verification/clean-export.json)、[parking-layering 回执](verification/parking-layering.json)、[production-slam-only 回执](verification/production-slam-only.json)：启动/容器/分层验证的机器可读证据（补充索引）。

模块级职责与入口见[源码模块索引](../src/embodied_navigation/README.md)。大型场景、地图、日志、模型和视频保存于外部项目数据目录；工程文档引用其证据，不将这些数据纳入 Git。

## 边界

- 文档里的行号、测试数量、run 标识都是**写报告时**的快照；结构重构后可能过期，引用前按当前磁盘复核。
- 本目录不产生运行时配置，也不被运行时读取；不要把文档中的示例 JSON/命令当成生效配置。
- 外部数据卷（`runs/`、`experiments/`、地图、视频、权重）不入 Git；文档中的绝对路径是记录，不是仓库内文件。
