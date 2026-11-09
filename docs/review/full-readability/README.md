# 全量代码/配置可读性报告总入口

本目录是当前仓库 **589 个代码/配置目标**的可读性改造成果与事实复核总入口。
三套报告分别覆盖不同 ownership，路径集合互不重叠，合计严格等于 589：

| 报告 | ownership | 目标数 | 入口 |
| --- | --- | ---: | --- |
| project | 自有源码、脚本、构建、配置 | 310 | [coverage.json](project/coverage.json) · [findings.md](project/findings.md) · [summary.md](project/summary.md) |
| vendor | `third_party/` 下引入的开源算法与配置 | 265 | [coverage.json](vendor/coverage.json) · [findings.md](vendor/findings.md) · [summary.md](vendor/summary.md) |
| addendum | 上两任务未覆盖的 launch/IDL/原始 patch | 14 | [coverage.json](addendum/coverage.json) · [findings.md](addendum/findings.md) · [summary.md](addendum/summary.md) |
| **合计** | | **589** | |

## 目录结构

```text
docs/review/full-readability/
├── addendum/          # 14 个补充目标的 coverage/findings/summary（3 文件）
├── project/           # 自有源码/脚本/构建/配置 310 个目标的报告（3 文件）
├── vendor/            # third_party 265 个目标的报告（3 文件）
├── README.md          # 本文件：总入口、统计口径与验证边界
└── verification.json  # 最终独立验证的机器可读记录（6142 行）
```

## 代码、日志与配置

本目录的对象是**报告与证据**，不是代码；三问按“报告 / 证据 / 配置”口径解释：

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 报告 | `project/`、`vendor/`、`addendum/` 下的 `findings.md`、`summary.md` | 逐文件/分组的发现、证据与建议（`findings.md`）；范围、方法、统计、验证边界与未完成事项（`summary.md`）。三套路径集合互不重叠。 |
| 证据（逐文件） | `project/coverage.json`、`vendor/coverage.json`、`addendum/coverage.json` | 逐目标记录 `path / baseline_sha256 / final_sha256 / kind / action / readability_work / read_evidence / checks_and_thresholds / validation`。三份合计 673 条 = 589 个目标 + 84 条 project 只读 reference。 |
| 证据（最终验证） | [`verification.json`](verification.json) | 最终验证快照：scope 计数、Python 归一化 AST（276/277）、C/C++ 194 份 token 复核、native manifest 304 条（刷新 175 条）、测试命令/结果、配置/IDL/CMake 专项、`limits` 与 `code_remarks`。行数 6142，是机器可读长记录。 |
| 配置 | `coverage.json` 中的配置/pin 目标 | 本目录审阅的是**仓库里的配置文件**（YAML/JSON/XML/SDF/launch），不是本目录自己的配置；`full-readability/` 下没有配置读取点，也没有代码加载这些 JSON。 |
| 代码 | 无 | 本目录没有可执行代码。README 提到的 `check_dsh_coverage.py` **不在仓库主树**（本轮检索只在 `.dsh-subagents/.../scratch/checks/` 找到任务期副本），复现覆盖校验需自备该脚本；`addendum` 的 `validation_script`/`validation_evidence` 同样指向任务 scratch。 |
| 日志/输出 | 无 | 不运行、不写日志；所有数字是报告生成/验证时的快照。`coverage.json` 与 `verification.json` 是证据，不要为了让数字“看起来一致”而改它们（SPEC 明确：不得改 `project/vendor/addendum/coverage.json`）。 |

## 模块内文件耦合

- 依赖方向：`README.md → 3×coverage/findings/summary + verification.json` 是唯一索引关系；三套报告按 ownership 并列，互不引用；`verification.json` 汇总三者的最终结果，但不改写各自 `coverage.json`。
- 路径集合关系（已用脚本核对）：三个 `coverage.json` 的目标路径互不重叠，合计 589 = 310 + 265 + 14；`project` 另有 84 条 `kind=reference` 记录，vendor 的 `reference_read_only.paths` 有 41 条，两者与 addendum 的 14 个目标有 14 条重叠——所以**关联参考 111 = 84 + 41 − 14**，且 700 个 tracked baseline 文件 = 589 目标 + 111 参考。这是本目录最容易被误算的隐式口径。
- 隐式契约（数字必须跨文件一致，但无自动校验在仓库内）：
  - `589` / `310+265+14` / `402 changed = 228 project + 164 vendor + 10 addendum` / `304 manifest records, 175 updated` / `276/277 AST` 等数字同时出现在三份 `coverage.json` 与本 README，改一处就要全改。
  - `action` 有两套读法：按 **589 目标** 统计是 `already_clear 107 / changed 402 / empty 33 / locked_exact 47`；按**全部 673 条记录**（含 84 条 reference）统计是 `locked_exact 131`。README 原表的“逐项完整审阅”描述针对目标集合，不要拿 131 当目标数。
  - `verification.json` 的 scope 口径是 `504 code/interface/script/build/patch + 85 configuration = 589`；三份 `coverage.json` 按 `kind` 合计是 `494 code + 89 configuration + 4 source_patch + 2 interface_definition = 589`。两套口径差 4，现有文档未说明差异来源，属**未核实**；本 README 保留 verification 的 504/85 作为最终口径。
- 与仓库其余部分的耦合：这些 JSON 是只读证据，不被 `src/`/`tests/`/`scripts/` 加载；`check_dsh_coverage.py` 的校验逻辑在任务 scratch 中，仓库内没有对应测试。

| 文件/目录 | 规模 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `README.md` | 本文件 | 3 套报告 + verification.json | 人；docs/README、根 README 链接 | 偏重：总入口同时承担口径解释、验证结论与边界 |
| `project/` | 3 文件（coverage 5145 行） | 自有文件 | 本 README | 清楚：310 目标 + 84 参考 |
| `vendor/` | 3 文件（coverage 12223 行） | `third_party/` | 本 README | 清楚：265 目标 |
| `addendum/` | 3 文件（coverage 1524 行） | launch/IDL/patch | 本 README | 清楚：14 目标，第二轮串行恢复 |
| `verification.json` | 6142 行 | 三套报告/测试日志 | 本 README | 偏重：机器可读，人工只读 scope/tests/limits 几节 |

结论：耦合**清楚但口径隐式**——路径集合干净、ownership 不重叠，但“589 目标 / 673 记录 / 111 参考 / 131 vs 47 locked”这些计数关系只靠文字维护，没有仓库内自动校验；这是本目录的主要维护风险。

## 可读性评估

- `README.md`（原 72 行）：**可接受偏高**。优点：把“已独立确认”与“仍需数据”分开，明确列出被否定的初稿说法（不把旧说法当事实），并声明“不作通过声明”；统计与验证结果都有具体数字。缺点：**口径层次混在一起**（589 目标、673 记录、111 参考、504/85、402 changed 同时出现却未解释关系），“111 关联参考”没有给出 84+41−14 的来源，`check_dsh_coverage.py` 未说明不在仓库内；另外“CPU 单元测试 54 通过 / 源码回归 57 通过（合计 62）”是**该任务收尾快照**，与当前 [tests/README.md](../../../tests/README.md) 的 `323/86` 不是同一时点，原表未标注，容易被当成当前测试数。
- 三份 `coverage.json`：**可接受（机器可读）**。字段完整、逐文件可追溯；人工审阅 673 条不现实，需要 `findings.md`/`summary.md` 辅助。
- `findings.md`/`summary.md`：**可接受**。`project` 95/88 行较短、可通读；`vendor` 469/120 行、`addendum` 259/94 行较长，按分组读。
- `verification.json`：**需改进（人工可读性）**。6142 行、机器格式，README 未给出“该读哪几节”（`scope`、`tests_and_native_checks`、`limits` 最有价值），新人打开成本高。

没有发现空话：边界、未验证项与撤回的初稿说法都写得很具体；这是本目录最强的部分。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 README 增加“统计口径”小表：589 目标 = 310+265+14；673 记录 = 589+84；111 参考 = 84+41−14；目标 `locked 47` vs 记录 `locked 131`；504/85 与 494/89/4/2 并存的说明 | 现在数字互相关联但无一处解释，最易被误引 | 无（纯文档；数字须从 coverage/verification 重新统计） | 用 `python3` 读三份 coverage 重算 310/265/14、402、131/47、111，与表逐项对照 |
| P2 | 给“最终独立验证”表加一行快照说明：`checked_at_utc=2026-11-09T01:07:15Z`、baseline `5e7982e`；当前测试数见 `tests/README.md`（unit 323 / regression 86），两者不是同一时点 | 54/62 与当前 321/86 混读会造成“测试退化”误判 | 无 | 对照 `verification.json` 的 `checked_at_utc` 与 tests/README 的记录日期 |
| P2 | 说明 `check_dsh_coverage.py` 不在仓库主树（仅任务 scratch），并给出可复现的最小校验步骤（路径集合、无重复/缺失/pending、SHA 与磁盘一致） | 原 README 让读者以为仓库内有现成校验脚本 | 无（如未来入库，需单独测试） | 在 `git ls-files` 输出里过滤 `check_dsh_coverage` 为空；按 README 步骤用自备脚本抽查 5 条 |
| P3 | 在 README 指出 `verification.json` 最该读的 3 节（`scope`/`tests_and_native_checks`/`limits`），并说明 `files` 等长数组只用于机器复核 | 6142 行人工不可读 | 无 | 人工按指引入口试读一遍 |

没有 P0/P1：本目录是历史审查证据，不参与运行；错误只影响阅读与引用，且已有“不作通过声明”“仍需数据”的分层，不需要结构调整。**不要修改 `project/vendor/addendum/coverage.json`**；任何口径说明只写在 README 中。

## 589 的构成

- **504 = 代码 / 接口 / 脚本 / 构建 / patch**：Python、C/C++、CMake、Dockerfile、`.msg`、`.idl`、shell 与原始 patch 等实现层文件。
- **85 = 配置与 pin**：YAML/JSON/XML/SDF 模板（含 ROS `.launch`）、路由、契约与依赖 pin 等运行/构建配置。
- 另有 **111 个关联参考**：README、LICENSE、历史回执、来源 manifest 等，不计入 589。原分工按只读交叉核对；最终同步了当前文档和来源校验元数据，保留原始身份及历史回执。

覆盖清单与 ownership 来自 scope 定义；`check_dsh_coverage.py` 会逐条校验 589 个目标路径集合、无重复/缺失/pending，以及 `baseline_sha256`/`final_sha256` 与当前磁盘一致。该脚本不在本仓库主树（见“可读性评估”），复现需自备。

## 怎么读这三套报告

- `coverage.json`：逐文件记录 `path / baseline_sha256 / final_sha256 / kind / action / readability_work / read_evidence / checks_and_thresholds / validation`。`action` 的 `changed / already_clear / empty / locked_exact` 都表示**已逐项完整审阅**：
  - `changed`：有注释、排版或说明层面的改动；
  - `already_clear`：原文件已足够清晰，formatter 无可读性收益，保持字节不变；
  - `empty`：0 字节包标记文件；
  - `locked_exact`：宏/行号/补丁身份敏感，或原始 patch 与历史回执，**刻意保持字节不变**。
- **并非每个文件都必须有 diff**。已清晰、空文件、行号/宏敏感文件以及原始 patch 可以不改，但都已逐项审阅并记录了保留理由；改动本身也不等于行为变更。
- `findings.md`：逐文件/分组的发现、证据与建议，区分“仅记录、未改行为”和“已修正说明”。
- `summary.md`：范围、方法、统计、验证边界与未完成事项。

## 独立事实复核与串行合入

本任务是 **DSH 全量实现，Codex 独立事实复核，DSH 串行合入**：

- DSH 三个会话完成 589/589 的逐文件阅读、格式化/注释与等价性检查；addendum 第二轮串行恢复了被并发回退的 10 项。
- Codex 对 DSH 新增说明做独立事实复核，产出 6 份 `source_fact_reviews`（control/other/planning-robot/fastlivo/vendor/scan-final8）与 109 个已核实候选；另复核第三方算法报告的 5 项算法发现 + Huber 共 11 条关联记录（Huber 误报已撤回），其余约 186 条明确保留“未逐项独立确认”的限定。
- DSH 串行合入：109/109 候选在写入前再次核对 `current==preimage` 与候选 SHA，全部按字节应用、保持原 mode；Python 归一化 AST 不变，仅注释/docstring 文案变化；算法、阈值、数值、接口、调用顺序与配置含义未改。VIO 采用“事实更正 + 原 floor 补丁 4 处共 30 行上下文兼容”的最终单一候选，原 patch 与 `docker/verify_native_sources.py` 逻辑保持，reverse dry-run 与正逆 roundtrip 通过。

已被独立复核否定、并已从报告与注释中更正的**初稿**说法（不是当前事实）包括：0.8 被写成最小 0.8 m 扫掠、四元数按模长而非平方范数描述、gravity-free 静止初始化、IMU 时间 offset 方向、均匀 B 样条写成 clamped、`planning_horizon_` 单位写成秒、队列写成阻塞式背压、把分母为 count 的总体标准差误称样本标准差、runtime 写成 ONNX 选择等；逐条见各 `coverage.json` 的 `independent_fact_review`。

## 已独立确认 vs 仍需数据

**已独立确认（本次 Codex 复核 + 源码定位）**

- 109 个事实/上下文候选涉及的说明与调用范围限定，已同步到最终源码与三套报告；写入后 Python 归一化 AST 不变、文件 mode 不变。
- 第三方算法报告中：旧 Raycast 距离量纲/调用路径、img_align 层数与零有效像素、A* 默认分数状态路径、RingBuffer `empty()/size()/getSum()`、`GradientDescentOptimizer` 初始化、Huber 误报撤回。这些均限定为“代码事实 + 影响范围”，不等于当前导航必然触发。
- addendum 14 条（8 launch + 2 IDL + 4 原始 patch）的最终 SHA/字节与磁盘一致；4 个 patch 保持字节锁定，未改其身份。

**仍需运行数据/单独验证（本报告不作通过声明）**

- 是否在真实导航/实机路径触发上述潜在缺陷（A* 重访、零有效像素、未初始化 API、RingBuffer 调用方等），需要针对性运行数据与契约确认。
- 完整 ROS/Gazebo/Isaac 软件栈构建、训练及实机仍未验证；PCT 局部 CPU 编译与合成路径测试已通过。C/C++ 未做全量宏展开与全部目标构建，不据 token 相同宣称完整运行等价。
- 活跃来源清单已按当前文件刷新 175 项，304/304 指纹、大小和修改状态一致；原 `source_sha256`、上游提交、模型、档案及外部资产身份保持。

## 最终独立验证

基线为 `5e7982ea3c82f2ae263b46e156a90cafca86780c`。589 项全部覆盖，实际修改 402 项；其余文件已逐项说明保留依据。完整逐文件 SHA、检查方法、测试命令与限制见[机器可读验证记录](verification.json)。

| 检查 | 实际结果 |
| --- | --- |
| CPU 单元测试 | 54 通过 |
| 源码回归 | 57 通过，5 项因缺少可选 CPU Torch 跳过；合计 62 项 |
| Python 规范化 AST | 276/277 相同；唯一例外为已说明的源码拼写断言改为 AST 结构断言 |
| C/C++ | 194 份算法 token、字符串与字面量保持；5 项预处理差异另有具体复核，保留宏/源码位置限制 |
| 配置/构建/IDL | 类型和有序值、显式 XML/模板模型及具体源文本检查；2 个 IDL 为 59/40 个 token，执行内容不变 |
| 活跃来源清单 / VIO 原补丁 | 304/304 指纹通过；原 patch 原样，严格逆向预检及正逆往返通过 |
| PCT | CPU 编译通过；合成 free-grid 路径 found=true、5 个路径点 |

本轮没有执行完整 ROS/Gazebo/Isaac/实机/训练。C/C++ 的宏展开、全部模板实例化与调试源码位置变化未作全面运行证明；通过的 PCT 局部编译不能替代整栈验证。旧验证回执保留原始快照，不回填本轮指纹。

所有改动均为可读性/说明层，**未删除校验、未放宽阈值，生产算法标记、数值与接口保持**；仅 `tests/unit/test_building_entry_closure.py` 两处“依赖源码拼写的 assert → 同义/更精确的 AST 结构 assert”是唯一的测试 AST 例外（未删断言、未加 skip、未放宽容差），已在 project summary 单列。

## 边界

- 本目录是**历史审查快照**：数字、哈希与 action 记录都是检查时的状态；之后的结构重构/新测试不会自动回填，引用前先看 `verification.json` 的 `checked_at_utc` 与 baseline commit。
- 本目录不是验收报告：`changed`、AST/token 相同、CTest 通过都不等于导航/仿真/实机通过，也不等于完整运行等价。
- 不要修改 `project/vendor/addendum/coverage.json` 来“对齐”当前状态；如需修正说明，写在本 README 或新增回执中。
