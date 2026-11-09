# 全量代码/配置可读性报告总入口

本目录是当前仓库 **589 个代码/配置目标**的可读性改造成果与事实复核总入口。
三套报告分别覆盖不同 ownership，路径集合互不重叠，合计严格等于 589：

| 报告 | ownership | 目标数 | 入口 |
| --- | --- | ---: | --- |
| project | 自有源码、脚本、构建、配置 | 310 | [coverage.json](project/coverage.json) · [findings.md](project/findings.md) · [summary.md](project/summary.md) |
| vendor | `third_party/` 下引入的开源算法与配置 | 265 | [coverage.json](vendor/coverage.json) · [findings.md](vendor/findings.md) · [summary.md](vendor/summary.md) |
| addendum | 上两任务未覆盖的 launch/IDL/原始 patch | 14 | [coverage.json](addendum/coverage.json) · [findings.md](addendum/findings.md) · [summary.md](addendum/summary.md) |
| **合计** | | **589** | |

## 589 的构成

- **504 = 代码 / 接口 / 脚本 / 构建 / patch**：Python、C/C++、CMake、Dockerfile、`.msg`、`.idl`、shell 与原始 patch 等实现层文件。
- **85 = 配置与 pin**：YAML/JSON/XML/SDF 模板（含 ROS `.launch`）、路由、契约与依赖 pin 等运行/构建配置。
- 另有 **111 个关联参考**：README、LICENSE、历史回执、来源 manifest 等，不计入 589。原分工按只读交叉核对；最终同步了当前文档和来源校验元数据，保留原始身份及历史回执。

覆盖清单与 ownership 来自 scope 定义；`check_dsh_coverage.py` 会逐条校验 589 个目标路径集合、无重复/缺失/pending，以及 `baseline_sha256`/`final_sha256` 与当前磁盘一致。

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
