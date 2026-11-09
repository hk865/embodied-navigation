> Root 最终复验已完成：54 项单元通过；回归 57 通过、5 项缺少 Torch 跳过；304 项当前来源指纹与原 VIO 补丁检查通过，PCT CPU 编译/合成路径通过。以下保留 DSH 分工阶段记录，其中“待 root 刷新/复验”已由[最终验证记录](../verification.json)接续；完整 ROS/仿真/实机/训练仍未验证。

# owner=project 全量代码/配置可读性复核摘要

## 范围、基线与环境

- 仓库：`/home/hyh001/projects/1.Project/embodied-navigation`
- 分支：`improve/module-docs-readability`
- 基线：`5e7982ea3c82f2ae263b46e156a90cafca86780c`（本摘要中的 baseline_sha256 均取自该基线的 scope.json；worktree 在开始前与该提交一致）
- 范围：`scope.json` 中 `ownership=project` 的全部 394 项 = 310 个代码/配置目标 + 84 个只读关联参考。目标组成为 265 code + 45 configuration；参考为 84 reference。
- 环境限制：本会话 bash 的 `/tmp` 是每次调用独立的私有 tmpfs，与 host `/tmp` 隔离；因此分组中间记录放在工作区 `.dsh-subagents/.../work/`，host `/tmp` 副本不作为合并来源。未修改模型/DSH/网络/密钥/权限，未安装、未构建全栈、未启动 ROS/Gazebo/Isaac/机器人/训练，未 git commit/push。

## 执行方式

- DSH 原生 Agent Team：5 个互不重叠的 owner=project 分组（project-1..5，40/59/86/63/62 个目标文件）各由一名 teammate 全量逐文件审阅；另 1 名 teammate 只读审阅 84 个 reference。全部通过共享任务板领取、报告并标记完成。
- 每个文件都记录了 `path / baseline_sha256 / final_sha256 / kind / action / readability_work / read_evidence / checks_and_thresholds / validation`，见 [coverage.json](coverage.json)。
- 格式化：`ruff format --isolated --target-version py312 --line-length 100 --no-cache`，253 个 `.py` 全部**显式传路径**（含 tests/scripts/docker 下目标），最终 `253 files already formatted`。C/C++ 不使用未经授权的执行方式：本沙箱内 clang-format 不可执行（host `/tmp` 隔离、扩展二进制无执行位；按约束未复制/chmod/绕过），4 个 `.cpp` 由人工审阅/等价排版，并由令牌级检查兜底。JSON 仅在语义等价前提下重排 2 个文件；YAML/XML/CMake/Dockerfile/.sdf.in 以保留注释的手工排版与说明为主。
- 语义检查工具（位于任务 work/tools）：
  - Python：归一化 AST（忽略位置与前导 module/class/function docstring；允许普通多名称 `import` 按原顺序拆行；其余节点、字面量、比较、导入顺序与 `type_comments` 均参与比较）。
  - C/C++：去注释后的 token 序列（保留预处理逻辑行与 include 顺序，字符串/字符字面量精确比较）。
  - JSON：键顺序、标量类型、重复键与值；YAML：解析值 + 每个 mapping 的原始 key 序列；XML：元素/属性/text/tail；TOML：解析值；CMake：token 序列；Dockerfile/.sh/.dockerignore 等：忽略注释/空行后的显著行。

## 覆盖与改动统计

| 项 | 数量 |
| --- | --- |
| 覆盖条目（= manifest 集合，无 missing/duplicates/pending） | 394 |
| 310 个代码/配置目标 | 310 |
| — action=changed | 228 |
| — action=already_clear | 58 |
| — action=empty（0 字节包标记） | 24 |
| 84 个只读 reference（DSH 审阅 checkpoint：action=locked_exact，记录时最终 sha256 与基线一致） | 84 |
| code 目标中 changed / already_clear / empty | 219 / 22 / 24 |
| configuration 目标中 changed / already_clear | 9 / 36 |
| changed 文件扩展分布 | .py 214、.yaml 3、.cpp 3、.json 2、.xml 2、Dockerfile 1、Dockerfile.* 1、.sdf.in 1、.txt 1 |
| 目标文件相对基线的 diff | 228 files changed, +39099 / -18056（含格式化重排） |

- 214 个 changed `.py` 中，ruff format 覆盖全部 253 个 `.py`；其中相当一部分另有逐文件的人工 docstring/注释补充、长条件/字典/分号链展开、import 拆行。具体到每个文件做了什么是单文件具体的，不复用模板；`readability_work` 394/394 唯一。
- 58 个 already_clear 均给出保留依据（例如生成型路线 JSON 的数据语义、`__init__.py` 导出行为、锁文件精确版本、Docker 规则顺序等），未以“已审核”泛化。
- 24 个 empty 是 0 字节 Python 包标记文件；逐一标明所属包与路径，hash 保持基线。
- 84 个 reference 在 DSH 审阅阶段仅只读，未改任何 README/许可证/历史回执/来源 manifest，记录时逐字节 sha256 与基线一致。尾轮串行收尾只按批准范围改了 2 个文档入口：根 `README.md` 增加全量报告入口、`docs/robogauge/README.md` 刷新“当前模板 SHA”指纹；这两处作为 finalization 文档/元数据修改单列（见 coverage.json 的 `integration_*` 字段），不回填成 DSH 审阅时的 checkpoint 哈希，也不重写任何历史回执。

## 独立验证结果（Lead 在全部成员完成后重跑）

| 检查 | 命令/方法 | 结果 |
| --- | --- | --- |
| 生产/目标语义等价 | `validate.py --group project-1..5` | 310/310 PASS（Python AST / C++ tokens / JSON / YAML / XML / TOML / CMake / significant lines） |
| Python 格式化完整性 | `ruff format --check`（253 个显式路径） | `253 files already formatted` |
| 参考只读 | 84 个 reference 的 sha256 重算 | DSH 审阅阶段 84/84 与基线一致；尾轮仅 2 个文档入口按批准范围更新（见上文） |
| 单元测试 | `PYTHONPATH=src PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s tests/unit -v` | Ran 54 tests, OK |
| 回归测试 | 同上 `discover -s tests/regression -v` | Ran 62 tests, OK (skipped=5)，与本环境基线一致（可选 CPU Torch 缺失） |
| 覆盖完整性 | `check_dsh_coverage.py`（只读，输出到本任务 scratch）对 310 目标逐一校验 | 集合相等、无重复、无 pending、baseline/final sha 与当前 worktree 一致、action 与文件是否变化一致 |
| 写范围 | 与 scope.json owner 对照 `git status` | project 内 228 个改动全部在目标清单；无 owner=project 之外的越界改动（vendor 下的并发改动属另一任务） |

## 测试 AST 变更（单列）

仅有 1 处测试代码的 AST 有意变化，已按用户规则保持/加强语义并单列：

- 文件：`tests/unit/test_building_entry_closure.py`，用例 `test_process_module_and_scan_config_resolve_inside_clone`。
- 原因：`ruff format` 重排了 project-5 的 `src/embodied_navigation/launch/building.py`，原测试的两条断言依赖源码字符串拼写：
  1. `assertIn("'-m','embodied_navigation.evaluation.process_resources'", source)`
  2. `assertIn("ROOT/'third_party/scan/plan_manage/config/planner.yaml'", source)`
- 处理：改为对 `building.py` 的 AST 断言——唯一定位 `start("resources", [...])` 的 argv List，断言 `-m` 存在且紧跟模块名；收集 `ROOT / <Constant str>` 的右值集合并断言包含 planner 配置相对路径。原 `is_file()` 断言与两处 `assertNotIn` 保留。
- 等价性：原断言表达的是“resources 子进程以 `-m` 模块名启动”“scan planner 配置由 ROOT 拼 repo 相对路径”，新断言直接验证同一结构事实，且“唯一调用/`-m` 相邻”比原字符串包含更严格；未删断言、未加 skip、未放宽任何容差。该文件 `7 tests OK`。
- 其余 tests/ 目标只做 ruff 格式与 docstring 层改动，归一化 AST 与基线一致（validate 默认即 PASS，无需白名单）。

## 可追溯性与产物

- 每个文件的基线/最终 sha256、kind、动作、审阅证据、阈值发现与验证方式：见 [coverage.json](coverage.json)。
- 全部发现与阈值/校验审计（含未改行为的原因）：见 [findings.md](findings.md)。
- 结论可追溯到基线 `5e7982e`；参考文件在 DSH 审阅 checkpoint 的 hash 与 scope.json 一致，尾轮未重写历史回执或来源 hash，只记录批准的 2 处文档/指纹修改。


## 串行事实修正合入（2026-11-09，DSH 全量实现，Codex 独立事实复核，DSH 串行合入）

- 批准清单：`approved-fact-integration-plan.json` 的 **109** 个独立事实/上下文候选；其中 **60** 个属于 owner=project 目标。写入前逐一复核 `current_sha256 == preimage_sha256` 且候选 SHA 正确，109/109 按字节应用、保持原 mode；Python 归一化 AST 不变，仅注释/docstring 文案变化。算法、阈值、数值、接口、调用顺序与配置含义未改。
- 本报告与 `coverage.json` 中受这些修正影响的条目已同步最终源码：`readability_work` / `read_evidence` / `checks_and_thresholds` 不再保留已被否定的初稿说法（例如 0.8 是最小时间尺度而非最小 0.8 m 扫掠、四元数约束的是平方范数 q@q、静止初始化不等于 gravity-free、bridge 不是唯一命令发布者等，逐条见 coverage.json 的 `independent_fact_review.negated_draft_claims`）。
- `coverage.json` 的 310 个 target 已按写入后的 worktree 重算 `final_sha256`（并新增 `final_bytes`/`final_lines`）；`baseline_sha256` 保持基线不变；84 个 reference 的审阅 checkpoint 不回填当前 SHA。
- 唯一测试 AST 例外仍只有 `tests/unit/test_building_entry_closure.py` 两处“源码拼写 assert → 同义/更精确 AST 结构 assert”（未删断言、未加 skip、未放宽容差），已在第 5 节单列。
- **尚未完成的统一复验**：root 将在全部串行合入完成后统一重跑合并检查、CPU 单元/回归测试与（可用时）原生 verifier；本轮未宣称整栈、ROS/仿真/实机或原生构建通过，活跃 `vendored-source-manifest.json` 的刷新也留给 root 最终执行。

## 未验证 / 限制（不夸大）

- 未运行 ROS、Gazebo、Isaac、Docker、原生编译、完整路线、实机或训练；本摘要中的“通过”仅指静态语义等价与既有 CPU 单元/回归测试。
- 本环境未安装可选 CPU Torch，回归中 5 项按 `skipUnless` 跳过，不计为通过。
- clang-format 在本沙箱不可执行；4 个 `.cpp` 中 3 个做了人工等价排版/注释、1 个判定已清晰未改，均由 token 序列等价比对兜底，未做编译器级验证。
- 超长数据/清单文件按获批方法处理：大 JSON（如 `configs/robogauge/routes/combined_floor1_inside.json` 5821 行、`configs/simulation/settings.json` 3294 行）程序化遍历全部结构与键、人工阅读关键片段；`docs/dependencies/vendored-source-manifest.json`（2455 行）解析全部 304 条并重算哈希、抽样人工阅读。方法均写入对应条目的 validation。
- 部分阈值/校验/死代码发现属“仅记录、未改行为”，建议见 findings.md；是否修改需后续单独任务与运行数据/契约确认。
