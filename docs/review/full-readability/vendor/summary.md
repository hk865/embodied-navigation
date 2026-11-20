> Root 最终复验已完成：54 项单元通过；回归 57 通过、5 项缺少 Torch 跳过；304 项当前来源指纹与原 VIO 补丁检查通过，PCT CPU 编译/合成路径通过。以下保留 DSH 分工阶段记录，其中“待 root 刷新/复验”已由[最终验证记录](../verification.json)接续；完整 ROS/仿真/实机/训练仍未验证。

# vendor 全量可读性改造总结（owner=vendor，265 个代码/配置目标）

> DSH 初稿，Codex 独立事实复核。复核仅覆盖旧 Raycast 距离量纲/调用路径、img_align 层数与零有效像素、A* 默认分数状态路径、RingBuffer empty/getSum、GradientDescentOptimizer 初始化，以及 Huber 错误断言；其余发现保留 DSH 初稿，未宣称全部独立证实。未执行原生构建、ROS、训练或算法运行，也未修改代码/阈值。

- 基线提交：`5e7982ea3c82f2ae263b46e156a90cafca86780c`（分支 `improve/module-docs-readability`）
- 范围：`scope.json` 中 `ownership=vendor` 的 **265** 个代码/配置目标，分 5 个不重叠分组（vendor-1 52、vendor-2 60、vendor-3 51、vendor-4 51、vendor-5 51）；另有 41 个 vendor 关联参考（LICENSE/README/patch/ROS1 launch/idl）只读，未重写。
- 覆盖清单：[coverage.json](coverage.json)（265 条，与 manifest 路径集合完全相等）。另有 41 个 vendor reference 在 coverage.json 的 `reference_read_only` 中单列，只读、未改写。

## 1. 方法与工具

5 名 DSH Agent Team 成员各自拥有 `scope.json` 中一个 vendor 分组，只写本组清单内文件；Lead 建立共享 scratch（基线副本、清单、验证/记录工具）并在合并时**独立重跑全部等价性检查**，不采信成员自报。逐文件动作：完整读取（长文件分段读至文件末尾）→ 理解职责/接口/配置键 → 按策略排版或补充注释 → 机器验证 → 记录证据。

| 工具 | 版本/参数 | 用途 |
|---|---|---|
| ruff | 0.17.1，`format --isolated --target-version py312 --line-length 100`，显式传文件路径，禁 `check --fix`/import 排序 | 24 个 `.py` |
| clang-format | 23.1.0，`{BasedOnStyle: LLVM, IndentWidth: 4, ColumnLimit: 100, SortIncludes: Never, IncludeBlocks: Preserve, ReflowComments: false, FixNamespaceComments: false, InsertBraces: false, RemoveBracesLLVM: false}` | 147 个 C/C++ |
| 独立校验 | 自写 lexical token/AST/配置解析比较器（见 summary 第 4 节） | 265/265 |

**格式化统计与内容审阅统计分开**：164 个文件内容改变，其中 154 个由 formatter 排版（clang-format 143 / ruff 11），10 个仅由人工补充注释/排版（manual-cmake/yaml/sh）；其余 101 个文件在 formatter 尝试后无可读性收益、为空文件或宏/行号锁定而保持字节不变。所有 changed 文件都经过逐文件完整阅读并记录 `read_evidence`；formatter 本身不算完成审阅。

## 2. 结果统计

| 分组 | 文件数 | changed | already_clear | empty | locked_exact |
|---|---:|---:|---:|---:|---:|
| vendor-1 | 52 | 40 | 4 | 0 | 8 |
| vendor-2 | 60 | 27 | 14 | 9 | 10 |
| vendor-3 | 51 | 33 | 11 | 0 | 7 |
| vendor-4 | 51 | 34 | 9 | 0 | 8 |
| vendor-5 | 51 | 30 | 11 | 0 | 10 |
| **合计** | **265** | **164** | **49** | **9** | **43** |

按格式化策略：

| 策略 | 目标数 | changed | 说明 |
|---|---:|---:|---|
| clang-format | 147 | 143 | C/C++ 排版（include 顺序/宏结构不动） |
| ruff | 24 | 11 | Python 排版（AST 等价） |
| manual-cmake | 14 | 4 | 人工审阅/保留原布局 |
| manual-yaml | 11 | 5 | 人工审阅/保留原布局 |
| manual-xml | 13 | 0 | 人工审阅/保留原布局 |
| manual-json | 7 | 0 | 人工审阅/保留原布局 |
| manual-msg | 4 | 0 | 人工审阅/保留原布局 |
| manual-sh | 1 | 1 | 人工审阅/保留原布局 |
| manual-other | 1 | 0 | 人工审阅/保留原布局 |
| locked-exact | 43 | 0 | 宏/行号敏感，保留基线布局（见 findings.md 第 2 节） |

代码规模：59,415 行 / 2,063,475 字节（按 scope.json 的 lines/bytes 汇总）。格式化 diff：+25,289 / -24,474 行（全部 changed 文件合计）。成员新增的可读性注释（近似统计，排除 formatter 影响）：约 767 行，涉及 124 个文件。

## 3. 覆盖完整性与不变原因

- 路径集合：manifest 265 条 = 记录 265 条；missing=0、extra=0、duplicates=0、pending=0。实际完整读取 265/265 个文件（合计 59,415 行），写入（changed）164 个文件，其余 101 个按具体理由保持字节不变。
- 每条 coverage 记录含 path/baseline_sha256/final_sha256/kind/action/readability_work/read_evidence/checks_and_thresholds/validation；`read_evidence` 全集合唯一性已检查（空文件除外），未见机械模板。
- 未改动文件的分类原因：49 个 `already_clear`（formatter 无差异或原排版/注释已清晰，逐条记录具体依据），9 个 `empty`（0 字节 `__init__.py`），43 个 `locked_exact`（格式化无法证明宏/行号等价，逐文件记录宏名与风险证据）。

## 4. 独立检查与阈值

- Python：AST 等价（忽略位置与 leading module/class/function docstring；`import a, b` 允许按原顺序拆行；包含 type_comment）。24 个 `.py` 目标全部通过。
- C/C++：lexical token 序列（字符串/字符字面量逐字保留）、预处理指令序列、`#define` 名称与 `(` 的邻接性（函数宏 vs 对象宏）、include 顺序、许可证注释保留、无新增 `__LINE__`/`__FILE__`/`#line`；147 个 clang-format 目标全部通过（143 个有排版 diff，4 个无差异或已恢复基线）。43 个 locked-exact 文件按字节比对，全部未变。
- 配置：JSON（键序/重复键/类型严格相等）、YAML（解析等价+重复键+基线注释保留）、XML（元素树等价+<license> 文本保留）、CMake（去注释有效行等价 + `cmake -P` 语法解析）、shell（`bash -n` + 有效行）、`.msg`（字段 token）。
- 全局：合并阶段重新计算 265 个文件 SHA，核对 action 与磁盘一致；扫描 `third_party` 下清单外修改/未跟踪文件（最终见第 6 节）。
- 权限：格式化/沙箱写入曾把部分文件归一化为 0600 并令 8 个 0775 文件丢失可执行位；合并后已按仓库工作区惯例恢复为 0664（普通）/0775（可执行），git 可执行位与 index 一致、mode change=0，内容 SHA 不受 chmod 影响。
- 测试：未删除/放宽任何断言、未加 skip；vendor 内测试文件（如 `test_uniform_bspline.cpp`、`test_planner_startup.py`、`test_waypoint_parameters.py`）仅排版，未出现依赖源码字符串的断言，因此没有测试 AST 变更需要向 root 单列。

**验证边界（不夸大）**：以上全部是无外部依赖的静态检查；未运行原生构建、ROS/仿真、训练或机器人，未安装依赖、未联网。因此不宣称“原生构建通过”或“运行验证通过”。格式化对 `#include` 的语义（如宏展开后类型/ABI）不在静态可证范围内，但 token/指令等价与 include 顺序保持已逐文件验证。

## 5. 来源与补丁影响（需 root 处理，Lead 未改）

- `docs/dependencies/vendored-source-manifest.json` 的 `vendored_sha256` 是活跃的复制完整性校验；163 个被改文件在该 manifest 中有记录，其 `vendored_sha256` 与当前磁盘不一致（串行合入后又应用了 49 个 vendor 事实修正，after 值已按最终源码重算并写入 coverage.json `summary.source_manifest_impact` 与 findings.md 第 3 节），`docker/verify_native_sources.py` 在 manifest 刷新前仍会在第一个不一致处失败。逐文件 before/after SHA、upstream source 路径与 `source_sha256` 见相应字段；Lead **未**修改该 manifest，也未改任何 `source_sha256`/upstream commit/历史回执。
- 1 个被改文件不在 source manifest 中（`third_party/pct/CMakeLists.txt`），需 root 决定是否纳入。
- VIO floor patch：`third_party/fast_livo2/patches/0001-consistent-negative-voxel-floor.patch` 的 reverse dry-run 依赖 `vio.cpp` 的精确上下文。串行合入采用“事实更正 + 原 floor 补丁 4 处共 30 行上下文兼容”的最终单一候选，故 default 与 `--fuzz=0` reverse dry-run 以及正向/反向 roundtrip 均通过（见 `vio-facts-minimal-compat/report.json`）；patch 与验证器本身未修改，也未新增 296 行 helper/格式层 patch。

## 6. 范围纠正与遗留

- 并发进程（经各组独立核对，均否认且其工具/清单证据一致指向不是 5 个 vendor 成员）曾修改 10 个清单外 reference：third_party/livox_ros_driver2/launch_ROS1/ 下 8 个 .launch 与 third_party/scan/scan_planner_msgs/msg/ 下 2 个 .idl。Lead 用 `git show 5e7982e:<path>` 恢复，逐个 SHA-256 与 scope.json 记录一致；之后多次复查 git status，third_party 下无清单外修改、无未跟踪文件。reference 不进入 coverage，不重写历史回执/来源 hash。
- 格式化过程中发现 clang-format 23.1.0 默认会拆分 C 长字符串字面量（BreakStringLiterals），使 token 流不等价。受影响文件（planner_manager.cpp、r107_transport_probe.h、LIVMapper.cpp、scan_replan_fsm.cpp、CustomPoint.h、CustomMsg.h 等）已由成员按基线逐字面量拼回或整文件恢复，最终 token/字符串字面量校验通过；建议后续格式器显式加 BreakStringLiterals: false（本次未改动用户给定的 clang-format style）。
- 任务建议把各组工作记录放 /tmp/embodied-full-readability/，但本会话沙箱中 /tmp 是每次 bash 调用独立的 tmpfs（跨调用/跨成员不可见），因此共享 scratch 实际放在 .dsh-subagents/2026-11-08T20-06-07-316Z-5970cd5a/scratch/（未提交、未纳入交付目录）；5 组记录在该 scratch 下合并后写入 docs/review/full-readability/vendor/。
- 权限：clang-format/ruff 与沙箱写入曾把部分文件权限归一化为 0600，并令 8 个原本 0775 的 C/C++ 文件丢失可执行位。Lead 在合并后已按仓库工作区惯例恢复为 0664（普通）/0775（可执行），并用 git index 校验可执行位：265 个目标权限直方图 0664×251 + 0775×14，git 记录的 mode change 为 0，内容 SHA 不受 chmod 影响。

未决/需要数据：

- 来源 manifest 刷新由 root 统一处理：被改的 164 个文件中有 163 个受 docs/dependencies/vendored-source-manifest.json 跟踪，其 vendored_sha256/bytes/modified 需要更新（逐文件 before/after 见 findings.md 第 3 节与 coverage.json 的 summary.source_manifest_impact；after 已按最终源码重算，保留 source_sha256/upstream 身份），是否把仅因可读性排版/注释变化的记录标为 modified 需 root 决策；刷新前 docker/verify_native_sources.py 必然在首个不一致处失败。
- VIO floor patch 兼容性已复核通过（30 行原上下文保留；default 与 `--fuzz=0` reverse dry-run + 正逆 roundtrip 通过）；root 仍需在最终源稳定后复跑 `docker/verify_native_sources.py --compile-pct`，本报告不据此宣称原生构建通过。
- 14 个 C/C++ 源文件基线为混合 CRLF/LF，clang-format 后部分行以 LF 归一化（无 BOM 变化）；token/AST 等价不受影响，但 diff 与 patch 上下文会变化。若要求保留原行尾，需 root 决定回退或显式 LineEnding 策略。
- vendor-5 在 third_party/scan/plan_manage/config/controllers.yaml 补的单位注释中，heading_error_threshold 的 rad 语义属静态推断，建议控制侧用数据复核（只改注释，未改值）。
- 静态检查不能覆盖：宏展开后的类型/ABI、模板实例化、原生构建、ROS/仿真/训练/机器人运行；本任务未执行这些验证，也未安装依赖或联网。

## 7. 参考文件（只读，不计入 265）

本次只读审阅了 vendor 关联参考中的来源/许可/patch 说明，用于校准来源影响；未重写任何 LICENSE、README、patch、ROS1 launch/idl 或历史回执。实际只读参考：

- third_party/README.md（16 行，完整）：声明本目录为可提交的实际源码而非链接；唯一有意改动的算法源是 fast_livo2_core/src/vio.cpp（负数体素 floor 修复，与迁移前隔离修复版字节一致），patch 保存在 fast_livo2/patches/；逐文件 SHA256 仅用于复制与来源校验，不作为运行验收。
- third_party/fast_livo2/patches/0001-consistent-negative-voxel-floor.patch（42 行）：对 vio.cpp 两处循环共 4 个 floor 修改及 2 行注释；本次只格式化 vio.cpp，未改/未删 patch。
- docs/dependencies/vendored-source-manifest.json（只读抽查）：304 条 destination 记录，其中 264 条属于本次 265 目标（pct/CMakeLists.txt 不在其中），记录成对保存 source_sha256/vendored_sha256，2 条 modified=true（vio.cpp、livox_ros_driver2/.gitignore）。
- docs/dependencies/verification.json（只读）：基线记录 manifest_files_verified=304、vio_patch_reverse_dry_run=passed、cmake_files_parsed=12、python_files_parsed=28、PCT 编译与合成 free-grid smoke 通过，ros_compile/docker_build/simulation 均为 not run。
- third_party/fast_livo2/LICENSE（GPLv2 文本头）、third_party/vikit/COPYING（GPLv3 文本头）、third_party/livox_ros_driver2/3rdparty/rapidjson/license.txt（MIT 与第三方组件清单头）、third_party/fastdds/README.md（Fast DDS 2.14.6 backport 三个 patch 的说明，作用于公开上游源码而不是本仓库已提交文件）：均只读，未改任何许可证/版权文本。
- third_party/livox_ros_driver2/launch_ROS1/*.launch 与 third_party/scan/scan_planner_msgs/msg/*.idl：发现被并发进程修改后已恢复为基线字节并逐个校验（见 corrections）；未列入 coverage。


## 8. Codex 对关键算法发现的独立事实复核（限定范围）

以下复核修正文案与影响范围，未改变算法。前文的覆盖、格式化与校验统计仍属 DSH 成员/Lead 阶段记录；其余发现不因本节而成为已独立证实的结论。

- **Raycast 距离**：旧指针自由函数确实将长度与长度平方比较；尺度影响可宽可窄，不能一概称距离上限失效。当前 GridMap 使用独立 `RayCaster::setInput/step`，其距离检查被注释、实际按终点格与线段参数终止；受版本管理 C/C++ 源码未发现自由 `Raycast` 调用。冻结 `original_raycast.cpp` 保留原始问题用于回归，不修它。
- **img_align 层数与零像素**：`ForwardCompositionalSE3` 的两个构造函数都未赋 `n_levels_`，带初始模型的构造函数会立即进入读取该成员的默认优化路径。ESM 用形参 `n_levels`，不属于这一初始化问题。两类残差函数都无条件除以有效像素数，零像素输入的失败契约需单独定义。它们被编入库，但未发现仓库 C/C++ 实例化，当前导航触发情况未证实。
- **A* `inf=1>>20`**：表达式为 0 是确定事实；正常首次可通行分支先赋分，故不能据此直接判定当前搜索失败。也不能说默认值绝不比较：`rounds` 标记早于带 yaw 的占用检查，双圆柱足迹会随入射方向改变。需验证同轮先阻塞、后另一方向可通行的重访及重复搜索等状态不变量；本轮不换成大数或无穷。
- **RingBuffer**：正容量新对象 `empty()==false` 而 `size()==0` 确认是 API 不一致；未发现仓库 C/C++ 实例化。`getSum()` 在当前只追加、写满后覆盖的模型中遍历的元素集合正确，`getMean()` 已防零计数，不应把求和另列为现存缺陷。
- **GradientDescentOptimizer**：实际可发生读前未初始化的是调用方未先设置的 `min_grad_`；另三个无默认值的 double 在当前实现没有读取。未发现仓库 C/C++ 实例化或调用，编入库不等于活跃主优化器使用。设为 0 会被现有 guard 拒绝，删除公开设置器也不能未经外部契约检查称为等价。
- **撤回 Huber 误报**：`configure(const float &param) { k = param; }` 的参数名是 `param`，左侧 `k` 是成员；构造函数已经调用它赋值。原“形参遮蔽、成员未初始化”结论错误，无需为此修改源码。

本节复核的候选已由 DSH 串行合入执行（见第 9 节）；算法/阈值/接口/配置含义未改。

## 9. DSH 串行合入完成记录（DSH 全量实现，Codex 独立事实复核，DSH 串行合入）

- 批准清单 `approved-fact-integration-plan.json` 共 **109** 个候选（owner=vendor **49** 个）。写入前逐一复核 `current==preimage` 且候选 SHA 正确，109/109 按字节应用、保持原 mode；Python 归一化 AST 不变，仅注释/docstring 文案变化；未改算法、阈值、数值、接口、调用顺序或配置含义，未删除校验、未放宽阈值。
- 受这些修正影响的 coverage 条目已同步最终源码（`readability_work`/`read_evidence`/`checks_and_thresholds`），不再保留被否定的初稿说法：例如均匀 B 样条并非 clamped、`planning_horizon_` 为米、`publish_interval_` 不先取整到 10 ns 网格、广播码白名单未被填充、`frame_counter_` 非原子，以及 A*/RingBuffer 等复核范围内的限定结论。逐条见 `coverage.json` 的 `independent_fact_review` 与 findings.md 第 8 节。
- `coverage.json` 的 **265/265** 条已按最终 worktree 重算 `final_sha256`/`bytes`/`line_count`；`source_sha256`/upstream/model/archive/历史回执身份未重写；41 个只读 reference 未改写。
- VIO 使用最终单一候选（SHA `f394b293…`）：事实更正 + 原 floor 补丁 4 处共 30 行上下文兼容；default 与 `--fuzz=0` reverse dry-run 及正向/反向 roundtrip 通过（`vio-facts-minimal-compat/report.json`）。原 patch 与 `docker/verify_native_sources.py` 未修改，未新增 helper 或格式层 patch。LBFGS 宏 replacement separator 已并入唯一候选。
- **仍留给 root**：活跃 `docs/dependencies/vendored-source-manifest.json` 的 `vendored_sha256`/`bytes`/`modified` 刷新、`docker/verify_native_sources.py --compile-pct` 复跑、统一 CPU 单元/回归测试与最终合并检查。在上述复验完成前，不宣称原生 verifier、整栈、ROS/仿真/实机或训练已通过。
