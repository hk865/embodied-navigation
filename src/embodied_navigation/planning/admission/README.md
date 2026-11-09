# planning/admission

本目录是路径准入与静态几何证明层：判断 SCAN 轨迹能否进入控制回路，并对 PCT 折线做静态支撑/碰撞证明；所有输出都只是必要条件，不是运动、到达或执行 ACK。

## 边界

- 做：route-fence 包含性证明（`path_admission.py`）；typed ground/body 静态证书与起点连接器（`static_geometry.py`）；把几何证明绑定到真实准入（`physical_gate.py`）；异步单 owner 证书（`async_admission.py`、`geometry_worker.py`）；替换预览、因果提交与私有绑定（`path_handoff.py`、`pending_path_commit.py`、`path_commit.py`）；固定 FIFO 预算与默认 OFF 开关（`queue_budget.py`、`exhausted_replacement.py`、`replan_ack.py`）。
- 不做：不替代来源新鲜度、航向、停车、native guard 等独立门禁；不新增规划或控制数学；不写决策日志（决策归档在 `navigation/pid_admission.py`，且默认证据写入器是空操作，见下）。

### 两条 fence 分支（读代码前先分清）

| 分支 | 类 | 何时使用 | 本检出状态 |
| --- | --- | --- | --- |
| fallback | `FrozenFence`（`path_admission.py`） | 仅在 R85 关闭时：`pid_navigation.py` 用 `AutomaticRouteFence if automatic_reference is not None else FrozenFence(...)` 选择 | **未闭合**：要求 `navigation_anchor.json` 带 `frozen_once`、`scene_axis_registration_file`、`scene_axis_registration_file_sha256` 等字段，而唯一写入方 `navigation/mission/mission_runtime.py` 写出的 anchor 字段集不同（grep 全仓，`frozen_once` 只在 `path_admission.py` 出现），`load()` 会抛 `ValueError` 而非放行 |
| 默认 profile | `AutomaticRouteFence`（`reference/automatic_reference_runtime.py`）+ `R85PhysicalPathGate`（`physical_gate.py`） | `configs/planning/settings.json` 的 `r85_auto_reference_contract.enabled=true` ⇒ `automatic_reference` 非 None，同时构造两道门 | 实际默认路径；`AutomaticRouteFence.load()` 返回 `body_points + registered_route_fence + r85_automatic_route_fence_binding/v1`，`R85PhysicalPathGate` 在其上再加物理证书 |

## 目录结构

```text
src/embodied_navigation/planning/admission/
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），包不导出 API，import 需写全路径
├── async_admission.py  # 1199 行，R88 异步单 owner 证书（opt-in）
├── exhausted_replacement.py  # 13 行，R103 耗尽路径替换开关（默认 OFF）
├── geometry_worker.py  # 513 行，独立解释器跑静态证书（子进程 + 管道 IPC）
├── path_admission.py  # 486 行，连续 cubic 曲线的 FrozenFence 证明
├── path_commit.py  # 68 行，回调提升前的私有 cascade 绑定
├── path_handoff.py  # 405 行，替换预览/提前请求（不改控制数学）
├── pending_path_commit.py  # 168 行，等真实来源因果后再提交
├── physical_gate.py  # 625 行，R85 几何证明↔真实准入的绑定
├── queue_budget.py  # 29 行，固定 2 MiB 业务 FIFO 预算
├── replan_ack.py  # 17 行，早期重规划 ACK 开关（默认 OFF）
├── static_geometry.py  # 1009 行，typed ground/body 静态证书 + CLI
└── validate_route_support.py  # 1002 行，PCT 折线有限面/OBB 静态校验 + CLI
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（活库） | `path_admission.py`、`path_commit.py`、`path_handoff.py`、`pending_path_commit.py`、`physical_gate.py`、`queue_budget.py` | 被 navigation 侧直接 import 的准入逻辑；`certify_curve` 用 de Casteljau 分区 + Bézier 凸包球证明连续 cubic 曲线落在冻结 3D 走廊内，超预算或不确定即拒绝；`path_handoff.py` 的函数首参是 `self`（node），靠鸭子类型操作调用方对象 |
| 代码（活库） | `async_admission.py`、`geometry_worker.py` | R88 路径：主线程只在原事务中复用子进程算好的证书，事件走有界 FIFO（`MAX_EVENTS=32`、`MAX_EVENT_BYTES=1 MiB`、总 `MAX_QUEUE_BYTES=2 MiB`）；`R88_ASYNC_PHYSICAL` 默认 `0`，opt-in |
| 代码（CLI + 库） | `static_geometry.py` | 既被 `physical_gate`/`geometry_worker` 当库，又有 CLI：`python3 -m embodied_navigation.planning.admission.static_geometry {prepare,curve}`；`prepare` 写 4 个新文件、`curve` 写 1 个 receipt，都拒绝覆盖；返回值里的 `allowed` 只代表静态几何，不授控制权 |
| 代码（CLI） | `validate_route_support.py` | 独立 CLI：`--route` 或 `--body-npy/--ground-npy`、`--profile`（必填）、`--output`（必填，已存在则 `FileExistsError`）；同时被 `static_geometry` 以文件路径动态加载 |
| 代码（开关） | `exhausted_replacement.py`、`replan_ack.py` | import 时读 `R103_EXHAUSTED_DRIVE_REPLACEMENT` / `R103_EARLY_REQUEST_ACK`，默认 `"0"`，非 `0/1` 立即 `ValueError` |
| 代码（空） | `__init__.py` | 0 字节，无 `__all__`、无重导出 |

日志/输出（谁写、何时、是否真落盘）：

| 位置 | 写入方 | 触发条件 | 是否有代码消费者 |
| --- | --- | --- | --- |
| `runs/<run_id>/admission_failure.json` | `async_admission._save_failure_snapshot` | 仅 intake 故障时写一次（文件已存在则跳过），字段含 `reason`、`counts`、队列快照 `events`、`worker_busy`、`worker_exit` | 无（grep 只命中本文件） |
| `runs/<run_id>/r88_async_admission.jsonl` | `async_admission.record` | 每个异步事件一次；走 `node.evidence.append` | 无。**默认不落盘**：`navigation/ros_node.py` 的 `make_evidence_writer` 返回 `RuntimeTelemetry`，其 `append/enqueue` 直接 `return None`（`common/runtime_telemetry.py`） |
| `runs/<run_id>/navigation_pending_path_commits.jsonl` | `pending_path_commit.record` | 每次 pending 提交事件；同样走 `node.evidence.append` | 无；默认不落盘 |
| `runs/<run_id>/path_handoff_early_requests.jsonl` | `path_handoff.early_request` | 每次 early request 预览；走 `self.evidence.append` | 无；默认不落盘 |
| `--output` / `--output-dir` 指定的文件 | `static_geometry`、`validate_route_support` 的 CLI | 只在显式跑 CLI 时写；`static_geometry prepare` 产物为 `typed_r85_global_reference.json`、`physical_route_admission.json`、`typed_ground_xyz.npy`、`typed_body_xyz.npy` | 无代码消费者，人工核对用 |
| `path_admission_decisions.jsonl`、`path_admission_candidates/` | **不是本目录**：`navigation/pid_admission.py` 的 `archive_admission` | 同样走 `self.evidence`，默认不落盘 | 无 |

`validate_route_support` 的 receipt 固定写 `ready_for_control=false`、`navigation_PASS=false`、`schema=pct_static_polyline_support_receipt/v1`——即 receipt 本身不授权。

配置（读取点已核实）：

| 配置 | 读取位置 | 语义 |
| --- | --- | --- |
| `configs/planning/static_support_limits.json` | `static_geometry.py` 模块级经 `common/paths.py` 的 `GEOMETRY_CONFIG` 取路径并算 SHA；`_context()` 用 `validate_route_support.limits()` 加载 | 冻结的静态支撑上限；`limits()` **只许收紧不许放宽**：`body_radius_m≥0.25`、`body_offset_m≥0.18`、`body_below/above_m≥0.12`、`sample_spacing_m≤0.025`、`support_height_tolerance_m≤0.10`、`max_support_slope_rad≤0.35`，`yaw_half_range_rad=π`、`body_ground_height_m=0.32` 为冻结等值。默认值即采样间距 0.025 m、body 半径 0.25 m、高度带 ±0.12 m |
| `assets/geometry/metadata.json` | `static_geometry.METADATA`（`common/paths.GEOMETRY_METADATA`），首次 `_context()` 才读 | 私有测量图（外部数据卷，不入 Git）；缺失时 `certify_*` 抛错 |
| `runs/<run_id>/known_scene_reference.json` | `physical_gate._bind_reference_context`（2 MB 上限 + SHA receipt） | R85 物理绑定的实际场景参考 |
| profile 传入（非直接读文件） | `path_admission.FrozenFence.load` 用 `profile["original_scenario"]`（46 区域）与 `profile["registered_route_fence"]`（水平 0.45 m / 高度 0.3 m） | fallback 分支的路由先验 |
| CLI 默认路径（本检出缺失） | `validate_route_support.main` 的 `--geometry-metadata` 默认 `admission/data/known_scene43_camera_init_h005/metadata.json`、`--config` 默认 `admission/support_validation_config.json` | 两个文件未随检出提供，直接跑默认参数会失败，需显式传参 |

环境变量（全部由代码核实）：`R88_ASYNC_PHYSICAL`、`R110_POLL_YIELD`、`R116_RECEIVE_PRESSURE`、`SCAN_SHORT_INTAKE`、`R94_SCAN_READY_POLL`、`R93_HANDOFF_PROBE`（后两者在 `geometry_worker`/`async_admission`）、`R97_DIAGNOSTIC_QUEUE_BYTES`（只接受未设置或恰为 `2097152`）、以及两个默认 OFF 的 `R103_*`。

## 模块内文件耦合

**方向结论**：无环，分三层——基础库（`path_admission` / `path_commit` / `queue_budget` / 两个开关）、证书层（`static_geometry` ← `geometry_worker` / `physical_gate` ← `async_admission`）、编排层（`path_handoff` → `pending_path_commit`）。所有跨模块 import 都是普通 import，唯一例外见"最可疑"一条。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `path_admission.py` | 486 | 无内部依赖（numpy/scipy） | `pending_path_commit`、`navigation/pid_support.py` | 偏重（`certify_curve` 214 行） |
| `path_commit.py` | 68 | 无内部依赖 | `navigation/pid_support.py` | 清楚 |
| `queue_budget.py` | 29 | 无 | `async_admission` | 清楚 |
| `exhausted_replacement.py` | 13 | 无 | `path_handoff` | 清楚 |
| `replan_ack.py` | 17 | 无 | `path_handoff`、`reference/automatic_reference_runtime` | 清楚 |
| `path_handoff.py` | 405 | `exhausted_replacement`、`pending_path_commit`、`replan_ack`、`reference/global_route_reference` | `navigation/pid_support.py` | 可接受（7 个 node 方法式函数） |
| `pending_path_commit.py` | 168 | `path_admission`、`common/observation` | `path_handoff`、`navigation/pid_support.py` | 清楚 |
| `static_geometry.py` | 1009 | `common/config`、`common/paths`、`reference/route_suffix`、**动态加载 `validate_route_support.py`** | `geometry_worker`、`physical_gate`、`reference/automatic_reference_runtime`（运行期 importlib） | 偏重 + 耦合可疑（见下） |
| `geometry_worker.py` | 513 | `static_geometry`、`common/observation` | `async_admission` | 可接受（IPC 边界自包含） |
| `physical_gate.py` | 625 | `static_geometry`、`common/config`、`common/observation`、`localization_mapping/known_scene_matcher` | `async_admission`、`navigation/pid_navigation.py`、`control/safety/duplicate_source.py` | 偏重（157 行单函数、鸭子类型 node 契约） |
| `async_admission.py` | 1199 | `geometry_worker`、`physical_gate`、`queue_budget`、`reference_protocol`、`common/observation` | `navigation/pid_entrypoint.py` | 偏重（31 函数、4 个 >80 行） |
| `validate_route_support.py` | 1002 | `common/config` | `static_geometry`（动态加载）、CLI 人工调用 | 偏重（`run()` 360 行） |
| `__init__.py` | 0 | 无 | 无 | 清楚（纯空文件） |

四类问题：

- **上帝文件**：`async_admission.py` 1199 行、`static_geometry.py` 1009 行、`validate_route_support.py` 1002 行，三个都超过 800 行；`physical_gate.py` 625 行接近阈值。
- **重复实现**：`payload()` 在本目录 `path_admission.py`、上层 `planning/reference_protocol.py`（schema 3）、`planning/trajectory_contract.py`（schema 1）各有一份，代表三代消息协议；`canonical`/`digest`（`json.dumps(sort_keys=True, separators=…)` + SHA256）在 `path_admission.py` 与 `physical_gate.py` 各实现一次。
- **跨层直达**：①`static_geometry` 用 `importlib.util.spec_from_file_location` 把同目录 CLI 脚本当库加载（`_helpers()`），并在 **import 时** 读+hash 该文件（`LEGACY_SHA` 会进 receipt 的 `frozen_geometry_helper_sha256`）——兄弟文件任何字节改动都会改变证据字段；②`control/safety/duplicate_source.py` 直接读 `node.r85_physical_gate.static_sources` 与 physical binding dict，而非通过函数返回。
- **隐式契约**：证据文件名（4 个 `.jsonl` + 1 个 `.json`）与 JSONL 字段是散落的字符串；`path_handoff`/`pending_path_commit`/`physical_gate` 的方法都收 `node` 并访问其多个属性（`run`、`pose_stamp`、`request_id`、`trajectory_association.route_key`、`automatic_reference.doc`…），没有 `Protocol`/类型声明，靠调用方满足。

结论：耦合**方向清楚但不轻**——分层的 import 无环，可读/可测的单元在基础层；真正的负担是三个千行文件和 `static_geometry ↔ validate_route_support` 的"文件路径动态加载 + 内容哈希"耦合，改动其中任一文件都要同时想到对侧证据字段。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `queue_budget.py` | 良好 | 29 行 1 函数、最长 18 行，模块 docstring + 常量注释交代"4 MiB 只是诊断上限、这里不请求" |
| `exhausted_replacement.py`、`replan_ack.py` | 良好 | 13/17 行纯开关，模块 docstring 说清"默认 OFF、不改任何阈值"；失败即 `ValueError` |
| `path_commit.py` | 良好 | 68 行 1 函数、docstring 比 1.0，函数 docstring 写明返回 None 的条件与两处 `ValueError` |
| `pending_path_commit.py` | 可接受 | 168 行 5 函数、docstring 比 0.8、最长 79 行 |
| `geometry_worker.py` | 可接受 | 513 行 18 函数、docstring 比 0.167、最长 100 行；协议常量（128 KiB/2 MiB/8 s）集中，但多数方法无 docstring |
| `path_handoff.py` | 可接受 | 405 行 7 函数、docstring 比 0.286、最长 76 行；每个函数第一句都声明"不改控制数学" |
| `path_admission.py` | 需改进 | 486 行、docstring 比 0.5、最长函数 214 行（`certify_curve`，de Casteljau 分区 + 凸包球证明全在一个函数里） |
| `physical_gate.py` | 需改进 | 625 行 19 函数、docstring 比 0.316、最长 157 行；`_bind_reference_context`/`retain` 逐字段比对字符串键 |
| `async_admission.py` | 需改进 | 1199 行 31 函数、docstring 比 0.323、4 个 >80 行函数、注释 37 行；队列/传输/证书状态机混在一个类 |
| `static_geometry.py` | 需改进 | 1009 行 19 函数、5 个 >80 行、最长 167 行；`certify_start_connector`/`certify_physical_curve` 各含完整几何推导，注释 18 行偏少 |
| `validate_route_support.py` | 需改进 | 1002 行 27 函数、最长 360 行（`run()`）、docstring 比 0.333；receipt 字段与检查项在同一个巨函数里 |

抽查确认：`path_admission.certify_curve` 覆盖第 191–404 行（214 行单函数）；`validate_route_support.run` 覆盖第 578–938 行（≈360 行）。区分"结构问题"与"文档问题"：本目录主要是**结构问题**（千行模块 + 超长证明函数），纯文档缺口集中在 `geometry_worker`/`path_handoff` 的方法级 docstring 与三个千行文件的模块内导览。

## 重构建议

没有 P0：两道门都是 fail-closed——`FrozenFence.load()` 对缺失字段抛 `ValueError`（不会静默放行），`certify_curve` 的 "inconclusive or out-of-budget rejects" 有代码注释与 `allowed=False` 默认值支撑，未发现可静默通过的正确性缺陷。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `validate_route_support.run()`（360 行）按检查族拆成 `limits/speed/faces/center+union/collision/ray/yaw-domain` 子函数，`run()` 只做 receipt 组装 | 单函数 360 行使任何一条检查的修改都要通读全部 | 中：receipt 字段顺序/示例数上限（`max_report_examples`）会变，下游人工核对依赖字段稳定 | 先固化一份 CLI receipt 对照（`--output` 到 /tmp，比较拆分前后 JSON 键集合与 `status`），再迁移 |
| P1 | 拆开 `static_geometry ↔ validate_route_support` 的 `importlib` 文件加载：把共享的 `limits()/geometry()/make_face()/sha()` 抽到中立模块，两边正常 import；`LEGACY_SHA` 改为显式声明来源 | 现在"import 兄弟 CLI 脚本当库 + import 时 hash 内容"使两份文件互为隐式契约，重命名/重排都会改变证据 | 中：`frozen_geometry_helper_sha256` 是 receipt 字段，改名会改证据字节 | 抽模块后重跑 `tests/regression/test_navigation_safety.py` 与一次 `static_geometry curve` CLI，比较 receipt 除该 sha 外逐字段相同 |
| P1 | 按"队列/传输"与"证书生命周期"拆 `async_admission.py`（1199 行 / 31 函数） | 类同时管 FIFO 预算、短 intake、子进程 Worker 事件与证书状态机 | 中：R88 默认 OFF，但状态转移多、出错即静默降级 | 先给 `queue_snapshot()`/`intake_snapshot()` 输出加纯 CPU 对照测试，再拆 |
| P2 | 拆 `path_admission.certify_curve`（214 行）：区间遍历与"最近 3D 段 + 凸包球"判定分开 | 证明逻辑要能被单独复核 | 中：安全证明，任何拆分都必须零行为变化 | 用既有允许/拒绝样例做前后对照（`test_navigation_safety.py` 覆盖 `certify_physical_curve`，`certify_curve` 需另补样例） |
| P2 | 统一三代 `payload()` 的命名并互相指路；把 `canonical/digest` 合并到一个模块 | 同名函数三份、`canonical` 两份，改协议时易漏 | 低 | `grep -rn "def payload\|def canonical\|def digest" src/` 核对后再改名 |
| P2 | 给 `geometry_worker.Worker.*`、`path_handoff.*`、`physical_gate.*` 补方法级 docstring（输入/失败原因） | 这些函数的契约全在 `ValueError` 字符串里 | 低 | 与 README 失败原因表对照，确认无遗漏 |

本轮只写建议、不改代码。P1 三条都涉及"证据字节或状态机"，动之前必须先有对照（receipt JSON 或 CLI 输出）并通过 `tests/regression/test_navigation_safety.py`。

## 输入输出与调用

输入是 SCAN 候选/BSpline、冻结 anchor 与路线走廊、PCT 折线与 `../../../../configs/planning/static_support_limits.json`；输出 allow/reject 证明与证据字典；通过当前默认 `RuntimeTelemetry` 的历史 JSONL 写入为空操作。主要调用方是 `../../navigation/pid_entrypoint.py`（异步分支）、`../../navigation/pid_support.py`（handoff/commit/pending）、`../../navigation/pid_navigation.py`（选 fence、构造 `R85PhysicalPathGate`）与 `../../control/safety/duplicate_source.py`（只读绑定做最终一致性核对）。注意 `navigation/pid_admission.py` 是准入**生命周期 mixin**，不 import 本目录；它负责写决策归档。

## 测试

`../../../../tests/regression/test_navigation_safety.py` 覆盖 `static_geometry.certify_physical_curve` 的允许/拒绝结果（`skipUnless` 私有测量图 `assets/geometry/metadata.json` 与 `assets/geometry/route.json`；本机数据卷存在时实际执行，实测 `10 passed in 0.18s`）；该文件虽导入 `certify_start_connector`，没有直接调用它的测试；其余准入文件没有专用测试。

## 限制

静态证明不覆盖跟踪误差、摩擦、步态与实机；私有测量图与几何元数据不在 Git。`FrozenFence` 期望的 `frozen_once`、`navigation_scene_axis_registration.json` 等字段没有对应的写入方——`../../navigation/mission/mission_runtime.py` 写出的 anchor 字段集不同，因此 **`FrozenFence` 这条 fallback 分支**在本检出中未闭合、未验证。默认 profile 启用 `r85_auto_reference_contract`，实际准入走 `AutomaticRouteFence` + `R85PhysicalPathGate`（`../../navigation/pid_navigation.py` 构造、`../../navigation/pid_admission_apply.py` 使用），不要把 fallback 的未闭合当成默认路径未闭合。`validate_route_support.py` 的两个 CLI 默认输入文件（`data/known_scene43_camera_init_h005/metadata.json`、`support_validation_config.json`）未随检出提供；`geometry_worker` 的 R93 probe 需要显式绝对路径 `DEMO_RUN_DIR`。
