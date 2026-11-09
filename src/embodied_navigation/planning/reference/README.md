# planning/reference

本目录是运行时的参考引导层：把注册路线或 PCT 折线转成方向、速度预算、投影与路线后缀，供控制与准入使用；它只提供先验，不授予运动或到达。

## 边界

- 做：规划 guide（`global_route_reference.py`）、几何方向与速度预算（`spatial_reference.py`）、一次性全局参考与局部重规划 wire（`automatic_reference_runtime.py`）、从实测位姿选不可变路线后缀（`route_suffix.py`）、冻结先验的 3D 走廊（`route_fence.py`）。
- 不做：不做碰撞/支撑证明，不替代 SCAN 实测样本、native guard、区域与截止时间契约；`reference` 给出的弦向只是航向估计，不是可执行路径。

## 目录结构

```text
src/embodied_navigation/planning/reference/
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），包不导出 API
├── automatic_reference_runtime.py  # 491 行，R85 一次性参考 + scoped replan + AutomaticRouteFence
├── global_route_reference.py  # 244 行，mission-leg 规划 guide 与方向一致性审核
├── route_fence.py  # 53 行，冻结先验的多段 3D 走廊证据
├── route_suffix.py  # 136 行，从实测位姿选不可变 PCT 后缀
└── spatial_reference.py  # 284 行，折线上的方向/曲率与速度预算
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（活库） | `spatial_reference.py` | 被 `control/tracking/cascade_core.py` 直接 import：`reference()` 给出方向与 normal/cross 误差、`speed_budget()` 收紧速度；`configuration()` 拒绝未知键，`DEFAULTS` 即文档 |
| 代码（活库） | `global_route_reference.py` | `contract()` 解析并边界校验 profile；`SegmentGuides` 按目标身份冻结"实测激活点→原始目标"的规划 guide（间距 1.0 m、最多 64 点）；`StableProjectionController(Controller)` 继承控制层基类，让预览与实际控制共用同一条 checked-curve 投影；`direction_check` 在 0.8 m 弧长上审核方向一致性（上限 0.2 rad，`fixed_goal_tail` 例外） |
| 代码（活库） | `automatic_reference_runtime.py` | 读 run 内 `global_reference.json`，校验后**只发一次**全局参考；之后只发 scoped local replan 请求。`AutomaticRouteFence` 是给准入用的薄访问器 |
| 代码（活库） | `route_suffix.py`、`route_fence.py` | 纯函数；`select_route_suffix` 用最近 3D 段投影选不可变路线后缀（resume 用），不搜索新路线、不改起点/终点；`tube_evidence` 给相对冻结的 sensor-registered 先验的水平 0.45 m、高度 0.3 m 走廊证据 |
| 代码（空） | `__init__.py` | 0 字节；无重导出，import 必须写全路径 |
| 日志/输出 | 本目录**不写任何文件** | `grep -n "write_text\|open(" reference/*.py` 无命中；唯一的磁盘动作是读 `run/global_reference.json`（`load()`）与 `digest_file()` 读文件算 SHA |
| 日志/输出（声明但默认不落盘） | `runs/<run_id>/r85_actual_start_connectors.jsonl`、`r85_reference_publications.jsonl`、`r85_local_replan_requests.jsonl`、`r85_reference_protocol_authorizations.jsonl` | 4 处 `owner.evidence.append(...)`；`owner.evidence` 当前是 `RuntimeTelemetry`，其 `append/enqueue` 直接 `return None`（`common/runtime_telemetry.py`），所以默认运行不产生这些文件。写入调用点仍在，换成可写记录器即可生效 |
| 配置 | 无直接 `configs/` 读取点 | `reference/` 里没有任何 `configs/...` 路径字符串；所有配置经 profile 传入（见下表） |

配置键（全部经 profile，profile 由 `launch/prepare.py` 合并 `configs/planning/settings.json` 等并写入 `runtime_context.json`/`navigation_profile.json`）：

| 键 | 消费者 | 当前值 / 语义 |
| --- | --- | --- |
| `r85_auto_reference_contract.enabled` | `automatic_reference_runtime.enabled` | `true`；同时强制 `body_ground_offset_m=0.32`、`scan_transport_offset_m=0.4`、`max_reference_points=4096`，不匹配即 `ValueError` |
| `r85_auto_reference_contract.reference_file` | `AutomaticReferenceRuntime.__init__` | 类内缺省 `r85_global_reference.json`，但 `launch/prepare.py` 已 **pin 成 `global_reference.json`**（实际写该文件的是 `navigation/mission/mission_runtime.py`） |
| `r85_auto_reference_contract.resume_from_actual_route_projection` | `request_plan` | `true` 时才调 `route_suffix.select_route_suffix`；值必须是 bool |
| `global_route_reference_contract` | `global_route_reference.contract` | `enabled=true`、`guide_spacing_m=1.0`、`guide_start_ahead_m=0.35`、`direction_audit_arc_m=0.8`、`max_guide_points=64`；启用时做区间校验（间距 0.2–2.0、审核弧 0.2–1.2、点数 2–128） |
| `registered_route_fence` | `AutomaticRouteFence.fence()` 返回、`robot/unitree/bridge.py` 传给 `tube_evidence` | `horizontal_radius_m=0.45`、`height_error_m=0.3`；`route_fence.py` 自身只用参数默认值，真正传值的是调用方 |
| `cascade.spatial_reference`（`configs/control/settings.json`） | `control/tracking/cascade_core.py` → `spatial_reference.configuration(...)` | 弧长 0.2 m、`yaw_rate_reserve_radps=0.12`、`yaw_acceleration_reserve_radps2=0.2`、`yaw_command_slew_radps2=0.8`、弦偏差上限 0.03 m |

`automatic_reference_runtime.load(planner_epoch, generation)` 的校验（读代码核实）：文件 ≤2 MB、schema 必须是 `r85_pct_scan_global_reference/v1`、`run_id` 匹配、`frame_id=camera_init`、epoch/generation 匹配、`reference_id` 非空、`physical_body_ground_offset_m=0.32` 且 `scan_transport_offset_m=0.4`、`navigation_ground_truth_used=false`、body/ground 点数 2–4096 且等长、相邻点不重合、逐点 body−ground 仅在 Z 上差 0.32、最后调 `terminal_heading_contract.validate_reference_heading`。

## 模块内文件耦合

**方向结论**：单向为主——`spatial_reference` 是叶子，`global_route_reference → spatial_reference`，`automatic_reference_runtime → route_suffix`，`route_fence` 完全独立。但 `automatic_reference_runtime` 还 import 兄弟子包的 `admission.replan_ack`，而 `admission/static_geometry.py` 又 import 本目录的 `route_suffix`：两个子包**互为上游**，目前不成环只因两边都只取叶子模块，且 `admission.static_geometry` 在 `_physical_connector` 里是**延迟 import**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `spatial_reference.py` | 284 | 无内部依赖 | `control/tracking/cascade_core.py`、`navigation/pid_ingress.py`、`global_route_reference.py` | 清楚（纯几何 + 纯预算） |
| `global_route_reference.py` | 244 | `spatial_reference.point_at`、`control/tracking/cascade_core.Controller` | `navigation/mission/continuous_route.py`、`navigation/pid_support.py`、`admission/path_handoff.py` | 可接受：继承控制层基类使本层依赖控制层 |
| `route_suffix.py` | 136 | 无内部依赖 | `automatic_reference_runtime.py`、`admission/static_geometry.py` | 清楚 |
| `route_fence.py` | 53 | 无内部依赖 | `robot/unitree/bridge.py` | 清楚 |
| `automatic_reference_runtime.py` | 491 | `route_suffix`、`admission/replan_ack`、`scan_reference_protocol`、`common/observation`、`control/parking/terminal_heading_contract`；运行期 importlib 加载 `admission.static_geometry` | `navigation/base_node.py`、`pid_admission_apply.py`、`pid_navigation.py`、`planning/scan_epoch_supervisor.py` | 偏重（18 函数、docstring 比 0.056，`request_plan` 151 行） |
| `__init__.py` | 0 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：无 >800 行文件；最大的 `automatic_reference_runtime.py` 491 行。
- **重复实现**：`StableProjectionController._project`（`global_route_reference.py`）几乎逐行复制了父类 `Controller._project`（`control/tracking/cascade_core.py`）：两者返回同一个 7 元组，唯一实质差异是弧长下界（`self.progress` vs `max(0, self.progress - 0.15)`）。两份投影靠覆写保持同源，改一侧不会自动同步。
- **跨层直达**：①`global_route_reference.StableProjectionController` 继承 `control.tracking.cascade_core.Controller`——规划参考层直接依赖控制层基类；②`automatic_reference_runtime` 通过 `owner` 鸭子类型访问 `owner.reference_pub`、`owner.known_scene_scan_epoch`、`owner.evidence`、`owner.pose` 等十余个属性，无类型声明；③对 `admission.static_geometry` 用 `importlib.import_module` 延迟导入。
- **隐式契约**：参考文档与绑定全部是字符串 schema（`r85_pct_scan_global_reference/v1`、`r85_automatic_route_fence_binding/v1`、`mission_segment_planning_guides/v1`、`checked_scan_direction_coherence/v1`）；`request_plan` 依赖 `owner` 属性名与调用顺序（publish → association 注册 → evidence），顺序错了不会报错但证据缺失。

结论：耦合**方向清楚、整体合适**（叶子模块独立、无循环 import），但存在两条需要读代码才能发现的跨层连接：`StableProjectionController` 继承控制层基类，以及 `reference ↔ admission` 的互为上游（靠延迟 import 兜住）。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `route_fence.py` | 良好 | 53 行 1 函数、docstring 比 1.0，函数 docstring 给出半径/高度参数含义与"只是证据"的边界；退化段跳过、非有限值 `ValueError` |
| `route_suffix.py` | 可接受 | 136 行 3 函数、docstring 比 0.333，但 `select_route_suffix` 的 docstring 写清了"相邻并列取前边、非相邻并列报错"的歧义规则 |
| `global_route_reference.py` | 可接受 | 244 行 8 函数、docstring 比 0.25、最长 54 行；`contract()` 的边界校验与 `SegmentGuides` 的注释解释了"helper 合并仅规划用" |
| `spatial_reference.py` | 可接受 | 284 行 6 函数、docstring 比 0.5、最长 125 行（`speed_budget`）；预算不等式在 docstring 里写成公式，`DEFAULTS` 即参数目录 |
| `automatic_reference_runtime.py` | 需改进 | 491 行 18 函数、docstring 比 **0.056**、最长 151 行；`request_plan` 一个函数同时做 scope 复核、后缀选择、连接器证书、publish、association 注册、4 类 evidence 与本地重规划请求 |

抽查确认：`spatial_reference.speed_budget` 从第 160 行到第 284 行，确为 125 行；`automatic_reference_runtime.request_plan` 从第 292 行到第 442 行，确为 151 行。这里以**文档问题为主**（`automatic_reference_runtime` 的方法几乎无 docstring，且 `request_plan` 的状态前置条件只写在行内注释），结构问题仅"单函数过长"一项。

## 重构建议

没有 P0/P1：本目录最大文件 491 行、无写盘、无控制数学，耦合无环，`spatial_reference`/`route_fence` 都有清晰的纯函数边界；改动收益主要在文档与可测性。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `automatic_reference_runtime` 的 8 个方法补 docstring（`load` 的校验清单、`request_plan` 的前置状态、`fence` 的绑定字段） | 该文件 docstring 比 0.056，而它承载参考发布与重规划 wire 的完整契约 | 低：只加注释 | 与 `docs/glossary.md` 的 provenance 术语对照，确认措辞一致 |
| P2 | 把 `request_plan`（151 行）拆成 `validate_scope` / `select_suffix` / `certify_connector` / `publish_once` / `request_local_replan` 五段 | 一个函数混了五种职责，失败原因难定位 | 中：`owner` 鸭子契约与 publish/注册顺序是隐式契约，拆分不能改顺序 | 先加"顺序不变"的纯 CPU 测试（mock owner），再迁移；对比 `evidence.append` 调用序列 |
| P2 | 删除或标注未使用的 `reference_version_paths()` | 全仓 grep 只有定义、没有调用点；返回的两个文件名（`r85_global_reference_epoch_<e>_generation_<g>.json`、`r85_reference_registration_...json`）当前无人写 | 低（若确实是死代码） | `grep -rn "reference_version_paths"` 复核；若历史脚本在用则改为文档说明 |
| P2 | 在模块 docstring 里显式写出 `reference ↔ admission` 的依赖方向与"延迟 import 防环"的理由 | 现在只有读 import 才能发现两子包互为上游 | 低 | 与 `planning/admission/README.md` 的耦合节交叉对照 |

本轮只写建议、不改代码；P2 中的 `request_plan` 拆分必须先有 mock owner 测试，因为其正确性依赖调用顺序而非返回值。

## 输入输出与调用

输入是 `global_reference.json`（`navigation/mission/mission_runtime.py` 写、schema `r85_pct_scan_global_reference/v1`）、anchor、checked SCAN 曲线与实测位姿；输出参考文档、guides、方向审核、速度预算与路线后缀 receipt。

实际 import 点（grep 核实，按文件）：

- `automatic_reference_runtime`：`navigation/base_node.py`、`navigation/pid_admission_apply.py`、`navigation/pid_navigation.py`（选 `AutomaticRouteFence`）、`planning/scan_epoch_supervisor.py`（判 R85 是否启用）。
- `global_route_reference`：`navigation/pid_support.py`（重导出给 admission 用）、`navigation/mission/continuous_route.py`、`planning/admission/path_handoff.py`。
- `spatial_reference`：`control/tracking/cascade_core.py`、`navigation/pid_ingress.py`。
- `route_fence`：`robot/unitree/bridge.py`（拿 `registered_route_fence` 配置调 `tube_evidence`）。
- `route_suffix`：`automatic_reference_runtime.py`、`planning/admission/static_geometry.py`。

注意：旧文本写的"`pid_admission_apply.py`/`pid_route.py`"不准确——`navigation/pid_route.py` 只 import `pid_support`，不直接 import 本目录（grep `planning.reference` 无命中）。

## 测试

本子包无专用测试，逻辑通过导航链路使用；改动需自行补测。测试分层与命令见 [tests/README.md](../../../../tests/README.md)。

## 限制

guide 只是规划先验；`global_reference.json` 由 `../../navigation/mission/mission_runtime.py` 生成并依赖仓库根数据入口 `assets/geometry/route.json` 数据（外部数据卷，不入 Git），完整路线未验证。`route_fence` 的 0.45 m/0.3 m 只是证据阈值，不构成碰撞证明；`AutomaticReferenceRuntime` 只在 R85 开启（默认 profile 的 `r85_auto_reference_contract.enabled=true`）时构造，关闭时本层不参与参考发布。
