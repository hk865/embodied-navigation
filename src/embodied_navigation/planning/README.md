# planning

本目录是规划侧：SCAN 规划器子进程的拥有与重启、SCAN 输出与所发参考之间的 provenance 契约，以及路径准入、物理几何证明与 PCT 参考路线生成。

## 边界

- 做：拥有一个 frozen SCAN planner 子进程（`scan_epoch_supervisor.py`）；校验子进程启动、真实消费与发布 ACK（`child_startup.py`、`scan_child_consumption.py`、`scan_reference_protocol.py`）；把 SCAN Bspline 与当前参考绑定（`trajectory_contract.py`、`reference_protocol.py`）；生成 bounded SCAN 参数（`scan_runtime_parameters.py`）；纯 polyline 前瞻（`route.py`）。
- 不做：不执行控制、不判定到达；规划输出只是候选，运动许可仍由控制/导航持门禁。
- 不含算法本体：SCAN 与 PCT 是 `third_party/` 下的 vendored 源码（见 [third_party/README.md](../../../third_party/README.md)），本仓库只做接入、调度与消息交互，不把其算法写成自有实现。

## 目录结构

```text
src/embodied_navigation/planning/
├── admission/  # 路径准入与静态几何证明，见 admission/README.md
├── launch/  # SCAN 进程组 launch 装配，见 launch/README.md
├── pct/  # PCT 参考路线生成器（离线脚本），见 pct/README.md
├── reference/  # 运行时参考引导与路线后缀，见 reference/README.md
├── README.md  # 本文件
├── __init__.py  # 1 行，只重导出 route.lookahead_target
├── child_startup.py  # 190 行，SCAN 子进程启动 ACK（5 s 预算内）
├── reference_protocol.py  # 890 行，R85 候选↔所发参考↔receipt 关联（schema 3）
├── route.py  # 20 行，折线投影 + 固定前瞻距离
├── scan_child_consumption.py  # 198 行，子进程真实消费 odometry 的证据
├── scan_epoch_supervisor.py  # 657 行，唯一拥有/重启 SCAN 子进程的进程入口
├── scan_reference_protocol.py  # 268 行，子进程成功发布参考的 ACK 校验
├── scan_runtime_parameters.py  # 51 行，SCAN 参数包络（prepare 期写 yaml）
└── trajectory_contract.py  # 106 行，schema 1 Bspline/metadata provenance
```

`scan_epoch_supervisor.py` 是**进程入口**（`python -m embodied_navigation.planning.scan_epoch_supervisor`，由 `launch/components.py` 的 `owned()` 拉起），除 `main()` 外没有可复用库入口；其余文件都是可 import 的库。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（库，无 ROS） | `route.py` | 单函数 `lookahead_target`，20 行，唯一被单测直接覆盖的顶层文件 |
| 代码（库，无 ROS） | `trajectory_contract.py` | schema 1 老协议：`TrajectoryAssociation` 用有界 OrderedDict 缓存 Bspline/metadata，要求两者逐字段相等且 `traj_id` 前进；只证 provenance，不证碰撞自由或路线进度 |
| 代码（库，无 ROS） | `reference_protocol.py` | R85 schema 3 协议：`R85TrajectoryAssociation` 关联 candidates、exact sent reference 与 optimizer receipt，**不是执行 ACK**；自带 `_canonical/_digest` 等归一化工具 |
| 代码（库，无 ROS） | `scan_child_consumption.py` | 定义 ACK 话题 `/demo/navigation/scan_child_odometry_consumed` 与 `ChildConsumption`（16 行窗口） |
| 代码（库，无 ROS） | `child_startup.py`、`scan_reference_protocol.py` | 启动/发布阶段的纯校验；依赖 `scan_child_consumption` 的证据结构 |
| 代码（进程） | `scan_epoch_supervisor.py` | 唯一常驻进程：订阅三路同戳输入、管理唯一 SCAN 子进程、发布 epoch-ready |
| 代码（prepare 期） | `scan_runtime_parameters.py` | `parameters(profile)` 返回 SCAN 参数字典，由 `launch/prepare.py` 写成 `runs/<run_id>/v34_scan_overrides.yaml` |
| 代码（重导出） | `__init__.py` | 只有 `from .route import lookahead_target` |
| 日志/输出 | `runs/<run_id>/scan_epoch_history.jsonl` | supervisor 用 `open("x")` 独占创建（文件已存在则启动报错）；每行 JSON 固定含 `event`、`monotonic_wall`、`ros_clock_ns`、`epoch`、`planner_epoch`、`localization_generation`，再由 `record(kind, **data)` 追加事件字段。**仓库内没有代码消费者**（grep 只命中本文件），是证据/诊断产物 |
| 日志/输出 | `runs/<run_id>/scan_epoch_ready.json` | 每次 `publish()` 先写 `scan_epoch_ready.tmp` 再 `replace` 原子替换；内容 schema `known_scene_SCAN_epoch_ready/v1`，与发到 `/demo/navigation/scan_epoch_ready` 的 String 相同。仓库内同样没有代码消费者 |
| 日志/输出（ROS） | 话题 | 发布 `/demo/navigation/scan_epoch_ready`；订阅 `/demo/navigation/localization_status`、`/demo/navigation/localization_bundle`、`/demo/navigation/scan_body_odom`、`/demo/navigation/cloud`，R85 时另订 `/demo/navigation/scan_child_odometry_consumed`。子进程侧话题由 `REMAPPINGS` 统一重映射到 `/demo/navigation/*` |
| 配置 | 无直接 `configs/` 读取点 | 没有代码读 `configs/planning/settings.json`；参数以 profile 传入。`scan_epoch_supervisor.py` 在 `main()` 里读 run 目录的 `runtime_context.json`（取 `profile`）与 `navigation_profile.json`（判 R85 是否启用），这是**消费 prepare 产物**而非读 configs |

`scan_epoch_supervisor` 的 ready 判据（读代码核实）：

- 只 signal 自己保存的子进程 PID；`identity()` 记录 `pid/ppid/pgid/start_ticks/exe` 五项，启动前/重启时逐项比对。
- 每次 navigation frame 事务结束（generation 变化）就 `stop_child` + 重新 `start_child`，让新进程重建 grid。
- `localization_bundle` metadata、`scan_body_odom`、`cloud` 必须共享同一 `source_ns`，三路 receipt 各自的接收龄 **<0.3 s**、header 龄 **<300 ms**（下界 −50 ms）；旧文档写的"两路输入"不准确（代码 `receive()` 里要求 `metadata/odom/cloud` 三个 role，以此处为准）。
- 还需要新 planner 进程、graph 上出现 `scan_planner_node` 的两个订阅，以及（R85 时）子进程 odometry 消费 ACK。

`scan_runtime_parameters.parameters(profile)` 的读取键（代码核实）：`continuous_route_contract.local_planning_horizon_m`（不等于 `0.65` 直接 `ValueError`）、`r85_session.enabled`（必须是显式 bool）、`max_speed_mps`，以及 observation 开关 `r74_protocol_observation_python`。固定值包括 `fsm.planning_horizon=0.65 m`、`grid_map.resolution=0.08`、滑动地图 `10×10×5 m`、`fsm.navi_mode=3`；R85 只额外追加 `fsm.external_reference_protocol=true`。配置读取顺序与 prepare pin 见 [configs/README.md](../../../configs/README.md)。

## 模块内文件耦合

**方向结论**：无环，分两层——底层是无 ROS 的纯库（`route` / `trajectory_contract` / `reference_protocol` / `scan_child_consumption`），上层是进程（`scan_epoch_supervisor`）与校验库（`child_startup` / `scan_reference_protocol`）。子包（`admission`、`reference`）反向 import 顶层模块（如 `admission/async_admission.py` → `reference_protocol`、`reference/automatic_reference_runtime.py` → `scan_reference_protocol`），方向统一为"子包 → 父目录"，没有父目录 import 子包的循环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `route.py` | 20 | 无内部依赖 | `planning/__init__.py`、`control/relative_pid.py`、单测 | 清楚 |
| `trajectory_contract.py` | 106 | 无内部依赖 | `navigation/base_node.py` | 清楚 |
| `reference_protocol.py` | 890 | 无内部依赖 | `navigation/base_node.py`、`pid_admission_apply.py`、`admission/async_admission.py` | 偏重（被三处消费，自身 890 行） |
| `scan_child_consumption.py` | 198 | 无内部依赖 | `child_startup`、`scan_reference_protocol`、supervisor | 清楚 |
| `child_startup.py` | 190 | `scan_child_consumption` | supervisor | 单函数 174 行，偏重 |
| `scan_reference_protocol.py` | 268 | `scan_child_consumption` | `reference/automatic_reference_runtime.py`、supervisor | 偏重（`verified_revision` 127 行） |
| `scan_epoch_supervisor.py` | 657 | `child_startup`、`scan_child_consumption`、`scan_reference_protocol`、`reference/automatic_reference_runtime`、`localization_mapping/known_scene_matcher`、`common/input_freshness`、ROS | 无（进程入口） | 偏重（`main()` 597 行，内含 `Supervisor(Node)` 类） |
| `scan_runtime_parameters.py` | 51 | `common/observation` | `launch/prepare.py` | 清楚 |
| `__init__.py` | 1 | `route` | 无 | 清楚 |

四类问题：

- **上帝文件**：`reference_protocol.py` 890 行（>800 阈值）；`scan_epoch_supervisor.py` 657 行但 `main()` 单函数 597 行，属"函数级上帝入口"。
- **重复实现**：`payload()` / `key()` 在 `trajectory_contract.py`（schema 1）与 `reference_protocol.py`（schema 3）各有一份，`admission/path_admission.py` 还有第三份 `payload()`。三份对应三代消息协议，是有意并存，但函数同名、无交叉说明，容易改错。
- **跨层直达**：supervisor 直接依赖 `localization_mapping/known_scene_matcher`（跨模块）与 `reference/automatic_reference_runtime`（子包）；这属于进程编排层的合理取用，不是绕过契约读共享 dict。
- **隐式契约**：JSONL 事件字段靠 `record()` 手工拼 `**data`；跨进程契约全是字符串 schema（`known_scene_SCAN_epoch_ready/v1`、`r85_*`）与 topic 名字符串，没有集中常量表。

结论：耦合**方向清楚、基本合适**——纯库→校验库→进程的单向链，唯一实质负担是 `reference_protocol.py` 体量与 supervisor 的巨型 `main()`；重复的 `payload/key` 是协议代际并存，需要在文档而非代码里区分。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `route.py` | 良好 | 20 行 1 函数，docstring 覆盖 1.0，含退化段显式 `ValueError`，被 `tests/unit/test_control.py` 覆盖 |
| `scan_child_consumption.py` | 良好 | 198 行 7 函数，docstring 比 0.857，最长函数 55 行，常量集中（`TOPIC`/`SCHEMA`/`GID_BYTES`） |
| `trajectory_contract.py` | 可接受 | 106 行 10 函数、最长 21 行，函数级 docstring 比 0.0，但模块 docstring 与行内注释交代了"只证 provenance"的理由；命名清楚 |
| `scan_runtime_parameters.py` | 可接受 | 51 行单函数、无函数 docstring，但返回字典是一张可对照的参数表，越界值立即报错 |
| `child_startup.py` | 需改进 | 190 行里 `await_startup` 占 174 行，参数 15 个（其中 6 个 keyword-only 注入点），注释仅 2 行 |
| `reference_protocol.py` | 需改进 | 890 行 37 函数、docstring 比 0.405、注释 13 行（约每 68 行一条）；`_curve`/`_trajectory`/`_validate_metadata` 全是逐字段字符串键校验 |
| `scan_reference_protocol.py` | 需改进 | 268 行里 `publication_eligible` 110 行、`verified_revision` 127 行，主流程藏在层层 `if ... raise ValueError` 里 |
| `scan_epoch_supervisor.py` | 需改进 | 657 行、函数 docstring 比 0.0、4 个 >80 行函数、`main()` 597 行；`Supervisor(Node)` 类定义在函数体内，缩进 8 格的嵌套类难以局部阅读 |

抽查确认：`child_startup.await_startup` 从第 17 行到文件末（190 行），确有 174 行；`scan_epoch_supervisor.main` 从第 57 行起，确为单函数主体。指标可信。结构问题集中在"超长校验函数"和"进程入口巨型 main"，不是文档缺失。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `scan_epoch_supervisor.main()` 的 `Supervisor(Node)` 类提到模块级，`main()` 只留 argparse + 构造 + `rclpy.spin` | 597 行单函数使任何改动都要通读全文件；类本身已有清晰方法边界 | 中：进程内行为依赖闭包变量（`run`/`binary`/`digest`/`profile` 等在 `main()` 里），提类需改成构造参数 | 先补/保留"启动子进程 + 发布 ready"的集成路径对照；改后逐条比对 `scan_epoch_history.jsonl` 事件序列 |
| P1 | 按域拆 `reference_protocol.py`：归一化工具（`_canonical/_digest/_integer/...`）、候选校验（`_curve/_trajectory`）、`R85TrajectoryAssociation` 状态机三块 | 890 行跨两个抽象层，定位一个字段校验要滚 ~600 行 | 中：它被 3 处 import，且与 SCAN 消息 schema 强绑定 | 先固化"接受/拒绝各样例"的纯 CPU 对照测试，再按拆分前后结果一致迁移 |
| P2 | 统一 `payload()`/`key()` 的命名（如 `payload_schema1` / `payload_schema3`）并在模块 docstring 互相指路 | 三份同名函数代表三代协议，现状只能靠读 import 路径区分 | 低：纯重命名，import 点少 | `grep -n "import payload"` 核对全部调用点；跑 `tests/unit/test_control.py` 与 `test_navigation_safety.py` |
| P2 | 给 `await_startup`、`publication_eligible`、`verified_revision` 补"输入契约/失败原因表"docstring | 这几个函数 110–174 行，参数与失败字符串就是协议文档 | 低：只加注释 | 与 `scan_epoch_history.jsonl` 的 `event` 名对照，确认无遗漏 |
| P2 | 把 `main()` 里读 run JSON 的两处（`runtime_context.json`、`navigation_profile.json`）抽成 `load_run_profiles(run)` | 进程入口同时承担进程编排与 profile 解析，单测难覆盖 | 低 | 新增纯函数单测：给定 run 目录解析出两个 profile |

本轮只写建议、不改代码。所有 P0/P1 结论都建立在"读码 + 纯 CPU 测试通过"之上：没有 P0（未发现正确性/安全缺陷）；P1 仅两项，都是体量与入口结构问题，动之前必须先固定行为对照。

## 输入输出与调用

输入是 canonical SLAM 位姿/点云、route/request 与既有 SCAN 配置；输出是 SCAN 轨迹（Bspline/metadata）、准入证明与参考 JSON。SCAN 算法本体在 [third_party/README.md](../../../third_party/README.md) 所列的包中，由 [launch/scan.launch.py](launch/scan.launch.py) 启动的 supervisor 拉起仓库根 `install/ros` 中的 `scan_planner_node`，当前 launch 传入的配置是 `<ros_prefix>/share/scan_planner/config/planner.yaml`（`ros_prefix` 见 `configs/launch/settings.json` 的 `runtime_paths.ros_prefix`，默认 `install/ros`）；[third_party/scan/plan_manage/config/planner.yaml](../../../third_party/scan/plan_manage/config/planner.yaml) 是源码树中的配置。

## 测试

`route.py` 由 [tests/unit/test_control.py](../../../tests/unit/test_control.py) 覆盖（退化段显式报错，正常段参与 PID 等价对比）；其余顶层文件没有专用单测。测试分层与命令见 [tests/README.md](../../../tests/README.md)。

## 限制

仓库根的 `install/` 缺 SCAN 二进制只表示本机未构建；完整路线、跨楼层与实机未验证。supervisor 的 `scan_epoch_history.jsonl`/`scan_epoch_ready.json` 目前只写不读，属证据产物而非契约输入。
