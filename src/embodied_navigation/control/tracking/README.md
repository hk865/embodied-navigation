# control/tracking

本目录主要实现跟踪控制计算：把 SLAM 位姿、IMU 与 SCAN 轨迹换算成有界机体速度，并完成航向门、地形检查等几何判断；另含策略源码与模型路径选择适配。

## 边界

- 做：`cascade_core.py` 的 actual-SLAM/IMU/SCAN cascade；`control_core.py` 的轨迹跟随、姿态倾角、航向门、加速度限幅、障碍走廊判断；`pid_core.py` 的角度归一化与 slew 限幅；以及规划上坡、路线进度、楼梯限速等辅助判断。
- 控制计算不发布 ROS 或执行策略动作，也不负责任务状态机；节点协作见 [navigation/README.md](../../navigation/README.md)。
- 本包并非所有函数都无 IO：`policy_selection.py` 会检查文件存在性并通过 `common.config` 读取本地配置；`cascade_core.py` 本身不读文件、不写文件。
- 本目录不决定"何时停车"，只按调用方给的模式（`drive`/`turn`/`brake`/`hold`/`capture`）算命令；停车判定在 [parking/README.md](../parking/README.md)，触发时机在 `../../navigation/`。

## 目录结构

```text
src/embodied_navigation/control/tracking/
├── README.md               # 本文件
├── __init__.py             # 空包声明
├── cascade_core.py         # 1850 行，actual-SLAM/IMU/SCAN cascade 核心（Controller）
├── control_core.py         # 407 行，轨迹跟随/倾角/航向门/障碍走廊等共享几何策略
├── pid_core.py             # 26 行，角度归一化与 slew 限幅
├── planned_step_filter.py  # 74 行，只在原始 body 障碍门里解释计划上坡/地面
├── policy_selection.py     # 70 行，策略源码与模型路径选择（读配置与文件存在性）
├── replan_policy.py        # 26 行，区分保护性零命令与 SCAN 几何耗尽
├── route_progress.py       # 127 行，连续 3D 折线上的单调弧长进度
└── terrain_speed.py        # 20 行，计划楼梯几何附近的巡航速度压制
```

实测合计：9 个 Python 文件 2600 行；最大文件 `cascade_core.py` 1850 行，占本子包 71%。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `cascade_core.py` | 活代码；`Controller` 是现役控制核心，`planning/reference/global_route_reference.py` 与 `navigation/mission/continuous_route.py` 都**继承**它（`StableProjectionController(Controller)`、`CausalHandoffProjectionController(StableProjectionController)`） |
| 代码 | `control_core.py` | 活代码；`follow_trajectory`、`RawImuTilt`、`HeadingGate`、`obstacle_ahead` 被 `navigation/base_node.py`、`pid_support.py`、`ros_node.py` 直接调用 |
| 代码 | `pid_core.py`、`replan_policy.py`、`route_progress.py`、`terrain_speed.py`、`planned_step_filter.py` | 活代码，小工具/几何判断，调用点见耦合表 |
| 代码 | `policy_selection.py` | 活代码，但不是控制律：解析 Student worker 源码路径、模型路径并读 `common.config`；被 `robot/unitree/policy/worker.py` 与 `simulation/truth_worker.py` 使用 |
| 代码 | `__init__.py` | 空文件，无逻辑 |

**日志/输出**

- 本目录 9 个文件都不写盘（`grep -n "write_text\|open("` 在 `control/tracking/*.py` 无命中）。事实包里 `cascade_core.py` 的 `file_write=1` 是文本命中（`sha`/`evidence` 一类字样），不是真实写文件调用。
- `cascade_core.Controller` 把每帧诊断放在返回值和 `self.row`（`schema` 未定义，`kind="controller_update"`，字段含 `feedback_pose_stamp_ns`、`feedback_received_wall_ns`、`imu_stamp_ns`、`ack_stamp_ns`、`header_dt_s`、`source_expected_hz`、`integral_dt_s`、`error_cross_m`、`error_yaw_rad` 等）里，由外层 `navigation/pid_control.py` 决定是否写入 run 目录。
- `policy_selection.py` 只做只读检查：`selection()` 检查 `robot/unitree/policy/native/worker.py` 存在，`student_sources()` 检查 `student.py`、配置目录与 `path("student_model")` 存在，缺失时 `FileNotFoundError`。

**配置**

| 键（`configs/control/settings.json` 的 `cascade.*`） | 读取点 | 说明 |
| --- | --- | --- |
| `desired_speed` | `Controller.__init__` | 巡航速度，只接受 `(0, 1.0]` |
| `command_limits` | `Controller.__init__` | 命令界 `[0.6, 0.2, 0.6]`；要求每轴 >0 且不超过 `[1.0, 0.4, 1.0]` |
| `base_com_offset` | `Controller.__init__` | 3 维 COM 偏置，模长 >0.2 m 直接 raise |
| `maximum_gyro_pair_gap_s` | `Controller.__init__` | 默认 0.02 s，只接受 `(0, 0.05]` |
| `goal_z_tolerance_m` | `Controller.__init__` | 默认 0.2 m，只接受 `(0, 0.3]` |
| `gains` | `Controller.__init__` | **不接受调参**：只要不等于代码内 `DEFAULT_GAINS` 就 raise（"This candidate preserves the verified cascade gains"） |
| `external_heading_gate`、`r85_student_lower_loop_ack`、`spatial_reference`、`r85_turn_brake_reference` | `Controller.__init__` | 决定航向门归属、下层 ACK 契约与空间参考 |
| `feedback_expected_hz` | `Controller.update` 末尾 | 只写进诊断行 `source_expected_hz`，默认 10；不参与判据 |
| `r85_neutral_rate_brake_capture`、`r85_pre_zero_integral_unload` 等 | 延迟 import 的扩展构造/校验 | 形状严格校验，非法组合 raise |

不在本目录读取的键（易误读，已 `grep` 核实）：

- `cascade.command_slew_per_s`（`[0.6, 0.6, 0.8]`）在 `control/` 内**无读取点**；只在 `../../robot/unitree/policy/speed_tiers.py` 作为一致性校验（不等于 `[0.6, 0.6, 0.8]` 就 raise），实际 slew 由下游 worker 用同名常量执行（`../../robot/unitree/policy/native/worker.py:137` 的 `command_slew_acceleration=[0.6, 0.6, 0.8]`）。
- `cascade.feedback_ttl_sim_and_wall_s` 无读取点：`Controller.__init__` 里 `self.ttl = 0.3` 是硬编码，且 `self.ttl` 之后**从未被读取**；实际的 300 ms 新鲜度检查是函数内字面量（如 `0 <= clock - stamp <= 300_000_000`）。
- `cascade.actual_navigation_verified`、`cascade.controller_source_file`、`cascade.navigation_ground_truth_used` 无读取点（`navigation_ground_truth_used` 在别处按字段名出现，但不是读这个键）。

## 模块内文件耦合

依赖方向（包内）：`cascade_core → control_core`？——**否**，两者互不 import；`planned_step_filter → route_progress` 是唯一的包内 import 边（相对 import `.route_progress`）。`pid_core`、`replan_policy`、`terrain_speed`、`policy_selection` 是叶子。

跨包方向：`cascade_core.py` 模块级依赖 `../parking/ordinary_parking`、`../parking/parking_zero_contract`、`planning.reference.spatial_reference`，并延迟依赖 `../parking/{integral_unload,parking_pose10hz,terminal_reference}`、`robot/unitree/policy/lower_loop_ack`、`common.observation`。**`tracking` 是本 scope 里唯一模块级依赖 `parking` 状态机的子包**，反向边（`parking/terminal_owner`、`parking/transition_gate` → `cascade_core`）是延迟 import，所以包级双向但无导入期环（control 下 39 个非 launch 模块均可单独 import）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `cascade_core.py` | 1850 | `../parking/{ordinary_parking,parking_zero_contract}`、`planning.reference.spatial_reference`（模块级）；`../parking/{integral_unload,parking_pose10hz,terminal_reference}`、`robot/unitree/policy/lower_loop_ack`、`common.observation`（延迟） | `planning/reference/global_route_reference.py`（子类）、`navigation/mission/continuous_route.py`（子类）、`navigation/pid_support.py`、`perception/sensors/feedback_mailbox.py`、`../parking/{terminal_owner,transition_gate}.py` | 偏重：1594 行单类 + 1053 行单方法 |
| `control_core.py` | 407 | 标准库 + numpy（无内部 import） | `navigation/{base_node,pid_support,ros_node}.py` | 可接受：`follow_trajectory` 86 行 |
| `pid_core.py` | 26 | 标准库 + numpy | `navigation/pid_support.py` | 清楚 |
| `route_progress.py` | 127 | numpy | `planned_step_filter.py` | 清楚 |
| `planned_step_filter.py` | 74 | `.route_progress`（模块级）、numpy | `navigation/building_navigation.py` | 清楚 |
| `policy_selection.py` | 70 | `common.config`、标准库 | `robot/unitree/policy/worker.py`、`simulation/truth_worker.py` | 可疑：控制包里的资产路径选择器 |
| `replan_policy.py` | 26 | 无 | `navigation/base_node.py` | 清楚 |
| `terrain_speed.py` | 20 | numpy | `navigation/building_navigation.py` | 清楚 |
| `__init__.py` | 空 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：`cascade_core.py` 1850 行，`Controller` 的 `update()` 单方法 1053 行（`__init__` 105、`set_path` 93），3 个 >80 行函数，39 行注释。
- **重复实现**：`cascade_core.wrap` 与 `pid_core.wrap`（以及根目录 `relative_pid.wrap_angle`）三份同义实现；`cascade_core` 内嵌的 `PARKING` 常量表（`capture_xy_entry_m=0.025`、`first_fixed_parking_window_s=5.0`、`maximum_xy_drift_m=0.05` 等）与 `../parking/` 的 `SPEC`/`CONFIG` 是"同一组停车数值的第四份副本"，两处不引用同一来源。
- **跨层直达**：`cascade_core.py` 模块级 import `../parking/ordinary_parking`（状态机，不只是契约），并直接调用 `ordinary.update(...)`、`ordinary.capture_grid.evidence()`、`ordinary.observe_business_rejection(...)`；`Controller.__init__` 还直接 `copy.deepcopy(config)` 后自己解析 10+ 个 `cascade.*` 键，绕过任何配置层封装。
- **隐式契约**：`Controller.update()` 的返回值是 `(ndarray[3], dict)`，mode 字符串 `drive`/`turn`/`brake`/`hold`/`capture` 与 `pre_turn`/`settle`/`path_end_hold`/`reference_constraint_hold` 都靠字符串比较；`ordinary_parking.update()` 的 8 个位置参数由 `cascade_core` 按顺序传；`feedback_mailbox.py` 直接 import `cascade_core.rotation/vector/wrap` 三个工具函数（跨子包复用私有工具）。

结论：耦合**偏重**。`control_core`、`pid_core`、`route_progress`、`terrain_speed`、`replan_policy` 是清楚的纯计算叶子；债务全部集中在 `cascade_core.py` —— 它同时是控制律、模式调度器、停车状态机宿主和几何工具库，并且是 `tracking ↔ parking` 双向依赖的 tracking 侧端点。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `route_progress.py` | 良好 | 127 行、1 类、docstring 比 1.0、22 行注释，类 docstring 说明 forward-prefix 搜索与 witness 阶段 |
| `pid_core.py` | 良好 | 26 行、2 函数、docstring 比 0.5，`slew` 的 dt 钳位与"零请求吸附"写在 docstring 里 |
| `replan_policy.py` | 良好 | 26 行、1 函数 docstring 1.0，明确"不授权运动/不延长新鲜度" |
| `planned_step_filter.py` | 可接受 | 74 行、3 函数 docstring 比 0.0，最长 37 行，5 行注释解释"计划地面不是墙" |
| `terrain_speed.py` | 可接受 | 20 行、2 函数 docstring 比 0.0，但只有 0.01/0.1/0.8/0.45 四个阈值，且模块 docstring 说明"只用计划几何" |
| `policy_selection.py` | 可接受 | 70 行、6 函数 docstring 比 0.167，最长 13 行；返回纯 dict，但"哪些路径必须存在"分散在三个函数里 |
| `control_core.py` | 可接受 | 407 行、20 函数 docstring 比 0.4、最长 `follow_trajectory` 86 行、14 行注释；`HeadingGate.update` 44 行无 docstring |
| `cascade_core.py` | 需改进 | 1850 行、26 函数 docstring 比 0.654（模块函数有 docstring，14 个 `Controller` 方法大多没有）、最长 1053 行、3 个 >80 行函数；1053 行里混了模式分派、停车委托、积分卸载校验、捕获/保持 dwell、诊断行拼装 |

- 结构问题 vs 文档问题：`cascade_core.py` 是**结构问题**（单方法 1053 行，改一行要在千行上下文里定位）；`control_core.py`、`policy_selection.py`、`planned_step_filter.py` 主要是文档问题（缺 docstring，函数本身不长）。
- 抽查复核：`Controller.update` 实测 798–1850 行共 1053 行（等于文件尾），`control_core.follow_trajectory` 实测 144–229 行共 86 行，与事实包一致。

## 重构建议

本轮只写建议、不改代码；任何动作都要"先有测试/对照再动"。`cascade_core.py` 与 `control_core.py` 都**没有专用单测**（`grep -rln cascade_core tests/` 与 `control_core` 均 0 命中），所有拆分建议都以"先补冒烟/录制对照"为前提。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 按 `mode_override` 把 `Controller.update()`（1053 行）拆成 `_update_drive/_update_turn/_update_brake/_update_hold/_update_capture` 五个方法，共享一个显式状态对象；先做纯搬移、不改判据 | 1053 行单方法、26 函数里 3 个 >80 行 | mode 之间有大量共享可变状态（`self.turn_phase`、`self.capture_*`、`self.hold_*`、`self.velocity_integral`/`hold_integral`、`self.protected`），搬移时最容易改变"先保护后计算"的顺序 → 命令变化 | 先录一段真实 run 的 `Controller.update` 返回命令序列（drive/turn/capture/zero_hold 各若干帧），拆分前后逐帧比对；再补 `__init__` + 单帧 drive 的冒烟测试 |
| P1 | 把 `update()` 里的普通停车委托段（`ordinary.update(...)`、`capture_measurement_eligible`、`capture_grid.evidence()`、`ordinary_zero_event` 组装）抽成 `parking/ordinary_terminal_coordinator.py` 或 `tracking/terminal_stage.py` | 让 cascade 只消费停车阶段的**结果**，不再持有停车状态机；同时打断 `tracking` 模块级 import `parking.ordinary_parking` | 停车状态（`OrdinaryParking` 实例）是可变对象，搬家会改变其生命周期与复位时机（`ordinary_reset_pending`） | 对照一次普通停车 run 的 `stage` 序列（capture→controlled_deceleration→handoff_zero→zero_hold）与 `first_T0_set` 时间 |
| P1 | 把 `PARKING` 常量表与 `../parking/` 的 `CONFIG`/`SPEC` 去重：由 `parking` 侧导出唯一来源，`cascade_core` 引用它 | 同一组停车数值第 4 份副本，改一处不同步就不会报错 | 两处数值当前一致，但语义不同（一个是控制增益、一个是窗口契约），错误合并会混淆"捕获容差"与"控制死区" | 逐键比对两表交集数值，只把重复键改为引用，不合并语义不同的键 |
| P2 | 把 8 个模块级纯工具函数（`canonical`/`sha`/`wrap`/`vector`/`integer`/`rotation`/`deadband`/`bounded`）移到 `common/`，与 `pid_core.wrap`、`relative_pid.wrap_angle` 合并 | 三处角度归一化 + 三处 `canonical` JSON 序列化 | `feedback_mailbox.py` 等外部模块直接 import 这些函数名，移动会破坏调用方 | 对 `[-3π, 3π]` 采样比对 wrap 逐位一致；`grep` 全部 import 点后一次性改 |
| P2 | `Controller.__init__`（105 行）里的配置解析抽成 `CascadeConfig` dataclass（保留全部 raise 与边界） | 105 行构造 + 10+ 个 `config.get` 散落 | 构造期 raise 是"配置错误即失败"契约 | 对每类非法 `cascade.*` 各写 1 条断言 raise 的用例 |
| P2 | `policy_selection.py` 移出 `control/`（放 `robot/unitree/` 或 `common/`） | 它是资产路径选择，不是控制计算，却让"控制包"带上文件系统 IO | 两个调用方的 import 路径会变 | 先确认调用方只有 `robot/unitree/policy/worker.py` 与 `simulation/truth_worker.py` |
| P2 | 删掉 `Controller.ttl`（未读取的赋值）或在诊断行里真正使用它 | 死字段容易被误读成"TTL 可配" | 无 | `grep self.ttl` 为 0 处读取 |

没有 P0：未发现会改变控制输出正确性的问题；但要注意 `cascade_core.py` 无测试覆盖，任何改动都属"高频风险区"，必须先建对照再动。

## 输入输出与调用

输入是 SLAM 位姿/旋转/速度、IMU、SCAN 采样、点云、路线与目标；主要控制计算输出 `[vx, vy, wz]` 与诊断字典；路径选择等辅助接口另有本地文件访问。主要调用方是 `../../navigation/pid_control.py`/`pid_route.py`、`../../navigation/ros_node.py`、`../../navigation/building_navigation.py` 与 `../../planning/reference/`。时间参数全部是调用方给的原生整数纳秒（`Controller.update` 的 `clock_ns`/`wall_ns`），本模块不采样时钟。

术语见 [docs/glossary.md](../../../../docs/glossary.md)；四条执行路径与默认状态机见 [docs/architecture.md](../../../../docs/architecture.md)。

## 测试

`../../../../tests/regression/test_planned_step_filter.py`、`test_route_progress.py`、`test_terrain_speed.py` 分别覆盖对应文件。`cascade_core.py` 与 `control_core.py` 没有专用单测，只随导航运行链路使用，改动需自行补测；`pid_core.py`、`replan_policy.py`、`policy_selection.py` 同样无直接 import 命中。

## 限制

控制律只在仿真/离线环境使用；完整路线、跨楼层与实机未验证。`cascade_core.py` 的 `gains` 被强制等于代码内 `DEFAULT_GAINS`，所以它不是可调参数表；要改增益得改代码并重新做等价/回归对照。诊断输出（`self.row`）当前只由外层决定是否落盘，默认 `RuntimeTelemetry` 不写历史 JSONL。
