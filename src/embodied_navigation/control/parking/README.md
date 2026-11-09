# control/parking

本目录实现停车阶段的控制律与契约：从"到达末端点"到"5 秒零速度保持"的测量、握手、ACK 与失败判定，全部基于实测位姿与零输入证据。

## 边界

- 做：判定停车窗口是否成立（`parking_contract.py`、`parking_pose10hz.py`）、零输入事件与帧授权（`parking_zero_contract.py`）、末端朝向推导（`terminal_heading_contract.py`）、过渡段判据（`transition_gate.py`）、1 秒 settle 证据（`navigation_stop_window.py`），以及若干默认关闭的扩展（`integral_unload.py`、`terminal_owner.py`、`source_hold.py`、`causal_join.py`）。
- 不做：不判定任务成功/失败（由 `../../navigation/pid_control.py` 与 `../../navigation/base_node.Navigation` 决定），不产生路线，不提供物理安全；零速度只由测量与 ACK 契约确认，不靠计时器推定静止。
- 不由本目录决定"何时进入停车"：触发时机在 `../../navigation/`（`pid_control.control_parking` 与 `base_node` 的状态机），见 [navigation/README.md](../../navigation/README.md)。

## 目录结构

```text
src/embodied_navigation/control/parking/
├── README.md                          # 本文件
├── __init__.py                        # 空包声明
├── causal_join.py                     # 467 行，R104 预提交 IMU 因果配对（默认关闭）
├── integral_unload.py                 # 425 行，R16 协调 yaw 积分卸载扩展，无停止权
├── navigation_stop_window.py          # 228 行，1 秒 settle 统计窗口与过渡上下文绑定
├── ordinary_parking.py                # 493 行，末端普通停车接管：10 Hz 捕获 + 零保持阶段
├── parking_contract.py                # 58 行，R80 停车契约常量与 enabled(profile) 校验
├── parking_pose10hz.py                # 279 行，10 Hz 位姿栅格与捕获栅格
├── parking_zero_contract.py           # 410 行，零输入事件/帧/ACK 的纯校验契约
├── source_hold.py                     # 41 行，R106 目标包络保持（默认关闭）
├── terminal_heading_contract.py       # 85 行，末端朝向由 route approach 几何推导
├── terminal_owner.py                  # 208 行，R105 末端接管 owner（默认关闭）
├── terminal_reference.py              # 133 行，固定目标末端参考（不是 SCAN 路径）
└── transition_gate.py                 # 1730 行，教师停/转向相位门（drive/pre_turn/align/settle）
```

实测合计：12 个 Python 文件 4557 行（另有空的 `__init__.py`）；最大文件 `transition_gate.py` 1730 行，占本子包 38%。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `transition_gate.py` | 活代码；`TeacherHeadingGate` 由 `../../navigation/ros_node.py` 构造，并在初始化时替换 `base_node` 里的同名 gate（见 [docs/architecture.md](../../../../docs/architecture.md)）；`localization_mapping/known_scene_hold_guard.py` 延迟引用它 |
| 代码 | `ordinary_parking.py` | 活代码；`OrdinaryParking` 由 `../tracking/cascade_core.py` 持有并逐帧 `update`，阶段为 `capture` / `controlled_deceleration` / `handoff_zero` / `zero_hold` / `failed` |
| 代码 | `parking_pose10hz.py`、`parking_zero_contract.py`、`navigation_stop_window.py` | 活代码；被 `navigation/` 与 `tracking/` 直接 import |
| 代码 | `parking_contract.py` | 契约常量；`LEGACY_MODE`/`LEGACY_SPEC` 是保留的 v1 文档形状（**已不被运行时选择**），`MODE=ordinary_pose10hz_zero_ack_fixed5` 与 `SPEC` 才是现役 |
| 代码 | `causal_join.py`、`integral_unload.py`、`terminal_owner.py`、`source_hold.py` | 显式扩展，开关见下；默认关闭 |
| 代码 | `terminal_heading_contract.py`、`terminal_reference.py` | 活代码；被 `navigation/mission`、`planning/reference` 与 `robot/unitree/bridge.py` 引用 |
| 代码 | `__init__.py` | 空文件，无逻辑 |

**日志/输出**（本目录整体不写 JSONL：`node.evidence` 默认是 `common/runtime_telemetry.py` 的 `RuntimeTelemetry`，其 `append()` 是空操作，只有 `.atomic()` 真写盘）

- `terminal_owner.py:196`：`node.evidence.append(run/"r105_terminal_owner.jsonl", {...})`，字段 `event="created"`、`reference`、`reference_sha256`、`region_fact_preserved`、`SCAN_motion_bundle_fabricated`、`first_T0_set`。走 `append`，**默认运行不落盘**。
- `causal_join.py:451`：`finish()` 里直接 `(node.run/"r104_parking_causal_join.json").write_text(...)`，schema `r104_parking_causal_join/v1`，含 `counts`、`events`、`accounting`、`source_receipts_refreshed=False`、`new_zero_publications=False`、`positive_qualification_granted=False`。这是本目录**唯一绕过 evidence 抽象的同步写盘**。
- 其余 10 个文件无写盘调用（`grep -n "write_text\|open("` 只命中 `causal_join.py`）。

**配置**（全部经 profile 读入：`configs/` base 值 → 启动器合并 → prepare 写 `runtime_context.json` → 节点 profile，见 [configs/README.md](../../../../configs/README.md)）

| 键 | 读取点 | 说明 |
| --- | --- | --- |
| `ordinary_terminal_parking_contract` | `parking_contract.enabled()` | 交给 `../safety/safety_configuration.parking()` 逐值校验：5 s 窗口、`capture_xy_m=0.025`、`capture_yaw_rad=0.035`、`capture_dwell_ns=6e8`、`source_age_ns=3e8`、`reset_window=false` |
| `cascade.parking_pose10hz` | `ordinary_parking.configure_pose10hz()` | 100 ms 栅格、5 s、0.05 m / 0.1 rad 漂移、`interpolation=false` |
| `navigation_settle_statistics_contract` | `transition_gate.__init__` → `NavigationStopWindow` | 1 s settle 的 gyro/euler/xy 界限 |
| `teacher_transition` | `transition_gate.__init__` | 停/转向判据：`stop_window_sim_s=0.3`、`stop_body_gyro_radps=0.05`、`stop_pose_gap_max_s=0.2`、`turn_complete_rad=0.1` 等 9 个键 |
| `max_yaw_rate_radps` | `transition_gate.__init__` | 只接受 `(0, 0.6]` rad/s，否则 raise；错误信息明确 NAV 0.6 尚无物理验证 |
| `cascade.command_limits`、`cascade.r85_student_lower_loop_ack` | `transition_gate` active-braking 分支 | 仅 `r85_active_turn_braking.enabled=true` 时读 |
| `r85_auto_reference_contract.terminal_heading_from_pct_approach` | `terminal_heading_contract.enabled()` | 注意该键在 [configs/planning/settings.json](../../../../configs/planning/settings.json)，不在 `control/` |
| `r85_active_turn_braking`、`r85_quiet_handoff_target`、`ordinary_turn_control_handoff`、`r85_pre_zero_integral_unload`、`r85_stance_support_handoff`、`r85_neutral_rate_brake_capture` | 各扩展 | 严格校验形状，非法组合直接 raise；base 配置当前只有 `ordinary_turn_control_handoff.enabled=true`，其余为 false |

环境开关（**模块导入时读一次**，非法值直接 `raise ValueError`，不会静默降级）：

- `R104_PARKING_CAUSAL_JOIN`（`causal_join.py:14`）、`R105_TERMINAL_PARKING_OWNER`（`terminal_owner.py:8`）、`R106_PREQUALIFICATION_SOURCE_HOLD`（`source_hold.py:5`），代码缺省 `"0"`。
- `configs/launch/settings.json` 的 `features` 当前把这三项都设为 `"1"`（与 `R103_IMU_SAFETY_INTAKE` 一起由启动器注入）；"模块内默认 0、launch features 为 1"成立，但**只在 A. Native 主路径**经启动器启动时成立。`demo.launch.py` 之外（Building / 独立入口）这些环境变量不会自动注入。

## 模块内文件耦合

依赖方向（包内，实测 AST）：

- 契约层：`parking_zero_contract.py`（无模块级内依赖，只有延迟的 `terminal_reference`）；`source_hold.py` 完全独立。
- 采样层：`parking_pose10hz.py`（模块级只依赖标准库，`../safety/` 两个 import 都在函数内）。
- 阶段层：`ordinary_parking.py → source_hold`（模块级，`parking_pose10hz` 延迟）；`transition_gate.py → navigation_stop_window`（模块级），`integral_unload`、`../tracking/cascade_core`、`robot/unitree/policy/lower_loop_ack` 全部延迟。
- 扩展层：`terminal_owner.py → terminal_reference`（模块级）+ `parking_contract`、`../tracking/cascade_core`（延迟）；`causal_join.py → parking_zero_contract`（模块级）+ `robot/unitree/bridge.validate_controller_source`（模块级）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `transition_gate.py` | 1730 | `navigation_stop_window`、`common.input_freshness`（模块级）；`integral_unload`、`../tracking/cascade_core`、`lower_loop_ack`、`common.observation`（延迟） | `../../navigation/ros_node.py`、`localization_mapping/known_scene_hold_guard.py` | 偏重：37 个方法 + 1 个模块函数 |
| `ordinary_parking.py` | 493 | `source_hold`（模块级）；`parking_pose10hz`、`common.observation`（延迟） | `../tracking/cascade_core.py`、`../safety/duplicate_source.py`、`../../navigation/pid_control.py` | 偏重：`update` 124 行 |
| `causal_join.py` | 467 | `parking_zero_contract`、`robot/unitree/bridge`（模块级） | `../../navigation/pid_support.py`、`localization_mapping/known_scene_inputs.py`、`../safety/imu_causal_buffer.py` | 偏重：`prepare` 113 行 |
| `integral_unload.py` | 425 | 无（纯标准库 + 延迟 import） | `transition_gate`、`../tracking/cascade_core`、`robot/unitree/policy/{lower_loop_ack,yaw_measurement_feedback}` | 可接受 |
| `parking_zero_contract.py` | 410 | 无（延迟 `terminal_reference`） | `executor_command`、`causal_join`、`parking_contract`、`../safety/duplicate_source`、`../tracking/cascade_core`、`../../navigation/pid_support`、`robot/unitree/bridge` | 清楚（纯校验） |
| `parking_pose10hz.py` | 279 | `../safety/{feedback_source,safety_configuration}`（延迟） | `ordinary_parking`、`parking_contract`、`../tracking/cascade_core`、`../../navigation/building_navigation`、`perception/sensors/feedback_mailbox` | 清楚 |
| `navigation_stop_window.py` | 228 | `../safety/safety_configuration`（延迟） | `transition_gate`、`../../navigation/{base_node,pid_support,ros_node}` | 可接受 |
| `terminal_owner.py` | 208 | `terminal_reference`、`localization_mapping/known_scene_hold_guard`（模块级）；`../tracking/cascade_core`、`parking_contract`、`common.input_freshness`（延迟） | `../safety/duplicate_source`、`localization_mapping/known_scene_inputs`、`../../navigation/pid_support` | 可疑：control 顶层依赖 localization_mapping |
| `terminal_reference.py` | 133 | 无 | `executor_command`、`parking_zero_contract`、`terminal_owner`、`../safety/duplicate_source`、`../tracking/cascade_core`、`../../navigation/pid_control`、`robot/unitree/bridge` | 清楚 |
| `terminal_heading_contract.py` | 85 | 无 | `../../navigation/mission/mission_runtime`、`planning/reference/automatic_reference_runtime` | 清楚 |
| `parking_contract.py` | 58 | `parking_pose10hz`、`parking_zero_contract`（模块级）；`../safety/safety_configuration`（延迟） | `terminal_owner`、`../../navigation/pid_support` | 清楚 |
| `source_hold.py` | 41 | 无 | `ordinary_parking.py` | 清楚 |
| `__init__.py` | 空 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：`transition_gate.py` 1730 行、`TeacherHeadingGate` 37 个方法 + 1 个模块函数、7 个 >80 行函数（`update` 252、`evidence` 165、`__init__` 161、`observe` 125、`_observe_quiet` 114、`note_quiet_publication` 104、`bind_zero_ack` 93）。
- **重复实现**：`clock` 类常量有两处同源副本 —— `../safety/clock_hold.py` 的 `CONTRACT` 与 `../safety/safety_configuration.clock()` 的期望值（`freshness_sim_and_wall_s=0.3`、`continuous_stale_fail_wall_s=8.0`、`same_clock_policy`、`backward_clock_policy`），而后者没有调用点。本子包内 `navigation_stop_window.SPEC` 与 `safety_configuration.settle()` 的 8 个数值也是"字面量 + 必须相等"的副本关系。
- **跨层直达**：`terminal_owner.py` 在模块级 import `localization_mapping/known_scene_hold_guard._frame`（私有函数，跨到上层子包）；`causal_join.py` 与 `../safety/duplicate_source.py` 在模块级 import `robot/unitree/bridge.validate_controller_source`，而 `bridge.py` 又 import `parking_zero_contract` —— parking ↔ robot.unitree 是**双向包依赖**。
- **隐式契约**：`causal_join.SOURCE_KEYS` 是 24 个字符串键的元组，必须与 `robot/unitree/bridge.py` 的 envelope 完全一致；`ordinary_parking.update()` 有 8 个位置参数 + 2 个关键字参数，调用方 `cascade_core` 按顺序传；`transition_gate` 通过 `profile["teacher_transition"]` 的 9 个字符串键判停。

结论：耦合**偏重**。契约层（`parking_zero_contract`、`terminal_reference`、`parking_pose10hz`、`source_hold`、`parking_contract`）是清楚的纯校验；但 `transition_gate.py` 一个类同时承担 gyro 停检、quiet 窗口、active braking、handoff deadline、settle、扩展绑定 6 组职责，并且 `tracking ↔ parking`（`cascade_core` 模块级 import `ordinary_parking`；`terminal_owner`/`transition_gate` 延迟 import `cascade_core`）与 `parking ↔ robot.unitree` 都是包级双向依赖。control 下 39 个非 launch 模块可单独 import，说明环被"延迟 import"压住了，没有导入期失败，但依赖方向本身不干净。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `parking_zero_contract.py` | 良好 | 410 行但 11 个纯校验函数，docstring 比 0.818，最长 `validate_event`/`validate_zero_ack` 63 行，无 IO、无状态 |
| `source_hold.py` | 良好 | 41 行、1 函数 27 行、docstring 比 1.0，`eligible()` 把 9 组前置条件写成显式布尔与 |
| `terminal_heading_contract.py` | 良好 | 85 行、4 函数、docstring 比 0.5，最长 42 行 |
| `parking_pose10hz.py` | 可接受 | 279 行、2 类、docstring 比 0.077；函数都短（最长 32 行），但 `PoseGrid`/`CaptureGrid` 的公开状态（`points`、`ready`）没有 docstring |
| `integral_unload.py` | 可接受 | 425 行、13 函数、docstring 比 0.385、最长 `validate_receipt` 66 行 |
| `parking_contract.py` | 可接受 | 58 行、1 函数、docstring 比 0.0，但 4 行注释解释了 v1 契约保留原因 |
| `terminal_reference.py` | 可接受 | 133 行、4 函数、docstring 比 0.0，最长 `fields` 72 行 |
| `navigation_stop_window.py` | 需改进 | 228 行、8 函数、docstring 比 0.125、`observe` 90 行、`SPEC` 里 16 个键没有注释 |
| `causal_join.py` | 需改进 | 467 行、18 函数、docstring 比 0.222、`prepare` 113 行 |
| `ordinary_parking.py` | 需改进 | 493 行、15 函数、docstring 比 0.267、`update` 124 行、`stage` 是裸字符串（5 个取值，没有枚举） |
| `terminal_owner.py` | 需改进 | 208 行、docstring 比 1.0 但 `control()` 134 行；用 `getattr(node, "_r105_terminal_owner_enabled", False)` 读隐式绑定 |
| `transition_gate.py` | 需改进 | 1730 行、docstring 比 0.026（38 个函数里只有 1 个模块函数有 docstring）、最长 252 行、7 个 >80 行函数、注释 36 行；`self.config` 直接是 profile 的 9 键 dict |

- 结构问题 vs 文档问题：`transition_gate.py`、`ordinary_parking.py`、`causal_join.py`、`terminal_owner.py` 是**结构问题**（职责多、函数长）；`parking_pose10hz.py`、`terminal_reference.py`、`parking_contract.py` 基本只是 docstring 缺口。
- 抽查复核：`transition_gate.TeacherHeadingGate.update` 实测 1313–1564 行共 252 行，`ordinary_parking.OrdinaryParking.update` 323–446 行共 124 行，与事实包一致。

## 重构建议

本轮只写建议、不改代码；任何动作都要"先有测试/对照再动"。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 按 phase 把 `TeacherHeadingGate.update()`（252 行）拆成 `_update_drive` / `_update_pre_turn` / `_update_align` / `_update_settle` 四个方法，返回值与状态突变保持不变 | 252 行单函数、7 个 >80 行函数 | phase 间共享可变状态（`self.heading`、`self.settle_control_stage`、`self.stop_window`），拆错会改变相位顺序 → 停车行为变化 | `grep transition_gate tests/` 为 0，当前**没有测试**；先用录制的 `alignment_phase` 序列 + `navigation_settle_statistics_contract` 做逐帧对照，再拆 |
| P1 | 把 `TeacherHeadingGate.__init__`（161 行）里 6 组扩展配置校验抽到 `parking/transition_config.py`（输入 profile，输出 frozen dataclass） | 161 行构造、6 组 `raise` 规则混在状态初始化里 | 构造期 raise 是"配置错误即失败"契约，抽走后报错栈变、报错时机不变 | 对每类非法 profile 各写 1 条断言 raise 的用例 |
| P1 | 断开 `tracking ↔ parking` 包级双向依赖：让 `../tracking/cascade_core.py` 只依赖 parking 的契约层，`OrdinaryParking` 实例改由 `navigation` 侧注入（或在 `parking/__init__.py` 暴露协议） | `cascade_core.py` 已 1850 行，还模块级 import 停车状态机；反向边靠延迟 import 压住 | `OrdinaryParking` 是可变状态对象，注错实例会跨请求泄漏停车状态 | `cascade_core` 无专用测试（grep 0）；先补 `Controller.__init__` + 一帧 drive 的冒烟测试，再对照一次普通停车 run 的 `zero_hold` 阶段序列 |
| P1 | 把 `ordinary_parking.py` 的 `stage` 从裸字符串改为模块级枚举/常量元组，并在 `update_pose10hz` 与 `update` 共用 | 5 个阶段字符串散落在 15 处赋值/比较；拼错不报错 | 阶段字符串会进证据字典，改名会改变历史记录 | 先 grep 全仓 `"zero_hold"`/`"controlled_deceleration"` 消费点，确认无外部按字符串匹配后再改 |
| P2 | `transition_gate.evidence()`（165 行）按证据类别拆成 3~4 个小函数 | 纯拼字典，但 165 行 | 证据字段名变化会影响消费端 | 对照一份已有 evidence 字典做 key 集合比对 |
| P2 | `clock_hold.CONTRACT` 与 `safety_configuration.clock()` 的期望值去重，保留一个来源 | 同一组常量两份副本，后者无调用点 | 若未来有消费方按 CONTRACT 校验，删除会导致误报 | 先确认 `safety_configuration.clock()` 仍无调用点（当前 grep 为 0） |
| P2 | 给 `parking_pose10hz.PoseGrid`/`CaptureGrid`、`terminal_reference.fields`、`navigation_stop_window.SPEC` 补 docstring/注释 | 文档缺口 | 无 | 自检脚本 + 人工 review |

没有 P0：未发现会改变停车判定、零命令或安全结果的问题；以上都是结构与文档成本。

## 输入输出与调用

输入是 canonical SLAM 反馈、paired IMU、实际零输入 ACK、route/goal 与 profile；输出是停车阶段判定、零命令与证据字典；通过当前默认 `RuntimeTelemetry` 调用的历史 JSONL 写入为空操作（例外是 `causal_join.finish()` 的同步 `r104_parking_causal_join.json`）。主要调用方为 `../../navigation/pid_control.py`、`../../navigation/building_navigation.py` 与 `../tracking/cascade_core.py`。`R105_TERMINAL_PARKING_OWNER`、`R106_PREQUALIFICATION_SOURCE_HOLD` 等环境开关在模块内默认 0，launch 的 features 中为 1。

## 测试

- `../../../../tests/unit/test_parking_source_contract.py`：真实 SLAM 反馈保持非真值、时钟保留；真值、外来 producer、缺失字段都被拒绝。
- `../../../../tests/regression/test_navigation_safety.py`：10Hz 位姿栅格、零契约、末端朝向与障碍解除窗口；该文件没有直接测试 `safety_configuration.py`。
- 未覆盖（`grep -rln <模块名> tests/` 无 import 命中）：`transition_gate.py`（1730 行，最大文件）、`ordinary_parking.py`（493 行，第二大文件）、`causal_join.py`、`integral_unload.py`、`navigation_stop_window.py`、`parking_contract.py`、`terminal_owner.py`、`terminal_reference.py`、`source_hold.py`。两个最大文件都在未覆盖集合里；`tests/regression/test_navigation_safety.py` 里出现的 `ordinary_parking_stage="handoff_zero"` 只是一个字符串字段，不 import 该模块。

## 限制

没有覆盖整条停车链的测试，也没有实机或完整路线验证；私有几何数据缺失时部分回归用例会 skip。`transition_gate.py` 是仿真实验实现：profile 允许把转向上限开到 0.6 rad/s，NAV 0.6 尚无物理验证；0.3 rad/s 下过渡数学、实测停止条件与 300 ms 新鲜度保持不变。默认运行下多数证据文件不落盘（`RuntimeTelemetry.append` 空操作），排查时不能把"文件缺失"当成"事件未发生"。
