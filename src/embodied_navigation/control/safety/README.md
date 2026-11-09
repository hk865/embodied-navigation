# control/safety

本目录是仿真安全门禁与停车反馈来源检查：对同一帧的 typed 仿真安全状态给出 fault/stop/reason，并拒绝不合法的停车控制来源；同时包含 SLAM 来源、时钟与 IMU 的运行保护。

## 边界

- 做：`simulation_gate.py` 作出仿真安全决策；`feedback_source.py` 校验停车使用的 canonical SLAM 反馈；`clock_hold.py` 处理重复/回退时钟；其余文件做障碍解除窗口、IMU 交接、门禁原因码、bridge 安全交接与命令约束。
- 不做：未提供实机安全后端；`SimulationSafetyGate` 不能接受 `REAL_SENSOR` 来源，`../../common/state_contract.py` 中该类型仍为保留接口。本体接触在 `simulation_gate.py` 中仅作诊断，不新增停车条件；子包还包含独立的 SLAM/IMU/时钟保护。
- 安全状态的来源是仿真真值（`gazebo.native64.preupdate` / `gazebo_simulation`），只属于仿真安全边界，不能作为 SLAM 或导航定位输入。
- 本目录不拥有任务状态机与停车时机；它只对调用方给出的帧/时钟/来源作判定，调用点在 `../../navigation/`、`../../robot/unitree/` 与 `../../simulation/`。

## 目录结构

```text
src/embodied_navigation/control/safety/
├── README.md                    # 本文件
├── __init__.py                  # 空包声明
├── bridge_safety_handoff.py     # 265 行，bridge 安全消息的固定交接（ingress 记录，默认组拥有控制）
├── clear_guard_window.py        # 32 行，障碍保持解除需要真实新鲜几何观测
├── clock_hold.py                # 318 行，墙钟/ROS 时钟边界，重复时钟只发精确零
├── contact_mailbox.py           # 86 行，只传接触值的非阻塞 Unix datagram IPC
├── duplicate_source.py          # 425 行，一次新鲜短重复源上保留既有 R85 零字节
├── feedback_source.py           # 23 行，停车反馈的来源/时间门（无几何阈值）
├── gate_reasons.py              # 50 行，bridge veto 稳定原因码目录（不改判据）
├── guard_audit.py               # 54 行，截获原生转向门里两次 obstacle_ahead 结果
├── handoff_stop_policy.py       # 61 行，不兼容非固定尾段只请求停止、不授予运动
├── imu_causal_buffer.py         # 203 行，同源因果 IMU FIFO + 可选安全信箱
├── imu_mailbox.py               # 218 行，有界 IMU 安全交接（接收方采样、控制方应用）
├── launch_command_policy.py     # 70 行，物理门之前的额外输出限制
├── safety_configuration.py      # 86 行，数值安全设置的"必须等于"校验器
└── simulation_gate.py           # 119 行，统一的 RoboGauge 仿真安全决策
```

实测合计：15 个 Python 文件 2010 行（含一个空的 `__init__.py`）；最大文件 `duplicate_source.py` 425 行。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `simulation_gate.py` | 活代码，唯一"安全决策"模块；`latch_runtime_faults` / `evaluate` / `reject_state` 被 `../../robot/unitree/policy/native/worker.py` 与 `../../simulation/gazebo/robogauge/native_loop.py` 逐帧调用 |
| 代码 | `clock_hold.py` | 活代码；`../../navigation/pid_navigation.py:182` 构造 `ControlClockHold()`，`before_control` 在到达/航向/PI/native guard 之前拦截 |
| 代码 | `duplicate_source.py` | 活代码；被 `clock_hold`（延迟 import）与 `../../navigation/pid_control.py` 调用 |
| 代码 | `clear_guard_window.py`、`guard_audit.py`、`bridge_safety_handoff.py` | 活代码；都在 `../../navigation/ros_node.py` 里构造，`bridge_safety_handoff` 处理 `/demo/control/safety` 话题 |
| 代码 | `contact_mailbox.py` | 活代码，IPC；`ContactPublisher` 被 `../../navigation/building_navigation.py`、`../../simulation/gazebo/robogauge/worker.py`、`../../simulation/truth_worker.py` 使用 |
| 代码 | `imu_causal_buffer.py` | 活代码；`CausalImuBuffer` 由 `pid_navigation.py:120` 在 `r85_imu_source_buffer.enabled` 时构造 |
| 代码 | `imu_mailbox.py` | 活代码，但只被 `imu_causal_buffer.py` 使用（`SafetyIntake`）；开关 `R103_IMU_SAFETY_INTAKE` |
| 代码 | `handoff_stop_policy.py`、`launch_command_policy.py` | 活代码；被 `../../navigation/pid_support.py` 调用 |
| 代码 | `gate_reasons.py` | 活代码；被 `../../robot/unitree/bridge.py` 调用，只追加稳定码，不改判据 |
| 代码 | `feedback_source.py` | 活代码；唯一调用点是 `../parking/parking_pose10hz.from_feedback` |
| 代码 | `safety_configuration.py` | 活代码校验器；`parking()` / `pose()` / `settle()` 各有调用点，`clock()` **无调用点**（见下） |
| 代码 | `__init__.py` | 空文件，无逻辑 |

**日志/输出**

- `imu_causal_buffer.finish()` 用 `node.evidence.atomic` 真正写两个文件：
  - `runs/<run>/imu_source_timing_receipt.json`，schema `same_source_causal_imu_timing_receipt/v1`，字段含 `fifo_depth=64`、`history_depth=128`、`event_cap=64`、`recorded_events`、`counters`、`last_pairing_event`、`pair_max_ns=20 ms`、`navigation_ground_truth_used=False`。
  - `runs/<run>/r103_imu_safety_intake.json`，来自 `SafetyIntake.receipt()`（`counts`、`windows`、`latest`、`pending_peak`、`last_consumed_mailbox_version` 等）。
- `imu_causal_buffer` 的事件行（schema `same_source_causal_imu_timing/v1`：`phase`、`pose_stamp_ns`、`control_clock_ns`、`observed_wall_ns`、`original_pair_max_ns`、`source_topic="/livox/imu"`、`paired_stamp_ns`、`recent_callbacks`、`counters`）与 `clock_hold` 的 `navigation_control_clock_hold.jsonl`（schema `teacher_control_clock_exact_zero_hold/v1`：`sequence`、`compute_ros_clock_ns`、`compute_monotonic_wall`、`publish_ros_clock_ns`、`command_publication_sequence`、`previous_control_clock_ns`、`previous_actual_guard_clock_ns`）都走 `node.evidence.append`，默认 `RuntimeTelemetry.append` 是空操作，**默认运行不落盘**。
- `contact_mailbox.py` 不走文件：`socket.socket(AF_UNIX, SOCK_DGRAM)`，定长报文 `struct "<Q5dQ"`（`stamp_ns` + 5 个接触标量 + 故障标志）。
- 其余 11 个文件无写盘调用（`grep "write_text\|open("` 无命中）。

**配置**

- `r85_imu_source_buffer.enabled`（profile，`configs/control/settings.json` 当前为 `true`）决定 `CausalImuBuffer` 是否构造；不构造时 `pid_navigation` 保持 `imu_source_buffer = None`。
- 环境变量：`R103_IMU_SAFETY_INTAKE`（`imu_mailbox.py`，缺省 `"0"`，非法值 raise）与 `BRIDGE_SAFETY_INGRESS`（`bridge_safety_handoff.py`，缺省 `"0"`）；`configs/launch/settings.json` 的 `features` 把 `R103_IMU_SAFETY_INTAKE` 设为 `"1"`。
- `safety_configuration.py` 自己不读 `configs/`：它接收调用方传入的 dict 并逐值断言等于固定常量（`duration_ns=5e9`、`period_ns=1e8`、`window_s=1.0`、`gyro_mean_norm_radps=0.005` 等），来源是 `../parking/parking_contract.py`、`../parking/parking_pose10hz.py`、`../parking/navigation_stop_window.py` 传进来的 profile 片段。
- `clock()` 期望的 dict（`freshness_sim_and_wall_s=0.3`、`continuous_stale_fail_wall_s=8.0`、`same_clock_policy`、`backward_clock_policy`、`source_receipt_refresh_permitted`）在 `configs/control/settings.json` 中**没有对应键**（grep `clock_protection` 无命中），且全仓无调用点 —— 这是一个未接线的校验器。同样的常量在 `clock_hold.CONTRACT` 里另有一份副本，而 `CONTRACT` 在本文件内也没有引用。

## 模块内文件耦合

依赖方向：本子包基本是叶子层；跨子包边集中在 `parking`（2 个文件）、`robot/unitree`（1 个文件）、`common`（2 个文件），包内只有 1 条边。

- 包内：`imu_causal_buffer.py → imu_mailbox.py`（延迟，构造时）。
- `→ ../parking/`：`duplicate_source.py` 模块级 import `parking_zero_contract`；`imu_causal_buffer.py` 延迟 import `causal_join.scalar_snapshot`。
- `→ ../robot/unitree/`：`duplicate_source.py` 模块级 import `bridge.validate_controller_source`，而 `bridge.py` 模块级 import `parking_zero_contract`，构成 `safety → robot → parking → (延迟) safety` 的包级双向关系。
- `→ ../common/`：`clock_hold.py`（`input_freshness`）、`simulation_gate.py`（`state_contract`）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `duplicate_source.py` | 425 | `../parking/parking_zero_contract`、`../../robot/unitree/bridge`（模块级）；`ordinary_parking`、`terminal_owner`、`terminal_reference`、`planning.admission.physical_gate`、`common.observation`（延迟） | `clock_hold.py`、`../../navigation/pid_control.py` | 可疑：单函数 372 行 + 跨 3 层 import |
| `clock_hold.py` | 318 | `common.input_freshness`（模块级）；`duplicate_source`（延迟） | `../../navigation/pid_navigation.py`、`../../navigation/pid_support.py` | 偏重：`before_control` 205 行 |
| `bridge_safety_handoff.py` | 265 | 标准库（`os`/`threading`/`time`） | `../../navigation/ros_node.py` | 可接受 |
| `imu_mailbox.py` | 218 | 标准库（`collections`/`math`/`os`/`threading`） | `imu_causal_buffer.py` | 可接受：`consume` 95 行 |
| `imu_causal_buffer.py` | 203 | `imu_mailbox`、`../parking/causal_join`（均延迟） | `../../navigation/{pid_navigation,pid_support,ros_node}`（`pid_ingress`/`pid_entrypoint` 只经 `node.imu_source_buffer` 属性使用） | 可接受：0/7 docstring |
| `simulation_gate.py` | 119 | `common.state_contract` | `../../robot/unitree/policy/native/worker.py`、`../../simulation/gazebo/robogauge/native_loop.py` | 清楚 |
| `contact_mailbox.py` | 86 | 标准库 socket | `../../navigation/building_navigation.py` 等 3 处 | 清楚（IPC 边界显式） |
| `safety_configuration.py` | 86 | 标准库 `math` | `../parking/{parking_contract,parking_pose10hz,navigation_stop_window}` | 清楚；`clock()` 是死函数 |
| `launch_command_policy.py` | 70 | numpy | `../../navigation/pid_support.py` | 清楚 |
| `handoff_stop_policy.py` | 61 | 标准库 `math` | `../../navigation/pid_support.py` | 清楚 |
| `guard_audit.py` | 54 | 标准库 `types` | `../../navigation/ros_node.py` | 可接受：靠函数闭包截获 `obstacle_ahead` |
| `gate_reasons.py` | 50 | 标准库 `dataclasses` | `../../robot/unitree/bridge.py` | 清楚 |
| `clear_guard_window.py` | 32 | 标准库 `collections` | `../../navigation/ros_node.py` | 清楚 |
| `feedback_source.py` | 23 | 无 | `../parking/parking_pose10hz.py` | 清楚 |
| `__init__.py` | 空 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：无（最大 425 行，未过 800 行）；但 `duplicate_source.preserve_existing_source` 是 372 行单函数，实质是"函数级上帝"。
- **重复实现**：`clock_hold.CONTRACT` 与 `safety_configuration.clock()` 的 5 个常量同源两份，且两边都没有消费方（`CONTRACT` 在本文件内未被引用，`clock()` 全仓无调用）。
- **跨层直达**：`duplicate_source.py` 同时 import `parking` 契约与 `robot/unitree/bridge`（模块级），使 safety 依赖 parking 与 robot 两个上层/邻接子包；`imu_causal_buffer.py` 反向依赖 `parking/causal_join`。包级没有 import 环错误（control 下 39 个非 launch 模块均可单独 import），但依赖方向是网状的。
- **隐式契约**：`duplicate_source` 依赖 24 个字符串键的 `SOURCE_KEYS` 与 `bridge` 的 envelope 一致；`imu_causal_buffer` 依赖 `node.raw_imu`、`node.run`、`node.evidence` 三个属性名；`guard_audit` 依赖传入函数 `__globals__["obstacle_ahead"]` 存在。

结论：耦合**偏重但可解释**。`simulation_gate`/`feedback_source`/`gate_reasons`/`clear_guard_window`/`contact_mailbox` 是干净的纯判定或纯 IPC 叶子；成本集中在 `duplicate_source.py`（372 行单函数 + 跨层 import）与 `clock_hold.before_control`（205 行），这两处是本子包唯一的结构性债务。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `feedback_source.py` | 良好 | 23 行、1 函数 docstring 1.0，docstring 明确三个来源字段与两个时钟域不可相减 |
| `safety_configuration.py` | 良好 | 86 行、5 函数 docstring 比 1.0、最长 21 行；每个 `_number(...)` 都带键名报错 |
| `clear_guard_window.py` | 良好 | 32 行、`observe` 23 行 docstring 说明 10 样本 / 1 s / 300 ms 间隔 / 1.35 s 尾窗 |
| `gate_reasons.py` | 良好 | 50 行、`GATE_SPECS` 是数据表，`GateReasons.append/reject` 各 4 行 |
| `simulation_gate.py` | 可接受 | 119 行、docstring 比 0.4；阈值（1.5 s/0.8 rad/0.15 m 观测，2 s/1.1 rad/0.10 m 持续，4 s 观察窗）在 `evaluate` 里是裸数字，只有模块 docstring 说明"保留原阈值" |
| `contact_mailbox.py` | 可接受 | 86 行、docstring 比 0.429、最长 19 行；线格式 `<Q5dQ` 有行内注释 |
| `guard_audit.py` | 可接受 | 54 行、4 个函数（最长 `__init__` 13 行，`__call__` 8 行）；用 `FunctionType` 重建函数命名空间，机制在模块 docstring 里解释 |
| `handoff_stop_policy.py` | 可接受 | 61 行、1 函数 56 行、docstring 1.0 |
| `launch_command_policy.py` | 可接受 | 70 行、1 函数 62 行、docstring 1.0 |
| `bridge_safety_handoff.py` | 需改进 | 265 行、15 函数 docstring 比 0.0、最长 `Handoff.service` 52 行 |
| `imu_causal_buffer.py` | 需改进 | 203 行、7 函数 docstring 比 0.0、最长 70 行；常量 `FIFO_DEPTH`/`PAIR_NS`/`EVENT_CAP` 有行内注释，但公开方法没有 |
| `imu_mailbox.py` | 需改进 | 218 行、8 函数 docstring 比 0.0、`SafetyIntake.consume` 95 行 |
| `clock_hold.py` | 需改进 | 318 行、docstring 比 0.143、`before_control` 205 行（唯一有 docstring 的是类本身） |
| `duplicate_source.py` | 需改进 | 425 行、3 函数、`preserve_existing_source` 372 行；docstring 比 0.667 掩盖了函数级复杂度 |

- 结构问题 vs 文档问题：`duplicate_source.py`、`clock_hold.py`、`imu_mailbox.py` 是结构问题（单函数过长）；`bridge_safety_handoff.py`、`imu_causal_buffer.py` 主要是文档问题（函数都短，0 docstring）。
- 抽查复核：`simulation_gate.evaluate` 实测 46 行，阈值与 `docs/architecture.md`「保留原阈值、宽限和锁存优先级」一致；`clock_hold.before_control` 实测 114–318 行共 205 行，与事实包一致。

## 重构建议

本轮只写建议、不改代码；任何动作都要"先有测试/对照再动"。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `duplicate_source.preserve_existing_source()`（372 行）按"上下文校验 / 源字节校验 / 时钟与 receipt / 结果登记"拆成 4 个内部函数，共用一个 receipt dict | 372 行单函数，且跨 import parking/robot 两包 | 它是"是否允许保留既有零字节"的唯一判据，任何重排都可能放宽/收紧通过条件；`grep -rln duplicate_source tests/` 无 import 命中，**当前没有直接单测** | 先从一次真实运行截取 `context` + `command_before` + `clock_ns` 做 golden 用例（allow/deny 各若干），再拆 |
| P1 | 把 `clock_hold.before_control()`（205 行）拆成 `_source_freshness()` / `_clock_progress()` / `_hold_or_publish()`，`classify` 保持纯函数 | 205 行混了来源新鲜度、时钟分类、发布路径 | 它决定"是否只发精确零"，顺序错会让重复时钟漏过一次控制 | 现有测试无 `clock_hold` import；先补 `classify` 的 advance/duplicate/stalled/backwards 四态用例（纯函数，成本低），再做拆分 |
| P1 | 断开 `safety → robot` 模块级边：`duplicate_source` 所需 `validate_controller_source` 改由调用方（navigation）传入，或下沉到 `common/` | `safety → robot → parking` 的双向包依赖 | 校验函数是纯函数，注入即可；但少一处 import 会让"谁校验"变隐式 | 对照一次 duplicate 场景的 receipt 字段集合不变 |
| P2 | 删除或接线 `safety_configuration.clock()` 与 `clock_hold.CONTRACT` 的重复常量：保留一个来源 | 同一组常量两份、且都无消费方 | 若未来有配置方按 `clock()` 校验，删除会让非法配置静默通过 | 先确认 grep 无调用点；若接线，需要在 `configs/control/settings.json` 增键并在 prepare 侧确认 |
| P2 | 给 `bridge_safety_handoff.py`、`imu_causal_buffer.py`、`imu_mailbox.py` 的公开方法补 docstring | docstring 比 0.0 | 无 | 自检脚本 |
| P2 | `imu_mailbox.SafetyIntake.consume()`（95 行）按"取窗口 / 判中断 / 记 receipt"拆小 | 单函数 95 行 | 它影响 IMU 安全交接的峰值保持，`>=0.18` 中断条件在 docstring 里 | 用 `SafetyIntake.receipt()` 的字段做前后对照 |

没有 P0：未发现会放宽安全判定或把真值来源引入导航的问题；仿真安全边界与"拒绝错误来源/时钟"的语义在代码里是显式 raise/return 的。

## 测试

- `../../../../tests/unit/test_simulation_safety.py`：门禁的故障优先级、宽限与来源拒绝（直接 import `SimulationSafetyGate`）。
- `../../../../tests/regression/test_native_safety_integration.py`：AST 检查两个原生循环都 import 共享门禁；装 Torch 时用真实观测构造 + 假 socket 跑完 Student/Teacher 与 RoboGauge 两个循环，验证逐帧调用门禁、动作不变（Torch 缺失时该组 skip）。
- `../../../../tests/regression/test_navigation_safety.py`：障碍解除窗口的真实样本与缓存拒绝（`ClearGuardWindow`）。
- 未覆盖（`grep -rln <模块名> tests/` 无 import 命中）：`clock_hold.py`、`duplicate_source.py`、`imu_causal_buffer.py`、`imu_mailbox.py`、`bridge_safety_handoff.py`、`guard_audit.py`、`gate_reasons.py`、`handoff_stop_policy.py`、`launch_command_policy.py`、`contact_mailbox.py`、`feedback_source.py`、`safety_configuration.py`。

## 限制

无实机安全验证，也没有实机后端；仿真安全状态只覆盖仿真安全边界，完整路线与实机仍未验证。系统级说明见 `../../../../docs/architecture.md`，其「安全门禁是实际调用，不只是约定」一节给出调用链；术语（provenance、时钟域、raw/corrected）见 `../../../../docs/glossary.md`。默认运行下多数证据 JSONL 不落盘（`RuntimeTelemetry.append` 空操作），只有 `imu_causal_buffer` 的两个 receipt 会真写。
