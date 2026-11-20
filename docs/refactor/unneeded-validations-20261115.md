# 不必要校验清查结果（2026-11-15，HEAD b314337）

## 怎么查的

1. **初查**：4 个只读代理分区查找，范围是准入链路、导航节点/PID、控制/机器人/传感器、DynaNav/脚本。按 5 类判定：
   - A：版本开关取值恒定
   - B：上游已经校验过
   - C：当前调用图下不可能发生
   - D：只服务证据，没有消费方
   - E：防御性冗余
2. **对抗复核**：发现分成 11 组，每组派 1 个反驳者，尽力证明“这段代码会执行”或“删了会改变行为”。另派 2 个查缺代理扫初查没覆盖的目录。共 13 个代理，全部返回。
3. **复核结果**：

| 结论 | 条数 | 估算行数 |
|---|---|---|
| confirmed（反驳失败，可删） | 47 | ~3,800 |
| partial（部分成立，删除边界已收窄） | 62 | ~3,800 |
| needs_human（需要你决定） | 15 | ~1,300 |
| refuted（声明错误，不删） | 11 | — |
| 查缺补充（高置信） | 33 | ~2,900 |

复核拦下了几处会删坏现役代码的原方案：

- `transition_gate._clear_quiet` 在活路径 `update():1719` 上，原定的删除区间包含它。
- `transition_gate.py:1769-1782` 是默认配置下零速边沿的唯一入口。
- `cascade_core` 的三个删除区间切在语句中间，照删会产生语法错误或删掉活代码。
- `pid_support.py:33`、`:94-98` 是真实的业务导入。
- 同步 `certify` 的声明把条件写反了：它在 R88 **关闭**时才执行。

**删除施工必须使用复核后的边界（每条的 `safe_delete` 字段），不能用初查给的行区间。** 原始结果保存在工作流记录里：
`C:\Users\han001\.claude\projects\D--1-project-Software-embodied-navigation\cf4281ed-546c-4cbe-bc61-993edf7cec03\subagents\workflows\wf_a6466c54-98a\journal.jsonl`

## 根因：三个“假开关”

1. **观测端口恒为空实现**。`common/observation.py:86-88` 的 `api(name)` 不管传什么名字，都返回同一个 `DisabledObservation`。所以全仓的 `_observer_api("rNN_...")` 探针都不会生效（30 多个导入点）。注意它不全是空操作：`consumer_call`、`receipt_copy`、`lock`、`parsed` 会把调用透传下去，删除时要在调用点内联。
2. **启动器把开关钉成常量**。`launch/main.py:81` 注入 `configs/launch/settings.json` 的 features：R88/R90/R91/R92/R104/R105/R106/R110/R114/R119/SCAN_SHORT_INTAKE 都设为 1，代码默认值却全是 "0"。`main.py:82-92` 再把 7 个探针强制设为 0。其中 6 个探针**全仓没有读取点**，只有 R107 被 C++ 读取（`r107_transport_probe.h:40`）。
3. **基线配置把实验开关钉成常量**：
   - `r73_minimal_diagnostic_baseline` 恒关，而且 `input_freshness.enabled()` 在代码里写死返回 False。
   - `diagnostic_observe_only` 恒关。
   - `r85_*` 的刹车类开关全部为 false。
   - `r85_auto_reference_contract`、`r86_prepared_feedback` 恒开。

## 复核中发现的真问题（不属于“多余”，属于 bug 或隐患）

| # | 位置 | 问题 |
|---|---|---|
| B1 | `simulation/dynanav/capture/lockstep.py:101-145` | 开启 `--nonrealtime-sensor-lockstep` 并使用现役追踪器（Native/PctScan）时会崩溃：`pipeline_hint` 读的属性只有旧追踪器才有，抛 AttributeError；目标排队时 `original_wall_deadline` 抛 KeyError。测试用 SimpleNamespace 替身，所以没暴露出来。 |
| B2 | `pid_support.observe_business_gate` 等诊断开关 | `diagnostic_observe_only` 打开后，一部分路径抛 RuntimeError，另一部分**悄悄绕过业务门**：`_diagnostic_business_observed` 跳过航段超时，`_observe_stop_wait` 提前放行转向停车，`admission_callback_rejected` 不再拒绝超预算回调。这个开关本身就是隐患，删除它比保留它更安全。 |
| B3 | `robot/unitree/bridge.py:993` | `except RuntimeError` 接不住 `runtime_context.load` 实际抛出的 ValueError、FileNotFoundError、JSONDecodeError。`--prepare` 遇到坏配置会直接打印 traceback，打不出拒绝收据。 |
| B4 | `navigation/base_node.py:1658` | `getattr(self, "cascade_last_row", {}).get(...)`：这个属性的初值是 None，所以默认值 `{}` 用不上，一旦执行到就会抛错。目前不可达，建议改成 `(... or {})`。 |
| B5 | `ros_node.py:214`、`async_admission.py:134` | 报错文案与实际条件相反：文案写“需要 R88 和 R116 都关闭”，代码实际要求 R88=1、R116=0。 |
| B6 | `navigation/README.md:76` | 文档说计时文件“只在选中 r74_teacher_substage_timing 时写”，实际正好相反：每次运行都会写 `teacher_control_substage_timing.json`，全仓没有读取方。 |
| B7 | `student.py:470` 与 `lower_loop_ack.py:31` | 同一个扩展对应两个不同的文件名（`student_yaw_measurement_feedback.py` 和 `yaw_measurement_feedback.py`）。即使打开开关，两条路径也对不上。 |
| B8 | `base_node.main()`（1796-1819） | 默认的 `scenario_path` 指向一个不存在的文件，这个独立入口按默认参数无法启动。 |

## 删除包（按依赖和风险分）

### 包 V1：不需要你决定，直接可做（低风险）

都是代码层面的不变量（写死的 False、零调用方、`return` 之后的代码），与启动入口和配置无关：

| 内容 | 估算行数 |
|---|---|
| `r73 minimal_baseline` 的全部分支：`input_freshness` 和 6 个消费文件、`scan_epoch_supervisor._minimal_*`、`sensor_gate` | ~550 |
| 零调用方的死代码：`known_scene_adapter` 的 ExactSourceJoin/MatcherWorker/validate_runtime_witness；`point_provenance` 的解包链；`mission/state_machine.py` + `route_regions.py`；`dynamic_stop_event` 的审计；`static_geometry.deterministic_business_chain_prefix`；`physical_gate.file_value_world`；`building_navigation` 里 `return` 之后的代码；`odom_adapter` 的 R60 身份链（订阅的话题没有发布者）；`speed_tiers.select_profile` | ~1,900 |
| `pid_admission_apply.py:544-582、621-637（自相矛盾的嵌套判断）、843-844` | ~45 |
| 观测端口：内联透传语义后删除 `common/observation.py` 和所有 `_observer_api` 调用点 | ~400 |
| 零散的已确认冗余（C10 的 10 条、DynaNav 的 `--interactive-forward-turn-mode`、`lidar.py:113-114`、`experiment.py:96-102` 两个 raise、客户端 `:1520-1521`、`:2177-2178` 等） | ~100 |
| `main.py:82-92` 中 6 个没有读取点的探针键（保留 R107），连同 settings.json 中的惰性键 | ~15 |

### 包 V2：取决于决定 D1/D2（入口与 R85）

只在“控制器一定经 `launch/main.py` 启动”和“不再支持关闭 R85”这两个前提下才成立：

- 环境变量默认为 "0" 的分支：R88 同步 certify、`_r88_admission` 为 None 的回退、旧的非短 intake receive 路径、R92/R104 前置条件、R103 等。
- R85 关闭时走的旧路径：`pid_admission_apply.py:465-700` 和 112-122、FrozenFence、`pending_path_commit`、`continuous_route`（约 450 行）、`base_node` 的非 R85 分支。

合计约 1,500 行。做法是先在 `pid_entrypoint.main` 加一道检查（缺少启动器注入或 R85 关闭时拒绝启动），再删除。

### 包 V3：取决于决定 D3（已放弃的实验）

- 主动刹车整条链：`transition_gate` + `cascade_core` 的 brake/quiet/unload/neutral/stance/taper，约 1,300 行。必须按复核后的边界删，保留 `_clear_quiet` 和零速边沿。
- student 动态 helper 和 `yaw_measurement_feedback.py`：约 560 行，需要补一条“未知扩展键即拒绝”。
- `diagnostic_observe_only` 全家：约 580 行，见 B2，建议删。
- `_r47` 计时：约 540 行，产出的文件没有读取方。
- `exploration_waypoints.py` 整个文件：约 2,600 行，连带 3 个测试文件，约 181 条测试。

### 包 V4：B1 的 lockstep 崩溃

二选一：修复（适配现役追踪器）或者删掉 lockstep 功能。
