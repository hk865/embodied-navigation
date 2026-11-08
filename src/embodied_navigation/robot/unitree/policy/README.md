# robot/unitree/policy：策略适配与底层控制环

本目录承接当前 Student 策略运行：[worker.py](worker.py) 是 actor 子进程入口，复用 native 循环并注入 Student 推理、指令预适应与 ACK；[native/README.md](native/README.md) 提供 247D 中间观测与 native64 循环。

## 关键文件与接口

- `worker.py`：`python -m embodied_navigation.robot.unitree.policy.worker --run <run> --socket <sock>`，由 [simulation/launch/components.py](../../../simulation/launch/components.py) 以 `actor` 子进程启动。读取 `runtime_context.json` 的 `actor_kind`，复用 `native/worker.py` 循环并构造 `StudentRuntime`；当前普通导航要求 `student`，历史 `teacher` 选项会被显式拒绝。
- `student.py`：`exact_student45(state, command, previous_action)` 从 native64 状态计算 45D Student 输入；`CorrectionOnlyPID` 是 Student 的 COM 速度 PID；`StudentRuntime` 校验 45D、GRU、0.25 动作 scale、50 Hz 等契约并加载导出。`observation_frame()` 同时收到 247D Teacher 观测，但只作诊断（`teacher247_used_for_student_inference=False`），实际推理用 exact45。
- `lower_loop_ack.py` 是 Student 底层控制环 ACK 契约；`student_xy_stop_takeover_ack.py`、`yaw_measurement_feedback.py` 是可选控制扩展，均为显式模式。
- `speed_tiers.py`：只允许 0.2 / 0.5 m/s 两档，改的是 Student 纵向命令上限（0.3 / 0.6 m/s）与 0.65 m planning horizon，不改 PID 增益或 slew。

## 输入输出与依赖

输入是 run 目录、socket 与 profile；输出是 12 维关节目标、状态/ACK 文件和可选 telemetry。依赖 [native/README.md](native/README.md) 观测/循环、[control/tracking/policy_selection.py](../../../control/tracking/policy_selection.py)、[simulation/safety_state.py](../../../simulation/safety_state.py)。

## 测试与限制

本目录无专用单测；[native/worker.py](native/worker.py) 由 [tests/regression/test_native_safety_integration.py](../../../../../tests/regression/test_native_safety_integration.py) 用真实观测与假 socket 覆盖（247D），`StudentRuntime` 与实际权重导出未被 CPU 测试直接执行。运行需要外部模型/导出与可构建环境；无实机验证。
