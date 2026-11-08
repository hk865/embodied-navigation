# control/safety

本目录是仿真安全门禁与停车反馈来源检查：对同一帧的 typed 仿真安全状态给出 fault/stop/reason，并拒绝不合法的停车控制来源。

## 边界

- 做：`simulation_gate.py` 作出仿真安全决策；`feedback_source.py` 校验停车使用的 canonical SLAM 反馈；`clock_hold.py` 处理重复/回退时钟；其余文件做障碍解除窗口、IMU 交接、门禁原因码、bridge 安全交接与命令约束。
- 不做：未提供实机安全后端；`SimulationSafetyGate` 不能接受 `REAL_SENSOR` 来源，`../../common/state_contract.py` 中该类型仍为保留接口。本体接触在 `simulation_gate.py` 中仅作诊断，不新增停车条件；子包还包含独立的 SLAM/IMU/时钟保护。
- 安全状态的来源是仿真真值（`gazebo.native64.preupdate` / `gazebo_simulation`），只属于仿真安全边界，不能作为 SLAM 或导航定位输入。

## 关键文件

- `simulation_gate.py`：`SimulationSafetyGate.evaluate` 保持原阈值、锁存顺序与宽限：策略非有限值 → 执行器故障 → 倾倒/低离地；1.5 s / 2 s 宽限、0.8 / 1.1 rad 倾角、0.15 / 0.10 m 离地、4 秒持续倾倒观察窗。来源、run、时钟域或采样时间不符时立即 `reject_state` 并请求故障阻尼。默认 Student/Teacher 循环与 RoboGauge 循环逐帧调用同一门禁。
- `feedback_source.py`：`require_canonical_slam_feedback` 只接受 `frame_id=camera_init`、`pose_producer=fast_livo2_async_backend`、`navigation_ground_truth_used=false` 的反馈；header/callback 用 ROS 时间，接收时间用单调墙钟，两者不互相相减。
- `clock_hold.py`：`ControlClockHold.before_control` 在到达、航向、PI 与 native guard 之前拦截重复/回退时钟，只发精确零。
- `clear_guard_window.py`：`ClearGuardWindow.observe` 要求真实新鲜几何观测（10 个样本、1 s 跨度、单次间隔 ≤0.3 s）才释放障碍保持。
- 其余：`duplicate_source.py`（短重复源保留）、`bridge_safety_handoff.py`（固定交接）、`contact_mailbox.py`（只传接触值的非阻塞 IPC）、`guard_audit.py`（拦截两个原始 `obstacle_ahead` 结果）、`handoff_stop_policy.py`、`imu_causal_buffer.py`、`imu_mailbox.py`、`launch_command_policy.py`、`gate_reasons.py`（bridge veto 稳定码）、`safety_configuration.py`（固定停车、位姿采样、时钟与 settle 设置的校验函数）。

## 测试

- `../../../../tests/unit/test_simulation_safety.py`：门禁的故障优先级、宽限与来源拒绝。
- `../../../../tests/regression/test_native_safety_integration.py`：用真实观测构造加假 socket，验证 Student/Teacher 与 RoboGauge 两个循环逐帧调用门禁、动作不变。
- `../../../../tests/regression/test_navigation_safety.py`：障碍解除窗口的真实样本与缓存拒绝。

## 限制

无实机安全验证，也没有实机后端；仿真安全状态只覆盖仿真安全边界，完整路线与实机仍未验证。系统级说明见 `../../../../docs/architecture.md`。
