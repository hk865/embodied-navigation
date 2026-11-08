# control/parking

本目录实现停车阶段的控制律与契约：从“到达末端点”到“5 秒零速度保持”的测量、握手、ACK 与失败判定，全部基于实测位姿与零输入证据。

## 边界

- 做：判定停车窗口是否成立（`parking_contract.py`、`parking_pose10hz.py`）、零输入事件与帧授权（`parking_zero_contract.py`）、末端朝向推导（`terminal_heading_contract.py`）、过渡段判据（`transition_gate.py`）、1 秒 settle 证据（`navigation_stop_window.py`），以及若干默认关闭的扩展（`integral_unload.py`、`terminal_owner.py`、`source_hold.py`、`causal_join.py`）。
- 不做：不判定任务成功/失败（由 `../../navigation/pid_navigation.py` 决定），不产生路线，不提供物理安全；零速度只由测量与 ACK 契约确认，不靠计时器推定静止。

## 关键文件

- `parking_contract.py`：`enabled(profile)` 校验 `ordinary_terminal_parking_contract`。T0 是首次 10Hz 几何捕获加实际零输入 ACK；`duration_ns=5e9` 是固定 5 秒窗口；`capture_xy_m=0.025`、`capture_yaw_rad=0.035`、`capture_dwell_ns=600000000`（0.6 s） 是捕获容差与驻留；`source_age_ns=300000000`（0.3 s） 是来源新鲜度。
- `parking_pose10hz.py`：`PoseGrid`/`CaptureGrid`/`from_feedback` 按 100 ms 栅格取“该时刻之前最近的不同 header 位姿”，不插值；5 秒窗口共 51 个采样，漂移上限 0.05 m / 0.1 rad。
- `parking_zero_contract.py`：`make_event`、`validate_event`、`authorize_frame`、`before_actor_observation`、`validate_zero_ack`，把事件、帧与 ACK 绑定到同一实际输入。
- `terminal_heading_contract.py`：由当前 route 的 approach 几何计算末端朝向；named start/goal connector 的短假航向被排除。
- `ordinary_parking.py`：`OrdinaryParking` 在原始 SLAM/IMU/ACK 门禁之后接管末端，不改变路线成功、不启动正式窗口、失败不重试。
- `transition_gate.py`：`TeacherHeadingGate` 只用新鲜 SLAM 与原始 body gyro 证明停/转向；不用真值、纯计时静止、CHAMP 相位重置或 IMU 航位推算。仿真实验允许 profile 转向上限到 0.6 rad/s，NAV 0.6 尚无物理验证。
- `navigation_stop_window.py`：`SPEC` 定义 1 s settle 统计（gyro 均值/峰值/RMS、euler 峰峰值与净变化、xy 位移，单位 rad、rad/s、m）。在合格的普通 settle 阶段，它与原 300 ms legacy 停止判据是 OR 关系；3 mm / 1.5 mrad 等统计界限并非所有停车必须额外通过的 AND 门槛。动态恢复仍使用 legacy 判据。

## 输入输出与调用

输入是 canonical SLAM 反馈、paired IMU、实际零输入 ACK、route/goal 与 profile；输出是停车阶段判定、零命令与证据字典；通过当前默认 `RuntimeTelemetry` 调用的历史 JSONL 写入为空操作。主要调用方为 `../../navigation/pid_navigation.py`、`../../navigation/building_navigation.py` 与 `../tracking/cascade_core.py`。`R105_TERMINAL_PARKING_OWNER`、`R106_PREQUALIFICATION_SOURCE_HOLD` 等环境开关在模块内默认 0，launch 的 features 中为 1。

## 测试

- `../../../../tests/unit/test_parking_source_contract.py`：真实 SLAM 反馈保持非真值、时钟保留；真值、外来 producer、缺失字段都被拒绝。
- `../../../../tests/regression/test_navigation_safety.py`：10Hz 位姿栅格、零契约、末端朝向与障碍解除窗口；该文件没有直接测试 `safety_configuration.py`。

## 限制

没有覆盖整条停车链的测试，也没有实机或完整路线验证；私有几何数据缺失时部分回归用例会 skip。
