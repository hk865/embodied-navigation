# robot/unitree：Go2 部署层

`robot/unitree` 是 Unitree Go2 的部署实现层：`bridge.py` 把导航命令门控成执行文件，`policy/` 管理当前 Student 策略部署，`robogauge/` 管理 RoboGauge 45D 部署。

## 关键文件与接口

- `bridge.py`：进程角色 `command_bridge`（ROS 节点名 `teacher_scan_slam_bridge`），由 [control/launch/components.py](../../control/launch/components.py) 启动。订阅 `/demo/slam/body_odom`、`/demo/cmd_vel`，经 `CommandGate` 校验后把 Twist 原子写成 `navigation_command.json`，并发布 `/demo/teacher/navigation_bridge/status`、`/stop`。`TIMEOUT = COMMAND_SOURCE_FRESHNESS_S`（300 ms），`LIMITS = (.3, .2, .3)`。它不订阅 Gazebo 真值，也不直接驱动关节；`validate_controller_source` 同时被 control 的重复源与停车因果代码复用。
- `policy/`：历史 Teacher native 中间观测（247D，当前用于诊断）与 Student 推理、底层控制环 ACK 扩展，见 [policy/README.md](policy/README.md)。
- `robogauge/`：RoboGauge MoE-CTS 的 45D 直接观测、CPU TorchScript 推理与 Go2HV 数值契约，见 [robogauge/README.md](robogauge/README.md)。

## 两条部署路径不要混用

- `policy/native` 保留历史 Teacher 的 247D 中间观测适配；当前普通导航要求 Student，不加载 Teacher actor。Student 实际推理走 `policy/student.py` 从同一 native64 状态算出的 exact45，247D 仅作诊断。
- `robogauge` 的 45D 是独立直接观测接口，有自己的 scale 与关节顺序，不是 247D 的裁剪，也不含 Student 的 COM 速度 PID。
- 两者不能当成同一个策略输入或 actuator 契约。

## 测试与限制

本层无独立目录测试；`policy/native` 与 `robogauge` 分别由 [tests/regression/test_native_safety_integration.py](../../../../tests/regression/test_native_safety_integration.py)、[tests/regression/test_robogauge_contract.py](../../../../tests/regression/test_robogauge_contract.py) 等覆盖。`bridge` 是集成边界，不代表导航成功；没有实机后端，本会话未启动仿真或机器人。
