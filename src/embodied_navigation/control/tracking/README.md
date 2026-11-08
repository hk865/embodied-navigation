# control/tracking

本目录主要实现跟踪控制计算：把 SLAM 位姿、IMU 与 SCAN 轨迹换算成有界机体速度，并完成航向门、地形检查等几何判断；另含策略源码与模型路径选择适配。

## 边界

- 做：`cascade_core.py` 的 actual-SLAM/IMU/SCAN cascade；`control_core.py` 的轨迹跟随、姿态倾角、航向门、加速度限幅、障碍走廊判断；`pid_core.py` 的角度归一化与 slew 限幅；以及规划上坡、路线进度、楼梯限速等辅助判断。
- 控制计算不发布 ROS 或执行策略动作，也不负责任务状态机；节点协作见 `../../navigation/README.md`。本包并非所有函数都无 IO：`policy_selection.py` 会检查文件存在性并通过配置辅助函数读取本地配置。

## 关键文件

- `cascade_core.py`：`Controller` 输出 bounded velocity，不输出动作；时间全部是调用方给的原生整数纳秒。命令界与 slew 来自 `../../../../configs/control/settings.json` 的 `cascade.command_limits`（`[0.6, 0.2, 0.6]`）与 `command_slew_per_s`（`[0.6, 0.6, 0.8]`/s），反馈 TTL 0.3 s；gyro 样本必须不晚于 pose，二者相差 ≤0.02 s。20 ms 是因果配对间隔，300 ms 是来源新鲜度，两者职责不同。文件里还有若干显式 profile 扩展（R43 heading-P taper、R85 neutral-rate 等），默认关闭。
- `control_core.py`：`follow_trajectory`（位置反馈到 SCAN 几何，先对准航向再走）、`RawImuTilt`/`body_tilt`（只做本体倾角安全，原始姿态不改 SLAM 位姿或航向）、`HeadingGate`（一次转向只锁一个目标航向）、`obstacle_ahead`/`steering_obstacle_ahead`（在 swept body 走廊里找真实 LiDAR 回波）、`remove_go2_self_returns`、`trajectory_has_progress`（拒绝紧急停车样条冒充路线）。常量 `MAX_TURN_RATE=0.12`、`MAX_WALK_YAW_RATE=0.08` rad/s。
- `pid_core.py`：`wrap` 做角度归一化，`slew` 根据前次/目标命令、dt 与加速度限制命令变化；该文件本身不读取 SLAM，也不计算位置 PID。
- `planned_step_filter.py`：`PlannedStepObstacle` 只在原始 body 障碍门中解释计划上坡/地面，未知区域保留原门禁。
- `route_progress.py`：`ConnectedRouteProgress` 在连续 3D 折线上维护单调弧长进度，支持折返/闭环。
- `terrain_speed.py`：`StairSpeedLimit` 在计划楼梯几何附近把巡航速度压到 0.25 m/s（只用路线几何，不用地形或真值传感器）。
- `policy_selection.py`：当前策略与模型位置的绑定；`replan_policy.py`：区分保护性零命令与 SCAN 几何耗尽（V24 Teacher 用）。

## 输入输出与调用

输入是 SLAM 位姿/旋转/速度、IMU、SCAN 采样、点云、路线与目标；主要控制计算输出 `[vx, vy, wz]` 与诊断字典；路径选择等辅助接口另有本地文件访问。主要调用方是 `../../navigation/pid_navigation.py`、`../../navigation/ros_node.py`、`../../navigation/building_navigation.py` 与 `../../planning/reference/`。

## 测试

`../../../../tests/regression/test_planned_step_filter.py`、`test_route_progress.py`、`test_terrain_speed.py` 分别覆盖对应文件。`cascade_core.py` 与 `control_core.py` 没有专用单测，只随导航运行链路使用，改动需自行补测。

## 限制

控制律只在仿真/离线环境使用；完整路线、跨楼层与实机未验证。
