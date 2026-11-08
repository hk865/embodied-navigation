# control

本目录是控制侧的实现集合：单步跟踪控制律、停车阶段的测量与零速度契约、仿真安全门禁，以及供 launch 组装的进程入口；它不含任务状态机，也不生成参考路线。

## 边界

- 做：把 canonical SLAM 位姿、IMU、点云与 SCAN 轨迹换算成有界机体速度；维护停车窗口与零输入 ACK 契约；在仿真安全门禁上给出 fault/stop/reason；检查停车反馈来源。
- 不做：任务级状态机（在 `../navigation/task.py` 与 `../navigation/mission/`，见 `../navigation/README.md`）、路径规划与准入（在 `../planning/README.md`）、进程装配（见 `launch/README.md`）。
- 不提供实机安全后端；`safety/` 包含仿真物理门禁，以及 SLAM 来源、时钟和 IMU 等运行保护，见 `safety/README.md`。

## 关键文件

- `relative_pid.py`：`follow_segment` 从相对 SLAM 段生成纯驾驶指令，不发布 ROS，由 `../navigation/task.py` 使用。
- `collision.py`：`certified_support`、`braking_sweep`、`depth_collision`，分别做支撑栅格、制动扫掠与深度碰撞检查。
- `executor_command.py`：`CommandReader` 校验生产者、来源时钟与 `command_after_slew` 绑定；机器人侧 worker 读取命令时使用。
- `publication_ledger.py`：只读记录实际发布前后的命令与时钟，由 `../navigation/pid_navigation.py` 调用。
- `io.py`：运行状态与最新命令写入器的构造入口，被 bridge 与节点复用；当前 `make_evidence_writer` 返回 `RuntimeTelemetry`，历史 `append/enqueue` 为空操作，`atomic` 仍写实时状态。

子目录 `tracking/`、`parking/`、`safety/`、`launch/` 各有独立 README。

## 输入输出与调用

输入是 `/demo/slam/body_odom` 的 canonical SLAM 反馈（`pose_producer=fast_livo2_async_backend`、`navigation_ground_truth_used=false`）、IMU、点云、SCAN 轨迹与路线/目标；输出是有界速度、零命令与实时状态；代码保留 JSONL 证据接口，但当前默认 `RuntimeTelemetry` 不写这些历史记录。`launch/components.py` 用 `pid_navigation`、`robot.unitree.bridge`、`mission_runtime` 三个受管进程组成控制组：bridge 把 `/demo/cmd_vel` 落成 `navigation_command.json`，策略 worker 再读取执行。

## 测试

- `../../../tests/unit/test_control.py`：`relative_pid.follow_segment` 与原表达式 1000 组逐位等价，另覆盖支撑/制动/碰撞边界。
- `../../../tests/unit/test_simulation_safety.py`：仿真安全门禁的 fault/stop 判定边界。
- `../../../tests/regression/test_navigation_safety.py`：停车零契约、10Hz 位姿栅格与障碍解除窗口。

## 限制

控制律与门禁都是仿真/离线可用代码；完整路线、Building 实际运行与实机未验证。子包中仍有大量 profile/环境变量扩展（模块内默认关闭，部分由 launch 的 features 打开），改动前先确认对应配置项。
