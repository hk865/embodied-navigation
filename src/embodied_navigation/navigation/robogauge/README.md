# navigation/robogauge

本目录是 RoboGauge 仿真部署的导航侧：fail-closed 的命令准入、bounded Building 基准编排与离线证据审计；它不是主相对 SLAM 导航任务。

## 边界

- 做：把 run 内导航命令按同 run、边界与新鲜度准入（`commands.py`）；准备、预检与显式运行 Building 基准（`benchmark.py`）；离线审计同 stamp 的执行器证据（`analyze.py`）；独立的 native-pose 路线跟踪辅助类（`route_tracker.py`，有回归用例；当前 Building 基准启动 `building_navigation`）。
- 不做：策略训练、45D 观测构造与执行器 socket。它们分别在 `../../robot/unitree/robogauge/` 与 `../../simulation/gazebo/robogauge/`。

## 45D 与 247D 是两条链

RoboGauge 的 45D 是本部署的直接观测基准接口（`../../robot/unitree/robogauge/contract.py` 的 `observation()`，输入 state64/command3/last_action12）；既有 native 适配链还会构造 Teacher 的 247D 观测。当前默认 Student 的实际推理输入也是 45D（`../../robot/unitree/policy/student.py`），247D 在该 Student 路径仅作诊断，不能把 247D 称作 Student 的实际输入。两者共享同一仿真安全门禁，但不是同一策略输入或 actuator 契约，不能互相替代。

## 关键文件

- `commands.py`：`admit_command`/`read_command` 校验 `run_id`、命令形状与界（`[0.6, 0.2, 0.3]`）、源龄 `< 0.3 s`、接收墙钟龄 `< 0.3 s`；`admit_command` 对非法记录抛异常；`read_command` 捕获列出的读取/校验异常，返回零命令并附原因。
- `benchmark.py`：`--check` 只做文件/hash 预检，`--prepare-only` 只写 CPU 准备文件，`--run` 才启动 ROS/Gazebo；编排 actor、sensor bridge、FAST-LIVO2、backend、localization、`building_navigation`、SCAN 与 Gazebo。
- `analyze.py`：离线审计 `telemetry.jsonl` 与 `actuator_physics.jsonl` 的 policy exchange/physics step；推理 receipt 不等于实际运动。
- `route_tracker.py`：`RouteTracker` 纯控制器，没有仿真或策略副作用。

## 测试

`../../../../tests/regression/test_robogauge_contract.py`、`test_robogauge_evidence.py`、`test_robogauge_route_tracker.py`、`test_robogauge_benchmark.py` 分别覆盖命令准入/native 契约、证据审计、路线跟踪与基准准备；`test_robogauge_native_loop.py` 覆盖 robot/simulation 侧的 native 循环。

## 限制

仓库根的 install 构建产物缺失只表示未构建，不代表缺源码。CPU 预检不是执行证据；完整 Isaac 运行、GPU 与实机均未验证，历史 N=1 结果不是可靠性统计。详见 `../../../../docs/robogauge/README.md`。
