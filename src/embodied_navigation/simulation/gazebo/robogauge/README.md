# simulation/gazebo/robogauge

本目录是 RoboGauge MoE-CTS 的 Gazebo 仿真部署：准备真实 SLAM benchmark 的输入、用 native request64/response16 回路驱动机器人，并记录接触与命令证据。

## 关键文件

- [world.py](world.py)：`prepare(case,label,mode='slam',backend_capture=False)` 只接受真实 SLAM（拒绝 truth/capture 模式），从 `configs/robogauge` 的 route、`world.sdf.in`、传感器契约与 FAST-LIVO2 配置生成运行目录；`actuator_path()` 指向 `install/robogauge/lib/librobogauge_actuator.so`。
- [worker.py](worker.py)：独立 actor 进程，不导入 `StudentRuntime`；用 `robot/unitree/robogauge` 的 `Runtime` 推理，读命令文件并写 `native_pose_source.jsonl`、`actual_command_ack.jsonl`。
- [native_loop.py](native_loop.py)：无 ROS 的 socket 主循环，含 `prepare_velocity_input`、`recv_exact`、`atomic_json`，逐帧调用 `control.safety.simulation_gate`；仅用于仿真。
- [motor_src/README.md](motor_src/README.md)：Go2HV 原生执行器源码与 CMake。

- 配置来源：`configs/robogauge/deployment_profile.json`、`world.sdf.in`、`sensor_bridge.yaml`、`sensor_contract.json`、`terrain_targets.json` 与 `routes/<case>.json`；`CASES` 共 7 个，短 case 超时 160 s、其余 600 s。
- `prepare()` 要求 label 目录不存在（`exist_ok=False`），写入 `runtime_context.json`、`PCT_route.json`、`world.sdf`、`experiment_setup.json` 等；模式只允许真实 SLAM。

## 调用与测试

`navigation/robogauge/benchmark.py` 的 `process_commands` 把本目录的 `worker` 与 Gazebo、FAST-LIVO2、backend、Building 导航装配成完整进程链；`preflight` 只查文件与哈希，不代表可运行。
专用回归：[tests/regression/test_robogauge_benchmark.py](../../../../../tests/regression/test_robogauge_benchmark.py)、[test_robogauge_native_loop.py](../../../../../tests/regression/test_robogauge_native_loop.py) 与 [test_native_safety_integration.py](../../../../../tests/regression/test_native_safety_integration.py)，都使用 fixture 或假 socket，不启动 Gazebo。Gazebo 原生 state64 中的姿态、角速度和关节量用于策略观测；COM 线速度与地形不进入 MoE-CTS 观测，位置、速度和接触等还用于评分、安全与诊断。导航定位仍使用 SLAM 反馈，`navigation_ground_truth_used` 固定为 false。
