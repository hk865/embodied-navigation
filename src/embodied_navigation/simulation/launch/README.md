# simulation/launch

本目录装配通用仿真进程：actor 就绪后再启动 Gazebo，由顶层 `demo.launch.py` 包含。

- [simulation.launch.py](simulation.launch.py)：声明 `run_dir`，用 `OpaqueFunction` 调 `components(run)`。
- [components.py](components.py)：先启动 `actor`（`embodied_navigation.robot.unitree.policy.worker`，以 `TEACHER_SOCKET` 和运行目录为参数）与 `sensor_bridge`（`ros_gz_bridge/parameter_bridge` 加载运行目录的 `sensor_bridge.yaml`）；再启动 `actor_ready` 屏障（[../wait_actor.py](../wait_actor.py) 检查 `worker_ready` 文件与 Unix socket 是否就绪）；屏障成功退出后才启动 `gz sim -s -r --headless-rendering`，失败则关停整个 launch。
- 被 [顶层 demo.launch.py](../../launch/launch/demo.launch.py) 作为 `simulation` 模块包含；进程由 `launch/launching.py` 的 `owned()` 托管，退出即整体关停。actor 解释器来自 `common.config.path('actor_python')`，`TEACHER_SOCKET` 必须由调用方（`launch/main.py` 等）提前注入。
- 屏障自身不 import ROS（`ros=False`），最多等 30 秒；它检查运行目录下的 `worker_ready` 文件中的 PID 是否仍存活，以及 socket 是否已是 Unix socket，两者都满足才放行 Gazebo。

注意：这里启动的是当前普通 Student/native 链（协议仍有 Teacher 历史命名）；Building 的 `truth_worker` 由 `launch/building.py` 单独启动，RoboGauge 的 `worker` 由 `navigation/robogauge/benchmark.py` 启动。本目录无直接验证组件编排或就绪屏障的专用测试。上层 CLI 参数检查见 [tests/unit/test_launch_cli.py](../../../../tests/unit/test_launch_cli.py)；`sensor_gate`、`pid_navigation` 节点的信号与 executor/context 关停测试见 [tests/unit/test_ros_shutdown_lifecycle.py](../../../../tests/unit/test_ros_shutdown_lifecycle.py)。这些测试不验证本目录组件实际可启动。
