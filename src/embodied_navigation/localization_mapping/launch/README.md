# localization_mapping/launch

本目录装配 `localization_mapping` 的 ROS 进程：一个 launch 文件加一个组件清单，由顶层 `demo.launch.py` 包含。

- [slam.launch.py](slam.launch.py)：只声明 `run_dir` 参数，用 `OpaqueFunction` 调 `components(run)`。
- [components.py](components.py)：按固定顺序返回 `sensor_relay`、`self_filter`、`camera_parameters`、`slam`、`odom_adapter`、`map_archive`、`global_matching`、`sensor_gate` 八个进程；`owned()` 会在任一进程退出时关停整个 launch，并把退出码记到 `component_exit.jsonl`。
- 其中 `sensor_relay`、`self_filter`、`sensor_gate` 三项来自 `perception.sensors`，不属于本模块；`self_filter` 把 `/livox/lidar` 重映射成 `/demo/teacher/raw_lidar`。`slam` 直接执行 `install/ros/lib/fast_livo2_ros/fastlivo_mapping`，并把 `/cloud_registered_full` 改到原始名 `/demo/slam/raw_registered_full`；`odom_adapter` 对应 [../odom_adapter.py](../odom_adapter.py)，`map_archive` 对应 [../mapping/map_archive.py](../mapping/map_archive.py)，`global_matching` 对应 [../localization_runtime.py](../localization_runtime.py)。
- 被 [顶层 demo.launch.py](../../launch/launch/demo.launch.py) 作为 `localization` 模块包含；顶层入口是 `start.py` 调 `launch/main.py`。
- `run_dir` 指向 `launch/prepare.py` 准备好的运行目录；`components()` 只读取其中 `runtime_context.json`、`navigation_scenario.json`、`navigation_camera.yaml`、`navigation_fastlivo.yaml` 等已有文件。ROS domain、DDS、`DEMO_RUN_DIR` 等环境由 `launch/main.py` 统一注入，不在本目录设置。

本目录无专用测试；[tests/unit/test_launch_cli.py](../../../../tests/unit/test_launch_cli.py) 覆盖的是通用启动 CLI，不验证这八个组件是否真能启动。普通 `start.py` 启动会先由全局文件前检拦截缺失的 `install/ros`、`install/actuators` 等产物；若直接调用本 launch，`slam` 需要配置的 ROS install 中的 `fastlivo_mapping`，`camera_parameters` 另依赖 `/opt/ros/jazzy` 下的 `demo_nodes_cpp` 节点。
