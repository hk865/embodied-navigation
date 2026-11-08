# perception/sensors：感知侧 ROS 传感流适配

本目录把相机、LiDAR、IMU 与 SLAM 反馈适配成导航可用的 ROS 话题、健康门和有界交接结构，属于感知侧运行组件，不是顶层数据契约层。

## 关键文件与接口

- `sensor_relay.py`：把 `/livox/imu`、`/demo/camera` 原样转发到 `/demo/teacher/slam/imu`、`/demo/teacher/slam/image`，保留字段与原始时间戳；`--mode sync|async` 只改这两条 writer 的 `publishMode`，并写 `sensor_relay_loaded.json` / `sensor_relay_exit.json`。
- `self_echo_filter.py`：把 `/livox/lidar` 过滤后发到 `/demo/slam/lidar_filtered`，用固定 URDF/scenario 外参和机体包络剔除 Go2 自身点；不读 SLAM 位姿、世界轨迹或真值。
- `sensor_gate.py`：订阅 IMU、原始 LiDAR、图像、CameraInfo、SLAM body odom、配准点云，校验帧、尺寸、K/零 D、图-Info 50 ms 配对、时间戳递增与发布者图，写 `navigation_sensor_gate.json` 并在 `/demo/teacher/navigation/sensor_health` 发布 `ready`/`hold`/`failed`；`WarmupRecovery` 负责初始化与恢复预热。
- `feedback_mailbox.py`：有界不可变 corrected-message 交接（24 slot、16 frame、32 MiB 上限）；intake 组做 join/校验，control 组做反序列化与原始业务回调。
- `ready_commit.py`：环境变量 `R92_READY_COMMIT`（未设置环境变量时默认 `0`；当前普通导航的 `configs/launch/settings.json` 配置为 `1`）控制；用 guard condition 在现有 control 组触发一次 commit，不新增控制 tick。
- `sampling.py`：仅在 fresh、未启动的生成场景上应用已审核采样变体（64 线、480 水平采样、10/20/30 Hz），只改 world 的四个允许叶子；`metadata_qos.py` 决定 corrected metadata 订阅深度。
- `async_writers.py`：`R119_METADATA_ASYNC`（及 body 扩展）在 `rclpy.init` 前预载命名 publisher profile，并用 `writer_qos.so` 读实际 QoS。

## 调用与依赖

[localization_mapping/launch/components.py](../../localization_mapping/launch/components.py) 以子进程启动 relay、self_filter 与 sensor_gate；[launch/prepare.py](../../launch/prepare.py) 调用 `override_sensors`；[localization_mapping/known_scene_inputs.py](../../localization_mapping/known_scene_inputs.py) 与 `navigation/` 使用反馈邮箱、回执和订阅深度。依赖 [common/runtime_io.py](../../common/runtime_io.py)、[common/runtime_telemetry.py](../../common/runtime_telemetry.py)、[common/integration_limits.py](../../common/integration_limits.py)。

## 测试与限制

本目录无功能专用测试；`sensor_gate.py` 仅被 [tests/unit/test_ros_shutdown_lifecycle.py](../../../../tests/unit/test_ros_shutdown_lifecycle.py) 作为 AST 源码检查关停生命周期，其余模块未直接覆盖。运行需要完整 ROS 图与原始 `/livox`、`/demo` 话题；本会话未启动 ROS/仿真。
