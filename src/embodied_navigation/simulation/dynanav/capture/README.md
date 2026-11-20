# capture（DynaNav 采集探针实现包）

盘点日期 2026-11-20。本包是同级入口 [`../capture_go2_rgbd.py`](../capture_go2_rgbd.py) 的实现拆分：把一次 Isaac Sim 采集运行按「场景 → 传感器 → ROS → 物理/控制循环 → 收据」显式构造，普通 `import` 不创建运行目录、不启动 Isaac/ROS、不加载策略。

## 模块定位与架构层级

- 架构层级：七层中的第 1 层 —— 仿真（`simulation/`）。它负责场景、传感器仿真与仿真到 ROS 的桥，不包含 SLAM、规划、控制律或机器人执行内部实现。
- 输入：
  - 入口参数：`argv[1]=output`（本次 run 目录）、`argv[2]=scene`（USD stage），实验字典由 `DYNANAV_EXPERIMENT` 指向的 JSON 提供；
  - ROS 订阅：里程计 `experiment["odometry_topic"]`（默认 `/aft_mapped_to_init`，纯位姿后端显式置 `estimated_velocity=None`）与已批准指令 `/dynanav/nav/approved_cmd`；
  - Isaac 渲染输出：RGB/深度/CameraParams 标注器、`RtxLidar` 的原生 GenericModelOutput、PhysX 接触与关节读回。
- 输出：
  - ROS 话题：`/dynanav/imu`、`/dynanav/camera`、`/dynanav/camera_info`、`/dynanav/depth`、`/dynanav/lidar_filtered`、`/dynanav/lidar_static`、`/dynanav/lidar_perception_filtered`（learned mapper 存在时）、`/dynanav/lidar_rtx`、`/dynanav/lidar_rtx/complete_scan`（`DYNANAV_RTX_COMPLETE_SCAN=1` 时，`dynanav_sensor_msgs/CompleteLidarScan`）、`/clock`、`/dynanav/truth/base_odometry`（原生位姿诊断 + `record_simulation_truth`）；
  - 真值模式（`slam_pose_source=simulation_truth`）额外发布 `/dynanav/truth/cloud_deskewed`、`/dynanav/truth/imu_odometry`、`/dynanav/truth/control_odometry`，见下文「200 Hz 真值去畸变接入」；
  - run 目录产物：`result.json`（0.25 s 限频、tmp 原子替换）、JSONL journal（`camera_frames.jsonl`、`camera_render_pose.jsonl`、`rtx_lidar_frames.jsonl`、`interactive_waypoint_control.jsonl`、`policy_frames.jsonl`、`imu_stream.jsonl`、`simulation_pause.jsonl`，以及可选 `sensor_publish_trace.jsonl`）、`multiwaypoint05_overview_and_rgb.mp4`、`latest_overview.jpg`、`video_encoder.log`、`rtx_scan_0001/0010/0050.npz`；入口另在 `<output>/sensor_config/` 冻结 LiDAR 资产与 `provenance.json`。
- 上游 / 下游：上游唯一调用方是 `../capture_go2_rgbd.py`（显式构造本包各模块，`lidar`/`video`/`pause` 在函数内延迟导入）；下游是 `../truth_deskew.py`（真值位姿缓冲与去畸变）、`robot/unitree/robogauge/{runtime,contract}.py`（策略与力矩）、`navigation/robogauge/route_tracker.py`、`../native_waypoints.py`、`../pct_scan_waypoints.py`、`../session_collision.py`、`perception/depth.py`。
- 边界：不做点云建图、不做 SLAM 位姿估计、不写速度命令（只消费上游已批准的 `approved_cmd`）。本包没有 `configs/` 字面读取点（`grep -rn "configs/" capture/` 为空，参数全部来自实验字典与 argv/env）；唯一的碰撞契约由 `../session_collision.py` 按入口传入的路径读取。

## 当前文件夹结构

```text
src/embodied_navigation/simulation/dynanav/capture/
├── README.md          # 本文件
├── __init__.py        # 14 行，只重导出 CaptureContext/Receipts/record_failure
├── control.py         # 493 行，50 Hz 控制 tick、路线跟踪、命令新鲜度、停车与终止
├── frames.py          # 403 行，10 Hz RGB-D 帧流水线、RTX 点云发布与真值去畸变排队
├── lidar.py           # 574 行，RTX 旋转 LiDAR 包、complete-scan 证书与资产冻结
├── pause.py           # 70 行，native-route mailbox 暂停门
├── physics.py         # 379 行，0.005 s 物理步、时钟/力矩/安全门
├── policy.py          # 128 行，RoboGauge MoE-CTS 策略加载、动作与力矩
├── receipts.py        # 232 行，result.json、JSONL journal、故障收据
├── ros_interface.py   # 557 行，rclpy 节点、/dynanav/* 话题与订阅回调
├── scene.py           # 602 行，SimulationApp/USD 场景、Go2 spawn、接触与物理准备
├── sensors.py         # 749 行，相机/IMU/annotator/overview 安装与绑定校验
└── video.py           # 94 行，overview/RGB 视频的 ffmpeg 编码器
```

12 个 Python 文件、合计 4295 行（`wc -l`，2026-11-20，以 `git ls-files` 为准），无子目录。入口是同级的 `../capture_go2_rgbd.py`（766 行，只做 argv/experiment 配置、编排与失败收据）。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `__init__.py` | 包入口，仅重导出收据相关符号，导入本包会连带导入 `receipts`（标准库 + json），不会拉起 Isaac。 | `CaptureContext`、`Receipts`、`record_failure` 与 `__all__` |
| `control.py` | 50 Hz 控制 tick（入口每 4 个物理步调用一次）：组装 64 维原生状态、stand gate、消费 `/dynanav/nav/approved_cmd` 的请求与新鲜度 hold、调用路线跟踪器与策略动作，并实现停车/终止判定。 | `resolve_limits`、`create_route_controller`（`RouteTracker` / `NativeWaypointTracker` / `PctScanWaypointTracker`）、`annotate_policy_receipt`、`ControlLoop.tick/observe_imu/check_termination`；journal `interactive_waypoint_control.jsonl`、`policy_frames.jsonl`；终止读 `NAV_STOP_REQUEST`（默认 `/tmp/no_multistage_stop_request`）并要求 5 s 静止 + 1 s 驻车窗口 |
| `frames.py` | 10 Hz RGB-D 帧流水线：读取 RGB/深度/CameraParams 标注器、归档互动静帧、发布 ROS 深度/内参/RGB 消息、记录相机渲染位姿与自过滤收据、定期存 RGB-D 快照、写 overview/RGB 视频帧。RTX 点云发布与真值去畸变排队也在这里。 | `FrameStream.__init__/process/publish_rtx_pending/_process_rtx_frame`；journal `camera_frames.jsonl`、`camera_render_pose.jsonl`、`rtx_lidar_frames.jsonl`；扫描 1/10/50 存 `rtx_scan_%04d.npz`；真值模式经 `truth_pose_buffer` + `deskew_rtx_packet_to_world` 调 `ros.publish_truth_deskewed_and_lio` |
| `lidar.py` | RTX 旋转 Ouster LiDAR：保留原生逐点发射时间与 ring，只做固定 sensor→IMU 安装变换；含 complete-scan 资产/prim 认证、证书、包封装与 PointCloud2 序列化，以及官方 OS1 资产的本地冻结。Isaac 导入延迟到函数内，便于 CPU 测试。 | `LIDAR_VARIANT="OS1_REV6_32ch10hz1024res"`、`LIDAR_TOPIC="/dynanav/lidar_rtx"`、`COMPLETE_SCAN_TOPIC="/dynanav/lidar_rtx/complete_scan"`、`OUSTER_DTYPE`、`MAX_SCAN_POINTS=65536`、`pack_lidar_scan`、`pointcloud_message`、`complete_scan_message`、`CompleteScanCertificate`、`attest_complete_scan_asset`、`prepare_lidar_asset`、`RtxLidar`、`LidarPacket` |
| `pause.py` | native-route mailbox 暂停门：只读 `pause_request.json` 并写 `pause_status.json`，暂停期间只 spin ROS、不推物理/渲染/发布；进入/退出计数只作诊断，不构成新的运行准入。 | `FilePause.request`、`FilePause.wait`（输入 `tracker`/`cancelled`/`stop_requested`/`idle`）；journal `pause_enter`、`pause_exit`、`pause_request_invalid` |
| `physics.py` | 单次 0.005 s（200 Hz）物理步：写并回读关节力矩、执行物理（Fabric 或 USD 更新）、校验仿真时钟与 IMU 时钟、执行姿态/接触/速度/位置安全门，返回测量值。 | `Measurement` 数据类、`PhysicsStepper.step`；`_native_static_exploration` 只对显式静态交互实验收紧验收包络 |
| `policy.py` | 只支持 RoboGauge 45D MoE-CTS 体策略：加载运行时、核对默认关节位姿与 articulation 契约，按 50 Hz 生成动作与目标关节，并把目标关节映射为 Go2HV effort 命令、执行关节速度回读守卫。 | `load_policy`、`PolicyDriver.step`（`tcontrol = i*0.005`，0.1 s 后启用动作）、`compute_torque`（`go2hv_torque` + `joint_speed_observation`）；写 `result["policy"]`、`result["joint_speed_peak"]` |
| `receipts.py` | 定义 `<output>/result.json` 契约与只追加 JSONL journal；保存限频 0.25 s 避免 200 Hz 循环写数据卷，先写 `result.json.tmp` 再原子替换；提供失败收据与显式资源上下文。导入本模块不加载 Isaac/ROS/torch。 | `initial_result`、`Receipts.save/journal/close_journals`、`CaptureContext`、`record_failure`；`CaptureContext` 保存本次创建的资源与故障现场 |
| `ros_interface.py` | 独占 `rclpy` 初始化/销毁：创建节点、`/dynanav/*` 发布者、里程计与 approved_cmd 订阅者，把传感器数据编码成 ROS 消息；`sensor_qos` 默认 `qos_profile_sensor_data`，录制时可用 `DYNANAV_RECORD_RELIABLE=1` 换可靠 QoS，`DYNANAV_PUBLISH_TRACE=1` 时写发布边界诊断。 | `RosInterface`、`create_ros_interface`、`publish_imu_and_clock/publish_cloud/publish_rtx_cloud/publish_depth_and_info/publish_camera_info/publish_rgb/publish_truth/publish_truth_vio/publish_truth_deskewed_and_lio`、`spin_once/drain/destroy` |
| `scene.py` | 所有会改动 USD stage/session 或物理应用的动作：`SimulationApp` 创建（含 RTX 需要的 sync loads 与 multi-tick 设置）、headless display delegate、stage 打开与世界/坐标轴、地下室碰撞会话修复、Go2 articulation spawn、contact report 安装、物理与 articulation 初始化、静态支撑 preflight。 | `start_simulation_app`（`"sync_loads": bool(rtx_lidar)`、`supportMultiTickRate`、`fabricDefaultSimPeriod=1/200`、`RenderingManager.set_dt(0.005)`）、`install_headless_display_delegates`、`resolve_spawn`、`prepare_rtx_scene_stage`、`attach_prepared_rtx_stage`、`open_stage`、`spawn_articulation_asset`、`ContactMonitor`、`setup_physics`、`initialize_articulation`、`run_preflights`、`play`/`stop` |
| `sensors.py` | 物理传感器装置本身：相机安装几何与 authored USD 相机、Isaac IMU、replicator 的 RGB/深度/CameraParams 标注器（含原生帧绑定校验与 quarantine）、第三人称 overview 相机与渲染产品；只返回普通值对象与安装收据，不启动应用、不打开 stage。 | `CameraRig`、`RgbdRender`（`accept_rtx_rgb_frame`、`_verify_binding`、`quarantine_native_frame`）、`OverviewRender`、`create_camera_rig`、`create_imu_sensor`、`create_rgbd_annotators`、`overview_camera_view`、`create_overview`、`sensor_installation_receipt`；常量 `OVERVIEW_EYE_BASE`/`OVERVIEW_FOCUS_BASE` |
| `video.py` | overview + RGB 的 1024×512 面板编码：启动 `ffmpeg` 子进程（`libx264`、10 fps、`ultrafast`），叠加不变的 overlay 文本，每 10 帧存一张 `latest_overview.jpg`；成功/失败路径共享 close 语义。 | `OverviewVideo.__init__/write/close`、`returncode`；产物 `multiwaypoint05_overview_and_rgb.mp4`、`video_encoder.log` |
| `README.md` | 本文件：模块定位、逐文件职责、三条关键契约与验证入口。 | — |

## 三条关键契约

### 1. 暂停 mailbox（`pause.py`）

`capture_go2_rgbd.py` 只在跟踪器为 `native_route` 时构造**一次** `FilePause`，此后每个 `i % 4 == 0` 的物理步调用一次 `wait(...)`，并把 `paused_wall_s` 只加回循环调度起点（不改时间戳）。暂停只读 `<mailbox>/pause_request.json`（`{"paused": bool, "reason": str}`），把状态写 `<mailbox>/pause_status.json`。暂停期间不推进物理、不渲染、不发布，只 `tracker.poll_request()` 与 `idle(0.02)`；`pause_exit` 记录 `paused_wall_s`、前后计数快照与 `counters_unchanged`，只作诊断，不改变任何准入判定。非法请求写 `invalid_request` 并向上返回 `stop_requested`。

### 2. RTX 同步加载（`scene.start_simulation_app`）

只有显式 RTX LiDAR（`rtx_lidar`）才把 `sync_loads` 置真、打开 `supportMultiTickRate` 与 `perSensorTickTlas`，并把 Fabric 默认仿真周期固定为 `1/200`；随后 `RenderingManager.set_dt(0.005)`，保证传感器与 200 Hz 物理步共用同一时间基。非 RTX 路径保持原 `SimulationApp` 配置，不静默切换。

### 3. 200 Hz 真值去畸变接入

`slam_pose_source == "simulation_truth"` 时，入口用 `TruthPoseBuffer(camera_rig.imu_mount)`（[`../truth_deskew.py`](../truth_deskew.py)，340 行）每个 0.005 s 物理步 `record(actual_ns, pos, quat)` 一次，形成 200 Hz 真值位姿缓冲。`FrameStream.publish_rtx_pending` 在读到 RTX 包后：

1. 先按原生时间戳发布原始 `/dynanav/lidar_rtx` 点云；
2. 把包与 `[first_ns, last_ns]` 入队，等 `/clock` 已覆盖末点且缓冲覆盖该窗口后，调用 `deskew_rtx_packet_to_world` 做逐点真值去畸变；
3. 由 `ros.publish_truth_deskewed_and_lio` 先发同戳 `/dynanav/truth/imu_odometry`，紧接着发世界系 `/dynanav/truth/cloud_deskewed`（`camera_init` 系、`go2_imu` 子系）。

配套的 `/dynanav/truth/control_odometry` 由每 4 步一次的 `publish_truth_vio` 发布，供 PCT+SCAN 的 `simulation_truth` 位姿源消费。这条链路只在显式真值模式启用，默认 SLAM 模式不发布任何 `/dynanav/truth/*` 去畸变产物，也不把真值当作 SLAM 证据。

## 验证与测试入口

- 全量单测：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`；本包直接相关的有 `test_dynanav_capture_orchestration.py`（编排/清理/失败收据）、`test_simulation_pause.py`（暂停 mailbox）、`test_truth_deskew_and_mode.py`（真值缓冲与模式）、`test_dynanav_rtx_lidar.py`、`test_dynanav_rtx_complete_scan.py`、`test_dynanav_rtx_writer_timing.py`、`test_dynanav_camera_binding.py`、`test_dynanav_joint_readback.py`、`test_dynanav_odometry_source.py`、`test_dynanav_rtx_scene_preparation.py`、`test_receipt_completion_throttle.py`。
- 只跑本包相关的一组：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -p 'test_dynanav_*.py'`。
- 这些测试用替身跑真实编排入口，不启动 Isaac/ROS/GPU/ffmpeg；真实传感器时序、PhysX 接触与 RTX 帧仍需在 Linux Isaac 主机上按入口实际运行验证。
- 配置注意：本包没有 `configs/` 字面读取点（碰撞契约经 `../session_collision.py` 读）。`output` 必须位于 `$PROJECT_DATA_ROOT/embodied-navigation` 下；RTX 模式还需要 `DYNANAV_RTX_COMPLETE_SCAN=1` 与 `dynanav_sensor_msgs`（由 `scripts/run_dynanav_native_exploration.sh` 的 build prefix 提供），`prepare_lidar_asset` 会优先复用数据卷 `runtime_assets/sensors/` 下已核验的官方 OS1 资产。

## 相关入口

- 采集入口与失败收据：[capture_go2_rgbd.py](../capture_go2_rgbd.py)
- 真值去畸变实现：[truth_deskew.py](../truth_deskew.py)
- 实验配置与源码快照：[experiment.py](../experiment.py)
- Agent 交互航点：[native_waypoints.py](../native_waypoints.py)、[pct_scan_waypoints.py](../pct_scan_waypoints.py)
- 上层包说明：[DynaNav README](../README.md)
