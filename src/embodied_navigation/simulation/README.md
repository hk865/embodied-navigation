# simulation：Isaac Sim / DynaNav 仿真层

盘点日期 2026-11-20。仿真层负责 Isaac Sim 场景生命周期、Go2 传感器采集与仿真到 ROS 的桥；当前只有 DynaNav 一条活跃路径，具体实现集中在 [dynanav/](dynanav/)。

## 模块定位与架构层级

- 架构层级：七层中的第 1 层 —— 仿真（场景/资产/传感器仿真 + 仿真到 ROS 的桥）。
- 输入：场景 USD 与机器人资产（数据卷）、实验 profile（`experiment.json`）与 CLI 参数、ROS 侧已批准指令与停车请求。
- 输出：`/dynanav/*` 传感器与真值话题（`/dynanav/imu`、`/dynanav/camera`、`/dynanav/depth`、`/dynanav/camera_info`、`/dynanav/lidar_rtx`、`/dynanav/truth/{cloud_deskewed,imu_odometry,control_odometry,base_odometry}`）、run 目录下的 `result.json`/JSONL/视频，以及 FAST-LIVO2、SLAM 后端、PCT/SCAN worker 和 operator map 的输入。
- 上游 / 下游：由 [scripts/run_dynanav_native_exploration.sh](../../../scripts/run_dynanav_native_exploration.sh) 统一拉起；下游是 `localization_mapping/`（前端 FAST-LIVO2 与后端）、`planning/`（PCT/SCAN worker）、`navigation/`（RouteTracker）与 `robot/`（RoboGauge）。
- 边界：不做 SLAM/建图、不做路线规划、不做策略训练、不做任务状态机；仿真物理读回（姿态、接触、关节速度）只作安全门与诊断，仿真真值只在显式 `--truth-pose`/诊断模式发布，永远不重标为 SLAM。仿真层也没有实机或硬件驱动入口。

## 当前文件夹结构

```text
src/embodied_navigation/simulation/
├── README.md        # 本文件：层职责、话题与数据边界
├── __init__.py      # 1 行：包 docstring（"Simulation and scene lifecycles..."）
└── dynanav/         # 唯一子包，见 dynanav/README.md
```

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `__init__.py` | 包标记，仅 1 行 docstring "Simulation and scene lifecycles, independent of navigation algorithms."；无 import、无副作用 | 无 |
| `README.md` | 本文件：仿真层定位、ROS 接口面与数据边界 | 索引页 |
| `dynanav/` | DynaNav/Isaac 实验路径：场景会话、RGB-D/IMU/RTX LiDAR 采集、真值去畸变、PCT/SCAN worker、operator map；逐文件职责见 [dynanav/README.md](dynanav/README.md)，采集编排实现见 [dynanav/capture/README.md](dynanav/capture/README.md) | `scripts/run_dynanav_experiment.py`、`scripts/run_dynanav_pct_scan.py` |

## 仿真到 ROS 的接口

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | 发布 | 采集循环按传感器时间发布，同批数据前先发时钟 |
| `/dynanav/imu` | `sensor_msgs/Imu` | 发布 | 默认 200 Hz；`--imu-hz 100` 复现旧抽样 |
| `/dynanav/camera` / `/dynanav/depth` | `sensor_msgs/Image` | 发布 | 512×512；深度为沿光轴的 `distance_to_image_plane` |
| `/dynanav/camera_info` | `sensor_msgs/CameraInfo` | 发布 | 与当帧光学参数一致 |
| `/dynanav/lidar_rtx` | `sensor_msgs/PointCloud2` | 发布 | 原始旋转 RTX LiDAR 包，保留原生 firing time 与 ring |
| `/dynanav/lidar_rtx/complete_scan` | `dynanav_sensor_msgs/CompleteLidarScan` | 发布 | 需 `DYNANAV_RTX_COMPLETE_SCAN=1` 与原生消息包，启动器负责 |
| `/dynanav/lidar_filtered` / `/dynanav/lidar_static` | `sensor_msgs/PointCloud2` | 发布 | 采集侧点云产品 |
| `/dynanav/truth/{cloud_deskewed,imu_odometry,control_odometry}` | `PointCloud2` / `Odometry` | 发布 | 仅 `--truth-pose` 模式：200 Hz 真值去畸变与真值里程计 |
| `/dynanav/truth/base_odometry` | `nav_msgs/Odometry` | 发布 | 仿真 base 真值，仅 `native_pose_diagnostic` 且 `record_simulation_truth` 时创建 |
| `/dynanav/nav/approved_cmd` | `geometry_msgs/TwistStamped` | 订阅 | 已批准 body 指令，经门控后才进入策略 |
| 估计器里程计（默认 `/aft_mapped_to_init`，或 post-VIO/真值话题） | `nav_msgs/Odometry` | 订阅 | 由 `--interactive-pose-source`/`--odometry-topic` 决定 |

## 与 ROS 及数据卷的边界

- **ROS 接口面**：仿真侧只作为发布者与少量订阅者。订阅 `/dynanav/nav/approved_cmd`（`TwistStamped`）与里程计话题（默认 `/aft_mapped_to_init`，或 post-VIO/真值话题）；发布上述 `/dynanav/*` 话题。停车请求走 run 目录的 `stop_request.json` 文件（`NAV_STOP_REQUEST` 指向），不占 ROS 话题。话题名、消息类型与 QoS 集中定义在 `simulation/dynanav/capture/ros_interface.py`，不在本文件重复实现。
- **真值边界**：`simulation_truth` 只出现在显式 `--truth-pose` 实验与隔离诊断中；发布到 `/dynanav/truth/*` 后由 operator/PCT-SCAN 显式声明消费，不能伪装成 SLAM 定位输出。
- **数据边界**：场景、机器人资产、策略权重、运行日志、点云、视频和逐帧 JSON 全部在数据卷 `$PROJECT_DATA_ROOT/embodied-navigation/` 下；仓库只保留代码与配置。渲染与采集输出目录必须位于数据卷内，且要求是新目录（`experiment.py` 会校验）。
- **进程边界**：采集进程、FAST-LIVO2 估计器、SLAM 后端、PCT/SCAN sidecar、operator map 客户端是彼此独立的进程，通过 ROS 话题与 mailbox 目录通信；仿真层不拥有估计器或规划器的生命周期。

## 验证与测试入口 / 配置注意

- CPU 单测覆盖纯逻辑部分：`tests/unit/test_dynanav_*`（采集编排、传感器时钟、RTX LiDAR/writer、真值去畸变、native/PCT-SCAN waypoints、operator map 等）与 `tests/regression/test_dynanav_session_collision.py`；命令 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 完整仿真验证需要 Linux + Isaac Sim + ROS 2 Jazzy，并显式提供场景 USD、机器人 USD 和 robogauge 权重；构建与依赖见 [docs/dependencies/BUILD.md](../../../docs/dependencies/BUILD.md)。
- 默认入口：`bash scripts/run_dynanav_native_exploration.sh [NEW_RUN_DIRECTORY]`；`--truth-pose` 切换到真值位姿 + 200 Hz 真值去畸变 RTX 点云驱动 PCT+SCAN 的隔离验证模式。启动器会构建原生 SCAN 闭包、起 FAST-LIVO2/后端/PCT-SCAN worker，并把本次配置冻结到 run 目录。
- 仿真侧配置：场景碰撞选择在 `configs/simulation/dynanav_basement_collision.json`，前端 YAML 在 [configs/simulation/dynanav/](../../../configs/simulation/dynanav/README.md)，由启动器复制到 run 目录后再使用，见 [configs/README.md](../../../configs/README.md)。
