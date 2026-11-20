# simulation/dynanav：DynaNav / Isaac 场景接入与实测导航数据面

盘点日期 2026-11-20。本目录负责办公室 DynaNav 场景的 Isaac 会话、Go2 RGB-D/IMU/RTX LiDAR 采集、200 Hz 仿真真值去畸变、PCT/SCAN sidecar 与 operator map 客户端；SLAM、规划、控制算法本体仍在各自模块。

## 模块定位与架构层级

- 架构层级：第 1 层「仿真」的 DynaNav 实现子包，同时向上提供 Agent 探索所需的数据面（观测话题 + mailbox）。
- 输入：场景 USD 与机器人资产（数据卷）、`experiment.json` profile 与 CLI 参数、ROS 侧 `/dynanav/nav/approved_cmd` 与停车请求、FAST-LIVO2 输出的点云/里程计、operator 通过 mailbox 下发的 `request.json`。
- 输出：`/dynanav/*` 传感器与真值话题（见下）、run 目录收据（`result.json`、JSONL、视频、`experiment.json`/`source_manifest.json`/`source_snapshot/`）、mailbox 里的 `observation.json`/`receipts.jsonl`/`planner_*` 文件。
- 上游 / 下游：上游是 [scripts/run_dynanav_native_exploration.sh](../../../../scripts/run_dynanav_native_exploration.sh) 与 [scripts/run_dynanav_experiment.py](../../../../scripts/run_dynanav_experiment.py)；下游是 FAST-LIVO2/后端（`localization_mapping/`）、PCT/SCAN worker（`planning/`）、RouteTracker（`navigation/robogauge/`）与 RoboGauge 执行（`robot/`）。
- 边界：本目录不实现 SLAM 前端/后端、不实现 PCT/SCAN 算法、不实现控制律；`operator_map.py` 只把**实测**点云与估计位姿变成占据/frontier 快照，拒绝真值来源；仿真真值只在显式 `--truth-pose` 模式进入导航，且明确不称为 SLAM。

## 当前文件夹结构

```text
src/embodied_navigation/simulation/dynanav/
├── README.md                    # 本文件
├── __init__.py                  # 1 行：包 docstring
├── capture/                     # 采集实现包（11 个 .py + README），见 capture/README.md
├── capture_go2_rgbd.py          # 766 行：采集显式入口，argv/experiment 配置与编排
├── collision_bounds.py          # 72 行：导出非机器人碰撞体包围盒证据
├── experiment.py                # 536 行：实验 CLI、profile、源码快照、runpy 入口
├── native_waypoints.py          # 529 行：native RouteTracker 的 mailbox 适配
├── operator_map.py              # 3185 行：operator 侧实测占据/frontier 图（纯 NumPy）
├── pct_scan_waypoints.py        # 254 行：PCT/SCAN 路线跟踪器与连续 SCAN 跟随
├── pct_scan_worker.py           # 782 行：PCT → native SCAN ROS sidecar
├── render_diagnostics.py        # 241 行：只读 USD/Fabric 渲染状态收据
├── rtx_deskewed_perception.py   # 282 行：FAST-LIVO2 去畸变点云消费契约
├── session_collision.py         # 279 行：会话级碰撞修复 + PhysX 支撑 preflight
└── truth_deskew.py              # 340 行：200 Hz 真值位姿缓冲与 RTX 去畸变
```

## 进程与数据流

1. **启动**：`scripts/run_dynanav_native_exploration.sh` 创建新 run 目录、冻结配置、构建原生 SCAN 闭包，然后拉起估计器（默认）、SLAM 后端（默认）、采集进程、PCT/SCAN worker 与 operator map 客户端。
2. **实验入口**：`scripts/run_dynanav_experiment.py` → `experiment.main()` 组装 profile、写出 `experiment.json` 与源码快照，设置 `DYNANAV_EXPERIMENT`/`PROBE_STEPS`/`PROBE_RTF`/`NAV_STOP_REQUEST` 后用 `runpy` 执行 `capture_go2_rgbd`。
3. **采集**：`capture_go2_rgbd.run()` 按 config → session → ROS → physics/control loop → receipts 顺序构造 [capture/](capture/README.md) 模块；200 Hz 物理、50 Hz 策略、10 Hz RGB-D，RTX LiDAR 10 Hz。
4. **估计与规划**：默认模式订阅 `/dynanav/rtx_livo/cloud_registered_full`、`/dynanav/rtx_livo/aft_mapped_to_init`、`/dynanav/rtx_livo/post_vio_odometry`；`pct_scan_worker` 用 PCT 规划全局路线，再把参考交给 **native SCAN** 子进程，只跟随真实 `Bspline` 消息。
5. **航点消费**：采集侧按 `--interactive-controller` 选择 `native_route`（`NativeWaypointTracker` 包装原 `RouteTracker`）或 `pct_scan`（`PctScanWaypointTracker` + `ContinuousScanFollower`）；operator/Agent 通过 [scripts/explore_waypoint_client.py](../../../../scripts/explore_waypoint_client.py) 的 `heartbeat/goto/yaw/maneuver/scan/observe/map/stop` 子命令交互，命令 SOP 见 [.agents/skills/dynanav-explore/SKILL.md](../../../../.agents/skills/dynanav-explore/SKILL.md)。
6. **真值模式**：`--truth-pose` 跳过 SLAM，改订阅 `/dynanav/truth/{cloud_deskewed,imu_odometry,control_odometry}`，由 `truth_deskew.py` 用 200 Hz `go2_imu` 位姿对 RTX 点云逐点去畸变。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `__init__.py` | 包标记，1 行 docstring "DynaNav scene integration. Scene/texture assets are external data." | 无 |
| `README.md` | 本文件：目录职责、进程数据流、mailbox 契约与测试入口 | 索引页 |
| `capture/` | 采集实现包：receipts、scene、sensors、lidar、frames、physics、policy、control、ros_interface、pause、video；逐文件职责见 [capture/README.md](capture/README.md) | `capture_go2_rgbd.run_capture()` 显式构造 |
| `capture_go2_rgbd.py` | 显式入口：`CaptureConfig`、`load_experiment()`（只读 `DYNANAV_EXPERIMENT`）、`parse_config()`（`argv[1]` 输出目录、`argv[2]` 场景 USD；拒绝已退役模式与不受支持的语义开关）、`run()`/`main()`、`run_capture()` 编排、信号取消与失败收据；import 无副作用 | `run(argv)`、`main(argv)` |
| `collision_bounds.py` | `export_bounds(stage, out)`：在 authored time 0 遍历 `/World/Go2` 之外启用的碰撞 prim，把 mesh 点/authoring 范围变换为世界 AABB，平面单独记录，输出 JSON 证据 | `export_bounds()` |
| `experiment.py` | 实验 CLI：`make_profile()`（速度上界、时长、spawn、碰撞预算）、`configure_point_cloud_source()`（`rgbd`/`rtx_lidar`，RTX 强制 Fabric）、`slam_odometry_topic()`（LIO/post-VIO/真值话题绑定）、`joint_speed_observation()`（只报告不夹紧）、`main()`（`--check` 只校验资产与权重；实跑写 profile/源码快照后 `runpy`） | `main(argv)`、`make_profile()`、`slam_odometry_topic()` |
| `native_waypoints.py` | `NativeWaypointTracker`：把实测里程计和 mailbox `request.json` 的 goto/cancel/stop 适配到原 `RouteTracker`；`PROTOCOL_VERSION=1`，每个文件 revision 只消费一次；持续写 `observation.json` 与 `receipts.jsonl`；yaw 动作向 `planner_yaw_query.json` 查询空间；失败只终止当前目标，不终止传感/SLAM 会话 | `NativeWaypointTracker.tick()`、`poll_request()` |
| `operator_map.py` | Operator 侧实测占据/frontier 图（纯 NumPy/stdlib，模块级不 import rclpy/Isaac）：`PosePairer`（按 stamp 最近邻 + 线性/球面插值）、`LevelTracker`/`LevelGrid`（按实测 z 分层、稀疏占据、frontier 与 unknown 下界估计）、`OccupancyAccumulator`（多帧累积 + 时钟/新鲜度/地面证据门）、`build_map_snapshot()` 一次性快照；拒绝 `ground_truth`/`gt_mesh`/`oracle_mesh` 与裸旋转扫描 | `build_map_snapshot()`、`OccupancyAccumulator.add_frame()/snapshot()` |
| `pct_scan_waypoints.py` | `PctScanWaypointTracker` 继承 `NativeWaypointTracker`，只替换路线生产/跟随：写 `planner_request.json`、读 `planner_path.json`、记 `planner_provenance.jsonl`；`ContinuousScanFollower` 连续投影/前瞻跟随 native SCAN 曲线，不在样点处停车，只在最终目标做一次物理停车 | `PctScanWaypointTracker`、`ContinuousScanFollower.tick()` |
| `pct_scan_worker.py` | 独立 ROS sidecar（不 import Isaac）：订阅去畸变云/LIO/VIO 与 SCAN 的 `bspline`/`trajectory_metadata`/`odometry_consumption_ack`/`route_definition`/`reference_status`，线程池里跑实测分层 3D PCT（多层测量 tomogram），把实测地图写成 `planner_measured_map.npz`、路线诊断写 `planner_global_route.json`；每次请求只管理自己启动的 `OwnedScan` 子进程；`build_scan()` 只构建已 checkout 的原生 SCAN 包 | `main(argv)`、`run(args)`、`OwnedScan` |
| `render_diagnostics.py` | `capture_stage_receipt(stage, paths)`：在一次采集完成渲染后**只读** USD 与既有 Fabric 矩阵，返回可 JSON 化的场景状态收据；不刷新、不设 transform、不改 stage | `capture_stage_receipt()` |
| `rtx_deskewed_perception.py` | FAST-LIVO2 去畸变点云的生产消费契约：固定 topic 角色（`cloud_registered_full` + 同 stage `aft_mapped_to_init`）、拒绝 raw wire 布局、要求 `curvature` 里 1 ms 级逐点时间散布、要求同整数 stamp/frame/stage、`map_points_to_base()` 一次 map→base 变换、`state_age_gate()` 用 source/arrival 两个时间域 | `verify_topic_roles()`、`verify_cloud_layout()`、`verify_deskew_marker()`、`state_age_gate()` |
| `session_collision.py` | `load_config()` 读 `configs/simulation/dynanav_basement_collision.json`；`repair_basement_session()` 只在 USD 会话层启用原办公室地面 mesh、禁用 GroundPlane（不改资产文件）；`preflight_support()` 用实时 PhysX 射线核对配置采样点的真实支撑，失败抛 `SupportPreflightError` 并附完整报告 | `repair_basement_session()`、`preflight_support()` |
| `truth_deskew.py` | 200 Hz 仿真真值：`TruthPoseBuffer` 固定容量环形缓冲（记录 `go2_base` 世界位姿并换算到 `go2_imu`），`interpolate_imu_pose(s)` 线性位置 + 最短弧 SLERP；`deskew_rtx_packet_to_world()` 用逐点原生 firing time 把 RTX 包变换到 `camera_init`；`truth_odometry_message()`/`truth_deskewed_cloud_message()` 发布 `/dynanav/truth/*` | `TruthPoseBuffer`、`deskew_rtx_packet_to_world()` |

## mailbox 契约

| 文件 | 写入方 | 读取方 | 含义 |
| --- | --- | --- | --- |
| `request.json` | operator/Agent 客户端 | `NativeWaypointTracker.poll_request()` | goto/cancel/stop 请求，带 sequence/revision，每个 revision 只消费一次 |
| `observation.json` | `NativeWaypointTracker.tick()` | 客户端 `observe`、`pct_scan_worker` | 最近一次位姿/速度/状态快照（原子替换） |
| `receipts.jsonl` | `NativeWaypointTracker` | 客户端/复核 | 目标与事件的只追加证据 |
| `planner_request.json` | `PctScanWaypointTracker` | `pct_scan_worker` | 当前目标与 frame 契约 |
| `planner_path.json` / `planner_provenance.jsonl` | `pct_scan_worker` / `PctScanWaypointTracker` | `PctScanWaypointTracker` / 复核 | 规划路径与来源证据 |
| `planner_measured_map.npz`、`planner_global_route.json`、`planner_worker_parameters.json`、`planner_yaw_space.json` | `pct_scan_worker` | 客户端/诊断 | 实测地图、活动全局路线诊断（advisory only）、worker 参数与偏航空间查询答复 |
| `planner_yaw_query.json` | `NativeWaypointTracker` | `pct_scan_worker` | 独立的偏航空间查询，不伪造平移目标 |

## 验证与测试入口 / 配置注意

- 与上述模块直接对应的纯 CPU 测试：`tests/unit/test_dynanav_native_waypoints.py`、`test_dynanav_pct_scan_waypoints.py`、`test_dynanav_pct_scan_worker.py`、`test_dynanav_rtx_deskewed_perception.py`、`test_dynanav_rtx_complete_scan.py`、`test_dynanav_rtx_lidar.py`、`test_dynanav_rtx_writer_timing.py`、`test_dynanav_capture_orchestration.py`、`test_dynanav_experiment_summary.py`、`test_dynanav_source_snapshot.py`、`test_truth_deskew_and_mode.py`、`test_operator_map.py`、`test_operator_floor_capacity.py`、`test_operator_floor_transitions.py`、`test_operator_grid_export.py`；回归 `tests/regression/test_dynanav_session_collision.py`。
- 命令：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`、`PYTHONPATH=src python3 -B -m unittest discover -s tests/regression`；上述用例为纯 CPU 契约/编排测试，不启动 Isaac 仿真或 ROS graph（个别模块只做延迟导入与消息结构断言）。
- 实跑前置：Linux + Isaac Sim + ROS 2 Jazzy；场景 USD、机器人 USD、robogauge 权重与 FAST-LIVO2 二进制均为数据卷外部资产，构建见 [docs/dependencies/BUILD.md](../../../../docs/dependencies/BUILD.md)，前端配置见 [configs/simulation/dynanav/](../../../../configs/simulation/dynanav/README.md)。
- 所有输出必须写 `$PROJECT_DATA_ROOT/embodied-navigation/` 下的新目录；仓库不保存运行产物。
