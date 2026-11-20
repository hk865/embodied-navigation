# 测试入口、边界与逐文件职责

项目自有的自动化验证都在本目录：两条 `unittest` 发现根（`tests/unit`、`tests/regression`）加一组需要 ROS 2/CMake 的原生 fixture（`tests/native/`）；vendored 包自带测试在 `third_party/` 内，不在此处。盘点日期 2026-11-20，基线 `refactor/06-clean-base`（HEAD `5b880b2`）。

## 模块定位与架构层级

- 架构层级：不属于七层运行栈（仿真 / 感知与通信 / 定位建图 / 规划 / 控制 / 机器人执行 / Agent 接口），而是横跨全仓的**离线验证层**。
- 输入：被测包（`PYTHONPATH=src` 下的 `embodied_navigation`）、`scripts/` 薄 CLI、源码文本与 AST、少量外部构建产物（已安装的 `fast_livo2_core`/`plan_env`/SCAN prefix）。
- 输出：`unittest` 的 `Ran N tests`/`OK`/`FAILED` 与退出码；原生目录输出 `ctest` 退出码。测试不写仓库内文件，临时目录一律 `tempfile`。
- 上游 / 下游：没有生产代码 import 测试；由开发者或 CI 手动执行。测试反向依赖生产代码，并额外用源码文本/AST 断言锁住入口闭包。
- 边界：不启动 Isaac Sim 或任何仿真器、不做 GPU 推理、不连接机器人、不发导航目标；仿真与实机验收不属于测试套件。原生 CTest 通过也只证明 fixture 契约，不是物理路线验收。

## 当前文件夹结构

```text
tests/
├── README.md            # 本文件：命令、环境/skip 语义、逐文件职责
├── unit/                # 61 个 test_*.py + 7 个 *_fixtures.py（共享 helper，非发现根）
├── regression/          # 5 个 test_*.py，跨模块契约与入口闭包
└── native/              # C++/ROS 原生 fixture，独立 CMake/CTest 或单独脚本，不在两条 unittest 命令内
    ├── dynanav_lio/           # CMakeLists.txt + README.md + 10 个 .cpp（9 个 CTest + 1 个回放工具）
    ├── dynanav_pct_scan/      # ros_smoke.py（可选合成 ROS 冒烟）
    ├── local_initialization/  # acceptance.cpp + run_acceptance.py
    ├── yaw_consumption/       # ros_consumption_fixture.py + test_ros_consumption.py
    └── yaw_rotation/          # CMakeLists.txt + rotation_acceptance.cpp
```

逐文件清单见下一节：93 个 git 跟踪文件全部在表内（含本文件）。

## 逐文件职责

### `tests/unit/` —— 61 个 `test_*.py`，646 条用例（本次实测 0 失败 0 跳过）

发现根是 `tests/unit`；`discover` 只收集 `test_*.py`，因此 `*_fixtures.py` 只被 import、不被当作用例文件。61 个文件中 60 个被 `unittest` 收集（`test_livo_yaw_sampler_policy.py` 例外，见文末“已知缺口”），用例数来自 2026-11-20 本机加载结果。

| 文件 | 核心职责 | 用例数 | 关键接口/入口 |
| --- | --- | ---: | --- |
| [`test_data_paths.py`](unit/test_data_paths.py) | `project.data_root()` 必须解析到 `$PROJECT_DATA_ROOT/embodied-navigation`，缺失时报可操作 `RuntimeError` | 2 | `DataPathTests` |
| [`test_depth_packet.py`](unit/test_depth_packet.py) | `perception.depth` 深度/点云/像素对应关系 | 6 | `DepthPacketTests` |
| [`test_dynanav_camera_binding.py`](unit/test_dynanav_camera_binding.py) | `capture/frames` 相机帧绑定与 SDK 回调（attr/节点/ann 桩） | 33 | `BindingTests`、`SdkCallbackTests` |
| [`test_dynanav_capture_orchestration.py`](unit/test_dynanav_capture_orchestration.py) | 拆分后 `capture_go2_rgbd` 的编排层：零步/单步/首 tick 失败、清理二次失败、receipt 写入失败、publish trace、native 关闭边界 | 24 | `CaptureOrchestrationTests`、`FailureBoundaryTests`、`PublishTraceTests`、`NativeShutdownBoundaryTests` |
| [`test_dynanav_experiment_summary.py`](unit/test_dynanav_experiment_summary.py) | `scripts/summarize_dynanav_experiment.py`：记录运动指标排除站立，不推断任务成功 | 8 | `DynaNavSummaryTests` |
| [`test_dynanav_exploration_tools.py`](unit/test_dynanav_exploration_tools.py) | `scripts/dynanav_exploration.py` 与 `scripts/dynanav_exploration/` 的离线入口/快照契约 | 12 | `ExplorationToolTests` |
| [`test_dynanav_freshness_receipt.py`](unit/test_dynanav_freshness_receipt.py) | `capture/control.py` 新鲜度收据字段（执行真实赋值，不启动 Isaac/ROS） | 1 | `FreshnessReceiptTests` |
| [`test_dynanav_joint_readback.py`](unit/test_dynanav_joint_readback.py) | `experiment.joint_speed_observation`、`capture/policy.compute_torque`、`capture/scene.initialize_articulation` 与 RoboGauge `contract` 的关节读回/力矩换算 | 5 | `JointReadbackTests`、`CaptureJointSpeedTests` |
| [`test_dynanav_native_waypoints.py`](unit/test_dynanav_native_waypoints.py) | `dynanav.native_waypoints` 与 `navigation.robogauge.route_tracker` 的航点协议、实测来源 | 11 | `NativeWaypointsTest` |
| [`test_dynanav_observer_transport.py`](unit/test_dynanav_observer_transport.py) | observer UDP 传输配置选择与 preflight/launch 收据；不启动 ROS、不分配 socket | 8 | `ObserverTransport` |
| [`test_dynanav_odometry_source.py`](unit/test_dynanav_odometry_source.py) | `capture_go2_rgbd` 的里程计源与入口闭包（源码文本/AST 断言） | 5 | `CaptureOdometrySource` |
| [`test_dynanav_pct_scan_waypoints.py`](unit/test_dynanav_pct_scan_waypoints.py) | `dynanav.pct_scan_waypoints`：mailbox 协议、来源新鲜度、连续 SCAN 跟随 | 13 | `PctScanMailboxTests`、`ContinuousScanTests` |
| [`test_dynanav_pct_scan_worker.py`](unit/test_dynanav_pct_scan_worker.py) | `dynanav.pct_scan_worker`：原生样条求值与 active goal 关联 | 4 | `PctScanWorkerTests` |
| [`test_dynanav_post_vio_output.py`](unit/test_dynanav_post_vio_output.py) | post-VIO 发布的源码级接线检查（真实发布由原生 fixture 覆盖） | 1 | `PostVioOutputWiring` |
| [`test_dynanav_route_information.py`](unit/test_dynanav_route_information.py) | `planning.exploration.route_information` 虚拟扫描收益；无仿真/真值 | 12 | `RouteInformationTests` |
| [`test_dynanav_rtx_complete_scan.py`](unit/test_dynanav_rtx_complete_scan.py) | capture 完整扫证书契约（mock GMO/ROS）；不是 RTX 运行验收 | 14 | `CompleteScanTests` |
| [`test_dynanav_rtx_deskewed_perception.py`](unit/test_dynanav_rtx_deskewed_perception.py) | `dynanav.rtx_deskewed_perception`：安装变换、raw wire 布局、same-stage 配对、age gate | 11 | `MountTransformTests`、`RawWireAndLayoutTests`、`SameStagePairTests`、`AgeGateTests` |
| [`test_dynanav_rtx_lidar.py`](unit/test_dynanav_rtx_lidar.py) | `capture/{frames,lidar,sensors}`：RTX LiDAR 线格式、帧、容量与原生 RGB 帧 | 16 | `LidarWireTests`、`RtxFrameTests`、`LidarCapacityTests`、`NativeRgbFrameTests` |
| [`test_dynanav_rtx_production_bridge.py`](unit/test_dynanav_rtx_production_bridge.py) | 去畸变 LIO 云 → `operator_map` + perception 的生产桥 | 13 | `DeskewedBridgeTests` |
| [`test_dynanav_rtx_scene_preparation.py`](unit/test_dynanav_rtx_scene_preparation.py) | RTX 场景准备：实例先在内存展开再填充 renderer stage | 5 | `RtxScenePreparationTests` |
| [`test_dynanav_rtx_source.py`](unit/test_dynanav_rtx_source.py) | RTX source 选择与来源（含 RoboGauge 默认模型路径校验） | 7 | `RtxSourceSelection` |
| [`test_dynanav_rtx_writer_timing.py`](unit/test_dynanav_rtx_writer_timing.py) | capture writer/队列时序观测（合成 GMO parser） | 6 | `NativeWriterTimingTests` |
| [`test_dynanav_sensor_clock.py`](unit/test_dynanav_sensor_clock.py) | `experiment.float32_sensor_time_tolerance` 与浮点时间舍入 | 2 | `SensorClockTests` |
| [`test_dynanav_slam_reconstruction.py`](unit/test_dynanav_slam_reconstruction.py) | `scripts/reconstruct_dynanav_slam_map.py` 的配对与重建证据 | 8 | `PairingTests`、`ReconstructionTests` |
| [`test_dynanav_source_snapshot.py`](unit/test_dynanav_source_snapshot.py) | 来源快照必须覆盖实际执行查询的源码（semantic_mapping 查询、dynanav capture） | 2 | `SourceSnapshotTest` |
| [`test_dynanav_unlimited_acceptance.py`](unit/test_dynanav_unlimited_acceptance.py) | 无总时长预算的交互会话验收（native/pct 航点 + route_tracker + RoboGauge runtime） | 10 | `UnlimitedSessionAcceptance` |
| [`test_dynanav_waypoint_candidates.py`](unit/test_dynanav_waypoint_candidates.py) | `planning.exploration.waypoint_candidates`：候选只建议、不提交控制、不声称实测自由空间 | 9 | `CandidateTests` |
| [`test_explore_waypoint_client.py`](unit/test_explore_waypoint_client.py) | `scripts/explore_waypoint_client.py` 协议、CLI、地图流、时钟复核与配对预览（本目录最大文件 1858 行） | 98 | `ClientProtocolTests`、`LocalActionCliTests`、`OperatorMapParsingTests`、`ProductionMapStreamTests` 等 13 个类 |
| [`test_layered_mapping.py`](unit/test_layered_mapping.py) | `localization_mapping.semantic_mapping` 分层地图、调度器与 query CLI | 20 | `LayeredMapTests`、`SchedulerTests`、`QueryCliTests` |
| [`test_livo_yaw_sampler_policy.py`](unit/test_livo_yaw_sampler_policy.py) | 原始云采样策略（pytest 风格模块级函数）；**不被 `unittest discover` 收集**，见"已知缺口" | 0 | 模块级 `test_*` 函数 |
| [`test_measured_pct_tomogram.py`](unit/test_measured_pct_tomogram.py) | `planning.pct.measured_tomogram` 对真实原生 PCT A* 的实测格网用例（合成返回） | 12 | `MeasuredPctTests` |
| [`test_native_camera_recovery.py`](unit/test_native_camera_recovery.py) | 无效静态帧恢复：隔离坏帧但不放松已接受绑定 | 3 | `NativeCameraRecoveryTests` |
| [`test_native_reverse_waypoint.py`](unit/test_native_reverse_waypoint.py) | 显式反向航点的独立 CPU 契约（route_tracker + native/pct 航点） | 10 | `ReverseRouteTest`、`ReverseAdapterTest` |
| [`test_operator_floor_capacity.py`](unit/test_operator_floor_capacity.py) | operator_map 高度图容量：20 层支撑图可容纳、`LevelTracker` 64 层上限报 `too_many_levels` | 1 | `FloorChartCapacityTests` |
| [`test_operator_floor_transitions.py`](unit/test_operator_floor_transitions.py) | `operator_map` 上楼时实测支撑面的层切换证据 | 2 | `FloorTransitionTests` |
| [`test_operator_grid_export.py`](unit/test_operator_grid_export.py) | operator grid 导出与只读 PNG 渲染（PIL） | 11 | `OperatorGridExportTests` |
| [`test_operator_map.py`](unit/test_operator_map.py) | `dynanav.operator_map` 全量离线：位姿配对、外参、点云声明、层跟踪、floor/clearance、actionable 门、free mode、地面证据、锚点（本目录最大单文件 2292 行） | 105 | 12 个类，含 `PosePairerTests`、`LevelGridTests`、`ActionableGateTests` |
| [`test_pct_grid_budget.py`](unit/test_pct_grid_budget.py) | 真实原生 PCT 与真实 worker/CLI 分发的预算契约 | 10 | `NativeGridBudgetTests`、`WorkerBudgetTests`、`LauncherBudgetTests` |
| [`test_perception.py`](unit/test_perception.py) | `perception.depth.depth_to_cloud` 投影、自车过滤、降采样的逐元素等价 | 1 | `DepthTests` |
| [`test_pose_velocity_contract.py`](unit/test_pose_velocity_contract.py) | `perception.pose_velocity` 与 native 航点的源时速度窗契约 | 5 | `PoseVelocityContract` |
| [`test_process_log_routing.py`](unit/test_process_log_routing.py) | `capture/scene` 的 Kit 日志路由：有界启动器显式关闭、其他入口保留文件日志 | 3 | `KitLogRoutingTests` |
| [`test_receipt_completion_throttle.py`](unit/test_receipt_completion_throttle.py) | 收据完成节流从真实写入（含原子替换）完成后才开始 | 1 | `ReceiptCompletionThrottleTests` |
| [`test_scan_diagnostic_storage.py`](unit/test_scan_diagnostic_storage.py) | 诊断存储改动不改变固定原生跟随决策 | 2 | `ScanDiagnosticStorageTests` |
| [`test_scan_handoff_worker_acceptance.py`](unit/test_scan_handoff_worker_acceptance.py) | 经真实 `pct_scan_worker` ROS 回调的黑盒 handoff 结果 | 10 | `ScanHandoffWorkerAcceptance` |
| [`test_scan_source_staging_acceptance.py`](unit/test_scan_source_staging_acceptance.py) | `planning.scan_source` 的 patch-only 源码暂存验收（隔离 vendored 树） | 9 | `ScanSourceStagingAcceptance` |
| [`test_simulation_pause.py`](unit/test_simulation_pause.py) | `capture/pause` 原生 mailbox 暂停契约；不启动 Isaac/ROS | 8 | `PauseTests` |
| [`test_slam_backend_backend.py`](unit/test_slam_backend_backend.py) | `localization_mapping.backend.backend` 端到端：建图与重定位（冻结 `SlamBackend.ingest` API） | 4 | `SlamBackendMappingTests`、`SlamBackendRelocalizationTests` |
| [`test_slam_backend_config_and_imu_patch.py`](unit/test_slam_backend_config_and_imu_patch.py) | SLAM 后端严格配置加载（`BackendConfig.from_dict/from_json`）、`anisotropic_information` 各向异性信息矩阵、`node.py --config`、`0011` IMU 偏置协方差补丁 | 20 | `BackendConfigLoadingTests`、`AnisotropicInformationTests`、`ImuBiasCovariancePatchTests` 等 6 个类 |
| [`test_slam_backend_correction.py`](unit/test_slam_backend_correction.py) | `backend.correction` 的 map→odom 准入 | 4 | `CorrectionAdmissionTests` |
| [`test_slam_backend_map_store.py`](unit/test_slam_backend_map_store.py) | `backend.map_store` 地图持久化格式与内容 | 2 | `MapStoreTests` |
| [`test_slam_backend_pose_graph.py`](unit/test_slam_backend_pose_graph.py) | `backend.pose_graph` SE(3) 位姿图求解精度 | 2 | `PoseGraphTests` |
| [`test_slam_backend_registration.py`](unit/test_slam_backend_registration.py) | `backend.registration` point-to-plane ICP | 3 | `RegistrationTests` |
| [`test_slam_backend_scan_context.py`](unit/test_slam_backend_scan_context.py) | `backend.scan_context` 位置识别与配准联动 | 5 | `ScanContextTests` |
| [`test_slam_backend_spool.py`](unit/test_slam_backend_spool.py) | `backend.keyframes` 的 spool 读取 | 9 | `SpoolTests` |
| [`test_truth_deskew_and_mode.py`](unit/test_truth_deskew_and_mode.py) | `dynanav.truth_deskew` 200 Hz 真值去畸变、truth/mode 隔离契约（含 `common.geometry`） | 14 | `TruthPoseBufferTests`、`VectorizedDeskewTests`、`ModeAndIsolationContractTests` |
| [`test_yaw_blocked_latch.py`](unit/test_yaw_blocked_latch.py) | 阻塞旋转在后续在途样本到达前就关闭动作（真实 worker mailbox） | 1 | `YawBlockedLatchAcceptance` |
| [`test_yaw_clearance_feedback.py`](unit/test_yaw_clearance_feedback.py) | 真实控制器 mailbox 边界与作用域化原生净空响应（`YawRig` 驱动实测位姿适配器） | 8 | `YawClearanceFeedbackAcceptance` |
| [`test_yaw_delayed_cancel.py`](unit/test_yaw_delayed_cancel.py) | worker 取消权威性高于延迟到达的原生回复 | 1 | `DelayedYawCancelAcceptance` |
| [`test_yaw_request_lifecycle.py`](unit/test_yaw_request_lifecycle.py) | yaw 请求生命周期协议（实现评审前冻结，`YawRig` 驱动 mailbox） | 6 | `YawRequestLifecycleAcceptance` |
| [`test_yaw_shared_control.py`](unit/test_yaw_shared_control.py) | 声明的 `yaw_action` API：旧控制/死区/来源滞留语义 | 6 | `SharedYawControlAcceptance` |
| [`test_yaw_worker_bridge.py`](unit/test_yaw_worker_bridge.py) | 真实 `pct_scan_worker` 回调必须保留 owned-child 与采样顺序 | 2 | `YawWorkerBridgeAcceptance` |

### `tests/unit/` 共享 fixture（7 个，不以 `test_` 开头，不被发现为用例）

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [`duration_acceptance_fixtures.py`](unit/duration_acceptance_fixtures.py) | 时长验收用的实测 mailbox 装置（不含 ROS） | `MailboxRig` |
| [`pct_budget_acceptance_fixtures.py`](unit/pct_budget_acceptance_fixtures.py) | PCT 预算验收的 transport：真实 worker/planner，伪造的只有 ROS/进程所有权 | `BudgetWorkerRig`、`build_native`、`launch_without_external_processes` |
| [`scan_diagnostic_fixtures.py`](unit/scan_diagnostic_fixtures.py) | 诊断类改动的确定性观测与基线输出 | `make_follower`、`path_payload`、`command_sequence` |
| [`scan_source_acceptance_fixtures.py`](unit/scan_source_acceptance_fixtures.py) | 源码暂存契约用的最小隔离 vendored 树 | `make_vendor`、`replacement_patch` |
| [`scan_worker_acceptance_fixtures.py`](unit/scan_worker_acceptance_fixtures.py) | 执行真实 measured SCAN worker 的受控 ROS/进程 transport（fake ROS 模块 + 子进程封装） | `WorkerRig`、`Process`、`Publisher`、`Node` |
| [`slam_backend_fixtures.py`](unit/slam_backend_fixtures.py) | 合成世界、轨迹与 spool writer，供 `test_slam_backend_*` 共用 | `SyntheticWorld`、`transform_points`、`pose_error` |
| [`yaw_acceptance_fixtures.py`](unit/yaw_acceptance_fixtures.py) | 驱动真实 measured-pose 适配器与临时 mailbox 的 yaw 装置 | `YawRig` |

### `tests/regression/` —— 5 个文件，60 条用例（本次实测 0 失败，3 条按环境跳过）

| 文件 | 核心职责 | 用例数 | 关键接口/入口 |
| --- | --- | ---: | --- |
| [`test_dynanav_lio_launcher.py`](regression/test_dynanav_lio_launcher.py) | `scripts/run_dynanav_lio.sh` 的 argv/prefix/来源收据闭包，用假 ELF 依赖与假 runner，不启动真实 LIO | 33 | `ObserverOverride` |
| [`test_dynanav_session_collision.py`](regression/test_dynanav_session_collision.py) | `simulation.dynanav.session_collision` 场景查询准入与 floor 会话修复；本机无 CPU USD 绑定时 3 条 skip | 7 | `Query`、`SessionSupportAdmission`、`FloorSessionRepair` |
| [`test_robogauge_contract.py`](regression/test_robogauge_contract.py) | RoboGauge 45D 契约、`go2hv_torque` 与 policy runtime 数值 fixture（不需 torch/仿真） | 5 | `RoboGaugeContract` |
| [`test_robogauge_route_tracker.py`](regression/test_robogauge_route_tracker.py) | `navigation.robogauge.route_tracker` 的路线跟踪契约 | 14 | `RouteChecks` |
| [`test_snapshot_bridge.py`](regression/test_snapshot_bridge.py) | 真实 C++ `snapshot_bridge` spool 流由 D 路径后端读取（需 `g++`，缺失时 skip） | 1 | `SnapshotBridgeFlow` |

### `tests/native/` —— 19 个文件，独立 CMake/CTest 或单独脚本，不在两条 unittest 命令内

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [`native/dynanav_lio/CMakeLists.txt`](native/dynanav_lio/CMakeLists.txt) | 注册 9 个 CTest（`dynanav_lio_native`、`dynanav_post_vio_native`、`dynanav_ouster_native`、`dynanav_sync_observation_native`、`dynanav_same_pair_native`、`dynanav_complete_scan_native`、`dynanav_multicamera_shadow_native`、`dynanav_visual_snapshot_native`、`dynanav_multicamera_joint_native`）；`replay_multicamera_shadow` 只构建不注册 CTest；不下载依赖、不重建 core | `add_test` 9 项 |
| [`native/dynanav_lio/README.md`](native/dynanav_lio/README.md) | 原生 fixture 的构建/运行命令、历史 CTest 结果与边界（命令中的既有 prefix 用外部数据卷示例，复制时替换为 `$PROJECT_DATA_ROOT`；其目录表仍停留在早期文件数，以本表为准） | 人读 |
| [`native/dynanav_lio/test_lio_mode.cpp`](native/dynanav_lio/test_lio_mode.cpp) | 真实 `LIVMapper`/`VIOManager`：poisoned storage 上的生命周期、image-free LIO 显式 opt-in、sparse cloud 队列、IMU 传播点参考时间 | CTest `dynanav_lio_native` |
| [`native/dynanav_lio/test_post_vio_output.cpp`](native/dynanav_lio/test_post_vio_output.cpp) | post-VIO 发布契约的本地 pub/sub fixture：opt-in、原 image stamp、frame/status、非法样本拒绝 | CTest `dynanav_post_vio_native` |
| [`native/dynanav_lio/test_ouster_mode.cpp`](native/dynanav_lio/test_ouster_mode.cpp) | Ouster `type=3` 显式 opt-in、PointCloud2 字段/timing、serial ingress → preprocess → ONLY_LIO sync → IMU deskew | CTest `dynanav_ouster_native` |
| [`native/dynanav_lio/test_sync_observation.cpp`](native/dynanav_lio/test_sync_observation.cpp) | 复用 ouster fixture 的同步观测 fixture：point coverage/IMU 端点不足时等待且不消费 | CTest `dynanav_sync_observation_native` |
| [`native/dynanav_lio/test_same_pair_handoff.cpp`](native/dynanav_lio/test_same_pair_handoff.cpp) | 真实 `Process2`/`handleLIO`/`handleVIO` 的同时间 pair 交接 | CTest `dynanav_same_pair_native` |
| [`native/dynanav_lio/test_complete_scan.cpp`](native/dynanav_lio/test_complete_scan.cpp) | 完整扫证书/连续性/身份/阶段故障、owner-only 失败持久化、early-image 冷启动；可选 `COMPLETE_SCAN_REPLAY_BIN` 读 SHA 校验的存储片段 | CTest `dynanav_complete_scan_native` |
| [`native/dynanav_lio/test_multicamera_shadow.cpp`](native/dynanav_lio/test_multicamera_shadow.cpp) | 原生 FAST-LIVO2 多相机 shadow 分支（合成纹理几何，不声称传感器/GPU 结果） | CTest `dynanav_multicamera_shadow_native` |
| [`native/dynanav_lio/replay_multicamera_shadow.cpp`](native/dynanav_lio/replay_multicamera_shadow.cpp) | 多相机 shadow 回放工具（消费冻结快照，仅构建，不注册 CTest） | 可执行文件 |
| [`native/dynanav_lio/test_visual_snapshot.cpp`](native/dynanav_lio/test_visual_snapshot.cpp) | `visual_snapshot` 输出契约 fixture | CTest `dynanav_visual_snapshot_native` |
| [`native/dynanav_lio/test_multicamera_joint.cpp`](native/dynanav_lio/test_multicamera_joint.cpp) | 多相机联合更新的数值契约（gtest 风格） | CTest `dynanav_multicamera_joint_native` |
| [`native/dynanav_pct_scan/ros_smoke.py`](native/dynanav_pct_scan/ros_smoke.py) | 可选的合成 ROS 冒烟：启动真实 `scripts/run_dynanav_pct_scan.py` worker，注入 clock/LIO/VIO/cloud，校验 revision、取消与紧急保持；需 `DYNANAV_SCAN_TEST_PREFIX` 与隔离 `ROS_DOMAIN_ID` | `main()` |
| [`native/local_initialization/acceptance.cpp`](native/local_initialization/acceptance.cpp) | 对机械提取的真实 SCAN 局部初始化块做数值验收（close-goal guard、多项式轨迹） | `main()` |
| [`native/local_initialization/run_acceptance.py`](native/local_initialization/run_acceptance.py) | 从 staged SCAN 树提取 `reboundReplan` 的局部初始化/guard 片段并编译验收，可选与 baseline 源码对比 | `--scan-source`、`--output`、`--baseline-source` |
| [`native/yaw_consumption/ros_consumption_fixture.py`](native/yaw_consumption/ros_consumption_fixture.py) | 真实 ROS 传输 + 故意源发布延迟的装置 | `RosConsumptionRig` |
| [`native/yaw_consumption/test_ros_consumption.py`](native/yaw_consumption/test_ros_consumption.py) | 黑盒原生延迟消费验收（unittest；需 `rclpy`、隔离 domain 与 evidence 目录） | `NativeDelayedConsumptionAcceptance` |
| [`native/yaw_rotation/CMakeLists.txt`](native/yaw_rotation/CMakeLists.txt) | 独立构建 `rotation_acceptance`（依赖 `plan_env`） | `add_executable` |
| [`native/yaw_rotation/rotation_acceptance.cpp`](native/yaw_rotation/rotation_acceptance.cpp) | 对真实 `GridMap` 的独立膨胀/边界用例，不复用另一套占据模型 | `main()` |
| [`README.md`](README.md) | 本文件：命令、环境/skip 语义与逐文件职责 | 人读 |

## 验证与测试入口 / 配置注意

在仓库根目录执行（Python 3.12，`PYTHONPATH=src` 让 `embodied_navigation` 可 import）：

```sh
PYTHONPATH=src python3 -B -m compileall -q src scripts
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit        # 646 tests, OK
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression  # 60 tests, OK (skipped=3)
```

- `-B`：不写 `__pycache__`，避免污染工作树（本仓库要求运行产物不进 Git）。
- **不要加 `-t .`**：`tests/` 与各子目录都没有 `__init__.py`，`discover -t .` 会直接抛 `ImportError: Start directory is not importable`；用上面的 `-s` 写法即可。
- 单个文件/用例（模块不在包内，需把 `tests/unit` 加入 `PYTHONPATH`）：

  ```sh
  PYTHONPATH=src:tests/unit python3 -B -m unittest test_operator_map -v
  PYTHONPATH=src:tests/unit python3 -B -m unittest test_layered_mapping.QueryCliTests -v
  ```

- 基线（2026-11-20 本机实测，`refactor/06-clean-base` / `5b880b2`）：unit `Ran 646 tests` → `OK`（0 跳过）；regression `Ran 60 tests` → `OK (skipped=3)`，3 条均为 `test_dynanav_session_collision.FloorSessionRepair` 的 `CPU USD bindings unavailable`。
- 环境相关：skip 既不是通过也不是失败；`rclpy`、USD 绑定、`g++`、外部构建 prefix 缺失时对应用例会如实 skip 或无法构建。换机器后 `Ran`/`skipped` 数字变化属于环境差异，不要直接判定回归。
- 原生目录：`tests/native/dynanav_lio` 需已安装的 `fast_livo2_core` 与 ROS 2 Jazzy 依赖，构建命令见其 README；构建产物与 CTest 日志写到 `$PROJECT_DATA_ROOT/embodied-navigation/build_cache/` 这类外部目录，不入 Git。
- 测试之间不共享可变状态：三个子目录是互不 import 的独立发现根，跨文件共用逻辑只通过 `*_fixtures.py` 或同目录 import。

## 已知缺口与维护注意

- **`unit/test_livo_yaw_sampler_policy.py` 当前不生效**：它用 pytest 风格的模块级 `test_*` 函数，`unittest discover` 不会收集（因此 `Ran 646` 不含它）；且模块常量指向一个本机会话产物路径，按当前检出执行会失败。按 `AGENTS.md` 规则不得用加 skip 掩盖：要么改成 `unittest.TestCase` 并把输入改成仓库内/临时构造，要么删除并说明。**本次文档盘点不修改测试代码**，留待单独修复。
- 部分文件直接读源码文本/AST（例如 `test_dynanav_source_snapshot.py`、`test_dynanav_odometry_source.py`、`test_dynanav_freshness_receipt.py`）：改入口结构或文件路径会先让这类测试失败，不一定代表行为变了；重构时先 `git grep` 该文件是否被测试按路径读取。
- 新增测试默认放 `tests/unit/`，保持无 ROS、无网络、无 GPU；需要真实数据、原生构建或仿真的检查写成 `tests/native/` 下的显式脚本并注明"不在两条 unittest 命令内"。vendored 包的测试（如 `third_party/scan/plan_manage/test/`、`third_party/scan/plan_env/test/`）属于上游资产，既不在这里运行，也不随两条命令执行。
- 测试用 `tempfile` 构造输入、不写仓库内文件；少数用例会读真实配置：`test_dynanav_rtx_source.py` 读 `configs/simulation/dynanav/fastlivo_rtx_os1_32.yaml` 的 `ros__parameters`，`pct_budget_acceptance_fixtures.py` 把 `configs/simulation/dynanav/` 复制进临时构建根。唯一输出是 stdout/stderr，**有 traceback 不等于失败**（部分用例故意触发并捕获异常），以最终 `OK/FAILED` 与退出码为准。
