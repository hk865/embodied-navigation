# localization_mapping：定位建图

`localization_mapping` 是七层架构中的定位建图层：把 FAST-LIVO2 前端的关键帧变成回环闭合/先验地图重定位后的 map→odom 修正，并提供纯 CPU 的语义点层与只读查询。它不包含 FAST-LIVO2 算法本体（在 `third_party/fast_livo2`），也不做规划、控制或任务状态机。

盘点日期 2026-11-20（分支 `refactor/06-clean-base`，HEAD 5b880b2）。

## 模块定位与架构层级

- 架构层级：第 3 层「定位建图」（见仓库根 `AGENTS.md` 第 2 节）。
- 输入：FAST-LIVO2 的 `snapshot_bridge` 写出的关键帧 spool（`$RUN/observer/native_keyframes/kf_<epoch>_<seq>/`）；先验地图目录（`PRIOR_MAP`）；语义层的观测点、mask 与位姿（由调用方提供）。
- 输出：TF `map → camera_init`、`/dynanav/backend/corrected_odometry`（`nav_msgs/Odometry`）、`/dynanav/backend/status`（`std_msgs/String` JSON）、`--save-map` 目录下的 `map.json` + 每关键帧 `.npz`；语义层输出 `layered_observed_map/v1` 快照与确定性查询结果。
- 上游/下游：由 `scripts/run_dynanav_native_exploration.sh` 在 FAST-LIVO2 就绪后启动；被 `scripts/reconstruct_dynanav_slam_map.py`、`scripts/query_scene_map.py`、`simulation/dynanav/capture/ros_interface.py`（只取 `STATIC` 常量）与测试使用。
- 边界：不改写前端里程计，只发布修正副本；导航当前仍订阅原始 odometry（启动脚本注释明确「Navigation below still consumes the raw odometry」）；语义层不做检测/分割推理、不读 ROS/Isaac、不授予控制权。

## 当前文件夹结构

```text
src/embodied_navigation/localization_mapping/
├── backend/                      # 点云 SLAM 后端（9 个 .py），见 backend/README.md
├── semantic_mapping/             # 纯 CPU 语义点层与查询（4 个 .py），见 semantic_mapping/README.md
├── README.md                     # 本文件
└── __init__.py                   # 1 行：模块 docstring
```

17 个跟踪文件（14 个 `.py` + 3 个 README），共 2,211 行。**只有 `backend/node.py` import ROS**（`git grep rclpy` 只命中该文件），其余模块可被纯 CPU 测试直接导入。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：模块定位、数据流、文件清单、测试与配置注意。 | — |
| [__init__.py](__init__.py) | 模块 docstring（1 行），无重导出、无副作用。 | 无 |
| [backend/README.md](backend/README.md) | 后端子模块文档：坐标约定、spool 数据流、配置加载、逐文件职责。 | — |
| [backend/__init__.py](backend/__init__.py) | 后端包 docstring（6 行），说明只 `node` 依赖 ROS、后端不写前端 odometry。 | 无 |
| [backend/backend.py](backend/backend.py) | 后端核心：建图（关键帧、回环、位姿图优化）与先验地图重定位，产出 `C = T_world_odom`。 | `BackendConfig`（含 `from_dict`/`from_json` 严格加载与 `anisotropic_information`）、`information`、`SlamBackend.ingest/save_map/T_world_odom` |
| [backend/correction.py](backend/correction.py) | map→odom 修正的时域准入：在本体位姿处比较候选、两次确认与跳变门。 | `CorrectionAdmission.propose/current` |
| [backend/keyframes.py](backend/keyframes.py) | 关键帧 spool 读取（`metadata.json` schema `livo_keyframe/v1` + ASCII PCD）与体素降采样。 | `Keyframe`、`ready_directories`、`read_keyframe`、`read_spool`、`voxel_downsample`、`transform_points` |
| [backend/map_store.py](backend/map_store.py) | 关键帧地图：子图/全局图与 save/load（schema `embodied_navigation_map/v1`）。 | `MapStore.add/update_poses/points_world/submap/global_map/save/load` |
| [backend/node.py](backend/node.py) | ROS 2 薄封装：轮询 spool、发布 TF/修正里程计/状态、退出时存图。 | `python -m embodied_navigation.localization_mapping.backend.node --spool … --namespace … --odometry-topic … [--prior-map …] [--save-map …] [--config …]`；`parse_args`、`main` |
| [backend/pose_graph.py](backend/pose_graph.py) | SE(3) 位姿图：稀疏法方程 LM、首节点固定、回环边 Cauchy 核。 | `PoseGraph.add_node/add_edge/pose/optimize`、`se3_exp/se3_log/adjoint/invert/pose_difference` |
| [backend/registration.py](backend/registration.py) | point-to-plane ICP（目标法向量用 k 近邻 PCA），由粗到细两级配准的基本单元。 | `icp_point_to_plane`、`IcpResult`（fitness/rmse/converged） |
| [backend/scan_context.py](backend/scan_context.py) | ScanContext 位置识别：描述子、列位移距离、yaw 换算、ring-key KD 树索引。 | `ScanContextConfig`、`descriptor`、`distance`、`shift_to_yaw`、`ScanContextIndex.query` |
| [semantic_mapping/README.md](semantic_mapping/README.md) | 语义点层子模块文档：来源绑定、掩码规则、查询入口、有界存储。 | — |
| [semantic_mapping/__init__.py](semantic_mapping/__init__.py) | 唯一门面：重导出 `Observation`/`PointMask`/`LayeredMap`/`SemanticScheduler`/`read_snapshot` 与三个状态常量。 | `__all__` 共 9 项 |
| [semantic_mapping/core.py](semantic_mapping/core.py) | 点层、对象关联、修订/重投影、快照与原子保存。 | `Observation`、`PointMask`、`LayeredMap.integrate/revise_mask/update_anchor/static_points/query/summary/snapshot/save`、`read_snapshot` |
| [semantic_mapping/query.py](semantic_mapping/query.py) | 只读、确定性的快照过滤与结构化上下文。 | `select`、`context_text`（schema `observed_scene_map_query/v1`） |
| [semantic_mapping/scheduler.py](semantic_mapping/scheduler.py) | 传感器时钟上的语义准入（周期/事件/epoch），不做推理。 | `SemanticScheduler.request/complete` |

## 输入输出与启动

```bash
python -m embodied_navigation.localization_mapping.backend.node \
    --spool "$RUN/observer/native_keyframes" --namespace /dynanav/backend \
    --odometry-topic /dynanav/rtx_livo/post_vio_odometry --save-map "$RUN/map" \
    [--prior-map "$PRIOR_MAP"] [--config <backend.json>]
```

- 后端每 `--poll-s`（默认 0.2 s）按数值序读取就绪 `kf_*` 目录，`SlamBackend.ingest` 之后删除该目录（确认路径在 `--spool` 之下），因此导出器的 `max_ready_directories` 不会被写满；解码失败的目录改名 `rejected_kf_*` 并记入状态。
- 输出话题与 TF、spool 文件格式、建图/重定位两种模式的判定流程见 [backend/README.md](backend/README.md)。
- 语义层的离线入口是 `scripts/reconstruct_dynanav_slam_map.py`（用录制的 LIO 估计重建点层）与 `scripts/query_scene_map.py`（读快照并过滤）；当前 HEAD 的 DynaNav 在线 `publish_cloud` 调用点传的是 `None` mapper，在线 adapter 未接线（`git grep oracle_mapper` 只命中参数与定义）。

## 验证与测试入口 / 配置注意

- 后端单测（合成世界，纯 numpy/scipy）：`tests/unit/` 下的 [test_slam_backend_backend.py](../../../tests/unit/test_slam_backend_backend.py)、[test_slam_backend_config_and_imu_patch.py](../../../tests/unit/test_slam_backend_config_and_imu_patch.py)、[test_slam_backend_correction.py](../../../tests/unit/test_slam_backend_correction.py)、[test_slam_backend_map_store.py](../../../tests/unit/test_slam_backend_map_store.py)、[test_slam_backend_pose_graph.py](../../../tests/unit/test_slam_backend_pose_graph.py)、[test_slam_backend_registration.py](../../../tests/unit/test_slam_backend_registration.py)、[test_slam_backend_scan_context.py](../../../tests/unit/test_slam_backend_scan_context.py)、[test_slam_backend_spool.py](../../../tests/unit/test_slam_backend_spool.py)，共用 [tests/unit/slam_backend_fixtures.py](../../../tests/unit/slam_backend_fixtures.py)（30 m × 20 m 合成房间与 360° 雷达）。
- 原生 spool 对接回归（需要 g++）：[tests/regression/test_snapshot_bridge.py](../../../tests/regression/test_snapshot_bridge.py) 编译并运行真实的 C++ `snapshot_bridge`，再用 `keyframes.read_keyframe` 解码。
- 语义层：[tests/unit/test_layered_mapping.py](../../../tests/unit/test_layered_mapping.py) 20 个用例；[tests/unit/test_dynanav_source_snapshot.py](../../../tests/unit/test_dynanav_source_snapshot.py) 用 AST 断言来源快照包含 `semantic_mapping/query.py` 与 `scripts/query_scene_map.py`。
- 全量 CPU 测试：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 配置：后端阈值集中在 `BackendConfig`，可由 `node.py --config <json>` 经 `BackendConfig.from_json` 严格加载（未知键/非法值直接报错）；当前启动脚本没有传 `--config`，因此使用 `BackendConfig()` 默认值。先验地图用环境变量 `PRIOR_MAP` 传给启动脚本，再由脚本转成 `--prior-map`；`--save-map` 指向 `$RUN/map`。`--truth-pose` 模式不启动本模块。
- 本机只做 `compileall` 与纯 CPU 测试：ROS 节点、真实 FAST-LIVO2 吞吐、回环正确性与先验重定位都需要 Linux + Isaac/ROS 环境，未在本次文档盘点中运行。
