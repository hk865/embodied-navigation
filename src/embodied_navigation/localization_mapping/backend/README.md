# localization_mapping/backend

D 路径（DynaNav/Isaac，FAST-LIVO2 前端）的点云 SLAM 后端：消费 FAST-LIVO2 导出的关键帧 spool，做关键帧管理、ScanContext 位置识别、ICP 回环与 SE(3) 位姿图优化、地图保存与加载、先验地图重定位，输出 map→odom 修正 `C = T_world_odom`。

算法取成熟方案的简单实现，追求可读、可离线测试，不追求论文级性能。只依赖 numpy、scipy 与标准库；只有 `node.py` import ROS。导入本包没有副作用。

## 坐标约定

- 位姿一律是 4×4 `float64` 齐次矩阵，`T_a_b` 把 b 系的点变换到 a 系。
- `odom`：前端里程计固定系（`camera_init`）；`body`：前端估计的本体系；`lidar`：关键帧点云所在的传感器系；`world`：地图系。
- 建图模式下 `world` 等于首帧的 `odom`；重定位模式下 `world` 是先验地图的系。
- 输出 `T_world_body = C @ T_odom_body`。

## 目录结构

```text
src/embodied_navigation/localization_mapping/backend/
├── README.md        # 本文件
├── __init__.py      # 包 docstring
├── keyframes.py     # Keyframe、spool 读取（只读 kf_*，按数值 (epoch, seq) 排序）、体素降采样
├── scan_context.py  # ScanContext 描述子、列位移距离、yaw 换算、ring-key KD 树索引
├── registration.py  # point-to-plane ICP（目标法向量用 k 近邻 PCA）
├── pose_graph.py    # SE(3) 位姿图：稀疏 LM，首节点固定，回环边可用 Cauchy 核
├── map_store.py     # 关键帧地图：子图/全局图、save/load
├── correction.py    # CorrectionAdmission：在本体位姿处比较的两次确认与跳变门
├── backend.py       # SlamBackend 核心（建图 / 重定位），BackendConfig 集中所有阈值
└── node.py          # ROS 2 薄封装：轮询 spool、发布 TF / 修正里程计 / 状态
```

## 数据流

1. FAST-LIVO2 的 `snapshot_bridge`（`third_party/fast_livo2/fast_livo2_core/src/backend/snapshot_bridge.cpp`）先写 `tmp_<epoch>_<seq>/`，完成后 rename 为 `kf_<epoch>_<seq>/`，内含 `metadata.json`（schema `livo_keyframe/v1`，矩阵按行嵌套列表）和 ASCII `points_lidar.pcd`。
2. `node.py` 每 `--poll-s` 秒按数值序读取就绪目录，`SlamBackend.ingest` 之后删除该目录（确认路径在 `--spool` 之下），所以导出器的 `max_ready_directories` 不会被写满。解码失败的目录改名为 `rejected_kf_*` 并记入状态。
3. 建图模式：每帧加入位姿图，与上一帧连 odom 相对位姿边；ScanContext 在最近 `exclude_recent` 帧和 `min_loop_gap_s` 秒之外找候选，带 yaw 初值对候选的时间邻域子图做由粗（`coarse_correspondence_m`）到细（`fine_correspondence_m`）的两级 ICP，fitness/RMSE 过门限就加 Cauchy 回环边并优化。回环边锚定在邻域里离匹配位置最近的关键帧上。
4. 重定位模式：在先验地图的 ScanContext 索引里全局搜索，对先验子图做 ICP，两次确认后采纳 C；之后每 `relocalize_every` 帧在当前 C 附近做局部 ICP 跟踪，连续 `lost_after` 次失败（无匹配或被判跳变）回到全局搜索，期间继续发布旧 C。新关键帧按当前 C 放入本次会话地图。
5. `CorrectionAdmission` 在当前本体位姿处比较候选（`C1 @ T_odom_body` 与 `C2 @ T_odom_body`），`window_s` 内 `confirmations` 个一致候选即采纳**最后一个**；首次采纳不做跳变检查；跳变候选记入 `rejections`，不清空已采纳的 C。建图模式的跳变门是 `loop_max_jump_*`（回环本来就会按累计漂移移动 C），重定位模式是 `max_jump_*`。

## 进程入口与输出

```bash
python -m embodied_navigation.localization_mapping.backend.node \
    --spool <dir> --odometry-topic <topic> --namespace <ns> \
    [--prior-map <dir>] [--save-map <dir>] [--world-frame map] [--odom-frame camera_init] [--poll-s 0.2]
```

| 输出 | 内容 |
| --- | --- |
| TF `world_frame → odom_frame` | 采纳 C 之后每次轮询发布 |
| `<ns>/corrected_odometry`（`nav_msgs/Odometry`） | 订阅 `--odometry-topic`，位姿左乘 C 后转发（未采纳前按单位矩阵），`frame_id` 改为 `world_frame`，twist 不变 |
| `<ns>/status`（`std_msgs/String` JSON） | `adopted`、`loops`、`keyframes`、`last_error` |
| `--save-map` 目录 | 收到 SIGINT/SIGTERM 时保存：`map.json`（schema `embodied_navigation_map/v1`）+ 每帧 `kf_<epoch>_<seq>.npz`（`points_lidar`、`descriptor`），退出码 0 |

`scripts/run_dynanav_native_exploration.sh` 默认开启：给 observer 的覆盖参数里加 `backend.snapshot_directory: $RUN/observer/native_keyframes`，observer 就绪后后台启动本节点（`--namespace /dynanav/backend`、`--odometry-topic /dynanav/rtx_livo/post_vio_odometry`、`--save-map $RUN/map`）；环境变量 `PRIOR_MAP` 非空时传 `--prior-map`。导航仍订阅原来的 odom 话题，是否改用 `/dynanav/backend/corrected_odometry` 待服务器验证后再定。

## 测试与限制

- `tests/unit/test_slam_backend_*.py`：合成世界（30 m × 20 m，墙、立柱、箱体，360° 雷达 20 m 量程）上的单元与流程用例，只依赖 numpy/scipy。
- `tests/regression/test_snapshot_bridge.py`：编译并运行真实的 C++ `snapshot_bridge` 流程，用 `keyframes.read_keyframe` 解码其输出（需要 g++）。
- `node.py` 本机只做语法检查（`compileall`），ROS 运行验证在服务器上进行。
- 阈值（`BackendConfig`）在合成世界上标定，真实点云上需要按服务器结果调整。位姿图只有 odom 边和回环边：单个大回环闭合后，环中段仍会残留与漂移形态有关的误差。
