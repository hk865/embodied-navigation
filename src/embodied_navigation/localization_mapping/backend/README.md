# localization_mapping/backend：点云 SLAM 后端

D 路径（DynaNav/Isaac，FAST-LIVO2 前端）的点云 SLAM 后端：消费 FAST-LIVO2 导出的关键帧 spool，做关键帧管理、ScanContext 位置识别、ICP 回环与 SE(3) 位姿图优化、地图保存/加载、先验地图重定位，输出 map→odom 修正 `C = T_world_odom`。算法取成熟方案的简单实现，追求可读与可离线测试，不追求论文级性能。

盘点日期 2026-11-20（分支 `refactor/06-clean-base`，HEAD 5b880b2）。

## 模块定位与架构层级

- 架构层级：第 3 层「定位建图」的后端子包，隶属 [localization_mapping](../README.md)；算法本体是自有 Python（FAST-LIVO2 在 `third_party/`，本包只消费它的导出物）。
- 输入：`--spool` 目录下 FAST-LIVO2 `snapshot_bridge` 写出的 `kf_<epoch>_<seq>/`（`metadata.json` schema `livo_keyframe/v1` + ASCII `points_lidar.pcd`）；可选的先验地图目录 `--prior-map`；`--odometry-topic` 上的 `nav_msgs/Odometry`（`/dynanav/rtx_livo/post_vio_odometry`）。
- 输出：TF `world_frame → odom_frame`（默认 `map → camera_init`）、`<ns>/corrected_odometry`、`<ns>/status`（JSON）、`--save-map` 目录。启动脚本使用 `--namespace /dynanav/backend`、`--save-map $RUN/map`。
- 上游/下游：由 `scripts/run_dynanav_native_exploration.sh` 在 FAST-LIVO2 就绪后启动；下游导航当前仍订阅原始 odometry（脚本注释：「Navigation below still consumes the raw odometry」），是否改订阅 `/dynanav/backend/corrected_odometry` 待服务器验证。
- 边界：只依赖 numpy、scipy 与标准库，**只有 `node.py` import ROS**，导入其余模块无副作用；不改写前端里程计，只发布修正副本；不做规划/控制，也不做前端 ESIKF 或体素地图。

## 坐标约定

- 位姿一律是 4×4 `float64` 齐次矩阵，`T_a_b` 把 b 系的点变换到 a 系。
- `odom`：前端里程计固定系（`camera_init`）；`body`：前端估计的本体系；`lidar`：关键帧点云所在的传感器系；`world`：地图系。建图模式下 `world` 等于首帧 `odom`，重定位模式下 `world` 是先验地图的系。
- `T_target_source`（ICP 结果）把 source 点云变换到 target 系；后端输出 `C = T_world_odom`，满足 `T_world_body = C @ T_odom_body`。

## 当前文件夹结构

```text
src/embodied_navigation/localization_mapping/backend/
├── README.md        # 本文件
├── __init__.py      # 6 行：包 docstring（只 node 依赖 ROS）
├── backend.py       # 513 行：BackendConfig（严格加载 + 各向异性信息矩阵）与 SlamBackend
├── correction.py    # 65 行：CorrectionAdmission（两次确认、在本体位姿处比较、跳变门）
├── keyframes.py     # 111 行：Keyframe、spool 读取、体素降采样
├── map_store.py     # 105 行：关键帧地图 save/load、子图/全局图
├── node.py          # 165 行：ROS 2 薄封装（唯一 import ROS 的文件）
├── pose_graph.py    # 166 行：SE(3) 位姿图，稀疏 LM，Cauchy 核
├── registration.py  # 94 行：point-to-plane ICP
└── scan_context.py  # 110 行：ScanContext 描述子与检索
```

10 个跟踪文件（9 个 `.py` / 1,335 行 + 本 README）。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：坐标约定、文件清单、配置加载、数据流、测试注意。 | — |
| [__init__.py](__init__.py) | 包 docstring（6 行）：说明关键帧/ScanContext/ICP/位姿图/地图/修正的归属，以及「只 `node` import ROS、不动前端里程计」。 | 无重导出 |
| [backend.py](backend.py) | 后端核心：建图模式（逐帧加节点 + 回环检测 + 图优化）与先验地图重定位模式（全局搜索 → 两次确认 → 局部跟踪 → 失落回退）。 | `BackendConfig`（`from_dict`/`from_json`/各阈值）、`anisotropic_information`、`information`、`yaw_pose`、`SlamBackend.ingest/save_map/T_world_odom`、`_search/_register/_propose` |
| [correction.py](correction.py) | map→odom 修正的时域准入：`window_s` 内 `confirmations` 个一致候选才采纳（采纳最后一个）；首次采纳不做跳变检查，之后超跳变门限的候选记入 `rejections` 且不清掉已采纳值。 | `CorrectionAdmission.propose(T_world_odom, T_odom_body, stamp_ns)`、`current`；比较发生在 `C @ T_odom_body`（本体位姿处），不是 `C` 平移之差 |
| [keyframes.py](keyframes.py) | FAST-LIVO2 关键帧 spool 编解码与体素降采样。 | `SCHEMA = "livo_keyframe/v1"`、`Keyframe`（`key=(epoch, sequence)`、`stamp_ns`、`T_odom_body`、`T_body_lidar`、`points_lidar`）、`ready_directories`（按数值 `(epoch, seq)` 排序、跳过缺文件目录）、`read_keyframe`、`read_spool`、`transform_points`、`voxel_downsample` |
| [map_store.py](map_store.py) | 关键帧地图：点云存 lidar 系、`add` 时降采样一次，`points_world` 用 `T_world_body @ T_body_lidar` 按需变换，因此图优化不重写点数据。 | `MapStore.add/update_poses/points_world/submap/global_map/save/load`；磁盘格式 `map.json`（schema `embodied_navigation_map/v1`，插入顺序）+ 每帧 `kf_<epoch>_<seq>.npz`（`points_lidar`、`descriptor`） |
| [node.py](node.py) | ROS 2 薄封装：定时轮询 spool、`SlamBackend.ingest` 后删目录、发布 TF/修正里程计/状态、SIGINT/SIGTERM 时存图退出 0。 | `parse_args`（`--spool`/`--odometry-topic`/`--namespace` 必填；`--prior-map`/`--save-map`/`--config`/`--world-frame`/`--odom-frame`/`--poll-s`）、`BackendNode`、`main` |
| [pose_graph.py](pose_graph.py) | SE(3) 位姿图：节点 `T_world_body`、边 `(a, b, T_a_b)`，切线向量顺序 `(translation rho, rotation phi)`，更新右乘 `T <- T @ exp(delta)`，首节点固定；鲁棒边用 Mahalanobis 范数的 Cauchy 核。 | `PoseGraph.add_node/add_edge/pose/optimize`、`se3_exp`/`se3_log`/`adjoint`/`invert`/`pose_difference` |
| [registration.py](registration.py) | point-to-plane ICP：目标法向量由 k 近邻 PCA 得到，每个 Gauss-Newton 步在 target 系左乘 `(rho, phi)`；点数不足/对应不足/法方程奇异返回 `converged=False` 而不抛异常。 | `icp_point_to_plane(source, target, T_init, *, max_correspondence_m, iterations, voxel_m)`、`IcpResult`（`T_target_source`、`fitness`、`rmse_m`、`converged`）、常量 `MIN_POINTS=50`、`NORMAL_NEIGHBORS=10` |
| [scan_context.py](scan_context.py) | ScanContext 位置识别（Kim & Kim, IROS 2018）：body 系点云按 `rings × sectors` 极坐标格取最大高度；列位移余弦距离给出候选与 yaw 初值；ring-key KD 树做预筛。 | `ScanContextConfig(rings=20, sectors=60, max_range_m=20.0)`、`descriptor`、`ring_key`、`distance`（返回 `(dist, shift)`）、`shift_to_yaw`、`ScanContextIndex.add/query`（返回 `(key, dist, yaw)`） |

## 配置加载：BackendConfig（严格）与各向异性信息矩阵

```python
from embodied_navigation.localization_mapping.backend.backend import BackendConfig

config = BackendConfig()                      # 默认值，node.py 不传 --config 时使用
config = BackendConfig.from_dict({...})       # JSON 风格映射，未知键直接 ValueError
config = BackendConfig.from_json("backend.json")
```

- `from_dict`：参数必须是映射（否则 `ValueError`）；顶层键必须在 `dataclasses.fields(BackendConfig)` 之内，未知键按名字排序后整体报错；`scan_context` 子映射再走一次同样的严格校验（未知键、非法 `rings`/`sectors`/`max_range_m` 都拒绝）；构造后调用 `_validate()`，拒绝非正/非有限 sigma 与步长、`min_fitness` 不在 `(0, 1]`、计数类字段不是正整数、`loop_submap_neighbors`/`exclude_recent`/`min_loop_gap_s` 为负等。
- `from_json(path)`：读文件后调用同一个 `from_dict`，不存在第二套校验。
- `node.py --config <json>`：`argparse` 默认 `None`，`main()` 里 `BackendConfig() if args.config is None else BackendConfig.from_json(args.config)`。当前 `scripts/run_dynanav_native_exploration.sh` 的 `BACKEND_ARGS` 不传 `--config`，生产入口目前使用默认配置（文档盘点时的 grep 结论）。
- `anisotropic_information(sigma_xy_m, sigma_z_m, sigma_rp_rad, sigma_yaw_rad)`：返回 `diag(w_xy, w_xy, w_z, w_rp, w_rp, w_yaw)`，`w = 1/sigma²`，顺序与位姿图的 `(rho_x, rho_y, rho_z, phi_x, phi_y, phi_z)` 一致；水平平移共用 `sigma_xy_m`、roll/pitch 共用 `sigma_rp_rad`，高度与 yaw 各有独立 sigma。`information(sigma_m, sigma_rad)` 是它的各向同性特例。
- `odometry_information()` / `loop_information()`：每个 odom 边/回环边写进位姿图的 6×6；`odometry_sigma_z_m`、`odometry_sigma_yaw_rad`、`loop_sigma_z_m`、`loop_sigma_yaw_rad` 为 `None` 时退回对应各向同性标量，非 `None` 时只覆盖该轴（字段追加在 dataclass 末尾以保持位置兼容）。`__post_init__` 会先算一次两个矩阵，因此默认构造也会校验 sigma。
- 逐轴覆盖示例：`BackendConfig(odometry_sigma_z_m=0.05, loop_sigma_yaw_rad=0.004)` → odom 信息 `diag(0.2⁻², 0.2⁻², 0.05⁻², 0.02⁻², 0.02⁻², 0.02⁻²)`（未覆盖轴回退默认 `odometry_sigma_m=0.2`/`odometry_sigma_rad=0.02`）。
- FAST-LIVO2 的 IMU 偏置协方差参数由 [0011-imu-bias-covariance-params.patch](../../../../third_party/fast_livo2/patches/0011-imu-bias-covariance-params.patch) 提供（vendored 源码不原地改，改动走 patch）；单测只 `git apply --check`，不实际应用。

## 数据流

1. FAST-LIVO2 的 [snapshot_bridge.cpp](../../../../third_party/fast_livo2/fast_livo2_core/src/backend/snapshot_bridge.cpp) 先写 `tmp_<epoch>_<seq>/`，完成后 rename 为 `kf_<epoch>_<seq>/`；`node.py` 只认完整目录。
2. 建图模式（无先验图）：每帧成为位姿图节点，与前一帧用 odom 相对位姿连边；ScanContext 在最近 `exclude_recent` 帧与 `min_loop_gap_s` 之外找候选，带 yaw 初值对候选时间邻域子图做由粗（`coarse_correspondence_m`）到细（`fine_correspondence_m`）的 ICP；ICP 本体位姿还要与图当前估计在随 odom 路径长度增长的预算内一致（拒绝对称性误配），通过后加 Cauchy 回环边并优化；随后 `C = T_world_body(latest) @ inv(T_odom_body(latest))` 交给 `CorrectionAdmission`。
3. 重定位模式（有先验图）：ScanContext 全局搜索 + 先验子图 ICP，两次确认后采纳 C；之后每 `relocalize_every` 帧从当前 C 起做局部 ICP 跟踪，连续 `lost_after` 次失败（无匹配或被判跳变）回到全局搜索并重置准入，期间继续发布旧 C；新关键帧按当前 C 写入本次会话地图，退出时由 `save_map` 写盘。
4. 跳变门限按模式不同：建图用 `loop_max_jump_translation_m`/`loop_max_jump_rotation_rad`（回环本来会按累计漂移移动 C），重定位用 `max_jump_translation_m`/`max_jump_rotation_rad`。

## 进程入口与输出

```bash
python -m embodied_navigation.localization_mapping.backend.node \
    --spool <dir> --odometry-topic <topic> --namespace <ns> \
    [--prior-map <dir>] [--save-map <dir>] [--config <json>] \
    [--world-frame map] [--odom-frame camera_init] [--poll-s 0.2]
```

| 输出 | 内容 |
| --- | --- |
| TF `world_frame → odom_frame` | 采纳 C 之后每次轮询发布 |
| `<ns>/corrected_odometry` | 订阅 `--odometry-topic`，位姿左乘 C（未采纳前为单位阵）后转发，`frame_id` 改为 `world_frame`，twist 不变 |
| `<ns>/status` | `adopted`、`loops`、`keyframes`、`last_error` |
| `--save-map` 目录 | 收到 SIGINT/SIGTERM 时保存 `map.json` + 每帧 `kf_<epoch>_<seq>.npz`，退出码 0 |

## 验证与测试入口 / 配置注意

- 单测（合成世界，纯 numpy/scipy）：`tests/unit/` 下 8 个 `test_slam_backend_*.py` 与 [slam_backend_fixtures.py](../../../../tests/unit/slam_backend_fixtures.py)；本目录的配置面由 [test_slam_backend_config_and_imu_patch.py](../../../../tests/unit/test_slam_backend_config_and_imu_patch.py) 固定（`from_dict`/`from_json` 严格性、`anisotropic_information` 对角元、写进 `PoseGraph` 的边矩阵、`node --config` 解析、0011 patch 存在且可 apply-check）。
- 原生 spool 回归（需要 g++）：[test_snapshot_bridge.py](../../../../tests/regression/test_snapshot_bridge.py)。
- 全量 CPU 测试：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 阈值在合成世界上标定，真实点云上需要按服务器结果调整；位姿图只有 odom 边与回环边，单个大回环闭合后环中段仍会残留与漂移形态有关的误差。
- `node.py` 本机只做 `compileall`：ROS 运行、真实 FAST-LIVO2 吞吐与先验地图重定位都需要 Linux + Isaac/ROS 环境，未在本次文档盘点中运行。运行产物（spool、地图、日志）写 `$PROJECT_DATA_ROOT/embodied-navigation/runs/<批次>/<run-id>/`，不进仓库。
