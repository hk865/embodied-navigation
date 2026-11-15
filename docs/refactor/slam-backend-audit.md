# SLAM 后端现状与先验地图重定位

> 依据：对 HEAD b314337 的只读盘点、正确性审查和对抗复核。没有修改仓库。只有 B/C 后端的测试在 WSL 里跑过，其余结论都来自读代码和 grep。

## 0. 结论

**D 路径（DynaNav，FAST-LIVO2）没有 SLAM 后端。** 它只有前端：ESIKF 加体素平面地图。另有一个关键帧导出器，只在 online_person 路径打开，而且写出的关键帧没有任何程序读取。关键帧匹配、回环、位姿图、地图加载、先验地图重定位，D 路径都没有。

所以你说的前提“现有 SLAM 已有后端”在 D 上不成立。按你定的规则，这种情况要如实汇报，不能以“已有后端”为由删掉先验地图重定位。仓库里确实有一套具备回环、图优化和先验重定位的 Python 后端，但它只接在 B/C（Building、RoboGauge）路径上，没有接入 D。

| # | 能力 | D 路径状态 | 位置（file:line） | 说明 |
|---|---|---|---|---|
| 1 | 地图管理 | **部分** | `third_party/fast_livo2/fast_livo2_core/src/voxel_map.cpp:850-1000`（Build/UpdateVoxelMap）、`:1383-1430`（mapSliding）；`LIVMapper.cpp:796-799,870,875-877` | 体素八叉树地图在用。滑窗裁剪关闭（`configs/simulation/dynanav/fastlivo_lio.yaml:67-70`、`fastlivo_livo.yaml:82-85` 均为 `map_sliding_en: false`），所以地图无界增长。没有子图，也不支持多会话。 |
| 2 | 关键帧选取与存储 | **部分**（只在 online_person 启用，只写不读） | `fast_livo2_core/src/backend/snapshot_bridge.cpp:40-57,156-210`；`LIVMapper.cpp:2962-3037`；`scripts/run_dynanav_online_person.py:127-147` | 判据：1-2 s 一帧，平移 0.8 m 或旋转 0.2 rad 时提前选。每帧存位姿和 LiDAR 点云。`run_dynanav_native_exploration.sh:94` 不导出关键帧。 |
| 3 | 关键帧匹配 / 位置识别 | **不存在** | 在 `third_party/fast_livo2` 中 grep `loop|isam|gtsam|g2o|DBoW|scan.?context|relocal`，只命中 snapshot_bridge 和 pcd_save | 上游 FAST-LIVO2 本身也没有。 |
| 4 | 回环检测 | **不存在** | 同上 | patches 0001-0010 都没有加回环。 |
| 5 | 位姿图优化与回写 | **不存在** | 同上 | 在 fast_livo2 中 grep `C_world_odom`、`backend_result`，零命中。 |
| 6 | 地图保存与加载 | **存在但未启用**（只能保存，不能加载） | `LIVMapper.cpp:957-1020,1120,2215-2280`（savePCD）；`visual_snapshot.cpp:83-111`（只用于诊断） | D 的 yaml 里 `pcd_save_en: false`。体素地图本身不能保存或加载。 |
| 7 | 先验地图重定位（map→odom） | **不存在** | 在 `simulation/dynanav` 和 `scripts/run_dynanav_*` 下 grep `relocal|C_world_odom|map_to_odom|known_scene|Relocalizer|PriorTracker`，零命中 | 导航直接使用 odom 系位姿。 |

## 1. 另外两套实现的能力，以及能否给 D 复用

### 1.1 A 路径：known_scene（`launch/main.py → slam.launch.py`）

它的定位是“已知场景几何辅助的局部位姿修正”，算不上先验点云地图重定位。所谓先验图，是每次 prepare 阶段从仿真 SDF 解析出的静态盒子（`launch/prepare.py:214-216`、`known_scene_reference_builder.py:25-80`）。

| 能力 | 状态 | 位置 | 说明 |
|---|---|---|---|
| 地图管理 | 部分 | `mapping/map_archive.py:42-276`；`semantic_mapping/core.py:145-670` | 0.08 m 体素存档，只存不读，坐标系是未校正的 camera_init。LayeredMap 只有单个 epoch。 |
| 关键帧 / 位置识别 / 回环 / 位姿图 | 不存在 | — | 每 1 s 取一帧，用完即丢。 |
| 保存 / 加载 | 部分 | `map_archive.py:229-276` | 能保存 colored_map.pcd，但没有任何代码读回它。 |
| 先验重定位 | 部分 | `localization_runtime.py:154-425`；`known_scene_matcher.py:480-713` | 初值必须是已知出生点，只在 Z 方向枚举种子，然后做 point-to-plane ICP。需要两帧确认，跳变门限 0.65 m / 0.2 rad，TTL 3 s。不做 XY 和 yaw 方向的全局搜索。 |

**能否给 D 复用：不能直接复用。** 它依赖 SDF 里按 `floor_/ramp_/wall_/panel_/landmark_` 命名的盒子，而 D 用的是 Isaac `office.usd`。它还依赖 A 路径的 run 目录契约、话题和导航 mixin。能搬过去的只有纯 numpy 部分（`_fit`/`score` 的框架、`CorrectionState`、`BackgroundMatcher`），而且搬之前还得换成点云参考、加上全局搜索，并修好下文表格里的 A-1/A-3。

### 1.2 B/C：Python 后端（`src/embodied_navigation/localization_mapping/backend/`）

| 能力 | 状态 | 位置 | 说明 |
|---|---|---|---|
| 地图管理 | 部分 | `external_map.py:135`；`history.py:62`；`worker.py:138-141` | 外部 PCD 分块加 KD 树。History 每 8 帧拼一个子图，上限 2000 帧。 |
| 关键帧消费 | 存在 | `run.py:20-66`；`history.py:17,208` | 读取 FAST-LIVO2 写出的 `livo_keyframe/v1` spool，格式与 D 的导出一致。 |
| 关键帧匹配 | 部分 | `worker.py:188-214`；`registration.py:211-388` | 回环候选只按位姿距离 <3 m 选，没有 ScanContext 或 BoW。Relocalizer 先用 PCA 和形状特征给块排序，再做 RANSAC 加 ICP。 |
| 回环检测 | 存在 | `worker.py:188-312`；`history.py:115-192`；`tracking.py:28-156` | 用 PriorTracker 做几何验证，30 s 内两次结果一致才入图。 |
| 位姿图优化 | 部分 | `native/graph_solver.cpp:45-89`；`native_graph.py:12-73` | 自写的 SE(3) 高斯牛顿，窗口 80 个节点，最多 8 次迭代。需要 `install/backend/lib/liblivo_graph.so`，本检出里没有这个文件。**在生产配置下结果不会被使用**：`worker.py:282` 只在 `relocalizer is None` 时发布结果，而 B/C 总会传 `--external-map`。优化结果也不回写前端。 |
| 保存 / 加载 | 部分 | `history.py:208-273`；`external_map.py:185-233` | History 每帧保存一次，但 `History.load` 在生产代码里没有调用方。 |
| 先验地图重定位 | **存在并启用（只在 B/C）** | `tracking.py:28`；`correction.py:15-170`；`building_runtime.py:116-146,216-220`；启动方 `launch/building.py:360-393`、`navigation/robogauge/benchmark.py:97-114` | 先在先验 PCD 上做 PriorTracker 跟踪，经 TemporalAdmission 两次确认后输出 C_world_odom，再发布 map→odom。这是 B/C 生产中唯一真正起作用的后端能力。 |

**能否给 D 复用：技术上可以接，但接之前必须先修错误。** 两边的 spool 格式一致，在 D 的启动脚本里加一个 `python -m ...backend.run --spool <native_keyframes>` 进程就能接上。前提是先修下表的 #3、#4、#5、#6、#7，并补编 .so。

## 2. 已确认的错误（verdict.real = true）

“影响 D”一列：D 当前没有接任何后端，所以多数条目是“否”。标“接入后继承”的，表示把后端搬到 D 后会出现。

| # | 位置 | 问题 | 失败场景 | 严重度 | 影响 D | 修复方法 |
|---|---|---|---|---|---|---|
| D-1 | `snapshot_bridge.cpp:157-168`；`run_dynanav_online_person.py:143-146` | D 的关键帧 spool 没有读取方，写满 256 个后永久停写 | `--duration` 大于约 256-512 s 时，后半程的关键帧被计入 spool_dropped，而且不报警。默认 45 s 不会触发 | low | **是**（只影响 online_person 导出） | 参数检查时，duration >256 s 就告警。运行结束读 `bridge_status.json`，spool_dropped >0 就报警。长期方案是接一个读取方，或给导出器加 archive 模式 |
| B-1 | `run.py:42`；`snapshot_bridge.cpp:169-172`；`history.py:76-79` | spool 按字典序消费，seq 没有补零，跨位数时乱序丢帧 | spool 积压时，`kf_E_10` 先于 `kf_E_9` 提交，9 号帧被判为乱序，而且目录已被 rmtree。启动积压满 10 个以上时可能连丢 8 帧 | low~medium | 接入后继承 | 按 `(int(epoch), int(seq))` 排序；C++ 写目录名时可选补零 |
| B-2 | `tracking.py:103-105`；`worker.py:213-229,259` | 回环只能修正不超过 0.5 m / 0.2 rad 的漂移；worker 的 1.0 m / 0.3 rad 一致性门永远不会触发 | 走一个大环，漂移 0.8 m，回环始终是 tracking_geometry_rejected，图优化一次都不触发。测试只注入了 0.4 m 漂移，所以没测出来 | medium | 接入后继承（决定回环在 D 上能否起作用） | 把盆地参数化，按路径长度放宽（上限约 2 m / 0.35 rad），并加一层更粗的 ICP；让 :259 的门限与盆地对齐；补一个 1.5 m 漂移的用例 |
| B-3 | `run.py:100-105`；`worker.py:156-186`；`tracking.py:103-105` | 先验地图跟踪没有全局重定位兜底，超出盆地后永久丢失 | 走廊里连续被拒、漂移累积到 0.6 m 以上后，map→odom 修正永久冻结；初始先验误差 >0.5 m 时一开始就定位不上 | medium | 否（D 没有先验图）；**这是先验重定位自身的缺陷** | 同时持有 PriorTracker 和 Relocalizer，连续 N 帧失败或行驶超过 T m 后切到全局重定位，仍需两次确认；状态里输出 tracking_lost |
| B-4 | `worker.py:138-141`；`run.py:66`；`graph_solver.cpp:38-39` | 达到 2000 个关键帧后，每帧都抛 backend_error，但 spool 照样被删；先验重定位也被一起停掉 | Building 连续运行约 17-67 分钟后，回环和先验修正同时失效（有错误计数，但没有结构化标志） | low | 接入后长时探索会遇到 | 把上限检查改为只跳过 history 和回环，重定位继续跑；加 `history_full` 标志；纯回环模式下停止消费 spool，不再 rmtree |
| B-5 | `correction.py:81-87`；`building_runtime.py:121-124`；`worker.py:266-268` | 比较 C_world_odom 时直接比平移部分，误差被到 odom 原点的距离（杠杆臂）放大 | 离起点 30 m 时，±0.003 rad 的 yaw 噪声就会让两次确认的差值达到 0.18 m，永远确认不了；0.02 rad 的真实修正会被判成 0.6 m 跳变而拒绝，而且不留任何记录 | medium | 接入后继承 | 改为在当前本体位姿处比较：`pose_distance(C_new@T_ob, C_prev@T_ob)`；运行时的跳变门同样改，并记录被拒的情况 |
| B-6 | `worker.py:185-186,323-334`；`building_runtime.py:116-124` | 后端先验与运行时实际采用的 C 会分叉，而且不会自己恢复 | 后端接受了一个新 C，运行时以跳变为由拒绝，后端仍围绕新 C 跟踪，之后所有提议都被拒，导航静默使用过期修正 | medium | 接入后继承 | 后端用与运行时相同的跳变门；结果写盘成功后才更新 world_odom_prior；或者让运行时把已采用的 C 写回给后端 |
| B-7 | `graph_solver.cpp:86`；`native_graph.py:57-63`；`worker.py:277` | 返回码 1 同时表示“图已一致”和“线搜索失败”，一律报成 native_solver_rejected；传入的 30 次迭代被悄悄截成 8 | 只是诊断标签错误，位姿和地图没有错 | low | 否 | 拆分返回码，加 `graph_already_consistent`；worker 直接传 8 |
| B-8 | `native_graph.py:8,22-23`；`tests/regression/test_backend_flow.py:223-224`、`test_backend_auto_loops.py:40` | 基线失败测试的根因：依赖未构建的 `liblivo_graph.so`，又没有 skip 守卫；README 里“.so 已存在”的说法过时 | 在干净检出上，2 个图优化用例必然失败。在 WSL 编好 .so 后 5/5 通过，说明算法没有问题 | low | 否 | 加 `skipUnless(LIBRARY.is_file())`，或通过 `docker/build_owned_native.py --execute` 构建；更正 README |
| A-1 | `known_scene_matcher.py:596-602,619-626`；`localization_runtime.py:36-42` | 与 B-5 同类：known_scene 的三道 C 平移门限也被杠杆臂放大 | 离出生点 20 m、yaw 漂移 2° 时，每次都报 correction_jump_rejected，3 s 后 stale，导航永久停住 | medium | 否（只在移植时继承） | 三处门限都改为在本体处比较 |
| A-2 | `known_scene_matcher.py:683-693` | 修正混合时，旋转和平移各自限速，二者不同步完成 | 当 \|p\|=12 m、θ=0.04 rad 时，停车期间发布的位姿会先“甩出”约 0.4 m 再回来，事务时间拉长到约 12 s | low | 否 | 平移和旋转用同一个截断比例 s；也可以改为以本体为中心插值 |
| A-3 | `localization_runtime.py:301-311,367-388`；`known_scene_matcher.py:678-683` | 修正事务开启后，没有中止、重新定目标或超时的路径，一旦 stale 就永久死锁 | 确认得到的目标与静止后的测量相差 0.08-0.15 m，或者停车时有人挡住视野，transition 永远为 True，control_allowed 永远为 False | medium | 否 | 事务中收到新的确认结果时重新定目标；加约 15 s 超时并上报失败原因 |
| A-4 | `localization_runtime.py:367-388` | 新鲜度与是否实际应用修正脱钩：idle 时被接受的测量只刷新新鲜度，不应用 | 每个任务刚开始时“起步即停”一次 | low | 否 | 允许在 idle（静止）状态下开启事务 |
| A-5 | `localization_runtime.py:389-395`；`known_scene_matcher.py:581-595`；`integration_limits.py:7` | TTL 3 s 与“1 s 提交 + 两帧确认 + 计算时间”之间裕量不足 | 只要一帧质量门禁失败，就会因 stale 短暂停车一次 | medium | 否 | 把 TTL 提到约 5 s；或者在跟踪态遇到 no_quality 时不清零 pending |

**被反驳的条目共 8 条**，不列入上表：
- D 范围 3 条：前端 epoch 变化导致 spool 卡死、结果被判 stale、mapSliding 的 position_last_ 未初始化。
- A 范围 3 条：box 无空间索引导致超预算、geometry_ratio 量纲问题、存档坐标系问题。
- B/C 范围 2 条：epoch 卡死（与 D 范围那条是两次独立审查、都被驳回）、spawn yaw 丢失。

驳回原因都是在现有启动器和配置下无法触发，或者属于有意的设计。其中几条可以作为可选的加固项。

## 3. 建议（只列选项，由你决定）

### 3.1 B/C 的 Python 后端（含先验地图重定位）能不能删

| 角度 | 事实 |
|---|---|
| 对 D 运行时的影响 | **零损失。** D 不 import 也不启动它。删掉以后，D 写出的 native_keyframes 只是留在盘上的证据。 |
| 对 B/C 的影响 | Building 和 RoboGauge 生产中唯一生效的 map→odom 修正（PriorTracker）会失去。如果这两条路径本身也不再需要，就没有影响。 |
| 按你的判定规则 | 前提“D 已有后端”**不成立**，所以不能以“已有后端”为理由删。如果要删，理由只能是“D 不需要后端”或“打算另行补齐”。 |
| 删除时要同步清理 | `building_runtime.py:19-20` 的 import；`launch/building.py:360-393,601`；`robogauge/benchmark.py:97-114,168`；3 个 `tests/regression/test_backend_*.py`；`docker/build_owned_native.py` 里的 backend 映射。 |
| 删除带来的损失 | 失去以后可以复用的参考代码：SE(3) 求解器、校正契约、已经和 C++ spool 对接好的消费框架。 |

### 3.2 如果要给 D 补齐后端，可选路径

| 选项 | 内容 | 工作量 | 风险 |
|---|---|---|---|
| O1 维持现状 | D 保持纯前端，直接删除 Python 后端；只修 D-1（加告警） | 小（约 0.5 天） | 长时探索会漂移，没有回环，内存无界增长 |
| O2 把 Python 后端接到 D | 在 D 脚本里启动 backend.run 读 native_keyframes。先修 B-1、B-2、B-3、B-5、B-6，B-4 视情况修；补编 .so；重新设计 `worker.py:282`，让图优化结果真正以 map→odom 输出 | 中（约 1-2 周） | 纯 Python 单线程配准，性能存疑；只在小规模合成场景上验证过；回环候选靠位姿距离，漂移大时找不到 |
| O3 复用 A 的 known_scene 做先验重定位 | 把参考图换成点云加 KD 树，加上 XY 和 yaw 的全局搜索，修 A-1、A-3、A-5，并解除对 A 路径契约的依赖 | 中到大（约 2-3 周） | 实际上等于重写；在 D 的 office.usd 场景上没有现成的先验图 |
| O4 引入成熟后端 | 例如 FAST-LIO-SAM / LIO-SAM 式的 ScanContext + GTSAM（iSAM2）做回环；先验重定位用 ScanContext 做全局候选，再用 GICP/NDT 精配准，输出 map→odom | 大（约 2-4 周，含 ROS2 移植和依赖） | 算法成熟，风险集中在集成：坐标系约定、时序、与导航的修正契约 |
| O5 从 B/C 提取部件 | 保留 `native/graph_solver.cpp`（约 90 行，exp/log 和 IRLS 已核对，WSL 实测把末帧误差从 0.539 m 降到 0.017 m）、`TemporalAdmission/BackendResultReader` 的修正契约、spool 解码；其余删除。可以和 O4 组合，用 ScanContext 生成候选，再接这个求解器 | 小到中（3-5 天，不含新的位置识别模块） | 求解器只优化 80 个节点的窗口、最多 8 次迭代，大回环只能修正一部分；契约里的杠杆臂问题（B-5）必须一起修 |

**附带说明**：如果 D 需要长时间运行，不管选哪个选项，都应考虑开启 `map_sliding_en` 并评估内存。注意 `clearMemOutOfMap` 是硬删除，被删区域回访时只能重新建图；VIO 的 feat_map 也不会随滑窗一起裁剪。
