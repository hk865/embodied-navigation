# 实测局部子图、表面和视觉回环验证第一版

## 范围

基于受控实验备份提交 `42fc6fa`，独立分支 `refactor/05-observed-loop-evidence`。
原工作树未改动，不推送。本版改变后端的可选候选验证，不改变导航订阅的原始里程计。
无配置时三个模块均关闭；配置通过启动器复制到 run/configuration 后读取，不新增环境变量开关。

## 实现

- `backend/submaps.py`：查询端及历史候选端都使用真实多帧激光。
  从锚点向过去连续选择，限制 epoch、时间间隔、相对平移、世界高度变化、相对旋转及帧数。
  点先按前端相对位姿变到锚点 body，再合并体素；保留每个体素的来源关键帧。
  历史子图统一用锚点已优化位姿变到 world，避免把不同优化修正混入一段窗口。
  这些预算是相对位姿风险的代理量；没有前端协方差，不能宣称窗口相对位姿已被统计认证。
- `backend/surfaces.py`：重力对齐坐标下局部 PCA 和连通平面分组；水平面按实际高度分开，
  竖直面独立，稀疏、混合、斜面留为未知。保留点、法向、高度、支撑数和可用来源。
  不区分桌面与可走平台，不补全踏面，不根据机身姿态猜地面，不固定 Z。
  逐关键帧记录实测表面摘要；回环 ICP 后按水平/竖直类别检查实测覆盖与残差。
  必须显式确认 `gravity_aligned:true`，否则拒绝开启此模块。
- `backend/visual.py`：整流图像上的实测激光投影 patch，双向 ratio test、3D RANSAC 刚体拟合。
  返回 body 到 body 的 SE3 仅作 ICP 初始化，并检查视觉和 ICP 的位姿一致性；
  缺图、超时、无纹理、错误几何或退化时明确拒绝。外观相似度不生成图边。
  当前仍由 Scan Context 提名候选，视觉尚无独立地点召回索引。
- `keyframes.py` 接收可选 `visual.npz`；ROS 节点可订阅 Image/CameraInfo，按时间关联，
  要求显式 optical-frame 外参、整流声明、同图像 frame/尺寸和合法 pinhole 标定。
  不做时间插值，不隐含相机安装方向；地图 NPZ 保存/恢复视觉观测。
- 三个模块独立开关；接受的回环仍只有一条原有 ICP 图边和原有信息矩阵，
  不把重复点、视觉复核或表面复核当作额外独立信息。原运动一致性与修正准入门继续执行。

## 使用

默认不传参数即为原算法。先仅试多帧模块：

```bash
bash scripts/run_dynanav_native_exploration.sh "$NEW_RUN_DIRECTORY" \
  --backend-evidence-config configs/localization_mapping/slam_loop_evidence.json
```

`NEW_RUN_DIRECTORY` 必须是新的外部运行目录。配置例子只打开多帧模块；
配置的 backend 字典还可设置 `use_surface_verification` 和 `use_visual_verification`。
启用表面前确认前端 odom 的重力对齐约定，并设置 `gravity_aligned:true`。
启用视觉时配置根节点需提供以下字段，矩阵必须来自实测/发布标定，不能照抄单位阵：

```text
visual_input:
  rectified: true
  image_topic: <整流图像主题>
  camera_info_topic: <对应CameraInfo主题>
  T_body_camera: <4x4 optical相机到body实测外参>
  max_time_delta_s: 0.05
```

文件实际使用 JSON。离线导入也可在完整关键帧目录写 `visual.npz`，字段为
`gray`、`K`、`T_body_camera`、`stamp_ns`。现有 C++ 桥未改，在线通过 ROS 关联。
相机缓存有界，延迟或不同步会拒绝视觉回环；使用 trace 检查 `visual_verification` 状态。
没有足够相机观测时请保留视觉开关关闭，不能把拒绝改为通过。

## 真实录制离线检查

只读 `controlled-slam/20261116T154438Z-controlled-loops/map`，未使用 GT。
共 958 个关键帧；默认子图配置来源帧数中位数 4、最多 5；仅 2 个单帧窗口。
输出点数中位数 7799.5；窗口 body Z 跨度最大 0.499566 米。
累计 12,049,996 点合并成 7,388,825 体素，减少 38.68%。
这里是子图构造检查，不是后端完整重放或 Z 精度对照。

```bash
PYTHONPATH=src python3 -B -m embodied_navigation.localization_mapping.evaluation.measured_submaps \
  --map "$PROJECT_DATA_ROOT/embodied-navigation/runs/controlled-slam/20261116T154438Z-controlled-loops/map"
```

历史图像在 `capture/interactive_frames`，索引有 source_time_s/stamp_s，约 2 秒采样，
不是关键帧同步图像。旧地图 NPZ 只有 points_lidar/descriptor，因此不能用其证明视觉闭环性能。
证据：`git grep -n 'write_snapshot\|metadata.json' -- third_party/fast_livo2/fast_livo2_core/src/backend/snapshot_bridge.cpp`；
旧桥输出 metadata 和 points_lidar.pcd，没有 RGB/相机标定。

## 审计与限制

`scan_context.py:descriptor` 使用 body 原始 Z，零初始化后 `np.maximum.at`，
负高度被零吞掉，未按重力消除 roll/pitch；本版保持原描述子兼容，未修改历史地图格式。
候选召回仍可能漏检楼梯上下或低纹理区域，表面门也不能排除所有重复台阶混淆。
尚无可信的坡段语义/楼层归属，水平物体不能直接当作地面真值。
未做梯段连续曲面/接触因子、前端状态回灌、质量自适应协方差或在线 Z 校正。
本机未运行 ROS/Isaac/机器人，不宣称定位改进。服务器需记录同步整流图像、
CameraInfo 与 optical 外参，在同一录制对照旁路/各模块启用后的回环拒绝、Z 修正和轨迹误差。

## 测试

新增 35 项 evidence 测试覆盖多级台阶、平面/立面/未知、负高度、坐标/时间/外参、
连续子图、来源去重、缺图和错误匹配拒绝、保存加载、默认旁路及图信息不膨胀。
按 AGENTS 的难包规则，另一代理先按约定 API 独立编写验收，再做集成测试。
独立 LF 克隆 `8184b65` 全量单测 1249 项，48.731 秒，原有 1 fail + 4 error，
跳过 3 项；失败名单未新增：`test_dynanav_rtx_deskewed_perception` 4 项、
`test_dynanav_source_snapshot` 1 项。回归 81 项，10.093 秒，通过，原有跳过 9 项。
重连后先检查进程与输出，确认原测试完成，未并发重复运行。
本次另外增加 2 项生产调用链集成验收，专项共 35 项通过。新增测试后的独立 LF 克隆
全量复验 1251 项，48.403 秒，仍为同样 1 fail + 4 error、3 skip，失败名单未新增。
回归沿用相同生产代码的上述 81 项结果；本次追加仅测试和报告，未改生产逻辑。
命令：

```bash
PYTHONPATH=src python3 -B -m compileall -q src scripts
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
```

## 实际接线审计

下表以 `git grep -n 'verify_visual\|compare_surfaces\|build_submap\|_register' -- src/embodied_navigation/localization_mapping/backend` 的实际调用为据。

| 能力 | 生产路径与决策作用 | 尚未实现 |
| --- | --- | --- |
| 视觉 | `BackendNode.ingest → attach_visual → SlamBackend.ingest → _find_loop → _search`。`index.query` 先提名 Scan Context 候选，随后 `verify_visual` 验证；不足直接跳过该候选，通过后 `store.poses[key] @ visual.T_target_source` 初始化 ICP，ICP 与视觉不一致亦拒绝。 | 没有独立视觉候选索引；不能越过 Scan Context 召回限制，也不是视觉 bundle adjustment 或额外图边。 |
| 表面 | `ingest → model_surfaces` 的逐帧摘要是观察；另有 `_search → _register → model_surfaces/compare_surfaces`，后者失败会清空 ICP 结果，不能进入回环图。因此不是只有诊断。 | 不进入 ICP 残差/雅可比，不产生地面高度先验或独立图因子，不是持续楼梯地形优化。 |
| 多帧 | `ingest` 缓存原始关键帧，`_search → build_submap` 生成查询 source；`_find_loop → _search` 的 target 回调 `_loop_submap → build_submap` 生成候选端 target，两者实际传入 `_register → icp_point_to_plane` 的粗/细配准。 | 没有相对位姿协方差或联合局部 BA；窗口运动预算只控制使用范围。 |

生产模块的 `BackendConfig.use_measured_submaps/use_visual_verification/use_surface_verification`
均默认 false。启动器 `--backend-evidence-config` 复制 JSON 到新 run/configuration，
`node.main` 加载并传给 `SlamBackend`。示例 overlay 仅打开多帧。
无配置时保持原有算法；导航依然订阅原始前端里程计。

重定位边界：`_relocalize_step` 的 global search 调 `_search`，因此其查询端也使用新模块；
但 `_prior_submap` 仍是原有空间邻域，不是连续实测多帧窗口。
进入 tracking 后的 `_track` 直接对单帧和 prior 空间子图调用 `_register`，
不执行新视觉/表面门，也不使用新查询子图；不能宣称已覆盖整个先验跟踪链。
本次目标是建图模式下的回环验证，保持上述 tracking 路径原行为。

验收证据位于 `tests/unit/test_loop_evidence_acceptance.py`：
`test_production_loop_registration_receives_measured_historical_target` 检查实际 ingest→回环路径
传给 `_register` 的历史 target 含两个过去帧且不含近期排除帧；
`test_measured_source_contains_historical_returns_without_information_inflation` 检查查询 source
包含历史观测且信息矩阵不膨胀；`test_missing_visual_stops_before_icp_and_does_not_add_loop`
确认视觉不足阻断 ICP；`test_unobservable_surface_rejects_even_successful_icp` 与
`test_observed_surface_agreement_allows_original_icp_graph_edge` 分别确认表面门拒绝和接受。
这些集成测试用可控 ICP 返回值隔离接线行为，不能替代实测配准性能测试。
原后端测试另覆盖真实合成点云的 ICP 和闭环图流程。

## 下一轮最小独立视觉候选通路（本轮未实现）

下一轮先增加有界的轻量整流图像描述子索引，用于独立提名候选；
`_search` 对视觉 top-k 与 Scan Context top-k 做 key 并集，并记录候选来源。
视觉独立候选不能被 Scan Context 距离阈值提前截断；两类分数必须分别处理。
两个来源都经过相同 epoch/近期/时间排除、真实激光支撑的视觉 3D 几何验证、
ICP、运动预算和修正准入；图像相似度本身始终不产生图边。
关键验收是 SC 无候选但视觉独立召回且几何通过的回环、重复楼梯/错误外观匹配拒绝、
缺图不建立虚假候选、合并候选不重复加边、默认旁路图输出一致。
这比改变现有描述子和历史地图格式更适合作为下一步独立可测增量。
本轮没有添加该索引，不能将当前实现称为视觉独立回环检测。
