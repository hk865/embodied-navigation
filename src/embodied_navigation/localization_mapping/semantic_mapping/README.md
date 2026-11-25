# semantic_mapping：来源绑定的语义点层与关键帧场景图

本包是纯 CPU 的语义地图与场景图实现：包含两层互补能力——(1) `core.py` 持有静态/动态/未知点层、对象 ID 与可查询对象表；(2) `scene_graph.py` + `backend_adapter.py` 提供以 SLAM 关键帧位姿为锚点的层次化 3D 场景图（楼层、楼梯、房间、门洞、对象），支持回环位姿重投影、显式双图原子同步、流式 `MapStore` 构图与只读查询。本包**不含** ROS、模拟器、检测器、网络或模型加载依赖，导入即用、无副作用。

盘点日期 2026-11-23（分支 `feat/semantic-keyframe-graph`）。

## 模块定位与架构层级

- 架构层级：第 3 层「定位建图」下的语义子包（隶属 [localization_mapping](../README.md)），同时属于 Agent 感知与控制介入层的只读数据面。
- 输入：
  - 点层侧：`Observation`（点、`T_map_sensor`、epoch/sequence、frame/标定、`pose_source`）与可选 `PointMask`（逐点 `UNKNOWN/STATIC/DYNAMIC`、instance ID、标签、provenance）；查询侧输入 `LayeredMap.save()` 写出的 JSON 快照。
  - 场景图侧：`MapStore` 或磁盘目录（`map.json` + `.npz`，含子目录相对路径如 `keyframes/kf_*.npz`）以及可选的 `interactive_frames/index.jsonl` RGB 索引（支持 `stamp_ns` / `source_time_s` / `stamp_s` 与 `rgb_path` / `path` / `image_path`，逐行容错跳过坏行并对迁移归档路径回退到索引同目录 basename）。
- 输出：
  - 点层：`integrate()` 返回导航点云与按状态切分的点/索引；`snapshot()`/`save()` 产出 schema `layered_observed_map/v1`。
  - 场景图：`save()` 产出 schema `keyframe_scene_graph/v1`；`query()` 产出 `keyframe_scene_graph_query/v1`；`compact_agent_summary()` 产出 `keyframe_scene_graph_summary/v1`。
- 上游/下游与**当前接线状态**：
  - 离线入口：`scripts/build_semantic_scene_graph.py`（流式读取 `MapStore` 目录构建场景图并输出紧凑摘要/JSON）、`scripts/reconstruct_dynanav_slam_map.py`（用录制的 LIO 估计重建点层）、`scripts/query_scene_map.py`（读快照查询）。
  - 在线接线状态：**当前仓库在线后端循环（`backend/backend.py`）只更新 `MapStore.update_poses`，尚未自动调用 `sync_backend_loop_update`**（`git grep -n 'sync_backend_loop_update(' -- src scripts` 只命中 `backend_adapter.py` 定义）；回环同步目前通过显式调用 `sync_backend_loop_update(...)` 触发，在线 `publish_cloud` 的 `oracle_mapper` 参数仍传 `None`。
- 边界：不做感知推理（不跑检测/分割、不读图像/ROS/Isaac），mask 与对象检测由调用方给出；不把 oracle 结果重标为 learned；查询只读，不授予导航/执行器权限。

## 当前文件夹结构

```text
src/embodied_navigation/localization_mapping/semantic_mapping/
├── README.md            # 本文件
├── __init__.py          # 唯一门面：重导出核心、场景图、适配器与调度接口
├── core.py              # 点层、对象关联、修订/重投影、SE(3) 校验、快照与原子保存
├── scene_graph.py         # 关键帧锚定 3D 场景图（楼层/楼梯/房间/门洞/对象/重投影/查询）
├── hierarchical_memory.py # L0–L3 四层子地图检索、Attempt & Failure Ledger 黑名单与前置 STOP 校验
├── object_mesh_memory.py  # 纯 LiDAR Surface Nets 物体网格下界基线（L4 几何对照）
├── backend_adapter.py     # MapStore 流式/内存构图、RGB 索引容错解析、净空估计与原子双图同步
├── query.py               # 只读确定性点层快照过滤与结构化上下文
└── scheduler.py           # 传感器时钟上的语义准入
```

9 个跟踪文件（8 个 `.py` + 本 README）。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：定位、文件清单、规则、显式同步契约、离线 CLI 与测试。 | — |
| [__init__.py](__init__.py) | 唯一门面：重导出 `LayeredMap`、`Observation`、`PointMask`、`KeyframeSceneGraph`、`RgbIndexReport`、`build_scene_graph_from_map_dir`、`build_scene_graph_from_map_store`、`sync_backend_loop_update` 等。 | `from embodied_navigation.localization_mapping.semantic_mapping import KeyframeSceneGraph, ...` |
| [core.py](core.py) | 点层与对象表：两个 frozen dataclass 与 `LayeredMap`；提供 `validate_rigid_pose` 校验有限刚体 SE(3) 矩阵。 | `Observation`、`PointMask`、`LayeredMap`、`read_snapshot`、`validate_rigid_pose` |
| [scene_graph.py](scene_graph.py) | 关键帧锚定 3D 场景图：按连续轨迹分段与局部纵坡 `\|dz/ds\|` 聚类楼层并做 Z 平移不变峰抑制（跨 epoch/时间断层不连楼梯）；用 `cKDTree` + 轨迹邻接/跨轨迹自由空间守卫与扇区视线检查分割房间；按瓶颈局部接触与轨迹穿越顺序建门洞（持久化 `bottleneck_anchor_ids`）；支持引用完整性校验、空间距离过滤查询与幂等/拓扑自愈重投影。 | `KeyframeSceneGraph`、`FloorNode`、`StairEdge`、`RoomNode`、`DoorwayEdge`、`ObjectNode` |
| [hierarchical_memory.py](hierarchical_memory.py) | 四层子地图检索与治理：`L0` 全局拓扑（<2 KB）→ `L1` 房间摘要检索 → `L2` 物体与失败黑名单（带 `map_revision` 与 `stale` 门控）→ `L3` 按需关键帧 RGB；提供 `Attempt & Failure Ledger` 候选过滤与前置 `verify_stop_claim` 几何校验。 | `HierarchicalMemoryStore`、`AttemptRecord`、`StopVerificationResult` |
| [object_mesh_memory.py](object_mesh_memory.py) | 纯 LiDAR Surface Nets 物体级 3D 网格与关键帧锚定空间记忆基线（L4 纯几何下界对照）。 | `build_object_mesh_memory` |
| [backend_adapter.py](backend_adapter.py) | 后端桥接：逐扇区净空估计（无有效点云返回 `None` 未知）；`load_rgb_index_report` 容错解析；`build_scene_graph_from_map_dir` 调用 `MapStore.iter_keyframes_from_dir` 逐帧流式构图；`sync_backend_loop_update` 显式双图原子同步。 | `estimate_keyframe_clearance_m`、`estimate_keyframe_clearance_profile`、`load_rgb_index_report`、`load_rgb_index`、`build_scene_graph_from_map_dir`、`build_scene_graph_from_map_store`、`sync_backend_loop_update` |
| [query.py](query.py) | 只读、确定性快照过滤：`select`/`context_text`（schema `observed_scene_map_query/v1`）。 | `select`、`context_text` |
| [scheduler.py](scheduler.py) | 传感器时钟上的语义准入（周期/事件/epoch）。 | `SemanticScheduler.request/complete` |

## 显式同步契约（`sync_backend_loop_update`）

调用方将后端回环位姿同步至 `KeyframeSceneGraph` 与可选 `LayeredMap` 时遵守以下契约：

- **批前原子校验**：先校验全部输入位姿为合法 SE(3)、合成目标传感器位姿，并核对所有匹配锚点的修订号；任意锚点遇到过期修订或同修订号冲突位姿时立即抛 `ValueError`，在修改任一图之前原子拒绝。
- **修订号规则**：未传 `revision`（默认 `None`）时，取 `LayeredMap` 当前最大锚点修订号 `+1` 单调递增；重复投递相同修订号且位姿一致时幂等跳过。
- **观测锚点映射与外参合成**：支持 `anchor_to_keyframe_map`（如将 `rgbd:{seq}` 映射到后端 `{epoch}:{seq}`）与 `T_keyframe_sensor`（默认单位阵，支持单矩阵或按锚点字典），按 `T_map_sensor = T_map_keyframe @ T_keyframe_sensor` 合成后更新 `LayeredMap`。关联与外参可随 `KeyframeSceneGraph` 持久化。

## 磁盘布局、离线 CLI 与查询入口

- **MapStore 磁盘布局（`localization_mapping/backend/map_store.py`）**：`map.json`（schema `embodied_navigation_map/v1`）允许平铺（`kf_1_0.npz`）或地图目录内子目录相对路径（`keyframes/kf_1_0.npz`），加载与流式读取（`MapStore.iter_keyframes_from_dir`）均校验路径位于地图目录内并保留子目录结构。
- **场景图离线构建 CLI**：

  ```bash
  PYTHONPATH=src python3 scripts/build_semantic_scene_graph.py \
      --map-dir "$RUN/map" \
      [--rgb-index-jsonl "$RUN/capture/interactive_frames/index.jsonl"] \
      [--output "$RUN/scene_graph.json"] [--summary]
  ```

  CLI 通过 `build_scene_graph_from_map_dir` 流式逐帧提取净空并立即释放点云，显著降低大规模地图的峰值内存；RGB 索引中的坏行号与跳过数会输出至 `stderr`。
- **点层离线 CLI**：`scripts/reconstruct_dynanav_slam_map.py` 与 `scripts/query_scene_map.py`。

## 验证与测试入口 / 配置注意

- 点层单测：[tests/unit/test_layered_mapping.py](../../../../tests/unit/test_layered_mapping.py)。
- 场景图与后端适配单测：[tests/unit/test_semantic_scene_graph.py](../../../../tests/unit/test_semantic_scene_graph.py)，覆盖斜坡/漂移平台楼层聚类、Z 平移不变性、跨 epoch/时间断层不连楼梯、门洞锚点持久化与重投影、房间拆分/迁移/幂等、走廊相邻建边、平行双走廊隔墙防穿墙、未知净空 `None`、流式子目录读取与路径越界拒绝、RGB 索引容错与归档解析、SE(3)/引用完整性校验、远距离空间查询过滤、代表图上限以及双图原子同步/外参合成。
- 全量 CPU 测试：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 配置：本包**不读环境变量**、不写日志框架；所有阈值均为方法/构造器关键字参数默认值，调用方可显式覆盖。
