# semantic_mapping：来源绑定的语义点层

本包是纯 CPU 的观测地图小实现：持有静态/动态/未知点层、对象 ID 与可查询对象表，**不含** ROS、模拟器、检测器、网络或模型加载依赖，导入即用、无副作用。真实的传感器点、同刻位姿与逐点分类证据都由调用方提供，本包只负责证据绑定、有界存储与只读查询。

盘点日期 2026-11-20（分支 `refactor/06-clean-base`，HEAD 5b880b2）。

## 模块定位与架构层级

- 架构层级：第 3 层「定位建图」下的语义子包（隶属 [localization_mapping](../README.md)），同时属于 Agent 感知与控制介入层的只读数据面。
- 输入：`Observation`（点、`T_map_sensor`、epoch/sequence、frame/标定、`pose_source`）与可选的 `PointMask`（逐点 `UNKNOWN/STATIC/DYNAMIC`、instance ID、标签、provenance）；查询侧输入是 `LayeredMap.save()` 写出的 JSON 快照。
- 输出：`integrate()` 返回完整导航点云与按状态切分的点/索引；`snapshot()`/`save()` 产出 schema `layered_observed_map/v1` 的自包含快照；`query.select`/`context_text` 产出确定性过滤结果与紧凑上下文文本。
- 上游/下游：仓库内调用方为 `scripts/reconstruct_dynanav_slam_map.py`（用录制的 LIO 估计重建点层）、`scripts/query_scene_map.py`（读快照查询）、`simulation/dynanav/capture/ros_interface.py`（只 import `STATIC` 常量，用于发布 `/dynanav/lidar_static`）与测试；当前 HEAD 的在线 mapper 未接线（`git grep oracle_mapper` 只命中 `publish_cloud` 的参数，实际调用点传 `None`）。
- 边界：不做感知推理（不跑检测/分割、不读图像/ROS/Isaac），mask 由调用方给出；不把 oracle 结果重标为 learned（`pose_source` 只能是 `slam` 或 `sim_oracle`，mask `kind` 只能是 `sim_oracle`/`learned`/`geometric`）；查询只读，不授予导航/执行器权限。

## 当前文件夹结构

```text
src/embodied_navigation/localization_mapping/semantic_mapping/
├── README.md      # 本文件
├── __init__.py    # 23 行：唯一门面，重导出 7 个名字 + __all__ 9 项
├── core.py        # 670 行：点层、对象关联、修订/重投影、快照与原子保存
├── query.py       # 130 行：只读确定性过滤与结构化上下文
└── scheduler.py   # 52 行：传感器时钟上的语义准入
```

5 个跟踪文件（4 个 `.py` + 本 README），共 875 行。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：定位、文件清单、掩码/身份/容量规则、查询入口、测试。 | — |
| [__init__.py](__init__.py) | 唯一门面：重导出 `DYNAMIC/STATIC/UNKNOWN`、`LayeredMap`、`Observation`、`PointMask`、`SemanticScheduler`、`read_snapshot`。 | `__all__`；`from embodied_navigation.localization_mapping.semantic_mapping import LayeredMap, …` |
| [core.py](core.py) | 点层与对象表：两个 frozen dataclass 在构造时校验 epoch/sequence/时间/位姿/形状/provenance；`LayeredMap` 做有界积分、对象关联、延迟 mask 修订、位姿重投影、快照与原子保存。 | `Observation`（`observation_id = "{epoch}:{sequence}"`）、`PointMask`、`LayeredMap.integrate/revise_mask/update_anchor/static_points/query/summary/snapshot/save`、`read_snapshot`；常量 `UNKNOWN, STATIC, DYNAMIC = 0, 1, 2` |
| [query.py](query.py) | 只读、确定性查询：按 `label`/`near`+`radius`/`include_expired` 过滤已有记录，绝不推断缺失对象、楼层、关系或控制权。 | `select(snapshot, *, label, near, radius, include_expired)`（schema `observed_scene_map_query/v1`）、`context_text(result)`（`observed_scene_map_context/v1`）；只 import `json`/`math`，不 import `core` |
| [scheduler.py](scheduler.py) | 传感器时钟上的语义准入（周期/事件/epoch），调用方负责推理与超时。 | `SemanticScheduler.request(source_ns, *, epoch, event=False)` → bool、`complete(source_ns, *, epoch)`（只释放被准入的那一个任务，陈旧完成不会清掉新任务）；计数器 `requested/skipped_busy/skipped_interval` |

## 掩码、身份与修订规则（对应 `core.py`）

- `PointMask` 的 epoch、sequence、源时间戳、frame、标定与点数必须与 `Observation` 一致；`produced_ns` 必须落在 `source_ns` 与调用方给的 `now_ns` 之间，且 `now_ns - source_ns <= max_mask_age_ns`（默认 2 s）。缺失/过期/不匹配一律给 UNKNOWN，**绝不默认 STATIC**。
- 整组含动态点、或被标为 `person`/`pedestrian`/`human`（`movable_labels`）的 instance，整组排除出静态积分——站着的人仍算可移动物。
- `instance_id_scope="epoch"` 用 `(epoch, provenance.kind, provenance.producer, instance)` 作为稳定键；默认 `frame` 作用域用「类别兼容 + 3D 最近邻（`association_distance`）+ `object_ttl_ns`」关联，并列歧义时分配新 ID 而不是合并。这不是通用视觉 re-ID。
- `revise_mask(observation_id, mask, *, now_ns)` 只替换该保留观测的贡献；对象转动态后，与它关联的更早贡献在 `_effective_states` 里一并排除。`update_anchor(anchor_id, T_map_sensor, *, revision)` 要求 revision 递增，并在导出时重投影。
- 有界存储默认值：`voxel_size=0.08`、`max_observations=512`、`max_points_per_observation=8192`、`max_objects=256`、`history_length=32`、`max_mask_age_ns=2_000_000_000`、`object_ttl_ns=5_000_000_000`、`association_distance=1.0`。容量拒绝会计数并把地图标为不完整，但**不会**丢掉已返回的完整导航点云。

## 产物与查询入口

- `LayeredMap.save(path, *, now_ns=None, include_points=True)`：写 schema `layered_observed_map/v1`，用同目录 `NamedTemporaryFile` + `flush` + `os.fsync` + `os.replace` 原子替换；`read_snapshot(path)` 只认这个 schema。实时遥测应显式用 `include_points=False`，避免每次写整张静态点图。
- 本包**不指定产物路径**：写到哪里由调用方决定（`scripts/reconstruct_dynanav_slam_map.py`、DynaNav 适配层）。`summary()` 只产出计数/provenance/当前对象，不物化点图。
- CLI：`python3 scripts/query_scene_map.py <snapshot.json> [--label person] [--near 0 0 1 --radius 5] [--include-expired] [--context]`，默认输出确定性 JSON，`--context` 输出紧凑结构化文本；查询时钟是快照里记录的 `query_clock_ns`，不是机器当前墙钟。
- 快照里 `semantic_coverage_complete` 是 `mask_coverage_complete` 的 legacy 别名（旁边有 `semantic_coverage_complete_is_legacy_mask_alias=true`）；`semantic_label_coverage_complete` 才问「保留的已知点是否都有 instance ID 与非 unknown 标签」。`complete` 只描述已保留观测的 mask 覆盖与容量，不表示整场景已探索或语义正确。

## 验证与测试入口 / 配置注意

- 行为用例：[tests/unit/test_layered_mapping.py](../../../../tests/unit/test_layered_mapping.py) 20 个用例，覆盖 UNKNOWN 隔离、完整导航点云保留、迟到修订、站立/移动对象、epoch 作用域 ID、楼层分离、位姿重投影、原子产物、容量界限与周期/事件/epoch 调度。
- 来源快照：[tests/unit/test_dynanav_source_snapshot.py](../../../../tests/unit/test_dynanav_source_snapshot.py) 用 AST 断言 DynaNav 的源码快照包含 `semantic_mapping/query.py` 与 `scripts/query_scene_map.py`。
- 单测命令：`PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover -s tests/unit -p 'test_layered_mapping.py' -v`；全量 CPU 测试 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 配置：本包**没有配置读取点**（不读 `configs/`、不读环境变量、不写日志框架），所有界限都是 `LayeredMap.__init__` 与 `SemanticScheduler.__init__` 的关键字参数默认值，调用方可覆盖。
- 边界提醒：只有本包与它的 CPU 测试属于本实现范围。SLAM 之后过滤归档**不能**证明 FAST-LIVO2 内部几何/视觉地图排除了动态物体；前端 mask 准入与真实话题接线需要各自的集成与仿真检查，未在本次文档盘点中运行。
