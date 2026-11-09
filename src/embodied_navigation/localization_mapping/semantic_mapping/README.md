# semantic_mapping：来源绑定的语义点层

本包是纯 CPU 的观测地图小实现：持有静态/动态/未知点层、对象 ID 与可查询对象表，**不含** ROS、模拟器、检测器、网络或模型加载依赖，导入即用、无副作用。真实的传感器点、同刻位姿与逐点分类证据都由 live adapter 提供；本仓库里负责真实话题接线的是 DynaNav 适配层（[../../../simulation/dynanav/README.md](../../simulation/dynanav/README.md)）。

`LayeredMap` 把持久的静态贡献与动态/未知点分开。当 mask 缺失、过期或不兼容时，`integrate()` **总是**返回完整的自有导航点云；这些点以"未知"隔离，不进入静态持久层。既有障碍物消费者必须继续收到完整点云；如果前端要"只含静态"的话题，必须另开一条分支，不能替换现有输出。

## 边界

- 不做感知推理：不跑检测/分割模型，不读图像、不读 ROS、不读 Isaac；分类证据（mask）由调用方给出。调用方负责推理与超时处理（`SemanticScheduler` 只做准入）。
- 不把 oracle 说成 learned：`pose_source="slam"` 才是真实 SLAM 位姿；仿真位姿显式标 `sim_oracle`，mask 来源另标 `sim_oracle`/`learned`/`geometric` 及 producer/version。快照 provenance 报告 oracle 使用，**绝不把 oracle 结果重标为 learned 感知**。
- 不承诺语义正确性：标签与 mask 是证据，不是检测召回率或仿真性能的保证；`complete` 只描述已保留观测的 mask 覆盖与容量，不表示整场景已探索或语义正确。
- 不授予控制权：查询只读，不下发导航/执行器权限；对象历史与 `velocity` 都不代表真实刚体速度或可通行性。

## 目录结构

```text
src/embodied_navigation/localization_mapping/semantic_mapping/
├── README.md      # 本文件
├── __init__.py    # 重导出 Observation/PointMask/LayeredMap/SemanticScheduler/read_snapshot（23 行）
├── core.py        # 点层、对象关联、修订/重投影、快照与原子保存（670 行）
├── query.py       # 只读、确定性的快照过滤与结构化上下文（130 行）
└── scheduler.py   # 传感器时钟的语义准入（周期/事件/epoch），不做推理（52 行）
```

4 个 Python 文件、合计 875 行（2026-11-09 实测）；`core.py` 670 行是唯一的大文件。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `core.py` | `Observation`/`PointMask` 两个 frozen dataclass（构造时校验 epoch/sequence/时间/位姿/形状）、`LayeredMap`（`integrate/revise_mask/update_anchor/static_points/query/summary/snapshot/save`）、`read_snapshot`。常量 `UNKNOWN, STATIC, DYNAMIC = 0, 1, 2`。 |
| 代码 | `scheduler.py` | `SemanticScheduler.request(source_ns, epoch=…, event=…)` 返回是否准入；`complete(source_ns, epoch=…)` 只释放**被准入的**那一个任务（陈旧完成不会清掉新任务）。epoch 变化或时钟倒退会拒绝。 |
| 代码 | `query.py` | 两个纯函数：`select(snapshot, label=,near=,radius=,include_expired=)` → 确定性 dict；`context_text(result)` → 确定性紧凑文本。**只 import json/math，不 import `core`**。 |
| 代码 | `__init__.py` | 重导出，使 `from embodied_navigation.localization_mapping.semantic_mapping import LayeredMap, …` 可用。 |
| 日志/输出 | `LayeredMap.save(path)` 写的 JSON | 唯一产物。默认 `path.parent.mkdir(parents=True, exist_ok=True)`，用 `NamedTemporaryFile` + `flush` + `os.fsync` + `os.replace` 原子替换；schema `layered_observed_map/v1`，`read_snapshot` 只认这个 schema。 |
| 日志/输出 | `snapshot()` 的结构 | 含 `schema`、`map_id/epoch/frame_id/revision/source_ns`、`geometry_included`、`observation_count`、`observation_point_counts{static,dynamic,unknown}`、`mask_rejections`、`capacity_rejections`、`complete`、`capacity_complete`、`mask_coverage_complete`、`semantic_label_coverage_complete`、`semantic_coverage_complete`（legacy 别名，旁边有 `semantic_coverage_complete_is_legacy_mask_alias=true`）、`label_coverage_scope`、`provenance_summary{mask_kinds,pose_sources,sim_oracle_used}`、`observations[]`（每条保留 observation_id/source_ns/anchor_id/mask_status/pose_source/mask_provenance 与三类计数）。 |
| 日志/输出 | `scene_map.json`（实际文件名由调用方定） | 本包**不指定路径**：DynaNav 的 learned/oracle 适配层、`scripts/reconstruct_*.py` 决定写到哪里；`scripts/query_scene_map.py` 与 `scripts/reconstruct_dynanav_learned_person_map.py` 负责读。 |
| 日志/输出 | 无日志框架 | `grep -n "import logging\|getLogger\|print(\|environ\|getenv" *.py` 命中 0：不写日志、不 print、不读环境变量。诊断靠快照里的计数/标志。 |
| 配置 | 无配置读取点 | 不读 `configs/`；所有界限都是 `LayeredMap.__init__` 的关键字参数默认值：`voxel_size=0.08`、`max_observations=512`、`max_points_per_observation=8192`、`max_objects=256`、`history_length=32`、`max_mask_age_ns=2 s`、`object_ttl_ns=5 s`、`association_distance=1.0`、`movable_labels=("person","pedestrian","human")`。调用方可覆盖（DynaNav learned 用 `max_observations=2000`，oracle 用 `301`）。 |

## 模块内文件耦合

依赖方向：`__init__ → {core, scheduler}`，`scheduler → core`（只借 `_integer` 校验），`query` **独立**——它不 import `core`，只按 dict 键操作快照。没有 import 环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `core.py` | 670 | 标准库 + NumPy | `__init__`、`scheduler`、`query` 的调用方（经快照 dict，非 import） | 偏重：20 个函数里 `_associate` 99 行、`query` 85 行、`summary` 74 行 |
| `query.py` | 130 | 仅 json/math | `__init__`?（否，仅被 CLI/脚本直接 import）、DynaNav learned 适配层、`scripts/query_scene_map.py`、`scripts/reconstruct_dynanav_learned_person_map.py` | 清楚：纯函数 + 显式 ValueError |
| `scheduler.py` | 52 | `core._integer` | `__init__`、DynaNav oracle 适配层 | 清楚：状态机 3 个方法 |
| `__init__.py` | 23 | `core`、`scheduler` | DynaNav（4 处）、3 个 `scripts/*.py` | 清楚：唯一门面 |

四类问题：

- **上帝文件**：`core.py` 670 行，低于 800 行阈值，但它是"点层 + 对象关联 + 历史 + 快照 + 保存"五种职责的合集；`_associate` 99 行同时做空间关联、TTL、标签与 epoch 身份判定。
- **重复实现**：无。
- **跨层直达**：无 import 层面的跨层；但 `query.py` 与 `core.py` 之间是**dict 契约**（`query` 不认识 `Observation`/`PointMask`，只读 `snapshot["objects"]`/`["map"]`），改快照键名会静默破坏查询——这是本模块最需要留意的隐式契约。
- **隐式契约**：① 快照 dict 的键名与 `context_text` 里硬编码的字段名列表；② `PointMask.provenance` 必须含 `kind`（`summary` 直接 `r["mask"].provenance["kind"]`，缺 key 会 `KeyError`）；③ `Observation.points_sensor` 与 `PointMask.states/instance_ids` 必须同长且同序（README 示例与 `_states` 都依赖"同一最终采样顺序"，代码只做长度校验）；④ `anchors` 由 `anchor_id` 字符串索引，调用方必须保证一 anchor 一位姿。

结论：耦合**清楚**（文件少、方向单向、query 与核心解耦），但"dict schema + 顺序契约"是主要维护风险；`core.py` 的职责密度偏高但尚未到必须拆的程度。

## 可读性评估

- **良好**：`query.py`——2 个函数、docstring 覆盖 1.0、模块 docstring 声明"只过滤已有记录，不推断缺失对象/楼层/关系"；每个 `ValueError` 都说明了输入契约；`context_text` 的字段列表集中，输出可复现。
- **可接受**：`scheduler.py`——52 行、3 个函数、docstring 0.333，`complete` 的 docstring 点明"陈旧完成不能清掉新任务"这一关键语义；`__init__.py` 23 行纯重导出 + `__all__`。
- **需改进**：`core.py`——670 行、函数 docstring 0.25（20 个函数只有 5 个有）、**注释行仅 1 行**（按事实包 AST 统计）；`LayeredMap` 类 docstring 写得好（"持久静态几何只包含带有效 STATIC 证据的点"），但 `integrate`/`_associate`/`_states`/`update_anchor` 这些最需要解释的成员函数没有 docstring；`_states` 的 mask 兼容性判定依赖多个条件，缺注释。
- 这是**结构问题较轻、文档问题较重**的典型：单包 875 行、无环、有 20 个行为用例覆盖，缺的是 `core.py` 的长函数说明与键名契约文档。

## 重构建议

本轮只写建议、不改代码。已有 20 个行为用例（见"CPU 检查"），改动前先跑它们。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `LayeredMap.integrate/_states/_associate/update_anchor/query` 补 docstring，写清 mask 兼容性、UNKNOWN 隔离、ID 关联与修订语义 | `core.py` 670 行仅 1 行注释；这些成员是唯一被外部依赖的行为面 | 无（纯文档） | 人工复核；`test_layered_mapping.py` 20 项不变 |
| P2 | 把快照 dict 的键名收敛为常量（或 `TypedDict`），`query.select`/`context_text`/`summary` 共用 | 目前键名在 core、query、`scripts/query_scene_map.py`、DynaNav 四处按字符串重复 | 中：会碰到 CLI 与脚本 | 先用一个"保存→读取→查询"闭环用例锁定键集合，再替换 |
| P2 | 在 `_associate` 内部分段注释（空间关联 / TTL / 标签 / epoch 身份） | 99 行单函数承担 4 类判定，读一遍很难分清失败分支 | 低（纯注释） | 人工复核 |
| P3 | 校验 `provenance["kind"]` 缺失时给明确错误，而不是让 `summary()` 抛 `KeyError` | 现在契约失败表现为内部异常 | 低 | 加一个不带 kind 的 mask 用例，断言错误消息 |

没有 P0/P1：本包不接触控制/执行器，`integrate` 的"完整点云永远返回"与"UNKNOWN 隔离"两条安全语义已有测试直接覆盖；875 行、无环，拆包收益不足以抵消 API 迁移成本。

## Live adapter 用法

```python
from embodied_navigation.localization_mapping.semantic_mapping import (
    Observation, PointMask, LayeredMap, SemanticScheduler, STATIC, DYNAMIC, UNKNOWN,
)

store = LayeredMap(max_observations=512, max_points_per_observation=8192)
semantics = SemanticScheduler(interval_ns=2_000_000_000,
                              event_min_interval_ns=500_000_000)

# xyz 与 states/instance_paths 必须用同一个最终采样顺序。
# 像素投影、自车 mask 与 stride 已在前端一致处理过。
obs = Observation(epoch=epoch, sequence=sequence, source_ns=source_ns,
                  pose_source_ns=source_ns, points_sensor=xyz,
                  T_map_sensor=T_map_imu, anchor_id=f"frame:{sequence}",
                  frame_id="go2_imu", calibration_id="rgbd:v1",
                  pose_source="sim_oracle")  # 诊断用 GT 图；不是 SLAM 结果
proof = PointMask(epoch=epoch, sequence=sequence, source_ns=source_ns,
                  produced_ns=source_ns, frame_id="go2_imu", calibration_id="rgbd:v1",
                  states=states, instance_ids=tuple(instance_paths), labels=labels,
                  provenance={"kind": "sim_oracle", "producer": "isaac.instances",
                              "instance_id_scope": "epoch"})
split = store.integrate(obs, proof, now_ns=source_ns)
# 发布完整导航源；不要用 static_points 替换它。
navigation_xyz = split["navigation_points_sensor"]
static_xyz = split["static_points"]
static_source_indices = split["static_indices"]  # 同时用于索引原始 RGB

# 昂贵/类别标注与廉价帧 mask 分开调度。
if semantics.request(source_ns, epoch=epoch, event=novel_scene):
    # 只提交一个有界异步任务；本包不跑推理。
    # 完成/取消后，只释放被准入的那一个 source/epoch token。
    semantics.complete(source_ns, epoch=epoch)

compact = store.summary(now_ns=source_ns)  # 不物化/不序列化整张点图
store.save("scene_metadata.json", now_ns=source_ns, include_points=False)
# 最终导出或显式请求时：
static_map_xyz = store.static_points()
objects = store.query(label="person", center=[0, 0, 0], radius=5,
                      now_ns=source_ns, include_history=False)
```

`pose_source="slam"` 才是真实 SLAM 位姿；仿真位姿显式标 `sim_oracle`，mask 来源另标 `sim_oracle`/`learned`/`geometric` 及 producer/version 字段。快照 provenance 报告 oracle 使用，绝不把 oracle 结果重标为 learned 感知。**匹配上 header 不等于证明渲染采集时刻**：传感器适配层必须自行校验实际 RGB/深度时间与标定。只用于真值打分的 NPZ 位姿不得被静默当作 SLAM 估计传入。

## 掩码、身份与修订规则

- mask 的 epoch、sequence、源时间戳、frame、标定与点数必须与 observation 一致。生产时间必须落在"测量时刻"与"调用方给的当前仿真时钟"之间，且在 `max_mask_age_ns` 内。缺失/无效证据一律给 UNKNOWN，**绝不默认 STATIC**。上一帧图像的 mask 不能因为点数相同就去分类新图像。
- `STATIC=1, DYNAMIC=2, UNKNOWN=0`。整组含动态点、或被标为 `person`/`pedestrian`/`human` 的组，整体排除出静态积分——站着的人仍然是可移动物。标签与 mask 是证据，不保证检测召回或仿真表现。
- `instance_id_scope="epoch"` 明确承诺"源身份持久"：ID 被 epoch、来源类型、producer 与源实例路径共同绑定，oracle USD 路径能在遮挡、质心漂移与类别更新后保持内部对象 ID；不同路径不能靠空间关联合并。默认的 `frame` 作用域用带门限的"类别兼容 + 3D 最近邻"关联，并列歧义时分配新 ID。这不是通用视觉 re-ID。
- 查询对象与每条历史都保留原始 instance ID、epoch/frame/calibration、源时间戳、位姿与 mask provenance。全 XYZ 关联避免"同 XY 不同楼层"被投影合并。不推断房间/楼梯可通行性，也不声称真实刚体速度；`velocity` 是观测到的可见质心在修订后地图坐标下的有限差分，部分可见与位姿校正都会改变它。
- `revise_mask(observation_id, proof, now_ns=…)` 替换该保留观测的贡献；当被跟踪对象转为动态时，与它关联的更早贡献也一并排除。修订后应从 `static_points()` 重建持久导出；把更早的增量点云永久追加到不相关归档会破坏这种分离。
- `update_anchor(anchor_id, T_map_sensor, revision=…)` 要求位姿 revision 递增，会重投影点与历史并保留对象 ID。一个实际位姿只对应一个源传感器/关键帧 anchor；新的时钟 epoch 必须开新的地图段，混 epoch 会被拒绝而不是拼接。

## 有界存储与产物语义

在选定的地图关键帧上调用 `integrate`，同时让全速率的导航与 mask 发布保持独立。按 1–2 Hz、90 秒估算，默认 512 条观测上限够用——这是**容量计算，不是实测运行时预算**。单观测点数上限 8192；轨迹上限 256 条、每条 32 个趋势项。容量拒绝会被计数并把地图标记为不完整，但**不会**丢掉已返回的完整导航点云，也不会驱逐已接受的几何。

`complete` 只关心已保留观测的 mask 覆盖与容量，从不表示整场景已探索或语义感知正确。`mask_coverage_complete` 表示"每条保留点都有有效的非 UNKNOWN mask 证据"；legacy 的 `semantic_coverage_complete` 作为该 mask 标志的显式别名保留（旁边有 `semantic_coverage_complete_is_legacy_mask_alias=true`）。`semantic_label_coverage_complete` 才问"保留的已知点是否还有实例 ID 与非 unknown 类别标签"；`scene_geometry` 这类通用 oracle 标签也能满足该标志，但它不代表 learned 语义或细粒度房间/物体含义。

`summary()` 产出紧凑计数、provenance 与当前对象，不构建/序列化点图。`snapshot(include_points=False)` 保留观测/历史证据但省略完整几何。`save` 原子替换一份自包含 JSON 修订；默认对小地图包含几何，因此**实时遥测应显式用 `include_points=False`**。`read_snapshot` 加载这份已提交表示供 agent 查询/检查。ID 会持久化到导出产物；从 checkpoint 恢复可变跟踪不在本 API 范围内。查询只读，不授予导航/执行器权限。

## Agent 查询入口

```sh
python3 scripts/query_scene_map.py /data/run/scene_map.json --label person \
  --near 0 0 1 --radius 5
python3 scripts/query_scene_map.py /data/run/scene_map.json --include-expired --context
```

默认输出确定性 JSON：过滤后的对象 + 原始地图/证据元数据，省略大块静态点。`--context` 输出紧凑的确定性结构化文本。查询用的是快照里记录下来的仿真查询时刻，**不是**机器当前墙钟，也不声称地图是实时的。过滤要求"记录地图 frame 下的有限三维中心 + 非负半径"。坐标/时钟/位姿/mask provenance、UNKNOWN 计数与容量标志都保持可见。不推断缺失楼层标签、对象关系或场景完整性。`--include-expired` 会包含快照 owner 已标为过期的轨迹；查询本身不修改地图、也不重新识别对象。

## CPU 检查与限制

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover \
  -s tests/unit -p test_layered_mapping.py -v
```

[tests/unit/test_layered_mapping.py](../../../../tests/unit/test_layered_mapping.py) 有 20 个用例，覆盖隔离、完整导航点云保留、迟到撤销、站立与移动对象、来源作用域 ID、楼层分离、位姿重投影、紧凑原子产物、容量界限与周期/事件/epoch 调度；另有 [tests/unit/test_dynanav_source_snapshot.py](../../../../tests/unit/test_dynanav_source_snapshot.py) 用 AST 断言 DynaNav 的源码快照包含 `semantic_mapping/query.py` 与 `scripts/query_scene_map.py`（防止"归档了查询逻辑却没归档实现"）。这些测试**不**跑语义模型、ROS、Isaac、native SLAM 或机器人。

只有本包与它的 CPU 测试属于本实现范围：SLAM 之后过滤归档，**不能**证明 FAST-LIVO 内部几何/视觉地图排除了动态物体；前端 mask 准入与真实话题接线需要各自的集成与仿真检查（本仓库里由 DynaNav 适配层负责，见 [../../../simulation/dynanav/README.md](../../simulation/dynanav/README.md)）。
