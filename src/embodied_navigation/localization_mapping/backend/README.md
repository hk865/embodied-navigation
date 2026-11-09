# localization_mapping/backend

本目录是可选的后端定位组件：在与前端分离的线程或进程中收集关键帧、对外部地图做配准、验证回环并做 SE(3) 图优化，最后输出可被定位 owner 采纳的校正候选。整包无 ROS 依赖，导入模块不启动线程，只有 `BackendWorker(...)`、`run.main()` 这类显式构造/入口才有副作用。

## 边界

- 不做前端 IMU/滤波/里程计，也不写前端状态。
- 不直接发布导航姿态。worker 可通过 `poll` 返回候选，也可将候选写入结果文件；当前 Building 路径由 [../building_runtime.py](../building_runtime.py) 读取 `backend_result.json` 并决定是否应用。`../localization_runtime.py` 使用独立的已知场景匹配链路，不经过本目录。
- CPU Python 配准是参考实现，不代表 C++ 或实时吞吐指标。几何提议不等于导航授权（见 `correction.py` 模块 docstring）。

## 目录结构

```text
src/embodied_navigation/localization_mapping/backend/
├── native/                # C++ SE(3) 图求解器源码与 CMake，见 native/README.md
├── README.md              # 本文件
├── __init__.py            # 仅包 docstring（4 行）
├── correction.py          # 时间确认、结果读取门限、位姿+点云同一校正（170 行）
├── external_map.py        # 无轨迹 PCD/外部地图与重叠空间分块（260 行）
├── geometry.py            # 后端专用 SE(3) 小工具：校验、逆、变换、刚性拟合、采样（82 行）
├── history.py             # 自有 LiDAR 关键帧与锚定 SE(3) 位姿图，持久化（316 行）
├── native_graph.py        # 通过 ctypes 加载 liblivo_graph.so 的有界图求解（73 行）
├── registration.py        # 有界 CPU 几何重定位：假设、精配准、可观测性判定（443 行）
├── run.py                 # 无 ROS 的 worker 进程入口，消费 C++ 关键帧 spool（171 行）
├── tracking.py            # 显式先验盆地内的无轨迹跟踪（156 行）
└── worker.py              # 有界尽力而为的后台 worker，写结果 JSON/JSONL（356 行）
```

10 个 Python 文件、合计 2031 行（2026-11-09 实测）；最大文件 `registration.py` 443 行，其次 `worker.py` 356 行、`history.py` 316 行。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（对外接口） | `worker.py` | `BackendWorker.try_submit/poll/close` + `stats`。`try_submit` 只入队（队列满/加锁失败/epoch 不一致/超点数都直接丢弃**新**输入，不丢已接受的图边）；`poll` 取最近结果；`close(timeout)` 返回"线程是否已退出"。 |
| 代码（算法） | `registration.py`、`tracking.py`、`history.py`、`external_map.py` | 纯函数+dataclass；`Relocalizer.localize`（143 行）与 `PriorTracker.localize`（114 行）各自返回 `RegistrationResult`，不可观测/歧义时返回 UNKNOWN 式结果而不是最佳猜测。 |
| 代码（契约/门限） | `correction.py` | `TemporalAdmission`（默认 `confirmations=2`、`minimum_dt_ns=150 ms`、`maximum_age_ns=3 s`、平移/旋转容差 0.15 m/0.08 rad）、`apply_bundle`（同一 C 同时作用于位姿与点云）、`BackendResultReader`（按文件 mtime、schema、来源年龄、epoch/sequence 单调、`C@T_odom_body≈T_world_body` 设门限）。 |
| 代码（原生桥） | `native_graph.py` | 固定加载 `install/backend/lib/liblivo_graph.so`，最多 80 个活跃节点；只接受 solver 返回 0、输出有限、`final_cost<=initial_cost` 的结果；库缺失时返回诊断报告而不抛异常。 |
| 代码（进程入口） | `run.py` | 参数 `--spool/--history/--external-map/--result/--initial-world-odom/--map-world-offset/--queue-capacity/--poll-s/--maximum-points`；有 `--external-map` 时按是否给 `--initial-world-odom` 选 `PriorTracker` 或 `Relocalizer`。 |
| 日志/输出 | `--result` 指向的文件（Building 传 `<run>/backend_result.json`） | `BackendWorker` 每处理一帧原子写一次，schema `livo_backend_correction/v1`，含 `accepted`、`reason`、`C_world_odom`、`T_odom_body`、`T_world_body`、`epoch/sequence/source_ns`、`published_wall_ns`。消费方是 `../backend/correction.py` 的 `BackendResultReader`，由 `../building_runtime.py` 轮询。 |
| 日志/输出 | `<result 同目录>/backend_results.jsonl` | 同一个 worker 以追加模式写的流水（每帧一行 JSON），**没有仓库内读取方**，属运行证据。 |
| 日志/输出 | `<result 同目录>/backend_status.json` | `run.main()` 每 1 秒（退出时再写一次）原子写：`schema=livo_backend_status/v1`、计数器、`worker.stats`、`backend_alive`、`drained`、`correction_authority=false`、`realtime_budget_verified=false`。无仓库内读取方。 |
| 日志/输出 | `--history` 目录下 `epoch_<n>/` | `History.save` 写 `history.json`（schema `livo_history/v1`）、`keyframe_<epoch>_<seq>.npy`、`submap_NNNNNN.npy`；`History.load` 校验 schema 并拒绝逃出目录的 `cloud` 路径。 |
| 日志/输出 | `--external-map` 目录（`ExternalMap.save`） | `points.npy`、可选 `colors.npy`、`map.json`（schema `external_xyz_map/v1`，`trajectory=null`）；`load` 拒绝带轨迹的地图。 |
| 输入（非本目录产物） | `--spool` 目录下 `kf_*/` | 由 vendored C++ 侧写出（[third_party/fast_livo2/fast_livo2_core/src/backend/snapshot_bridge.cpp](../../../../third_party/fast_livo2/fast_livo2_core/src/backend/snapshot_bridge.cpp) 写 `{"schema":"livo_keyframe/v1",…}` 与 `points_lidar.pcd`）；`run.decode` 校验 schema、点数上限后才交给 worker。本目录只消费。 |
| 配置 | 无 `configs/` 读取点 | 本模块不读 `configs/` 下任何文件；所有门限是构造参数或 `RegistrationConfig`/`TemporalAdmission` 默认值，运行参数全部来自 `run.py` 命令行。 |

`install/backend/lib/liblivo_graph.so` 在本检出**已存在**（76,960 字节，2026-11-09 实测）；若换机器缺失，只表示该机器未构建原生库，`optimize_graph` 会降级为诊断报告、历史收集照常。

## 模块内文件耦合

依赖方向是**单向分层**：`run.py`（进程入口）→ `worker.py`（生命周期/队列）→ `history.py`（关键帧+图）→ {`external_map.py`、`registration.py`、`tracking.py`、`native_graph.py`} → `geometry.py`（纯工具）；`correction.py` 被 `worker.py` 使用，同时其 `BackendResultReader` 被上层 `../building_runtime.py` 使用。**包内无 import 环**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `registration.py` | 443 | `geometry` | `history`、`run`、`tracking` | 偏重：13 函数中 `localize` 143 行，假设/精配准/可观测性都在一个方法里 |
| `worker.py` | 356 | `correction`、`geometry`、`history` | `run` | 偏重：`_run` 225 行是主循环+回环+图优化+落盘，但边界（"尽力而为、有新丢旧"）写在 docstring 里 |
| `history.py` | 316 | `external_map`、`geometry`、`native_graph`、`registration`、`tracking` | `run`、`worker` | 清楚：数据类 + `add/verify_and_add_loop/optimize/save/load` |
| `external_map.py` | 260 | `geometry` | `history`、`run` | 可接受：`read_pcd` 105 行要同时支持 ASCII/binary，长但线性 |
| `correction.py` | 170 | `geometry` | `worker`、`../building_runtime` | 清楚：三个小类/函数，docstring 明确"几何提议不是导航授权" |
| `tracking.py` | 156 | `geometry`、`registration` | `history`、`run` | 清楚：复用 `registration._refine/_observability`，是显式依赖私有函数 |
| `run.py` | 171 | `external_map`、`history`、`registration`、`tracking`、`worker` | 无（`launch/building.py`、`navigation/robogauge/benchmark.py` 以 `-m` 启动） | 清楚：`main()` 99 行，参数解析+信号+轮询 |
| `geometry.py` | 82 | 仅 NumPy/SciPy | `correction`、`external_map`、`history`、`registration`、`tracking`、`worker`、`../building_runtime` | 清楚：7 个 ≤13 行的纯函数 |
| `native_graph.py` | 73 | `embodied_navigation.project` | `history` | 清楚：单函数 ctypes 封装 |
| `__init__.py` | 4 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：无（最大 443 行，低于 800 行阈值）。
- **重复实现**：与根目录 `../known_scene_matcher.py:28` 各有一份 `checked_transform`，容差不一致（本目录末行 `atol=1e-8`，matcher 是 `1e-10`）。`tracking.py` 直接 import `registration._refine`/`_observability` 两个下划线函数，属于"同包内私有复用"，可接受但应视为接口。
- **跨层直达**：没有。`native_graph.py` 直接按 `install/backend/lib/liblivo_graph.so` 路径加载，是唯一的"越过 Python 契约"点，但它被 docstring 和 README 明确标注，且失败可降级。
- **隐式契约**：结果 JSON 的字段名（`C_world_odom`、`T_odom_body`、`T_world_body`、`epoch`、`sequence`、`published_wall_ns`）是 `worker.py` 写、`correction.BackendResultReader` 与 `../building_runtime.py` 读的字符串契约，没有 dataclass 约束；schema 版本字符串是唯一的版本闸门。

结论：耦合**清楚且合适**。入口→worker→算法→工具的单向依赖成立，算法层互不 import，唯一的跨包契约（结果 JSON）有 schema + 年龄 + epoch 三重校验兜底。主要成本在 `registration.localize`/`worker._run` 两个长函数，而不是结构。

## 可读性评估

- **良好**：`geometry.py`（函数 docstring 覆盖 0.714，7 个函数最长 13 行，模块 docstring 说明 `T_AB` 方向约定）、`native_graph.py`（1.0，`optimize_graph` docstring 写清数组转换/80 节点/接受条件）；`correction.py`（0.167 但 3 个类/函数都有类 docstring，`apply_bundle`/`BackendResultReader` 的语义写得很足）、`tracking.py`（0.333，`PriorTracker` 类 docstring 明确"不代表全局唯一"）。
- **可接受**：`registration.py`（0.231，`localize` 143 行、`_coarse_hypotheses` 50 行；但门限集中在 `RegistrationConfig` dataclass，命名带单位后缀 `_m`/`_rad`，这是它可读的主要原因）、`history.py`（0.2，`save` 63 行要写 3 类文件，逻辑线性）、`external_map.py`（0.2，`read_pcd` 105 行但只做格式分支）。
- **需改进**：`worker.py`（**函数 docstring 0.0**，`_run` 225 行同时做队列消费、回环确认、图优化、统计与写盘；`__init__` 59 行铺 22 个字段）、`run.py`（0.0，`main` 99 行；模块 docstring 里的启动命令已是 `embodied_navigation...` 路径，旧 README 说的"docstring 仍是 `localization.backend.run`"在当前 checkout 已过时）、`correction.TemporalAdmission.consider`（64 行、无 docstring）。

没有虚构结构缺陷：这三个文件的分层是清楚的，缺的是 docstring 与"长函数里各段落的分界注释"；`worker.py` 里有少量分界注释但不足以替代文档。指标抽查：AST 复算 `backend/run.py`（4 函数、docstring 0.0、最长 `main` 99 行）、`worker.py`（6 函数、0.0、最长 `_run` 225 行）与事实包一致。

## 重构建议

本轮只写建议、不改代码；改动前先跑现有的 3 个回归文件（纯 CPU、不需要 ROS）。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `BackendWorker.__init__`/`_run`、`run.main`、`Relocalizer.localize` 补 docstring 并加段内分界注释（队列/回环/图/写盘） | 这 4 个函数是接手者必读，却是 docstring 最少的地方 | 无（纯文档） | 人工复核 diff；`test_backend_flow.py` 行为不变 |
| P2 | 把结果 JSON 的字段名收敛为一个 `CorrectionResult` dataclass（或集中常量），写入/读取都引用它 | 现在 `C_world_odom` 等 key 在 worker、correction、building_runtime 三处按字符串重复 | 中：会碰到跨包读取方 | 先加"序列化-反序列化闭环"单测，再替换；`test_backend_flow.py` 已有 decode/结果断言可复用 |
| P2 | 把 `tracking.py` 对 `registration._refine/_observability` 的依赖改成公开名（去下划线或加 `__all__`） | 跨文件用私有函数，重命名会静默断裂 | 低（纯重命名） | `test_backend_registration.py` + `test_backend_auto_loops.py` |
| P3 | 在 `native_graph.py` 的缺失分支里把"缺库"与"求解失败"的 reason 分开命名 | 现在都落到 `native_graph_solver_unavailable` 语义附近，排障时分不清"没构建"和"求解拒绝" | 低 | 单测用不存在路径 patch `LIBRARY` |

没有 P0/P1：443 行是最大文件，包内无环，核心算法有 3 个回归文件覆盖（含真正的图优化断言，且本机 `.so` 存在）。

## 调用与依赖

Building 入口 [launch/building.py](../../launch/building.py)（`start("backend", …)`）与 RoboGauge benchmark [navigation/robogauge/benchmark.py](../../navigation/robogauge/benchmark.py) 都用 `python3 -B -m embodied_navigation.localization_mapping.backend.run` 启动，参数为 `--spool/--history/--external-map/--initial-world-odom/--map-world-offset`；队列、历史、地图写在运行目录，不依赖 ROS。**主路径 `localization_mapping/launch/slam.launch.py` 里没有这个进程**，不要以为默认 demo 跑起来就一定有 backend。

## 测试与限制

专用回归用例：[tests/regression/test_backend_flow.py](../../../../tests/regression/test_backend_flow.py)、[test_backend_registration.py](../../../../tests/regression/test_backend_registration.py)、[test_backend_auto_loops.py](../../../../tests/regression/test_backend_auto_loops.py)，均为纯 CPU、不启动 ROS/Gazebo；图优化相关断言要求 `install/backend/lib/liblivo_graph.so` 已构建（见 [native/README.md](native/README.md)）。

限制：Python 配准只做几何参考，不含学习模型、不做 GPU；没有真实 LiDAR/IMU 回放或实时延迟验收，`backend_status.json` 里也明写 `realtime_budget_verified=false`。`run.py` 的旧 docstring 路径问题已在代码里修复（现在写的是 `embodied_navigation.localization_mapping.backend.run`），此处按磁盘更正。
