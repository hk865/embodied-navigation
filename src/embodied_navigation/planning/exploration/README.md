# planning/exploration（探索候选、多轮通信契约与 Habitat 闭环认知评测）

> 盘点日期：2026-11-26 · 分支 `feat/static-memory-streaming` · HEAD `4f81110`

本目录承载两类职责：(1) 为 D 路径（DynaNav / Isaac Sim）提供只读探索候选与离线/哨兵邮箱协同；(2) 提供开放词表具身 LLM 客户端与 Habitat-Sim HM3D 闭环认知评测适配器（`habitat_agent_adapter.py` 将世界自更新守护 `HabitatWorldDaemon`、工具包与终止门接入独立的 `agent_runtime/`）。注意：`agent_runtime/` 仍暂时从本目录 `llm_agent_client.py` 复用 `LlmClientConfig` 与 `_encode_image_part`，这是已知待收口的依赖债。

## 模块定位与架构层级

- 架构层级：物理位置在第 4 层 —— 规划层（PCT/全局路线）；职责上服务第 7 层 —— Agent 感知与控制介入层，只提供 advisory 数据，不拥有执行权。
- 输入：
  - 当前 run 的 `capture/interactive/planner_measured_map.npz`：`provenance_json`、`points_xyz_m`（实测端点）、`trajectory_body_xyz_m`（实测 body 轨迹）、可选 `lio_pose_xyz_xyzw`（配对 LIO 位姿，用于建模 baseline 可见性）；
  - 同目录 `observation.json`（post-VIO pose、`waypoints.active_seq`、`active_goal_xyz_m` 等）与可选 `planner_global_route.json`（PCT 全局路线 artifact）；
  - 调用方显式候选列表 `{id, goal_xyz_m, role, reason}`，或可选 frontier 快照 JSON；
  - [`os1_calibration.json`](os1_calibration.json)（OS1 32 线标定）与 [`route_information.py`](route_information.py) 的固定假设。
- 输出：一份 `schema='dynanav_waypoint_candidates/v1'`、`advisory_only=true` 的 JSON（stdout 或 `--output`，且 `--output` 必须写在 action mailbox 之外），含候选列表、按 `utility_voxels_per_m` 排序的 id、准备模式、frontier 报告、诊断与 limitations。
- 上游 / 下游：主入口是 [`prepare_candidates()`](waypoint_candidates.py)（脚本另用 `run_binding()` 做 run 路径规范化），由 [`scripts/prepare_dynanav_waypoint_candidates.py`](../../../../scripts/prepare_dynanav_waypoint_candidates.py) 调用，也可经 `python3 scripts/dynanav_exploration.py candidates ...` 转发；候选由 DSH/操作者阅读后，仍通过 `scripts/explore_waypoint_client.py goto` 显式下发。下游不写 mailbox 请求，也不调用原生规划器之外的执行接口。
- 边界：这是**建模的潜在新观测体积**，不是实测自由空间、不是信息增益证书、不是安全/支撑/覆盖/视觉定位质量证明。`ground_truth_used=True` 的 snapshot 或 frontier 会被直接拒绝；不重写 source 时间（`source_times_rewritten=false`）。

## 当前文件夹结构

```text
src/embodied_navigation/planning/exploration/
├── README.md                 # 本文件
├── __init__.py               # 1 行包 docstring（无重导出）
├── os1_calibration.json      # OS1 32 线传感器标定快照
├── route_information.py      # 246 行：DynaNav 实测端点快照、弧长采样与建模可见性打分
├── waypoint_candidates.py    # 256 行：DynaNav 只读探索候选准备入口（读 run mailbox → 规划 → 打分 → JSON）
├── waypoint_mailbox.py       # 583 行：文件 mailbox 协议读写与心跳/序号管理
├── sentinel_coordinator.py   # 371 行：哨兵异步安全门协调
├── visual_map_overlay.py     # 546 行：操作员占用图与语义叠加渲染
├── llm_agent_client.py       # 1,713 行：LLM/VLM HTTP 配置与图像编码基元（LlmClientConfig 等）
├── streaming_task_agent.py   # 1,977 行：早期流式单轮决策 Agent（保留作兼容与离线任务套件）
├── static_task_suite.py      # 1,537 行：静态离线导航与场景图问答基准套件
├── habitat_memory_runner.py  # 1,838 行：无状态单轮 Habitat 评测基线 + DepthOccupancyGrid（含 render_global_floorplan 2.5D 平面图）
└── habitat_agent_adapter.py  # 1,887 行：HabitatWorldDaemon（自更新感知/Zone/双触发 KF）+ 通用工具集 + ObjectNav/Report FinishVerifier + HabitatAgentRunner
```

13 个跟踪文件（11 个 `.py` + 1 个 `.json` + 本 README）；逐文件说明优先覆盖下方核心只读候选链，多轮 Habitat 运行时细节见 [`agent_runtime/README.md`](../../agent_runtime/README.md) 与 `scripts/run_habitat_agent_eval.py`。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `README.md` | 本文件：定位、输入输出、打分语义与测试入口。 | — |
| `__init__.py` | 只有包 docstring `Advisory measured-map exploration preparation; no execution authority.`；无重导出。 | — |
| `os1_calibration.json` | 传感器标定快照：`model=OS1_REV6_32ch10hz1024res`、32 个 `elevation_deg`、32 个 `azimuth_offset_deg`、`azimuth_columns=1024`、`near_range_m=0.3`、`far_range_m=120.0`、`lidar_translation_in_base_m=[0,0,0.25]`、`lidar_rotation_in_base="identity"`、冻结资产 `asset_sha256`。 | 被 [`route_information.CALIBRATION`](route_information.py) 在 import 时读取；`asset_sha256` 进入打分输出的 `provenance.sensor_asset_sha256`。 |
| `route_information.py` | 实测端点快照与“虚拟扫描”打分：只有实测端点遮挡射线，未知空间视为透明；baseline 由历史实测位姿（及可选剩余主动路线）建模，用来做差集。 | `MeasuredSnapshot`（拒绝 truth-derived）、`load_measured_snapshot(path, expected_generation=None)`（校验 only same-stage LIO / 配对 / frame，保留文件 sha256，并声明 reader 不验证物理配对）、`voxel_keys(points, voxel_m)`、`sample_route(route, spacing_m=1., max_samples=32)`（全程弧长采样、含两端点、预算不足时如实标记 coarsen）、`RouteInformationScorer`（`score_route()` 返回 `potential_novel_voxel_count/volume_m3`、`utility_voxels_per_m`、`provenance`、`assumptions`）、`newly_measured_voxel_delta(before, after, voxel_m=.5)`。 |
| `waypoint_candidates.py` | 主入口：把一次 run 的不可变实测快照、观测、可选 frontier 与显式候选合成 advisory JSON；逐候选调用 `MeasuredPctPlanner.plan()` 再用 `RouteInformationScorer` 打分。 | `run_binding(run_dir)`、`frontier_candidates(snapshot, z=None)`、`remaining_active_route(observation, artifact, binding, current)`、`prepare_candidates(run_dir, candidates, *, prepare_next=False, scorer_options=None, planner_options=None, frontier_snapshot=None, frontier_z=None, offline=False, snapshot_path=None, observation_path=None, frontier_path=None, planner_factory=MeasuredPctPlanner, scorer_factory=RouteInformationScorer)`。 |

## 评分语义与安全边界（读码核实）

- 打分量是“**乐观的建模未观测体积**”：`assumptions` 明确写 `observed_free_available=False`、`measured_information_gain=False`、`unknown_is_transparent=True`，遮挡只来自实测端点 voxel（稀疏墙可能漏），baseline 是建模的虚拟扫描而非实测自由空间。
- `RouteInformationScorer` 在构造时校验：`near_range_m < max_range_m <= far_range_m`、`azimuth_samples` 为 1..1024 的整数、`range/voxel_m <= 512` 的 CPU 预算、`max_route_samples` 为 2..256；每条射线从 LiDAR 起点沿 32 线标定方向做 3D voxel DDA，命中实测 occupied voxel 即停止。
- `prepare_candidates()` 强制 `pose.source == 'livo_post_vio'`、snapshot/observation/frontier 均在本 run source root 内、snapshot 的 `run_id`/`map_frame`/`child_frame` 与 observation 一致；`prepare_next=True` 时还要把 `planner_global_route.json` 绑定到当前 `active_seq`/`active_goal`/frame/generation，并把“当前位姿→最近投影→剩余路线”作为额外建模 baseline（最多 8 条），避免长历史饿死已承诺路线的可见性差集。
- 候选 id 必须是唯一非空字符串，`role` 只能是 `long_goal`/`auxiliary`；单个候选规划抛异常时记为 `route_found=false`、`reason='candidate_evaluation_error:<error>'`，不中断整批。frontier 聚类只作为补充候选，且必须由调用方显式给 `--frontier-z` 才能产生 `goal_xyz_m`（图表高度不被当作物理楼层或航点 Z）。
- 计算结束后重新读取 `observation.json` 的 `active_seq`，在输出里报告 `binding_unchanged_at_completion`：规划期间目标变化只会被报告，advisory 永不安装缓存路线。

## 验证与测试入口 / 配置注意

- [`tests/unit/test_dynanav_route_information.py`](../../../../tests/unit/test_dynanav_route_information.py)：遮挡（墙后不可见）、距离与垂直 FOV、baseline 与重叠 voxel 去重、全程采样覆盖、真值推导元数据不进入占据、loader 保留 hash/frame、显式 baseline yaw 与 mount 变换、剩余主动路线不被长历史饿死。
- [`tests/unit/test_dynanav_waypoint_candidates.py`](../../../../tests/unit/test_dynanav_waypoint_candidates.py)：只读复用与排序/失败记录、frontier 必须显式 Z、`prepare_next` 扣除建模剩余腿、active seq/生成绑定、重复 id 拒绝、offline 跨 run 拒绝、真值拒绝、CLI 拒绝把输出写进 action mailbox（fixture 用 `FakePlanner`/`FakeScorer`）。
- 命令（仓库根执行）：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- CLI 示例（纯 CPU，不启动仿真）：
  ```sh
  python3 scripts/dynanav_exploration.py candidates \
    --run-dir "$PROJECT_DATA_ROOT/embodied-navigation/runs/<批次>/<run-id>" \
    --candidates candidates.json \
    --lidar-offset 0 0 0.25 \
    --output advisory-candidates.json
  ```
  `--lidar-offset`（LiDAR 相对 route child-origin 的 XYZ）必填；`--candidates` 与 `--frontier-json` 至少给一个；`--snapshot`/`--observation-json` 覆盖参数只在显式 `--offline` 下允许（frontier JSON 不受此限制）；默认 `--output` 为 stdout。
- 配置注意：本包不读 `configs/`；阈值来自 CLI/调用方参数。原生 PCT 侧参数 `--body-support-offset`、`--pct-start-blind-radius`、`--pct-binary` 直接传给 [`MeasuredPctPlanner`](../pct/measured_tomogram.py)。
- 限制：传感器为 `OS1_REV6_32ch10hz1024res` 标定模型，不是所有 LiDAR 通用；传感器位姿是重力对齐 + 路线切线 yaw 的建模假设，未模拟开火运动；不提供视觉定位质量分（`visual_localization.status='unavailable'`）。系统层面的使用边界见 [`docs/dynanav-exploration.md`](../../../../docs/dynanav-exploration.md)。
