# 开发路线图（ROADMAP）

> 盘点日期：2026-11-21 · 集成基线 `main` `2144e5f`（= `origin/main`） · 当前开发推进分支 `exp/startup-walk-imu-bias` `9e3408d`（本地领先 origin 6 个提交） · 活体文档，随阶段推进更新。

本文件是项目唯一主线的开发计划入口。阅读顺序：根 [README.md](../../README.md) → 本文件 → [TASKS.md](TASKS.md) → 架构决策记录（[ADR-0001](adr/0001-livo-yaw-pipeline-overlap.md)、[ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md)、[ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md)、[ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)）→ 各模块 `README.md`。协作与提交规则以 [AGENTS.md](../../AGENTS.md) 为准。

## 1. 唯一主线与七层架构边界

项目唯一主线是 **D 路径：Isaac Sim / DynaNav + Unitree Go2 + RoboGauge MoE-CTS**。A 路径（Gazebo、`start.py` → `launch/main.py`、`control/`、私有 Fast DDS）已在干净底座中整块退役删除，仅保留于 `main` 历史、`archive/pre-review-20261109` 等分支，不再恢复。

代码按七层边界组织，跨层只走显式接口，不越层直连：

| # | 层 | 主要目录 / 入口 | 职责边界 |
| --- | --- | --- | --- |
| 1 | 仿真 | `src/embodied_navigation/simulation/` | Isaac Sim / DynaNav 场景、Go2 RGB-D / RTX LiDAR / IMU 采集、`--truth-pose` 真值模式、仿真暂停与回执 |
| 2 | 感知与通信 | `src/embodied_navigation/perception/` + `common/` | 传感器消息适配、去畸变配对、健康门、位姿/速度契约；`common/` 提供几何、运行时 IO、进程日志与遥测基础设施 |
| 3 | 定位建图（前端 + 后端） | `localization_mapping/` + `third_party/fast_livo2/` | FAST-LIVO2 前端（post-VIO 位姿 / 实测点云）、SLAM 后端（关键帧、回环、位姿图、地图存取、先验重定位）、语义建图 |
| 4 | 规划 PCT + SCAN | `planning/` + `third_party/pct/` + `third_party/scan/` | PCT 实测分层支撑全局路线、原生 SCAN 局部轨迹、扫描交接与来源、Agent 航点候选 |
| 5 | 控制跟踪 | `navigation/` | `RouteTracker` / `ContinuousScanFollower` 跟踪、`yaw_action.py` 转向动作、RoboGauge 路线跟踪接口；输出到身体速度命令为止 |
| 6 | 机器人执行 | `robot/` | Go2 关节契约、RoboGauge MoE-CTS 策略运行时、策略原生观察与执行桥 |
| 7 | Agent 感知与控制介入层 | `simulation/dynanav/operator_map.py`、`localization_mapping/semantic_mapping/`、`planning/exploration/`、`scripts/explore_waypoint_client.py`、`.agents/skills/dynanav-explore/` | 目标下发、运行中干预、状态/地图查询、语义建图与稀疏航点决策；不写速度命令、不读仿真真值 |

默认端到端入口见 [根 README 的快速启动](../../README.md)；PCT/SCAN 时序与证据边界见 [PCT/SCAN 说明](../dynanav-pct-scan.md)。

## 2. 阶段里程碑

### Phase 1 — 干净底座重构（已完成）

- 分支：`refactor/06-clean-base`（基于 `refactor/05-agents-rules`，HEAD `f061a7e`）。
- 成果：累计删减 >8.2 万行；A 路径整块退役；P1/P2 废弃模块清理；动态行人支线完整归档至参考分支 `ref/dynanav-dynamic-pedestrian`；单测零失败。
- 验收：全量单测/回归失败名单相对起点不新增；逐行 diff 复核后由用户决定合并。

### Phase 2 — 第一优先真值闭环验证（已完成并合入干净底座）

- 分支：`exp/pct-scan-stairs-truth`（合入提交 `bb2eef2`）。
- 目标：跳过 SLAM 定位漂移干扰，用仿真真值位姿 + 200 Hz 真值去畸变 RTX 点云验证规划完整性。
- 通用 3D PCT+SCAN：
  - `third_party/scan/patches/reference_z_profile.patch` 提供参考 z 上包络；
  - PCT 全局规划接入起点脚印邻域实测支撑，不再依赖场景先验；
  - 零楼梯特化规则、零场景 tomogram、零目标吸附、零直线兜底。
- 结果：完成地下室至二楼大厅全链路，净爬升 **+3.25 m**，零撞墙 / 零撞扶手、零 SCAN 失败。
- 保留：分支保留作复现入口；不改变默认 FAST-LIVO2 前端链路。

### Phase 3 — 可交付干净基线收口（已完成，已合入并推送 `main`）

- 目标：把已验证能力收口成可交付、可审核的干净基线。
- 收口内容（均已合入 `main`）：
  1. **SLAM 公共接口初步规范化**（`5b880b2`）：IMU 偏置协方差进 `third_party/fast_livo2/patches/0011-imu-bias-covariance-params.patch`；后端 `anisotropic_information` 各向异性信息矩阵；`BackendConfig.from_dict/from_json` 严格配置加载与 `node.py --config` 显式入口。
  2. **默认 FAST-LIVO2 前端 + PCT/SCAN + Agent 探索烟雾测试**：`runs/20261120-main-smoke/livo-frontend-smoke-01` 跑通平地导航与 180° 折返爬楼梯（+0.53 m），`capture_exit=0`、零撞墙/扶手、零实质躯干碰撞、零 SCAN 失败。
  3. **LIVO `yaw` 延迟修复与 IMU patch 原生构建接线**：`fix/livo-yaw-and-imu-patch`（`5049ec7`）经 merge `9b6dc85` 合入；`runs/20261120-yaw-and-imu-fix/livo-yaw-04` 在默认 live FAST-LIVO2 下完成独立 `yaw +180°`（误差 `-0.036 rad`）与 `goto 1.5 m`。
  4. **文档与分支基线重建**：根 `README.md`、`docs/plan/`、ADR-0001/0002（`9816a2b`）与全部活跃模块 `README.md` 对齐；RViz 实时点云配置/录制 CLI（`ca754ab`、`2144e5f`）。
- 收口判据（全部满足）：
  - [x] `git diff --check` 通过，工作树只包含文档与已审核代码改动；
  - [x] 单测全绿：`main` 650 条、开发分支 `exp/startup-walk-imu-bias` 679 条（均 0 失败 0 跳过）；回归 60 条 0 失败（本机 3 条按环境 skip）；失败名单相对起点不新增；
  - [x] 默认入口烟雾测试有可复核的 run 目录与结论（含 `--truth-pose` 对照 `truth-stairs-09` 与默认 LIVO `livo-frontend-smoke-01`）；
  - [x] 根 README 与 `docs/plan/` 链接全部可达，模块 README 与 `git ls-files` 一致；
  - [x] 用户完成审核，`main` = `origin/main` = `2144e5f`。

### Phase 4 — SLAM 后端优化与双层数据闭环（进行中，当前重点）

**目标**：用带噪 IMU + 真实双层全房间数据，把 SLAM 后端从“能跑”推进到“同输入可量化、精度可复现”；评测口径以 [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) 为唯一打分尺。

1. **两份核心后端评测数据集已录制完成**（13 话题：RTX LiDAR + 标定 RGB + 带噪 IMU + 外参；详见第 5 节）：
   - `20261121T133734Z`：一楼窄角前后端双失效包（219 关键帧 / 159 回环），复现**无重力约束 6-DoF ICP 把 Roll/Pitch 拉歪 6°~8.3°** 与长停滞前端发散；
   - `20261121T142308Z`：双层 16 区域、46 分钟、70 m × 27 m 超长包（1,567 关键帧），复现老后端 **407+ 过密回环把二楼 Z 拽偏 -74 cm**。
2. **三条互不耦合的 Phase 4 后端对照分支已完成初版并通过 Codex 审计**（均从 `9816a2b` 切出，见第 3 节）：
   - `exp/slam-measured-submaps`（`5b300b0`）：多帧实测 LiDAR 子图对回环配准的影响；
   - `exp/slam-surface-gate`（`958ec01`）：平面/表面覆盖门控对 ICP 回环误配与漏配的影响；
   - `exp/slam-visual-retrieval`（`16883d1`）：图像检索补充候选、视角不变特征与几何验证。
3. **前端/规划实机闭环修复分支 `exp/startup-walk-imu-bias`（当前开发 HEAD `9e3408d`，基线 `main`，共 12 个提交）**：起步预热稳态门 + 物理级带噪 IMU（`ce3bac1`）、RViz RGB8 彩色重建图层（`6f79507`）、局部滑窗建图 / 近期扫描恢复 / `--backend-pose` 订阅（`95fc675`）、PCT 净空 0.95 m（`04ceede`）、不依赖 PCT/SCAN 的直通脱困 `maneuver`（`2f64d8d`）、ADR-0003/0004（`8ed74c2`、`9e3408d`）。
4. **下一阶段主线（按顺序推进，详细任务见 [TASKS.md](TASKS.md)）**：
   - [ ] 离线关键帧抽取缓存（约 200–500 MB）+ 三层轨迹评测器（在线/终图 ATE·AOE·RTE·ROE + 逐关键帧演化曲线 + 逐秒位姿分布 + TUM 导出）；
   - [ ] 后端修复 ①：6-DoF ICP / 位姿图重力 **Roll/Pitch 强先验**；
   - [ ] 后端修复 ②：**原地/近邻过密回环抑制**；
   - [ ] 后端修复 ③：Agent 思考停驻期 **ZUPT 停车锚定约束**；
   - [ ] 三条回环对照分支在同一份离线缓存上横向评测并汇总收益/失败/适用场景；
   - [ ] 评测结论明确后再推进动态行人支线与语义拓扑层（ADR-0002/0003）。

Phase 4 完成判据：同一份离线关键帧缓存上，后端只改一项即产生可复核的 ATE/AOE/RTE/ROE 对比；三项后端修复均有失败名单不新增的回归与实机（或录包回放）证据；用户审核后由用户合并推送。

## 3. 活跃分支现状与关键提交 Log 摘要

2026-11-21 本地 `git branch -avv` / `git log --oneline` 与 `git ls-remote --heads origin` 核对结果；提交号是分支头，复现时以运行记录中的源码快照为准。

| 分支 | HEAD | 远端同步 | 定位 |
| --- | --- | --- | --- |
| `main` | `2144e5f` | 与 `origin/main` 一致 | 集成、发布与独立验收入口（Phase 3 已收口） |
| `exp/startup-walk-imu-bias` | `9e3408d` | 领先 `origin/exp/startup-walk-imu-bias`（`6f79507`）6 个提交 | **当前开发推进分支**：前端/规划实机闭环修复 + ADR-0003/0004 |
| `exp/slam-measured-submaps` | `5b300b0` | 与远端一致 | Phase 4 后端对照：多帧实测子图回环配准 |
| `exp/slam-surface-gate` | `958ec01` | 与远端一致 | Phase 4 后端对照：平面/表面覆盖门控 |
| `exp/slam-visual-retrieval` | `16883d1` | 与远端一致 | Phase 4 后端对照：视觉检索候选 |
| `fix/livo-yaw-and-imu-patch` | `5049ec7` | 已并入 `main`（merge `9b6dc85`） | LIVO `yaw` 延迟修复 + IMU patch 原生构建接线（证据保留） |
| `exp/pct-scan-stairs-truth` | `bb2eef2` | 已并入 `refactor/06-clean-base` 历史 | Phase 2 真值闭环复现入口 |

### 3.1 `main` — `2144e5f`（= `origin/main`）

定位：集成与发布基线；Phase 3 全部成果已由用户合并、推送。近期关键提交：

| 提交 | 摘要 |
| --- | --- |
| `2144e5f` | `fix(viz): use dynanav_lio_map as Fixed Frame in dynanav_live_pointcloud.rviz` |
| `ca754ab` | `feat(viz): add live RViz2 point-cloud growth config and recorder CLI` |
| `9816a2b` | `docs(adr): add ADR-0001 yaw latency decision and ADR-0002 embodied-agent outlook` |
| `9b6dc85` | `Merge fix/livo-yaw-and-imu-patch: LIVO yaw latency fix and 0011 IMU patch native build wiring` |
| `5049ec7` | `fix(yaw,slam): eliminate LIVO yaw rotation_space_stale and wire 0011 IMU patch into native build` |

### 3.2 `exp/startup-walk-imu-bias` — `9e3408d`（当前开发分支；领先 `main` 12 个提交，领先 origin 6 个）

定位：在 `main` 之上承载 Phase 4 前置的**前端/规划实机闭环修复**与两份架构决策记录，供用户在真实双层场景验证后决定并入。关键提交（新→旧）：

| 提交 | 摘要 |
| --- | --- |
| `9e3408d` | `docs(adr): add ADR-0004 for incremental mesh/3DGS outlook and 3-layer SLAM backend evaluation` |
| `2f64d8d` | `feat(navigation): support direct PCT/SCAN-free maneuver recovery action and update explore skill anti-trap rules` |
| `8ed74c2` | `docs(adr): add ADR-0003 for streaming fork control, keyframe-anchored semantic scene graph, and ZUPT backend constraints` |
| `04ceede` | `fix(pct): raise default overhead clearance_m from 0.65m to 0.95m to block low under-stair wedges` |
| `62d5015` | `tune(pct-scan): raise backend voxel realignment threshold to 5cm / 0.5deg` |
| `95fc675` | `feat(pct-scan): add local sliding-window map, recent-scan recovery, and backend pose subscription`（含 `--backend-pose`） |
| `6f79507` | `feat(viz): add RGB8 LIVO2 colored point-cloud layer and backend trajectory toggle to RViz config`（origin 当前 ref） |
| `ce3bac1` | `feat(dynanav): add startup warmup walk, trend stability gate, and configurable IMU bias/noise` |

### 3.3 `exp/slam-measured-submaps` — `5b300b0`（远端一致；基线 `9816a2b`）

定位：Phase 4 对照分支，隔离多帧实测 LiDAR 子图对回环配准的影响；只带本方向最小改动与独立配置。

| 提交 | 摘要 |
| --- | --- |
| `5b300b0` | `fix(slam): address Codex audit findings on exp/slam-measured-submaps` |
| `0d8b6bc` | `feat(slam): isolate measured LiDAR submaps with query/history/anchor ablations` |
| `9816a2b` | `docs(adr): add ADR-0001 yaw latency decision and ADR-0002 embodied-agent outlook`（共同基线） |

### 3.4 `exp/slam-surface-gate` — `958ec01`（远端一致；基线 `9816a2b`）

定位：Phase 4 对照分支，对照平面/表面覆盖门控对 ICP 回环误配与漏配的影响。

| 提交 | 摘要 |
| --- | --- |
| `958ec01` | `fix(slam): address Codex audit findings on exp/slam-surface-gate` |
| `647e9f7` | `feat(slam): add clean surface-gate backend experiment branch` |
| `9816a2b` | `docs(adr): add ADR-0001 yaw latency decision and ADR-0002 embodied-agent outlook`（共同基线） |

### 3.5 `exp/slam-visual-retrieval` — `16883d1`（远端一致；基线 `9816a2b`）

定位：Phase 4 对照分支，图像检索补充候选、视角不变特征与几何验证。

| 提交 | 摘要 |
| --- | --- |
| `16883d1` | `feat(slam): upgrade visual retrieval to viewpoint-invariant features (ORB/AKAZE/BRISK/SIFT)` |
| `44e23d0` | `fix(slam): address Codex audit findings on exp/slam-visual-retrieval` |
| `da08937` | `feat(slam): add clean visual-retrieval backend experiment branch` |
| `9816a2b` | `docs(adr): add ADR-0001 yaw latency decision and ADR-0002 embodied-agent outlook`（共同基线） |

### 3.6 `fix/livo-yaw-and-imu-patch` — `5049ec7`（已并入 `main`）

定位：LIVO 独立 `yaw` 延迟修复与 0011 IMU patch 原生构建接线；作为 ADR-0001 的证据基线保留，不再继续叠加。

| 提交 | 摘要 |
| --- | --- |
| `5049ec7` | `fix(yaw,slam): eliminate LIVO yaw rotation_space_stale and wire 0011 IMU patch into native build` |
| `88156de` | `Merge refactor/06-clean-base: clean D-path baseline, universal 3D PCT+SCAN, SLAM config standardization, and rebuilt docs` |

### 3.7 `exp/pct-scan-stairs-truth` — `bb2eef2`（Phase 2 复现入口）

定位：真值位姿 + 200 Hz 真值去畸变 RTX 驱动 PCT/SCAN 多层闭环；成果已并入 `refactor/06-clean-base` 历史，分支保留作复现入口。

| 提交 | 摘要 |
| --- | --- |
| `bb2eef2` | `feat(pct-scan): replace stair-only hacks with universal 3D SCAN & PCT navigation` |
| `214b8a3` | `feat(pct-scan): add 200 Hz simulation-truth deskew and --truth-pose mode` |
| `2aab845` | `feat(pct-scan): port stair geometry, local timing patch, yaw action, and PCT grid budget onto clean base` |
| `5bb6a64` | `docs(agents): record clean-base milestone, DSH workspace workflow, and Agent layer` |

### 3.8 历史 / 归档分支（只读查证，不继续叠加）

| 分支 | 用途 |
| --- | --- |
| `refactor/00-plan-docs` … `refactor/05-agents-rules` | 分包重构历史：计划、几何去重、死代码、旧路径移除、SLAM 后端、协作规则 |
| `ref/dynanav-dynamic-pedestrian` | 动态行人 / 学习分割支线完整保全，Phase 4 评测结论后按需提取 |
| `exp/loop-measured-submaps`、`exp/loop-surface-gate`、`exp/loop-visual-retrieval` | 上一轮回环共同比较口径下的三条对照（远端已核实），结论保留 |
| `refactor/05-loop-evidence-base` | 回环比较实现、配置与证据保全 |
| 本地 `exp/two-floor-demo`、`backup/exp-two-floor-demo-wip-20261120` | 拆分前两层联调代码、楼梯补丁、SLAM 评测与采集改动 |
| `archive/pre-review-20261109`、`wip/navigation-local-snapshot-20261115`、`backup/slam-controlled-experiment-20261116` | 审查前源码、在途快照、受控 SLAM 追踪与反向航点 |
| `improve/module-docs-readability`、本地 `feat/navigation-review-web` | 模块文档与阅读入口、导航审阅页面 |

## 4. 架构决策记录（ADR）索引

| ADR | 标题 | 状态 | 关键结论 / 关联 |
| --- | --- | --- | --- |
| [ADR-0001](adr/0001-livo-yaw-pipeline-overlap.md) | LIVO 独立 `yaw` 清阻塞与 ≤ 0.2 s 点云融合重叠（暂不重写控制链路） | 已采纳（2026-11-20） | 独立 `yaw` 不放松 `max_sim_age_s=0.25 s`；证据在 `fix/livo-yaw-and-imu-patch`（`5049ec7`）与 `runs/20261120-yaw-and-imu-fix/` |
| [ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md) | 具身 Agent 异步行动、语义/动态地图管理与热路径演进备忘 | 提案 / 远期（2026-11-20） | 异步“边想边走”、语义/动态地图、后端热路径下沉 C++ 的触发条件 |
| [ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md) | 单主干流式 Fork 异步控制与关键帧锚定语义图片图数据库 | 提案 / 架构演进（2026-11-21） | System 0/1/2 职责红线；语义节点绑定 `keyframe_id`；**后端先补齐停驻期 ZUPT、重力 Roll/Pitch 先验、过密回环抑制** |
| [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) | 面向 FAST-LIVO2 的增量 Mesh / 3DGS 重建演进与三层 SLAM 后端评测口径 | 提案 / 评测规范（2026-11-21） | 在线/终图 ATE·AOE·RTE·ROE + 逐关键帧演化曲线 + 逐秒位姿分布 + TUM 导出；先做离线关键帧缓存（200–500 MB） |

## 5. 数据卷与产物约定

工程数据根统一为 `$PROJECT_DATA_ROOT/embodied-navigation/`（由 checkout 的 `.env.local` 提供，Linux 上解析到数据盘），仓库只放源码、配置、实验定义与结论。

| 路径 | 内容 |
| --- | --- |
| `runs/<批次>/<run-id>/` | 新原生运行与回放的完整产物：冻结配置、`runtime_context.json`、日志、回执、`manifest.json` 等 |
| `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261121-noisy-imu-rgb-twofloor-explore/20261121T133734Z/` | 一楼窄角前后端双失效包：≈9.3 GB（3 个 MCAP 共 8.8 GiB）；219 关键帧 / 159 回环；仿真 414.5 s；复盘复现无重力约束 6-DoF ICP 把 Roll/Pitch 拉歪 6°~8.3°、z 估计跳变与长停滞前端发散 |
| `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261121-noisy-imu-rgb-twofloor-explore/20261121T142308Z/` | 双层 16 区域 46 分钟 70 m × 27 m 超长包：≈80 GB（20 个 MCAP 共 76.4 GiB）；1,567 关键帧；累计 704 条回环；13 话题 1,428,125 条消息；复盘复现 407+ 过密回环把二楼 Z 拽偏 -74 cm；门角脱困与下楼未完成 |
| `experiments/<run>/` | DynaNav / Isaac 历史实验产物（capture、observer、点云与逐帧回执） |
| `build_cache/` | 原生 SCAN / LIO 构建产物与安装前缀 |
| `runtime_assets/{assets,models,runs}/` | 仓库根 `assets/`、`models/`、`runs/` 软链指向的部署资产、权重与历史运行 |
| `reports/`、`deliverables/` | 整理后的报告与交付物 |

- 两份 Phase 4 数据包均含 `capture/truth_scoring_stream.jsonl` 与 `capture/imu_bias_truth.jsonl`，是三层评测器的真值时间轴与带噪 IMU 输入；最新记录见各自 `codex_pilot_report.md` 与 `manifest.json`。
- 规则：运行产物不进仓库；单个文件 >200 KB 默认不提交；开跑前用 `findmnt -T` 与 `df` 核对挂载与容量；详细放置与清理规则见数据根 `AGENTS.md` 与仓库 [AGENTS.md 第 6 节](../../AGENTS.md)。

## 6. 推进原则

1. **一条主线**：只做 D 路径；不恢复 A 路径、B/C 基准或已移出的行人支线。
2. **分支按主题切**：一个分支只验证一个假设，不在地基分支上叠加混合改动；验证通过的行为就是唯一实现，不用环境变量做开关。
3. **证据优先**：CPU / 合成测试 / 进程退出码都不等于物理到达；结论必须绑定 run 目录与时间轴。
4. **先评测后扩张**：Phase 4 先落地离线关键帧缓存 + 三层评测器，三项后端修复与三条回环对照必须有同输入、同时间轴、同对齐的可比证据；评测结论明确前不合并到 `main`。
5. **由用户推送**：agent 只提交到本地分支，合并与推送由用户完成。
