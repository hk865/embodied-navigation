# 开发路线图（ROADMAP）

> 盘点日期：2026-11-28 · 集成基线 `main` `2144e5f`（= `origin/main`） · 当前主力记忆/Agent 开发分支 `feat/static-memory-streaming`（文档基线 `a7fd71a` / 代码基线 `e159c8a`，**948 单测 / 60 回归全绿**，`agent-v4` 5 场景 HM3D Dev `81.25%` SR） · D 路径真物理闭环分支 `exp/live-v4-ba-sliding-window`（`aada1f1`，V4 在线定向 Planar BA + 29 MB Object Surface Nets） · Phase 4 稳定化底座 `exp/slam-backend-stabilization`（`6e1a489`） · 活体文档，随阶段推进更新。

本文件是项目唯一主线的开发计划入口。阅读顺序：根 [README.md](../../README.md) → 本文件 → [BASELINES.md](BASELINES.md)（七层架构冻结基线与子模块优化边界）→ [2026-11 学术创新查重与论文方向总评](research/2026-11-novelty-audit-and-directions.md)（2025–2026 顶会排雷与五大候选方向决策矩阵）→ [VLN 评测真相与「感知 + 记忆 + 失败驱动控制」演进总纲](research/2026-11-vln-perception-memory-strategy-synthesis.md)（Benchmark Tricks 核查、Agent 弱点与下一阶段三轴蓝图）→ [3D/4D 表征、混合记忆检索与算力策略 Trade-Off](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)（Mesh 平面压缩与骨架地基、树/图过程检索、3 天遮挡实验、视觉 vs LiDAR 裁决与 Colab 1,000 CU 规划）→ [TASKS.md](TASKS.md) → [TEST_PLANS.md](TEST_PLANS.md)（各活跃分支测试与验收计划） → [Phase 4 后端评测报告](../../experiments/slam-backend-phase4/REPORT.md) & [各后端分支机制与公式手册](../design/slam-backend-branches-guide.md) → 架构决策记录（[ADR-0001](adr/0001-livo-yaw-pipeline-overlap.md)、[ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md)、[ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md)、[ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)、[ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)）、[ADR 深度调研蓝图](research/adr-0002-0003-0004-survey.md) 与 [四大方向入门精讲与总施工图](research/four-directions-roadmap-and-blueprint.md) → [Agent 教程](../agent-tutorial/README.md) & [SLAM 教程](../slam-tutorial/README.md) → 各模块 `README.md`。协作与提交规则以 [AGENTS.md](../../AGENTS.md) 为准。

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
| 7 | Agent 编排与介入层 | **`src/embodied_navigation/agent/`**（`runtime/`、`tools/`、`prompts/`、`adapters/`，详见 [agent/README.md](../../src/embodied_navigation/agent/README.md)）+ D 路径桥接（`simulation/dynanav/operator_map.py`、`planning/exploration/`、`scripts/explore_waypoint_client.py`、`scripts/run_habitat_agent_eval.py`、`.agents/skills/dynanav-explore/`）；单向调用第 3 层的 `localization_mapping/semantic_mapping/` | **单向向下依赖**（底层 8 子包严禁反向导入，由 AST 单测守卫）；承载多轮 Agent 运行时、前缀稳定 KV 上下文与压缩、三态熔断 LLM 路由、工具/`STOP` 校验、多会话并发与仿真适配器；不写底盘速度命令、不读仿真真值 |

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

### Phase 4 — SLAM 后端稳定化与三路正交 v2 离线闭环（离线评测收口，待实机接线验收）

**目标**：用带噪 IMU + 真实双层全房间数据，把 SLAM 后端从“能跑”推进到“同输入可量化、精度可复现”；评测口径以 [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) 为唯一打分尺，统一机身原点（`imu_in_base_xyz`）与 5 s 初窗 SE(3) 对齐。

1. **三份统一离线数据集（`D1/D2/D3`）已完成 30/30 组干净（`dirty=False`）对照评测**（产物位于 `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261122-slam-backend-phase4-clean/`，完整结论见 [REPORT.md](../../experiments/slam-backend-phase4/REPORT.md) 与 [slam-backend-branches-guide.md](../design/slam-backend-branches-guide.md)）：
   - `D1`（`20261121T133734Z`，219 KFs）：前端 `0.1923 m` → `B0_default` 倾角崩塌至 `8.48°`（`0.2206 m`）→ 新底座 `stabilized` 与三路 v2 均 **`0.0997 m`（0 误回环、`zupt_pivot=138`）**；
   - `D2`（`20261121T142308Z`，1,567 KFs）：前端 `0.0772 m` → `B0_default` 负优化至 `0.1858 m` → 新底座 `stabilized` 达 **`0.0554 m`（Z `0.0380 m`，零 L2 负跳变）**，`v2_sg_soft` 达 **`0.0608 m`（倾角 `0.28°`）**，`v2_visual_akaze_only` 因前视单目 FOV 难召回反向重访仅 `11` 边（`0.1837 m`）；
   - `D3`（`same-876`，876 KFs，统一口径 HBA `0.3172 m`）：前端 `0.2085 m` → `stabilized` `0.1882 m` → `v2_ms` **`0.1268 m`**（`nospacing` **`0.1126 m`**）→ `v2_visual_akaze_only` **`0.0825 m`（Z `0.0561 m`、Online `0.1022 m`、仅 `84.1 s`）**。
2. **GPT-6.2-Astra（非 fast，高推理强度）独立全量审计与修复已完成**：修复全部 4 项 P0（IMU/机身评分原点、位姿图精确 $\mathcal{J}_r^{-1}(e)$、Layer 2 同前缀回环收益、视觉 v2 在线 spool 加载与先验存图）与多项 P1/P2（ICP 质心平移不变性、MAP 精确重力权重、表面门 ZUPT 体素解耦、子图 `candidate` 锚点、视觉 3D 回波去重、全预设 `graph_gravity_sigma_rad=null` 同步、JSON 禁 `NaN`）。
3. **三条正交 v2 分支已全部基于稳定化新底座重切并通过 99/99 SLAM 单测**（旧 v1 分支保留作历史参照）：
   - `exp/slam-v2-measured-submaps`（`59c60a4`）：实测多帧局部子图 + `candidate` 锚点；
   - `exp/slam-v2-surface-gate`（`0e98406`）：平面/表面覆盖门 + 解耦 ZUPT 体素；
   - `exp/slam-v2-visual-retrieval`（`2aeea82`）：视觉检索 + 实测 LiDAR 3D 去重 RANSAC + 在线 ROS 加载与存图。
4. **前端/规划实机闭环修复分支 `exp/startup-walk-imu-bias`（`76138ba`）**：起步预热稳态门 + 带噪 IMU、RViz RGB8 图层、局部滑窗与 `--backend-pose`、PCT 净空 `0.95 m`、直通脱困 `maneuver`、ADR-0003/0004。
5. **Phase 4 推进与下一步清单（详见 [TASKS.md](TASKS.md) 与 [ADR 调研蓝图](research/adr-0002-0003-0004-survey.md)）**：
   - [x] ADR-0004 三层离线轨迹评测器（含机身原点转换、同前缀 Layer 2、入图起点 Layer 3 与 TUM 导出）；
   - [x] 后端稳定化 `B1`（回环节流）+ `B2`（ICP 精确 MAP 重力先验与倾角门）+ `B3`（纯传感器几何 ZUPT）；
   - [x] 三条正交 v2 分支在 `D1/D2/D3` 统一机身原点上的 30/30 干净对照与机制/公式手册；
   - [ ] 将推荐预设显式冻结接入 `scripts/run_dynanav_native_exploration.sh` 并完成 Isaac Sim / ROS 2 实时链路验收（含回环时延与视觉 spool 在线入库）；
   - [ ] 精度向 `2–4 cm` 进阶：航向门控混合准入（同向视觉验真 + 反向 360° 几何/表面门）、精 ICP 细体素 / 平面因子（BALM2/X-ICP 思路）与前端重力初始化优化；
   - [ ] 数据盘清理审批与 ADR-0002/0003/0004 新分支立项。

### Phase 5 — 「3 条主线 + 4D 叠加」4 阶段 8 分支演进（2026-11-28 按 ADR-0005 定稿）

**总体思路**：在 `feat/static-memory-streaming`（代码基线 `e159c8a` / 文档 `a7fd71a`，`948` 单测、`81.25%` 5 场景 Dev SR）与 `exp/live-v4-ba-sliding-window@aada1f1`（两层 `2.02 cm` 闭环 + `29 MB` Surface Nets）双底座之上，**先把静态整套“存、建、查、用”做扎实，再把动态感知与 4D 维度叠加上去**（详见 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md) 与 [3D/4D 表征 Trade-Off](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)）：

1. **主线 1 · Agent 导航（360° 喂入 + 官方评测 + 瘦身事件流式 + 双仿真，L7）**：
   - **已建成底座（`e159c8a`）**：`src/embodied_navigation/agent/` 独立运行时（单向依赖守卫）、`PrefixStableContextManager`（`96.40%` KV 命中 + `155K` 里程碑压缩）、三态熔断 LLM 路由、`ObjectNavFinishVerifier` 与 `SharedHabitatSceneHost` 并发；
   - **演进内容**：① 对接官方 `HM3D-ObjectNav / OVON / GOAT` Scorer，优化「全局地图 + 局部地图 + 360° 探索视角 (`separate4 / mosaic2x2`) + `PixelGoal` (`goto_pixel / query_pixel_3d`)」信息喂入；② **流式监控彻底瘦身**：取消独立的“几何动态危险小模型”（由主线 3 原生吸收）与重复语义建图（由主线 2 异步回告），主线 1 改为**事件驱动任务仲裁器**（只在临近岔口或语义歧义时按需轻量 Fork，不再 2–3 Hz 空转）；③ 提取 `8aebc95` 抢占接口并修复 2 处现存 Bug（[llm_agent_client.py](../../src/embodied_navigation/planning/exploration/llm_agent_client.py#L1193-L1200) 判旧逻辑、[native_waypoints.py](../../src/embodied_navigation/simulation/dynanav/native_waypoints.py#L287-L298) `cancel` vs `stop` 语义），新增 `agent/adapters/dynanav_adapter.py` 接回 Isaac Sim；④ 接入 `Attempt & Failure Ledger` 失败闭环与 8 项 Agent 认知诊断指标。
2. **主线 2 · 记忆（存 / 建 / 查 / 改）→ 3D 重建（L4 / L7）**：
   - **单一不可变底账（存+建）**：全栈共用不可变 `observation_id` 关键帧底片（`RGB + 16-bit 压缩深度 PNG`，单栋 `+1.2 MB`）+ **SE(3) 不变里程计过程轨迹链 ($\Delta x, \Delta y, \Delta\theta$)** + **历史推理先验/语义直觉**，补齐 Habitat 与 D 路径 `MapStore` 统一；采用 **Mesh（压平墙地大面）+ 点云（保细部残差）+ 关键帧** 混合构建与「骨架 + Mesh」物理预测地基，并在分支 `3` 内用 2–3 天低成本检验“Mesh 遮挡重排”子项值不值得开；
   - **树检索 + 四类边时空过程图检索（查+改）**：沿 `建筑→楼层→房间→探索过程段→关键帧` 多归属层次树（永久保留全局回退）+ **里程计过程 / 门洞邻接 / 跨视角外观 / 推理先验** 四类边时空图检索，外观特征按 `observation_id` 建库、**严禁混入绝对世界坐标**（回环后外观库免重算），首版采用精确矩阵乘 $O(Nd)$ + 子图体素哈希 + 实例时序表；结合 `Colab 1,000 CU` 训冻结 `SigLIP` 同屋实例重排头；
   - **3D 重建（离线旁路）**：基于统一定型底账离线派生转连通 Mesh、`Hunyuan3D` 物体补全生成、Blender/UE5/Isaac 游戏材质资产库与 `2DGS` 对照，零在线帧率干扰。
3. **主线 3 · 感知（多模态互补动态 SLAM 与室内外泛化收尾，L2 / L3 / L6）**：
   - **多模态互补原则（废除机械传感器隔离）**：`RGB`、32 线 `RTX LiDAR`、`IMU / 关节` 共同参与，按场景特征强弱互补增强，不设死板隔离墙；
   - **2026-11-28 数学严审与动态前端拍板**：放弃纯单目时序模拟双目硬算深度（512×512 / $0.6\text{ m/s}$ 下 25 Hz 相邻帧基线仅 `2.4 cm`、2 m/5 m 误差达 `0.28 / 1.78 m`，正前方 FOE 零视差且跨帧行人算错数倍深度并撕裂）；主线拍板为 **`FAST-LIVO2` 静态底座 + 10 Hz LiDAR 静中剥动（按逐点时戳防时间锚错配）+ KF/IMM 30 Hz 惯性外推 + 20–30 Hz Object-Centric 切向光度可拒绝修正**（前端零深度学习小模型；迎面径向躯干仅 `0.76 / 0.19 px`、条件数 `92 / 369`，默认优先开放横向 2D 视觉更新，径向靠 LiDAR + 速度先验兜底；先做相机视点深度遮挡剔除再内缩防 `0.311 m` 外参背景沾色；拟合锁躯干、占据包四肢）。如需更高密前向静态体素，优先走物理双目 / `/dynanav/depth` 或预训练单目深度 + 10 Hz LiDAR 仿射对齐（只标占据、禁不清自由空间）；
   - **工程红线与泛化收尾**：严禁把 32 线束间空白当自由空间、严禁把 `MoE-CTS` `cmd_vel` 当里程计；完成室外坡道与余下场景泛化收尾（按需接入 `ffd7882`）；保留 PCT + SCAN + 失败驱动控制与 8 项 Agent 认知诊断指标。
4. **4D 叠加（动态 SLAM 站稳后，把“物体分割 + 运动跟踪”叠到 1 + 2 + 3 上）**：
   - 用不可变 `observation_id` 证据链串联主线 3 的短时 `track_id` 与主线 2 的长期 `instance_id`；做**物体分割 + 跨帧运动跟踪**（遮挡与丢失帧允许靠学习模型与 Track 补全信息），自然延伸到刚体 $T(t)$、门/抽屉铰接结构与按需骨架（行人阶段一避障优先存时效占据 + 速度 + 外观，出现交互姿态需求时再升格分段骨架）；同一 `pose_revision` 下严格区分 `not_visible / observed_empty_at_old_pose / identity_swapped`。

5. **4 阶段 × 8 分支施工总表与四条轻量跨主线契约**（起点简称 `H = feat/static-memory-streaming`、`D = exp/live-v4-ba-sliding-window@aada1f1`）：
   - **现状起点**：`feat/static-memory-streaming`（代码 `e159c8a` / 文档 `a7fd71a`，`948` 单测 / `60` 回归全绿）已在 5 场景 HM3D-val Dev（16 有效子任务，1.8 m，单目 90° RGB-D，GT 位姿）跑出 `SR 81.25% (13/16)`（95% Wilson CI `57%–93%`）、`OSR 87.50% (14/16)`、跨子任务 `81.82% (9/11)`、`0` 超时；D 路径 `exp/live-v4-ba-sliding-window`（`aada1f1`）已完成 1h 51m / 343.72 m 两层闭环（终图 `2.02 cm` / 在线 `2.68 cm`）+ 29 MB Object Surface Nets。

| 阶段 / 顺序 | 分支 | 基线 / 提取素材 | 核心任务与最小验收 | 归属 |
| :--- | :--- | :--- | :--- | :--- |
| **阶段 1（静态底座：先存与用）** | `3. feat/memory-hybrid-store-build` | `H` + `46e0b48` | 统一 `observation_id` 底账（16-bit 深度 PNG + 里程计过程链 + 推理先验）+ Mesh/点云混合构建 + 2–3 天低成本验证 Mesh 遮挡重排；回放幂等、零未来泄漏 | 主线 2（存+建） |
| **阶段 1（静态底座：先存与用）** | `1. feat/agent-habitat-benchmarks` | `H` | 对齐官方 `HM3D / OVON / GOAT` Scorer；优化 360° 观测与全局/局部地图喂入；按需校准轻量状态/`STOP` 头（按 `1.0 / 1.8 m` 与 Dev/盲测拆报） | 主线 1（评测与喂入） |
| **阶段 2（静态全链路）** | `4. feat/memory-tree-graph-retrieval` | 分支 `3` + Colab | 层次树 + 四类边时空过程图检索 + 回环增量修订（外观免重算）+ Colab 训同屋实例重排头；GOAT 同起点配对 $\Delta\text{SR}/\Delta\text{SPL}$ 净增益 | 主线 2（查+改） |
| **阶段 2（静态全链路）** | `5. exp/recon-3d-mesh-assets` | 分支 `3` / `D`（离线） | 离线派生转连通 Mesh、`Hunyuan3D` 物体补全生成、游戏 PBR 素材库、`2DGS` 对照；零在线帧率拖累 | 主线 2 → 3D 重建 |
| **阶段 2（静态全链路）** | `2. feat/agent-streaming-monitor` | 分支 `1` + `8aebc95` | 瘦身事件驱动流式仲裁与按需 Fork（取消独立动态危险模型）、修 `llm_agent_client.py` / `native_waypoints.py` 2 处 Bug、接 `dynanav_adapter.py` 到 Isaac Sim | 主线 1（流式+双仿真） |
| **阶段 3（动态感知）** | `6. exp/slam-dynamic-and-generalize` | `D` + `ref/dynanav-dynamic-pedestrian`（按需 `ffd7882`） | `FAST-LIVO2` 静中剥动 + 10 Hz LiDAR / 30 Hz KF 外推 + 切向光度可拒绝修正（消融净收益）+ 室外坡道与余下场景泛化；2 cm 静态基准零退化 | 主线 3（动态 SLAM） |
| **阶段 4（4D 叠加与大闭环）** | `7. feat/4d-object-seg-tracking` | 分支 `6` + `3`（可选 Colab） | 串联短时 `track_id` 与长期 `instance_id`；物体分割 + 跨帧跟踪 + 遮挡补全 + 刚体/铰接/可选骨架；严格区分未观测/已搬空/换 ID | 4D 核心叠加 |
| **阶段 4（4D 叠加与大闭环）** | `8. feat/4d-memory-and-embodied-loop` | 汇合 `2 + 4 + 7` | 把 4D 跟踪接通记忆查改与事件流式仲裁，在 Isaac Sim 动态多层真物理环境跑通全链路大闭环 | 全栈 4D 合龙 |

   - **四条跨主线轻量接口契约**：① **主线 3 → SCAN / 主线 1**：`DynamicOccupancy`（带 `track_id`、来源 `lidar_measured / vision_updated / predicted`、首帧 `unknown` 占据、有效期）+ `LocalizationHealth`；② **主线 3 / 共享视觉 → 主线 2 / 4D**：不可变 `observation_id` 证据链；③ **主线 2 / 4D → 主线 1**：`SceneDelta`（带 `pose_revision`）；④ **主线 1 → 执行**：`TaskEvent`（带 `run_id, action_epoch, frame_seq`，打断重规划发可恢复 `cancel` / `preempt`）。
   - **既有分支提取纪律与算力配额**：`feat/semantic-keyframe-graph@46e0b48` 供分支 `3/4` 提取节点契约；`feat/agent-preemptable-active-slam@8aebc95` 供分支 `2` 提取 `action_epoch` 抢占接口；`perf/slam-backend-cpp-icp@ffd7882` 仅实机剖面确认卡顿时进分支 `6`；`ref/dynanav-dynamic-pedestrian` 提供动态复现。严禁整枝一次性混并；`Google Colab 1,000 CU` 按 `ADR-0005` 分档封顶投入分支 `1/4/7`。

## 3. 活跃分支现状与关键提交 Log 摘要

2026-11-28 本地 `git branch -vv` 核对结果；所有本地分支均交由用户审核后自行合并与推送。

| 分支 | HEAD | 定位 |
| --- | --- | --- |
| `main` | `2144e5f` | 集成、发布与独立验收入口（Phase 3 已收口，与 `origin/main` 一致） |
| **`feat/static-memory-streaming`** | **`a7fd71a`** | **主力记忆与 L7 Agent 分支（代码基线 `e159c8a` / 文档 `a7fd71a`，与远端一致）**：独立 `agent/` 运行时、`SAM 3 + SigLIP` 服务、Set-Cover 关键帧（`79.54%` 压缩）、`GlobalRegionMemory`、`ObjectNavFinishVerifier`、三态熔断、`SharedHabitatSceneHost`（**948 单测 / 60 回归全绿**） |
| **`exp/live-v4-ba-sliding-window`** | **`aada1f1`** | **D 路径 V4 在线闭环基线（领先远端 2）**：1h 51m / 343.72 m 两层闭环（Umeyama 终图 `2.02 cm` / 在线 `2.68 cm`）、29 MB Object Surface Nets、冻结基线与三轴总纲 |
| `exp/slam-v4-oriented-planar-ba` | `6da00f3` | V4 室外通用对照配置（R3LIVE / M3DGR 边界实测） |
| `feat/semantic-keyframe-graph` | `46e0b48` | 已过 Astra 审计的异步语义关键帧图（待按需提取到分支 `3`/`4`） |
| `feat/agent-preemptable-active-slam` | `8aebc95` | 已过 Astra 审计的流式 Fork 与 `action_epoch` 抢占（待提取到分支 `2`） |
| `perf/slam-backend-cpp-icp` | `ffd7882` | C++17 / Eigen3 后端热路径加速器（待剖面确认后单独验收） |
| `exp/slam-backend-stabilization` | `6e1a489` | **Phase 4 稳定化新底座**：ADR-0004 评测器、B1+B2+B3、Astra 全量 P0/P1/P2 修复、30 组报告与公式手册 |
| `exp/slam-v2-measured-submaps` | `59c60a4` | 正交 v2：多帧实测子图 + `candidate` 锚点 |
| `exp/slam-v2-surface-gate` | `0e98406` | 正交 v2：平面/表面覆盖门 + ZUPT 体素解耦 |
| `exp/slam-v2-visual-retrieval` | `2aeea82` | 正交 v2：视觉检索 + 3D 去重验证 + ROS 在线加载与存图 |
| `exp/startup-walk-imu-bias` | `76138ba` | 前端/规划实机闭环修复（起步稳态门、带噪 IMU、滑窗、`0.95 m` 净空、直通脱困 `maneuver`） |
| `exp/slam-measured-submaps` / `exp/slam-surface-gate` / `exp/slam-visual-retrieval` | `5b300b0` / `958ec01` / `16883d1` | 旧 v1 Phase 4 对照分支（基于 `9816a2b`，保留作历史参照） |
| `fix/livo-yaw-and-imu-patch` | `5049ec7` | 已并入 `main`（merge `9b6dc85`），证据保留 |
| `exp/pct-scan-stairs-truth` | `bb2eef2` | Phase 2 真值闭环复现入口 |

### 3.1 `main` — `2144e5f`（= `origin/main`）

定位：集成与发布基线；Phase 3 全部成果已由用户合并、推送。近期关键提交：

| 提交 | 摘要 |
| --- | --- |
| `2144e5f` | `fix(viz): use dynanav_lio_map as Fixed Frame in dynanav_live_pointcloud.rviz` |
| `ca754ab` | `feat(viz): add live RViz2 point-cloud growth config and recorder CLI` |
| `9816a2b` | `docs(adr): add ADR-0001 yaw latency decision and ADR-0002 embodied-agent outlook` |
| `9b6dc85` | `Merge fix/livo-yaw-and-imu-patch: LIVO yaw latency fix and 0011 IMU patch native build wiring` |
| `5049ec7` | `fix(yaw,slam): eliminate LIVO yaw rotation_space_stale and wire 0011 IMU patch into native build` |

### 3.1b `feat/static-memory-streaming` — `a7fd71a`（主力记忆与 L7 Agent 开发分支；与 origin 一致，代码基线 `e159c8a`）

定位：建立顶层独立 `src/embodied_navigation/agent/` 编排层（16 个 `.py` / 5,009 行，含全仓 AST 单向依赖守卫），打通三态熔断 LLM 路由、三层解耦并发、Set-Cover 纹理关键帧与 `GlobalRegionMemory`（948 单测 / 60 回归全绿；5 场景 HM3D Dev `agent-v4` `81.25%` SR）。关键提交（新→旧）：

| 提交 | 摘要 |
| --- | --- |
| `a7fd71a` | `docs(plan): align roadmap, ADR-0005, perception/memory baselines, and Colab budget` |
| `e159c8a` | `feat(agent): add 3-state LLM failover circuit and shared-scene Habitat concurrency` |
| `0311bd0` | `refactor(agent): establish top-level agent/ layer with unidirectional dependency guard` |
| `e6b90af` | `feat(agent-runtime): enable multi-floor stair traversal, pure 500-atomic budget, and confusable stop verifier` |
| `7eb9d43` | `feat(semantic_mapping): add surface-visibility Set-Cover keyframes and geometric+semantic global region memory` |
| `4f81110` | `feat(agent_runtime): add standalone 200K multi-turn runtime, contrastive vision, and epistemic world memory` |

### 3.2 `exp/startup-walk-imu-bias` — `76138ba`（历史 Phase 4 前置实机修复分支；关键表列至 `9e3408d`）

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
| [ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md) | 单主干流式 Fork 异步控制与关键帧锚定语义图片图数据库 | 提案 / 2026-11-28 按 ADR-0005 瘦身 | System 0/1/2 职责红线；语义节点绑定 `keyframe_id`；后端停驻期 ZUPT/重力先验；**System 1 已瘦身为事件驱动任务仲裁器（取消独立几何动态模型）** |
| [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) | 面向 FAST-LIVO2 的增量 Mesh / 3DGS 重建演进与三层 SLAM 后端评测口径 | 提案 / 评测规范（2026-11-21） | 在线/终图 ATE·AOE·RTE·ROE + 逐关键帧演化曲线 + 逐秒位姿分布 + TUM 导出；先做离线关键帧缓存（200–500 MB） |
| [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md) | 因果关键帧记忆、Mesh+骨架混合底座、树/图过程检索与 3 主线 + 4D 8 分支规划 | 已定稿 · 2026-11-28 修订 | `observation_id` + 过程链 + 推理先验权威账本与 4 条跨主线契约；事件驱动流式瘦身；动态前端切向光度拍板；4 阶段 8 分支与 Colab `1,000 CU` 封顶 |

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
