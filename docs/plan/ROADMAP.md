# 开发路线图（ROADMAP）

> 盘点日期：2026-11-28 · 集成基线 `main` `2144e5f`（= `origin/main`） · 当前主力记忆/Agent 开发分支 `feat/static-memory-streaming`（`e159c8a`，**948 单测 / 60 回归全绿**，`agent-v4` 5 场景 HM3D Dev `81.25%` SR） · D 路径真物理闭环分支 `exp/live-v4-ba-sliding-window`（`aada1f1`，V4 在线定向 Planar BA + 29 MB Object Surface Nets） · Phase 4 稳定化底座 `exp/slam-backend-stabilization`（`6e1a489`） · 活体文档，随阶段推进更新。

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

### Phase 5 — 感知 × 记忆 × 失败驱动控制 × Agent 编排四轴演进主线（2026-11-25 确立，2026-11-28 按 ADR-0005 修订）

**定位与原则**：在 V4 两层 `2.02 cm` 闭环（`aada1f1`）与 `feat/static-memory-streaming@e159c8a`（`948` 单测、`81.25%` 5 场景 Dev SR）双底座冻结之上，结合 `Google Colab 1,000 CU` 云端增援，按“先对齐官方口径/跑 3 天低成本证伪、再扩张”推进（详见 [VLN 三轴演进总纲](research/2026-11-vln-perception-memory-strategy-synthesis.md)、[3D/4D 表征与算力策略 Trade-Off](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md) 与 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)）：

1. **感知主线（语义感知 + 动态感知，L2 / L6 → L2）**：
   - **语义感知**：① **已落地底座**（`e159c8a`）：`perception/vision_tool_server.py` + `vision_tools.py` 无状态共享视觉服务，提供 `SAM 3` 概念分割 + `SigLIP` 嵌入 + 跨标签掩码/框 NMS；② **下一步增量**：补充 `DINO + SigLIP` 空间网格 Patch 双特征避免漏框即不可逆丢信息、接入 360° 四向环视（`separate4 / mosaic2x2`）与 `PixelGoal`（`goto_pixel / query_pixel_3d`）及语义墙/门洞轮廓叠加，并在动态场景按需补高频轻量 `YOLOE` 初筛。
   - **动态感知与 Go2 跨模态分工**：明确按传感器优势分工——`RGB` 主管人/物身份、交叉遮挡 ReID、32 线束间细小物候选与门/抽屉铰接线索；`32 线 RTX LiDAR` 主管暗光制动净空、门扇剩余通道宽度、运动去畸变静态定位与实测自由射线清鬼影（**铁律：32 线束间空白绝不等价于实测自由空间；迁移 128 线 `Dynablox` 前须先完成 32 线束型 + 四足步态抖动消融**）；`IMU + 关节 + 可靠足端接触` 提供高频传播与打滑告警（**严禁把 `MoE-CTS` `cmd_vel` 当已实现里程计**）。先在 `ref/dynanav-dynamic-pedestrian` 与 `D-Dynamic-M3DGR` 用准确机体运动验证人群遮挡动态分割上界，成立后再走 RoboGauge 极端地形（30 cm 台阶/凹坑/悬崖/烂泥阻力）`AirIO / Legolas` 风格本体不确定性训练与交替迭代动态 SLAM，并用“几何静 + 外观动”指纹隔离屏幕动画。
2. **记忆主线（统一底账 / Mesh+骨架混合 / 树图过程检索 / 4D 演化，L4 / L7）**：
   - **单一权威底账**：全系统共用不可变 `observation_id` 关键帧底片（`RGB + 16-bit 深度 / 局部 LiDAR + 光照锚点`）+ **SE(3) 不变过程轨迹链（里程计步间位移 $\Delta x,\Delta y,\Delta\theta$ 与访问时序）** + **历史推理先验/语义直觉**。Habitat 已落地 `TextureKeyframeSelector`（`79.54%` 冗余剪枝）与 `GlobalRegionMemory`，补存 `16-bit` 压缩深度 PNG（单栋 `+1.2 MB`）即与 D 路径 `MapStore` 彻底统一；
   - **Mesh + 点云混合压缩与「骨架 + Mesh」物理预测地基**：墙/地/顶等大平面走 `Mesh` 极致压缩冗余点云，细部与曲面保留点云残差；静态建筑骨架 + 动态刚体 $T(t)$/铰接转轴骨架搭配 `Mesh`（`MoMa-SG`）作为可建模与低维前向预测地基（行人先存时效占据+速度+外观防半身遮挡脑补，交互姿态需求时再升格骨架）。在线优选原图 Patch 投影，**单材质 PBR 与全场景在线 `3DGS/4DGS` 不进在线主链**（`2DGS / Hunyuan3D-Omni / PBR USD` 仅按需离线派生）；在线“Mesh 遮挡重排”子项由 `exp/mesh-visibility-retrieval` 3 天实验单独裁决；
   - **「树检索 + 四类边时空过程图检索」与 4D 失效治理**：上层构建 `建筑→楼层→房间→探索过程段→关键帧` 多归属层次树（永久保留全局回退）+ **里程计过程 / 门洞邻接 / 跨视角外观 / 推理先验直觉**四类边时空过程图；外观特征按 `observation_id` 建库、严禁混入绝对世界坐标；底层首版固定精确矩阵乘 $O(Nd)$ + 子图体素哈希 + 实例时序表（实测超延迟再开 `HNSW / FreshDiskANN` 墓碑压实）。借助 `Colab 1,000 CU` 训冻结 `SigLIP` 同屋实例重排头与 4D 存在/重激活判别头；拆存 `movable_class / current_motion_evidence / last_existence_evidence`，同 `pose_revision` 下先扣静态背景共同位移，显式区分 `not_visible / observed_empty_at_old_pose / identity_swapped`。
3. **控制与策略主线（失败中优化，L5 / L7）**：
   - 底层 PCT + SCAN + RouteTracker + MoE-CTS 保持冻结，只叠加**回环机体系相对净空风险交付门控**与**前置 `STOP` 几何校验**；
   - 策略侧围绕失败反馈闭环演进：回合内 `Attempt & Failure Ledger` 地理黑名单 + 跨回合受置信域门控的 Skill/Tool 迭代；
   - 评测侧在兼容上游 DynaNav 指标之上，引入 8 项超越 `SR/SPL` 的 **Agent 认知与记忆诊断指标**（证据命中翻错率、过时经验翻错率、无效重访率、跨层记忆混淆率、重复踩坑率、停驻/停机失误、长程退化斜率与定位落差 $\Delta\text{SR}$）。
4. **Agent 编排层演进主线（L7 `src/embodied_navigation/agent/`，详见 [agent/README.md](../../src/embodied_navigation/agent/README.md)）**：
   - **已建成底座（`feat/static-memory-streaming` @ `e159c8a`）**：独立 `EmbodiedAgentRuntime` 多轮推理循环、`PrefixStableContextManager`（实测 `96.40%` KV 缓存命中 + `155K` 里程碑压缩）、三态共享熔断 LLM 路由（`OpenCode Go ↔ DeepSeek Official`）、`tools/` 与 `ObjectNavFinishVerifier` 校验、以及三层解耦并发架构（Tier 1 `SAM 3 + SigLIP` 共享视觉服务 / Tier 2 多会话隔离记忆与上下文 / Tier 3 `SharedHabitatSceneHost` 单 EGL 线程）；
   - **后续 Agent 层三大演进项**：
     1. **双环境适配器统一**：在现有 `adapters/habitat_{daemon,runner,scene_pool}.py` 之外，新增薄适配器 `adapters/dynanav_adapter.py` 接回 Isaac Sim / DynaNav（经 `explore_waypoint_client.py` 与 `action_epoch` 邮箱），实现同一套 `agent/` 运行时同时驱动 Habitat 与真实物理四足仿真；
     2. **工具集与流式抢占升级**：接入 `PixelGoal`（`goto_pixel / query_pixel_3d`）、树/图过程记忆检索工具（含里程计过程链与推理先验写回），并从 `feat/agent-preemptable-active-slam@8aebc95` 提取 **System 1 阅后即焚流式 Fork 哨兵**，支撑行进中语义抢占与过期航点撤销；
     3. **技能进化与认知诊断闭环**：将回合内 `Attempt & Failure Ledger`、受置信域门控的跨回合 Skill/Prompt 迭代，以及 8 项 Agent 认知诊断指标接入 `agent/runtime/hooks.py` 与 `agent/prompts/`。

5. **2026-11-28 路线收口：两阶段推进（Route 1 完整 Habitat → Route 2 真实仿真 + 4D 动态）与 11 分支单假设总表**（详见 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md) 与 [Trade-Off 第 3–6 节](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)）：
   - **现状起点**：`feat/static-memory-streaming` (`e159c8a`) 的 `agent-v4` 已在 5 场景 HM3D-val Dev（16 有效子任务，1.8 m，单目 90° RGB-D，GT 位姿）跑出 `SR 81.25% (13/16)`（95% Wilson CI `57%–93%`）、`OSR 87.50% (14/16)`、跨子任务 `81.82% (9/11)`、`0` 超时、KV 命中 `96.40%`、帧压缩 `79.54%`；D 路径 `exp/live-v4-ba-sliding-window` (`aada1f1`) 已完成 1h 51m / 343.72 m 两层闭环（终图 `2.02 cm` / 在线 `2.68 cm`）+ 29 MB Object Surface Nets。
   - **第一周三件必交验收（Week 1 Gate）**：① 官方 scorer 逐题一致性报告（回答“分数可信吗”）；② 3 天 Mesh 可见性重排错例配对表（回答“Mesh 值得吗”）；③ D 路径单次回环前后同一对象的因果回放（回答“双栈真能共用记忆吗”）。
   - **11 条预计开发分支与推进顺序**（基线简称 `H = feat/static-memory-streaming@e159c8a`，`D = exp/live-v4-ba-sliding-window@aada1f1`；依赖项均指验收固定提交）：

| 顺序 / 分支 | 基线 | 单点假设与最小验收 / 止损门控 | 归属 |
| :--- | :--- | :--- | :--- |
| `1. exp/habitat-score-parity` | `H` | 现有运行可由官方 scorer 忠实重放；逐题成功位一致、SPL 一致、分母对账；失败先修评分 | Route 1 |
| `2. exp/memory-causal-replay` | `H` | `observation_id`、里程计过程轨迹链、推理先验与 `pose_revision` 足以因果重放；重复幂等、迟到归属正确、前缀零未来证据 | 双栈公共契约 |
| `3. exp/mesh-visibility-retrieval` | `2` | 仅加局部 Mesh 首交遮挡重排子功能（**不否定 Mesh 平面压缩与骨架建模地基**）；3 天门槛：比最强便宜对照（体素/深度重投影）Top-1 ≥ +5 pp、完全遮挡误留相对降 ≥ 25%、可见误杀 ≤ +2 pp、增量 p95 ≤ 50 ms；不达标只停本子项 | 混合记忆子项证伪 |
| `4. exp/habitat-memory-retention` | `1` | 融合树/图过程记忆（外观+里程计过程+相对拓扑+推理先验）在同预算下改善 GOAT 后续任务；先拆失败，再按同起点持久/清空配对，无 $\Delta\text{SR}/\Delta\text{SPL}$ 净增益即停扩 | Route 1 / 树图记忆 |
| `5. exp/goal-frontier-prior` | `1` | 无可靠候选时，融合历史推理语义直觉的目标条件前沿提升首次可见率；固定步数对比普通占据前沿，OSR 或首次发现距离无改善即停 | Route 1 / 探索 |
| `6. exp/instance-reranking` | `2` | 训练场景同屋同类难负例改善跨模态实例选择；留出房屋提升 ≥ +5 pp 且类别不退，再接 `1` 正式评测 | Colab → Route 1 |
| `7. exp/stop-calibration` | `1` | 先按全量 `OSR-SR` 估“到达后错停”可挽回上限；上限过小则不开训 | Colab → Route 1 |
| `8. exp/submap-pose-revisions` | `D` | 局部锚点随回环同步重投影；与逐帧重投影对照，分开验刚体修订与子图内形变，静态回放零搬移误报 | Route 2 / 基础 |
| `9. exp/dynamic-free-space` | `8` | 去畸变实测自由射线可识短时变化（先做 32 线束型 + 步态抖动消融）；出现静态定位退化即停 | Route 2 / 几何 |
| `10. exp/object-existence-history` | `9` | 区分“不可见”与“看到旧址为空”降低旧址误访；对照 TTL 与纯几何规则，旧址误访相对降 ≥ 25% 且不增错删再考虑学习式重激活 | Route 2 / 可选 Colab |
| `11. exp/agent-revisit-on-revision` | `10` | 位姿或存在更新后及时撤销过期航点减少白跑；静态任务出现误取消即退回 | Route 2 / 闭环 |

   - **既有分支提取纪律**：`feat/semantic-keyframe-graph@46e0b48` 供分支 `2`、`10` 提取图节点契约；`feat/agent-preemptable-active-slam@8aebc95` 供分支 `11` 提取 `action_epoch` 抢占接口；`perf/slam-backend-cpp-icp@ffd7882` 仅在实机回放剖面证实 ICP 卡死在线预算后单独验收；`ref/dynanav-dynamic-pedestrian` 提供动态场景与复现定义。严禁整枝一次性混并。

## 3. 活跃分支现状与关键提交 Log 摘要

2026-11-28 本地 `git branch -vv` 核对结果；所有本地分支均交由用户审核后自行合并与推送。

| 分支 | HEAD | 定位 |
| --- | --- | --- |
| `main` | `2144e5f` | 集成、发布与独立验收入口（Phase 3 已收口，与 `origin/main` 一致） |
| **`feat/static-memory-streaming`** | **`e159c8a`** | **主力记忆与 L7 Agent 分支（领先远端 1）**：独立 `agent/` 运行时、`SAM 3 + SigLIP` 服务、Set-Cover 关键帧（`79.54%` 压缩）、`GlobalRegionMemory`、`ObjectNavFinishVerifier`、三态熔断、`SharedHabitatSceneHost`（**948 单测 / 60 回归全绿**） |
| **`exp/live-v4-ba-sliding-window`** | **`aada1f1`** | **D 路径 V4 在线闭环基线（领先远端 2）**：1h 51m / 343.72 m 两层闭环（Umeyama 终图 `2.02 cm` / 在线 `2.68 cm`）、29 MB Object Surface Nets、冻结基线与三轴总纲 |
| `exp/slam-v4-oriented-planar-ba` | `6da00f3` | V4 室外通用对照配置（R3LIVE / M3DGR 边界实测） |
| `feat/semantic-keyframe-graph` | `46e0b48` | 已过 Astra 审计的异步语义关键帧图（待按需提取到 `2`/`10`） |
| `feat/agent-preemptable-active-slam` | `8aebc95` | 已过 Astra 审计的流式 Fork 与 `action_epoch` 抢占（待提取到 `11`） |
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

### 3.1b `feat/static-memory-streaming` — `e159c8a`（主力记忆与 L7 Agent 开发分支；领先 origin 1 个提交）

定位：建立顶层独立 `src/embodied_navigation/agent/` 编排层（16 个 `.py` / 5,009 行，含全仓 AST 单向依赖守卫），打通三态熔断 LLM 路由、三层解耦并发、Set-Cover 纹理关键帧与 `GlobalRegionMemory`（948 单测 / 60 回归全绿；5 场景 HM3D Dev `agent-v4` `81.25%` SR）。关键提交（新→旧）：

| 提交 | 摘要 |
| --- | --- |
| `e159c8a` | `feat(agent): add 3-state LLM failover circuit and shared-scene Habitat concurrency` |
| `0311bd0` | `refactor(agent): establish top-level agent/ layer with unidirectional dependency guard`（origin 当前 ref） |
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
| [ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md) | 单主干流式 Fork 异步控制与关键帧锚定语义图片图数据库 | 提案 / 架构演进（2026-11-21） | System 0/1/2 职责红线；语义节点绑定 `keyframe_id`；**后端先补齐停驻期 ZUPT、重力 Roll/Pitch 先验、过密回环抑制** |
| [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) | 面向 FAST-LIVO2 的增量 Mesh / 3DGS 重建演进与三层 SLAM 后端评测口径 | 提案 / 评测规范（2026-11-21） | 在线/终图 ATE·AOE·RTE·ROE + 逐关键帧演化曲线 + 逐秒位姿分布 + TUM 导出；先做离线关键帧缓存（200–500 MB） |
| [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md) | 因果关键帧记忆、Mesh+骨架混合底座、树/图过程检索与 Route 1→2 规划 | 已采纳草案 / 待首周三项门控验收（2026-11-28） | `observation_id` + 过程链 + 推理先验权威账本；Mesh 平面压缩/骨架预测地基 + 3 天遮挡实验；树/图检索与 ANN 分层；11 分支与 Colab `1,000 CU` 封顶 |

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
