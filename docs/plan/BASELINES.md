# 七层架构冻结基线与子模块优化边界（BASELINES）

> 盘点与冻结日期：2026-11-28 · 主力记忆/Agent 分支：`feat/static-memory-streaming`（HEAD `e159c8a`，948 单测 / 60 回归） · D 路径 V4 分支：`exp/live-v4-ba-sliding-window`（HEAD `aada1f1`） · 后端对照分支：`exp/slam-v4-oriented-planar-ba`（`6da00f3`） · 集成主线：`main`（`2144e5f`）
>
> 配套决策与学术查重报告见 [2026-11 学术创新查重与论文方向总评](research/2026-11-novelty-audit-and-directions.md)、[3D/4D 表征与混合检索 Trade-Off](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md) 与 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)。

---

## 1. 设立原则

在 2026 年底，具身导航宏观大框架（冻结 VLM + 场景图 + 规划控制 + 四足执行）已高度趋同（如 `SuperNav`、`SuperMap`、`TIC-VLA`）。本项目的工程与科研策略明确收拢为：

1. **控制变量**：全栈 7 层全部冻结**内部基线（Internal Baseline）**；未来优化任意单个子模块时，其余 6 层严格锁定版本、配置与数据输入。
2. **按需引入外部强基线**：不一次性把 7 层的外部框架全装进本机，仅针对正在优化的那 1 层拉入对应的 1–2 个 2025–2026 顶会强基线做同台对照。
3. **证据口径分层**：严格区分**闭环物理实跑**、**同包离线重算（仅评估计器）**与**待验证假设**；禁止把离线终图 ATE 或自家旧版 bug 的修复降幅直接包装为闭环任务收益。

---

## 2. 本机硬件与算力基线（用于约束模块选型）

- **GPU**：单卡 `RTX 5070 Ti`（**16 GB VRAM**；空载可用约 `14.3 GB`）
- **Host RAM**：`62 GB`（空载可用约 `51 GB`）
- **云端增援算力**：**Google Colab `1,000 Compute Units (CU)`**（2026 年动态耗速参考：`L4 ~1.7–3.0 CU/hr` 约 `330–580 h`、`A100 ~5.4–13.0+ CU/hr` 约 `75–185 h`、`H100 ~18 CU/hr` 约 `~55 h`）。严格按阶段解锁并先跑 `30 CU` 吞吐测账：特征/难负例缓存 `190 CU` → 冻结 SigLIP 实例重排 `210 CU` → `STOP` 校准 `70 CU` → 动态存在/重激活 `140 CU` → 冻结候选官方清单批推理 `240 CU` → 复验容错 `120 CU`（详见 [Trade-Off 第 6.2 节](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md) 与 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)）。
  - **三条禁烧卡红线**：禁从头训练世界模型/VLA/大 VLM；禁对验证场景离线跑全场景 GS/PBR 再把未来视角送在线 Agent；禁在 A100/H100 上串行等待 Isaac/LLM 网关或对着 5 场景 Dev 反复调参。
- **本地显存红线**：
  - **离线跑（读录包 / MapStore，不开 Isaac Sim）**：`14.3 GB` 显存 + `62 GB` 内存支持串行分阶段显存复用（`Grounded-SAM2 + DINOv2` 约 `3–5 GB` → `gsplat/2DGS` + CPU Open3D TSDF 约 `4–8 GB` → `Hunyuan3D-2 Shape` 约 `6–8 GB` 或低显存 `TRELLIS` 约 `12 GB`）；`GOF`（常超 `24 GB`）直接排除。
  - **在线并发（同时运行 Isaac Sim 6.0 RTX 仿真）**：仿真本身占 `6–10 GB`，剩余 `4–8 GB` **仅够轻量在线分割 + 资产库检索 + CPU/轻量配准**，新款物体 3D 生成必须异步离线或调用外部/云端服务。

---

## 3. 标准化数据集与冻结实验证据包

所有大体积数据位于数据卷 `$PROJECT_DATA_ROOT/embodied-navigation/`（不进仓库）：

| 编号 | 场景与性质 | 数据路径（相对 `$PROJECT_DATA_ROOT/embodied-navigation/`） | 规模与特点 | 当前用途 |
| :--- | :--- | :--- | :--- | :--- |
| **D-Indoor-2F** | 室内两层带噪 IMU 全传感器物理闭环 | `runs/20261123-live-v4-online-ba/live_v4_raw_20261123T193220Z/` | `sim_s = 4602.4 s`（1h 51m）、`343.72 m`、`2,460 KF`、`101` 回环、`80.2 GiB` 原包（含 `map/ 256 MB`、`agent_evidence/ 2.6 GB`、`capture/ 4.0 GB`） | 室内多层后端精度/抖动、回环交付事件表、L4 空间记忆压缩与重建基线（注：单一建筑长轨，不可当作多场景独立样本） |
| **D-Outdoor-R3LIVE** | 真实室外/坡道公开数据（4 序列） | `runs/20261124-r3live-outdoor-eval/` | `degenerate_seq_00`、`hku_park_00`（`209 m / 7.2 m` 坡）、`hku_campus_seq_02`（`332 m`）、`hku_park_01`（`350 m / 8.1 m` 坡，放回偏置 `56.8 cm`） | 检验室内平层/重力先验在室外大坡道上的反例与泛化边界 |
| **D-Dynamic-M3DGR** | 真实动态公开数据（VRPN / RTK 真值） | `runs/20261124-m3dgr-outdoor-eval/` | `Dynamic01`（室内，VRPN GT）、`Dynamic03`（室外人车，RTK GT） | 检验无动态点过滤时前端/在线定位退化与动态滤除模块的真实收益 |
| **D-Sim-Dynamic** | 仿真动态行人支线（待按需抽回） | 参考分支 `ref/dynanav-dynamic-pedestrian` | 原生 DynaNav 动态行人场景与切分代码 | L2 动态点滤除与本体先验选优的受控消融环境 |

---

## 4. 七层冻结基线总表

| 层级 | 核心目录 / 入口 | 内部冻结基线（现状） | 已验证实测基线数据 | 推荐外部强基线（仅优化该层时引入） | 核心验收指标 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **L1 仿真与数据** | `src/embodied_navigation/simulation/dynanav/` | Isaac Sim 6.0 + 物理级带噪 IMU + 200 Hz 去畸变 32 线 RTX LiDAR + RGB + 原生 USD 真值；`--truth-pose` 真值对照组 | Phase 2 真值两层爬升 `+3.25 m` 零撞墙；Phase 4 原生感知两层运行 1h 51m | `TIC-VLA / DynaNav` (ICML 2026)、`I2O2I-Nav` (2026)、`ShapeR` (CVPR 2026) | 跨建筑/跨坡道/跨动态覆盖度；严禁真值泄漏进策略 |
| **L2 感知与动态过滤** | `src/embodied_navigation/perception/` | **几何前置**：自车点过滤 + 200 Hz 去畸变配对（**当前无动态点滤除**，下界基线；迁移时严守 32 线束间空白禁清空占据）<br>**语义视觉底座 (`e159c8a`)**：`perception/vision_tool_server.py` + `vision_tools.py` 无状态常驻视觉服务（`SAM 3` 概念分割 + `SigLIP` 嵌入 + 跨标签掩码/框 NMS） | M3DGR `Dynamic01` 在线 `1.45 m`/终图 `0.97 m`；`Dynamic03` 在线 `4.20 m`/终图 `1.79 m`；`SAM 3 + SigLIP` 常驻服务已支撑 `agent-v4` 并发评测 | **几何派**：`DUFOMap` / `Dynablox`（128 线方法迁移须先做 32 线束型 + 四足步态消融）<br>**语义派**：`YOLOE` 高频初筛、`DINO + SigLIP` 空间 Patch、`YOLO-World + SAM2` 反投影 | 动态点剔除 F1、静图鬼影残余率、高动态在线 ATE、Recall@20 与单帧延迟 (ms) |
| **L3 定位建图（前端 + 后端）** | `third_party/fast_livo2/` + `src/embodied_navigation/localization_mapping/backend/` | **前端**：打 0011 IMU + yaw 延迟补丁的 `FAST-LIVO2`<br>**室内统一在线 BA**：[slam_combined_lidar_fine_gravity_ba.json](../../configs/localization_mapping/slam_combined_lidar_fine_gravity_ba.json)（含 `abbe3da` 统一目标修复）<br>**室外坡道配置**：[slam_combined_lidar_ba_outdoor.json](../../configs/localization_mapping/slam_combined_lidar_ba_outdoor.json)<br>**纯 PGO 对照**：[slam_combined_lidar_gravity.json](../../configs/localization_mapping/slam_combined_lidar_gravity.json) | **两层 343 m**（前端 / 旧双目标 / 统一修复）：<br>- Umeyama 在线 `2.40 / 8.56 / 2.68 cm`；终图 `2.40 / 2.49 / 2.02 cm`<br>- 初始 SE(3) 在线 `9.69 / 11.52 / 8.80 cm`；终图 `6.62 cm`<br>- Layer 3：`max_cum_jitter_m` `13.67 → 1.52 m`、`mean_cum_jitter_m` `4.97 → 0.34 m`、`max_shift_m` `47.5 → 14.5 cm`、`p95_iqr_m` `11.9 → 3.5 cm`<br>**R3LIVE 室外**：`campus_02` 闭合 `28.23 → 5.29 cm`、`park_01` Z `76.48 → 0.12 cm`、`park_00` 3D `2.19 → 3.25 cm`（轻度回退反例） | **在线整栈**：`Voxel-SLAM`（港大 MARS）<br>**离线终图上限**：`HBA`、`MAD-BA` (RA-L 2025)<br>**一致性/地形**：`Consistency-Improved LiDAR-Inertial BA` (RA-L 2026)、地面动态加权基线 | 三层因果指标（Online-Adopted ATE / Peak Causal / 往返多余抖动 $E_i$）+ Final ATE + 峰值时延 + 零调参跨域通过率 |
| **L4 空间记忆与 3D 重建** | `src/embodied_navigation/localization_mapping/semantic_mapping/` + 分支 `feat/semantic-keyframe-graph` / `feat/static-memory-streaming` | **点云存储下界**：`MapStore`（`256 MB`）+ `agent_evidence/`（`2.6 GB`）<br>**Mesh 平面压缩 + 骨架预测地基**：[object_mesh_memory.py](../../src/embodied_navigation/localization_mapping/semantic_mapping/object_mesh_memory.py)（Surface Nets，`29 MB`，`fb35dd8`）+ 细部点云残差<br>**关键帧稀疏覆盖 + 树/图过程检索方向**：[texture_keyframe_selector.py](../../src/embodied_navigation/localization_mapping/semantic_mapping/texture_keyframe_selector.py)（`79.54%` 冗余剪枝）+ `GlobalRegionMemory`（`e159c8a`）<br>**场景图基线**：`feat/semantic-keyframe-graph`（`46e0b48`） | 343 m 场景下纯 LiDAR Surface Nets 体积 `29 MB`，但存在接触家具粘连、背面空洞与偶发穿墙裁剪；`HabitatWorldDaemon` 已落地 Set-Cover 纹理帧压缩（`782 → 160` 帧），待补 16-bit 压缩深度 PNG、过程轨迹/推理先验树图检索及 `exp/mesh-visibility-retrieval` 3 天遮挡过滤实验 | **4D 语义图**：`SuperMap` (RSS 2026, `2608.22896`) / `Hydra++` (IROS 2026) / `CGFM-Nav` (`2608.29114`)<br>**视觉记忆检索**：`3D-Mem` (CVPR 2025) / `MTU3D` (ICCV 2025)<br>**表面与材质对照**：`Facet-SLAM`、`2DGS + CPU TSDF`、`GS-2M` (`2509.22276`, 仅作离线重打光对照) | 存储压缩比 (MB)、重建总耗时与单物延迟、相对原生 USD 的 **3D IoU / Chamfer / Free-Space 侵入率**、遮挡剔除 `Top-1`、树/图检索 **Recall@1/20 & 查询 p95** |
| **L5 规划与交付接口** | `src/embodied_navigation/planning/` + `src/embodied_navigation/navigation/` | `PCT` 全局规划（净空 `0.95 m`）+ `SCAN` 局部规划 + `RouteTracker` + 停-转-稳 + 直通脱困 `maneuver`；`backend.py` 负责 `map→odom` 修正发布 | 真值两层零碰撞；原生感知两层长时可闭环巡航，但回环发布与控制器消费的精确延迟/净空因果影响尚未量化 | **修订交付强基线**：ROS REP-105 标准连续 `odom` 跟踪 + 周期重规划、每次回环全重规划、RTAB-Map 图节点锚定路径<br>**备选规划器**：FAR Planner / Nav2 MPPI | 导航成功率 (SR/SPL)、窄门/楼梯碰撞率与最小净空尾部、不必要重规划次数、停顿恢复时间 |
| **L6 机器人执行与本体** | `src/embodied_navigation/robot/unitree/robogauge/` | `MoE-CTS` 四足 RL 运动策略（直接响应 `cmd_vel` 上下楼梯，零楼梯特化规则） | 两层楼梯反复上下稳定运行 | **本体里程计对照**：纯 IMU 预积分、正向运动学腿部里程计（FK）、`AirIO` / `Legolas` 协方差先验 | 无外感盲走漂移 (RPE)、踩滑/楼梯冲击下的协方差校准度（NEES / NLL） |
| **L7 Agent 决策与 Skill** | `src/embodied_navigation/agent/` + `.agents/skills/dynanav-explore/` + 分支 `feat/static-memory-streaming` / `feat/agent-preemptable-active-slam` | **L7 独立运行时 (`e159c8a`)**：`SAM 3 + SigLIP` 常驻视觉服务、`ObjectNavFinishVerifier`、三态 LLM 熔断、`SharedHabitatSceneHost` 并发<br>**异步流式哨兵草稿**：`feat/agent-preemptable-active-slam` (`8aebc95`) | `feat/static-memory-streaming@e159c8a`（**948 单测 / 60 回归**）；5 场景 HM3D Dev（16 有效子任务，1.8 m，单目 90° RGB-D，GT 位姿）达 **SR `81.25% (13/16)`**（95% Wilson CI `57%–93%`）、**OSR `87.50% (14/16)`**、Sub 2–4 `81.82% (9/11)`、KV `96.40%`、`0` 超时；待 `exp/habitat-score-parity` 官方 scorer 校准 | **Agent 标杆与协议对比**：<br>• `CGFM-Nav` (`2608.29114`): GOAT Val Unseen 36 ep/278 子任务 `63.0% / 39.6%` @ 0.25 m（全量 360 ep/2,669）<br>• `SuperNav` (`2610.12126`): OVON 120 题 360° Geo `73.33% / 0.4105` @ 1 m、`68.33%` @ 0.25 m、学习执行器 SPL `0.1443`<br>• 端到端 VLA：`NaVILA` / `TIC-VLA` | 官方完整清单 SR / SPL、OSR、`1.0 m / 1.8 m` 双口径、同起点持久 vs 清空 $\Delta\text{SR}/\Delta\text{SPL}$、总 API Token 成本、Oracle vs 非 Oracle 掉点 |

---

## 5. 后续模块划分与演进纪律

未来无论选择哪一条工程或论文主线，均按**“4 个可独立推进的子模块包”**拆分支演进，严禁把多条方向的改动混在同一条分支里：

| 子模块包 | 建议独立分支名 | 所属层 | 核心产出边界 | 依赖前提 |
| :--- | :--- | :--- | :--- | :--- |
| **包 M1：回环修订相对风险交付门控** | `exp/loop-revision-control-gate` | L3 ↔ L5 | 5 时间戳对齐事件表 + 机体系相对路径/净空变化 $Y^{(k)}$ 计算 + 交付门控与分叉闭环评测协议 | 零外部重依赖，直接基于 `D-Indoor-2F` + 至少 1 个独立建筑场景 |
| **包 M2：四足 RL 本体先验动态点滤除** | `exp/prop-latent-dynamic-filter` | L6 → L2 | 从 `MoE-CTS` 隐状态 + 关节/IMU 输出 $(\hat{\mathbf{v}}_{\text{prop}}, \hat{\boldsymbol{\omega}}_{\text{prop}}, \Sigma_{\text{prop}})$ + GNC-IRLS 动态点选优 + 动态目标历史追踪 | 接回 `ref/dynanav-dynamic-pedestrian` + `D-Dynamic-M3DGR` 对照 |
| **包 M3：原型实例化 Mesh 与四层时空记忆** | `exp/prototype-mesh-hierarchical-memory` | L4 | `Grounded-SAM2` 遮挡感知分割 + DINOv2/几何原型库检索 + `Hunyuan3D Shape / TRELLIS` 冷启动 + LiDAR 表面/Free-Space 配准 + L0–L3 金字塔检索（可选：回环证据撤销） | 离线使用 `D-Indoor-2F`（`16 GB` 显存串行复用）+ 原生 USD 留出真值评测 |
| **包 M4：地形自适应重力/支撑面统一 BA** | `exp/terrain-adaptive-unified-ba` | L3 | 重力 $g$ 与支撑面法向 $n_k$ 双潜变量解耦 + 退化可观性分析 + 零调参统一室内外配置 | 固定 `D-Indoor-2F` + `D-Outdoor-R3LIVE` + 新增坡上启动/平坡过渡场景 |

> **与 2026-11-28「3 主线 + 4D 叠加 · 4 阶段 8 分支」的映射**（详见 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md) 与 [ROADMAP 第 2 节 Phase 5](ROADMAP.md)）：上述 **M1–M4** 聚焦**算法主线级论文命题**（其中 `M3` 对应阶段 1/2 记忆主线分支 `3. feat/memory-hybrid-store-build`、`4. feat/memory-tree-graph-retrieval`、`5. exp/recon-3d-mesh-assets` 与阶段 4 `7. feat/4d-object-seg-tracking`；`M2` 与 `M4` 对应阶段 3 感知主线分支 `6. exp/slam-dynamic-and-generalize`；`M1` 对应阶段 2/4 闭环分支 `2. feat/agent-streaming-monitor` 与 `8. feat/4d-memory-and-embodied-loop`），而 `1. feat/agent-habitat-benchmarks` 先行对齐 Habitat 官方 `ObjectNav / OVON / GOAT` Scorer 与 360° 观测喂入评测。

> **可视化工具归位**：多视角防穿模录制模块（`src/embodied_navigation/visualization/spring_arm_camera.py`、`scripts/record_dynanav_rviz.py`）与三层评测器（`localization_mapping/backend/evaluation.py`）作为公共基础设施保留，不作为独立论文创新点。
