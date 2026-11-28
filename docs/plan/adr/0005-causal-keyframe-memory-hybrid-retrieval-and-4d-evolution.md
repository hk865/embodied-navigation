# ADR-0005: 因果关键帧记忆、Mesh+骨架混合底座、树/图过程检索与 3 主线 + 4D 8 分支规划

- 状态：已定稿 · 2026-11-28 修订
- 关联文档：[ROADMAP.md](../ROADMAP.md)、[TASKS.md](../TASKS.md)、[BASELINES.md](../BASELINES.md)、[ADR-0003](0003-streaming-fork-control-and-semantic-scene-graph.md)、[ADR-0004](0004-incremental-mesh-3dgs-and-layered-backend-eval.md)、[3D/4D 表征、混合检索与算力策略 Trade-Off](../research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)

---

## 1. 背景与核心目标

当前仓库在两条主线上各自跑出了硬底座：
- **L7 记忆与 Agent 路线（`feat/static-memory-streaming`，代码基线 `e159c8a` / 文档基线 `a7fd71a`，948 单测 / 60 回归全绿）**：落地 `SAM 3 + SigLIP` 常驻视觉、Set-Cover 关键帧压缩（`79.54%`）、`GlobalRegionMemory`、`ObjectNavFinishVerifier`、三态熔断与 `SharedHabitatSceneHost`，5 场景 HM3D Dev 跑出 `SR 81.25% (13/16)`（95% Wilson CI `57%–93%`）、`OSR 87.50% (14/16)`、跨子任务 `81.82% (9/11)`、`0` 超时；
- **D 路径真物理与真 SLAM 路线（`exp/live-v4-ba-sliding-window` @ `aada1f1`）**：跑通 1h 51m / 343.72 m 两层带噪 IMU 闭环（Umeyama 终图 `2.02 cm` / 在线 `2.68 cm`）+ `29 MB` Object Surface Nets；另有 `feat/semantic-keyframe-graph@46e0b48`、`feat/agent-preemptable-active-slam@8aebc95`、`perf/slam-backend-cpp-icp@ffd7882` 与 `ref/dynanav-dynamic-pedestrian` 等可复用素材。

本 ADR 用于解决多分支各自为政和过早引入冗余模块的问题，将演进框架统一收口为 **「3 条主线（Agent 导航 · 记忆存建查改→3D 重建 · 感知动态 SLAM）+ 4D 叠加（物体分割与运动跟踪）」共 8 条干净分支 · 4 阶段从静态到动态推进**，同时敲定双栈共用底账、动态高频前端的数学边界与流式监控瘦身后的跨主线契约——核心要保证：**静态整套存、建、查、用先完全做扎实，动态与 4D 在站稳的底座上顺理成章叠加上去，不重复造轮子，也不在物理上无解的路径上硬碰。**

---

## 2. 核心架构决策

### 决策 1：单一不可变底账 + 过程轨迹链与推理先验 + 四条轻量跨主线契约

1. **不可变观测原语**：全栈共用同一份权威日志，用不可变 `observation_id` 绑定 `(timestamp_ns, image_patch_ref, local_depth_or_lidar, submap_id, extrinsic_ver, feature_model_ver, lighting_anchor)`；
2. **过程记忆与推理先验正式入账**：在权威底账中显式同步记录 **SE(3) 刚体不变的过程数据**——**里程计步间位移轨迹链（`Episodic Trajectory Chain`，$\Delta x, \Delta y, \Delta\theta$）**、关键帧访问顺序、以及探索中形成的**历史推理先验与语义直觉（`Reasoning Prior & Semantic Intuition`，如区域功能假设、失败黑名单）**；
3. **版本化位姿与实例时序表**：实例仅引用 `observation_ids[]` 并记录 `existence_interval` 与关联置信度；位姿图仅发布带版本号的 `(submap_id, pose_revision)` 子图变换；`HabitatWorldDaemon` 对保留纹理帧同步落盘轻量 **16-bit 压缩深度 PNG**（单栋约 `+1.2 MB`），补齐双栈可重构闭环；
4. **必须守死的 4 条跨层底线**：
   - ① 统一 `pose_revision` 读取权限，禁止上下层混用回环前后两版位姿；
   - ② 异步视觉回包只准写回发起查询时的 `frame_id` 与实例版本；
   - ③ “目标已不在”的否定判断必须同时满足“在视野内 + 无几何遮挡 + 在有效测距内”；
   - ④ 航点、取消与 `STOP` 校验绑定 `action_epoch`。
   - **最小契约验收**：同一回放经历重复消息、视觉迟到与 1 次回环后输出一致可追溯的实例历史，历史前缀查询零未来证据泄漏；
5. **四条跨主线轻量接口契约**（把 3 条主线 + 4D 干净串起来，互不越层）：

   | 生产者 → 消费者 | 轻量契约与核心字段 |
   | --- | --- |
   | **主线 3 → SCAN / 控制；健康摘要 → 主线 1** | `DynamicOccupancy`：含短时 `track_id`、连续局部坐标系、状态时刻与最后实测时刻、速度/协方差与有效期；明确区分来源 `lidar_measured` / `vision_updated` / `predicted`，首帧新前景先置为 `unknown` 占据（不盲目清空也不误并入静墙）。`LocalizationHealth` 报退化类型与证据；回环修订作为独立变换发布，避免全局跳变制造假速度。 |
   | **主线 3 / 共享视觉 → 主线 2 & 4D** | 不可变 `observation_id` 绑定源时戳、本地子图锚、标定与证据引用，让主线 3 的短时 `track_id` 与主线 2 的长期 `instance_id` 挂在同一条可修订因果证据链上。 |
   | **主线 2 / 4D → 主线 1；候选 → SLAM 后端** | `SceneDelta` / `place_discovered` / `target_candidate` / `target_stale`：携带观测引用、`pose_revision`、置信度与有效期，明确区分“不可见”、“实测旧址为空”与“身份交换”。 |
   | **主线 1 → 规划 / 执行接口** | `TaskEvent`：携带 `run_id`、`action_epoch`、`active_seq`、源 `frame_seq` / `observation_id`、截止时刻与原因；中途打断重规划发可恢复 `cancel` / 显式 `preempt`（不发会锁死整轮运行的终止 `stop`）。 |

### 决策 2：Mesh + 点云混合压缩、「骨架 + Mesh」物理地基与离线 3D 重建

1. **明确 Mesh 在系统里的三大核心地位**：
   - **① 大平面压缩 + 细部残差**：地板、天花板、大墙面从大量点云压缩成少片三角 `Mesh`，复杂物体细节保留点云/高密观测残差（大面 Mesh + 细部点云）；
   - **② 骨架 + Mesh = 好建模、好预测的物理地基**：静态建筑拓扑骨架 + 支撑平面 Mesh 提供可达与放置先验；动态刚体 $T(t)$ 或铰接转轴骨架搭配 `Mesh`（`MoMa-SG` 思路）把数万碎点压成低维运动学模型，支持前向物理预测；
   - **③ 无光照污染碰撞与离线 3D 资产出口**：脱离旧灯光烘焙，直接服务物理避障与离线资产派生（转 Mesh、`Hunyuan3D` 物体补全生成、Blender/UE5/Isaac 游戏材质与素材库、`2DGS` 外观对照）；在线纹理优选原始 Patch 投影（随时可回看原图），全场景在线 `3DGS/4DGS` 与引擎单材质 PBR 均不入在线主链；
2. **分支 3 内的低成本可选验证项（Mesh 遮挡重排）**：
   - 在 `feat/memory-hybrid-store-build` 内花 2–3 天对比纯 Patch（A）、点云/体素射线（B）、历史深度重投影（便宜强对照）与 `Surface Nets` 首交（C）（在 ≥ 3 留出场景、120 去重对上严守因果前缀），仅在 C 比最强便宜对照 Top-1 ≥ +5 pp、完全遮挡误留相对降 ≥ 25%、可见误杀 ≤ +2 pp 且 p95 ≤ 50 ms 时才接入检索重排；
   - **注意范围边界**：本小实验只裁决“检索打分时要不要加 Mesh 遮挡过滤”，即使不启用也不否定 Mesh 的大平面压缩与骨架+Mesh 建模地基价值。

### 决策 3：树检索 + 时空过程图检索（上层方向与底层渐进演进）

1. **绝不只靠纯外观向量干搜**：上层记忆检索正式立项 **层次树检索 + 四类边时空过程图检索**，联合利用四路信号：**外观特征（`DINO/SigLIP` Patch）+ SE(3) 不变里程计过程近邻 + 局部相对拓扑 + 推理语义直觉先验**：
   - **树检索**：沿 `建筑 → 楼层 → 房间/区域 → 探索过程段 → 关键帧/物体` 自顶向下筛选，门口/楼梯/开放区允许**多归属**，剪空时**永久保留全局回退**；
   - **时空过程图检索**：构建含 **① 里程计/时序过程边、② 门洞与相对几何邻接边、③ 跨视角外观关联边、④ 推理先验与语义共现边** 的记忆图，支持“入口召回 → 限制跳数 $h$ 的多跳过程扩散 → 空间校验”；
2. **特征索引红线与底层数据结构渐进演进**：
   - 外观向量严格按 `observation_id` 索引，**严禁将绝对世界坐标拼入高维视觉特征**（保证全球回环时特征库与过程相对边免重算；绝对坐标过滤统一放在召回后执行）；
   - **首版固定简洁底座**：精确矩阵乘 $O(Nd)$ + 子图局部体素哈希 + 实例时序表 + 树/图检索；只有精确匹配在实测规模超延迟预算时才引入 `HNSW / FreshDiskANN`（带墓碑 + 增量压实）。若 BA 导致子图内部非刚性形变，子图局部体素/BVH 必须重投脏块。

### 决策 4：主线 3 动态感知数学拍板、多模态互补原则与 4D 叠加

1. **多模态互补原则（废除机械传感器隔离）**：
   - RGB、32 线 RTX LiDAR 与 IMU/关节编码器共同参与感知与定位建图，核心逻辑是“哪个传感器在当前场景下特征更强、信息量更高就优先用谁去互补增强”，而不是人工硬性规定某模态只能干某一件事。
2. **2026-11-28 数学推导与 `codex-cli` 严审拍板（动态避障与 20–30 Hz 高频更新）**：
   - **放弃纯单目时序模拟双目硬算深度**：在本仓库 512×512（$f_x = 292.57\text{ px}$）、Go2 步行 $0.6\text{ m/s}$ 下，25 Hz 相邻帧横向基线仅 `2.4 cm`，2 m / 5 m 处随机深度误差分别达 $\sigma_z \approx 0.28\text{ m}$ / `1.78 m`；若拉大间隔到 0.2 s 攒 `12–15 cm` 基线，延迟反超 10 Hz LiDAR（100 ms）；且走廊正前方直行落在前进对极点（FOE）视差趋零、停车/原地纯转 $b \to 0$、跨帧动态行人（同向 $0.5\text{ m/s}$ 可算出 $\mathbf{6z}$ 深度）违反静态刚体假设并引发肢体深度撕裂；
   - **拍板主线 3 动态前端方案**：
     - **底层定位与静态地图**：沿用带噪 IMU 与 `yaw` 修复的 `FAST-LIVO2`（可开启 `/LIVO2/imu_propagate` 作高频位姿传播，前端零深度学习小模型）；
     - **静中剥动 + 30 Hz KF/IMM 惯性外推**：由 10 Hz 实测 LiDAR 对照静态地图剥离动态点云簇（**按逐点时间戳对齐参考时刻**，避免把扫掠点粗暴贴到 100 ms 整帧末端引发 ~15 cm 错位），构建带恒速先验的卡尔曼/IMM 滤波器，在激光间隙外推至 20–30 Hz 动态占据；
     - **Object-Centric 切向光度可拒绝修正（同构 Dual FAST-LIVO2）**：固定已知静态相机位姿，在 20–30 Hz RGB 帧间对动态簇贴纹理 Patch 求解刚体增量。数值分析证实（4 m 处行人 $1.5\text{ m/s}$、33 ms 位移 5 cm）：横向光流达 **`7.31 px`（2 m）/ `3.66 px`（4 m）**，而迎面径向躯干（`0.4×0.6 m`）光流仅 **`0.76 px`（2 m）/ `0.19 px`（4 m）**、平移条件数达 **`92 / 369`**（`0.1°` 相机姿态误差即引入 `0.51 px` 污染）。因此**默认优先开放切向 2D 视觉更新**（专治近距横穿、急停、变向），**迎面径向与偏航默认交给 LiDAR + 速度先验兜底**，仅当近距且白化径向信息量达标时才按帧放开；上纹理前先做相机视点可见性/深度缓冲剔除再内缩（防止 `0.311 m` `Pcl` 外参带来 `~27 px` 背景墙沾色把目标拉回静止）；运动拟合盯躯干一致块、发给 `SCAN` 的避障占据包络四肢观测；残差超阈值则整帧拒绝更新并退回 KF/IMM 外推。
   - 若未来需要更高密度的前向近距静态体素地图，不靠纯单目三角化，优先走物理双目 / 深度相机（如 `/dynanav/depth`）或轻量预训练单目深度模型 + 10 Hz LiDAR 稀疏锚点仿射拟合 $a\hat\rho+b$（只标近距占据，自由空间清空仍由实测 LiDAR 射线兜底）。
   - **工程红线**：严禁把 32 线束间空白当自由空间；迁移 128 线 `Dynablox` 需先做束型与步态抖动消融；严禁把 `MoE-CTS` `cmd_vel` 当已实现里程计。
3. **4D 叠加与失效治理**：
   - **主线 3 产短时 `track_id`，主线 2 归档长期 `instance_id`**，4D 在 `observation_id` 证据链之上统一做**物体分割 + 跨帧运动跟踪**（遮挡与丢失帧允许靠学习模型与 Track 补全信息），并自然延伸到刚体 $T(t)$、门/抽屉铰接骨架（`MoMa-SG`）与按需人物骨架；行人阶段一避障优先存**时效 3D 占据体 + 速度 + 外观特征**（防半身遮挡瞎补 `SMPL` 腿位），出现交互姿态需求时再升格分段骨架；
   - **四类必防状态**：拆存 `movable_class / current_motion_evidence / last_existence_evidence`，停步行人不融进静态墙、只准实测穿过旧址的自由射线削弱旧占据；跨期变化在同一 `pose_revision` 先剥离静态背景位移，显式区分 `not_visible`、`observed_empty_at_old_pose`、`identity_swapped`（交叉遮挡保留 `pending` 态）。

### 决策 5：流式监控架构解构、4 阶段 8 分支施工顺序与 Colab `1,000 CU` 预算

1. **流式监控瘦身解构（不重复造轮子）**：
   - **取消独立的“几何动态危险小模型”**：行人、飞来物等几何近身动态避障已由主线 3 原生静中剥动 + 30 Hz 追踪/预测直接喂给 `SCAN`，不再由主线 1 另挂模型重复识别；
   - **异步语义持久化交给主线 2**：常规门洞、房间与目标物体识别在后台关键帧记忆流水线顺便完成并通过 `SceneDelta` 事件回告主线 1；
   - **主线 1 保留“事件驱动任务仲裁器”**：主线 3 不知道任务意图，主线 2 有异步队列延迟；主线 1 平时静默监听 `DynamicOccupancy / LocalizationHealth / SceneDelta` 并结合**当前任务与航段**决定继续、取消、改道或唤醒主 Agent，只在长航段临近岔口、目标短暂入镜或语义歧义时按需触发一次轻量 Fork（取代盲目 2–3 Hz 持续空转）；
   - **分支 2（`feat/agent-streaming-monitor`）必修的 2 处已核实现存抢占 Bug**：
     1. [llm_agent_client.py](../../../src/embodied_navigation/planning/exploration/llm_agent_client.py#L1193-L1200)：当前用 `job.frame_seq < self._latest_submitted_frame_seq` 判过期，导致抽帧周期（如 0.4 s）短于模型推理耗时（0.5–0.8 s）时有效判决持续被新提交帧顶掉 → 改为按同 `action_epoch` 内**已完成判决帧序水位 + 采集时刻过期门**判定；
     2. [native_waypoints.py](../../../src/embodied_navigation/simulation/dynanav/native_waypoints.py#L287-L298)：`action="stop"` 会把 `run_continues` 置 `False` 并锁存 `interactive_stop` 终结整次运行 → 流式抢占打断后还需重规划继续走时，必须发可恢复的 `cancel` 或显式 `preempt`，保留 `stop` 仅作终止语义。
2. **4 阶段 × 8 条干净分支（从静态做透再到动态，每枝只验证单一主线任务）**
   （起点底座简称：`H = feat/static-memory-streaming`、`D = exp/live-v4-ba-sliding-window@aada1f1`）：

   | 阶段 | 分支名 | 基线 / 复用素材 | 所属主线与核心任务 | 关键验收口径 |
   | :--- | :--- | :--- | :--- | :--- |
   | **阶段 1（静态底座：先立“存”和“用”）** | `3. feat/memory-hybrid-store-build` | 基于 `H`；按需提取 `46e0b48` | **主线 2（存 + 建）**：统一 `observation_id` 底账 + 16-bit 深度 PNG + 过程轨迹链/推理先验 + Mesh/点云/关键帧混合构建；2–3 天低成本验证 Mesh 遮挡重排 | 双栈因果回放幂等且前缀零未来泄漏；点云体积显著压缩；遮挡子实验达标才进检索链 |
   | **阶段 1（静态底座：先立“存”和“用”）** | `1. feat/agent-habitat-benchmarks` | 基于 `H` | **主线 1（评估与 360° 喂入）**：对接官方 `HM3D / OVON / GOAT` Scorer；优化 360° 观测、全局/局部地图与探索视角喂入；按需校准轻量状态/`STOP` 判别头 | 官方 Scorer 逐题对齐；按 `1.0 m / 1.8 m` 与 Dev/盲测拆报；OVON/GOAT 官方榜净提升 |
   | **阶段 2（静态全链路）** | `4. feat/memory-tree-graph-retrieval` | 基于分支 `3`（+ Colab） | **主线 2（查 + 改）**：围绕里程计 + 回环 + 语义先验的层次树与四类边时空过程图检索、回环增量修改（外观库免重算）；Colab 训同屋同类实例重排头 | GOAT 同起点「持久 vs 清空」配对 $\Delta\text{SR}/\Delta\text{SPL}$ 净增益；未见房 Top-1 ≥ +5 pp |
   | **阶段 2（静态全链路）** | `5. exp/recon-3d-mesh-assets` | 基于分支 `3` 或 `D`（离线旁线） | **主线 2 → 3D 重建**：离线转连通 Mesh、`Hunyuan3D` 物体补全生成、Blender/UE5/Isaac 游戏材质资产库与 `2DGS` 对照 | 高质量碰撞 Mesh 与资产导出；严守离线边界，零在线帧率拖累、零未来证据泄漏 |
   | **阶段 2（静态全链路）** | `2. feat/agent-streaming-monitor` | 基于分支 `1`；按需提取 `8aebc95` | **主线 1（瘦身事件流式 + 双仿真）**：事件驱动仲裁器与按需 Fork（取消独立动态危险模型）、修 `llm_agent_client.py` / `native_waypoints.py` 2 处 Bug、新增 `agent/adapters/dynanav_adapter.py` | 不再无意义空转且慢返回不丢有效抢占；Habitat 与 Isaac Sim 静态两层共用 L7 运行时 |
   | **阶段 3（动态感知）** | `6. exp/slam-dynamic-and-generalize` | 基于 `D` + `ref/dynanav-dynamic-pedestrian`（按需 `ffd7882`） | **主线 3（动态 SLAM + 室内外收尾）**：多模态互补；`FAST-LIVO2` 静中剥动 + 10 Hz LiDAR / 30 Hz KF 外推 + 切向光度可拒绝修正（前端零小模型）；室外坡道与余下场景泛化 | 同一输入消融 `LiDAR+KF` vs `加切向光度`；M3DGR/行人静图零鬼影、原有 2 cm 静态精度不退 |
   | **阶段 4（4D 叠加与大闭环）** | `7. feat/4d-object-seg-tracking` | 汇合分支 `6` + `3`（可选 Colab） | **4D（物体分割 + 运动跟踪）**：用 `observation_id` 串联短时 `track_id` 与长期 `instance_id`；分割 + 跨帧跟踪 + 遮挡补全，延伸刚体/铰接/可选骨架 | 动态实例连续 ID 与时效轨迹/预测精度提升；严格区分“未观测”、“实测为空”与“身份交换” |
   | **阶段 4（4D 叠加与大闭环）** | `8. feat/4d-memory-and-embodied-loop` | 汇合分支 `2 + 4 + 7` | **全栈 4D 合龙**：把 4D 跟踪接入记忆查改与事件流式仲裁（堵路/搬移/回环即时改记忆并抢占过期航点），在 Isaac Sim 动态多层跑通闭环 | 动态多层真物理闭环零静态错判误取消；PCT/SCAN + MoE-CTS 安全门独立兜底 |

3. **既有分支提取纪律（禁整枝硬并）**：`feat/semantic-keyframe-graph@46e0b48` 供分支 `3/4` 提取节点契约；`feat/agent-preemptable-active-slam@8aebc95` 供分支 `2` 提取 `action_epoch` 抢占逻辑；`perf/slam-backend-cpp-icp@ffd7882` 仅剖面确认卡顿时单独进分支 `6`；`ref/dynanav-dynamic-pedestrian` 提供动态场景与复现定义。
4. **`Google Colab 1,000 CU` 分档封顶**：`30 CU` 试跑测账 + `190 CU` train 难负例库（含 `79.54%` 剪枝召回曲线）+ `210 CU` 分支 4 实例重排头 + `70 CU` 分支 1 `STOP` 校准 + `140 CU` 分支 7 动态存在/重激活判别头 + `240 CU` 分支 1 全量官方清单批推理 + `120 CU` 复验预留 = **`1,000 CU`**。禁训世界模型/VLA/大 VLM、禁验证集提前跑离线 GS/PBR 送在线、禁在高端卡空等仿真或 API 网关。

---

## 3. 验收与复盘要求

- 所有分支沿用 [AGENTS.md](../../../AGENTS.md) 单分支主题隔离与逐行 diff 审阅，对比全量单测/回归失败名单，严禁删改旧断言。
- Habitat 评测（分支 `1/4`）须按官方原 Scorer、`1.0 m / 1.8 m`、5 场景 Dev 与未见盲测拆分报告；D 路径与 4D 动态分支（`6/7/8`）除 Umeyama 外须补固定尺度 SE(3)、起点对齐、分段 RPE、Z 误差、回环瞬间跳变、动态拦截延迟与自由空间误清率。
